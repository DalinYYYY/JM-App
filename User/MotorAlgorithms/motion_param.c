/**
 * @file        motion_param.c
 * @brief       电机运动参数解算模块（仅角度/速度，不含多圈计数）
 * 
 * @author      Dalin (dalin@robot.com)
 * @version     1.0
 * @date        2026-06-12
 * 
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 * 
 * 输入机械角度，按照配置的极对数 / 更新频率解算出：
 *   - 电角度 / 电弧度
 *   - 机械角度（透传）
 *   - 角速度（deg/s、rad/s、滑动滤波、rpm）与角加速度
 *   - 前馈补偿（速度 / 加速度）
 *
 * 多圈/绝对位置计数已拆分到独立的 multiturn 模块（multiturn.h），二者由调用方
 * 组合使用：本模块吃机械角度出运动量，multiturn 吃齿轮角度出绝对圈数/位置。
 * 
 * @par 修改日志:
 * | 日期       | 版本 | 作者   | 修改内容   |
 * |------------|------|--------|------------|
 * | 2026-06-12     | 1.0  | yangsl | 初始创建   |
 * | 2026-06-15     | 1.1  | yangsl | 拆分：多圈计数移至 multiturn 模块，本模块仅保留角度/速度   |
 * 
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 */

#include "motion_param.h"
#include <math.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/* 本地常量与工具（模块自包含，不依赖 imath.h / assert_report.h）       */
/* ------------------------------------------------------------------ */
#ifndef MOTION_PI
#define MOTION_PI 3.14159265358979323846f
#endif
#define MOTION_2PI (2.0f * MOTION_PI)
#define MOTION_DEG2RAD (MOTION_PI / 180.0f) // °  -> rad
#define MOTION_RAD2DEG (180.0f / MOTION_PI) // rad -> °

/* NULL 防护：替代缺失的 assert_report，非法入参直接返回 */
#define MOTION_GUARD(cond) \
	do                     \
	{                      \
		if (!(cond))       \
		{                  \
			return;        \
		}                  \
	} while (0)

/* 把角度归一化到 [0, 360) */
static float normalize_angle(float angle)
{
	float remainder = fmodf(angle, 360.0f);
	return remainder >= 0.0f ? remainder : (remainder + 360.0f);
}

/* ------------------------------------------------------------------ */
/* 轻量滑动平均滤波器                                                   */
/* ------------------------------------------------------------------ */
static void slide_filter_init(motion_slide_filter_t *f, uint16_t size)
{
	if (size == 0u)
		size = 1u;
	if (size > MOTION_SLIDE_WINDOW_MAX)
		size = MOTION_SLIDE_WINDOW_MAX;
	memset(f, 0, sizeof(*f));
	f->size = size;
}

static float slide_filter_calc(motion_slide_filter_t *f, float sample)
{
	/* 环形缓冲：减去被覆盖的旧样本，加上新样本，避免每次重新求和 */
	if (f->count == f->size)
	{
		f->sum -= f->buf[f->head];
	}
	else
	{
		f->count++;
		f->inv_count = 1.0f / (float)f->count; /* 仅填充阶段更新，满后恒定 */
	}
	f->buf[f->head] = sample;
	f->sum += sample;

	/* 环形递增，用比较复位代替取模，消除整数除法 */
	if (++f->head >= f->size)
		f->head = 0u;

	/* 用逆数乘法代替除法 */
	return f->sum * f->inv_count;
}

/* ------------------------------------------------------------------ */
/* 状态接口                                                            */
/* ------------------------------------------------------------------ */
static bool motor_get_eleangle_update_status(struct motion_param *pobj)
{
	return pobj->ele_angle_update_status;
}

static void motor_set_eleangle_update_status(struct motion_param *pobj, bool status)
{
	pobj->ele_angle_update_status = status;
}

static void motor_set_speed_update_state(struct motion_param *pobj, bool status)
{
	pobj->rpm_update_status = status;
}

static bool motor_get_speed_update_state(struct motion_param *pobj)
{
	return pobj->rpm_update_status;
}

/* ------------------------------------------------------------------ */
/* 电角度 / 电弧度                                                     */
/* ------------------------------------------------------------------ */
static float update_ele_radian(struct motion_param *pobj)
{
	pobj->ele_angle = normalize_angle(pobj->mechanical_angle * (float)pobj->poles);
	pobj->ele_radian = pobj->ele_angle * MOTION_DEG2RAD;
	motor_set_eleangle_update_status(pobj, true);
	return pobj->ele_radian;
}

/* ------------------------------------------------------------------ */
/* 速度 / 加速度                                                       */
/* ------------------------------------------------------------------ */
static void update_deg_s(struct motion_param *pobj)
{
	pobj->deg_s = (int32_t)(pobj->rad_s * MOTION_RAD2DEG);
}

/**
 * @brief 解算角速度（rad/s）与角加速度（rad/s^2）
 * @note  update_freq_hz 为每秒解算次数。支持三种方法（pobj->vel_method）：
 *        - DIFF：后向差分 + 滑动平均（兼容旧行为）
 *        - LSQ ：N 点最小二乘差分（FIR 微分器，固定群延迟，低噪声）
 *        - PLL ：二阶观测器，速度由积分得到，低滞后、平滑
 *        三者统一输出 rad_s / slide_rad_s；加速度统一对最终速度做差分滤波。
 */

/* 处理 ±180° 跳变，返回归一化到 (-180,180] 的角度增量(deg) */
static float wrap_delta_deg(float delta)
{
	if (fabsf(delta) > 180.0f)
	{
		delta = (delta > 0.0f) ? (delta - 360.0f) : (delta + 360.0f);
	}
	return delta;
}

/* 方法一：后向差分 + 滑动平均。out_raw 返回未滤波速度，函数返回滤波后速度，单位 rad/s */
static float vel_calc_diff(struct motion_param *pobj, float mechanical_angle, float *out_raw)
{
	float freq = (float)pobj->update_freq_hz;
	float delta = wrap_delta_deg(mechanical_angle - pobj->prev_mech_angle);
	float rad_s = delta * (MOTION_DEG2RAD * freq);
	pobj->prev_mech_angle = mechanical_angle;
	*out_raw = rad_s;
	return slide_filter_calc(&pobj->slide_filter, rad_s);
}

// https://k0uhb8quijf.feishu.cn/wiki/Tw7qwWYvkiwY9LkX8TRc6WPunVf?from=from_copylink
/* 方法二：N 点最小二乘差分（对最近 N 个角度拟合直线，斜率即速度）。
 * 角度先去跳变累加成连续序列，避免 360° 折返污染拟合。返回 rad/s。 */
static float vel_calc_lsq(struct motion_param *pobj, float mechanical_angle)
{
	float freq = (float)pobj->update_freq_hz;
	uint16_t n = pobj->lsq_size;
	uint16_t i;

	/* 连续角度 = 上一连续角度 + 去跳变增量 */
	float cont = pobj->prev_mech_angle + wrap_delta_deg(mechanical_angle - pobj->lsq_buf_last_raw);
	pobj->lsq_buf_last_raw = mechanical_angle;
	pobj->prev_mech_angle = cont;

	/* 左移历史，推入新点（窗口小，O(N) 可接受） */
	for (i = 0u; i + 1u < n; i++)
		pobj->lsq_buf[i] = pobj->lsq_buf[i + 1u];
	pobj->lsq_buf[n - 1u] = cont;

	if (pobj->lsq_count < n)
		pobj->lsq_count++;

	uint16_t m = pobj->lsq_count;
	if (m < 2u)
		return 0.0f; /* 点不足，速度记 0 */

	/* 斜率 = Σ(i-ī)(y-ȳ) / Σ(i-ī)^2，样本索引 i=0..m-1，间隔 1/freq */
	float mean_i = (float)(m - 1u) * 0.5f;
	float num = 0.0f;
	float denom;
	/* 用最近 m 个点：buf 尾部 m 个 */
	uint16_t base = n - m;
	for (i = 0u; i < m; i++)
	{
		num += ((float)i - mean_i) * pobj->lsq_buf[base + i];
	}
	if (m == n && pobj->lsq_inv_denom > 0.0f)
	{
		/* 满窗：分母恒定，用缓存逆数 */
		float slope_deg_per_sample = num * pobj->lsq_inv_denom;
		return slope_deg_per_sample * freq * MOTION_DEG2RAD;
	}
	/* 未满窗：现算分母 */
	denom = 0.0f;
	for (i = 0u; i < m; i++)
	{
		float d = (float)i - mean_i;
		denom += d * d;
	}
	if (m == n)
		pobj->lsq_inv_denom = (denom > 0.0f) ? (1.0f / denom) : 0.0f;
	float slope = (denom > 0.0f) ? (num / denom) : 0.0f;
	return slope * freq * MOTION_DEG2RAD;
}

// https://k0uhb8quijf.feishu.cn/wiki/Bth6wWwTii7YhWknRvEcAYYknmc?from=from_copylink
/* 方法三：PLL/龙伯格二阶观测器。位置误差驱动 PI，速度状态积分得位置。
 * 返回 rad/s（取观测器速度状态 pll_omega）。 */
static float vel_calc_pll(struct motion_param *pobj, float mechanical_angle)
{
	float dt = 1.0f / (float)pobj->update_freq_hz;

	/* 位置误差（去跳变，归一化到 (-180,180]） */
	float err = wrap_delta_deg(mechanical_angle - pobj->pll_theta);

	/* 速度积分：ω += Ki·err·dt */
	pobj->pll_omega += pobj->pll_ki * err * dt;

	/* 位置积分：θ += (ω + Kp·err)·dt，并归一化 */
	pobj->pll_theta += (pobj->pll_omega + pobj->pll_kp * err) * dt;
	pobj->pll_theta = normalize_angle(pobj->pll_theta);

	pobj->prev_mech_angle = mechanical_angle;
	return pobj->pll_omega * MOTION_DEG2RAD; /* deg/s -> rad/s */
}

static void update_rad_s(struct motion_param *pobj, float mechanical_angle)
{
	float freq = (float)pobj->update_freq_hz;

	/* 首次调用：只初始化各方法状态，不算速度，避免上电尖峰 */
	if (!pobj->vel_initialized)
	{
		pobj->prev_mech_angle = mechanical_angle;
		pobj->lsq_buf_last_raw = mechanical_angle;
		pobj->pll_theta = mechanical_angle;
		pobj->pll_omega = 0.0f;
		pobj->vel_initialized = true;
		pobj->rad_s = 0.0f;
		pobj->slide_rad_s = slide_filter_calc(&pobj->slide_filter, 0.0f);
		motor_set_speed_update_state(pobj, true);
		return;
	}

	float rad_s;
	float slide_rad_s;
	switch (pobj->vel_method)
	{
		case VEL_METHOD_LSQ:
			rad_s = vel_calc_lsq(pobj, mechanical_angle);
			slide_rad_s = rad_s; /* LSQ 自带平滑，不再叠滑窗 */
			break;
		case VEL_METHOD_PLL:
			rad_s = vel_calc_pll(pobj, mechanical_angle);
			slide_rad_s = rad_s; /* PLL 自带平滑，不再叠滑窗 */
			break;
		case VEL_METHOD_DIFF:
		default:
			slide_rad_s = vel_calc_diff(pobj, mechanical_angle, &rad_s);
			break;
	}
	pobj->rad_s = rad_s;
	pobj->slide_rad_s = slide_rad_s;

	/* 加速度：对最终速度做差分，再滑动滤波（三种方法统一） */
	pobj->omegaHistory[0] = pobj->omegaHistory[1];
	pobj->omegaHistory[1] = pobj->omegaHistory[2];
	pobj->omegaHistory[2] = slide_rad_s;
	float acc = (pobj->omegaHistory[2] - pobj->omegaHistory[1]) * freq;
	pobj->acceleration = slide_filter_calc(&pobj->slide_acc_filter, acc);

	motor_set_speed_update_state(pobj, true);
}

static float update_rpm(struct motion_param *pobj)
{
	pobj->rpm = (int32_t)(pobj->rad_s * MOTION_RAD2DEG * 60.0f / 360.0f);
	return (float)pobj->rpm;
}

/* ------------------------------------------------------------------ */
/* 获取接口                                                            */
/* ------------------------------------------------------------------ */
static float get_mechanical_angle(struct motion_param *pobj)
{
	return pobj->mechanical_angle;
}
static float get_ele_radian(struct motion_param *pobj)
{
	return pobj->ele_radian;
}

static void set_update_freq(struct motion_param *pobj, uint32_t freq_hz)
{
	pobj->update_freq_hz = (freq_hz == 0u) ? 1u : freq_hz;
}

/* 由带宽/阻尼算 PLL 二阶增益：ωn=2π·bw, Kp=2ζωn, Ki=ωn^2 */
static void pll_set_gains(struct motion_param *pobj, float bandwidth_hz, float damping)
{
	if (bandwidth_hz <= 0.0f)
		bandwidth_hz = 50.0f;
	if (damping <= 0.0f)
		damping = 1.0f;
	float wn = MOTION_2PI * bandwidth_hz;
	pobj->pll_kp = 2.0f * damping * wn;
	pobj->pll_ki = wn * wn;
}

/* 运行时切换速度解算方法：复位相关状态，下一帧按首次重新初始化，避免切换瞬间尖峰 */
static void set_vel_method(struct motion_param *pobj, motion_vel_method_e method)
{
	pobj->vel_method = method;
	/* 复位各方法历史，强制走首次初始化路径 */
	slide_filter_init(&pobj->slide_filter, pobj->slide_filter.size);
	slide_filter_init(&pobj->slide_acc_filter, pobj->slide_acc_filter.size);
	pobj->lsq_count = 0u;
	pobj->lsq_inv_denom = 0.0f;
	pobj->omegaHistory[0] = 0.0f;
	pobj->omegaHistory[1] = 0.0f;
	pobj->omegaHistory[2] = 0.0f;
	pobj->vel_initialized = false;
}

/* ------------------------------------------------------------------ */
/* 前馈补偿                                                            */
/* ------------------------------------------------------------------ */
static void feedforword_compute(struct motion_param *pobj, float expect_angle)
{
	float freq = (float)pobj->update_freq_hz;

	pobj->ff_expect_angle = expect_angle;

	/* 首次调用：只记录角度，不算速度/加速度，避免 prev=0 造成的尖峰 */
	if (!pobj->ff_initialized)
	{
		pobj->ff_prev_angle = expect_angle;
		pobj->ff_prev_rad_s = 0.0f;
		pobj->ff_initialized = true;
		return;
	}

	pobj->ff_delta_angle = pobj->ff_expect_angle - pobj->ff_prev_angle;

	/* 角速度 (rad/s) */
	pobj->ff_rad_s = pobj->ff_delta_angle * freq;
	pobj->ff_slide_rad_s = slide_filter_calc(&pobj->ff_vel_filter, pobj->ff_rad_s);

	/* 角加速度 (rad/s^2) */
	pobj->ff_delta_rad_s = pobj->ff_slide_rad_s - pobj->ff_prev_rad_s;
	pobj->ff_accel = pobj->ff_delta_rad_s * freq;
	pobj->ff_slide_acc = slide_filter_calc(&pobj->ff_acc_filter, pobj->ff_accel);

	pobj->ff_prev_angle = pobj->ff_expect_angle;
	pobj->ff_prev_rad_s = pobj->ff_slide_rad_s;
}

static float feedforword_get_vel(struct motion_param *pobj)
{
	return pobj->ff_slide_rad_s;
}
static float feedforword_get_acc(struct motion_param *pobj)
{
	return pobj->ff_slide_acc;
}

/* ------------------------------------------------------------------ */
/* 更新分发                                                            */
/* ------------------------------------------------------------------ */
static void motor_param_handle(struct motion_param *pobj, motion_type_e type,
							   float mechanical_angle)
{
	MOTION_GUARD(pobj != NULL);

	pobj->mechanical_angle = mechanical_angle;

	switch (type)
	{
		case MOTION_TYPE_ELE:
		case MOTION_TYPE_ELE_RADIAN:
			update_ele_radian(pobj);
			break;

		case MOTION_TYPE_ELE_VEL:
		case MOTION_TYPE_ELE_VEL_RADIAN:
			update_ele_radian(pobj);
			update_rad_s(pobj, pobj->mechanical_angle);
			update_deg_s(pobj);
			update_rpm(pobj);
			break;

		case MOTION_TYPE_ALL:
			update_ele_radian(pobj);
			update_rad_s(pobj, pobj->mechanical_angle);
			update_deg_s(pobj);
			update_rpm(pobj);
			break;

		default:
			break;
	}
}

/* ------------------------------------------------------------------ */
/* 初始化                                                              */
/* ------------------------------------------------------------------ */
void motion_param_init_cfg(motion_param_t *pobj, const motion_param_config_t *cfg)
{
	MOTION_GUARD(pobj != NULL);
	MOTION_GUARD(cfg != NULL);

	memset(pobj, 0, sizeof(motion_param_t));

	pobj->poles = cfg->poles;
	pobj->update_freq_hz = (cfg->update_freq_hz == 0u) ? 1u : cfg->update_freq_hz;

	/* 速度解算方法 */
	pobj->vel_method = cfg->vel_method;

	/* 最小二乘窗口：默认 5，限幅到 [3, MOTION_LSQ_WINDOW_MAX] */
	{
		uint16_t lw = (cfg->lsq_window_size == 0u) ? 5u : cfg->lsq_window_size;
		if (lw < 3u)
			lw = 3u;
		if (lw > MOTION_LSQ_WINDOW_MAX)
			lw = MOTION_LSQ_WINDOW_MAX;
		pobj->lsq_size = lw;
	}

	/* PLL 增益：由带宽/阻尼算出（0 走默认 50Hz / ζ=1） */
	pll_set_gains(pobj, cfg->pll_bandwidth_hz, cfg->pll_damping);

	/* 滤波器：速度用配置窗口，加速度/前馈用固定小窗口（沿用原值 5） */
	slide_filter_init(&pobj->slide_filter, cfg->slide_window_size);
	slide_filter_init(&pobj->slide_acc_filter, 5u);
	slide_filter_init(&pobj->ff_vel_filter, 5u);
	slide_filter_init(&pobj->ff_acc_filter, 5u);

	/* 状态接口 */
	pobj->set_eleangle_status = motor_set_eleangle_update_status;
	pobj->get_eleangle_status = motor_get_eleangle_update_status;
	pobj->set_speed_update_state = motor_set_speed_update_state;
	pobj->get_speed_update_state = motor_get_speed_update_state;

	/* 获取接口 */
	pobj->get_ele_radian = get_ele_radian;
	pobj->get_rpm = update_rpm;
	pobj->get_mechanical_angle = get_mechanical_angle;

	/* 配置接口 */
	pobj->set_update_freq = set_update_freq;
	pobj->set_vel_method = set_vel_method;

	/* 更新接口 */
	pobj->update = motor_param_handle;

	/* 前馈补偿接口 */
	pobj->feedforword_compute = feedforword_compute;
	pobj->feedforword_get_vel = feedforword_get_vel;
	pobj->feedforword_get_acc = feedforword_get_acc;
}

void motion_param_init(motion_param_t *pobj, uint8_t poles, uint16_t slide_window_size,
					   float (*unused_compensation_callback)(void))
{
	motion_param_config_t cfg;
	memset(&cfg, 0, sizeof(cfg));

	(void)unused_compensation_callback; /* 设备补偿已移至 multiturn 模块 */

	cfg.poles = poles;
	cfg.slide_window_size = slide_window_size;
	cfg.update_freq_hz = 1000u;

	motion_param_init_cfg(pobj, &cfg);
}
