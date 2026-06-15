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
	int n = (int)(angle / 360.0f);
	float remainder = angle - n * 360.0f;
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
	}
	f->buf[f->head] = sample;
	f->sum += sample;
	f->head = (uint16_t)((f->head + 1u) % f->size);
	return f->sum / (float)f->count;
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
 * @note  update_freq_hz 为每秒解算次数，rad_s = delta_deg * deg2rad * freq。
 */
static void update_rad_s(struct motion_param *pobj, float mechanical_angle)
{
	float freq = (float)pobj->update_freq_hz;

	/* 角度变化量，处理 ±180° 跳变 */
	float delta = mechanical_angle - pobj->prev_mech_angle;
	if (fabsf(delta) > 180.0f)
	{
		delta = (delta > 0.0f) ? (delta - 360.0f) : (delta + 360.0f);
	}

	/* 角速度 (rad/s) = 角度增量(rad) * 解算频率 */
	pobj->rad_s = delta * (MOTION_DEG2RAD * freq);
	pobj->slide_rad_s = slide_filter_calc(&pobj->slide_filter, pobj->rad_s);

	/* 加速度：对滤波后速度做差分，再滑动滤波 */
	pobj->omegaHistory[0] = pobj->omegaHistory[1];
	pobj->omegaHistory[1] = pobj->omegaHistory[2];
	pobj->omegaHistory[2] = pobj->slide_rad_s;
	float acc = (pobj->omegaHistory[2] - pobj->omegaHistory[1]) * freq;
	pobj->acceleration = slide_filter_calc(&pobj->slide_acc_filter, acc);

	pobj->prev_mech_angle = mechanical_angle;
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

/* ------------------------------------------------------------------ */
/* 前馈补偿                                                            */
/* ------------------------------------------------------------------ */
static void feedforword_compute(struct motion_param *pobj, float expect_angle)
{
	float freq = (float)pobj->update_freq_hz;

	pobj->ff_expect_angle = expect_angle;
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
	cfg.update_freq_hz = 2000u; /* 沿用原默认 2000Hz */

	motion_param_init_cfg(pobj, &cfg);
}
