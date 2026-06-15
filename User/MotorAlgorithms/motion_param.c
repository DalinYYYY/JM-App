/**
 * @file motion_param.c
 * @brief 电机运动参数解算模块实现（自包含，无外部幻影依赖）
 *
 * @author Eamon (eamon.zhang@hyfoss-tec.com)
 * @version 3.0
 * @date 2025-04-07
 *
 * @par 修改日志:
 * <table>
 * <tr><th>Date       <th>Version <th>Author  <th>Description
 * <tr><td>2025-04-07 <td>1.0     <td>Eamon   <td>初始化
 * <tr><td>2025-04-17 <td>2.0     <td>Dalin   <td>重构代码，增加速度位置获取接口
 * <tr><td>2026-06-12 <td>3.0     <td>Dalin   <td>完全重构，模块自包含，新增齿轮游标绝对多圈
 * </table>
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

/* 上电后丢弃的解算拍数（等待编码器稳定，沿用原 1000 拍） */
#define MOTION_POS_SETTLE_TICKS 1000u

/* NULL 防护：替代缺失的 assert_report，非法入参直接返回 */
#define MOTION_GUARD(cond) \
	do                     \
	{                      \
		if (!(cond))       \
		{                  \
			return;        \
		}                  \
	} while (0)
#define MOTION_GUARD_RET(cond, rv) \
	do                             \
	{                              \
		if (!(cond))               \
		{                          \
			return (rv);           \
		}                          \
	} while (0)

/* 把角度归一化到 [0, 360) */
static float normalize_angle(float angle)
{
	int n = (int)(angle / 360.0f);
	float remainder = angle - n * 360.0f;
	return remainder >= 0.0f ? remainder : (remainder + 360.0f);
}

/* 把弧度差归一化到 (-pi, pi]，用于处理过零跳变 */
static float wrap_rad_pi(float delta)
{
	while (delta > MOTION_PI)
		delta -= MOTION_2PI;
	while (delta <= -MOTION_PI)
		delta += MOTION_2PI;
	return delta;
}

/* ------------------------------------------------------------------ */
/* 轻量滑动平均滤波器（替代缺失的 slide_filter）                        */
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
 * @note  原实现把采样频率(Hz)放在 dt 字段里，rad_s = delta_deg * deg2rad * freq。
 *        这里 update_freq_hz 即每秒解算次数，保持同一量纲。
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
/* 位置：单编码器软件累圈（掉电丢失）                                   */
/* ------------------------------------------------------------------ */
static float update_position_soft(struct motion_param *pobj)
{
	/* 编码器上电需要稳定一定时间 */
	if (pobj->pos_settle_ticks < MOTION_POS_SETTLE_TICKS)
	{
		pobj->pos_settle_ticks++;
		pobj->last_single_rad = pobj->mechanical_angle * MOTION_DEG2RAD;
		return 0.0f;
	}

	/* 当前单圈弧度（含设备补偿），归一化到 [0, 2pi) */
	float comp = (pobj->device_compensation_callback != NULL)
					 ? pobj->device_compensation_callback()
					 : 0.0f;
	float curr_rad = pobj->mechanical_angle * MOTION_DEG2RAD - comp;
	curr_rad = curr_rad >= MOTION_2PI ? (curr_rad - MOTION_2PI)
									  : (curr_rad < 0.0f ? (curr_rad + MOTION_2PI) : curr_rad);

	/* 过零跳变修正后累加 */
	float delta = wrap_rad_pi(curr_rad - pobj->last_single_rad);
	pobj->position += delta;
	pobj->rotation_count = (int32_t)(pobj->position / MOTION_2PI);
	pobj->last_single_rad = curr_rad;

	/* SOFT 模式下绝对多圈量等于软件累圈量 */
	pobj->multiturn_position = pobj->position;
	pobj->multiturn_turns = pobj->rotation_count;
	return pobj->position;
}

/* ------------------------------------------------------------------ */
/* 位置：齿轮游标(Nonius)绝对多圈                                       */
/* ------------------------------------------------------------------ */
/**
 * @brief 圆周距离：两个单圈位置 [0,1) 的最近角差，范围 [0, 0.5]
 */
static float circular_dist(float a, float b)
{
	float d = a - b;
	d -= floorf(d); /* mod 1 -> [0,1) */
	return (d <= 0.5f) ? d : (1.0f - d);
}

/**
 * @brief 齿轮游标绝对多圈解算（predict-match 法）
 *
 * 物理模型：主齿轮（齿数 Nm）与副齿轮（齿数 Ns）啮合，主齿轮转 1 圈，副齿轮转
 * Nm/Ns 圈。已知主齿轮绝对圈数 P 时，副齿轮单圈位置可预测为 frac(P * Nm/Ns)。
 *
 * 重建：主齿轮单圈位置 θm 已由主编码器测得（[0,1)）。遍历候选整圈数 T，预测各副
 * 齿轮单圈位置 frac((T+θm)*Nm/Ns_i)，与实测副齿轮位置做圆周距离求和，取误差最小
 * 的 T。单游标(双齿轮)量程 = Ns1 圈；双游标(三齿轮)量程 = Ns1*Ns2 圈（齿数两两
 * 互质时），且第二个游标提供交叉校验抗噪。
 *
 * 该法相比解析相位差更鲁棒：无需齿数模逆，误差以圆周距离显式度量，可用于异常判定。
 *
 * @param pobj 运动参数对象
 * @param gear_angles 各齿轮机械角度 (deg)，[0]=主齿轮
 * @param count 齿轮数量
 */
static float update_position_nonius(struct motion_param *pobj,
									const float *gear_angles, uint8_t count)
{
	const multiturn_config_t *cfg = &pobj->multiturn_cfg;
	uint16_t Nm = cfg->gear_teeth[0];

	/* 主齿轮单圈位置 θm [0,1) */
	int8_t dir_m = (cfg->gear_dir[0] != 0) ? cfg->gear_dir[0] : 1;
	float theta_m = normalize_angle((float)dir_m * gear_angles[0]) / 360.0f;

	/* 各副齿轮单圈位置与齿数（最多 2 个） */
	uint8_t nsec = (count >= MOTION_NONIUS_GEAR_MAX) ? (MOTION_NONIUS_GEAR_MAX - 1u)
													 : (uint8_t)(count - 1u);
	if (cfg->mode == MULTITURN_MODE_NONIUS_2GEAR && nsec > 1u)
		nsec = 1u;

	float theta_s[MOTION_NONIUS_GEAR_MAX - 1u];
	uint16_t teeth_s[MOTION_NONIUS_GEAR_MAX - 1u];
	int32_t period = 1;
	for (uint8_t i = 0; i < nsec; i++)
	{
		int8_t dir = (cfg->gear_dir[i + 1u] != 0) ? cfg->gear_dir[i + 1u] : 1;
		theta_s[i] = normalize_angle((float)dir * gear_angles[i + 1u]) / 360.0f;
		teeth_s[i] = cfg->gear_teeth[i + 1u];
		period *= (int32_t)teeth_s[i]; /* 量程 = 各副齿轮齿数之积 */
	}

	/* predict-match：扫描候选整圈数，取副齿轮预测误差最小者 */
	int32_t best_turns = 0;
	float best_err = 1.0e30f;
	for (int32_t T = 0; T < period; T++)
	{
		float p_abs = (float)T + theta_m; /* 主齿轮绝对圈数 */
		float err = 0.0f;
		for (uint8_t i = 0; i < nsec; i++)
		{
			float pred = p_abs * (float)Nm / (float)teeth_s[i];
			pred -= floorf(pred); /* 预测副齿轮单圈位置 */
			err += circular_dist(pred, theta_s[i]);
		}
		if (err < best_err)
		{
			best_err = err;
			best_turns = T;
		}
	}

	/* 绝对多圈位置 = 整圈数 + 主齿轮单圈位置，转成弧度 */
	pobj->multiturn_turns = best_turns;
	pobj->multiturn_position = ((float)best_turns + theta_m) * MOTION_2PI;

	/* position 与 multiturn 对齐，rotation_count 给出整圈数 */
	pobj->position = pobj->multiturn_position;
	pobj->rotation_count = best_turns;
	return pobj->multiturn_position;
}

/* 位置解算分发：按多圈模式选择 */
static float update_position(struct motion_param *pobj,
							 const float *gear_angles, uint8_t count)
{
	switch (pobj->multiturn_cfg.mode)
	{
		case MULTITURN_MODE_NONIUS_2GEAR:
			if (count >= 2u)
				return update_position_nonius(pobj, gear_angles, count);
			return update_position_soft(pobj); /* 副齿轮缺失则回退软件累圈 */

		case MULTITURN_MODE_NONIUS_3GEAR:
			/* 三齿轮模式：齐全(>=3)用双游标，缺一路(==2)自动退化为单游标 */
			if (count >= 2u)
				return update_position_nonius(pobj, gear_angles, count);
			return update_position_soft(pobj);

		case MULTITURN_MODE_SOFT:
		case MULTITURN_MODE_NONE:
		default:
			return update_position_soft(pobj);
	}
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
static float get_position(struct motion_param *pobj)
{
	return pobj->position;
}
static float get_multiturn_position(struct motion_param *pobj)
{
	return pobj->multiturn_position;
}
static int32_t get_turns(struct motion_param *pobj)
{
	return pobj->multiturn_turns;
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
static void motor_param_handle_ex(struct motion_param *pobj, motion_type_e type,
								  const float *gear_angles, uint8_t count)
{
	MOTION_GUARD(pobj != NULL);
	MOTION_GUARD(gear_angles != NULL);
	MOTION_GUARD(count >= 1u);

	pobj->mechanical_angle = gear_angles[0];

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

		case MOTION_TYPE_ELE_POS:
		case MOTION_TYPE_ELE_POS_RADIAN:
			update_ele_radian(pobj);
			update_position(pobj, gear_angles, count);
			break;

		case MOTION_TYPE_ALL:
			update_ele_radian(pobj);
			update_rad_s(pobj, pobj->mechanical_angle);
			update_deg_s(pobj);
			update_rpm(pobj);
			update_position(pobj, gear_angles, count);
			break;

		default:
			break;
	}
}

static void motor_param_handle(struct motion_param *pobj, motion_type_e type,
							   float mechanical_angle)
{
	/* 单角度入口：等价于 count=1 的多角度入口 */
	motor_param_handle_ex(pobj, type, &mechanical_angle, 1u);
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
	pobj->multiturn_cfg = cfg->multiturn;
	pobj->device_compensation_callback = cfg->device_compensation_callback;

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
	pobj->get_position = get_position;
	pobj->get_multiturn_position = get_multiturn_position;
	pobj->get_turns = get_turns;

	/* 配置接口 */
	pobj->set_update_freq = set_update_freq;

	/* 更新接口 */
	pobj->update = motor_param_handle;
	pobj->update_ex = motor_param_handle_ex;

	/* 前馈补偿接口 */
	pobj->feedforword_compute = feedforword_compute;
	pobj->feedforword_get_vel = feedforword_get_vel;
	pobj->feedforword_get_acc = feedforword_get_acc;
}

void motion_param_init(motion_param_t *pobj, uint8_t poles, uint16_t slide_window_size,
					   float (*device_compensation_callback)(void))
{
	motion_param_config_t cfg;
	memset(&cfg, 0, sizeof(cfg));

	cfg.poles = poles;
	cfg.slide_window_size = slide_window_size;
	cfg.update_freq_hz = 2000u;				  /* 沿用原默认 2000Hz */
	cfg.multiturn.mode = MULTITURN_MODE_SOFT; /* 兼容旧行为：单编码器软件累圈 */
	cfg.device_compensation_callback = device_compensation_callback;

	motion_param_init_cfg(pobj, &cfg);
}