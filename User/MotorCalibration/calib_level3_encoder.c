/**
 * @file calib_level3_encoder.c
 * @brief L3 编码器校准（零位/方向/线性度/正余弦/多圈零点）
 * @note 当前实现零位标定（d轴对齐法）和方向标定（施加uq观测角度变化）。
 *       线性度/正余弦/多圈零点保持桩实现。
 *
 * @par 零位标定原理
 *   施加 ud 电压（强制电角度=0），转子磁场与 A 相绕组对齐。
 *   稳定后读取编码器原始角度作为 offset（机械零点对应的编码器读数）。
 *
 * @par 方向标定原理
 *   施加正向 uq 电压（电角度=0 时产生正向力矩），观测编码器角度变化方向。
 *   角度增大→CW（正向），角度减小→CCW（反向）。
 */
#include "calib_types.h"
#include "calib_mgr.h"
#include "dev_motor.h"
#include "dev_mt6701.h"
#include "motor_param.h"

/* ===================== 标定参数 =====================
 * 时间相关的 TICK 数用秒数 × 10000 计算（电流环 10kHz → dt=100us）。
 * 若电流环频率改动，只需调整这里的秒数定义。*/
#define CALIB_ALIGN_VOLTAGE   0.1f    /* d轴对齐电压(V) */
#define CALIB_ALIGN_TIME_S    2.0f    /* 对齐稳定等待时间(s) */
#define CALIB_ALIGN_TICKS     (uint32_t)(CALIB_ALIGN_TIME_S * 10000.0f)
#define CALIB_SAMPLE_TIMES    100     /* 采样次数（取平均滤波）*/
#define CALIB_DIR_VOLTAGE     0.5f    /* 方向测试uq电压(V) */
#define CALIB_DIR_TIME_S      1.0f    /* 方向测试持续时间(s) */
#define CALIB_DIR_TICKS       (uint32_t)(CALIB_DIR_TIME_S * 10000.0f)

/* ===================== 模块私有状态 ===================== */
static uint8_t s_submode;
static uint8_t s_step;          /* 标定步骤状态机 */
static uint32_t s_tick;         /* 周期计数器 */
static uint32_t s_sample_cnt;   /* 采样计数器 */
static float s_angle_sum;       /* 角度采样累加和 */
static float s_dir_start_angle; /* 方向测试起始角度 */

/* 标定期间临时替换的电角度回调 */
static float (*s_orig_ele_cb)(void);

/* ===================== 辅助函数 ===================== */

/**
 * @brief 标定期间强制电角度回调（返回固定值）
 * @note  零位标定时返回0，让 inverse_park 用 Θ=0 把 ud 电压投影到 α轴(A相)
 */
static float s_calib_ele_angle;
static float calib_ele_radian_cb(void)
{
	return s_calib_ele_angle;
}

/**
 * @brief 获取 MT6701 原始角度(°) [0,360)
 */
static float get_encoder_raw_deg(dev_motor_t *m)
{
	dev_mt6701_t *enc = &m->mt6701;
	enc->update(enc);
	return (float)enc->raw / MT6701_ANGLE_RESOLUTION * 360.0F;
}

/**
 * @brief 获取编码器当前机械角度(°) [0,360)，含 offset/dir 补偿
 */
static float get_encoder_mech_angle(dev_motor_t *m)
{
	m->encoder.update(&m->encoder);
	return m->encoder.mechanical_angle;
}

/**
 * @brief 施加 dq 电压（绕过电流环 PI，直接操作 FOC 链路）
 * @param m     电机设备
 * @param ud    d轴电压(V)
 * @param uq    q轴电压(V)
 * @param theta 强制使用的电角度(rad)，用于标定期间固定电角度
 */
static void apply_voltage(dev_motor_t *m, float ud, float uq, float theta)
{
	s_calib_ele_angle = theta;
	m->foc.set_udq(&m->foc, ud, uq);
	m->foc.inverse_park(&m->foc);
	m->foc.pfsvpwm(&m->foc);
	m->half_bridge.set_3pwm(&m->half_bridge,
	                        (uint32_t)(PWM_PERIOD * m->foc.svpwm.ta),
	                        (uint32_t)(PWM_PERIOD * m->foc.svpwm.tb),
	                        (uint32_t)(PWM_PERIOD * m->foc.svpwm.tc));
}

/**
 * @brief 撤销电压输出（PWM 置零）
 */
static void apply_zero_voltage(dev_motor_t *m)
{
	m->half_bridge.set_3pwm(&m->half_bridge, 0, 0, 0);
}

/**
 * @brief 进入标定：保存并替换电角度回调
 */
static void calib_enter(dev_motor_t *m)
{
	s_orig_ele_cb = m->ele_radian_callback;
	m->ele_radian_callback = calib_ele_radian_cb;
	m->foc.ele_radian_callback = calib_ele_radian_cb;
	s_calib_ele_angle = 0.0f;
}

/**
 * @brief 退出标定：恢复电角度回调
 */
static void calib_exit(dev_motor_t *m)
{
	apply_zero_voltage(m);
	if (s_orig_ele_cb != NULL)
	{
		m->ele_radian_callback = s_orig_ele_cb;
		m->foc.ele_radian_callback = s_orig_ele_cb;
		s_orig_ele_cb = NULL;
	}
}

/* ===================== 零位标定状态机 =====================
 * STEP 0: 初始化，施加 ud 电压（电角度强制为0），转子开始对齐
 * STEP 1: 等待转子稳定对齐（CALIB_ALIGN_TICKS 个周期）
 * STEP 2: 多次采样编码器原始角度取平均
 * STEP 3: 写入 offset 到 encoder_param 和 dev_mt6701，撤销电压，完成
 * ========================================================== */
static calib_state_e poll_zero_offset(void)
{
	const calib_io_t *io = calib_mgr_get_io();
	dev_motor_t *m = io->motor;

	switch (s_step)
	{
		case 0: /* 施加 d 轴对齐电压 */
			calib_enter(m);
			apply_voltage(m, CALIB_ALIGN_VOLTAGE, 0.0f, 0.0f);
			s_tick = 0;
			s_step = 1;
			return CALIB_STATE_RUNNING;

		case 1: /* 等待转子稳定对齐 */
			apply_voltage(m, CALIB_ALIGN_VOLTAGE, 0.0f, 0.0f);
			if (++s_tick < CALIB_ALIGN_TICKS)
				return CALIB_STATE_RUNNING;
			s_tick = 0;
			s_sample_cnt = 0;
			s_angle_sum = 0.0f;
			s_step = 2;
			return CALIB_STATE_RUNNING;

		case 2: /* 多次采样编码器原始角度 */
			apply_voltage(m, CALIB_ALIGN_VOLTAGE, 0.0f, 0.0f);
			s_angle_sum += get_encoder_raw_deg(m);
			if (++s_sample_cnt < CALIB_SAMPLE_TIMES)
				return CALIB_STATE_RUNNING;
			s_step = 3;
			return CALIB_STATE_RUNNING;

		case 3: /* 写入标定结果，完成 */
		{
			float avg_deg = s_angle_sum / (float)CALIB_SAMPLE_TIMES;
			/* 写入 dev_mt6701 运行时（offset = 对齐位置的原始角度，使 mech_angle=0）*/
			m->mt6701.offset = avg_deg;
			m->mt6701.dir = MT6701_DIR_CW; /* 零位标定先置 CW，方向由后续方向标定确定 */
			/* 写入 motor_param_t（持久化），enc_offset 用计数值 */
			int32_t raw_counts = (int32_t)(avg_deg / 360.0F * MT6701_ANGLE_RESOLUTION);
			motor_param_set_enc_offset(io->param, raw_counts);
			motor_param_set_enc_direction(io->param, 1); /* 1=CW */
			motor_param_set_elec_angle_bias(io->param, 0.0f);
			calib_exit(m);
			return CALIB_STATE_DONE;
		}

		default:
			calib_exit(m);
			return CALIB_STATE_FAILED;
	}
}

/* ===================== 方向标定状态机 =====================
 * STEP 0: 确保零位已标定，施加正向 uq 电压（电角度=0）
 * STEP 1: 持续施加 uq，等待 CALIB_DIR_TICKS 个周期让电机转动
 * STEP 2: 采样角度变化方向，判定 CW/CCW
 * STEP 3: 写入 direction，撤销电压，完成
 *
 * @note 方向标定前提：零位已标定（offset 已写入）。
 *       若未标定，先执行零位标定再执行方向标定。
 * ================================================================== */
static calib_state_e poll_direction(void)
{
	const calib_io_t *io = calib_mgr_get_io();
	dev_motor_t *m = io->motor;

	switch (s_step)
	{
		case 0: /* 施加正向 uq 电压，记录起始角度 */
			calib_enter(m);
			/* 电角度=0 时 uq>0 产生正向力矩（q轴超前d轴90°，即α轴方向）*/
			s_dir_start_angle = get_encoder_mech_angle(m);
			apply_voltage(m, 0.0f, CALIB_DIR_VOLTAGE, 0.0f);
			s_tick = 0;
			s_step = 1;
			return CALIB_STATE_RUNNING;

		case 1: /* 持续施加 uq，等待电机转动 */
			apply_voltage(m, 0.0f, CALIB_DIR_VOLTAGE, 0.0f);
			if (++s_tick < CALIB_DIR_TICKS)
				return CALIB_STATE_RUNNING;
			s_step = 2;
			return CALIB_STATE_RUNNING;

		case 2: /* 采样当前角度，判定方向 */
		{
			float end_angle = get_encoder_mech_angle(m);
			float delta = end_angle - s_dir_start_angle;
			/* 处理 0/360 跳变：若 delta 绝对值 >180，说明跨越了 0° 边界 */
			if (delta > 180.0f)
				delta -= 360.0f;
			else if (delta < -180.0f)
				delta += 360.0f;

			mt6701_dir_e dir;
			int8_t enc_dir;
			if (delta > 1.0f) /* 角度增大 → 正向 CW */
			{
				dir = MT6701_DIR_CW;
				enc_dir = 1;
			}
			else if (delta < -1.0f) /* 角度减小 → 反向 CCW */
			{
				dir = MT6701_DIR_CCW;
				enc_dir = -1;
			}
			else /* 角度几乎无变化，可能电机未转动 */
			{
				calib_exit(m);
				return CALIB_STATE_FAILED;
			}

			/* 写入 dev_mt6701 运行时 */
			m->mt6701.dir = dir;
			/* 写入 motor_param_t（持久化） */
			motor_param_set_enc_direction(io->param, enc_dir);
			s_step = 3;
			return CALIB_STATE_RUNNING;
		}

		case 3: /* 撤销电压，完成 */
			calib_exit(m);
			return CALIB_STATE_DONE;

		default:
			calib_exit(m);
			return CALIB_STATE_FAILED;
	}
}

/* ===================== 其余子模式（桩）===================== */
static calib_state_e poll_linearity(void)
{
	return CALIB_STATE_DONE;
}
static calib_state_e poll_sincos(void)
{
	return CALIB_STATE_DONE;
}
static calib_state_e poll_multiturn_zero(void)
{
	return CALIB_STATE_DONE;
}

/* ===================== ops 接口 ===================== */
static bool calib_level3_start(uint8_t submode, motor_param_t *param, float dt)
{
	(void)param;
	(void)dt;

	s_submode = submode;
	s_step = 0;
	s_tick = 0;
	s_sample_cnt = 0;
	s_angle_sum = 0.0f;
	s_orig_ele_cb = NULL;

	switch (submode)
	{
		case CALIB_L3_ZERO_OFFSET:
		case CALIB_L3_DIRECTION:
		case CALIB_L3_LINEARITY:
		case CALIB_L3_SINCOS:
		case CALIB_L3_MULTITURN_ZERO:
			return true;
		default:
			return false;
	}
}

static calib_state_e calib_level3_poll(void)
{
	switch (s_submode)
	{
		case CALIB_L3_ZERO_OFFSET: return poll_zero_offset();
		case CALIB_L3_DIRECTION: return poll_direction();
		case CALIB_L3_LINEARITY: return poll_linearity();
		case CALIB_L3_SINCOS: return poll_sincos();
		case CALIB_L3_MULTITURN_ZERO: return poll_multiturn_zero();
		default: return CALIB_STATE_FAILED;
	}
}

static void calib_level3_abort(void)
{
	const calib_io_t *io = calib_mgr_get_io();
	if (io != NULL && io->motor != NULL)
		calib_exit(io->motor);
	s_step = 0;
}

const calib_level_ops_t calib_level3_ops = {
	.start = calib_level3_start,
	.poll = calib_level3_poll,
	.abort = calib_level3_abort,
};
