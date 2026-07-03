/**
 * @file calib_level3_encoder.c
 * @brief L3 编码器校准（零位/方向/线性度/正余弦/多圈零点）
 * @note 重构后：硬件访问通过 calib_hw 共享层，参数通过 calib_config.h 集中管理。
 *       零位标定（d轴对齐法）与方向标定（施加uq观测角度变化）为真实实现；
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
#include "calib_config.h"
#include "calib_mgr.h"
#include "calib_hw.h"
#include "dev_motor.h"     /* 通过 dev_motor_t.encoder 抽象层访问编码器，不直接依赖具体芯片 */
#include "motor_param.h"
#include "calib_step.h"
#include "calib_validate.h"
#include "motor_info_calib.h"

/* ===================== 模块私有状态（合并为单一结构体）===================== */
static struct
{
	uint8_t submode;
	calib_step_t step;          /* 统一状态机骨架（cur/tick/sample_cnt/sample_sum）*/
	float dir_start_angle;      /* 方向测试起始角度 */
	calib_hw_session_t session; /* 标定电压会话（替换电角度回调 + 施加电压）*/
} s_l3;

/* ===================== 零位标定状态机 =====================
 * STEP 0: 初始化，施加 ud 电压（电角度强制为0），转子开始对齐
 * STEP 1: 等待转子稳定对齐（CALIB_CFG_L3_ALIGN_TICKS 个周期）
 * STEP 2: 多次采样编码器原始角度取平均
 * STEP 3: 写入 offset 到抽象编码器层和 encoder_param，撤销电压，完成
 * ========================================================== */
static calib_state_e poll_zero_offset(void)
{
	const calib_io_t *io = calib_mgr_get_io();
	dev_motor_t *m = io->motor;

	switch (s_l3.step.cur)
	{
		case 0: /* 施加 d 轴对齐电压 */
			calib_hw_enter(&s_l3.session, m);
			calib_hw_apply_voltage(&s_l3.session, CALIB_CFG_L3_ALIGN_VOLTAGE_V, 0.0f, 0.0f);
			calib_step_next(&s_l3.step, 1);
			return CALIB_STATE_RUNNING;

		case 1: /* 等待转子稳定对齐 */
			calib_hw_apply_voltage(&s_l3.session, CALIB_CFG_L3_ALIGN_VOLTAGE_V, 0.0f, 0.0f);
			if (calib_step_wait(&s_l3.step, CALIB_CFG_L3_ALIGN_TICKS))
				return CALIB_STATE_RUNNING;
			calib_step_next(&s_l3.step, 2);
			return CALIB_STATE_RUNNING;

		case 2: /* 多次采样编码器原始角度 */
			calib_hw_apply_voltage(&s_l3.session, CALIB_CFG_L3_ALIGN_VOLTAGE_V, 0.0f, 0.0f);
			calib_step_accumulate(&s_l3.step, calib_hw_get_encoder_raw_deg(m));
			if (s_l3.step.sample_cnt < CALIB_CFG_L3_SAMPLE_COUNT)
				return CALIB_STATE_RUNNING;
			calib_step_next(&s_l3.step, 3);
			return CALIB_STATE_RUNNING;

		case 3: /* 写入标定结果，完成 */
		{
			float avg_deg = calib_step_average(&s_l3.step);
			/* 通过抽象编码器层写入运行时（offset = 对齐位置的原始角度，使 mech_angle=0）*/
			m->encoder.set_offset(&m->encoder, avg_deg);
			m->encoder.set_dir(&m->encoder, 1); /* 零位标定先置 CW，方向由后续方向标定确定 */
			/* 写入 motor_param_t（持久化），enc_offset 用 deg 角度值 */
			if (!calib_validate_enc_offset(avg_deg))
			{
				calib_hw_exit(&s_l3.session);
				return CALIB_STATE_FAILED;
			}
			motor_param_set_enc_offset(io->param, avg_deg);
			motor_param_set_enc_direction(io->param, 1); /* 1=CW */
			motor_param_set_elec_angle_bias(io->param, 0.0f);
			/* 提交零位标定结果到 motor_info */
			(void)motor_info_calib_submit_enc_zero(0.0f, avg_deg, 1);
			calib_hw_exit(&s_l3.session);
			calib_mgr_mark_done(CALIB_LEVEL3_ENCODER, CALIB_L3_ZERO_OFFSET);
			calib_step_reset(&s_l3.step);
			return CALIB_STATE_DONE;
		}

		default:
			calib_hw_exit(&s_l3.session);
			return CALIB_STATE_FAILED;
	}
}

/* ===================== 方向标定状态机 =====================
 * STEP 0: 确保零位已标定，施加正向 uq 电压（电角度=0）
 * STEP 1: 持续施加 uq，等待 CALIB_CFG_L3_DIR_TICKS 个周期让电机转动
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

	switch (s_l3.step.cur)
	{
		case 0: /* 施加正向 uq 电压，记录起始角度 */
			calib_hw_enter(&s_l3.session, m);
			/* 电角度=0 时 uq>0 产生正向力矩（q轴超前d轴90°，即α轴方向）*/
			s_l3.dir_start_angle = calib_hw_get_encoder_mech_angle(m);
			calib_hw_apply_voltage(&s_l3.session, 0.0f, CALIB_CFG_L3_DIR_VOLTAGE_V, 0.0f);
			calib_step_next(&s_l3.step, 1);
			return CALIB_STATE_RUNNING;

		case 1: /* 持续施加 uq，等待电机转动 */
			calib_hw_apply_voltage(&s_l3.session, 0.0f, CALIB_CFG_L3_DIR_VOLTAGE_V, 0.0f);
			if (calib_step_wait(&s_l3.step, CALIB_CFG_L3_DIR_TICKS))
				return CALIB_STATE_RUNNING;
			calib_step_next(&s_l3.step, 2);
			return CALIB_STATE_RUNNING;

		case 2: /* 采样当前角度，判定方向 */
		{
			float end_angle = calib_hw_get_encoder_mech_angle(m);
			float delta = end_angle - s_l3.dir_start_angle;
			/* 处理 0/360 跳变：若 delta 绝对值 >180，说明跨越了 0° 边界 */
			if (delta > 180.0f)
				delta -= 360.0f;
			else if (delta < -180.0f)
				delta += 360.0f;

			int8_t enc_dir;
			if (delta > 1.0f) /* 角度增大 → 正向 CW */
				enc_dir = 1;
			else if (delta < -1.0f) /* 角度减小 → 反向 CCW */
				enc_dir = -1;
			else /* 角度几乎无变化，可能电机未转动 */
			{
				calib_hw_exit(&s_l3.session);
				return CALIB_STATE_FAILED;
			}

			if (!calib_validate_enc_direction(enc_dir))
			{
				calib_hw_exit(&s_l3.session);
				return CALIB_STATE_FAILED;
			}
			/* 通过抽象编码器层写入运行时方向（统一 -1/1 约定）*/
			m->encoder.set_dir(&m->encoder, enc_dir);
			/* 写入 motor_param_t（持久化） */
			motor_param_set_enc_direction(io->param, enc_dir);
			(void)motor_info_calib_submit_enc_direction(enc_dir);
			calib_step_next(&s_l3.step, 3);
			return CALIB_STATE_RUNNING;
		}

		case 3: /* 撤销电压，完成 */
			calib_hw_exit(&s_l3.session);
			calib_mgr_mark_done(CALIB_LEVEL3_ENCODER, CALIB_L3_DIRECTION);
			calib_step_reset(&s_l3.step);
			return CALIB_STATE_DONE;

		default:
			calib_hw_exit(&s_l3.session);
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

	s_l3.submode = submode;
	calib_step_reset(&s_l3.step);
	s_l3.session.motor = NULL;
	s_l3.session.orig_ele_cb = NULL;
	s_l3.session.forced_ele_angle = 0.0f;

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
	switch (s_l3.submode)
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
		calib_hw_exit(&s_l3.session);
	calib_step_reset(&s_l3.step);
}

const calib_level_ops_t calib_level3_ops = {
	.start = calib_level3_start,
	.poll = calib_level3_poll,
	.abort = calib_level3_abort,
};
