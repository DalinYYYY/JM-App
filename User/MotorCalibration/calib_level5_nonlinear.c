/**
 * @file calib_level5_nonlinear.c
 * @brief L5 非线性补偿（齿槽/摩擦/死区补偿/磁饱和）
 * @note L5.3 死区补偿标定已实现（三点最小二乘拟合 R 和 V_dt）。
 *       L5.1/2/4 仍为桩实现。
 */
#include <math.h>
#include "calib_types.h"
#include "calib_config.h"
#include "calib_mgr.h"
#include "calib_hw.h"
#include "dev_motor.h"
#include "motor_param.h"
#include "calib_step.h"
#include "calib_validate.h"
#include "motor_info_calib.h"

/* ===================== 模块私有状态 ===================== */
static struct
{
	uint8_t submode;
	calib_step_t step;
	calib_hw_session_t session;
	/* 三点拟合：3 个 (id, ud) 数据点 */
	float id_samples[3];
	float ud_samples[3];
	uint8_t point_idx;
	float test_voltage;
	float sum_id;
	uint8_t sample_cnt;
} s_l5;

/* ===================== L5.3 死区补偿标定（三点拟合）======================
 * 原理：ud = R×id + V_dt（V_dt 为死区等效压降，含体二极管/MOSFET-Rds/开关延迟）
 * 施加 3 个不同 ud 档位，稳态后测 id，最小二乘拟合斜率=R，截距=V_dt。
 *
 * STEP 0: 进入会话，施加低档电压 V1
 * STEP 1: 等待稳态
 * STEP 2: 采样 id 均值，切中档 V2
 * STEP 3: 等待稳态
 * STEP 4: 采样 id 均值，切高档 V3
 * STEP 5: 等待稳态
 * STEP 6: 采样 id 均值，三点拟合 R 和 V_dt，校验写入
 * ===================================================================== */
static calib_state_e poll_deadtime_comp(void)
{
	const calib_io_t *io = calib_mgr_get_io();
	dev_motor_t *m = io->motor;

	/* 三档测试电压：低/中/高，均产生正向 id */
	const float v_table[3] = {
		CALIB_CFG_L2_R_TEST_VOLTAGE_LO_V,
		CALIB_CFG_L2_R_TEST_VOLTAGE_V * 0.7f,
		CALIB_CFG_L2_R_TEST_VOLTAGE_V,
	};

	switch (s_l5.step.cur)
	{
		case 0: /* 施加 V1 */
			calib_hw_enter(&s_l5.session, m);
			s_l5.point_idx = 0;
			s_l5.test_voltage = v_table[0];
			calib_hw_apply_voltage(&s_l5.session, s_l5.test_voltage, 0.0f, 0.0f);
			s_l5.sum_id = 0.0f;
			s_l5.sample_cnt = 0;
			calib_mgr_set_step(0);
			calib_step_next(&s_l5.step, 1);
			calib_mgr_set_step(1);
			return CALIB_STATE_RUNNING;

		case 1: /* 等待稳态 */
		case 3: /* 等待稳态 */
		case 5: /* 等待稳态 */
			calib_hw_apply_voltage(&s_l5.session, s_l5.test_voltage, 0.0f, 0.0f);
			if (calib_step_wait(&s_l5.step, CALIB_CFG_L2_R_TEST_TICKS))
				return CALIB_STATE_RUNNING;
			/* 进入采样步 */
			s_l5.sum_id = 0.0f;
			s_l5.sample_cnt = 0;
			calib_step_next(&s_l5.step, s_l5.step.cur + 1);
			calib_mgr_set_step(s_l5.step.cur);
			return CALIB_STATE_RUNNING;

		case 2: /* 采样点 0 */
		case 4: /* 采样点 1 */
		case 6: /* 采样点 2 */
		{
			calib_hw_apply_voltage(&s_l5.session, s_l5.test_voltage, 0.0f, 0.0f);
			m->foc.clarke(&m->foc);
			m->foc.park(&m->foc);
			float id = m->foc.i_dq.d;
			if (!isfinite(id) || fabsf(id) < 0.001f)
			{
				calib_mgr_set_fail_reason(CALIB_FAIL_SAMPLE_ABNORMAL);
				calib_hw_exit(&s_l5.session);
				return CALIB_STATE_FAILED;
			}
			s_l5.sum_id += id;
			s_l5.sample_cnt++;
			if (s_l5.sample_cnt < CALIB_CFG_L2_R_SAMPLE_COUNT)
				return CALIB_STATE_RUNNING;

			/* 记录 (id, ud) 数据点 */
			uint8_t idx = (uint8_t)((s_l5.step.cur - 2) / 2); /* case 2→0, 4→1, 6→2 */
			s_l5.id_samples[idx] = s_l5.sum_id / (float)s_l5.sample_cnt;
			s_l5.ud_samples[idx] = s_l5.test_voltage;

			/* 切下一档或进入拟合 */
			if (idx < 2)
			{
				s_l5.point_idx = (uint8_t)(idx + 1);
				s_l5.test_voltage = v_table[s_l5.point_idx];
				calib_hw_apply_voltage(&s_l5.session, s_l5.test_voltage, 0.0f, 0.0f);
				calib_step_next(&s_l5.step, s_l5.step.cur + 1);
				calib_mgr_set_step(s_l5.step.cur);
				return CALIB_STATE_RUNNING;
			}
			/* 三点采完，进入拟合 */
			calib_step_next(&s_l5.step, 7);
			calib_mgr_set_step(7);
			return CALIB_STATE_RUNNING;
		}

		case 7: /* 三点最小二乘拟合 */
		{
			calib_hw_exit(&s_l5.session);
			/* 最小二乘: ud = R*id + V_dt
			 * R = (N×Σ(id×ud) - Σid×Σud) / (N×Σ(id²) - (Σid)²)
			 * V_dt = (Σud - R×Σid) / N */
			float s_id = 0, s_ud = 0, s_id2 = 0, s_id_ud = 0;
			for (int i = 0; i < 3; i++)
			{
				s_id    += s_l5.id_samples[i];
				s_ud    += s_l5.ud_samples[i];
				s_id2   += s_l5.id_samples[i] * s_l5.id_samples[i];
				s_id_ud += s_l5.id_samples[i] * s_l5.ud_samples[i];
			}
			float denom = 3.0f * s_id2 - s_id * s_id;
			if (fabsf(denom) < 1e-9f)
			{
				calib_mgr_set_fail_reason(CALIB_FAIL_SAMPLE_ABNORMAL);
				return CALIB_STATE_FAILED;
			}
			float R_fit = (3.0f * s_id_ud - s_id * s_ud) / denom;
			float V_dt = (s_ud - R_fit * s_id) / 3.0f;

			if (!calib_validate_r(R_fit) || !isfinite(V_dt) || V_dt < 0.0f || V_dt > 2.0f)
			{
				calib_mgr_set_fail_reason(CALIB_FAIL_OUT_OF_RANGE);
				return CALIB_STATE_FAILED;
			}
			/* V_dt 写入 motor_param，供 foc_decoupling 运行时使用 */
			motor_param_set_deadtime_comp_v(io->param, V_dt);
			motor_param_set_deadtime_comp_enable(io->param, 1);
			/* R 也顺便更新（三点法精度高于两点法）*/
			motor_param_set_r(io->param, R_fit);
			(void)motor_info_calib_submit_r(R_fit);
			calib_mgr_mark_done(CALIB_LEVEL5_NONLINEAR, CALIB_L5_DEADTIME_COMP);
			calib_step_reset(&s_l5.step);
			return CALIB_STATE_DONE;
		}

		default:
			calib_mgr_set_fail_reason(CALIB_FAIL_TIMEOUT);
			calib_hw_exit(&s_l5.session);
			return CALIB_STATE_FAILED;
	}
}

/* ===================== L5.1/2/4 桩实现 ===================== */
static calib_state_e poll_cogging(void)
{
	return CALIB_STATE_DONE;
}
static calib_state_e poll_friction(void)
{
	return CALIB_STATE_DONE;
}
static calib_state_e poll_saturation(void)
{
	return CALIB_STATE_DONE;
}

static bool calib_level5_start(uint8_t submode, motor_param_t *param, float dt)
{
	(void)param;
	(void)dt;
	s_l5.submode = submode;
	calib_step_reset(&s_l5.step);
	s_l5.session.motor = NULL;
	s_l5.session.orig_ele_cb = NULL;
	s_l5.session.forced_ele_angle = 0.0f;

	switch (submode)
	{
		case CALIB_L5_COGGING:
		case CALIB_L5_FRICTION:
		case CALIB_L5_DEADTIME_COMP:
		case CALIB_L5_SATURATION:
			return true;
		default:
			return false;
	}
}

static calib_state_e calib_level5_poll(void)
{
	switch (s_l5.submode)
	{
		case CALIB_L5_COGGING:        return poll_cogging();
		case CALIB_L5_FRICTION:       return poll_friction();
		case CALIB_L5_DEADTIME_COMP:  return poll_deadtime_comp();
		case CALIB_L5_SATURATION:     return poll_saturation();
		default: return CALIB_STATE_FAILED;
	}
}

static void calib_level5_abort(void)
{
	const calib_io_t *io = calib_mgr_get_io();
	if (io != NULL && io->motor != NULL)
		calib_hw_exit(&s_l5.session);
	calib_step_reset(&s_l5.step);
}

const calib_level_ops_t calib_level5_ops = {
	.start = calib_level5_start,
	.poll  = calib_level5_poll,
	.abort = calib_level5_abort,
};
