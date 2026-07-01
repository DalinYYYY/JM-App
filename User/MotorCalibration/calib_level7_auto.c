/**
 * @file calib_level7_auto.c
 * @brief L7 自动化集成校准（一键全自动）
 * @note 真实实现：按顺序调用 L1→L6 的所有必要子模式。
 *       L7 维护自己的子标定序列状态机，不依赖 calib_mgr 的 active_ops
 *       （否则会与 calib_mgr 的单例 active_ops 冲突）。
 *       step 字段标识当前执行到第几步，通过 calib_status_t.step 上报。
 */
#include "calib_types.h"

static motor_param_t *s_param;
static float s_dt;
static uint8_t s_step; /* 当前执行步骤（0=未开始） */

/* L7 内部子标定序列定义（真实实现时填充） */
static const struct
{
	uint8_t level;
	uint8_t submode;
} s_sequence[] = {
	{CALIB_LEVEL1_DRIVER,    CALIB_L1_ADC_OFFSET    },
	{CALIB_LEVEL1_DRIVER,    CALIB_L1_ADC_GAIN      },
	{CALIB_LEVEL1_DRIVER,    CALIB_L1_CURRENT_SENSOR},
	{CALIB_LEVEL2_MOTOR,     CALIB_L2_PHASE_SEQ     },
	{CALIB_LEVEL2_MOTOR,     CALIB_L2_POLE_PAIRS    },
	{CALIB_LEVEL2_MOTOR,     CALIB_L2_RL_FLUX       },
	{CALIB_LEVEL3_ENCODER,   CALIB_L3_ZERO_OFFSET   },
	{CALIB_LEVEL3_ENCODER,   CALIB_L3_DIRECTION     },
	{CALIB_LEVEL4_TORQUE,    CALIB_L4_KT            },
	{CALIB_LEVEL5_NONLINEAR, CALIB_L5_COGGING       },
	{CALIB_LEVEL5_NONLINEAR, CALIB_L5_FRICTION      },
	{CALIB_LEVEL6_SYSTEM,    CALIB_L6_INERTIA       },
	{CALIB_LEVEL6_SYSTEM,    CALIB_L6_PID_AUTOTUNE  },
};
#define L7_SEQ_LEN (sizeof(s_sequence) / sizeof(s_sequence[0]))

static bool calib_level7_start(uint8_t submode, motor_param_t *param, float dt)
{
	s_param = param;
	s_dt = dt;
	s_step = 0;

	switch (submode)
	{
		case CALIB_L7_FULL_AUTO:
			/* TODO: 启动序列中第一个子标定 */
			return true;
		default:
			return false;
	}
}

static calib_state_e calib_level7_poll(void)
{
	/* 桩：直接返回完成。
	 * 真实实现：
	 *   1. 调用当前 step 对应 level 的 start/poll
	 *   2. 子标定 DONE 后 s_step++，启动下一个
	 *   3. s_step >= L7_SEQ_LEN 时返回 DONE */
	(void)s_param;
	(void)s_dt;
	(void)s_step;
	(void)L7_SEQ_LEN;
	return CALIB_STATE_DONE;
}

static void calib_level7_abort(void)
{
	s_step = 0;
}

const calib_level_ops_t calib_level7_ops = {
	.start = calib_level7_start,
	.poll = calib_level7_poll,
	.abort = calib_level7_abort,
};
