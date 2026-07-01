/**
 * @file calib_level4_torque.c
 * @brief L4 转矩基础校准（力矩常数 Kt）
 */
#include "calib_types.h"

static motor_param_t *s_param;
static float s_dt;

static calib_state_e poll_kt(void) { return CALIB_STATE_DONE; }

static bool calib_level4_start(uint8_t submode, motor_param_t *param, float dt)
{
	s_param = param;
	s_dt = dt;

	switch (submode)
	{
		case CALIB_L4_KT:
			return true;
		default:
			return false;
	}
}

static calib_state_e calib_level4_poll(void)
{
	/* L4 只有一个子模式，无需 switch 分发 */
	return poll_kt();
}

static void calib_level4_abort(void) {}

const calib_level_ops_t calib_level4_ops = {
	.start = calib_level4_start,
	.poll  = calib_level4_poll,
	.abort = calib_level4_abort,
};
