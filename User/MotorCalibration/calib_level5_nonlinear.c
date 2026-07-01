/**
 * @file calib_level5_nonlinear.c
 * @brief L5 非线性补偿（齿槽/摩擦/死区补偿/磁饱和）
 */
#include "calib_types.h"

static uint8_t s_submode;
static motor_param_t *s_param;
static float s_dt;

static calib_state_e poll_cogging(void)
{
	return CALIB_STATE_DONE;
}
static calib_state_e poll_friction(void)
{
	return CALIB_STATE_DONE;
}
static calib_state_e poll_deadtime_comp(void)
{
	return CALIB_STATE_DONE;
}
static calib_state_e poll_saturation(void)
{
	return CALIB_STATE_DONE;
}

static bool calib_level5_start(uint8_t submode, motor_param_t *param, float dt)
{
	s_param = param;
	s_dt = dt;
	s_submode = submode;

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
	switch (s_submode)
	{
		case CALIB_L5_COGGING: return poll_cogging();
		case CALIB_L5_FRICTION: return poll_friction();
		case CALIB_L5_DEADTIME_COMP: return poll_deadtime_comp();
		case CALIB_L5_SATURATION: return poll_saturation();
		default: return CALIB_STATE_FAILED;
	}
}

static void calib_level5_abort(void)
{
}

const calib_level_ops_t calib_level5_ops = {
	.start = calib_level5_start,
	.poll = calib_level5_poll,
	.abort = calib_level5_abort,
};
