/**
 * @file calib_level6_system.c
 * @brief L6 负载系统级校准（惯量/阻尼/回程间隙/PID自整定）
 */
#include "calib_types.h"

static uint8_t s_submode;
static motor_param_t *s_param;
static float s_dt;

static calib_state_e poll_inertia(void)
{
	return CALIB_STATE_DONE;
}
static calib_state_e poll_damping(void)
{
	return CALIB_STATE_DONE;
}
static calib_state_e poll_backlash(void)
{
	return CALIB_STATE_DONE;
}
static calib_state_e poll_pid_autotune(void)
{
	return CALIB_STATE_DONE;
}

static bool calib_level6_start(uint8_t submode, motor_param_t *param, float dt)
{
	s_param = param;
	s_dt = dt;
	s_submode = submode;

	switch (submode)
	{
		case CALIB_L6_INERTIA:
		case CALIB_L6_DAMPING:
		case CALIB_L6_BACKLASH:
		case CALIB_L6_PID_AUTOTUNE:
			return true;
		default:
			return false;
	}
}

static calib_state_e calib_level6_poll(void)
{
	switch (s_submode)
	{
		case CALIB_L6_INERTIA: return poll_inertia();
		case CALIB_L6_DAMPING: return poll_damping();
		case CALIB_L6_BACKLASH: return poll_backlash();
		case CALIB_L6_PID_AUTOTUNE: return poll_pid_autotune();
		default: return CALIB_STATE_FAILED;
	}
}

static void calib_level6_abort(void)
{
}

const calib_level_ops_t calib_level6_ops = {
	.start = calib_level6_start,
	.poll = calib_level6_poll,
	.abort = calib_level6_abort,
};
