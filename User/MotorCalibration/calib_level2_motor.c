/**
 * @file calib_level2_motor.c
 * @brief L2 电机电气身份辨识（相序/极对数/R-L-flux）
 */
#include "calib_types.h"

static uint8_t s_submode;
static motor_param_t *s_param;
static float s_dt;

static calib_state_e poll_phase_seq(void)   { return CALIB_STATE_DONE; }
static calib_state_e poll_pole_pairs(void)  { return CALIB_STATE_DONE; }
static calib_state_e poll_rl_flux(void)     { return CALIB_STATE_DONE; }

static bool calib_level2_start(uint8_t submode, motor_param_t *param, float dt)
{
	s_param = param;
	s_dt = dt;
	s_submode = submode;

	switch (submode)
	{
		case CALIB_L2_PHASE_SEQ:
		case CALIB_L2_POLE_PAIRS:
		case CALIB_L2_RL_FLUX:
			return true;
		default:
			return false;
	}
}

static calib_state_e calib_level2_poll(void)
{
	switch (s_submode)
	{
		case CALIB_L2_PHASE_SEQ:  return poll_phase_seq();
		case CALIB_L2_POLE_PAIRS: return poll_pole_pairs();
		case CALIB_L2_RL_FLUX:    return poll_rl_flux();
		default:                  return CALIB_STATE_FAILED;
	}
}

static void calib_level2_abort(void) {}

const calib_level_ops_t calib_level2_ops = {
	.start = calib_level2_start,
	.poll  = calib_level2_poll,
	.abort = calib_level2_abort,
};
