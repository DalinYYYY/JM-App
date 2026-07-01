/**
 * @file calib_level3_encoder.c
 * @brief L3 编码器校准（零位/方向/线性度/正余弦/多圈零点）
 */
#include "calib_types.h"

static uint8_t s_submode;
static motor_param_t *s_param;
static float s_dt;

static calib_state_e poll_zero_offset(void)    { return CALIB_STATE_DONE; }
static calib_state_e poll_direction(void)      { return CALIB_STATE_DONE; }
static calib_state_e poll_linearity(void)      { return CALIB_STATE_DONE; }
static calib_state_e poll_sincos(void)         { return CALIB_STATE_DONE; }
static calib_state_e poll_multiturn_zero(void) { return CALIB_STATE_DONE; }

static bool calib_level3_start(uint8_t submode, motor_param_t *param, float dt)
{
	s_param = param;
	s_dt = dt;
	s_submode = submode;

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
		case CALIB_L3_ZERO_OFFSET:    return poll_zero_offset();
		case CALIB_L3_DIRECTION:      return poll_direction();
		case CALIB_L3_LINEARITY:      return poll_linearity();
		case CALIB_L3_SINCOS:         return poll_sincos();
		case CALIB_L3_MULTITURN_ZERO: return poll_multiturn_zero();
		default:                      return CALIB_STATE_FAILED;
	}
}

static void calib_level3_abort(void) {}

const calib_level_ops_t calib_level3_ops = {
	.start = calib_level3_start,
	.poll  = calib_level3_poll,
	.abort = calib_level3_abort,
};
