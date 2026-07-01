/**
 * @file motor_mode_position.c
 * @brief 位置模式：POSITION / POSITION_VELOCITY / POSITION_TORQUE / PP
 */
#include "motor_mode.h"

void motor_mode_position_run(motor_ctrl_t *ctrl)
{
	motor_param_t *p = ctrl->param;
	motor_ref_t *ref = &ctrl->ref;
	float peak_t = motor_param_get_peak_torque(p);

	ref->pos_profile = PID_PROFILE_POSITION;
	ref->vel_profile = PID_PROFILE_VELOCITY;
	ref->ctrl_type = REF_CTRL_POSITION;
	ref->pos = ctrl->cmd.pos;
	ref->vel_ff = 0.0f;

	switch (ctrl->run_state)
	{
		case RUN_STATE_POSITION_VELOCITY:
			ref->vel_ff = ctrl->cmd.vel;
			break;

		case RUN_STATE_POSITION_TORQUE:
			ref->torque = motor_mode_clamp(ctrl->cmd.torque, -peak_t, peak_t);
			break;

		case RUN_STATE_POSITION:
		default:
			/* PP 前期直接复用基础位置模式，轨迹规划后续在此扩展 */
			break;
	}
}
