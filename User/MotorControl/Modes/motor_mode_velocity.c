/**
 * @file motor_mode_velocity.c
 * @brief 速度模式：VELOCITY / VELOCITY_TORQUE
 */
#include "motor_mode.h"

void motor_mode_velocity_run(motor_ctrl_t *ctrl)
{
	motor_param_t *p = ctrl->param;
	motor_ref_t *ref = &ctrl->ref;
	float max_spd = motor_param_get_max_speed(p);
	float peak_t = motor_param_get_peak_torque(p);

	ref->pos_profile = MOTOR_PID_PROFILE_POSITION;
	ref->vel_profile = MOTOR_PID_PROFILE_VELOCITY;
	ref->ctrl_type = REF_CTRL_VELOCITY;
	ref->vel = motor_mode_clamp(ctrl->cmd.vel, -max_spd, max_spd);
	ref->torque_ff = ctrl->cmd.torque_ff;

	/* VELOCITY_TORQUE 变体：额外设置力矩限幅 */
	if (ctrl->run_state == RUN_STATE_VELOCITY_TORQUE)
	{
		ref->torque = motor_mode_clamp(ctrl->cmd.torque, -peak_t, peak_t);
	}
}
