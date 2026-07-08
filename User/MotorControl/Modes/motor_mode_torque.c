/**
 * @file motor_mode_torque.c
 * @brief 力矩模式
 */
#include "motor_mode.h"

void motor_mode_torque_run(motor_ctrl_t *ctrl)
{
	motor_param_t *p = ctrl->param;
	motor_ref_t *ref = &ctrl->ref;
	float peak_t = motor_param_get_peak_torque(p);
	ref->pos_profile = MOTOR_PID_PROFILE_POSITION;
	ref->vel_profile = MOTOR_PID_PROFILE_VELOCITY;
	ref->ctrl_type = REF_CTRL_TORQUE;
	ref->torque = motor_mode_clamp(ctrl->cmd.torque, -peak_t, peak_t);
	ref->torque_ff = 0.0f;
}
