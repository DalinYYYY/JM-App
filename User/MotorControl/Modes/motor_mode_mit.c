/**
 * @file motor_mode_mit.c
 * @brief MIT Cheetah 控制：kp*pos_err + kd*vel_err + torque_ff
 */
#include "motor_mode.h"

void motor_mode_mit_run(motor_ctrl_t *ctrl)
{
	motor_param_t *p = ctrl->param;
	motor_ref_t *ref = &ctrl->ref;
	float peak_t = (p)->motor_base.peak_torque;

	float pos_err = ctrl->cmd.pos - ctrl->fb.pos;
	float vel_err = ctrl->cmd.vel - ctrl->fb.vel;
	float torque = ctrl->cmd.kp * pos_err + ctrl->cmd.kd * vel_err + ctrl->cmd.torque_ff;

	ref->pos_profile = MOTOR_PID_PROFILE_POSITION;
	ref->vel_profile = MOTOR_PID_PROFILE_VELOCITY;
	ref->ctrl_type = REF_CTRL_TORQUE;
	ref->torque = motor_mode_clamp(torque, -peak_t, peak_t);
	ref->torque_ff = 0.0f;
}
