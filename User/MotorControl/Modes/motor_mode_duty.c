/**
 * @file motor_mode_duty.c
 * @brief 占空比直控模式
 */
#include "motor_mode.h"

void motor_mode_duty_run(motor_ctrl_t *ctrl)
{
	motor_ref_t *ref = &ctrl->ref;
	ref->pos_profile = PID_PROFILE_POSITION;
	ref->vel_profile = PID_PROFILE_VELOCITY;
	ref->ctrl_type = REF_CTRL_DUTY;
	ref->duty = motor_mode_clamp(ctrl->cmd.torque, -1.0f, 1.0f);
}
