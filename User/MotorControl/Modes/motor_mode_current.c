/**
 * @file motor_mode_current.c
 * @brief 电流模式：CURRENT / FIELD_WEAKENING / SENSORLESS
 */
#include "motor_mode.h"

void motor_mode_current_run(motor_ctrl_t *ctrl)
{
	motor_param_t *p = ctrl->param;
	motor_ref_t *ref = &ctrl->ref;
	float peak_i = (p)->motor_base.peak_current;
	ref->pos_profile = MOTOR_PID_PROFILE_POSITION;
	ref->vel_profile = MOTOR_PID_PROFILE_VELOCITY;
	ref->ctrl_type = REF_CTRL_CURRENT;
	ref->id = motor_mode_clamp(ctrl->cmd.id, -peak_i, peak_i);
	ref->iq = motor_mode_clamp(ctrl->cmd.iq, -peak_i, peak_i);
}
