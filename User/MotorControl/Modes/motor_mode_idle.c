/**
 * @file motor_mode_idle.c
 * @brief 空闲模式：PWM 置零，参考保持当前位置
 */
#include "motor_mode.h"

void motor_mode_idle_run(motor_ctrl_t *ctrl)
{
	motor_ref_t *ref = &ctrl->ref;
	ref->pos_profile = PID_PROFILE_POSITION;
	ref->vel_profile = PID_PROFILE_VELOCITY;
	ref->ctrl_type = REF_CTRL_IDLE;
	ref->pos = ctrl->fb.pos;
	ref->vel_ff = 0.0f;
}
