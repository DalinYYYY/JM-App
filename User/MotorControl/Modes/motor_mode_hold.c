/**
 * @file motor_mode_hold.c
 * @brief 位置保持模式：主动锁定当前位置（位置环）
 */
#include "motor_mode.h"

void motor_mode_hold_run(motor_ctrl_t *ctrl)
{
	motor_ref_t *ref = &ctrl->ref;
	ref->pos_profile = MOTOR_PID_PROFILE_POSITION;
	ref->vel_profile = MOTOR_PID_PROFILE_VELOCITY;
	ref->ctrl_type = REF_CTRL_POSITION;
	ref->pos = ctrl->fb.pos;  /* 目标=当前位置，实现主动保持 */
	ref->vel_ff = 0.0f;
}
