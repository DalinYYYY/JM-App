/**
 * @file motor_mode_profile_velocity.c
 * @brief 速度轮廓模式 PV：速度模式 + 斜坡规划（前期直接复用速度模式）
 * @todo 后续在此文件内实现斜坡发生器，平滑速度指令
 */
#include "motor_mode.h"

void motor_mode_profile_velocity_run(motor_ctrl_t *ctrl)
{
	motor_param_t *p = ctrl->param;
	motor_ref_t *ref = &ctrl->ref;
	float max_spd = motor_param_get_max_speed(p);

	ref->pos_profile = MOTOR_PID_PROFILE_POSITION;
	ref->vel_profile = MOTOR_PID_PROFILE_VELOCITY;
	ref->ctrl_type = REF_CTRL_VELOCITY;
	/* 前期直接透传，后续加斜坡：ref->vel = slope_limit(ctrl->cmd.vel, ...) */
	ref->vel = motor_mode_clamp(ctrl->cmd.vel, -max_spd, max_spd);
	ref->torque_ff = ctrl->cmd.torque_ff;
}
