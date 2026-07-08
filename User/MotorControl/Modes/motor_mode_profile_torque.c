/**
 * @file motor_mode_profile_torque.c
 * @brief 力矩轮廓模式 PT：力矩模式 + 斜坡规划（前期直接复用力矩模式）
 * @todo 后续在此文件内实现力矩斜坡，平滑力矩指令
 */
#include "motor_mode.h"

void motor_mode_profile_torque_run(motor_ctrl_t *ctrl)
{
	motor_param_t *p = ctrl->param;
	motor_ref_t *ref = &ctrl->ref;
	float peak_t = motor_param_get_peak_torque(p);

	ref->pos_profile = MOTOR_PID_PROFILE_POSITION;
	ref->vel_profile = MOTOR_PID_PROFILE_VELOCITY;
	ref->ctrl_type = REF_CTRL_TORQUE;
	/* 前期直接透传，后续加斜坡：ref->torque = slope_limit(ctrl->cmd.torque, ...) */
	ref->torque = motor_mode_clamp(ctrl->cmd.torque, -peak_t, peak_t);
	ref->torque_ff = 0.0f;
}
