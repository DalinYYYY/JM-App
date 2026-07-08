/**
 * @file motor_mode_test_sweep.c
 * @brief 扫频测试模式：生成正弦力矩指令
 * @note 相位/频率存于 motor_ctrl_t 实例字段，避免 static 导致多电机共享
 */
#include "motor_mode.h"
#include <math.h>

void motor_mode_test_sweep_run(motor_ctrl_t *ctrl)
{
	motor_ref_t *ref = &ctrl->ref;
	ref->pos_profile = MOTOR_PID_PROFILE_POSITION;
	ref->vel_profile = MOTOR_PID_PROFILE_VELOCITY;
	ref->ctrl_type = REF_CTRL_TORQUE;
	ref->torque = sinf(ctrl->test_phase) * 0.5f; /* 0.5N·m 振幅 */
	ref->torque_ff = 0.0f;
	ctrl->test_phase += 2.0f * (float)M_PI * ctrl->test_freq * ctrl->dt;
}
