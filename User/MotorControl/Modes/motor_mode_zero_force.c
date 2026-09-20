/**
 * @file motor_mode_zero_force.c
 * @brief 零力模式 (0x36)：电流环闭环 id=iq=0 的零转矩自由
 * @details 非PWM高阻，而是电流闭环把 dq 电流目标钳在零，电机可被外力
 *          自由拖动且不产生反电动势制动转矩，用于手拖示教/被动跟随。
 */
#include "motor_mode.h"

void motor_mode_zero_force_run(motor_ctrl_t *ctrl)
{
	motor_ref_t *ref = &ctrl->ref;
	ref->pos_profile = MOTOR_PID_PROFILE_POSITION;
	ref->vel_profile = MOTOR_PID_PROFILE_VELOCITY;
	ref->ctrl_type = REF_CTRL_CURRENT;
	ref->id = 0.0f;
	ref->iq = 0.0f;
}
