/**
 * @file motor_mode_open_loop.c
 * @brief 开环电压模式：OPEN_LOOP / VOLTAGE_VECTOR
 */
#include "motor_mode.h"

void motor_mode_open_loop_run(motor_ctrl_t *ctrl)
{
	motor_param_t *p = ctrl->param;
	motor_ref_t *ref = &ctrl->ref;
	float rated_v = motor_param_get_rated_voltage(p);
	ref->pos_profile = PID_PROFILE_POSITION;
	ref->vel_profile = PID_PROFILE_VELOCITY;
	ref->ctrl_type = REF_CTRL_VOLTAGE;
	ref->voltage = motor_mode_clamp(ctrl->cmd.torque, -rated_v, rated_v);
}
