/**
 * @file motor_mode_open_loop.c
 * @brief 开环电压模式：OPEN_LOOP / VOLTAGE_VECTOR
 */
#include "motor_mode.h"

void motor_mode_open_loop_run(motor_ctrl_t *ctrl)
{
	motor_param_t *p = ctrl->param;
	motor_ref_t *ref = &ctrl->ref;
	float rated_v = (p)->motor_base.rated_voltage;
	ref->pos_profile = MOTOR_PID_PROFILE_POSITION;
	ref->vel_profile = MOTOR_PID_PROFILE_VELOCITY;
	ref->ctrl_type = REF_CTRL_VOLTAGE;
	ref->ud       = motor_mode_clamp(ctrl->cmd.id,     -rated_v, rated_v);
	ref->voltage  = motor_mode_clamp(ctrl->cmd.torque, -rated_v, rated_v);
}
