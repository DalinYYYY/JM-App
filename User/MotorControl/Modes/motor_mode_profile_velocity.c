/**
 * @file motor_mode_profile_velocity.c
 * @brief 速度轮廓模式 PV：速度模式 + 恒加速度斜坡规划
 * @note 斜坡输出与解析加速度同拍生成（ref->accel=±rate 或 0），
 *       供速度环惯量前馈（accel_ff_gain=J/Kt）抵消动态滞后。
 *       斜坡步进 500rad/s²×dt 远小于 ref_smooth 触发阈值，两层平滑不冲突。
 */
#include "motor_mode.h"

/* 斜坡加速度上限(rad/s²)：与 ref_smooth 默认 vel_rate 同量级，
 * 50rad/s 突变 → 0.1s 过渡 */
#define MOTOR_MODE_PV_RAMP_ACCEL_RAD_S2 500.0f

void motor_profile_vel_reset(motor_ctrl_t *ctrl)
{
	ctrl->profile_vel.active = 0u;
}

void motor_mode_profile_velocity_run(motor_ctrl_t *ctrl)
{
	motor_param_t *p = ctrl->param;
	motor_ref_t *ref = &ctrl->ref;
	motor_profile_vel_t *pv = &ctrl->profile_vel;
	float max_spd = (p)->motor_base.max_speed;

	ref->pos_profile = MOTOR_PID_PROFILE_POSITION;
	ref->vel_profile = MOTOR_PID_PROFILE_VELOCITY;
	ref->ctrl_type = REF_CTRL_VELOCITY;

	/* 模式(重)进入：从实测速度无扰起步 */
	if (pv->active == 0u)
	{
		pv->active = 1u;
		pv->vel = ctrl->fb.vel;
	}

	float target = motor_mode_clamp(ctrl->cmd.vel, -max_spd, max_spd);
	float dv = target - pv->vel;
	float step = MOTOR_MODE_PV_RAMP_ACCEL_RAD_S2 * ctrl->dt;

	if (dv > step)
	{
		pv->vel += step;
		ref->accel = MOTOR_MODE_PV_RAMP_ACCEL_RAD_S2;
	}
	else if (dv < -step)
	{
		pv->vel -= step;
		ref->accel = -MOTOR_MODE_PV_RAMP_ACCEL_RAD_S2;
	}
	else
	{
		pv->vel = target;
		ref->accel = 0.0f;
	}

	ref->vel = pv->vel;
	ref->torque_ff = ctrl->cmd.torque_ff;
}
