/**
 * @file        cascade_control.c
 * @brief       三环级联控制算法层实现（PID 经 pid_profile 统一封装）
 *
 * @author      yangsl
 * @version     1.0
 * @date        2026-06-11
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 */

#include "cascade_control.h"
#include <string.h>

static float clamp(float value, float min, float max)
{
	if (value < min)
		return min;
	if (value > max)
		return max;
	return value;
}

void cascade_control_init(cascade_ctrl_t *c, motor_param_t *param, float dt_pos, float dt_vel)
{
	memset(c, 0, sizeof(cascade_ctrl_t));

	c->param = param;
	c->dt_pos = dt_pos;
	c->dt_vel = dt_vel;
	c->last_ctrl_type = REF_CTRL_IDLE;
	c->last_pos_profile = PID_PROFILE_POSITION;
	c->last_vel_profile = PID_PROFILE_VELOCITY;

	pid_profile_init_state(&c->pid_pos);
	pid_profile_init_state(&c->pid_vel);
}

void cascade_control_reset(cascade_ctrl_t *c)
{
	pid_profile_reset_state(&c->pid_pos);
	pid_profile_reset_state(&c->pid_vel);
	c->vel_setpoint = 0.0f;
}

/**
 * @brief 无扰切换：入环层级或配置文件变化时预装载新入环 PID 积分
 * @details 入环瞬间令 PID 输出≈当前实际值，避免输出突变。
 *          速度环输出为 iq 参考，用当前实际 iq 反推；
 *          位置环输出为速度设定，用当前实际速度反推。
 */
static void cascade_bumpless_preload(cascade_ctrl_t *c, const motor_ref_t *ref, const cascade_fb_t *fb)
{
	switch (ref->ctrl_type)
	{
		case REF_CTRL_POSITION:
			pid_profile_preload(&c->pid_pos, ref->pos_profile, fb->vel, fb->pos);
			pid_profile_preload(&c->pid_vel, ref->vel_profile, fb->iq, fb->vel);
			c->vel_setpoint = fb->vel;
			break;

		case REF_CTRL_VELOCITY:
			pid_profile_preload(&c->pid_vel, ref->vel_profile, fb->iq, fb->vel);
			break;

		default:
			// TORQUE/CURRENT/IDLE 无外环积分，无需预装载
			break;
	}
}

void cascade_control_run_position(cascade_ctrl_t *c, const motor_ref_t *ref, const cascade_fb_t *fb)
{
	if (ref->ctrl_type != REF_CTRL_POSITION)
		return;

	// 位置环：位置误差 → 速度设定（pid_profile 内部已按 max_speed 限幅）
	float vel_sp = pid_profile_calculate(&c->pid_pos, ref->pos_profile, ref->pos, fb->pos, c->dt_pos);

	// 叠加速度前馈
	vel_sp += ref->vel_ff * motor_param_get_velocity_ff_gain(c->param);

	// 速度限幅
	c->vel_setpoint = clamp(vel_sp, -motor_param_get_max_speed(c->param), motor_param_get_max_speed(c->param));
}

void cascade_control_run(cascade_ctrl_t *c, const motor_ref_t *ref, const cascade_fb_t *fb, cascade_out_t *out)
{
	motor_param_t *p = c->param;
	float kt = motor_param_get_kt(p);
	float peak_i = motor_param_get_peak_current(p);

	// 入环层级或配置文件变化：先做无扰预装载
	if (ref->ctrl_type != c->last_ctrl_type || ref->pos_profile != c->last_pos_profile || ref->vel_profile != c->last_vel_profile)
	{
		cascade_bumpless_preload(c, ref, fb);
		c->last_ctrl_type = ref->ctrl_type;
		c->last_pos_profile = ref->pos_profile;
		c->last_vel_profile = ref->vel_profile;
	}

	out->id_ref = 0.0f;
	out->iq_ref = 0.0f;

	switch (ref->ctrl_type)
	{
		// 位置模式：使用位置环输出的速度设定跑速度环
		case REF_CTRL_POSITION:
		{
			float iq = pid_profile_calculate(&c->pid_vel, ref->vel_profile, c->vel_setpoint, fb->vel, c->dt_vel);
			out->iq_ref = clamp(iq, -peak_i, peak_i);
			break;
		}

		// 速度模式：速度环 + 力矩前馈（前馈以电流形式叠加）
		case REF_CTRL_VELOCITY:
		{
			float iq_ff = (kt > 0.0f) ? (ref->torque_ff / kt) : 0.0f;
			float iq = pid_profile_calculate_with_ff(&c->pid_vel, ref->vel_profile, ref->vel, fb->vel, iq_ff, c->dt_vel);
			out->iq_ref = clamp(iq, -peak_i, peak_i);
			break;
		}

		// 力矩模式：力矩 ÷ kt 直接得电流参考，跳过外环
		case REF_CTRL_TORQUE:
			out->iq_ref = (kt > 0.0f) ? clamp(ref->torque / kt, -peak_i, peak_i) : 0.0f;
			break;

		// 电流模式：直接使用 dq 电流参考，跳过全部外环
		case REF_CTRL_CURRENT:
			out->id_ref = clamp(ref->id, -peak_i, peak_i);
			out->iq_ref = clamp(ref->iq, -peak_i, peak_i);
			break;

		// 开环电压 / 占空比：电流参考置零，由电流环以电压直通模式处理
		case REF_CTRL_VOLTAGE:
		case REF_CTRL_DUTY:
		case REF_CTRL_IDLE:
		default:
			break;
	}
}
