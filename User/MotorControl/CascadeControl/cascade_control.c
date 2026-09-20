/**
 * @file        cascade_control.c
 * @brief       三环级联控制算法层实现（PID 经 motor_pid_profile 统一封装）
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
#include "motor_loop_config.h" /* MOTOR_LOOP_POS_DEADBAND_EN: 位置误差死区开关 */
#include "friction_comp.h"     /* friction_comp_current: 摩擦前馈（模块内可裁剪） */
#include "cogging_comp.h"      /* cogging_comp_current: 齿槽前馈（模块内可裁剪） */
#include "fault_manager.h"     /* fault_mgr_get_derate/clamp_pos: 降功率与软限位钳制 */
#include <string.h>

static float clamp(float value, float min, float max)
{
	if (value < min)
		return min;
	if (value > max)
		return max;
	return value;
}

/** @brief 快速绝对值（避免引入 math.h） */
static float abs_f(float x)
{
	return (x >= 0.0f) ? x : -x;
}

void cascade_control_init(cascade_ctrl_t *c, motor_param_t *param, float dt_pos, float dt_vel)
{
	memset(c, 0, sizeof(cascade_ctrl_t));

	c->param = param;
	c->dt_pos = dt_pos;
	c->dt_vel = dt_vel;
	c->last_ctrl_type = REF_CTRL_IDLE;
	c->last_pos_profile = MOTOR_PID_PROFILE_POSITION;
	c->last_vel_profile = MOTOR_PID_PROFILE_VELOCITY;

	motor_pid_profile_init_state(&c->pid_pos);
	motor_pid_profile_init_state(&c->pid_vel);
}

void cascade_control_reset(cascade_ctrl_t *c)
{
	motor_pid_profile_reset_state(&c->pid_pos);
	motor_pid_profile_reset_state(&c->pid_vel);
	c->vel_setpoint = 0.0f;
	c->pos_in_deadband = 0u;
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
			motor_pid_profile_preload(&c->pid_pos, ref->pos_profile, fb->vel, fb->pos);
			motor_pid_profile_preload(&c->pid_vel, ref->vel_profile, fb->iq, fb->vel);
			c->vel_setpoint = fb->vel;
			break;

		case REF_CTRL_VELOCITY: motor_pid_profile_preload(&c->pid_vel, ref->vel_profile, fb->iq, fb->vel); break;

		default:
			// TORQUE/CURRENT/IDLE 无外环积分，无需预装载
			break;
	}
}

void cascade_control_run_position(cascade_ctrl_t *c, const motor_ref_t *ref, const cascade_fb_t *fb)
{
	if (ref->ctrl_type != REF_CTRL_POSITION)
		return;

	/* 软限位钳制(0x1201): 位置参考超界截断到边界, 允许反向运动 */
	float pos_ref = ref->pos;
	fault_mgr_clamp_pos(&pos_ref);

#if (MOTOR_LOOP_POS_DEADBAND_EN)
	/* 位置误差死区（带迟滞）: 带内不推（vel_sp=0）, 速度环以 0 为目标主动
	 * 刹停, 并按位置环节拍泄漏速度环积分, 切断"摩擦死区+积分蓄能"stick-slip
	 * 极限环的能量来源。位置 profile 为纯 P（ki=0）, 跳过计算无状态残留。
	 * 迟滞退出: 无迟滞时噪声/齿槽使误差在边界来回穿越, 每次出界触发一次
	 * 全增益位置环打击, 形成边界极限环（周期性"嗒"声）。 */
	float abs_err = abs_f(pos_ref - fb->pos);
	if (c->pos_in_deadband)
	{
		if (abs_err < MOTOR_LOOP_POS_DEADBAND_HYS_RAD)
		{
			c->vel_setpoint = 0.0f;
			if (MOTOR_LOOP_VEL_INT_LEAK_TAU > 0.0f)
				c->pid_vel.integral *= 1.0f - c->dt_pos / MOTOR_LOOP_VEL_INT_LEAK_TAU;
			return;
		}
		c->pos_in_deadband = 0u; /* 超出迟滞阈值, 恢复位置环出力 */
	}
	else if (abs_err < MOTOR_LOOP_POS_DEADBAND_RAD)
	{
		c->pos_in_deadband = 1u;
		c->vel_setpoint = 0.0f;
		if (MOTOR_LOOP_VEL_INT_LEAK_TAU > 0.0f)
			c->pid_vel.integral *= 1.0f - c->dt_pos / MOTOR_LOOP_VEL_INT_LEAK_TAU;
		return;
	}
#endif

	// 位置环：位置误差 → 速度设定（motor_pid_profile 内部已按 max_speed 限幅）
	float vel_sp = motor_pid_profile_calculate(&c->pid_pos, ref->pos_profile, pos_ref, fb->pos, c->dt_pos);

	// 叠加速度前馈
	vel_sp += ref->vel_ff * (c->param)->position_loop.velocity_ff_gain;

	// 速度限幅
	c->vel_setpoint = clamp(vel_sp, -(c->param)->motor_base.max_speed, (c->param)->motor_base.max_speed);
}

void cascade_control_run(cascade_ctrl_t *c, const motor_ref_t *ref, const cascade_fb_t *fb, cascade_out_t *out)
{
	motor_param_t *p = c->param;
	float kt = (p)->motor_base.kt;
	/* 峰值电流限幅 × 降功率系数(异常级故障降功率运行, 正常时 1.0) */
	float peak_i = (p)->motor_base.peak_current * fault_mgr_get_derate();

	// 入环层级或配置文件变化：先做无扰预装载（仅闭环模式需要）
	if ((ref->ctrl_type != c->last_ctrl_type || ref->pos_profile != c->last_pos_profile || ref->vel_profile != c->last_vel_profile) && ref->ctrl_type >= REF_CTRL_CURRENT)
	{
		cascade_bumpless_preload(c, ref, fb);
	}
	c->last_ctrl_type = ref->ctrl_type;
	c->last_pos_profile = ref->pos_profile;
	c->last_vel_profile = ref->vel_profile;

	out->id_ref = 0.0f;
	out->iq_ref = 0.0f;

	switch (ref->ctrl_type)
	{
		// 位置模式：使用位置环输出的速度设定跑速度环（叠加摩擦/齿槽前馈）
		case REF_CTRL_POSITION:
		{
			float iq_ff = friction_comp_current(kt, (p)->position_loop.friction_coulomb, (p)->position_loop.friction_viscous, fb->vel);
			iq_ff += cogging_comp_current(fb->mech_single_rad, (p)->position_loop.cogging_comp_enable, (p)->position_loop.cogging_comp_gain);
			float iq = motor_pid_profile_calculate_with_ff(&c->pid_vel, ref->vel_profile, c->vel_setpoint, fb->vel, iq_ff, c->dt_vel);
			out->iq_ref = clamp(iq, -peak_i, peak_i);
			break;
		}

		// 速度模式：速度环 + 前馈（前馈以电流形式叠加）
		case REF_CTRL_VELOCITY:
		{
			/* 力矩前馈(N·m→A) + 惯量加速度前馈(accel_ff_gain=J/Kt)。
			 * accel 为解析参考加速度(PV斜坡/过渡导数/扫频)，无差分噪声；
			 * 前馈在环外叠加，不改变环路特征方程，经 calculate_with_ff
			 * 统一限幅+抗饱和。
			 * 惯量前馈经 MOTOR_LOOP_VEL_ACCEL_FF_ENABLE 门控: 空载速度闭环
			 * 失稳问题排查期间默认关闭, 见 motor_loop_config.h。 */
			float iq_ff = 0.0f;
			if (kt > 0.0f)
				iq_ff += ref->torque_ff / kt;
#if (MOTOR_LOOP_VEL_ACCEL_FF_ENABLE)
			iq_ff += p->position_loop.accel_ff_gain * ref->accel;
#endif
			iq_ff += friction_comp_current(kt, (p)->position_loop.friction_coulomb, (p)->position_loop.friction_viscous, fb->vel);
			iq_ff += cogging_comp_current(fb->mech_single_rad, (p)->position_loop.cogging_comp_enable, (p)->position_loop.cogging_comp_gain);
			float iq = motor_pid_profile_calculate_with_ff(&c->pid_vel, ref->vel_profile, ref->vel, fb->vel, iq_ff, c->dt_vel);
			out->iq_ref = clamp(iq, -peak_i, peak_i);
			break;
		}

		// 力矩模式：力矩 ÷ kt 直接得电流参考，跳过外环
		case REF_CTRL_TORQUE: out->iq_ref = (kt > 0.0f) ? clamp(ref->torque / kt, -peak_i, peak_i) : 0.0f; break;

		// 电流模式：直接使用 dq 电流参考，跳过全部外环
		case REF_CTRL_CURRENT:
			out->id_ref = clamp(ref->id, -peak_i, peak_i);
			out->iq_ref = clamp(ref->iq, -peak_i, peak_i);
			break;

		// 开环电压 / 占空比：电流参考置零，由电流环以电压直通模式处理
		case REF_CTRL_VOLTAGE:
		case REF_CTRL_DUTY:
		case REF_CTRL_IDLE:
		default: break;
	}
}
