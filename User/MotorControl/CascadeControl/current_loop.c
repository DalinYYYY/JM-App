/**
 * @file        current_loop.c
 * @brief       FOC 电流环控制模块实现
 *
 * @author      yangsl
 * @version     1.0
 * @date        2026-06-11
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 *              电流环 PI 使用 pid_profile（PID_PROFILE_CURRENT_D / _Q）。
 *              底层链路（Clarke/Park/反Park/SVPWM/PWM）复用 dev_motor 的 foc 与
 *              half_bridge 接口，调用方式参考 foc_current_control.c。
 */

#include "current_loop.h"
#include "pid_profile.h"
#include "foc_core.h"

void cur_loop_init(cur_loop_t *cl, dev_motor_t *motor, float dt)
{
	cl->motor = motor;
	cl->dt = dt;

	pid_profile_init_state(&cl->pid_id);
	pid_profile_init_state(&cl->pid_iq);
}

void cur_loop_reset(cur_loop_t *cl)
{
	pid_profile_reset_state(&cl->pid_id);
	pid_profile_reset_state(&cl->pid_iq);
}

void cur_loop_calibrate_offset(cur_loop_t *cl)
{
	/* 电机不通电、PWM 未启动时调用: 多次采样取均值标定 INA199B1 REF
	 * (约1.65V) 对应的 ADC 零位, 消除分压网络/运放 REF 容差导致的零点漂移。
	 * 须在 phase_current.start 之后调用(注入组已运行才有有效ADC值)。 */
	cl->motor->phase_current.calibrate_offset(&cl->motor->phase_current, 256);
}

void cur_loop_run(cur_loop_t *cl, const motor_ref_t *ref, const cascade_out_t *out)
{
	dev_motor_t *m = cl->motor;

	/* ===== 直通模式：不经过 FOC 电流环 PI ===== */
	/* 注：编码器与角度解算已在 motor_loop_isr 开头统一刷新，所有模式均可获取角度 */

	/* 占空比直控：跳过 FOC 全链路，直接驱动 PWM */
	if (ref->ctrl_type == REF_CTRL_DUTY)
	{
		float duty = ref->duty;
		uint32_t ccr = (uint32_t)(PWM_PERIOD * (0.5f + 0.5f * duty));
		m->half_bridge.set_3pwm(&m->half_bridge, ccr, ccr, ccr);
		return;
	}

	/* IDLE：PWM 置零（安全失能输出）*/
	if (ref->ctrl_type == REF_CTRL_IDLE)
	{
		m->half_bridge.set_3pwm(&m->half_bridge, 0, 0, 0);
		return;
	}

	/* ===== 以下模式需要 FOC 链路 ===== */
	/* 编码器与电角度已在 ISR 开头刷新，此处直接采样电流并做 Clarke/Park */

	// step1: 三相电流采样
	m->phase_current.update(&m->phase_current);

	// step2: Clarke 变换（三相 → αβ）
	m->foc.clarke(&m->foc);

	// step3: Park 变换（αβ → dq）
	m->foc.park(&m->foc);

	float ud, uq;

	if (ref->ctrl_type == REF_CTRL_VOLTAGE)
	{
		/* 开环电压：旁路 PI，直接用 ref->voltage 作为 q 轴电压 */
		ud = 0.0f;
		uq = ref->voltage;
	}
	else
	{
		/* 电流闭环 PI（CURRENT / TORQUE / VELOCITY / POSITION）*/
		ud = pid_profile_calculate(&cl->pid_id, PID_PROFILE_CURRENT_D,
								   out->id_ref, m->foc.i_dq.d, cl->dt);
		uq = pid_profile_calculate(&cl->pid_iq, PID_PROFILE_CURRENT_Q,
								   out->iq_ref, m->foc.i_dq.q, cl->dt);
	}

	// step6: 设置 dq 电压并反 Park（dq → αβ）
	m->foc.set_udq(&m->foc, ud, uq);
	m->foc.inverse_park(&m->foc);

	// step7: SVPWM
	m->foc.pfsvpwm(&m->foc);

	// step8: PWM 输出
	m->half_bridge.set_3pwm(&m->half_bridge,
							(uint32_t)(PWM_PERIOD * m->foc.svpwm.ta),
							(uint32_t)(PWM_PERIOD * m->foc.svpwm.tb),
							(uint32_t)(PWM_PERIOD * m->foc.svpwm.tc));
}
