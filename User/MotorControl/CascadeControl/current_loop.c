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
	cl->motor->phase_current.update(&cl->motor->phase_current);
}

void cur_loop_run(cur_loop_t *cl, float id_ref, float iq_ref)
{
	dev_motor_t *m = cl->motor;

	// step1: 刷新编码器机械角度（抽象接口，与具体芯片型号无关）
	m->encoder.update(&m->encoder);
	// m->motor_param.update(&m->motor_param, MOTION_TYPE_ELE_RADIAN, m->encoder.mechanical_angle);

	// step2: 三相电流采样
	m->phase_current.update(&m->phase_current);

	// step3: Clarke 变换（三相 → αβ）
	m->foc.clarke(&m->foc);

	// step4: Park 变换（αβ → dq）
	m->foc.park(&m->foc);

	// step5: dq 轴电流环 PI（pid_profile 统一封装）
	float ud = pid_profile_calculate(&cl->pid_id, PID_PROFILE_CURRENT_D, id_ref, m->foc.i_dq.d, cl->dt);
	float uq = pid_profile_calculate(&cl->pid_iq, PID_PROFILE_CURRENT_Q, iq_ref, m->foc.i_dq.q, cl->dt);

	// step6: 设置 dq 电压并反 Park（dq → αβ）
	m->foc.set_udq(&m->foc, ud, uq);
	m->foc.inverse_park(&m->foc);

	// step7: SVPWM
	m->foc.pfsvpwm(&m->foc);

	// step8: PWM 输出
	// 真实模式：驱动半桥定时器；虚拟模式：用本拍 u_dq 推进物理模型一步（一拍延迟）
	m->half_bridge.set_3pwm(&m->half_bridge,
							(uint32_t)(PWM_PERIOD * m->foc.svpwm.ta),
							(uint32_t)(PWM_PERIOD * m->foc.svpwm.tb),
							(uint32_t)(PWM_PERIOD * m->foc.svpwm.tc));
}
