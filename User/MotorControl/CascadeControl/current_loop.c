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
 *              电流环 PI 使用 motor_pid_profile（MOTOR_PID_PROFILE_CURRENT_D / _Q）。
 *              底层链路（Clarke/Park/反Park/SVPWM/PWM）复用 dev_motor 的 foc 与
 *              half_bridge 接口，调用方式参考 foc_current_control.c。
 */

#include "current_loop.h"
#include "motor_pid_profile.h"
#include "foc_core.h"
#include "motor_loop_config.h"
#include <string.h>
#if MOTOR_LOOP_ENABLE_DEV_DRIVER
#include "dev_power_monitor.h" /* 真实电机：SVPWM 归一化用 Vbus */
#endif

/* 判断浮点是否为有限值(排除 NaN/Inf)，异常时走安全输出路径 */
static int cur_float_is_finite(float value)
{
	uint32_t bits;
	memcpy(&bits, &value, sizeof(bits));
	return (bits & 0x7F800000u) != 0x7F800000u;
}

/* 占空比[0,1] → TIM 比较寄存器值，越界/非法输入钳位到安全值 */
static uint32_t cur_pwm_compare(float duty)
{
	if (!cur_float_is_finite(duty))
		return 0u;
	if (duty <= 0.0f)
		return 0u;
	if (duty >= 1.0f)
		return PWM_PERIOD;
	return (uint32_t)(PWM_PERIOD * duty);
}

void cur_loop_init(cur_loop_t *cl, dev_motor_t *motor, motor_param_t *param, float dt)
{
	cl->motor = motor;
	cl->param = param;
	cl->dt = dt;

	motor_pid_profile_init_state(&cl->pid_id);
	motor_pid_profile_init_state(&cl->pid_iq);

	cur_loop_set_decoupling_config(cl); /* 从 param 构建初始解耦配置 */
}

void cur_loop_set_decoupling_config(cur_loop_t *cl)
{
	if (cl == NULL || cl->param == NULL)
		return;
	/* 将 motor_param 的 3 个字段映射为 foc_decoupling_config_t */
	cl->decoupling.algo = (foc_decouple_algo_e)cl->param->current_loop.decouple_algo;
	cl->decoupling.bemf_ff_enable = (cl->param->current_loop.bemf_ff_enable != 0);
	cl->decoupling.deadtime_comp_enable = (cl->param->current_loop.deadtime_comp_enable != 0);
}

void cur_loop_reset(cur_loop_t *cl)
{
	motor_pid_profile_reset_state(&cl->pid_id);
	motor_pid_profile_reset_state(&cl->pid_iq);
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
		if (!cur_float_is_finite(duty))
		{
			m->half_bridge.set_3pwm(&m->half_bridge, 0, 0, 0);
			return;
		}
		if (duty < -1.0f) duty = -1.0f;
		if (duty > 1.0f) duty = 1.0f;
		uint32_t ccr = cur_pwm_compare(0.5f + 0.5f * duty);
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
		/* 开环电压：旁路 PI，直接用 ref->ud / ref->voltage 作为 dq 轴电压 */
		ud = ref->ud;
		uq = ref->voltage;
#if MOTOR_LOOP_ENABLE_DEV_DRIVER
		/* 真实电机 SVPWM 归一化：foc_core.c 的 SVPWM Ts=1.0（归一化周期），
		 * 输入 u_alpha/u_beta 须为占空比（0~1）而非电压值（伏特）。
		 * ud/uq 是真实电压，须除以 Vbus 转换为占空比，否则电压值（如 1.0V）
		 * 会被当作占空比触发过调制限幅，实际电压幅值失真且随角度非线性波动。
		 * 虚拟电机直接用 ud/uq 推进物理模型（不走 SVPWM），不归一化。
		 * Vbus 由 dev_power_monitor 在 task 层 100ms 周期更新。 */
		float vbus = dev_power_monitor.vbus;
		if (vbus < 1.0f)
			vbus = 1.0f; /* 保护：Vbus 未就绪时避免除零，标称 Vbus >= 12V */
		ud /= vbus;
		uq /= vbus;
#endif
	}
	else
	{
		/* 电流闭环 PI（CURRENT / TORQUE / VELOCITY / POSITION）*/
		/* 每拍从 param 同步解耦配置，确保 0xE1/0xE7 修改立即生效（3 字段赋值，开销可忽略）*/
		cur_loop_set_decoupling_config(cl);

		ud = motor_pid_profile_calculate(&cl->pid_id, MOTOR_PID_PROFILE_CURRENT_D, out->id_ref, m->foc.i_dq.d, cl->dt);
		uq = motor_pid_profile_calculate(&cl->pid_iq, MOTOR_PID_PROFILE_CURRENT_Q, out->iq_ref, m->foc.i_dq.q, cl->dt);

		/* 提前取 Vbus：解耦死区补偿自动计算 + SVPWM 归一化共用，避免重复读取 */
#if MOTOR_LOOP_ENABLE_DEV_DRIVER
		float vbus = dev_power_monitor.vbus;
		if (vbus < 1.0f)
			vbus = 1.0f; /* 保护：Vbus 未就绪时避免除零 */
#else
		float vbus = 12.0f; /* 虚拟电机标称电压 */
#endif

		/* === DQ 解耦补偿（V 域，PI 输出之后、Vbus 归一化之前）=== */
		foc_decoupling_in_t din = {
			.ud_pi = ud,
			.uq_pi = uq,
			.id = m->foc.i_dq.d,
			.iq = m->foc.i_dq.q,
			.id_ref = out->id_ref,
			.iq_ref = out->iq_ref,
			.omega_mech = m->motor_param.slide_rad_s, /* PLL 滤波后机械角速度 */
			.vbus = vbus, /* 死区补偿自动计算用 */
		};
		foc_decoupling_out_t dout;
		foc_decoupling_run(&din, &dout, &cl->decoupling, cl->param);
		ud = dout.ud;
		uq = dout.uq;
#if MOTOR_LOOP_ENABLE_DEV_DRIVER
		/* 真实电机 SVPWM 归一化：PI 输出为电压值（伏特），SVPWM 期望占空比（0~1）， 
		 * 须除以 Vbus 转换。与 OPEN_LOOP 分支、calib_hw.c 保持一致。
		 * Vbus 由 dev_power_monitor 在 task 层 100ms 周期更新。 */
		ud /= vbus;
		uq /= vbus;
#endif
	}

	// step6: 设置 dq 电压并反 Park（dq → αβ）
	if (!cur_float_is_finite(ud) || !cur_float_is_finite(uq))
	{
		m->half_bridge.set_3pwm(&m->half_bridge, 0, 0, 0);
		return;
	}
	m->foc.set_udq(&m->foc, ud, uq);
	m->foc.inverse_park(&m->foc);

	// step7: SVPWM
	m->foc.pfsvpwm(&m->foc);

	// step8: PWM 输出
	if (!cur_float_is_finite(m->foc.svpwm.ta) ||
		!cur_float_is_finite(m->foc.svpwm.tb) ||
		!cur_float_is_finite(m->foc.svpwm.tc))
	{
		m->half_bridge.set_3pwm(&m->half_bridge, 0, 0, 0);
		return;
	}
	m->half_bridge.set_3pwm(&m->half_bridge,
		cur_pwm_compare(m->foc.svpwm.ta),
		cur_pwm_compare(m->foc.svpwm.tb),
		cur_pwm_compare(m->foc.svpwm.tc));
}
