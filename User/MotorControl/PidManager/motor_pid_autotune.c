/**
 * @file    motor_pid_autotune.c
 * @brief   PID 理论估计模块实现（零极点对消法）
 * @date    2026-07-08
 */
#include "motor_pid_autotune.h"
#include <stddef.h>
#include <stdbool.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* 推荐带宽默认值 (Hz) */
#define AUTOTUNE_DEFAULT_CURRENT_BW  500.0f
#define AUTOTUNE_DEFAULT_VELOCITY_BW 100.0f
#define AUTOTUNE_DEFAULT_POSITION_BW 20.0f

/**
 * @brief 检查辨识数据是否就绪
 * @return true=就绪, false=未就绪
 * @note 不依赖 is_calibrated 整体标志, 与 motor_profile_apply_info 的
 *       逐字段零值 fallback 设计一致: 启动时未标定字段已被默认值填充,
 *       此处只校验字段有效性(R/Ld 非零)。部分标定或纯默认值均可工作。
 */
static int check_calib_ready(const motor_info_t *info)
{
	if (info == NULL)
		return 0;
	/* R/Ld 有效性检查（启动时 motor_profile_apply_info 已用默认值填充零值字段） */
	float r = info->blocks.motor_calib.phase_resistance;
	float ld = info->blocks.motor_calib.phase_inductance_d;
	if (r < 0.001f || ld < 0.00001f)
		return 0;
	return 1;
}

int motor_pid_autotune_current(const motor_info_t *info, float bandwidth_hz,
                               autotune_result_t *out_d, autotune_result_t *out_q)
{
	if (info == NULL || out_d == NULL || out_q == NULL)
		return -1;

	if (!check_calib_ready(info))
		return -2;

	/* 带宽默认值 fallback：优先使用 motor_info 中的电机配置值。 */
	if (bandwidth_hz <= 0.0f)
	{
		float configured_bw = info->blocks.motor_calib.current_control_bandwidth;
		if (isfinite(configured_bw) && configured_bw > 0.0f)
			bandwidth_hz = configured_bw;
		else
			bandwidth_hz = AUTOTUNE_DEFAULT_CURRENT_BW;
	}

	float r = info->blocks.motor_calib.phase_resistance;
	float ld = info->blocks.motor_calib.phase_inductance_d;
	float lq = info->blocks.motor_calib.phase_inductance_q;
	float wc = 2.0f * M_PI * bandwidth_hz; /* 截止角频率 rad/s */

	/* 电流环零极点对消: Kp = ωc·L, Ki = ωc·R */
	out_d->kp = wc * ld;
	out_d->ki = wc * r;
	out_q->kp = wc * lq;
	out_q->ki = wc * r;

	/* 积分限幅 = Kp_q × 额定电流 × 1.5
	 * 额定电流取 current_lim × 0.6（持续电流约 60% 限幅） */
	float rated = info->blocks.motor_calib.current_lim * 0.6f;
	out_q->integral_limit = out_q->kp * rated * 1.5f;
	out_d->integral_limit = out_q->integral_limit;

	return 0;
}

int motor_pid_autotune_velocity(const motor_info_t *info, float bandwidth_hz,
                                autotune_result_t *out)
{
	if (info == NULL || out == NULL)
		return -1;

	if (!check_calib_ready(info))
		return -2;

	/* 检查 J/Kt 有效性 */
	float j = info->blocks.motor_calib.rotor_inertia;
	float kt = info->blocks.motor_calib.torque_constant;
	if (j < 1e-7f || kt < 0.01f)
		return -2;

	/* 带宽默认值 fallback */
	if (bandwidth_hz <= 0.0f)
		bandwidth_hz = AUTOTUNE_DEFAULT_VELOCITY_BW;

	float wc = 2.0f * M_PI * bandwidth_hz;

	/* 速度环二阶最佳阻尼 (ξ=0.707):
	 * Kp = J·ωc / Kt,  Ki = J·ωc² / (4·Kt)
	 * 传递函数分母 = s² + 2ξωc·s + ωc²，令 ξ=1/√2 得最佳阻尼 */
	out->kp = j * wc / kt;
	out->ki = j * wc * wc / (4.0f * kt);

	/* 积分限幅 = 额定电流 × 0.5 */
	out->integral_limit = info->blocks.motor_calib.current_lim * 0.5f;

	return 0;
}

int motor_pid_autotune_position(const motor_info_t *info, float bandwidth_hz,
                                autotune_result_t *out)
{
	if (info == NULL || out == NULL)
		return -1;

	/* 位置环不依赖辨识数据，纯比例控制 */

	/* 带宽默认值 fallback */
	if (bandwidth_hz <= 0.0f)
		bandwidth_hz = AUTOTUNE_DEFAULT_POSITION_BW;

	/* 位置环纯比例: Kp = 2π·f (带宽 = Kp) */
	out->kp = 2.0f * M_PI * bandwidth_hz;
	out->ki = 0.0f;
	out->integral_limit = 0.0f;

	return 0;
}

int motor_pid_autotune_apply(motor_info_t *info, uint8_t ring_mask,
                             float current_bw_hz, float velocity_bw_hz, float position_bw_hz)
{
	if (info == NULL)
		return -1;
	/* ring_mask 位掩码: bit0=电流 bit1=速度 bit2=位置, 0=空选无效, >0x07=越界 */
	if (ring_mask == 0 || ring_mask > 0x07)
		return -1;

	autotune_result_t d, q, v, p;
	bool do_cur = (ring_mask & 0x01) != 0;
	bool do_vel = (ring_mask & 0x02) != 0;
	bool do_pos = (ring_mask & 0x04) != 0;

	/* 事务语义：先计算所选环，任一失败则不写入 */
	if (do_cur)
	{
		int ret = motor_pid_autotune_current(info, current_bw_hz, &d, &q);
		if (ret != 0)
			return ret;
	}
	if (do_vel)
	{
		int ret = motor_pid_autotune_velocity(info, velocity_bw_hz, &v);
		if (ret != 0)
			return ret;
	}
	if (do_pos)
	{
		int ret = motor_pid_autotune_position(info, position_bw_hz, &p);
		if (ret != 0)
			return ret;
	}

	/* 全部所选环成功，事务提交：写入 ControlParam_t（未选环保留原值）*/
	ControlParam_t *ctl = &info->blocks.control;
	if (do_cur)
	{
		ctl->kp_ld = d.kp;
		ctl->ki_ld = d.ki;
		ctl->kp_lq = q.kp;
		ctl->ki_lq = q.ki;
		ctl->integral_limit = q.integral_limit;
	}
	if (do_vel)
	{
		ctl->kp_s = v.kp;
		ctl->ki_s = v.ki;
		ctl->speed_integral_limit = v.integral_limit;
	}
	if (do_pos)
	{
		ctl->kp_p = p.kp;
	}

	return 0;
}
