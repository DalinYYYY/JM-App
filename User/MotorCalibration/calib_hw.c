/**
 * @file calib_hw.c
 * @brief 标定共享硬件访问层：电压会话 + 编码器读取
 * @note 从 calib_level3_encoder.c 提取，供 L1/L2/L3 共享。
 *       单电机系统约束：s_active 同一时刻只指向一个会话。
 */
#include <math.h>
#include "calib_hw.h"
#include "calib_config.h"
#include "calib_mgr.h" /* calib_mgr_get_io() —— abort 时取 motor 用 */
#include "dev_motor.h"
#include "motor_loop_config.h"
#if MOTOR_LOOP_ENABLE_DEV_DRIVER
#include "dev_power_monitor.h" /* 真实电机：SVPWM 归一化用 Vbus */
#endif

/* ===================== 模块私有：当前活动会话（单例）===================== */
static calib_hw_session_t *s_active = NULL;

/* ===================== 标定期间电角度回调（返回强制值）===================== */
static float calib_hw_ele_radian_cb(void)
{
	return s_active ? s_active->forced_ele_angle : 0.0f;
}

/* ===================== 接口实现 ===================== */
void calib_hw_enter(calib_hw_session_t *s, struct dev_motor *m)
{
	s->motor = m;
	s->orig_ele_cb = m->ele_radian_callback;
	s->forced_ele_angle = 0.0f;
	s_active = s;
	m->ele_radian_callback = calib_hw_ele_radian_cb;
	m->foc.ele_radian_callback = calib_hw_ele_radian_cb;
}

void calib_hw_exit(calib_hw_session_t *s)
{
	if (s->motor == NULL)
		return;
	calib_hw_apply_zero(s->motor);
	if (s->orig_ele_cb != NULL)
	{
		s->motor->ele_radian_callback = s->orig_ele_cb;
		s->motor->foc.ele_radian_callback = s->orig_ele_cb;
	}
	s_active = NULL;
}

void calib_hw_apply_voltage(calib_hw_session_t *s, float ud, float uq, float theta)
{
	struct dev_motor *m = s->motor;

	/* 安全互锁：电压幅值上限保护，防止 level 模块 bug 烧管子 */
	float mag = sqrtf(ud * ud + uq * uq);
	if (mag > CALIB_CFG_MAX_VOLTAGE_MAG_V)
	{
		float scale = CALIB_CFG_MAX_VOLTAGE_MAG_V / mag;
		ud *= scale;
		uq *= scale;
	}

#if MOTOR_LOOP_ENABLE_DEV_DRIVER
	/* 真实电机 SVPWM 归一化：foc_core.c 的 SVPWM Ts=1.0（归一化周期），
	 * 输入 u_alpha/u_beta 须为占空比（0~1）而非电压值（伏特）。
	 * ud/uq 是真实电压，须除以 Vbus 转换为占空比，否则电压值（如 1.865V）
	 * 会被当作占空比（>>1.0）触发过调制限幅，实际电压幅值失真且随角度
	 * 非线性波动，导致开环标定（极对数/R/Ld/Lq/flux）结果错误。
	 * 虚拟电机直接用 ud/uq 推进物理模型（不走 SVPWM），不归一化。*/
	// float vbus = dev_power_monitor.vbus;
	float vbus = 12.0f;
	if (vbus < 1.0f)
		vbus = 1.0f; /* 保护：Vbus 未就绪时避免除零，标称 Vbus >= 12V */
	ud /= vbus;
	uq /= vbus;
#endif

	s->forced_ele_angle = theta;
	m->foc.set_udq(&m->foc, ud, uq);
	m->foc.inverse_park(&m->foc);
	m->foc.pfsvpwm(&m->foc);
	m->half_bridge.set_3pwm(&m->half_bridge,
	                        (uint32_t)(PWM_PERIOD * m->foc.svpwm.ta),
	                        (uint32_t)(PWM_PERIOD * m->foc.svpwm.tb),
	                        (uint32_t)(PWM_PERIOD * m->foc.svpwm.tc));
}

void calib_hw_apply_zero(struct dev_motor *m)
{
	m->half_bridge.set_3pwm(&m->half_bridge, 0, 0, 0);
}

float calib_hw_get_encoder_raw_deg(struct dev_motor *m)
{
	/* 通过抽象编码器层访问，不直接依赖 MT6701/MT6835 具体芯片 */
	return m->encoder.get_raw_deg(&m->encoder);
}

float calib_hw_get_encoder_mech_angle(struct dev_motor *m)
{
	m->encoder.update(&m->encoder);
	return m->encoder.mechanical_angle;
}
