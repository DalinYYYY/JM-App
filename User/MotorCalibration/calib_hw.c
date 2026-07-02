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
#include "dev_mt6701.h"

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
	dev_mt6701_t *enc = &m->mt6701;
	enc->update(enc);
	return (float)enc->raw / MT6701_ANGLE_RESOLUTION * 360.0F;
}

float calib_hw_get_encoder_mech_angle(struct dev_motor *m)
{
	m->encoder.update(&m->encoder);
	return m->encoder.mechanical_angle;
}
