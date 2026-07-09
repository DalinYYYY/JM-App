/**
 * @file    motor_pid_load.c
 * @brief   PID 三环独立加载模块实现
 * @date    2026-07-08
 */
#include "motor_pid_load.h"
#include "motor_pid_profile.h"
#include "motor_pid_autotune.h"
#include "runtime_param.h"
#include "motor_info_storage.h"

/* 模块内部状态：三环独立 source 标志，不持久化。
 * 每次上电默认 PID_SOURCE_DEFAULT（用 motor_param.c 默认值）。 */
static pid_source_e s_ring_source[PID_RING_MAX] = {
	PID_SOURCE_DEFAULT,
	PID_SOURCE_DEFAULT,
	PID_SOURCE_DEFAULT,
};

void motor_pid_set_source(pid_ring_e ring, pid_source_e src)
{
	if (ring < PID_RING_MAX)
		s_ring_source[ring] = src;
}

pid_source_e motor_pid_get_source(pid_ring_e ring)
{
	return (ring < PID_RING_MAX) ? s_ring_source[ring] : PID_SOURCE_DEFAULT;
}

void motor_pid_load_source_from_flash(const motor_info_t *info)
{
	if (info == NULL) return;

	uint32_t mask = info->blocks.control.pid_source_mask;
	/* 非 DEFAULT 值表示用户曾显式选择，覆盖 load_boot 的自动回退结果 */
	pid_source_e cur = pid_source_from_mask(mask, PID_RING_CURRENT);
	pid_source_e vel = pid_source_from_mask(mask, PID_RING_VELOCITY);
	pid_source_e pos = pid_source_from_mask(mask, PID_RING_POSITION);
	if (cur != PID_SOURCE_DEFAULT) s_ring_source[PID_RING_CURRENT]  = cur;
	if (vel != PID_SOURCE_DEFAULT) s_ring_source[PID_RING_VELOCITY] = vel;
	if (pos != PID_SOURCE_DEFAULT) s_ring_source[PID_RING_POSITION] = pos;
}

void motor_pid_load(motor_param_t *param, const motor_info_t *info)
{
	if (param == NULL || info == NULL)
		return;

	const ControlParam_t *ctl = &info->blocks.control;

	/* ---- 电流环：按 s_ring_source[PID_RING_CURRENT] 独立选择 ---- */
	switch (s_ring_source[PID_RING_CURRENT])
	{
		case PID_SOURCE_FLASH:
		case PID_SOURCE_AUTOTUNE:
			/* Flash 值由 0xE7 写入；autotune 值由 motor_pid_autotune_apply 写入。
		 * 两者都存于 ctl 同一字段，读取路径一致。 */
			param->current_loop.current_kp_d = ctl->kp_ld;
			param->current_loop.current_ki_d = ctl->ki_ld;
			param->current_loop.current_kp_q = ctl->kp_lq;
			param->current_loop.current_ki_q = ctl->ki_lq;
			param->current_loop.current_integral_limit = ctl->integral_limit;
			break;
		case PID_SOURCE_DEBUG:
			/* 不覆盖 motor_param_t，保留 0xA5 直接写入 s_motor_pid_profiles 的值 */
			break;
		case PID_SOURCE_DEFAULT:
		default:
			/* 保留 motor_param_init/motor_profile 的默认值，不覆盖 */
			break;
	}

	/* ---- 速度环：按 s_ring_source[PID_RING_VELOCITY] 独立选择 ---- */
	switch (s_ring_source[PID_RING_VELOCITY])
	{
		case PID_SOURCE_FLASH:
		case PID_SOURCE_AUTOTUNE:
			param->position_loop.speed_kp = ctl->kp_s;
			param->position_loop.speed_ki = ctl->ki_s;
			param->position_loop.speed_integral_limit = ctl->speed_integral_limit;
			break;
		case PID_SOURCE_DEBUG:
			/* 不覆盖 motor_param_t，保留 0xA5 直接写入 s_motor_pid_profiles 的值 */
			break;
		case PID_SOURCE_DEFAULT:
		default:
			break;
	}

	/* ---- 位置环：按 s_ring_source[PID_RING_POSITION] 独立选择 ---- */
	switch (s_ring_source[PID_RING_POSITION])
	{
		case PID_SOURCE_FLASH:
		case PID_SOURCE_AUTOTUNE:
			param->position_loop.position_kp = ctl->kp_p;
			param->position_loop.position_integral_limit = ctl->position_integral_limit;
			break;
		case PID_SOURCE_DEBUG:
			/* 不覆盖 motor_param_t，保留 0xA5 直接写入 s_motor_pid_profiles 的值 */
			break;
		case PID_SOURCE_DEFAULT:
		default:
			break;
	}
}

/* ==================== Flash ControlParam 有效性检查 ==================== */

static int flash_current_valid(const ControlParam_t *ctl)
{
	return (ctl->kp_ld > 0.0f && ctl->kp_ld < 1000.0f && ctl->ki_ld > 0.0f && ctl->ki_ld < 100000.0f && ctl->kp_lq > 0.0f && ctl->kp_lq < 1000.0f && ctl->ki_lq > 0.0f && ctl->ki_lq < 100000.0f && ctl->integral_limit > 0.0f);
}

static int flash_velocity_valid(const ControlParam_t *ctl)
{
	return (ctl->kp_s > 0.0f && ctl->kp_s < 10000.0f && ctl->ki_s > 0.0f && ctl->ki_s < 100000.0f && ctl->speed_integral_limit > 0.0f);
}

static int flash_position_valid(const ControlParam_t *ctl)
{
	/* 位置环纯比例，ki 可为 0，只检查 kp_p */
	return (ctl->kp_p > 0.0f && ctl->kp_p < 10000.0f);
}

void motor_pid_load_boot(motor_param_t *param, const motor_info_t *info)
{
	if (param == NULL || info == NULL)
		return;

	const ControlParam_t *ctl = &info->blocks.control;

	/* ---- 电流环：Flash → autotune → default ---- */
	if (flash_current_valid(ctl))
	{
		s_ring_source[PID_RING_CURRENT] = PID_SOURCE_FLASH;
		param->current_loop.current_kp_d = ctl->kp_ld;
		param->current_loop.current_ki_d = ctl->ki_ld;
		param->current_loop.current_kp_q = ctl->kp_lq;
		param->current_loop.current_ki_q = ctl->ki_lq;
		param->current_loop.current_integral_limit = ctl->integral_limit;
	}
	else
	{
		autotune_result_t d, q;
		if (motor_pid_autotune_current(info, 0.0f, &d, &q) == 0)
		{
			s_ring_source[PID_RING_CURRENT] = PID_SOURCE_AUTOTUNE;
			param->current_loop.current_kp_d = d.kp;
			param->current_loop.current_ki_d = d.ki;
			param->current_loop.current_kp_q = q.kp;
			param->current_loop.current_ki_q = q.ki;
			param->current_loop.current_integral_limit = q.integral_limit;
		}
		else
		{
			s_ring_source[PID_RING_CURRENT] = PID_SOURCE_DEFAULT;
			/* 保留 motor_param_init 默认值 */
		}
	}

	/* ---- 速度环：Flash → autotune → default ---- */
	if (flash_velocity_valid(ctl))
	{
		s_ring_source[PID_RING_VELOCITY] = PID_SOURCE_FLASH;
		param->position_loop.speed_kp = ctl->kp_s;
		param->position_loop.speed_ki = ctl->ki_s;
		param->position_loop.speed_integral_limit = ctl->speed_integral_limit;
	}
	else
	{
		autotune_result_t v;
		if (motor_pid_autotune_velocity(info, 0.0f, &v) == 0)
		{
			s_ring_source[PID_RING_VELOCITY] = PID_SOURCE_AUTOTUNE;
			param->position_loop.speed_kp = v.kp;
			param->position_loop.speed_ki = v.ki;
			param->position_loop.speed_integral_limit = v.integral_limit;
		}
		else
		{
			s_ring_source[PID_RING_VELOCITY] = PID_SOURCE_DEFAULT;
			/* 保留 motor_param_init 默认值 */
		}
	}

	/* ---- 位置环：Flash → autotune → default ---- */
	if (flash_position_valid(ctl))
	{
		s_ring_source[PID_RING_POSITION] = PID_SOURCE_FLASH;
		param->position_loop.position_kp = ctl->kp_p;
		param->position_loop.position_integral_limit = ctl->position_integral_limit;
	}
	else
	{
		autotune_result_t p;
		if (motor_pid_autotune_position(info, 0.0f, &p) == 0)
		{
			s_ring_source[PID_RING_POSITION] = PID_SOURCE_AUTOTUNE;
			param->position_loop.position_kp = p.kp;
			param->position_loop.position_integral_limit = p.integral_limit;
		}
		else
		{
			s_ring_source[PID_RING_POSITION] = PID_SOURCE_DEFAULT;
			/* 保留 motor_param_init 默认值 */
		}
	}
}

void motor_pid_reload(void)
{
	motor_param_t *param = &usr.motor_param[M1];
	const motor_info_t *info = motor_info_storage_get();

	/* 按 source 独立加载三环 PID 到 motor_param_t */
	motor_pid_load(param, info);

	/* 重新同步到 motor_pid_profile 管理器（motor_pid_profile 读 motor_param_t） */
	motor_pid_profile_load_from_motor_param(param);
}
