/**
 * @file    motor_pid_load.c
 * @brief   PID 三环独立加载模块实现
 * @date    2026-07-08
 */
#include "motor_pid_load.h"
#include "motor_pid_profile.h"
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
		case PID_SOURCE_DEFAULT:
		default:
			break;
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
