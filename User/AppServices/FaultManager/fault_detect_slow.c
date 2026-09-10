/**
 * @file        fault_detect_slow.c
 * @brief       慢速故障检测(5ms 线程): 温度分级/采样有效性/持续过流/堵转
 *
 * @author      Dalin (dalinyy@163.com)
 * @version     1.1
 * @date        2026-09-03
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @par 修改日志:
 * | 日期       | 版本 | 作者  | 修改内容   |
 * |------------|------|-------|------------|
 * | 2026-09-03 | 1.0  | Dalin | 初始创建   |
 * | 2026-09-04 | 1.1  | Dalin | 检测周期 20ms 改 5ms, 温度类检测编译期门控 |
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 *
 * @details 数据源: usr.motor_state[M1](thermal/power/motion, 由 thread_period
 *          刷新) + dev_power_monitor 原始 ADC(NTC 采样有效性)。
 *          检测顺序: 先判采样有效性, NTC 异常时跳过对应温度判据
 *          (开路钳 -40℃ / 短路钳 230℃ 会误触发低温/过温)。
 *          时间常数类判据(持续过流/堵转)自带拍计数, 拍周期取
 *          thread_config.h 的 THREAD_DELAY_PERIOD(单一数据源)。
 *          检测项启用由 fault_manager.h 的 FAULT_DET_EN_* 编译期开关控制。
 */

#include "fault_manager.h"
#include "system_state.h"
#include "runtime_param.h"
#include "dev_power_monitor.h"
#include "thread_config.h"

/* 慢速检测拍周期(s), 与 thread_period 调度周期一致 */
#define FDET_SLOW_DT_S ((float)THREAD_DELAY_PERIOD * 0.001f)

/* NTC 采样有效窗口(12bit ADC): 开路电压趋 0 / 短路趋 VREF */
#define FDET_NTC_RAW_MIN 30u
#define FDET_NTC_RAW_MAX 4060u
/* NTC 异常确认拍数(100ms) */
#define FDET_NTC_DEBOUNCE_MS 100u
#define FDET_NTC_BAD_CONFIRM ((uint32_t)(FDET_NTC_DEBOUNCE_MS / THREAD_DELAY_PERIOD))

static float fdet_absf(float v)
{
	return (v < 0.0f) ? -v : v;
}

#if defined(USE_DEV_POWER_MONITOR) && FAULT_DET_EN_THERMAL
/* NTC 通道采样有效性检查: raw 越界持续 100ms 判异常
 * 返回: 1=通道健康或未配置(不检测), 0=采样异常(温度判据须跳过) */
static uint8_t fdet_ntc_check(pm_channel_type_e ch, uint16_t code,
                              uint8_t *bad_cycles, float temp)
{
	/* 检查故障是否使能 */
	if (!fault_mgr_is_enabled(code))
	{
		*bad_cycles = 0u;
		fault_mgr_internal_clear(code);
		return 1u;
	}

	if (temp != 0.0f && dev_power_monitor_has_channel(ch))
	{
		uint32_t raw = dev_power_monitor.get_channel_raw(&dev_power_monitor, ch);
		if (raw < FDET_NTC_RAW_MIN || raw > FDET_NTC_RAW_MAX)
		{
			if (*bad_cycles < 0xFFu)
				(*bad_cycles)++;
			if (*bad_cycles >= FDET_NTC_BAD_CONFIRM)
			{
				fault_mgr_internal_set(code, (float)raw);
				return 0u;
			}
			return 1u; /* 消抖期: 温度值暂不可信但未确认, 先跳过判据 */
		}
		*bad_cycles = 0u;
		fault_mgr_internal_clear(code);
	}
	else
	{
		*bad_cycles = 0u;
		fault_mgr_internal_clear(code);
	}
	return 1u;
}
#endif /* USE_DEV_POWER_MONITOR && FAULT_DET_EN_THERMAL */

void fault_detect_slow(fault_mgr_t *fm)
{
	const fault_cfg_t *cfg = &fm->cfg;
	const motor_state_t *st = &usr.motor_state[M1];
	system_state_t *sys = (system_state_t *)fm->sys;

	/* ---- 温度类检测(NTC 有效性 + 分级判据): 编译期门控 ---- */
#if FAULT_DET_EN_THERMAL
	const float temp_fet = st->thermal.temp_fet;
	const float temp_motor = st->thermal.temp_motor;
	uint8_t fet_sample_ok = 1u, motor_sample_ok = 1u;

	/* ---- 0x3205/0x4205 NTC 采样异常(先于温度判据; 占位通道 temp=0 跳过) ---- */
#if defined(USE_DEV_POWER_MONITOR)
	fet_sample_ok = fdet_ntc_check(PM_CH_TEMP_DRIVER, FAULT_FET_TEMP_SAMPLE,
	                               &fm->ntc_fet_bad_cycles, temp_fet);
	motor_sample_ok = fdet_ntc_check(PM_CH_TEMP_MOTOR, FAULT_MOTOR_TEMP_SAMPLE,
	                                 &fm->ntc_motor_bad_cycles, temp_motor);
#endif /* USE_DEV_POWER_MONITOR */

	/* ---- 驱动器温度两级: 0x3303 预警 / 0x3201 过温降功率 ---- */
	if (fet_sample_ok != 0u)
	{
		/* 0x3201 驱动器过温 */
		if (fault_mgr_is_enabled(FAULT_FET_OVER_TEMP))
		{
			if (temp_fet > cfg->temp_fet_over_d)
				fault_mgr_internal_set(FAULT_FET_OVER_TEMP, temp_fet);
			else
				fault_mgr_internal_clear(FAULT_FET_OVER_TEMP);
		}
		else
		{
			fault_mgr_internal_clear(FAULT_FET_OVER_TEMP);
		}

		/* 0x3303 驱动器温度预警 */
		if (fault_mgr_is_enabled(FAULT_FET_TEMP_WARN))
		{
			if (temp_fet > (cfg->temp_fet_over_d - cfg->temp_warn_offset_d))
				fault_mgr_internal_set(FAULT_FET_TEMP_WARN, temp_fet);
			else
				fault_mgr_internal_clear(FAULT_FET_TEMP_WARN);
		}
		else
		{
			fault_mgr_internal_clear(FAULT_FET_TEMP_WARN);
		}

		/* ---- 0x3202 驱动器低温(DENY 禁止运行, 预热后解除) ---- */
		if (fault_mgr_is_enabled(FAULT_FET_UNDER_TEMP))
		{
			if (temp_fet < cfg->temp_under_d)
				fault_mgr_internal_set(FAULT_FET_UNDER_TEMP, temp_fet);
			else
				fault_mgr_internal_clear(FAULT_FET_UNDER_TEMP);
		}
		else
		{
			fault_mgr_internal_clear(FAULT_FET_UNDER_TEMP);
		}
	}

	/* ---- 电机温度三级: 0x4302 预警 / 0x4204 绕组过热降功率 / 0x4101 熔断 ---- */
	if (motor_sample_ok != 0u)
	{
		/* 0x4101 电机熔断级过温 */
		if (fault_mgr_is_enabled(FAULT_MOTOR_OVER_TEMP))
		{
			if (temp_motor > cfg->temp_motor_over_d)
				fault_mgr_internal_set(FAULT_MOTOR_OVER_TEMP, temp_motor);
			else
				fault_mgr_internal_clear(FAULT_MOTOR_OVER_TEMP);
		}
		else
		{
			fault_mgr_internal_clear(FAULT_MOTOR_OVER_TEMP);
		}

		/* 0x4204 绕组过热 */
		if (fault_mgr_is_enabled(FAULT_WINDING_HOT))
		{
			if (temp_motor > cfg->temp_motor_hot_d)
				fault_mgr_internal_set(FAULT_WINDING_HOT, temp_motor);
			else
				fault_mgr_internal_clear(FAULT_WINDING_HOT);
		}
		else
		{
			fault_mgr_internal_clear(FAULT_WINDING_HOT);
		}

		/* 0x4302 电机温度预警 */
		if (fault_mgr_is_enabled(FAULT_MOTOR_TEMP_WARN))
		{
			if (temp_motor > (cfg->temp_motor_hot_d - cfg->temp_warn_offset_d))
				fault_mgr_internal_set(FAULT_MOTOR_TEMP_WARN, temp_motor);
			else
				fault_mgr_internal_clear(FAULT_MOTOR_TEMP_WARN);
		}
		else
		{
			fault_mgr_internal_clear(FAULT_MOTOR_TEMP_WARN);
		}
	}
#endif /* FAULT_DET_EN_THERMAL */

	/* ---- 0x2104 母线过流(故障级瞬时) / 0x2203 持续过流(计时) ---- */
	{
		const float ibus = st->power.i_bus;

		/* 0x2104 母线过流 */
		if (fault_mgr_is_enabled(FAULT_IBUS_OVER))
		{
			if (fdet_absf(ibus) > cfg->ibus_over_A)
				fault_mgr_internal_set(FAULT_IBUS_OVER, ibus);
			else
				fault_mgr_internal_clear(FAULT_IBUS_OVER);
		}
		else
		{
			fault_mgr_internal_clear(FAULT_IBUS_OVER);
		}

		/* 0x2203 持续过流 */
		if (fault_mgr_is_enabled(FAULT_IBUS_CONT))
		{
			if (fdet_absf(ibus) > cfg->ibus_cont_A)
			{
				if (fm->ibus_cont_cycles < 0xFFFFFFFFu)
					fm->ibus_cont_cycles++;
				/* 拍数 = 时长(s) / 拍周期 */
				if (fm->ibus_cont_cycles >= (uint32_t)(cfg->ibus_cont_time_s / FDET_SLOW_DT_S))
					fault_mgr_internal_set(FAULT_IBUS_CONT, ibus);
			}
			else
			{
				fm->ibus_cont_cycles = 0u;
				fault_mgr_internal_clear(FAULT_IBUS_CONT);
			}
		}
		else
		{
			fm->ibus_cont_cycles = 0u;
			fault_mgr_internal_clear(FAULT_IBUS_CONT);
		}
	}

	/* ---- 0x4202 堵转: 速度≈0 且电流超阈值持续(仅 RUN 态) ---- */
	if (sys != NULL && sys->top_state == TOP_FSM_RUN)
	{
		if (fault_mgr_is_enabled(FAULT_MOTOR_STALL))
		{
			if (fdet_absf(st->motion.velocity_rad_s) < cfg->stall_vel && fdet_absf(st->electrical.iq_meas) > cfg->stall_current_A)
			{
				if (fm->stall_cycles < 0xFFFFFFFFu)
					fm->stall_cycles++;
				if (fm->stall_cycles >= (uint32_t)(cfg->stall_time_s / FDET_SLOW_DT_S))
					fault_mgr_internal_set(FAULT_MOTOR_STALL, st->electrical.iq_meas);
			}
			else
			{
				fm->stall_cycles = 0u;
				fault_mgr_internal_clear(FAULT_MOTOR_STALL);
			}
		}
		else
		{
			fm->stall_cycles = 0u;
			fault_mgr_internal_clear(FAULT_MOTOR_STALL);
		}
	}
	else
	{
		fm->stall_cycles = 0u;
		fault_mgr_internal_clear(FAULT_MOTOR_STALL);
	}
}
