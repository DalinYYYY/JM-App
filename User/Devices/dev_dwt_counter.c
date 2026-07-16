/**
 * @file        dev_dwt_counter.c
 * @brief       基于内核DWT周期计数器的耗时打点与阻塞延时
 *
 * @author      name (name@robot.com)
 * @version     1.1
 * @date        2026-06-9
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @par 修改日志:
 * | 日期       | 版本 | 作者   | 修改内容                                   |
 * |------------|------|--------|--------------------------------------------|
 * | 2026-06-10 | 1.0  | yangsl | 初始创建                                   |
 * | 2026-06-11 | 1.1  | yangsl | 修复延时溢出/下溢，增加索引边界检查         |
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 */

#include "dev_dwt_counter.h"

/* 跨板兼容: 包含 HAL 头文件以访问 RCC 寄存器和 HAL_RCC_GetSysClockFreq */
#if defined(STM32F405xx)
#include "stm32f4xx_hal.h"
#elif defined(STM32G474xx) || defined(STM32G473xx)
#include "stm32g4xx_hal.h"
#endif

dwtTimer_t dwt_timer;

/**
 * @brief  初始化DWT计数器
 */
void dev_dwt_counter_init(void)
{
	drv_dwt_timer_init();

	dwt_timer.sys_freq_hz = HAL_RCC_GetSysClockFreq();
	dwt_timer.ticks_to_us = 1000000.0f / (float)dwt_timer.sys_freq_hz;
}

void dev_dwt_counter_start(uint8_t index)
{
	if (index >= SYS_TIMER_RECORD_MAX_INDEX)
		return;

	dwt_timer.now_records[index] = drv_dwt_timer_get_ticks();
}

void dev_dwt_counter_stop(uint8_t index)
{
	if (index >= SYS_TIMER_RECORD_MAX_INDEX)
		return;

	/* 无符号回环减法天然正确处理CYCCNT溢出 */
	dwt_timer.duration_records[index] = drv_dwt_timer_get_ticks() - dwt_timer.now_records[index];
	dwt_timer.duration_us[index] = dwt_timer.ticks_to_us * (float)dwt_timer.duration_records[index];
}

/**
 * @brief  获取DWT计数器持续时间
 * @param  index 计数器索引 (范围: 0 ~ SYS_TIMER_RECORD_MAX_INDEX-1)
 * @return 持续时间 (单位: 微秒)，索引越界返回0
 */
float dev_dwt_counter_get_duration_us(uint8_t index)
{
	if (index >= SYS_TIMER_RECORD_MAX_INDEX)
		return 0.0f;

	return dwt_timer.ticks_to_us * (float)dwt_timer.duration_records[index];
}

/**
 * @brief  微秒级阻塞延时
 * @param  us 延时时长 (单位: 微秒)
 * @note   用64位计算目标周期数，避免 us * freq 在32位下溢出；
 *         经过的周期数用无符号回环减法判断，天然处理CYCCNT溢出。
 */
void dev_dwt_counter_blocking_delay_us(uint32_t us)
{
	if (us == 0u)
		return;

	uint32_t start = drv_dwt_timer_get_ticks();
	uint32_t wait = (uint32_t)(((uint64_t)us * dwt_timer.sys_freq_hz) / 1000000u);

	while ((drv_dwt_timer_get_ticks() - start) < wait)
	{
	}
}

/**
 * @brief  毫秒级阻塞延时
 * @param  ms 延时时长 (单位: 毫秒)
 * @note   按毫秒分段延时，单次目标周期数始终落在32位范围内，避免溢出。
 */
void dev_dwt_counter_blocking_delay_ms(uint32_t ms)
{
	while (ms-- > 0u)
	{
		dev_dwt_counter_blocking_delay_us(1000u);
	}
}
