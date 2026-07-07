/**
 * @file        drv_delay.c
 * @brief       延时驱动实现，基于DWT周期计数器实现us/ms级阻塞延时
 *
 * @author      Dalin (dalinyy@163.com)
 * @version     1.0
 * @date        2026-06-16
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @par 修改日志:
 * | 日期       | 版本 | 作者   | 修改内容   |
 * |------------|------|--------|------------|
 * | 2026-06-16 | 1.0  | Dalin  | 初始创建   |
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 */
#include "drv_delay.h"
#include "drv_dwt_timer.h"

/* CMSIS标准全局变量(系统内核时钟，Hz)，extern声明以避免引入HAL头 */
extern uint32_t SystemCoreClock;

/**
 * @brief       微秒级阻塞延时
 * @note         基于DWT周期计数，单次最大延时受时钟频率限制；大延时由drv_delay_ms分段调用
 */
void drv_delay_us(uint32_t us)
{
	uint32_t start = drv_dwt_timer_get_ticks();
	uint32_t cycles = us * (SystemCoreClock / 1000000U);

	while ((drv_dwt_timer_get_ticks() - start) < cycles)
		;
}

/**
 * @brief       毫秒级阻塞延时
 * @note         按毫秒分段调用drv_delay_us，避免周期数溢出32位
 */
void drv_delay_ms(uint32_t ms)
{
	while (ms--)
	{
		drv_delay_us(1000U);
	}
}
