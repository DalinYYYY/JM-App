/**
 * @file        drv_dwt_timer.c
 * @brief       DWT周期计数器驱动实现，提供CPU周期级高精度计时
 *
 * @author      Dalin (dalin@robot.com)
 * @version     1.0
 * @date        2026-06-12
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @par 修改日志:
 * | 日期       | 版本 | 作者   | 修改内容   |
 * |------------|------|--------|------------|
 * | 2026-06-12 | 1.0  | Dalin  | 初始创建   |
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 */
#include "drv_dwt_timer.h"

/**
 * @brief       初始化DWT周期计数器
 * @note         使能内核调试跟踪(TRCENA)与DWT周期计数器(CYCCNT)并清零；
 *               幂等，重复调用仅确保已使能并复位计数
 */
void drv_dwt_timer_init(void)
{
	/* 使能内核调试跟踪 */
	DEM_CR |= DEM_CR_TRCENA;

	/* 解锁DWT寄存器（部分内核需要） */
	DWT_LAR = DWT_LAR_UNLOCK;

	/* 清零并使能周期计数器 */
	DWT_CYCCNT = 0u;
	DWT_CR |= DWT_CR_CYCCNTENA;
}
