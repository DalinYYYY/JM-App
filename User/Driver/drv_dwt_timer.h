/**
 * @file        drv_dwt_timer.h
 * @brief       DWT周期计数器驱动接口，提供CPU周期级高精度计时
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
 * @note        DWT为Cortex-M3/M4/M7内核外设，仅用裸寄存器地址，不依赖HAL/具体型号；
 *              Cortex-M0/M0+无DWT，不适用
 */
#ifndef _DRV_DWT_TIMER_H_
#define _DRV_DWT_TIMER_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define DWT_LAR_UNLOCK (uint32_t)0xC5ACCE55		   /* DWT锁定访问寄存器解锁密钥 */
#define DWT_LAR (*(volatile uint32_t *)0xE0000FB0) /* DWT锁定访问寄存器地址 */

#define DWT_CR (*(volatile uint32_t *)0xE0001000)	  /* DWT控制寄存器地址 */
#define DWT_CYCCNT (*(volatile uint32_t *)0xE0001004) /* DWT时钟周期计数器地址 */
#define DEM_CR (*(volatile uint32_t *)0xE000EDFC)	  /* 调试异常与监视控制寄存器地址 */
#define DEM_CR_TRCENA (1UL << 24)					  /* DEM_CR: DWT/ITM跟踪使能位 */
#define DWT_CR_CYCCNTENA (1UL << 0)					  /* DWT_CR: 时钟周期计数器使能位 */

	/**
 * @brief       初始化DWT周期计数器(使能TRC与CYCCNT并清零)
 */
	void drv_dwt_timer_init(void);

	/**
 * @brief       读取DWT周期计数值
 * @return       : 当前CYCCNT计数 (单位: CPU时钟周期)
 * @note         inline实现，消除调用开销，供高频/阻塞延时直接读寄存器
 */
	static inline uint32_t drv_dwt_timer_get_ticks(void)
	{
		return DWT_CYCCNT;
	}

#ifdef __cplusplus
}
#endif

#endif /* _DRV_DWT_TIMER_H_ */
