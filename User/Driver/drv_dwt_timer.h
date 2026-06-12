#ifndef _DRV_DWT_TIMER_H_
#define _DRV_DWT_TIMER_H_

#include "stm32G4xx_hal.h"
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
 * @brief  初始化DWT周期计数器（使能TRC与CYCCNT，并清零）
 */
	void drv_dwt_timer_init(void);

	/**
 * @brief  读取DWT周期计数值
 * @return 当前CYCCNT计数（单位：CPU时钟周期）
 * @note   inline 实现，消除函数调用开销，供高频/阻塞延时直接读寄存器。
 */
	static inline uint32_t drv_dwt_timer_get_ticks(void)
	{
		return DWT_CYCCNT;
	}

#ifdef __cplusplus
}
#endif

#endif /* _DRV_DWT_TIMER_H_ */
