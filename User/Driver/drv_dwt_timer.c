#include "drv_dwt_timer.h"

/**
 * @brief  初始化DWT周期计数器
 * @details 使能内核调试跟踪(TRCENA)与DWT周期计数器(CYCCNT)，并清零计数。
 *          幂等：重复调用仅确保已使能并复位计数，不会反复配置。
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
