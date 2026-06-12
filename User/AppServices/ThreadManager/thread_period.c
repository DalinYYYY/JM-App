/**
 * @file thread_period.c
 * @brief 
 * 
 * @author dalin (dalin@robot.com)
 * @version 1.0
 * @date 2026-06-10
 * 
 * @copyright Copyright (c) 2026 Robot Tech.co, Ltd. All rights reserved.
 * 
 * @par 修改日志:
 * <table>
 * <tr><th>日期</th><th>版本</th><th>作者</th><th>修改内容</th></tr>
 * <tr><td>2026-06-10</td><td>1.0</td><td>yangsl</td><td>初始创建</td></tr>
 * </table>
 * 
 * @note 本文件遵循《嵌入式C代码规范V1.0》开发
 */

#include "drv_rtos.h"

#include "runtime_param.h"
#include "thread_period.h"
#include "thread_config.h"

void period_thread(void const *argument)
{

	/* Infinite loop */
	drv_rtos_delay_ms(INTO_THREAD_DELAY / 5);

	for (;;)
	{
		drv_rtos_delay_ms(THREAD_DELAY_PERIOD);

		/* 任务计数 */
		usr.sys.task_cnt.period_cnt++;
	}
}
