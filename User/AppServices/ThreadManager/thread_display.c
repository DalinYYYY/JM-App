/**
 * @file thread_display.c
 * @brief 
 * 
 * @author dalin (dalinyy@163.com)
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
#include "thread_display.h"
#include "thread_config.h"
#include "user_interface.h"
#include "dev_config.h"
#if defined(USE_DEV_LED)
#include "led_manager.h"
#endif

void display_thread(void const *argument)
{
	/* Infinite loop */
	drv_rtos_delay_ms(INTO_THREAD_DELAY / 2);

	for (;;)
	{

		drv_rtos_delay_ms(THREAD_DELAY_DISPLAY);

#if defined(USE_DEV_LED)
		/* LED 状态指示周期更新（10ms 周期, 由状态机驱动 LED1/LED2 行为） */
		led_manager_update();
#endif

		/* 任务计数 */
		usr.sys.task_cnt.display_cnt++;
	}
}
