/**
 * @file thread_control.c
 * @brief 
 * @author Dalin
 * @version 1.00
 * @date 2025-02-11
 * 
 * @copyright Copyright (c) 2025  RobotDance Technology Co., Ltd.
 * 
 * @par 修改日志:
 * <table>
 * <tr><th>Date           <th>Version     <th>Author      <th>Description
 * <tr><td>2025-02-11     <td>1.00        <td>Mr.Lin      <td>Init
 * </table>
 */
#include "drv_rtos.h"

#include "thread_config.h"
#include "thread_control.h"
#include "runtime_param.h"

void control_thread(void const *argument)
{
	drv_rtos_delay_ms(INTO_THREAD_DELAY / 20);

	for (;;)
	{

		drv_rtos_delay_ms(THREAD_DELAY_CONTROL * 500);
		/* 任务计数 */
		usr.sys.task_cnt.control_cnt++;
	}
}
