/**
 * @file thread_management.c
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

#include "main.h"

#include "drv_rtos.h"
#include "thread_config.h"
#include "thread_management.h"
#include "thread_control.h"
#include "thread_commun.h"
#include "thread_display.h"
#include "thread_period.h"
#include "thread_idle.h"

#define THREAD_IDLE_SIZE 128 * 2
#define THREAD_PERIOD_SIZE 128 * 1
#define THREAD_DISPLAY_SIZE 128 * 8
#define THREAD_COMMUN_SIZE 128 * 24 /* 3072B：容纳 motor_info_storage_save(1024B tmp) + crc32_compute 嵌套(1024B) + 调用链余量 */
#define THREAD_CONTROL_SIZE 128 * 4

/* 任务列表ID */
enum
{
	THREAD_ID_CONTROL,
	THREAD_ID_COMMUN,
	THREAD_ID_DISPLAY,
	THREAD_ID_PERIOD,
	THREAD_ID_IDLE,
	THREAD_ID_MAX
} threadID_e;

drv_rtos_thread_handle_t TaskHandle[THREAD_ID_MAX];

/**
 * @brief 应用层任务创建
 *
 *
 */
void thread_init(void)
{
	int index;

	drv_rtos_enter_critical();

	/* 控制任务 */
#ifdef USE_CONTROL_THREAD
	index = THREAD_ID_CONTROL;
	TaskHandle[index] = drv_rtos_thread_create("ControlTask",
											   control_thread,
											   DRV_RTOS_PRIORITY_HIGH,
											   THREAD_CONTROL_SIZE,
											   NULL);
#endif

#ifdef USE_COMMUN_THREAD
	/* 主通讯任务 */
	index = THREAD_ID_COMMUN;
	TaskHandle[index] = drv_rtos_thread_create("CommunTask",
											   commun_thread,
											   DRV_RTOS_PRIORITY_HIGH,
											   THREAD_COMMUN_SIZE,
											   NULL);
#endif

#ifdef USE_DISPLAY_THREAD
	/* 显示及可视化任务 */
	index = THREAD_ID_DISPLAY;
	TaskHandle[index] = drv_rtos_thread_create("DisplayTask",
											   display_thread,
											   DRV_RTOS_PRIORITY_ABOVE_NORMAL,
											   THREAD_DISPLAY_SIZE,
											   NULL);
#endif

#ifdef USE_PERIOD_THREAD
	/* 周期任务*/
	index = THREAD_ID_PERIOD;
	TaskHandle[index] = drv_rtos_thread_create("PeriodTask",
											   period_thread,
											   DRV_RTOS_PRIORITY_ABOVE_NORMAL,
											   THREAD_PERIOD_SIZE,
											   NULL);
#endif

#ifdef USE_IDLE_THREAD
	/* 空闲任务 */
	index = THREAD_ID_IDLE;
	TaskHandle[index] = drv_rtos_thread_create("IdleTask",
											   idle_thread,
											   DRV_RTOS_PRIORITY_ABOVE_NORMAL,
											   THREAD_IDLE_SIZE,
											   NULL);
#endif

	drv_rtos_exit_critical();
}
