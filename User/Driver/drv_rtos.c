/**
 * @file        drv_rtos.c
 * @brief       RTOS接口封装层源文件
 * 
 * @author      name (name@robot.com)
 * @version     1.0
 * @date        2026-06-10
 * 
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 * 
 * 
 * @par 修改日志:
 * | 日期       | 版本 | 作者   | 修改内容   |
 * |------------|------|--------|------------|
 * | 2026-06-10     | 1.0  | yangsl | 初始创建   |
 * 
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 */

#include "drv_rtos.h"

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"

/* 最大任务数量 */
#define DRV_RTOS_MAX_THREADS 8

/* 内部任务控制块 */
typedef struct
{
	osThreadDef_t def;
	osThreadId	  handle;
	uint8_t		  in_use;
} drv_rtos_thread_ctrl_t;

static drv_rtos_thread_ctrl_t s_thread_ctrl[DRV_RTOS_MAX_THREADS];

/**
 * @brief 获取FreeRTOS优先级
 */
static osPriority drv_rtos_get_freertos_priority(drv_rtos_priority_e priority)
{
	switch (priority)
	{
		case DRV_RTOS_PRIORITY_LOW:
			return osPriorityLow;
		case DRV_RTOS_PRIORITY_BELOW_NORMAL:
			return osPriorityBelowNormal;
		case DRV_RTOS_PRIORITY_NORMAL:
			return osPriorityNormal;
		case DRV_RTOS_PRIORITY_ABOVE_NORMAL:
			return osPriorityAboveNormal;
		case DRV_RTOS_PRIORITY_HIGH:
			return osPriorityHigh;
		case DRV_RTOS_PRIORITY_REALTIME:
			return osPriorityRealtime;
		default:
			return osPriorityNormal;
	}
}

/**
 * @brief 进入临界区
 */
void drv_rtos_enter_critical(void)
{
	taskENTER_CRITICAL();
}

/**
 * @brief 退出临界区
 */
void drv_rtos_exit_critical(void)
{
	taskEXIT_CRITICAL();
}

// 创建一个RTOS线程
drv_rtos_thread_handle_t drv_rtos_thread_create(const char			  *name,
												drv_rtos_thread_func_t func,
												drv_rtos_priority_e	   priority,
												uint32_t			   stack_size,
												void const			  *arg)
{
	int						i;
	drv_rtos_thread_ctrl_t *ctrl = NULL;

	(void)name;

	/* 查找空闲控制块 */
	for (i = 0; i < DRV_RTOS_MAX_THREADS; i++)
	{
		if (s_thread_ctrl[i].in_use == 0)
		{
			ctrl = &s_thread_ctrl[i];
			break;
		}
	}

	if (ctrl == NULL)
	{
		return NULL;
	}

	ctrl->def.pthread = (os_pthread)func;
	ctrl->def.tpriority = drv_rtos_get_freertos_priority(priority);
	ctrl->def.instances = 0;
	ctrl->def.stacksize = stack_size;

	ctrl->handle = osThreadCreate(&ctrl->def, (void *)arg);
	if (ctrl->handle == NULL)
	{
		return NULL;
	}

	ctrl->in_use = 1;

	return (drv_rtos_thread_handle_t)ctrl->handle;
}

/**
 * @brief 延迟指定毫秒数
 * @param  ms            : 延迟时间 (单位: ms, 范围: 0~UINT32_MAX)
 */
void drv_rtos_delay_ms(uint32_t ms)
{
	osDelay(ms);
}

/**
 * @brief 获取系统运行时间
 * @return 系统运行时间 (单位: ms, 范围: 0~UINT32_MAX)
 */
uint32_t drv_rtos_get_tick_ms(void)
{
	return osKernelSysTick();
}
