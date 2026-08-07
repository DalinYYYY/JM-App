/**
 * @file        drv_rtos.c
 * @brief       RTOS接口封装层实现(基于FreeRTOS + CMSIS-OS v1)
 *
 * @author      Dalin (dalinyy@163.com)
 * @version     1.0
 * @date        2026-06-10
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @par 修改日志:
 * | 日期       | 版本 | 作者   | 修改内容   |
 * |------------|------|--------|------------|
 * | 2026-06-10 | 1.0  | Dalin  | 初始创建   |
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
	osThreadId handle;
	uint8_t in_use;
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

/**
 * @brief       创建一个RTOS线程
 */
drv_rtos_thread_handle_t drv_rtos_thread_create(const char *name,
                                                drv_rtos_thread_func_t func,
                                                drv_rtos_priority_e priority,
                                                uint32_t stack_size,
                                                void const *arg)
{
	int i;
	drv_rtos_thread_ctrl_t *ctrl = NULL;

	(void)name; /* CMSIS-OS v1的osThreadDef_t无name成员，任务名暂不使用 */

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
	/* 入参为字节，CMSIS-OS v1的stacksize单位是字(4字节)，此处换算 */
	ctrl->def.stacksize = stack_size / 4U;

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

/* 超时(ms)转CMSIS-OS等待节拍 */
static inline uint32_t drv_rtos_to_wait(uint32_t timeout_ms)
{
	return (timeout_ms == DRV_RTOS_WAIT_FOREVER) ? osWaitForever : timeout_ms;
}

/*************************************** 信号量 ***************************************/

/**
 * @brief       创建信号量(计数/二值)
 */
drv_rtos_sem_handle_t drv_rtos_sem_create(uint32_t max_count, uint32_t init_count)
{
	osSemaphoreDef_t sem_def = {0};
	osSemaphoreId id;

	(void)max_count; /* CMSIS-OS v1按count创建，最大值由FreeRTOS内部管理 */

	id = osSemaphoreCreate(&sem_def, (int32_t)init_count);
	return (drv_rtos_sem_handle_t)id;
}

/**
 * @brief       获取信号量(P操作)
 */
int drv_rtos_sem_wait(drv_rtos_sem_handle_t sem, uint32_t timeout_ms)
{
	if (sem == NULL)
		return DRV_ERROR;

	/* osSemaphoreWait返回可用token数，>0为成功获取 */
	return (osSemaphoreWait((osSemaphoreId)sem, drv_rtos_to_wait(timeout_ms)) == osOK) ? DRV_EOK : DRV_ERROR;
}

/**
 * @brief       释放信号量(V操作)
 */
int drv_rtos_sem_release(drv_rtos_sem_handle_t sem)
{
	if (sem == NULL)
		return DRV_ERROR;

	return (osSemaphoreRelease((osSemaphoreId)sem) == osOK) ? DRV_EOK : DRV_ERROR;
}

/**
 * @brief       删除信号量
 */
int drv_rtos_sem_delete(drv_rtos_sem_handle_t sem)
{
	if (sem == NULL)
		return DRV_ERROR;

	return (osSemaphoreDelete((osSemaphoreId)sem) == osOK) ? DRV_EOK : DRV_ERROR;
}

/*************************************** 互斥量 ***************************************/

/**
 * @brief       创建互斥量
 */
drv_rtos_mutex_handle_t drv_rtos_mutex_create(void)
{
	osMutexDef_t mutex_def = {0};
	osMutexId id;

	id = osMutexCreate(&mutex_def);
	return (drv_rtos_mutex_handle_t)id;
}

/**
 * @brief       获取互斥量(加锁)
 */
int drv_rtos_mutex_lock(drv_rtos_mutex_handle_t mutex, uint32_t timeout_ms)
{
	if (mutex == NULL)
		return DRV_ERROR;

	return (osMutexWait((osMutexId)mutex, drv_rtos_to_wait(timeout_ms)) == osOK) ? DRV_EOK : DRV_ERROR;
}

/**
 * @brief       释放互斥量(解锁)
 */
int drv_rtos_mutex_unlock(drv_rtos_mutex_handle_t mutex)
{
	if (mutex == NULL)
		return DRV_ERROR;

	return (osMutexRelease((osMutexId)mutex) == osOK) ? DRV_EOK : DRV_ERROR;
}

/**
 * @brief       删除互斥量
 */
int drv_rtos_mutex_delete(drv_rtos_mutex_handle_t mutex)
{
	if (mutex == NULL)
		return DRV_ERROR;

	return (osMutexDelete((osMutexId)mutex) == osOK) ? DRV_EOK : DRV_ERROR;
}
