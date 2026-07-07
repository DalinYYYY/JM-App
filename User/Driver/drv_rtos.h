/**
 * @file        drv_rtos.h
 * @brief       RTOS接口封装层，封装任务创建/延时/临界区/系统节拍，隔离具体RTOS
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
 * @note        对外接口不暴露RTOS类型，FreeRTOS/CMSIS-OS类型仅在drv_rtos.c内部使用
 */

#ifndef __DRV_RTOS_H__
#define __DRV_RTOS_H__

#include "drv_config.h"
#include <stdint.h>

/* 任务优先级定义 */
typedef enum
{
	DRV_RTOS_PRIORITY_LOW = 1,
	DRV_RTOS_PRIORITY_BELOW_NORMAL,
	DRV_RTOS_PRIORITY_NORMAL,
	DRV_RTOS_PRIORITY_ABOVE_NORMAL,
	DRV_RTOS_PRIORITY_HIGH,
	DRV_RTOS_PRIORITY_REALTIME,
} drv_rtos_priority_e;

/* 任务句柄类型 */
typedef void *drv_rtos_thread_handle_t;

/* 信号量句柄类型 */
typedef void *drv_rtos_sem_handle_t;

/* 互斥量句柄类型 */
typedef void *drv_rtos_mutex_handle_t;

/* 永久等待(不超时) */
#define DRV_RTOS_WAIT_FOREVER 0xFFFFFFFFU

/* 任务入口函数类型 */
typedef void (*drv_rtos_thread_func_t)(void const *argument);

/**
 * @brief 进入临界区
 */
void drv_rtos_enter_critical(void);

/**
 * @brief 退出临界区
 */
void drv_rtos_exit_critical(void);

/**
 * @brief 创建任务
 *
 * @param[in] name      任务名称
 * @param[in] func      任务入口函数
 * @param[in] priority  任务优先级
 * @param[in] stack_size 任务栈大小(字节)
 * @param[in] arg       任务参数
 *
 * @return 任务句柄，创建失败返回NULL
 */

/**
  * @brief  创建一个RTOS线程
  * @param  name          : 线程名称 (单位: 字符串, 范围: 任意有效字符串)
  * @param  func          : 线程入口函数 (单位: 函数指针, 范围: 任意符合类型定义的函数)
  * @param  priority      : 线程优先级 (单位: 枚举值, 范围: DRV_RTOS_PRIORITY_LOW~DRV_RTOS_PRIORITY_REALTIME)
  * @param  stack_size    : 线程栈大小 (单位: 字节, 范围: 0~UINT32_MAX)
  * @param  arg           : 传递给线程入口函数的参数 (单位: void指针, 范围: 任意有效指针)
  * @return : 成功返回线程句柄，失败返回NULL
  */
drv_rtos_thread_handle_t drv_rtos_thread_create(const char *name,
                                                drv_rtos_thread_func_t func,
                                                drv_rtos_priority_e priority,
                                                uint32_t stack_size,
                                                void const *arg);

/**
 * @brief 任务延时
 *
 * @param[in] ms 延时时间，单位毫秒
 */
void drv_rtos_delay_ms(uint32_t ms);

/**
 * @brief 获取系统运行时间
 *
 * @return 系统运行时间，单位毫秒
 */
uint32_t drv_rtos_get_tick_ms(void);

/*************************************** 信号量 ***************************************/

/**
 * @brief       创建信号量(计数/二值)
 * @param        max_count         : 最大计数值(二值信号量传1)
 * @param        init_count        : 初始计数值
 * @return       : 信号量句柄，失败返回NULL
 */
drv_rtos_sem_handle_t drv_rtos_sem_create(uint32_t max_count, uint32_t init_count);

/**
 * @brief       获取信号量(P操作)
 * @param        sem               : 信号量句柄
 * @param        timeout_ms        : 超时时间 (单位: ms，DRV_RTOS_WAIT_FOREVER为永久等待)
 * @return       : DRV_EOK成功，DRV_ERROR超时或失败
 */
int drv_rtos_sem_wait(drv_rtos_sem_handle_t sem, uint32_t timeout_ms);

/**
 * @brief       释放信号量(V操作)
 * @param        sem               : 信号量句柄
 * @return       : DRV_EOK成功，DRV_ERROR失败
 */
int drv_rtos_sem_release(drv_rtos_sem_handle_t sem);

/**
 * @brief       删除信号量
 * @param        sem               : 信号量句柄
 * @return       : DRV_EOK成功，DRV_ERROR失败
 */
int drv_rtos_sem_delete(drv_rtos_sem_handle_t sem);

/*************************************** 互斥量 ***************************************/

/**
 * @brief       创建互斥量
 * @return       : 互斥量句柄，失败返回NULL
 */
drv_rtos_mutex_handle_t drv_rtos_mutex_create(void);

/**
 * @brief       获取互斥量(加锁)
 * @param        mutex             : 互斥量句柄
 * @param        timeout_ms        : 超时时间 (单位: ms，DRV_RTOS_WAIT_FOREVER为永久等待)
 * @return       : DRV_EOK成功，DRV_ERROR超时或失败
 */
int drv_rtos_mutex_lock(drv_rtos_mutex_handle_t mutex, uint32_t timeout_ms);

/**
 * @brief       释放互斥量(解锁)
 * @param        mutex             : 互斥量句柄
 * @return       : DRV_EOK成功，DRV_ERROR失败
 */
int drv_rtos_mutex_unlock(drv_rtos_mutex_handle_t mutex);

/**
 * @brief       删除互斥量
 * @param        mutex             : 互斥量句柄
 * @return       : DRV_EOK成功，DRV_ERROR失败
 */
int drv_rtos_mutex_delete(drv_rtos_mutex_handle_t mutex);

#endif /* __DRV_RTOS_H__ */
