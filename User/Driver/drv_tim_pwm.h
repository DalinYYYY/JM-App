/**
 * @file        drv_tim_pwm.h
 * @brief       PWM驱动接口，封装HAL的定时器PWM输出(含互补输出)功能
 *
 * @author      Dalin (dalinyy@163.com)
 * @version     1.0
 * @date        2026-06-16
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @par 修改日志:
 * | 日期       | 版本 | 作者   | 修改内容   |
 * |------------|------|--------|------------|
 * | 2026-06-16 | 1.0  | Dalin  | 初始创建   |
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 * @note        定时器/通道编号及句柄映射由drv_tim提供
 */
#ifndef HAL_CUBEMX_DRIVER_PWM_H
#define HAL_CUBEMX_DRIVER_PWM_H
#include <stdint.h>
#include "drv_tim.h"
#include "drv_config.h"

#ifdef USE_TIM_PWM_DRIVER

/**
 * @brief       启动PWM输出
 * @param        tim               : 定时器编号
 * @param        channel           : 通道编号
 * @return       : DRV_EOK成功，DRV_ERROR失败
 */
int drv_pwm_start(timNumber_e tim, timChannel_e channel);

/**
 * @brief       启动PWM互补输出
 * @param        tim               : 定时器编号
 * @param        channel           : 通道编号
 * @return       : DRV_EOK成功，DRV_ERROR失败
 */
int drv_pwmN_start(timNumber_e tim, timChannel_e channel);

/**
 * @brief       停止PWM输出
 * @param        tim               : 定时器编号
 * @param        channel           : 通道编号
 * @return       : DRV_EOK成功，DRV_ERROR失败
 */
int drv_pwm_stop(timNumber_e tim, timChannel_e channel);

/**
 * @brief       停止PWM互补输出
 * @param        tim               : 定时器编号
 * @param        channel           : 通道编号
 * @return       : DRV_EOK成功，DRV_ERROR失败
 */
int drv_pwmN_stop(timNumber_e tim, timChannel_e channel);

/**
 * @brief       设置PWM占空比(比较值)
 * @param        tim               : 定时器编号
 * @param        channel           : 通道编号
 * @param        duty              : 比较值(CCR)，范围0~自动重装载值ARR
 * @return       : DRV_EOK成功，DRV_ERROR失败
 */
int drv_pwm_set_dutycycle(timNumber_e tim, timChannel_e channel, uint32_t duty);

#endif /* USE_TIM_PWM_DRIVER */
#endif /* HAL_CUBEMX_DRIVER_PWM_H   */
