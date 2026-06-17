/**
 * @file        drv_tim.h
 * @brief       定时器驱动接口，封装HAL的定时器计数/重装载/中断等运行期接口
 *
 * @author      Dalin (dalin@robot.com)
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
 * @note        定时器初始化由CubeMX的MX_TIMx_Init完成，此层仅封装运行期接口
 */
#ifndef HAL_DRIVER_TIM_H
#define HAL_DRIVER_TIM_H
#include <stdint.h>
#include "drv_config.h"

#ifdef USE_TIM_PWM_DRIVER
#define USE_TIM_DRIVER
#endif
#ifdef USE_TIM_ENCODER_DRIVER
#define USE_TIM_DRIVER
#endif

#ifdef USE_TIM_DRIVER

/**
 * @brief 定时器编号
 * @param  DRV_TIM_INIT          : 无效占位(0)
 * @param  DRV_TIM1~DRV_TIM14    : TIM1~TIM14
 * @param  DRV_TIM_NUMBER_MAX    : 定时器数量边界
 */
typedef enum TIMNUMBER_
{
	DRV_TIM_INIT = 0,
	DRV_TIM1,
	DRV_TIM2,
	DRV_TIM3,
	DRV_TIM4,
	DRV_TIM5,
	DRV_TIM6,
	DRV_TIM7,
	DRV_TIM8,
	DRV_TIM9,
	DRV_TIM10,
	DRV_TIM11,
	DRV_TIM12,
	DRV_TIM13,
	DRV_TIM14,

	DRV_TIM_NUMBER_MAX
} timNumber_e;

/**
 * @brief 定时器通道
 * @param  TIM_CH1               : 通道1
 * @param  TIM_CH2               : 通道2
 * @param  TIM_CH3               : 通道3
 * @param  TIM_CH4               : 通道4
 * @param  TIM_CH_ALL            : 全部通道
 * @param  TIM_CH_MAX            : 通道数量边界
 */
typedef enum TIMCHANNEL_
{
	TIM_CH1,
	TIM_CH2,
	TIM_CH3,
	TIM_CH4,
	TIM_CH_ALL,
	TIM_CH_MAX
} timChannel_e;

/**
 * @brief       获取定时器预分频值
 * @param        tim               : 定时器编号
 * @param        psc               : 输出预分频值(已+1，即实际分频系数)
 * @return       : DRV_EOK成功，DRV_ERROR失败
 */
int drv_tim_get_prescaler(timNumber_e tim, uint32_t *psc);

/**
 * @brief       设置定时器自动重装载值(ARR)
 * @param        tim               : 定时器编号
 * @param        period            : 重装载值
 * @return       : DRV_EOK成功，DRV_ERROR失败
 */
int drv_tim_set_autoreload(timNumber_e tim, uint32_t period);

/**
 * @brief       获取定时器自动重装载值
 * @param        tim               : 定时器编号
 * @param        autoreload        : 输出重装载值(已+1，即实际计数周期)
 * @return       : DRV_EOK成功，DRV_ERROR失败
 */
int drv_tim_get_autoreload(timNumber_e tim, uint16_t *autoreload);

/**
 * @brief       设置定时器计数值(CNT)
 * @param        tim               : 定时器编号
 * @param        cnt               : 计数值
 * @return       : DRV_EOK成功，DRV_ERROR失败
 */
int drv_tim_set_counter(timNumber_e tim, uint32_t cnt);

/**
 * @brief       获取定时器计数值(CNT)
 * @param        tim               : 定时器编号
 * @param        cnt               : 输出计数值
 * @return       : DRV_EOK成功，DRV_ERROR失败
 */
int drv_tim_get_counter(timNumber_e tim, uint32_t *cnt);

/**
 * @brief       启动定时器(基本计数+更新中断)
 * @param        tim               : 定时器编号
 * @return       : DRV_EOK成功，DRV_ERROR失败
 */
int drv_tim_start_it(timNumber_e tim);

/**
 * @brief       停止定时器(基本计数+更新中断)
 * @param        tim               : 定时器编号
 * @return       : DRV_EOK成功，DRV_ERROR失败
 */
int drv_tim_stop_it(timNumber_e tim);

#endif /* USE_TIM_DRIVER */
#endif /* HAL_DRIVER_TIM_H   */
