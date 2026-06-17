/**
 * @file        drv_tim.c
 * @brief       定时器驱动实现，封装HAL的定时器计数/重装载/中断等运行期接口
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
 */
#include "drv_tim.h"

#if defined(USE_TIM_PWM_DRIVER) || defined(USE_TIM_ENCODER_DRIVER)
#define USE_TIM_DRIVER
#endif

#ifdef USE_TIM_DRIVER

#include "tim.h"

__weak TIM_HandleTypeDef htim1;
__weak TIM_HandleTypeDef htim2;
__weak TIM_HandleTypeDef htim3;
__weak TIM_HandleTypeDef htim4;
__weak TIM_HandleTypeDef htim5;
__weak TIM_HandleTypeDef htim6;
__weak TIM_HandleTypeDef htim7;
__weak TIM_HandleTypeDef htim8;
__weak TIM_HandleTypeDef htim9;
__weak TIM_HandleTypeDef htim10;
__weak TIM_HandleTypeDef htim11;
__weak TIM_HandleTypeDef htim12;
__weak TIM_HandleTypeDef htim13;
__weak TIM_HandleTypeDef htim14;

/* 定时器句柄查找表：以timNumber_e为索引，O(1)定位HAL句柄 */
static TIM_HandleTypeDef *const s_tim_map[DRV_TIM_NUMBER_MAX] = {
	[DRV_TIM1] = &htim1,
	[DRV_TIM2] = &htim2,
	[DRV_TIM3] = &htim3,
	[DRV_TIM4] = &htim4,
	[DRV_TIM5] = &htim5,
	[DRV_TIM6] = &htim6,
	[DRV_TIM7] = &htim7,
	[DRV_TIM8] = &htim8,
	[DRV_TIM9] = &htim9,
	[DRV_TIM10] = &htim10,
	[DRV_TIM11] = &htim11,
	[DRV_TIM12] = &htim12,
	[DRV_TIM13] = &htim13,
	[DRV_TIM14] = &htim14,
};

/* 通道查找表：TIM_CHANNEL_x为位域宏(非连续序号)，以timChannel_e为索引直接映射 */
static const uint32_t s_tim_channel_map[TIM_CH_MAX] = {
	[TIM_CH1] = TIM_CHANNEL_1,
	[TIM_CH2] = TIM_CHANNEL_2,
	[TIM_CH3] = TIM_CHANNEL_3,
	[TIM_CH4] = TIM_CHANNEL_4,
	[TIM_CH_ALL] = TIM_CHANNEL_ALL,
};

/* 句柄/通道转换为drv_tim.c内部使用，不对外暴露HAL类型(见drv_tim.h规则) */
static inline TIM_HandleTypeDef *get_tim_handle(timNumber_e tim)
{
	if (tim >= DRV_TIM_NUMBER_MAX)
		return NULL;

	return s_tim_map[tim];
}

static inline uint32_t get_tim_channel(timChannel_e channel)
{
	if (channel >= TIM_CH_MAX)
		return 0xFFFFFFFF;

	return s_tim_channel_map[channel];
}
/**
 * @brief       获取定时器预分频值
 */
int drv_tim_get_prescaler(timNumber_e tim, uint32_t *psc)
{
	TIM_HandleTypeDef *handle;

	handle = get_tim_handle(tim);
	if (handle == NULL)
	{
		return DRV_ERROR;
	}
	*psc = handle->Instance->PSC + 1;
	return DRV_EOK;
}

/**
 * @brief       设置定时器计数值
 */
int drv_tim_set_counter(timNumber_e tim, uint32_t cnt)
{
	TIM_HandleTypeDef *handle;

	handle = get_tim_handle(tim);
	if (handle == NULL)
	{
		return DRV_ERROR;
	}
	__HAL_TIM_SET_COUNTER(handle, cnt);
	return DRV_EOK;
}

/**
 * @brief       获取定时器计数值
 */
int drv_tim_get_counter(timNumber_e tim, uint32_t *cnt)
{
	TIM_HandleTypeDef *handle;

	handle = get_tim_handle(tim);
	if (handle == NULL)
	{
		return DRV_ERROR;
	}
	*cnt = __HAL_TIM_GET_COUNTER(handle);
	return DRV_EOK;
}

/**
 * @brief       启动定时器(基本计数+更新中断)
 */
int drv_tim_start_it(timNumber_e tim)
{
	TIM_HandleTypeDef *handle;

	handle = get_tim_handle(tim);
	if (handle == NULL)
	{
		return DRV_ERROR;
	}
	/* 启动函数 */
	HAL_TIM_Base_Start_IT(handle);

	return DRV_EOK;
}

/**
 * @brief       停止定时器(基本计数+更新中断)
 */
int drv_tim_stop_it(timNumber_e tim)
{
	TIM_HandleTypeDef *handle;

	handle = get_tim_handle(tim);
	if (handle == NULL)
	{
		return DRV_ERROR;
	}
	/* 停止函数 */
	HAL_TIM_Base_Stop_IT(handle);
	return DRV_EOK;
}

/**
 * @brief       设置定时器自动重装载值
 */
int drv_tim_set_autoreload(timNumber_e tim, uint32_t period)
{
	TIM_HandleTypeDef *handle;

	handle = get_tim_handle(tim);
	if (handle == NULL)
	{
		return DRV_ERROR;
	}
	/* 周期 */
	__HAL_TIM_SET_AUTORELOAD(handle, period);
	return DRV_EOK;
}
/**
 * @brief       获取定时器自动重装载值
 */
int drv_tim_get_autoreload(timNumber_e tim, uint16_t *autoreload)
{
	TIM_HandleTypeDef *handle;

	handle = get_tim_handle(tim);
	if (handle == NULL)
	{
		return DRV_ERROR;
	}
	*autoreload = (__HAL_TIM_GET_AUTORELOAD(handle) + 1);
	return DRV_EOK;
}

/* ----------------------------- PWM 输出(drv_tim_pwm.h) ----------------------------- */
/* PWM实现并入本编译单元，以复用static的get_tim_handle/get_tim_channel，避免对外暴露HAL类型 */
#ifdef USE_TIM_PWM_DRIVER

/**
 * @brief       启动PWM输出
 */
int drv_pwm_start(timNumber_e tim, timChannel_e channel)
{
	TIM_HandleTypeDef *handle;

	handle = get_tim_handle(tim);
	if (handle == NULL)
	{
		return DRV_ERROR;
	}
	HAL_TIM_PWM_Start(handle, get_tim_channel(channel));

	return DRV_EOK;
}

/**
 * @brief       启动PWM互补输出
 */
int drv_pwmN_start(timNumber_e tim, timChannel_e channel)
{
	TIM_HandleTypeDef *handle;

	handle = get_tim_handle(tim);
	if (handle == NULL)
	{
		return DRV_ERROR;
	}
	HAL_TIMEx_PWMN_Start(handle, get_tim_channel(channel));

	return DRV_EOK;
}

/**
 * @brief       停止PWM输出
 */
int drv_pwm_stop(timNumber_e tim, timChannel_e channel)
{
	TIM_HandleTypeDef *handle;

	handle = get_tim_handle(tim);
	if (handle == NULL)
	{
		return DRV_ERROR;
	}
	HAL_TIM_PWM_Stop(handle, get_tim_channel(channel));

	return DRV_EOK;
}

/**
 * @brief       停止PWM互补输出
 */
int drv_pwmN_stop(timNumber_e tim, timChannel_e channel)
{
	TIM_HandleTypeDef *handle;

	handle = get_tim_handle(tim);
	if (handle == NULL)
	{
		return DRV_ERROR;
	}
	HAL_TIMEx_PWMN_Stop(handle, get_tim_channel(channel));

	return DRV_EOK;
}

/**
 * @brief       设置PWM占空比(比较值)
 */
int drv_pwm_set_dutycycle(timNumber_e tim, timChannel_e channel, uint32_t duty)
{
	TIM_HandleTypeDef *handle;

	handle = get_tim_handle(tim);
	if (handle == NULL)
	{
		return DRV_ERROR;
	}
	/* 直接改写CCR寄存器，热路径不触发HAL重配置 */
	__HAL_TIM_SET_COMPARE(handle, get_tim_channel(channel), duty);

	return DRV_EOK;
}

#endif /* USE_TIM_PWM_DRIVER */

#endif /* USE_TIM_DRIVER */
