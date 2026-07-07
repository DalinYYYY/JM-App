/**
 * @file        drv_adc.c
 * @brief       ADC驱动实现，封装HAL的ADC规则/注入/DMA转换功能
 *
 * @author      Dalin (dalinyy@163.com)
 * @version     1.0
 * @date        2026-06-15
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @par 修改日志:
 * | 日期       | 版本 | 作者   | 修改内容   |
 * |------------|------|--------|------------|
 * | 2026-06-15 | 1.0  | Dalin  | 初始创建   |
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 */
#include "drv_adc.h"

#ifdef USE_ADC_DRIVER
#include "adc.h"

__weak ADC_HandleTypeDef hadc1;
__weak ADC_HandleTypeDef hadc2;
__weak ADC_HandleTypeDef hadc3;

/* ADC句柄查找表：以adcNumber_e为索引，O(1)定位HAL句柄 */
static ADC_HandleTypeDef *const s_adc_map[DRV_ADC_MAX] = {
	[DRV_ADC_1] = &hadc1,
	[DRV_ADC_2] = &hadc2,
	[DRV_ADC_3] = &hadc3,
};

/* 通道查找表：ADC_CHANNEL_x为位域宏(非连续序号)，以adcChannel_e为索引直接映射 */
static const uint32_t s_adc_channel_map[DRV_ADC_MAX_CH] = {
	[DRV_ADC_CH0] = ADC_CHANNEL_0,
	[DRV_ADC_CH1] = ADC_CHANNEL_1,
	[DRV_ADC_CH2] = ADC_CHANNEL_2,
	[DRV_ADC_CH3] = ADC_CHANNEL_3,
	[DRV_ADC_CH4] = ADC_CHANNEL_4,
	[DRV_ADC_CH5] = ADC_CHANNEL_5,
	[DRV_ADC_CH6] = ADC_CHANNEL_6,
	[DRV_ADC_CH7] = ADC_CHANNEL_7,
	[DRV_ADC_CH8] = ADC_CHANNEL_8,
	[DRV_ADC_CH9] = ADC_CHANNEL_9,
	[DRV_ADC_CH10] = ADC_CHANNEL_10,
	[DRV_ADC_CH11] = ADC_CHANNEL_11,
	[DRV_ADC_CH12] = ADC_CHANNEL_12,
	[DRV_ADC_CH13] = ADC_CHANNEL_13,
	[DRV_ADC_CH14] = ADC_CHANNEL_14,
	[DRV_ADC_CH15] = ADC_CHANNEL_15,
	[DRV_ADC_CH16] = ADC_CHANNEL_16,
	[DRV_ADC_CH17] = ADC_CHANNEL_17,
	[DRV_ADC_CH18] = ADC_CHANNEL_18,
};

/* 注入rank查找表：索引为rank值(1~4)，0号占位无效 */
static const uint32_t s_adc_injected_rank_map[] = {
	[DRV_ADC_RANK1] = ADC_INJECTED_RANK_1,
	[DRV_ADC_RANK2] = ADC_INJECTED_RANK_2,
	[DRV_ADC_RANK3] = ADC_INJECTED_RANK_3,
	[DRV_ADC_RANK4] = ADC_INJECTED_RANK_4,
};

/* 句柄转换为drv_adc.c内部使用，不对外暴露HAL类型(见drv_adc.h规则) */
static inline ADC_HandleTypeDef *get_adc_handle(adcNumber_e adcx)
{
	if (adcx >= DRV_ADC_MAX)
		return NULL;

	return s_adc_map[adcx];
}

static inline uint32_t get_adc_channel(adcChannel_e channel)
{
	if (channel >= DRV_ADC_MAX_CH)
		return 0xFFFFFFFF;

	return s_adc_channel_map[channel];
}

static inline uint32_t drv_adc_injected_get_rank(adc_injected_rank_e rank)
{
	if (rank < DRV_ADC_RANK1 || rank > DRV_ADC_RANK4)
		return 0;

	return s_adc_injected_rank_map[rank];
}

static uint32_t get_adc_mode(adcMode_e adc_mode)
{
	uint32_t mode = 0;

	switch (adc_mode)
	{
		case DRV_ADC_SINGLE_ENDED:
			mode = ADC_SINGLE_ENDED;
			break;
		case DRV_ADC_DIFFERENTIAL:
			mode = ADC_DIFFERENTIAL_ENDED;
			break;
		default:
			break;
	}
	return mode;
}

/**
 * @brief       启动ADC校准(不区分规则/注入)
 */
int drv_adc_calibration_start(adcNumber_e adcx)
{
	ADC_HandleTypeDef *handle;

	handle = get_adc_handle(adcx);
	if (handle == NULL)
	{
		return DRV_ERROR;
	}
	/* G4/L4/H7系列校准接口为双参(句柄+单端/差分)，签名跨型号一致 */
#if defined(STM32G4) || defined(STM32L4) || defined(STM32H7)
	uint32_t mode = get_adc_mode(DRV_ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(handle, mode);
#elif defined(STM32F1) || defined(STM32F3)
	/* F1/F3系列校准接口为单参 */
	HAL_ADCEx_Calibration_Start(handle);
#endif
	return DRV_EOK;
}

/**
 * @brief       启动ADC转换(规则组)
 */
int drv_adc_start(adcNumber_e adcx)
{
	ADC_HandleTypeDef *handle;

	handle = get_adc_handle(adcx);
	if (handle == NULL)
	{
		return DRV_ERROR;
	}

	HAL_ADC_Start(handle);

	return DRV_EOK;
}

/**
 * @brief       以DMA方式启动ADC转换(规则组)
 */
int drv_adc_start_dma(adcNumber_e adcx, uint32_t *pdata, uint16_t size)
{
	ADC_HandleTypeDef *handle;

	handle = get_adc_handle(adcx);
	if (handle == NULL)
	{
		return DRV_ERROR;
	}

	HAL_ADC_Start_DMA(handle, pdata, size);

	return DRV_EOK;
}

/**
 * @brief       启动ADC转换(注入组)，并使能注入转换完成中断
 */
int drv_adc_injected_start(adcNumber_e adcx)
{
	ADC_HandleTypeDef *handle;

	handle = get_adc_handle(adcx);
	if (handle == NULL)
	{
		return DRV_ERROR;
	}
	HAL_ADCEx_InjectedStart(handle);
	__HAL_ADC_ENABLE_IT(handle, ADC_IT_JEOC);

	return DRV_EOK;
}

/**
 * @brief       读取规则组转换结果
 */
uint16_t drv_adc_get_value(adcNumber_e adcx)
{
	ADC_HandleTypeDef *handle;

	handle = get_adc_handle(adcx);
	if (handle == NULL)
	{
		return 0xFFFF;
	}

	return (uint16_t)HAL_ADC_GetValue(handle);
}

/**
 * @brief       轮询等待注入组转换完成
 */
int drv_adc_injected_wait_conversion(adcNumber_e adcx, uint32_t timeout)
{
	ADC_HandleTypeDef *handle;

	handle = get_adc_handle(adcx);
	if (handle == NULL)
	{
		return DRV_ERROR;
	}

	return (int)HAL_ADCEx_InjectedPollForConversion(handle, timeout);
}

/**
 * @brief       读取注入组转换结果
 */
uint32_t drv_adc_injected_get_value(adcNumber_e adcx, adc_injected_rank_e injrank)
{
	ADC_HandleTypeDef *handle;
	uint32_t rank;

	handle = get_adc_handle(adcx);
	if (handle == NULL)
	{
		return 0xFFFF;
	}

	rank = drv_adc_injected_get_rank(injrank);
	if (rank == 0)
	{
		return 0xFFFF;
	}

	return (uint32_t)HAL_ADCEx_InjectedGetValue(handle, rank);
}

#endif /* USE_ADC_DRIVER */
