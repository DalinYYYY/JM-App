/**
 * @file        drv_adc.h
 * @brief       ADC驱动接口，封装HAL的ADC规则/注入/DMA转换功能
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
#ifndef DRV_ADC_H_
#define DRV_ADC_H_

#include "drv_config.h"
#ifdef USE_ADC_DRIVER
#include <stdint.h>
/* 注意：本头不include "adc.h"，对外接口不暴露HAL类型 */

/**
 * @brief ADC外设编号
 * @param  DRV_ADC_INIT          : 无效占位(0)
 * @param  DRV_ADC_1             : ADC1
 * @param  DRV_ADC_2             : ADC2
 * @param  DRV_ADC_3             : ADC3
 * @param  DRV_ADC_MAX           : 外设数量边界
 */
typedef enum
{
	DRV_ADC_INIT = 0,
	DRV_ADC_1,
	DRV_ADC_2,
	DRV_ADC_3,
	DRV_ADC_MAX
} adcNumber_e;

/**
 * @brief ADC转换通道
 * @param  DRV_ADC_CH0~CH18      : 通道0~18
 * @param  DRV_ADC_MAX_CH        : 通道数量边界
 */
typedef enum
{
	DRV_ADC_CH0 = 0,
	DRV_ADC_CH1,
	DRV_ADC_CH2,
	DRV_ADC_CH3,
	DRV_ADC_CH4,
	DRV_ADC_CH5,
	DRV_ADC_CH6,
	DRV_ADC_CH7,
	DRV_ADC_CH8,
	DRV_ADC_CH9,
	DRV_ADC_CH10,
	DRV_ADC_CH11,
	DRV_ADC_CH12,
	DRV_ADC_CH13,
	DRV_ADC_CH14,
	DRV_ADC_CH15,
	DRV_ADC_CH16,
	DRV_ADC_CH17,
	DRV_ADC_CH18,
	DRV_ADC_MAX_CH,
} adcChannel_e;

/**
 * @brief ADC注入序列rank
 * @param  DRV_ADC_RANK1         : 注入rank1
 * @param  DRV_ADC_RANK2         : 注入rank2
 * @param  DRV_ADC_RANK3         : 注入rank3
 * @param  DRV_ADC_RANK4         : 注入rank4
 */
typedef enum
{
	DRV_ADC_RANK1 = 1,
	DRV_ADC_RANK2,
	DRV_ADC_RANK3,
	DRV_ADC_RANK4,
} adc_injected_rank_e;

/**
 * @brief ADC输入模式
 * @param  DRV_ADC_SINGLE_ENDED  : 单端输入
 * @param  DRV_ADC_DIFFERENTIAL  : 差分输入
 */
typedef enum
{
	DRV_ADC_SINGLE_ENDED = 0,
	DRV_ADC_DIFFERENTIAL,
} adcMode_e;

/**
 * @brief       启动ADC校准
 * @param        adcx              : ADC外设编号
 * @return       : DRV_EOK成功，DRV_ERROR失败
 * @note         校准仅在G4/L4/H7或F1/F3系列下执行，按系列匹配HAL校准接口
 */
int drv_adc_calibration_start(adcNumber_e adcx);

/**
 * @brief       启动ADC转换(规则组)
 * @param        adcx              : ADC外设编号
 * @return       : DRV_EOK成功，DRV_ERROR失败
 */
int drv_adc_start(adcNumber_e adcx);

/**
 * @brief       启动ADC转换(注入组)，并使能注入转换完成中断
 * @param        adcx              : ADC外设编号
 * @return       : DRV_EOK成功，DRV_ERROR失败
 */
int drv_adc_injected_start(adcNumber_e adcx);

/**
 * @brief       以DMA方式启动ADC转换(规则组)
 * @param        adcx              : ADC外设编号
 * @param        pdata             : 数据存储缓冲区
 * @param        size              : 传输数据长度
 * @return       : DRV_EOK成功，DRV_ERROR失败
 */
int drv_adc_start_dma(adcNumber_e adcx, uint32_t *pdata, uint16_t size);

/**
 * @brief       读取规则组转换结果
 * @param        adcx              : ADC外设编号
 * @return       : 转换值，编号无效时返回0xFFFF
 */
uint16_t drv_adc_get_value(adcNumber_e adcx);

/**
 * @brief       读取注入组转换结果
 * @param        adcx              : ADC外设编号
 * @param        injrank           : 注入序列rank (范围: DRV_ADC_RANK1~DRV_ADC_RANK4)
 * @return       : 转换值，编号或rank无效时返回0xFFFF
 */
uint32_t drv_adc_injected_get_value(adcNumber_e adcx, adc_injected_rank_e injrank);

/**
 * @brief       轮询等待注入组转换完成
 * @param        adcx              : ADC外设编号
 * @param        timeout           : 超时时间 (单位: ms)
 * @return       : 0成功，其他失败
 */
int drv_adc_injected_wait_conversion(adcNumber_e adcx, uint32_t timeout);

#endif /* USE_ADC_DRIVER */
#endif /* DRV_ADC_H_ */
