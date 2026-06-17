/**
 * @file        dev_power_monitor.h
 * @brief       电源监控设备(ADC规则组DMA采样: 母线电压/电流/温度等板级监控量)
 *
 * @author      Dalin (dalin@robot.com)
 * @version     1.1
 * @date        2026-06-17
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @par 修改日志:
 * | 日期       | 版本 | 作者   | 修改内容                                   |
 * |------------|------|--------|--------------------------------------------|
 * | 2026-06-11 | 1.0  | Dalin  | 初始创建(dev_adc)                          |
 * | 2026-06-17 | 1.1  | Dalin  | 更名dev_power_monitor; 复用共享配置表; 去魔数 |
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 */
#ifndef __DEV_POWER_MONITOR_H
#define __DEV_POWER_MONITOR_H

#include "dev_config.h"
#if defined(USE_DEV_POWER_MONITOR)

#include "drv_adc.h" /* adcNumber_e / adcChannel_e / DRV_ADC_MAX */
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

	/**
	 * @brief 规则组监控通道(顺序须与 dev_config.c 的 power_monitor_list 一致)
	 * @param  PM_IBUS                : 母线电流
	 * @param  PM_VBUS                : 母线电压
	 * @param  PM_CH_MAX              : 通道数量边界
	 */
	typedef enum
	{
		PM_IBUS = 0,
		PM_VBUS,
		PM_CH_MAX,
	} dev_pm_channel_e;

#define PM_CH_NBRS (PM_CH_MAX)							 /* 规则组通道数 */
#define PM_AVERAGE_LPF (1)								 /* 平均滤波窗口(每通道采样倍数) */
#define PM_SAMPLE_NBRS ((PM_CH_NBRS) * (PM_AVERAGE_LPF)) /* DMA单ADC缓冲深度 */

	/**
	 * @brief 通道映射配置(在 dev_config.c 的 power_monitor_list 填表)
	 * @param  name                   : 通道名(调试用)
	 * @param  id                     : 所属ADC外设编号
	 * @param  channel                : ADC转换通道
	 */
	typedef struct
	{
		char name[20];
		adcNumber_e id;
		adcChannel_e channel;
	} dev_power_monitor_config_t;

	/* 配置表定义在 dev_config.c */
	extern const dev_power_monitor_config_t power_monitor_list[PM_CH_MAX];

	/**
	 * @brief 电源监控设备对象
	 * @param  adc_nbr                : 每个ADC上的规则通道数(start统计)
	 * @param  raw                    : 各ADC的DMA原始缓冲
	 * @param  channel_nbr            : 规则组总通道数
	 * @param  adc                    : 各通道平均后的ADC值
	 * @param  voltage                : 各通道采样电压 (单位: V)
	 * @param  offset                 : 各通道偏置(ADC计数)
	 * @param  vbus                   : 母线电压 (单位: V)
	 * @param  ibus                   : 母线电流 (单位: A)
	 * @param  temp_driver            : 驱动器温度 (单位: 摄氏度)
	 * @param  temp_motor             : 电机温度 (单位: 摄氏度)
	 * @param  enpr                   : 使能信号采样值
	 * @param  start                  : 启动规则组DMA, 返回DEV_EOK/DEV_ERROR
	 * @param  update                 : DMA完成/周期任务调用, 刷新物理量
	 * @param  get_vbus/get_ibus      : 取母线电压/电流
	 * @param  get_temp_driver/motor  : 取驱动器/电机温度
	 * @param  get_enpr               : 取使能信号
	 */
	typedef struct dev_power_monitor
	{
		uint8_t adc_nbr[DRV_ADC_MAX];
		uint32_t raw[DRV_ADC_MAX][PM_SAMPLE_NBRS];
		uint8_t channel_nbr;
		uint32_t adc[DRV_ADC_MAX][PM_CH_NBRS];
		float voltage[DRV_ADC_MAX][PM_CH_NBRS];
		int32_t offset[PM_SAMPLE_NBRS];

		float vbus;
		float ibus;
		float temp_driver;
		float temp_motor;
		float enpr;

		/* public */
		int (*start)(struct dev_power_monitor *pobj);
		void (*update)(struct dev_power_monitor *pobj);
		float (*get_vbus)(struct dev_power_monitor *pobj);
		float (*get_ibus)(struct dev_power_monitor *pobj);
		float (*get_temp_driver)(struct dev_power_monitor *pobj);
		float (*get_temp_motor)(struct dev_power_monitor *pobj);
		float (*get_enpr)(struct dev_power_monitor *pobj);
	} dev_power_monitor_t;

	/**
	 * @brief       初始化电源监控对象
	 * @param        pobj             : 电源监控设备对象
	 */
	void dev_power_monitor_init(dev_power_monitor_t *pobj);

	extern dev_power_monitor_t dev_power_monitor;

#ifdef __cplusplus
}
#endif
#endif /* USE_DEV_POWER_MONITOR */
#endif /* __DEV_POWER_MONITOR_H */
