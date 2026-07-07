/**
 * @file        dev_power_monitor.c
 * @brief       电源监控设备(ADC规则组DMA采样: 母线电压/电流/温度等板级监控量)
 *
 * @author      Dalin (dalinyy@163.com)
 * @version     1.1
 * @date        2026-06-17
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @par 修改日志:
 * | 日期       | 版本 | 作者   | 修改内容                                   |
 * |------------|------|--------|--------------------------------------------|
 * | 2024-10-31 | 1.0  | Dalin  | 初始创建(dev_adc)                          |
 * | 2026-06-17 | 1.1  | Dalin  | 更名dev_power_monitor; 复用共享配置表; 去魔数 |
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 */
#include "dev_power_monitor.h"
#if defined(USE_DEV_POWER_MONITOR)

#include "assert_report.h"

dev_power_monitor_t dev_power_monitor;

/* 启动规则组DMA: 统计各ADC通道数→逐ADC校准→逐ADC启动DMA */
static int dev_power_monitor_start(struct dev_power_monitor *pobj)
{
	assert_report(pobj != NULL);
	int status = DEV_EOK;
	memset(pobj->adc_nbr, 0, sizeof(pobj->adc_nbr));

	/* 统计每个ADC上的规则通道数, 同时逐通道校准其ADC */
	for (int i = 0; i < PM_CH_MAX; i++)
	{
		adcNumber_e id = power_monitor_list[i].id;
		if (id > DRV_ADC_INIT && id < DRV_ADC_MAX)
		{
			pobj->adc_nbr[id]++;
		}
		else
		{
			assert_report(0); /* 配置表ADC编号非法 */
		}
		drv_adc_calibration_start(id);
	}

	/* 逐ADC启动DMA(仅启动有通道的ADC) */
	pobj->channel_nbr = 0;
	for (adcNumber_e id = DRV_ADC_1; id < DRV_ADC_MAX; id++)
	{
		if (pobj->adc_nbr[id] != 0)
		{
			status |= drv_adc_start_dma(id, pobj->raw[id], (pobj->adc_nbr[id] * PM_AVERAGE_LPF));
			pobj->channel_nbr += pobj->adc_nbr[id];
		}
	}
	return status;
}

/* 从DMA缓冲读取各通道ADC值(支持每通道PM_AVERAGE_LPF倍平均) */
static void dev_power_monitor_get_value(struct dev_power_monitor *pobj)
{
	assert_report(pobj != NULL);

	for (adcNumber_e id = DRV_ADC_1; id < DRV_ADC_MAX; id++)
	{
		if (pobj->adc_nbr[id] == 0)
		{
			continue;
		}
		for (int i = 0; i < pobj->adc_nbr[id]; i++)
		{
			uint32_t sum = 0;
			/* DMA按 [通道0..N-1]*LPF 顺序排布, 同通道的多次采样跨步累加 */
			for (int j = 0; j < PM_AVERAGE_LPF; j++)
			{
				sum += pobj->raw[id][i + pobj->adc_nbr[id] * j];
			}
			pobj->adc[id][i] = sum / PM_AVERAGE_LPF;
		}
	}
}

/* 各通道ADC值转采样电压(V) */
static void dev_power_monitor_get_voltage(struct dev_power_monitor *pobj)
{
	const float lsb = PM_VREF / PM_RESOLUTION;
	assert_report(pobj != NULL);

	for (adcNumber_e id = DRV_ADC_1; id < DRV_ADC_MAX; id++)
	{
		if (pobj->adc_nbr[id] == 0)
		{
			continue;
		}
		for (int i = 0; i < pobj->adc_nbr[id]; i++)
		{
			pobj->voltage[id][i] = (float)pobj->adc[id][i] * lsb;
		}
	}
}

/* 采样流程: 读DMA→转电压(物理量由各getter按需解算) */
static void dev_power_monitor_update(struct dev_power_monitor *pobj)
{
	assert_report(pobj != NULL);
	dev_power_monitor_get_value(pobj);
	dev_power_monitor_get_voltage(pobj);
}

static float dev_power_monitor_get_vbus(struct dev_power_monitor *pobj)
{
	assert_report(pobj != NULL);
	pobj->vbus = pobj->voltage[DRV_ADC_1][PM_VBUS] * PM_VBUS_RATIO;
	return pobj->vbus;
}

static float dev_power_monitor_get_ibus(struct dev_power_monitor *pobj)
{
	assert_report(pobj != NULL);
	/* INA199B1 双向检测: 零电流时输出 VREF/2, 需减去偏置再乘比例 */
	pobj->ibus = -((pobj->voltage[DRV_ADC_1][PM_IBUS] - pobj->ibus_offset) * PM_IBUS_RATIO);
	return pobj->ibus;
}

/* 温度/使能通道暂未接入配置表, 接入后按对应通道解算; 当前返回缓存值 */
static float dev_power_monitor_get_temp_driver(struct dev_power_monitor *pobj)
{
	assert_report(pobj != NULL);
	return pobj->temp_driver;
}

static float dev_power_monitor_get_temp_motor(struct dev_power_monitor *pobj)
{
	assert_report(pobj != NULL);
	return pobj->temp_motor;
}

static float dev_power_monitor_get_enpr(struct dev_power_monitor *pobj)
{
	assert_report(pobj != NULL);
	return pobj->enpr;
}

void dev_power_monitor_init(struct dev_power_monitor *pobj)
{
	assert_report(pobj != NULL);

	pobj->ibus_offset = PM_IBUS_OFFSET_V;
	pobj->start = dev_power_monitor_start;
	pobj->update = dev_power_monitor_update;
	pobj->get_vbus = dev_power_monitor_get_vbus;
	pobj->get_ibus = dev_power_monitor_get_ibus;
	pobj->get_temp_driver = dev_power_monitor_get_temp_driver;
	pobj->get_temp_motor = dev_power_monitor_get_temp_motor;
	pobj->get_enpr = dev_power_monitor_get_enpr;
}

#endif /* USE_DEV_POWER_MONITOR */
