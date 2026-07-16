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

/* 启动规则组DMA: 统计各ADC通道数→分配DMA rank→逐ADC校准→逐ADC启动DMA
 * 合成源通道(PM_IBUS_SOURCE=1时的PM_IBUS)无ADC配置, 跳过不参与DMA */
static int dev_power_monitor_start(struct dev_power_monitor *pobj)
{
	assert_report(pobj != NULL);
	int status = DEV_EOK;
	memset(pobj->adc_nbr, 0, sizeof(pobj->adc_nbr));
	memset(pobj->ch_rank, 0xFF, sizeof(pobj->ch_rank));

	/* 统计每个ADC上的规则通道数, 同时为每个有效通道分配DMA rank
	 * rank = 该通道在其ADC上已统计的通道序号(0~N-1), 与DMA序列顺序一致
	 * 无ADC配置的通道(id<=DRV_ADC_INIT或id>=DRV_ADC_MAX)标记0xFF跳过 */
	for (int i = 0; i < PM_CH_MAX; i++)
	{
		adcNumber_e id = power_monitor_list[i].id;
		if (id > DRV_ADC_INIT && id < DRV_ADC_MAX)
		{
			pobj->ch_rank[i] = pobj->adc_nbr[id]; /* 分配当前rank */
			pobj->adc_nbr[id]++;
			drv_adc_calibration_start(id);
		}
		else
		{
			/* 合成源或未配置通道: ch_rank 保持 0xFF, 不校准不启动DMA */
		}
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
	uint8_t rank = pobj->ch_rank[PM_VBUS];
	assert_report(rank != 0xFFU); /* VBUS 必须有硬件ADC通道 */
	pobj->vbus = pobj->voltage[DRV_ADC_1][rank] * PM_VBUS_RATIO;
	return pobj->vbus;
}

static float dev_power_monitor_get_ibus(struct dev_power_monitor *pobj)
{
	assert_report(pobj != NULL);
#if (PM_IBUS_SOURCE == 1)
	/* SYNTH 源: 由 motor_loop_isr 调用 dev_power_monitor_synthesize_ibus 写入 pobj->ibus
	 * 此处仅返回缓存值, 不再做硬件采样解算 */
	return pobj->ibus;
#else
	/* HW_ADC 源: INA199B1 双向检测, 零电流时输出 VREF/2, 需减去偏置再乘比例 */
	uint8_t rank = pobj->ch_rank[PM_IBUS];
	assert_report(rank != 0xFFU); /* IBUS 必须有硬件ADC通道 */
	pobj->ibus = -((pobj->voltage[DRV_ADC_1][rank] - pobj->ibus_offset) * PM_IBUS_RATIO);
	return pobj->ibus;
#endif
}

#if (PM_IBUS_SOURCE == 1)
/**
 * @brief 三相电流 + SVPWM 占空比合成母线电流 (SYNTH 源)
 * @details 功率守恒推导: P_in = Vbus*Ibus = Va*Ia + Vb*Ib + Vc*Ic
 *          中心对齐 SVPWM 下 Va = (2*da-1)*Vbus/2, 代入并利用 Ia+Ib+Ic=0 化简得:
 *          Ibus = da*Ia + db*Ib + dc*Ic
 *          da/db/dc 为各相上桥臂占空比 (0~1, 来自 foc.svpwm.ta/tb/tc)
 *          合成结果写入 dev_power_monitor.ibus, 供 get_ibus 读取
 * @note  调用者: motor_loop_isr 在 cur_loop_run 之后 (RUN 态, 10kHz 高频)
 *         CALIB/IDLE 态不调用, ibus 保持上次值
 *         通过 dev_config.h 的 PM_IBUS_SOURCE 宏启用 (1=合成 / 0=硬件ADC)
 */
void dev_power_monitor_synthesize_ibus(float ia, float ib, float ic,
                                       float da, float db, float dc)
{
	/* da/db/dc 已归一化为 0~1, ia/ib/ic 单位 A
	 * 流入电机为正 (与硬件 INA199B1 极性约定一致: ibus_offset 减去后取反)
	 * 调用方应传入 phase_current 原始采样值 (未经 Clarke 两相重构),
	 * 避免重构相 (占空比最大=权重最大) 噪声放大 */
	float ibus_raw = da * ia + db * ib + dc * ic;
	/* 一阶低通滤波, 抑制小电流时三相采样噪声叠加导致的波动
	 * 硬件 INA199B1 自带 RC 滤波, 合成方法无对应物, 这里数字补偿 */
	dev_power_monitor.ibus = PM_IBUS_LPF_ALPHA * ibus_raw
	                         + (1.0f - PM_IBUS_LPF_ALPHA) * dev_power_monitor.ibus;
}
#endif

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
