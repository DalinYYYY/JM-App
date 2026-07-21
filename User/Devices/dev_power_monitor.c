/**
 * @file        dev_power_monitor.c
 * @brief       电源监控设备(ADC规则组DMA采样: 母线电压/电流/温度等板级监控量)
 *
 * @author      Dalin (dalinyy@163.com)
 * @version     1.2
 * @date        2026-07-21
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @par 修改日志:
 * | 日期       | 版本 | 作者   | 修改内容                                   |
 * |------------|------|--------|--------------------------------------------|
 * | 2024-10-31 | 1.0  | Dalin  | 初始创建(dev_adc)                          |
 * | 2026-06-17 | 1.1  | Dalin  | 更名dev_power_monitor; 复用共享配置表; 去魔数 |
 * | 2026-07-21 | 1.2  | Dalin  | 通用化改造: type字段+switch-case集中换算     |
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 */
#include "dev_power_monitor.h"
#if defined(USE_DEV_POWER_MONITOR)

#include "assert_report.h"
#include <string.h>

dev_power_monitor_t dev_power_monitor;

/* 按 type 查找通道在 power_monitor_list 中的索引, 未配置返回 -1
 * 配置表是 const 数组, 编译器可内联+常量折叠, 运行期无开销 */
static int pm_find_channel(pm_channel_type_e type)
{
	for (int i = 0; i < PM_CH_MAX; i++)
	{
		if (power_monitor_list[i].type == type)
		{
			return i;
		}
	}
	return -1;
}

/* 各通道电压值按 type 换算为物理量, 写入对象对应字段
 * switch-case 集中处理, 避免分散的换算逻辑(用户已确认不用函数指针) */
static void pm_convert_channel(struct dev_power_monitor *pobj, int idx)
{
	const dev_power_monitor_config_t *cfg = &power_monitor_list[idx];
	float voltage = 0.0f;

	/* 仅对有 ADC 配置的通道读取采样电压(SYNTH/UNUSED 类型 id=DRV_ADC_INIT 跳过) */
	if (cfg->id > DRV_ADC_INIT && cfg->id < DRV_ADC_MAX)
	{
		uint8_t rank = pobj->ch_rank[idx];
		if (rank == 0xFFU)
		{
			return;
		}
		voltage = pobj->voltage[cfg->id][rank];
	}

	switch (cfg->type)
	{
	case PM_CH_VBUS:
		/* 母线电压: 采样电压 * 分压比 */
		pobj->vbus = voltage * cfg->scale;
		break;
	case PM_CH_IBUS_HW:
		/* INA199B1 双向: 零电流输出 VREF/2, 流入电机为正需取反 */
		pobj->ibus = -(voltage - pobj->ibus_offset) * cfg->scale;
		break;
	case PM_CH_IBUS_SYNTH:
		/* 合成源: 由 motor_loop_isr 调用 synthesize_ibus 写入, 此处不动 */
		break;
	case PM_CH_TEMP_DRIVER:
		/* NTC 占位: 硬件采样已启动, 计算逻辑待实现(Beta公式或查表法) */
		pobj->temp_driver = 0.0f;
		break;
	case PM_CH_TEMP_MOTOR:
		pobj->temp_motor = 0.0f;
		break;
	case PM_CH_TEMP_MCU:
		pobj->temp_mcu = 0.0f;
		break;
	case PM_CH_ENPR:
		/* 使能信号: 返回原始电压, 调用方按阈值判断 */
		pobj->enpr = voltage;
		break;
	default:
		/* PM_CH_UNUSED 等: 不处理 */
		break;
	}
}

/* 启动规则组DMA: 统计各ADC通道数→分配DMA rank→逐ADC校准→逐ADC启动DMA
 * 合成源/未使用通道(id=DRV_ADC_INIT)无ADC配置, 跳过不参与DMA */
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
			/* 合成源/未使用通道: ch_rank 保持 0xFF, 不校准不启动DMA */
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

/* 采样流程: 读DMA→转电压→按type集中换算物理量 */
static void dev_power_monitor_update(struct dev_power_monitor *pobj)
{
	assert_report(pobj != NULL);
	dev_power_monitor_get_value(pobj);
	dev_power_monitor_get_voltage(pobj);
	for (int i = 0; i < PM_CH_MAX; i++)
	{
		pm_convert_channel(pobj, i);
	}
}

static float dev_power_monitor_get_vbus(struct dev_power_monitor *pobj)
{
	assert_report(pobj != NULL);
	int idx = pm_find_channel(PM_CH_VBUS);
	if (idx < 0)
	{
		return pobj->vbus; /* 板级未配置 VBUS, 返回默认值(0) */
	}
	pm_convert_channel(pobj, idx); /* 现场换算(供 HW_ADC 源实时刷新) */
	return pobj->vbus;
}

static float dev_power_monitor_get_ibus(struct dev_power_monitor *pobj)
{
	assert_report(pobj != NULL);
	int idx_hw = pm_find_channel(PM_CH_IBUS_HW);
	if (idx_hw >= 0)
	{
		/* HW_ADC 源: 现场采样换算 */
		pm_convert_channel(pobj, idx_hw);
	}
	/* SYNTH 源: 由 motor_loop_isr 调用 synthesize_ibus 写入, 此处仅返回缓存 */
	return pobj->ibus;
}

static float dev_power_monitor_get_temp_driver(struct dev_power_monitor *pobj)
{
	assert_report(pobj != NULL);
	int idx = pm_find_channel(PM_CH_TEMP_DRIVER);
	if (idx >= 0)
	{
		pm_convert_channel(pobj, idx);
	}
	return pobj->temp_driver;
}

static float dev_power_monitor_get_temp_motor(struct dev_power_monitor *pobj)
{
	assert_report(pobj != NULL);
	int idx = pm_find_channel(PM_CH_TEMP_MOTOR);
	if (idx >= 0)
	{
		pm_convert_channel(pobj, idx);
	}
	return pobj->temp_motor;
}

static float dev_power_monitor_get_enpr(struct dev_power_monitor *pobj)
{
	assert_report(pobj != NULL);
	int idx = pm_find_channel(PM_CH_ENPR);
	if (idx >= 0)
	{
		pm_convert_channel(pobj, idx);
	}
	return pobj->enpr;
}

/* 取指定类型通道的原始ADC值(12bit, 0~4095)
 * 供NTC等需上层换算的通道使用, 避免驱动层硬编码温度计算公式
 * 通道未配置或无ADC采样(合成源/未使用)时返回0 */
static uint32_t dev_power_monitor_get_channel_raw(struct dev_power_monitor *pobj, pm_channel_type_e type)
{
	assert_report(pobj != NULL);
	int idx = pm_find_channel(type);
	if (idx < 0)
	{
		return 0U; /* 板级未配置该类型通道 */
	}
	const dev_power_monitor_config_t *cfg = &power_monitor_list[idx];
	if (cfg->id <= DRV_ADC_INIT || cfg->id >= DRV_ADC_MAX)
	{
		return 0U; /* 无ADC采样(合成源/未使用) */
	}
	uint8_t rank = pobj->ch_rank[idx];
	if (rank == 0xFFU)
	{
		return 0U; /* DMA未分配rank(异常) */
	}
	return pobj->adc[cfg->id][rank];
}

int dev_power_monitor_has_channel(pm_channel_type_e type)
{
	return pm_find_channel(type) >= 0;
}

/**
 * @brief 三相电流 + SVPWM 占空比合成母线电流 (SYNTH 源)
 * @details 功率守恒推导: P_in = Vbus*Ibus = Va*Ia + Vb*Ib + Vc*Ic
 *          中心对齐 SVPWM 下 Va = (2*da-1)*Vbus/2, 代入并利用 Ia+Ib+Ic=0 化简得:
 *          Ibus = da*Ia + db*Ib + dc*Ic
 *          da/db/dc 为各相上桥臂占空比 (0~1, 来自 foc.svpwm.ta/tb/tc)
 *          合成结果写入 dev_power_monitor.ibus, 供 get_ibus 读取
 * @note  调用者: motor_loop_isr 在 cur_loop_run 之后 (RUN 态, 10kHz 高频)
 *         仅板级配置了 PM_CH_IBUS_SYNTH 通道时调用 (由 has_channel 检测)
 *         CALIB/IDLE 态不调用, ibus 保持上次值
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

void dev_power_monitor_init(dev_power_monitor_t *pobj)
{
	assert_report(pobj != NULL);

	memset(pobj, 0, sizeof(*pobj));
	memset(pobj->ch_rank, 0xFF, sizeof(pobj->ch_rank));

	/* VBUS 通道必配校验(SVPWM 归一化依赖, 缺失会导致除零/电压失真) */
	assert_report(pm_find_channel(PM_CH_VBUS) >= 0);

	/* 从 IBUS_HW 通道 offset 字段读取零电流偏置; 无 IBUS_HW 时用默认值 */
	int idx_ibus = pm_find_channel(PM_CH_IBUS_HW);
	pobj->ibus_offset = (idx_ibus >= 0) ? power_monitor_list[idx_ibus].offset : PM_IBUS_OFFSET_V;

	pobj->start = dev_power_monitor_start;
	pobj->update = dev_power_monitor_update;
	pobj->get_vbus = dev_power_monitor_get_vbus;
	pobj->get_ibus = dev_power_monitor_get_ibus;
	pobj->get_temp_driver = dev_power_monitor_get_temp_driver;
	pobj->get_temp_motor = dev_power_monitor_get_temp_motor;
	pobj->get_enpr = dev_power_monitor_get_enpr;
	pobj->get_channel_raw = dev_power_monitor_get_channel_raw;
}

#endif /* USE_DEV_POWER_MONITOR */
