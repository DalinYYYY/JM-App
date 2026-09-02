/**
 * @file        dev_power_monitor.c
 * @brief       电源监控设备(ADC规则组DMA采样: 母线电压/电流/温度等板级监控量)
 *
 * @author      Dalin (dalinyy@163.com)
 * @version     1.4
 * @date        2026-08-28
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @par 修改日志:
 * | 日期       | 版本 | 作者   | 修改内容                                   |
 * |------------|------|--------|--------------------------------------------|
 * | 2024-10-31 | 1.0  | Dalin  | 初始创建(dev_adc)                          |
 * | 2026-06-17 | 1.1  | Dalin  | 更名dev_power_monitor; 复用共享配置表; 去魔数 |
 * | 2026-07-21 | 1.2  | Dalin  | 通用化改造: type字段+switch-case集中换算     |
 * | 2026-08-27 | 1.3  | Dalin  | 实现NTC温度解算(查表+插值+钳制+LPF, V1两通道)|
 * | 2026-08-28 | 1.4  | Dalin  | NTC分度表板级可选(新增100k表); 修正拓扑注释与插值段选择 |
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 */
#include "dev_power_monitor.h"
#if defined(USE_DEV_POWER_MONITOR)

#include "assert_report.h"
#include <string.h>

dev_power_monitor_t dev_power_monitor;

/* NTC 分度表(单位: kΩ, 降序), 索引 i 对应温度 t_min + i*5 ℃, 板级经 ntc_table 字段选用
 * 10k 表: 与源工程 JointMotor_driver(therimal_resitant_table)一致, -40~125℃(34点) */
static const float pm_ntc_r_10k[] = {
	195.652f,
	148.171f,
	113.347f,
	87.559f,
	68.237f,
	53.650f,
	42.506f,
	33.892f,
	27.219f,
	22.021f,
	17.926f,
	14.674f,
	12.081f,
	10.000f,
	8.315f,
	6.948f,
	5.834f,
	4.917f,
	4.161f,
	3.535f,
	3.014f,
	2.586f,
	2.228f,
	1.925f,
	1.669f,
	1.452f,
	1.268f,
	1.110f,
	0.974f,
	0.858f,
	0.758f,
	0.672f,
	0.596f,
	0.531f,
};

/* 100k 表: 领技 CA-NTC24C018 (R25=100kΩ±5%, B25/50=3950±1%) 规格书 Rnor 列,
 * -30~230℃ 为传感器工作温度范围(53点)。注: 传感器规格书优先于 B 值公式,
 * B 公式单指数在两端偏差大(-30℃ 实测 1733k vs 公式外推 2002k) */
static const float pm_ntc_r_100k[] = {
	1733.200f,
	1283.000f,
	959.050f,
	724.800f,
	551.410f,
	423.700f,
	327.240f,
	254.800f,
	199.990f,
	157.600f,
	125.245f,
	100.000f,
	81.000f,
	65.650f,
	53.500f,
	43.780f,
	35.900f,
	29.990f,
	25.000f,
	20.900f,
	17.550f,
	14.760f,
	12.540f,
	10.660f,
	9.100f,
	7.784f,
	6.710f,
	5.850f,
	5.070f,
	4.410f,
	3.850f,
	3.340f,
	2.940f,
	2.580f,
	2.271f,
	2.000f,
	1.770f,
	1.589f,
	1.414f,
	1.259f,
	1.122f,
	0.997f,
	0.896f,
	0.797f,
	0.719f,
	0.643f,
	0.582f,
	0.533f,
	0.483f,
	0.437f,
	0.396f,
	0.360f,
	0.328f,
};

const pm_ntc_table_t pm_ntc_table_10k = {
	.r_kohm = pm_ntc_r_10k,
	.t_min = -40,
	.size = (uint8_t)(sizeof(pm_ntc_r_10k) / sizeof(pm_ntc_r_10k[0])),
};

const pm_ntc_table_t pm_ntc_table_100k = {
	.r_kohm = pm_ntc_r_100k,
	.t_min = -30,
	.size = (uint8_t)(sizeof(pm_ntc_r_100k) / sizeof(pm_ntc_r_100k[0])),
};

/* NTC 采样电压 -> 温度(℃): 阻值换算 + 查表线性插值
 * 分压网络(NTC 在 VREF 侧): 3.3V -- NTC -- ADC节点 -- R_gnd(kΩ) -- GND
 * 公式还原接 VREF 一侧的 NTC 阻值: r_ntc = ((VREF - v) / v) * r_gnd
 * 区间边界钳制(传感器故障语义):
 *   v 过小(NTC 开路)或 r >= 表首 → 钳表首温度; r <= 表尾(NTC 短路/超温) → 钳表尾温度 */
static float pm_ntc_voltage_to_temp(float voltage, float r_gnd_kohm, const pm_ntc_table_t *tbl)
{
	/* 开路/未上电: 电压近零, 阻值发散, 直接钳表首温度 */
	if (voltage < 0.01f)
	{
		return (float)tbl->t_min;
	}

	float r_ntc = ((PM_VREF - voltage) / voltage) * r_gnd_kohm;

	/* 阻值 >= 表首: 温度低于表下限 (含开路), 钳表首 */
	if (r_ntc >= tbl->r_kohm[0])
	{
		return (float)tbl->t_min;
	}

	/* 表内插值: r_ntc 落在 [r_kohm[i+1], r_kohm[i]] 区间, 在该 5℃ 步进段内线性插值
	 * 条件取 r_ntc >= r_kohm[i+1] 保证插值区间正确覆盖 r_ntc(恰落表点时取表点温度) */
	for (int i = 0; i < tbl->size - 1; i++)
	{
		if (r_ntc >= tbl->r_kohm[i + 1])
		{
			float r1 = tbl->r_kohm[i];
			float r2 = tbl->r_kohm[i + 1];
			float compensate = (r1 - r_ntc) / (r1 - r2) * 5.0f;
			return (float)(tbl->t_min + i * 5) + compensate;
		}
	}

	/* 阻值 <= 表尾: 温度高于表上限 (含短路), 钳表尾 */
	return (float)(tbl->t_min + (tbl->size - 1) * 5);
}

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

/* TEMP_* 通道通用处理: NTC 查表解算 + 一阶LPF
 * scale = NTC 电路接地侧电阻(kΩ), <=0 表示板级未启用(占位通道, 保持不动)
 * ntc_table 为 NULL 时用默认 10k 表(传感器规格见 dev_power_monitor.h) */
static void pm_update_temp(float *dst, float voltage, const dev_power_monitor_config_t *cfg)
{
	if (cfg->scale <= 0.0f)
	{
		return;
	}
	const pm_ntc_table_t *tbl = (cfg->ntc_table != NULL) ? cfg->ntc_table : &pm_ntc_table_10k;
	float temp = pm_ntc_voltage_to_temp(voltage, cfg->scale, tbl);
	*dst = PM_TEMP_LPF_ALPHA * temp + (1.0f - PM_TEMP_LPF_ALPHA) * *dst;
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
			/* NTC 解算: scale>0 启用, 分度表经 ntc_table 选用(NULL=默认10k表)
			 * (SFOC_V2 等未填 scale 的板保持 0 占位, 行为不变) */
			pm_update_temp(&pobj->temp_driver, voltage, cfg);
			break;
		case PM_CH_TEMP_MOTOR:
			pm_update_temp(&pobj->temp_motor, voltage, cfg);
			break;
		case PM_CH_TEMP_MCU:
			pm_update_temp(&pobj->temp_mcu, voltage, cfg);
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

/* 重新触发规则组转换: 规则组为单次转换(ContinuousConvMode=DISABLE),
 * HAL_ADC_Start_DMA 后转一轮即停, 须每周期由 period_thread 重新软触发才能刷新 vbus/NTC。
 *
 * 关键: 这里只调 drv_adc_start(HAL_ADC_Start, 纯软触发规则组), 绝不用 stop_dma。
 *   - 规则组与注入组(相电流)共用 ADC1;
 *   - HAL_ADC_Start_DMA 已在 init 阶段置好 DMAEN 并启动 DMA循环, 此后 DMAEN 一直有效;
 *   - HAL_ADC_Start 只置 ADSTART 触发规则组一轮转换, 不 disable ADC、不碰注入组 JADSTART,
 *     采样结果仍经已挂载的 DMA 写入 raw 缓冲;
 *   - 反例: HAL_ADC_Stop_DMA 会 ADC_ConversionStop(REGULAR_INJECTED) + ADC_Disable,
 *     每周期会连注入组一起停并关整个 ADC1, 周期性打断相电流采样, 严禁使用。*/
static int dev_power_monitor_restart(struct dev_power_monitor *pobj)
{
	assert_report(pobj != NULL);
	int status = DEV_EOK;

	for (adcNumber_e id = DRV_ADC_1; id < DRV_ADC_MAX; id++)
	{
		if (pobj->adc_nbr[id] != 0)
		{
			status |= drv_adc_start(id); /* 纯规则组软触发, 不停 DMA/不 disable ADC */
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
		return pobj->vbus;         /* 板级未配置 VBUS, 返回默认值(0) */
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
	pobj->restart = dev_power_monitor_restart;
	pobj->update = dev_power_monitor_update;
	pobj->get_vbus = dev_power_monitor_get_vbus;
	pobj->get_ibus = dev_power_monitor_get_ibus;
	pobj->get_temp_driver = dev_power_monitor_get_temp_driver;
	pobj->get_temp_motor = dev_power_monitor_get_temp_motor;
	pobj->get_enpr = dev_power_monitor_get_enpr;
	pobj->get_channel_raw = dev_power_monitor_get_channel_raw;
}

#endif /* USE_DEV_POWER_MONITOR */
