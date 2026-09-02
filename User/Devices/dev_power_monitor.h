/**
 * @file        dev_power_monitor.h
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
 * | 2026-08-28 | 1.3  | Dalin  | 实现NTC温度解算(查表+插值+钳制+LPF, V1两通道)|
 * | 2026-08-28 | 1.4  | Dalin  | NTC分度表板级可选(新增100k表, 领技CA-NTC24C018)|
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
	 * @brief 规则组监控通道类型(决定换算方式, 由板级配置表指定)
	 * @param  PM_CH_UNUSED        : 未使用/占位
	 * @param  PM_CH_VBUS          : 母线电压, y = voltage * scale (scale=分压比)
	 * @param  PM_CH_IBUS_HW       : 母线电流硬件采样, y = -(voltage - offset) * scale
	 * @param  PM_CH_IBUS_SYNTH    : 母线电流合成(不读ADC, 由 synthesize_ibus 写入)
	 * @param  PM_CH_TEMP_DRIVER   : 驱动器(PCB)温度, NTC查表换算(scale=上拉电阻kΩ, >0启用)
	 * @param  PM_CH_TEMP_MOTOR    : 电机温度, NTC查表换算(scale=上拉电阻kΩ, >0启用)
	 * @param  PM_CH_TEMP_MCU      : MCU内部温度(NTC占位, scale=上拉电阻kΩ, >0启用)
	 * @param  PM_CH_ENPR          : 使能信号(返回原始电压)
	 */
	typedef enum
	{
		PM_CH_UNUSED = 0,
		PM_CH_VBUS,
		PM_CH_IBUS_HW,
		PM_CH_IBUS_SYNTH,
		PM_CH_TEMP_DRIVER,
		PM_CH_TEMP_MOTOR,
		PM_CH_TEMP_MCU,
		PM_CH_ENPR,
		PM_CH_TYPE_MAX,
	} pm_channel_type_e;

	/* NTC 分度表描述: 阻值降序排列, 索引 i 对应温度 t_min + i*5 ℃ */
	typedef struct
	{
		const float *r_kohm; /* 阻值表(kΩ, 降序) */
		int8_t t_min;        /* 索引0对应温度(℃) */
		uint8_t size;        /* 表长 */
	} pm_ntc_table_t;

	/* 10k@25℃ 表: 与源工程 JointMotor_driver 一致, -40~125℃(34点)
	 * 100k@25℃ B=3950 表: 领技 CA-NTC24C018 规格书 Rnor 列, -30~230℃(53点) */
	extern const pm_ntc_table_t pm_ntc_table_10k;
	extern const pm_ntc_table_t pm_ntc_table_100k;

	/* PM_CH_MAX 校验在 dev_config.h 中统一管理(与其他板级必填参数一致)
	 * 通道数由板级 dev_config_board.h 决定, 支持 VBUS-only / VBUS+IBUS_HW / 全功能等组合 */
#define PM_CH_NBRS     (PM_CH_MAX)                       /* 规则组通道数 */
#define PM_AVERAGE_LPF (1)                               /* 平均滤波窗口(每通道采样倍数) */
#define PM_SAMPLE_NBRS ((PM_CH_NBRS) * (PM_AVERAGE_LPF)) /* DMA单ADC缓冲深度 */

	/**
	 * @brief 通道映射配置(在板级 dev_config_board.inc 的 power_monitor_list 填表)
	 * @param  name                   : 通道名(调试用)
	 * @param  id                     : 所属ADC外设编号(DRV_ADC_INIT表示不参与ADC采样)
	 * @param  channel                : ADC转换通道(id=DRV_ADC_INIT时无效)
	 * @param  type                   : 通道类型, 决定换算方式
	 * @param  scale                  : 线性换算系数(VBUS=分压比, IBUS_HW=1/(gain*shunt));
	 *                                   TEMP_*=NTC电路接地侧电阻(kΩ), >0 启用查表解算, <=0 保持占位
	 * @param  offset                 : 线性换算偏置(V), IBUS_HW使用PM_IBUS_OFFSET_V
	 * @param  ntc_table              : TEMP_*通道分度表(NULL=默认10k表), 线性通道不使用
	 */
	typedef struct
	{
		char name[20];
		adcNumber_e id;
		adcChannel_e channel;
		pm_channel_type_e type;
		float scale;
		float offset;
		const pm_ntc_table_t *ntc_table;
	} dev_power_monitor_config_t;

	/* 配置表定义在板级 dev_config_board.inc, 由 dev_config.c include */
	extern const dev_power_monitor_config_t power_monitor_list[PM_CH_MAX];

	/**
	 * @brief 电源监控设备对象
	 * @param  adc_nbr                : 每个ADC上的规则通道数(start统计)
	 * @param  ch_rank                : 各逻辑通道在其ADC的DMA序列中的rank(start分配)
	 *                                   有效通道=0~N-1, 无ADC配置(合成源)=0xFF
	 * @param  raw                    : 各ADC的DMA原始缓冲
	 * @param  channel_nbr            : 规则组总通道数
	 * @param  adc                    : 各通道平均后的ADC值
	 * @param  voltage                : 各通道采样电压 (单位: V)
	 * @param  vbus                   : 母线电压 (单位: V)
	 * @param  ibus                   : 母线电流 (单位: A)
	 * @param  ibus_offset            : IBUS 零电流偏置电压(V), 从IBUS_HW通道offset字段读取
	 * @param  temp_driver            : 驱动器温度 (单位: 摄氏度)
	 * @param  temp_motor             : 电机温度 (单位: 摄氏度)
	 * @param  temp_mcu               : MCU内部温度 (单位: 摄氏度)
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
		uint8_t ch_rank[PM_CH_MAX];
		uint32_t raw[DRV_ADC_MAX][PM_SAMPLE_NBRS];
		uint8_t channel_nbr;
		uint32_t adc[DRV_ADC_MAX][PM_CH_NBRS];
		float voltage[DRV_ADC_MAX][PM_CH_NBRS];

		float vbus;
		float ibus;
		float ibus_offset;
		float temp_driver;
		float temp_motor;
		float temp_mcu;
		float enpr;

		/* public */
		int (*start)(struct dev_power_monitor *pobj);
		int (*restart)(struct dev_power_monitor *pobj);
		void (*update)(struct dev_power_monitor *pobj);
		float (*get_vbus)(struct dev_power_monitor *pobj);
		float (*get_ibus)(struct dev_power_monitor *pobj);
		float (*get_temp_driver)(struct dev_power_monitor *pobj);
		float (*get_temp_motor)(struct dev_power_monitor *pobj);
		float (*get_enpr)(struct dev_power_monitor *pobj);
		/* 取指定类型通道的原始ADC值(12bit, 0~4095), 供NTC等需上层换算的通道使用
		 * @param type: 通道类型(如 PM_CH_TEMP_DRIVER/PM_CH_TEMP_MOTOR)
		 * @return 原始ADC值; 通道未配置或无ADC采样时返回0 */
		uint32_t (*get_channel_raw)(struct dev_power_monitor *pobj, pm_channel_type_e type);
	} dev_power_monitor_t;

	/**
	 * @brief  初始化电源监控对象
	 * @param   pobj             : 电源监控设备对象
	 * @note    从IBUS_HW通道offset字段读取零电流偏置; 校验VBUS通道必配
	 */
	void dev_power_monitor_init(dev_power_monitor_t *pobj);

	/**
	 * @brief  检测板级配置表是否含指定类型通道
	 * @param   type             : 通道类型
	 * @return  1=已配置, 0=未配置
	 * @note    配置表是const数组, 编译器可内联+常量折叠, 运行期无开销
	 *          供 motor_loop.c 等调用方按板级配置决定是否调用 synthesize_ibus
	 */
	int dev_power_monitor_has_channel(pm_channel_type_e type);

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
	                                       float da, float db, float dc);

	extern dev_power_monitor_t dev_power_monitor;

#ifdef __cplusplus
}
#endif
#endif /* USE_DEV_POWER_MONITOR */
#endif /* __DEV_POWER_MONITOR_H */
