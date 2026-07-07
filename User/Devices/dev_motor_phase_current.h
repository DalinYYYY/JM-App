/**
 * @file        dev_motor_phase_current.h
 * @brief       电机三相相电流采样设备(ADC注入组, 与PWM同步触发)
 *
 * @author      Dalin (dalinyy@163.com)
 * @version     1.0
 * @date        2026-06-17
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @par 修改日志:
 * | 日期       | 版本 | 作者   | 修改内容                                   |
 * |------------|------|--------|--------------------------------------------|
 * | 2026-06-16 | 1.0  | Dalin  | 初始创建                                   |
 * | 2026-06-17 | 1.1  | Dalin  | start按配置表推导(ADC,rank); 滤波状态对象化; 增加取值/标定接口 |
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 */
#ifndef __DEV_MOTOR_PHASE_CURRENT_H
#define __DEV_MOTOR_PHASE_CURRENT_H

#include "dev_config.h"
#if defined(USE_DEV_PHASE_CURRENT)

#include "drv_adc.h" /* adcNumber_e / adcChannel_e / adc_injected_rank_e */
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

	/**
	 * @brief 注入组相电流通道枚举(顺序须与 dev_config.c 的 phase_current_list 一致)
	 * @param  ADCX_IA                : A相电流
	 * @param  ADCX_IB                : B相电流
	 * @param  ADCX_IC                : C相电流
	 * @param  ADCX_INX_INJECTED_MAX  : 相数边界
	 */
	typedef enum
	{
		ADCX_IA = 0,
		ADCX_IB,
		ADCX_IC,
		ADCX_INX_INJECTED_MAX,
	} dev_injected_obj_e;

	/**
	 * @brief 通道映射配置(在 dev_config.c 的 phase_current_list 填表)
	 * @param  name                   : 通道名(调试用)
	 * @param  id                     : 所属ADC外设编号
	 * @param  channel                : ADC转换通道
	 */
	typedef struct
	{
		char name[20];
		adcNumber_e id;
		adcChannel_e channel;
	} dev_phase_current_config_t;

	/* 配置表定义在 dev_config.c */
	extern const dev_phase_current_config_t phase_current_list[ADCX_INX_INJECTED_MAX];

	/**
	 * @brief 三相整型量(ADC原始值/偏置)
	 * @param  a/b/c                   : A/B/C相
	 */
	typedef struct
	{
		int32_t a, b, c;
	} dev_current_i3axis_t;

	/**
	 * @brief 三相浮点量(电压/电流)
	 * @param  a/b/c                   : A/B/C相
	 */
	typedef struct
	{
		float a, b, c;
	} dev_current_f3axis_t;

	/**
	 * @brief 单相的注入组采样源(由 start 依据配置表推导)
	 * @param  id                     : 所属ADC外设编号
	 * @param  rank                   : 该相在注入序列中的rank
	 */
	typedef struct
	{
		adcNumber_e id;
		adc_injected_rank_e rank;
	} dev_phase_source_t;

	/**
	 * @brief 三相相电流采样设备对象
	 * @param  src                    : 各相的(ADC,rank)映射, start时按配置表推导
	 * @param  gain                   : 电流采样放大器增益
	 * @param  shunt_resistor         : 采样电阻 (单位: 欧姆)
	 * @param  lpf_alpha              : 一阶低通滤波系数 (范围: 0~1, 越大越跟随)
	 * @param  adc                    : 三相原始ADC值
	 * @param  offset                 : 三相零电流偏置(ADC计数)
	 * @param  voltage                : 三相采样电压 (单位: V)
	 * @param  current                : 三相滤波后电流 (单位: A)
	 * @param  prev_current           : 上一拍滤波电流(对象内滤波状态, 保证多实例可重入)
	 * @param  start                  : 依据配置表推导(ADC,rank)并启动注入组
	 * @param  update                 : 刷新三相电流(注入完成中断/控制周期调用)
	 * @param  get_current            : 取最新滤波三相电流
	 * @param  set_offset             : 设置零电流偏置
	 * @param  calibrate_offset       : 多次采样取均值标定零位(须在电机不通电时调用)
	 */
	typedef struct dev_adc_injected
	{
		dev_phase_source_t src[ADCX_INX_INJECTED_MAX];
		float gain;
		float shunt_resistor;
		float lpf_alpha;
		dev_current_i3axis_t adc;
		dev_current_i3axis_t offset;
		dev_current_f3axis_t voltage;
		dev_current_f3axis_t current;
		dev_current_f3axis_t prev_current;

		/* public */
		int (*start)(struct dev_adc_injected *pobj);
		void (*update)(struct dev_adc_injected *pobj);
		dev_current_f3axis_t (*get_current)(struct dev_adc_injected *pobj);
		void (*set_offset)(struct dev_adc_injected *pobj, dev_current_i3axis_t offset);
		void (*calibrate_offset)(struct dev_adc_injected *pobj, uint16_t samples);
	} dev_phase_current_t;

	/**
	 * @brief       初始化注入组相电流采样对象
	 * @param        pobj             : 相电流设备对象
	 * @param        gain             : 放大器增益
	 * @param        shunt_resistor   : 采样电阻阻值 (单位: 欧姆)
	 * @note         init只装配接口与参数; 注入组的实际启动在 start 中完成
	 */
	void dev_phase_current_init(dev_phase_current_t *pobj, float gain, float shunt_resistor);

#ifdef __cplusplus
}
#endif
#endif /* USE_DEV_PHASE_CURRENT */
#endif /* __DEV_MOTOR_PHASE_CURRENT_H */
