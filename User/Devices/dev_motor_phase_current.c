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
#include "dev_motor_phase_current.h"
#if defined(USE_DEV_PHASE_CURRENT)

#include "assert_report.h"

/* 一阶低通: y = alpha*x + (1-alpha)*y_prev */
#define _lpfilter(alpha, cur_val, prev_val) ((alpha) * (cur_val) + (1.0f - (alpha)) * (prev_val))

/**
 * @brief 依据配置表为每相推导(ADC,rank)并启动注入组
 * @note  同一ADC内按通道在 phase_current_list 中的出现次序递增rank,
 *        需与CubeMX注入序列顺序一致; 每个用到的ADC只启动一次。
 */
static int dev_phase_current_start(struct dev_adc_injected *pobj)
{
	assert_report(pobj != NULL);

	uint8_t rank_cnt[DRV_ADC_MAX] = {0}; /* 各ADC已分配的rank数 */
	bool adc_used[DRV_ADC_MAX] = {0};    /* 各ADC是否被相电流占用 */
	int status = DEV_EOK;

	/* step 1: 遍历配置表, 同一ADC内按出现次序分配rank1~rankN */
	for (int i = 0; i < ADCX_INX_INJECTED_MAX; i++)
	{
#if defined(DRV8301_TWO_PHASE_CURRENT)
		/* 2-shunt: 跳过合成相, 不为其分配注入序列rank(占位通道不参与ADC转换) */
		if (i == DRV8301_TWO_PHASE_SYNTH_PHASE)
			continue;
#endif
		adcNumber_e id = phase_current_list[i].id;
		assert_report(id > DRV_ADC_INIT && id < DRV_ADC_MAX);

		pobj->src[i].id = id;
		pobj->src[i].rank = (adc_injected_rank_e)(DRV_ADC_RANK1 + rank_cnt[id]);
		rank_cnt[id]++;
		adc_used[id] = true;
	}

	/* step 2: 每个被占用的ADC仅启动一次注入组 */
	for (adcNumber_e id = DRV_ADC_1; id < DRV_ADC_MAX; id++)
	{
		if (adc_used[id])
		{
			if (drv_adc_injected_start(id) != DEV_EOK)
			{
				status = DEV_ERROR;
			}
		}
	}
	return status;
}

/* 读取三相注入组原始ADC值(按各相推导出的 id/rank) */
static void dev_phase_current_get_value(struct dev_adc_injected *pobj)
{
	assert_report(pobj != NULL);
#if defined(DRV8301_TWO_PHASE_CURRENT)
	/* 2-shunt: 仅读实测两相, 合成相由基尔霍夫定律计算(不读占位通道).
	 * DRV8301_TWO_PHASE_SYNTH_PHASE 指定合成相: 0=IA 1=IB 2=IC (预处理阶段枚举不可见, 用数值) */
#if (DRV8301_TWO_PHASE_SYNTH_PHASE == 0) /* IA 合成: 读 IB, IC */
	pobj->adc.b = drv_adc_injected_get_value(pobj->src[ADCX_IB].id, pobj->src[ADCX_IB].rank);
	pobj->adc.c = drv_adc_injected_get_value(pobj->src[ADCX_IC].id, pobj->src[ADCX_IC].rank);
	pobj->adc.a = -(pobj->adc.b + pobj->adc.c);
#elif (DRV8301_TWO_PHASE_SYNTH_PHASE == 1) /* IB 合成: 读 IA, IC */
	pobj->adc.a = drv_adc_injected_get_value(pobj->src[ADCX_IA].id, pobj->src[ADCX_IA].rank);
	pobj->adc.c = drv_adc_injected_get_value(pobj->src[ADCX_IC].id, pobj->src[ADCX_IC].rank);
	pobj->adc.b = -(pobj->adc.a + pobj->adc.c);
#elif (DRV8301_TWO_PHASE_SYNTH_PHASE == 2) /* IC 合成: 读 IA, IB */
	pobj->adc.a = drv_adc_injected_get_value(pobj->src[ADCX_IA].id, pobj->src[ADCX_IA].rank);
	pobj->adc.b = drv_adc_injected_get_value(pobj->src[ADCX_IB].id, pobj->src[ADCX_IB].rank);
	pobj->adc.c = -(pobj->adc.a + pobj->adc.b);
#else
#error "DRV8301_TWO_PHASE_SYNTH_PHASE must be 0(IA), 1(IB) or 2(IC)"
#endif
#else
	/* 3-shunt: 三相全实测 (SFOC 等板) */
	pobj->adc.a = drv_adc_injected_get_value(pobj->src[ADCX_IA].id, pobj->src[ADCX_IA].rank);
	pobj->adc.b = drv_adc_injected_get_value(pobj->src[ADCX_IB].id, pobj->src[ADCX_IB].rank);
	pobj->adc.c = drv_adc_injected_get_value(pobj->src[ADCX_IC].id, pobj->src[ADCX_IC].rank);
#endif
}

/* 原始ADC值去偏置后转为采样电压(V) */
static void dev_phase_current_get_voltage(struct dev_adc_injected *pobj)
{
	const float lsb = PHASE_CURRENT_VREF / PHASE_CURRENT_RESOLUTION; /* 每LSB对应电压 */
	assert_report(pobj != NULL);
	pobj->voltage.a = (float)(pobj->adc.a - pobj->offset.a) * lsb;
	pobj->voltage.b = (float)(pobj->adc.b - pobj->offset.b) * lsb;
	pobj->voltage.c = (float)(pobj->adc.c - pobj->offset.c) * lsb;
}

/* 采样电压经增益与采样电阻转为相电流(A) */
static dev_current_f3axis_t dev_adc_get_current(struct dev_adc_injected *pobj)
{
	dev_current_f3axis_t current;
	float k = PHASE_CURRENT_POLARITY / (pobj->gain * pobj->shunt_resistor); /* V→A 换算系数(含极性) */

	assert_report(pobj != NULL);
	current.a = pobj->voltage.a * k;
	current.b = pobj->voltage.b * k;
	current.c = pobj->voltage.c * k;
	return current;
}

/* 设置三相零电流偏置(ADC计数) */
static void dev_phase_current_set_offset(struct dev_adc_injected *pobj, dev_current_i3axis_t offset)
{
	assert_report(pobj != NULL);
	pobj->offset = offset;
}

/* 多次采样取均值标定零位(须在电机不通电、相电流为0时调用) */
static void dev_phase_current_calibrate_offset(struct dev_adc_injected *pobj, uint16_t samples)
{
	int64_t sum_a = 0, sum_b = 0, sum_c = 0;

	assert_report(pobj != NULL);
	if (samples == 0)
	{
		return;
	}
	for (uint16_t i = 0; i < samples; i++)
	{
		dev_phase_current_get_value(pobj);
		sum_a += pobj->adc.a;
		sum_b += pobj->adc.b;
		sum_c += pobj->adc.c;
	}
	pobj->offset.a = (int32_t)(sum_a / samples);
	pobj->offset.b = (int32_t)(sum_b / samples);
	pobj->offset.c = (int32_t)(sum_c / samples);
}

/* 取最新滤波三相电流 */
static dev_current_f3axis_t dev_phase_current_get_current(struct dev_adc_injected *pobj)
{
	assert_report(pobj != NULL);
	return pobj->current;
}

/* 电流采样流程: 读ADC→去偏置转电压→转电流→低通滤波(滤波状态保存在对象内) */
static void dev_phase_current_update(struct dev_adc_injected *pobj)
{
	assert_report(pobj != NULL);

	dev_phase_current_get_value(pobj);
	dev_phase_current_get_voltage(pobj);
	dev_current_f3axis_t current = dev_adc_get_current(pobj);

	pobj->current.a = _lpfilter(pobj->lpf_alpha, current.a, pobj->prev_current.a);
	pobj->current.b = _lpfilter(pobj->lpf_alpha, current.b, pobj->prev_current.b);
	pobj->current.c = _lpfilter(pobj->lpf_alpha, current.c, pobj->prev_current.c);

	pobj->prev_current = pobj->current;
}

void dev_phase_current_init(struct dev_adc_injected *pobj, float gain, float shunt_resistor)
{
	assert_report(pobj != NULL);
	pobj->gain = gain;
	pobj->shunt_resistor = shunt_resistor;
	pobj->lpf_alpha = PHASE_CURRENT_LPF_ALPHA;

	pobj->offset = (dev_current_i3axis_t){0, 0, 0};
	pobj->current = (dev_current_f3axis_t){0.0f, 0.0f, 0.0f};
	pobj->prev_current = (dev_current_f3axis_t){0.0f, 0.0f, 0.0f};

	pobj->start = dev_phase_current_start;
	pobj->update = dev_phase_current_update;
	pobj->get_current = dev_phase_current_get_current;
	pobj->set_offset = dev_phase_current_set_offset;
	pobj->calibrate_offset = dev_phase_current_calibrate_offset;
}

#endif /* USE_DEV_PHASE_CURRENT */
