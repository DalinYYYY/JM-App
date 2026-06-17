/**
 * @file        dev_half_bridge.c
 * @brief       三相半桥PWM输出设备(含互补输出与ADC注入同步触发)
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
 * | 2026-06-17 | 1.0  | Dalin  | 初始创建                                   |
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 */
#include "dev_half_bridge.h"
#if defined(USE_DEV_HALF_BRIDGE)

#include "assert_report.h"

enum
{
	PHASE_U = 0, /* U相通道下标 */
	PHASE_V,	 /* V相通道下标 */
	PHASE_W,	 /* W相通道下标 */
	ADC_TRIG,	 /* 触发ADC注入组的通道下标 */
};

/* 启动三相PWM(含互补)及ADC注入触发通道, 并缓存ARR */
static int dev_half_bridge_start(struct dev_half_bridge *pobj)
{
	assert_report(pobj != NULL);
	const dev_half_bridge_config_t *cfg = &half_bridge_list[pobj->id];
	int status = DEV_EOK;

	/* 三相主输出 + 互补输出 */
	for (int ph = PHASE_U; ph <= PHASE_W; ph++)
	{
		status |= drv_pwm_start(cfg->tim, cfg->channel[ph]);
		status |= drv_pwmN_start(cfg->tim, cfg->channel[ph]);
	}

	/* 触发ADC注入组的通道: 比较值略大于ARR, 借更新事件采样 */
	status |= drv_pwm_start(cfg->tim, cfg->channel[ADC_TRIG]);
	status |= drv_pwm_set_dutycycle(cfg->tim, cfg->channel[ADC_TRIG], HALF_BRIDGE_ADC_TRIG_CCR);

	status |= drv_tim_start_it(cfg->tim); /* 启动定时器(更新中断) */

	pobj->tim = cfg->tim;
	drv_tim_get_autoreload(cfg->tim, &pobj->autoreload); /* 缓存ARR供set_3pwm限幅 */
	return status;
}

/* 关闭三相主输出与互补输出(刹车/安全态), 保留触发通道与定时器 */
static int dev_half_bridge_stop(struct dev_half_bridge *pobj)
{
	assert_report(pobj != NULL);
	const dev_half_bridge_config_t *cfg = &half_bridge_list[pobj->id];
	int status = DEV_EOK;

	for (int ph = PHASE_U; ph <= PHASE_W; ph++)
	{
		status |= drv_pwm_stop(cfg->tim, cfg->channel[ph]);
		status |= drv_pwmN_stop(cfg->tim, cfg->channel[ph]);
	}
	return status;
}

/* 设置三相比较值(自动限幅至ARR); output_enable=0时强制三相0占空比(软急停) */
static int dev_half_bridge_set_3pwm(struct dev_half_bridge *pobj,
									uint32_t ccr1, uint32_t ccr2, uint32_t ccr3)
{
	assert_report(pobj != NULL);
	const dev_half_bridge_config_t *cfg = &half_bridge_list[pobj->id];
	uint16_t arr = pobj->autoreload; /* 用start缓存的ARR, 免去热路径HAL读取 */
	int status = DEV_EOK;

	/* 限幅至ARR */
	ccr1 = (ccr1 > arr) ? arr : ccr1;
	ccr2 = (ccr2 > arr) ? arr : ccr2;
	ccr3 = (ccr3 > arr) ? arr : ccr3;

	if (!pobj->output_enable)
	{
		ccr1 = ccr2 = ccr3 = 0;
	}

	status |= drv_pwm_set_dutycycle(cfg->tim, cfg->channel[PHASE_U], ccr1);
	status |= drv_pwm_set_dutycycle(cfg->tim, cfg->channel[PHASE_V], ccr2);
	status |= drv_pwm_set_dutycycle(cfg->tim, cfg->channel[PHASE_W], ccr3);

	pobj->ccr[PHASE_U] = ccr1;
	pobj->ccr[PHASE_V] = ccr2;
	pobj->ccr[PHASE_W] = ccr3;
	return status;
}

/* 设置输出使能(软急停开关) */
static void dev_half_bridge_set_output_enable(struct dev_half_bridge *pobj, uint8_t enable)
{
	assert_report(pobj != NULL);
	pobj->output_enable = enable ? 1u : 0u;
}

void dev_half_bridge_init(dev_half_bridge_t *pobj, half_bridge_id_e id)
{
	assert_report(pobj != NULL);
	assert_report(id < BRIDGE_ID_MAX);

	pobj->id = id;
	pobj->output_enable = 1u; /* 默认允许输出 */

	pobj->start = dev_half_bridge_start;
	pobj->stop = dev_half_bridge_stop;
	pobj->set_3pwm = dev_half_bridge_set_3pwm;
	pobj->set_output_enable = dev_half_bridge_set_output_enable;
}

#endif /* USE_DEV_HALF_BRIDGE */
