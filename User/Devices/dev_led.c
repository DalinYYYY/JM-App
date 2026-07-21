/**
 * @file        dev_led.c
 * @brief       单色 LED 设备对象实现（on/off/toggle/set/set_blink/update）
 *
 * @author      Dalin (dalinyy@163.com)
 * @version     1.0
 * @date        2026-07-21
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @par 修改日志:
 * | 日期       | 版本 | 作者   | 修改内容   |
 * |------------|------|--------|------------|
 * | 2026-07-21 | 1.0  | Dalin  | 初始创建   |
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 * @note        分层: dev_led(本设备对象, 粘合层) → drv_gpio(GPIO 读写)
 *              上层(led_manager)负责状态映射, 调用 dev_led_get() 取实例后操作
 */
#include "dev_led.h"
#if defined(USE_DEV_LED)

#include <stdint.h>

/* 单 LED 设备对象池（与 led_id_e 一一对应, 供 dev_led_get() 查询） */
static dev_led_t s_led_pool[LED_ID_MAX];

/* update 中每 LED 独立的闪烁计数器（避免共享计数互相干扰） */
static uint32_t s_blink_tick[LED_ID_MAX] = {0u};

/* ==================================================================== */
/*  内部实现: GPIO 写入按极性统一处理                                     */
/* ==================================================================== */
static void led_on_impl(dev_led_t *pobj)
{
	const dev_led_config_t *cfg = &led_list[pobj->id];
	if (cfg->polarity == LED_ACTIVE_HIGH)
	{
		drv_gpio_set(cfg->gpio);
	}
	else
	{
		drv_gpio_reset(cfg->gpio);
	}
	pobj->state = LED_ON;
}

static void led_off_impl(dev_led_t *pobj)
{
	const dev_led_config_t *cfg = &led_list[pobj->id];
	if (cfg->polarity == LED_ACTIVE_HIGH)
	{
		drv_gpio_reset(cfg->gpio);
	}
	else
	{
		drv_gpio_set(cfg->gpio);
	}
	pobj->state = LED_OFF;
}

static void led_toggle_impl(dev_led_t *pobj)
{
	const dev_led_config_t *cfg = &led_list[pobj->id];
	drv_gpio_toggle(cfg->gpio);
	pobj->state = (pobj->state == LED_ON) ? LED_OFF : LED_ON;
}

static void led_set_impl(dev_led_t *pobj, led_state_e state)
{
	if (state == LED_ON)
	{
		pobj->on(pobj);
	}
	else
	{
		pobj->off(pobj);
	}
}

static void led_set_blink_impl(dev_led_t *pobj, uint32_t period_ms)
{
	/* period_ms == 0 表示常亮/常灭（由 on/off 决定）, 非 0 表示闪烁周期 */
	pobj->blink_period_ms = period_ms;
	/* 进入闪烁态时清零计数器, 保证从半周期起点开始 */
	if (period_ms != 0u)
	{
		s_blink_tick[pobj->id] = 0u;
	}
}

static void led_update_impl(dev_led_t *pobj)
{
	uint32_t half_period_ticks;

	/* 非闪烁态无需周期处理 */
	if (pobj->blink_period_ms == 0u)
	{
		return;
	}

	/* thread_display 周期 10ms, 一个完整周期需两个翻转
	 * 半周期 tick 数 = (period_ms / 2) / 10 = period_ms / 20 */
	half_period_ticks = pobj->blink_period_ms / 20u;
	if (half_period_ticks == 0u)
	{
		half_period_ticks = 1u;
	}

	s_blink_tick[pobj->id]++;
	if (s_blink_tick[pobj->id] >= half_period_ticks)
	{
		s_blink_tick[pobj->id] = 0u;
		led_toggle_impl(pobj);
	}
}

/* ==================================================================== */
/*  公开接口                                                             */
/* ==================================================================== */
void dev_led_init(dev_led_t *pobj, led_id_e id)
{
	if (id >= LED_ID_MAX)
	{
		return;
	}

	pobj->id = id;
	pobj->state = LED_OFF;
	pobj->blink_period_ms = 0u;
	pobj->on = led_on_impl;
	pobj->off = led_off_impl;
	pobj->toggle = led_toggle_impl;
	pobj->set = led_set_impl;
	pobj->set_blink = led_set_blink_impl;
	pobj->update = led_update_impl;

	/* 初始常灭（按极性写入正确电平） */
	led_off_impl(pobj);
}

dev_led_t *dev_led_get(led_id_e id)
{
	if (id >= LED_ID_MAX)
	{
		return NULL;
	}
	return &s_led_pool[id];
}

#endif /* USE_DEV_LED */
