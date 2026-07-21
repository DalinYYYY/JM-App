/**
 * @file        led_manager.c
 * @brief       LED 状态映射实现（顶层状态机 → LED1/LED2 行为）
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
 */
#include "led_manager.h"
#if defined(USE_DEV_LED)

#include "dev_led.h"
#include "motor_loop.h"
#include "state_define.h"

/* LED 行为类型 */
typedef enum
{
	LED_ACT_OFF = 0,    /* 常灭 */
	LED_ACT_ON,         /* 常亮 */
	LED_ACT_BLINK_SLOW, /* 慢闪 1000ms */
	LED_ACT_BLINK_FAST, /* 快闪 200ms */
} led_action_e;

/* ==================================================================== */
/*  状态映射表: top_fsm_e → LED 行为                                       */
/* ==================================================================== */
static led_action_e map_fault_led(top_fsm_e state)
{
	switch (state)
	{
		case TOP_FSM_FAULT: return LED_ACT_ON;
		case TOP_FSM_SAFETY: return LED_ACT_BLINK_FAST;
		case TOP_FSM_BOOTLOADER: return LED_ACT_BLINK_FAST;
		default: return LED_ACT_OFF;
	}
}

static led_action_e map_status_led(top_fsm_e state)
{
	switch (state)
	{
		case TOP_FSM_INIT: return LED_ACT_BLINK_SLOW;
		case TOP_FSM_IDLE: return LED_ACT_BLINK_SLOW;
		case TOP_FSM_READY: return LED_ACT_BLINK_SLOW;
		case TOP_FSM_RUN: return LED_ACT_ON;
		case TOP_FSM_CALIB: return LED_ACT_BLINK_FAST;
		case TOP_FSM_CONFIG: return LED_ACT_BLINK_SLOW;
		case TOP_FSM_BOOTLOADER: return LED_ACT_BLINK_FAST;
		default: return LED_ACT_OFF; /* IDLE / SAFETY / FAULT */
	}
}

static void apply_action(dev_led_t *led, led_action_e act)
{
	switch (act)
	{
		case LED_ACT_OFF:
			led->set_blink(led, 0u);
			led->off(led);
			break;
		case LED_ACT_ON:
			led->set_blink(led, 0u);
			led->on(led);
			break;
		case LED_ACT_BLINK_SLOW:
			led->set_blink(led, 1000u);
			break;
		case LED_ACT_BLINK_FAST:
			led->set_blink(led, 200u);
			break;
		default:
			break;
	}
}

/* 缓存上次动作, 避免每拍重复写 GPIO（仅状态变化时才重写） */
static led_action_e s_last_act_fault = LED_ACT_OFF;
static led_action_e s_last_act_status = LED_ACT_OFF;

void led_manager_init(void)
{
	dev_led_t *led1 = dev_led_get(LED_ID_1);
	dev_led_t *led2 = dev_led_get(LED_ID_2);

	if (led1 != NULL)
	{
		dev_led_init(led1, LED_ID_1);
	}
	if (led2 != NULL)
	{
		dev_led_init(led2, LED_ID_2);
	}

	s_last_act_fault = LED_ACT_OFF;
	s_last_act_status = LED_ACT_OFF;
}

void led_manager_update(void)
{
	top_fsm_e state = motor_loop_get()->sys.top_state;
	led_action_e act_fault = map_fault_led(state);
	led_action_e act_status = map_status_led(state);
	dev_led_t *led1 = dev_led_get(LED_ID_1);
	dev_led_t *led2 = dev_led_get(LED_ID_2);

	/* 仅在动作变化时重写 GPIO */
	if (act_fault != s_last_act_fault)
	{
		if (led1 != NULL)
		{
			apply_action(led1, act_fault);
		}
		s_last_act_fault = act_fault;
	}

	if (act_status != s_last_act_status)
	{
		if (led2 != NULL)
		{
			apply_action(led2, act_status);
		}
		s_last_act_status = act_status;
	}

	/* 闪烁态由 dev_led.update 周期翻转（即使动作未变也须每拍调用） */
	if (led1 != NULL)
	{
		led1->update(led1);
	}
	if (led2 != NULL)
	{
		led2->update(led2);
	}
}

#endif /* USE_DEV_LED */
