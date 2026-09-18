/**
 * @file        led_manager.c
 * @brief       LED 状态映射实现（顶层状态机 → 灯效）
 *
 * @author      Dalin (dalinyy@163.com)
 * @version     1.1
 * @date        2026-07-21
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @par 修改日志:
 * | 日期       | 版本 | 作者   | 修改内容   |
 * |------------|------|--------|------------|
 * | 2026-07-21 | 1.0  | Dalin  | 初始创建   |
 * | 2026-09-15 | 1.1  | Dalin  | 新增 RGB 灯效 |
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 */
#include "led_manager.h"
#if defined(USE_DEV_LED) || defined(USE_DEV_RGB_LED)

#include "dev_led.h"
#include "motor_loop.h"
#include "state_define.h"
#include "fault_manager.h"
#include "main.h"

/* LED 行为类型 */
typedef enum
{
	LED_ACT_OFF = 0,    /* 常灭 */
	LED_ACT_ON,         /* 常亮 */
	LED_ACT_BLINK_SLOW, /* 慢闪 1000ms */
	LED_ACT_BLINK_FAST, /* 快闪 200ms */
} led_action_e;

#if defined(USE_DEV_LED)

/* ==================================================================== */
/*  单色 LED 路径 (V1: LED1红=故障, LED2绿=运行状态)                       */
/* ==================================================================== */

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
		default: return LED_ACT_OFF; /* SAFETY / FAULT 等: 状态灯灭(由故障灯指示) */
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
		case LED_ACT_BLINK_SLOW: led->set_blink(led, 1000u); break;
		case LED_ACT_BLINK_FAST: led->set_blink(led, 200u); break;
		default: break;
	}
}

/* 缓存上次动作, 避免每拍重复写 GPIO（仅状态变化时才重写） */
static led_action_e s_last_act_fault = LED_ACT_OFF;
static led_action_e s_last_act_status = LED_ACT_OFF;
static uint32_t s_identify_until_ms = 0u;

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
	s_identify_until_ms = 0u;
}

void led_manager_identify(uint32_t duration_ms)
{
	s_identify_until_ms = (duration_ms == 0u) ? 0u : (HAL_GetTick() + duration_ms);
	/* 强制下一拍重新下发LED动作。 */
	s_last_act_fault = (led_action_e)0xFF;
	s_last_act_status = (led_action_e)0xFF;
}

void led_manager_update(void)
{
	top_fsm_e state = motor_loop_get()->sys.top_state;
	led_action_e act_fault = map_fault_led(state);
	led_action_e act_status = map_status_led(state);
	dev_led_t *led1 = dev_led_get(LED_ID_1);
	dev_led_t *led2 = dev_led_get(LED_ID_2);
	uint32_t now_ms = HAL_GetTick();

	/* 异常级故障活动(降功率运行, 无故障级停机): LED1 慢闪提示 */
	if (act_fault == LED_ACT_OFF)
	{
		uint32_t lv = fault_mgr_level_active();
		if ((lv & 0x2u) != 0u && (lv & 0x1u) == 0u)
			act_fault = LED_ACT_BLINK_SLOW;
	}

	if (s_identify_until_ms != 0u && (int32_t)(s_identify_until_ms - now_ms) > 0)
	{
		act_fault = LED_ACT_OFF;
		act_status = LED_ACT_BLINK_FAST;
	}
	else if (s_identify_until_ms != 0u)
	{
		s_identify_until_ms = 0u;
		s_last_act_fault = (led_action_e)0xFF;
		s_last_act_status = (led_action_e)0xFF;
	}

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

#if defined(USE_DEV_RGB_LED) && !defined(USE_DEV_LED)

/* ==================================================================== */
/*  RGB LED 路径 (SFOC_V2: 单颗 RGB 合并故障+状态语义)                      */
/* ==================================================================== */

/* 状态 → (颜色, 行为) 组合 */
typedef struct
{
	rgb_color_e color;
	led_action_e act;
} rgb_pattern_t;

static rgb_pattern_t map_state_pattern(top_fsm_e state)
{
	switch (state)
	{
		case TOP_FSM_FAULT: return (rgb_pattern_t){RGB_RED, LED_ACT_ON};
		case TOP_FSM_SAFETY: return (rgb_pattern_t){RGB_RED, LED_ACT_BLINK_FAST};
		case TOP_FSM_BOOTLOADER: return (rgb_pattern_t){RGB_RED, LED_ACT_BLINK_FAST};
		case TOP_FSM_RUN: return (rgb_pattern_t){RGB_GREEN, LED_ACT_ON};
		case TOP_FSM_CALIB: return (rgb_pattern_t){RGB_BLUE, LED_ACT_BLINK_FAST};
		default: return (rgb_pattern_t){RGB_GREEN, LED_ACT_BLINK_SLOW};
	}
}

static void rgb_apply(dev_rgb_led_t *rgb, const rgb_pattern_t *pat)
{
	switch (pat->act)
	{
		case LED_ACT_OFF:
			rgb->set_blink(rgb, 0u);
			rgb->off(rgb);
			break;
		case LED_ACT_ON:
			rgb->set_blink(rgb, 0u);
			rgb->set_color(rgb, pat->color);
			break;
		case LED_ACT_BLINK_SLOW:
			rgb->set_color(rgb, pat->color);
			rgb->set_blink(rgb, 1000u);
			break;
		case LED_ACT_BLINK_FAST:
			rgb->set_color(rgb, pat->color);
			rgb->set_blink(rgb, 200u);
			break;
		default: break;
	}
}

/* 缓存上次灯效, 避免每拍重复写 GPIO（仅灯效变化时才重写） */
static rgb_pattern_t s_last_pat = {RGB_BLACK, LED_ACT_OFF};
static uint32_t s_identify_until_ms = 0u;

void led_manager_init(void)
{
	dev_rgb_led_t *rgb = dev_rgb_led_get(RGB_LED_ID_1);

	if (rgb != NULL)
	{
		dev_rgb_led_init(rgb, RGB_LED_ID_1);
	}

	s_last_pat.color = (rgb_color_e)0xFF;
	s_last_pat.act = (led_action_e)0xFF;
	s_identify_until_ms = 0u;
}

void led_manager_identify(uint32_t duration_ms)
{
	s_identify_until_ms = (duration_ms == 0u) ? 0u : (HAL_GetTick() + duration_ms);
	/* 强制下一拍重新下发LED动作。 */
	s_last_pat.color = (rgb_color_e)0xFF;
	s_last_pat.act = (led_action_e)0xFF;
}

void led_manager_update(void)
{
	top_fsm_e state = motor_loop_get()->sys.top_state;
	rgb_pattern_t pat = map_state_pattern(state);
	dev_rgb_led_t *rgb = dev_rgb_led_get(RGB_LED_ID_1);
	uint32_t now_ms = HAL_GetTick();

	/* 异常级故障活动(降功率运行, 无故障级停机): 黄色慢闪提示 */
	if (state != TOP_FSM_FAULT && state != TOP_FSM_SAFETY && state != TOP_FSM_BOOTLOADER)
	{
		uint32_t lv = fault_mgr_level_active();
		if ((lv & 0x2u) != 0u && (lv & 0x1u) == 0u)
		{
			pat.color = RGB_YELLOW;
			pat.act = LED_ACT_BLINK_SLOW;
		}
	}

	if (s_identify_until_ms != 0u && (int32_t)(s_identify_until_ms - now_ms) > 0)
	{
		/* CAN-DI 物理识别临时覆盖: 白色快闪 */
		pat.color = RGB_WHITE;
		pat.act = LED_ACT_BLINK_FAST;
	}
	else if (s_identify_until_ms != 0u)
	{
		s_identify_until_ms = 0u;
		s_last_pat.color = (rgb_color_e)0xFF;
		s_last_pat.act = (led_action_e)0xFF;
	}

	/* 仅在灯效变化时重写 GPIO */
	if (pat.color != s_last_pat.color || pat.act != s_last_pat.act)
	{
		if (rgb != NULL)
		{
			rgb_apply(rgb, &pat);
		}
		s_last_pat = pat;
	}

	/* 闪烁态由 dev_rgb_led.update 周期翻转（即使灯效未变也须每拍调用） */
	if (rgb != NULL)
	{
		rgb->update(rgb);
	}
}

#endif /* USE_DEV_RGB_LED && !USE_DEV_LED */

#endif /* USE_DEV_LED || USE_DEV_RGB_LED */
