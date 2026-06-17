/**
 * @file dev_led.h
 * @brief LED 设备：单色 LED 与 RGB LED 统一对象接口
 */
#ifndef __DEV_LED_H
#define __DEV_LED_H

#include "dev_config.h"
#if defined(USE_DEV_LED) || defined(USE_DEV_RGB_LED)

#include "drv_gpio.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

	typedef enum
	{
		LED_OFF = 0u,
		LED_ON
	} led_state_e;

	/* LED 有效电平(硬件接法) */
	typedef enum
	{
		LED_ACTIVE_LOW = 0, /* 低电平点亮 */
		LED_ACTIVE_HIGH,	/* 高电平点亮 */
	} led_polarity_e;

/* ---------------- 单色 LED ---------------- */
#if defined(USE_DEV_LED)

	typedef enum
	{
		LED_ID_1 = 0,
		LED_ID_MAX,
	} led_id_e;

	/* 资源配置 (在 device_config.c 的 led_list 填表) */
	typedef struct
	{
		char name[20];
		gpioDrv_t gpio;
		led_polarity_e polarity;
	} dev_led_config_t;

	/* 配置表定义在 device_config.c */
	extern const dev_led_config_t led_list[LED_ID_MAX];

	typedef struct dev_led
	{
		led_id_e id;
		led_state_e state;
		uint32_t blink_period_ms; /* 闪烁周期, 0 表示常亮/常灭 */

		/* public */
		void (*on)(struct dev_led *pobj);
		void (*off)(struct dev_led *pobj);
		void (*toggle)(struct dev_led *pobj);
		void (*set)(struct dev_led *pobj, led_state_e state);
		void (*set_blink)(struct dev_led *pobj, uint32_t period_ms); /* 设置闪烁周期 */
		void (*update)(struct dev_led *pobj);						 /* 周期任务调用, 处理闪烁 */
	} dev_led_t;

	void dev_led_init(dev_led_t *pobj, led_id_e id);

#endif /* USE_DEV_LED */

/* ---------------- RGB LED ---------------- */
#if defined(USE_DEV_RGB_LED)

	typedef enum
	{
		RGB_LED_ID_1 = 0,
		RGB_LED_ID_MAX,
	} rgb_led_id_e;

	/* 预设颜色 */
	typedef enum
	{
		RGB_BLACK = 0,
		RGB_RED,
		RGB_GREEN,
		RGB_BLUE,
		RGB_YELLOW,
		RGB_CYAN,
		RGB_MAGENTA,
		RGB_WHITE,
	} rgb_color_e;

	/* 资源配置 (在 device_config.c 的 rgb_led_list 填表) */
	typedef struct
	{
		char name[20];
		gpioDrv_t r;
		gpioDrv_t g;
		gpioDrv_t b;
		led_polarity_e polarity;
	} dev_rgb_led_config_t;

	/* 配置表定义在 device_config.c */
	extern const dev_rgb_led_config_t rgb_led_list[RGB_LED_ID_MAX];

	typedef struct dev_rgb_led
	{
		rgb_led_id_e id;
		uint8_t r; /* 0~255, 若为 GPIO 开关型则 0/非0 */
		uint8_t g;
		uint8_t b;

		/* public */
		void (*set_color)(struct dev_rgb_led *pobj, rgb_color_e color);				/* 预设颜色 */
		void (*set_rgb)(struct dev_rgb_led *pobj, uint8_t r, uint8_t g, uint8_t b); /* 自定义 RGB */
		void (*off)(struct dev_rgb_led *pobj);
		void (*update)(struct dev_rgb_led *pobj); /* PWM 调光/闪烁周期调用 */
	} dev_rgb_led_t;

	void dev_rgb_led_init(dev_rgb_led_t *pobj, rgb_led_id_e id);

#endif /* USE_DEV_RGB_LED */

#ifdef __cplusplus
}
#endif
#endif /* USE_DEV_LED || USE_DEV_RGB_LED */
#endif /* __DEV_LED_H */
