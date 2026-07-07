/**
 * @file        drv_gpio.c
 * @brief       GPIO驱动实现，封装HAL的GPIO读写/翻转/运行期重配置功能
 *
 * @author      Dalin (dalinyy@163.com)
 * @version     1.0
 * @date        2026-06-15
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @par 修改日志:
 * | 日期       | 版本 | 作者   | 修改内容   |
 * |------------|------|--------|------------|
 * | 2026-06-15 | 1.0  | Dalin  | 初始创建   |
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 */
#include "drv_gpio.h"

#ifdef USE_GPIO_DRIVER
#include "gpio.h"

/* GPIO端口查找表：以gpioType_e为索引，O(1)定位HAL端口基址。
 * 未编译进的端口(如该封装无GPIOF/G)对应项为NULL。 */
static GPIO_TypeDef *const s_gpio_map[DRV_GPIO_MAX] = {
	[DRV_GPIOA] = GPIOA,
	[DRV_GPIOB] = GPIOB,
	[DRV_GPIOC] = GPIOC,
	[DRV_GPIOD] = GPIOD,
#ifdef GPIOE
	[DRV_GPIOE] = GPIOE,
#endif
#ifdef GPIOF
	[DRV_GPIOF] = GPIOF,
#endif
#ifdef GPIOG
	[DRV_GPIOG] = GPIOG,
#endif
};

static inline GPIO_TypeDef *get_gpio_type(gpioType_e gpiox)
{
	if (gpiox >= DRV_GPIO_MAX)
		return NULL;

	return s_gpio_map[gpiox];
}

static inline uint16_t get_gpio_pin(gpioPin_e pin)
{
	/* HAL中 GPIO_PIN_x == (1u << x)，DRV_PIN_x 与 x 一一对应，直接移位即可。 */
	if (pin >= DRV_PIN_ALL)
		return GPIO_PIN_All;

	return (uint16_t)(1u << pin);
}

static uint32_t get_gpio_mode(gpioMode_e mode)
{
	uint32_t mode_ = 0;

	switch (mode)
	{
		case DRV_INPUT:
			mode_ = GPIO_MODE_INPUT;
			break;
		case DRV_OUTPUT_PP:
			mode_ = GPIO_MODE_OUTPUT_PP;
			break;
		case DRV_OUTPUT_OD:
			mode_ = GPIO_MODE_OUTPUT_OD;
			break;
		case DRV_AF_PP:
			mode_ = GPIO_MODE_AF_PP;
			break;
		case DRV_AF_OD:
			mode_ = GPIO_MODE_AF_OD;
			break;
		default:
			break;
	}

	return mode_;
}

static uint32_t get_gpio_pull(gpioPull_e pull)
{
	uint32_t pull_ = 0;

	switch (pull)
	{
		case DRV_NOPULL:
			pull_ = GPIO_NOPULL;
			break;
		case DRV_PULLUP:
			pull_ = GPIO_PULLUP;
			break;
		case DRV_PULLDOWN:
			pull_ = GPIO_PULLDOWN;
			break;
		default:
			break;
	}

	return pull_;
}

static uint32_t get_gpio_speed(gpioSpeed_e speed)
{
	uint32_t speed_ = 0;

	switch (speed)
	{
		case DRV_LOW:
			speed_ = GPIO_SPEED_FREQ_LOW;
			break;
		case DRV_MEDIUM:
			speed_ = GPIO_SPEED_FREQ_MEDIUM;
			break;
		case DRV_HIGH:
			speed_ = GPIO_SPEED_FREQ_HIGH;
			break;
		default:
			break;
	}

	return speed_;
}

/**
 * @brief       运行期重新配置GPIO
 */
void drv_gpio_init(gpioDrv_t drv, gpioInit_t init)
{
	GPIO_InitTypeDef gpio_init = {0};
	GPIO_TypeDef *gpio;
	uint16_t pin;
	uint32_t mode, pull, speed;

	gpio = get_gpio_type(drv.gpiox);
	if (gpio == NULL)
		return;

	pin = get_gpio_pin(drv.pin);
	mode = get_gpio_mode(init.mode);
	pull = get_gpio_pull(init.pull);
	speed = get_gpio_speed(init.speed);

	gpio_init.Pin = pin;
	gpio_init.Mode = mode;
	gpio_init.Pull = pull;
	gpio_init.Speed = speed;
	/* 仅复用模式需要指定AF编号 */
	if (init.mode == DRV_AF_PP || init.mode == DRV_AF_OD)
	{
		gpio_init.Alternate = init.alternate;
	}
	HAL_GPIO_Init(gpio, &gpio_init);
}

void drv_gpio_write(gpioDrv_t drv, drvPinState_e ste)
{
	GPIO_TypeDef *gpio;
	uint16_t pin;

	gpio = get_gpio_type(drv.gpiox);
	if (gpio == NULL)
		return;

	pin = get_gpio_pin(drv.pin);

	HAL_GPIO_WritePin(gpio, pin, (GPIO_PinState)ste);
}

void drv_gpio_set(gpioDrv_t drv)
{
	GPIO_TypeDef *gpio;
	uint16_t pin;

	gpio = get_gpio_type(drv.gpiox);
	if (gpio == NULL)
		return;

	pin = get_gpio_pin(drv.pin);

	HAL_GPIO_WritePin(gpio, pin, (GPIO_PinState)DRV_PIN_HIGH);
}

void drv_gpio_reset(gpioDrv_t drv)
{
	GPIO_TypeDef *gpio;
	uint16_t pin;

	gpio = get_gpio_type(drv.gpiox);
	if (gpio == NULL)
		return;

	pin = get_gpio_pin(drv.pin);

	HAL_GPIO_WritePin(gpio, pin, (GPIO_PinState)DRV_PIN_LOW);
}

drvPinState_e drv_gpio_read(gpioDrv_t drv)
{
	GPIO_TypeDef *gpio;
	uint16_t pin;

	gpio = get_gpio_type(drv.gpiox);
	if (gpio == NULL)
		return DRV_PIN_LOW;

	pin = get_gpio_pin(drv.pin);

	return (drvPinState_e)(HAL_GPIO_ReadPin(gpio, pin));
}

void drv_gpio_toggle(gpioDrv_t drv)
{
	GPIO_TypeDef *gpio;
	uint16_t pin;

	gpio = get_gpio_type(drv.gpiox);
	if (gpio == NULL)
		return;

	pin = get_gpio_pin(drv.pin);

	HAL_GPIO_TogglePin(gpio, pin);
}

#endif /* USE_GPIO_DRIVER */
