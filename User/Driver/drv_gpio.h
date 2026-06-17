/**
 * @file        drv_gpio.h
 * @brief       GPIO驱动接口，封装HAL的GPIO读写/翻转/运行期重配置功能
 *
 * @author      Dalin (dalin@robot.com)
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
#ifndef _DRV_GPIO_H_
#define _DRV_GPIO_H_
#include "drv_config.h"

#ifdef USE_GPIO_DRIVER
#include "gpio.h"

/**
 * @brief IO电平状态
 * @param  DRV_PIN_LOW             : 低电平
 * @param  DRV_PIN_HIGH            : 高电平
 */
typedef enum
{
	DRV_PIN_LOW = 0u,
	DRV_PIN_HIGH
} drvPinState_e;

/**
 * @brief GPIO端口编号
 * @param  DRV_GPIO_INIT          : 无效占位(0)
 * @param  DRV_GPIOA              : 端口A
 * @param  DRV_GPIOB              : 端口B
 * @param  DRV_GPIOC              : 端口C
 * @param  DRV_GPIOD              : 端口D
 * @param  DRV_GPIOE              : 端口E(视封装而定)
 * @param  DRV_GPIOF              : 端口F(视封装而定)
 * @param  DRV_GPIOG              : 端口G(视封装而定)
 * @param  DRV_GPIO_MAX           : 端口数量边界
 */
typedef enum DRV_GPIO_TYPE
{
	DRV_GPIO_INIT = 0,
	DRV_GPIOA,
	DRV_GPIOB,
	DRV_GPIOC,
	DRV_GPIOD,
	DRV_GPIOE,
	DRV_GPIOF,
	DRV_GPIOG,

	DRV_GPIO_MAX
} gpioType_e;

/**
 * @brief GPIO引脚编号
 * @param  DRV_PIN_0              : 引脚0
 * @param  DRV_PIN_1              : 引脚1
 * @param  DRV_PIN_2              : 引脚2
 * @param  DRV_PIN_3              : 引脚3
 * @param  DRV_PIN_4              : 引脚4
 * @param  DRV_PIN_5              : 引脚5
 * @param  DRV_PIN_6              : 引脚6
 * @param  DRV_PIN_7              : 引脚7
 * @param  DRV_PIN_8              : 引脚8
 * @param  DRV_PIN_9              : 引脚9
 * @param  DRV_PIN_10             : 引脚10
 * @param  DRV_PIN_11             : 引脚11
 * @param  DRV_PIN_12             : 引脚12
 * @param  DRV_PIN_13             : 引脚13
 * @param  DRV_PIN_14             : 引脚14
 * @param  DRV_PIN_15             : 引脚15
 * @param  DRV_PIN_ALL            : 端口全部引脚
 * @param  DRV_PIN_MAX            : 引脚数量边界
 */
typedef enum
{
	DRV_PIN_0 = 0x0,
	DRV_PIN_1,
	DRV_PIN_2,
	DRV_PIN_3,
	DRV_PIN_4,
	DRV_PIN_5,
	DRV_PIN_6,
	DRV_PIN_7,
	DRV_PIN_8,
	DRV_PIN_9,
	DRV_PIN_10,
	DRV_PIN_11,
	DRV_PIN_12,
	DRV_PIN_13,
	DRV_PIN_14,
	DRV_PIN_15,
	DRV_PIN_ALL,

	DRV_PIN_MAX
} gpioPin_e;

/**
 * @brief GPIO工作模式
 * @param  DRV_INPUT             : 输入
 * @param  DRV_OUTPUT_PP         : 推挽输出
 * @param  DRV_OUTPUT_OD         : 开漏输出
 * @param  DRV_AF_PP             : 复用推挽
 * @param  DRV_AF_OD             : 复用开漏
 * @param  DRV_MODE_MAX          : 模式数量边界
 */
typedef enum
{
	DRV_INPUT = 0,
	DRV_OUTPUT_PP,
	DRV_OUTPUT_OD,
	DRV_AF_PP,
	DRV_AF_OD,
	DRV_MODE_MAX
} gpioMode_e;

/**
 * @brief GPIO上下拉配置
 * @param  DRV_NOPULL           : 无上下拉
 * @param  DRV_PULLUP           : 上拉
 * @param  DRV_PULLDOWN         : 下拉
 * @param  DRV_PULL_MAX         : 上下拉数量边界
 */
typedef enum
{
	DRV_NOPULL = 0,
	DRV_PULLUP,
	DRV_PULLDOWN,
	DRV_PULL_MAX
} gpioPull_e;

/**
 * @brief GPIO输出速率
 * @param  DRV_LOW              : 低速
 * @param  DRV_MEDIUM           : 中速
 * @param  DRV_HIGH             : 高速
 * @param  DRV_SPEED_MAX        : 速率数量边界
 */
typedef enum
{
	DRV_LOW = 0,
	DRV_MEDIUM,
	DRV_HIGH,
	DRV_SPEED_MAX
} gpioSpeed_e;

/**
 * @brief GPIO设备描述
 * @param  gpiox                 : 端口编号
 * @param  pin                   : 引脚编号
 * @param  ste                   : 引脚电平(可选,记录用)
 */
typedef struct DRV_GPIO_
{
	gpioType_e gpiox;
	gpioPin_e pin;
	drvPinState_e ste;
} gpioDrv_t;

/**
 * @brief GPIO初始化参数
 * @param  mode                  : 工作模式
 * @param  pull                  : 上下拉
 * @param  speed                 : 输出速率
 * @param  alternate             : 复用功能编号(GPIO_AFx),仅mode为DRV_AF_PP/OD时有效
 */
typedef struct DRV_GPIO_INIT
{
	gpioMode_e mode;
	gpioPull_e pull;
	gpioSpeed_e speed;
	uint8_t alternate;
} gpioInit_t;

/**
 * @brief       设置IO输出电平
 * @param        drv               : GPIO设备
 * @param        ste               : 目标电平 (范围: DRV_PIN_LOW~DRV_PIN_HIGH)
 */
void drv_gpio_write(gpioDrv_t drv, drvPinState_e ste);

/**
 * @brief       IO输出高电平
 * @param        drv               : GPIO设备
 */
void drv_gpio_set(gpioDrv_t drv);

/**
 * @brief       IO输出低电平
 * @param        drv               : GPIO设备
 */
void drv_gpio_reset(gpioDrv_t drv);

/**
 * @brief       读取IO输入电平
 * @param        drv               : GPIO设备
 * @return       : 引脚电平 (范围: DRV_PIN_LOW~DRV_PIN_HIGH), 端口无效时返回DRV_PIN_LOW
 */
drvPinState_e drv_gpio_read(gpioDrv_t drv);

/**
 * @brief       翻转IO输出电平
 * @param        drv               : GPIO设备
 */
void drv_gpio_toggle(gpioDrv_t drv);

/**
 * @brief       运行期重新配置GPIO
 * @param        drv               : GPIO设备
 * @param        init              : 初始化参数(模式/上下拉/速率/复用)
 * @note        上电初始化由CubeMX的MX_GPIO_Init完成, 此接口仅用于运行期动态重配置
 */
void drv_gpio_init(gpioDrv_t drv, gpioInit_t init);

#endif /* USE_GPIO_DRIVER */
#endif /* _DRV_GPIO_H_ */
