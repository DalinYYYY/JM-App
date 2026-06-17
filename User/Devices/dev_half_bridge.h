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
#ifndef __DEV_HALF_BRIDGE_H
#define __DEV_HALF_BRIDGE_H

#include "dev_config.h"
#if defined(USE_DEV_HALF_BRIDGE)

#include "drv_gpio.h"
#include "drv_tim.h"
#include "drv_tim_pwm.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

	/**
	 * @brief 半桥桥臂电平状态
	 * @param  BRIDGE_LOW             : 低电平
	 * @param  BRIDGE_HIGH            : 高电平
	 */
	typedef enum
	{
		BRIDGE_LOW = 0u,
		BRIDGE_HIGH
	} half_bridge_state_e;

	/**
	 * @brief 半桥设备编号
	 * @param  BRIDGE_DEV1            : 半桥1
	 * @param  BRIDGE_ID_MAX          : 设备数量边界
	 */
	typedef enum
	{
		BRIDGE_DEV1 = 0,
		BRIDGE_ID_MAX,
	} half_bridge_id_e;

	/**
	 * @brief 半桥资源配置(在 dev_config.c 的 half_bridge_list 填表)
	 * @param  name                   : 设备名(调试用)
	 * @param  tim                    : 所属定时器编号
	 * @param  channel                : U/V/W三相 + 触发ADC注入组的通道
	 */
	typedef struct
	{
		char name[20];
		timNumber_e tim;
		timChannel_e channel[4];
	} dev_half_bridge_config_t;

	/* 配置表定义在 dev_config.c */
	extern const dev_half_bridge_config_t half_bridge_list[BRIDGE_ID_MAX];

	/**
	 * @brief 三相半桥PWM输出设备对象
	 * @param  id                     : 半桥设备编号
	 * @param  tim                    : 所属定时器编号(start后缓存)
	 * @param  autoreload             : PWM计数周期ARR(start后缓存, 用于占空比限幅)
	 * @param  output_enable          : 输出使能(0=强制三相0占空比, 用于软急停)
	 * @param  ccr                    : 各通道最近一次比较值
	 * @param  start                  : 启动三相PWM(含互补)及注入触发, 返回DEV_EOK/DEV_ERROR
	 * @param  stop                   : 关闭三相PWM输出(刹车/安全态), 返回DEV_EOK/DEV_ERROR
	 * @param  set_3pwm               : 设置三相比较值(自动限幅至ARR), 返回DEV_EOK/DEV_ERROR
	 * @param  set_output_enable      : 设置输出使能(软急停开关)
	 */
	typedef struct dev_half_bridge
	{
		half_bridge_id_e id;
		timNumber_e tim;
		uint16_t autoreload;
		uint8_t output_enable;
		uint32_t ccr[4];

		/* public */
		int (*start)(struct dev_half_bridge *pobj);
		int (*stop)(struct dev_half_bridge *pobj);
		int (*set_3pwm)(struct dev_half_bridge *pobj,
						uint32_t ccr1, uint32_t ccr2, uint32_t ccr3);
		void (*set_output_enable)(struct dev_half_bridge *pobj, uint8_t enable);
	} dev_half_bridge_t;

	/**
	 * @brief       初始化半桥对象
	 * @param        pobj             : 半桥设备对象
	 * @param        id               : 半桥设备编号 (范围: 0~BRIDGE_ID_MAX-1)
	 * @note         init只装配接口与参数; PWM实际启动在 start 中完成
	 */
	void dev_half_bridge_init(dev_half_bridge_t *pobj, half_bridge_id_e id);

#ifdef __cplusplus
}
#endif
#endif /* USE_DEV_HALF_BRIDGE */
#endif /* __DEV_HALF_BRIDGE_H */
