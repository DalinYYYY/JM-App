/**
 * @file        dev_commun_vesc.h
 * @brief       VESC Tool 串口通信设备(USART+DMA空闲中断, 把本机伪装成VESC从机)
 *
 * @author      Dalin (dalin@robot.com)
 * @version     1.0
 * @date        2026-06-18
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @par 修改日志:
 * | 日期       | 版本 | 作者   | 修改内容   |
 * |------------|------|--------|------------|
 * | 2026-06-18 | 1.0  | Dalin  | 初始创建   |
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 * @note        设备层封装: 上接 vesc_slave 协议栈, 下接 drv_usart 的 DMA+空闲中断收发。
 *              VESC Tool 连接流程:
 *                1. VESC Tool 打开串口发 COMM_FW_VERSION → 本设备回复身份 → 被识别;
 *                2. VESC Tool 周期请求 COMM_GET_VALUES → 本设备回填实时值 → 仪表盘显示。
 *              使用方式:
 *                - 上电后调用 start() 启动空闲中断+DMA接收;
 *                - 在对应串口 IRQ 中调用 on_rx_idle()(检测IDLE并落数据);
 *                - 在主循环/线程周期调用 poll()(取数据喂协议栈, 自动回复);
 *                - 通过 set_values_cb() 注入仪表盘实时值数据源。
 */
#ifndef __DEV_COMMUN_VESC_H
#define __DEV_COMMUN_VESC_H

#include "dev_config.h"
#if defined(USE_DEV_COMMUN_VESC)

#include "drv_usart.h" /* usartNumber_e / 空闲中断收发接口 */
#include "vesc_slave.h" /* vesc_slave_t / vesc_values_t */
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

	/**
	 * @brief VESC 通信实例编号(顺序须与 dev_config.c 的 commun_vesc_list 一致)
	 * @param  VESC_COMM_ID_1         : 第一路 VESC 通信
	 * @param  VESC_COMM_ID_MAX       : 实例数量边界
	 */
	typedef enum
	{
		VESC_COMM_ID_1 = 0,
		VESC_COMM_ID_MAX,
	} vesc_comm_id_e;

	/**
	 * @brief VESC 通信设备配置(在 dev_config.c 的 commun_vesc_list 填表)
	 * @param  name                   : 实例名(调试用)
	 * @param  uart                   : 所属串口编号(须为已配置 DMA 的串口)
	 * @param  hw_name                : 硬件名称(显示在 VESC Tool)
	 * @param  fw_name                : 固件名称字符串
	 * @param  fw_major               : 固件主版本号
	 * @param  fw_minor               : 固件次版本号
	 */
	typedef struct
	{
		char name[20];
		usartNumber_e uart;
		const char *hw_name;
		const char *fw_name;
		uint8_t fw_major;
		uint8_t fw_minor;
	} dev_commun_vesc_config_t;

	/* 配置表定义在 dev_config.c */
	extern const dev_commun_vesc_config_t commun_vesc_list[VESC_COMM_ID_MAX];

	/**
	 * @brief VESC 通信设备对象
	 * @param  slave                  : VESC 从机协议栈实例(私有)
	 * @param  uart                   : 所属串口编号(私有)
	 * @param  started                : 接收已启动标志(私有)
	 * @param  rx_tmp                  : 空闲突发取数临时缓冲(私有)
	 * @param  fill_values            : 仪表盘实时值数据源回调(应用注入, 可为 NULL)
	 * @param  start                  : 启动空闲中断+DMA接收, 返回 DEV_EOK/DEV_ERROR
	 * @param  on_rx_idle             : 串口 IRQ 中调用, 检测 IDLE 并落本帧数据
	 * @param  poll                   : 主循环/线程周期调用, 取数据喂协议栈并自动回复
	 * @param  set_values_cb          : 注入仪表盘实时值数据源回调
	 */
	typedef struct dev_commun_vesc
	{
		vesc_slave_t slave;
		usartNumber_e uart;
		uint8_t started;
		uint8_t rx_tmp[DEV_VESC_RX_BUF_SIZE];

		/* public */
		void (*fill_values)(vesc_values_t *v);

		int (*start)(struct dev_commun_vesc *pobj);
		void (*on_rx_idle)(struct dev_commun_vesc *pobj);
		void (*poll)(struct dev_commun_vesc *pobj);
		void (*set_values_cb)(struct dev_commun_vesc *pobj, void (*cb)(vesc_values_t *v));
	} dev_commun_vesc_t;

	/**
	 * @brief       初始化 VESC 通信设备对象(按 id 取配置表组装协议栈)
	 * @param        pobj             : VESC 通信设备对象
	 * @param        id               : 实例编号(对应 commun_vesc_list 表项)
	 */
	void dev_commun_vesc_init(dev_commun_vesc_t *pobj, vesc_comm_id_e id);

	extern dev_commun_vesc_t dev_commun_vesc;

#ifdef __cplusplus
}
#endif
#endif /* USE_DEV_COMMUN_VESC */
#endif /* __DEV_COMMUN_VESC_H */
