/**
 * @file        dev_commun_uart.h
 * @brief       关节电机串口通信设备(USART+DMA空闲中断, 承载 joint_proto 协议)
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
 * @note        分层: 本设备(粘合层) → jm_proto_uart(串口编解码) → jm_proto(命令分发)
 *                                  ↘ drv_usart(USART+DMA空闲中断)
 *              使用方式:
 *                - 应用先 set_ops() 注入业务回调(电机控制/参数读写数据源);
 *                - 调用 start() 启动空闲中断+DMA接收;
 *                - 在对应串口 IRQ 中调用 on_rx_idle()(检测IDLE并落数据);
 *                - 在主循环/线程周期调用 poll()(取数据喂协议栈, 自动回复);
 *                - 可选 report() 主动上报实时反馈, 无需上位机轮询。
 */
#ifndef __DEV_COMMUN_UART_H
#define __DEV_COMMUN_UART_H

#include "dev_config.h"
#if defined(USE_DEV_COMMUN_UART)

#include "drv_usart.h"		/* usartNumber_e / 空闲中断收发接口 */
#include "jm_proto_uart.h"	/* jm_proto_uart_t / jm_proto_ops_t */
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

	/**
	 * @brief 关节电机串口通信实例编号(顺序须与 dev_config.c 的 commun_uart_list 一致)
	 * @param  JM_UART_COMM_ID_1       : 第一路串口通信
	 * @param  JM_UART_COMM_ID_MAX     : 实例数量边界
	 */
	typedef enum
	{
		JM_UART_COMM_ID_1 = 0,
		JM_UART_COMM_ID_MAX,
	} jm_uart_comm_id_e;

	/**
	 * @brief 关节电机串口通信设备配置(在 dev_config.c 的 commun_uart_list 填表)
	 * @param  name                   : 实例名(调试用)
	 * @param  uart                   : 所属串口编号(须为已配置 DMA 的串口)
	 * @param  motor_id               : 本机地址(串口可忽略, 与CAN保持一致)
	 */
	typedef struct
	{
		char name[20];
		usartNumber_e uart;
		uint8_t motor_id;
	} dev_commun_uart_config_t;

	/* 配置表定义在 dev_config.c */
	extern const dev_commun_uart_config_t commun_uart_list[JM_UART_COMM_ID_MAX];

	/**
	 * @brief 关节电机串口通信设备对象
	 * @param  jm                     : joint_proto 串口协议实例(私有, 须为首成员)
	 * @param  uart                   : 所属串口编号(私有)
	 * @param  motor_id               : 本机地址(私有)
	 * @param  started                : 接收已启动标志(私有)
	 * @param  rx_tmp                  : 空闲突发取数临时缓冲(私有)
	 * @param  ops                    : 业务回调(应用注入, start 前须设置)
	 * @param  set_ops                : 注入业务回调数据源
	 * @param  start                  : 启动空闲中断+DMA接收, 返回 DEV_EOK/DEV_ERROR
	 * @param  on_rx_idle             : 串口 IRQ 中调用, 检测 IDLE 并落本帧数据
	 * @param  poll                   : 主循环/线程周期调用, 取数据喂协议栈并自动回复
	 * @param  report                 : 主动上报一帧(cmd+载荷), 无需上位机轮询
	 */
	typedef struct dev_commun_uart
	{
		jm_proto_uart_t jm;
		usartNumber_e uart;
		uint8_t motor_id;
		uint8_t started;
		uint8_t rx_tmp[DEV_JM_UART_RX_BUF_SIZE];
		int last_error;
		uint32_t init_count;
		uint32_t start_count;
		uint32_t start_fail_count;
		uint32_t poll_count;
		uint32_t poll_not_started_count;
		uint32_t poll_rx_count;
		uint32_t poll_rx_bytes;
		uint32_t tx_count;
		uint32_t tx_fail_count;
		uint32_t idle_irq_count;

		/* public */
		const jm_proto_ops_t *ops;

		void (*set_ops)(struct dev_commun_uart *pobj, const jm_proto_ops_t *ops);
		int (*start)(struct dev_commun_uart *pobj);
		void (*on_rx_idle)(struct dev_commun_uart *pobj);
		void (*poll)(struct dev_commun_uart *pobj);
		void (*report)(struct dev_commun_uart *pobj, uint8_t cmd, const uint8_t *body, uint16_t len);
	} dev_commun_uart_t;

	/**
	 * @brief       初始化关节电机串口通信设备对象(按 id 取配置表组装)
	 * @param        pobj             : 设备对象
	 * @param        id               : 实例编号(对应 commun_uart_list 表项)
	 * @note         init 后须 set_ops() 注入业务回调, 再 start()。
	 */
	void dev_commun_uart_init(dev_commun_uart_t *pobj, jm_uart_comm_id_e id);

	extern dev_commun_uart_t dev_commun_uart;

#ifdef __cplusplus
}
#endif
#endif /* USE_DEV_COMMUN_UART */
#endif /* __DEV_COMMUN_UART_H */
