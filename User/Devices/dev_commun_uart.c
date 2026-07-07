/**
 * @file        dev_commun_uart.c
 * @brief       关节电机串口通信设备(USART+DMA空闲中断, 承载 joint_proto 协议)
 *
 * @author      Dalin (dalinyy@163.com)
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
 * @note        分层: 本设备(粘合层) → jm_proto_uart(编解码) → jm_proto(分发)
 *                                  ↘ drv_usart(USART+DMA空闲中断)
 *              jm_proto_uart 的发送回调签名无上下文参数, 故用模块级活动设备指针
 *              s_active 路由到正确串口(典型单条上位机链路)。
 */
#include "dev_commun_uart.h"
#if defined(USE_DEV_COMMUN_UART)

#include "assert_report.h"
#include <string.h>

dev_commun_uart_t dev_commun_uart;

/* 当前活动设备(支撑无上下文的 tx 回调; 每次 start/poll/report 前刷新) */
static dev_commun_uart_t *s_active = NULL;

#define JM_UART_TX_TIMEOUT_MS 20u

/* ==================================================================== */
/*  协议栈发送回调: jm_proto_uart 组好整帧后, 经串口 DMA 推出              */
/* ==================================================================== */
static void jm_uart_tx(uint8_t *data, uint16_t len)
{
	if (s_active != NULL)
	{
		if (drv_usart_send(s_active->uart, data, len, JM_UART_TX_TIMEOUT_MS) == DRV_EOK)
		{
			s_active->tx_count++;
		}
		else
		{
			s_active->tx_fail_count++;
			s_active->last_error = -4;
		}
	}
}

/* ==================================================================== */
/*  设备接口                                                            */
/* ==================================================================== */

/* 注入业务回调数据源(电机控制/参数读写); 须在 start 前调用 */
static void dev_commun_uart_set_ops(struct dev_commun_uart *pobj, const jm_proto_ops_t *ops)
{
	assert_report(pobj != NULL);
	pobj->ops = ops;
}

/* 启动空闲中断+DMA不定长接收(幂等: 重复调用只生效一次) */
static int dev_commun_uart_start(struct dev_commun_uart *pobj)
{
	assert_report(pobj != NULL);
	pobj->start_count++;
	if (pobj->started)
	{
		return DEV_EOK;
	}
	/* 组装串口协议: 注入业务回调与发送函数 */
	s_active = pobj;
	if (jm_proto_uart_init(&pobj->jm, pobj->ops, pobj->motor_id, jm_uart_tx) != 0)
	{
		pobj->start_fail_count++;
		pobj->last_error = -1;
		return DEV_ERROR;
	}
	if (usart_idle_init(pobj->uart, DEV_JM_UART_RX_BUF_SIZE) != DRV_EOK)
	{
		pobj->start_fail_count++;
		pobj->last_error = -2;
		return DEV_ERROR;
	}
	pobj->started = 1;
	pobj->last_error = DEV_EOK;
	return DEV_EOK;
}

/* 串口 IRQ 中调用: 检测 IDLE 标志, 停 DMA 并锁存本帧长度 */
static void dev_commun_uart_on_rx_idle(struct dev_commun_uart *pobj)
{
	assert_report(pobj != NULL);
	pobj->idle_irq_count++;
	drv_uart_idle(pobj->uart);
}

/* 主循环/线程周期调用: 取出空闲突发数据喂协议栈, 自动完成分发与回复 */
static void dev_commun_uart_poll(struct dev_commun_uart *pobj)
{
	uint16_t rx_len = 0;
	assert_report(pobj != NULL);
	pobj->poll_count++;
	if (!pobj->started)
	{
		pobj->poll_not_started_count++;
		return;
	}

	if (usart_idle_get_data(pobj->uart, pobj->rx_tmp, &rx_len) != DRV_EOK)
	{
		pobj->last_error = -3;
		return;
	}
	if (rx_len != 0)
	{
		s_active = pobj; /* 锁定当前实例, 保证 tx 回调指向正确串口 */
		pobj->poll_rx_count++;
		pobj->poll_rx_bytes += rx_len;
		jm_proto_uart_feed(&pobj->jm, pobj->rx_tmp, rx_len);
	}
}

/* 主动上报一帧(电机->上位机方向, 如周期反馈), 无需上位机轮询 */
static void dev_commun_uart_report(struct dev_commun_uart *pobj, uint8_t cmd,
                                   const uint8_t *body, uint16_t len)
{
	assert_report(pobj != NULL);
	if (!pobj->started)
	{
		return;
	}
	s_active = pobj;
	jm_proto_uart_send(&pobj->jm, cmd, body, len);
}

void dev_commun_uart_init(dev_commun_uart_t *pobj, jm_uart_comm_id_e id)
{
	assert_report(pobj != NULL);
	assert_report(id < JM_UART_COMM_ID_MAX);
	memset(pobj, 0, sizeof(dev_commun_uart_t));

	const dev_commun_uart_config_t *cfg = &commun_uart_list[id];
	pobj->uart = cfg->uart;
	pobj->motor_id = cfg->motor_id;
	pobj->ops = NULL; /* 由应用 set_ops 注入 */

	pobj->init_count = 1;

	/* public */
	pobj->set_ops = dev_commun_uart_set_ops;
	pobj->start = dev_commun_uart_start;
	pobj->on_rx_idle = dev_commun_uart_on_rx_idle;
	pobj->poll = dev_commun_uart_poll;
	pobj->report = dev_commun_uart_report;
}

#endif /* USE_DEV_COMMUN_UART */
