/**
 * @file        jm_proto_uart.c
 * @brief       关节电机协议-串口绑定层实现
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
 */
#include <stddef.h>
#include "jm_proto_uart.h"

/* packer 的回调签名无用户上下文参数(void(*)(uint8_t*,uint16_t))。
 * 这里用"当前活动实例"指针把回调路由回实例。
 * 适用于典型单条上位机链路(一个串口协议实例); 每次 feed/send 前刷新指针,
 * 中断与线程不应同时驱动同一实例。多实例场景需改用带上下文的 packer 回调。*/
static jm_proto_uart_t *g_active = NULL;

/* 整帧接收完成回调: data[0]=CMD, 其后为载荷; flen=CMD+载荷总长 */
static void on_frame(uint8_t *data, uint16_t flen)
{
	jm_proto_uart_t *u = g_active;
	uint8_t cmd;
	const uint8_t *payload;
	uint16_t plen;

	if (u == NULL || flen < 1)
	{
		return; /* 至少要有 CMD */
	}
	cmd = data[0];
	payload = (flen > 1) ? &data[1] : NULL;
	plen = (uint16_t)(flen - 1);

	jm_proto_dispatch(&u->proto, cmd, payload, plen);

	/* 有应答则封帧发回(reply[0]=CMD, 其后为载荷) */
	if (u->proto.reply_len > 0)
	{
		upacker_pack(&u->packer, u->proto.reply, u->proto.reply_len);
	}
}

/* packer 的发送回调: 已封好整帧(含帧头/CRC), 经底层串口发出 */
static void on_send(uint8_t *frame, uint16_t len)
{
	if (g_active != NULL && g_active->tx != NULL)
	{
		g_active->tx(frame, len);
	}
}

int jm_proto_uart_init(jm_proto_uart_t *u, const jm_proto_ops_t *ops, uint8_t motor_id, jm_uart_tx_fn tx)
{
	if (u == NULL || tx == NULL)
	{
		return -1;
	}
	jm_proto_init(&u->proto, ops, motor_id);
	u->tx = tx;
	g_active = u;

	if (upacker_init(&u->packer, MAX_PACK_SIZE) != 0)
	{
		return -1;
	}
	/* 解包整帧回调 on_frame; 封包输出回调 on_send */
	upacker_set_cb(&u->packer, on_frame, on_send);
	return 0;
}

void jm_proto_uart_feed(jm_proto_uart_t *u, uint8_t *buf, uint16_t len)
{
	if (u == NULL || buf == NULL)
	{
		return;
	}
	g_active = u; /* 喂数据前锁定当前实例, 保证 send 回调指向正确实例 */
	upacker_unpack(&u->packer, buf, len);
}

void jm_proto_uart_send(jm_proto_uart_t *u, uint8_t cmd, const uint8_t *body, uint16_t len)
{
	uint8_t frame[1 + JM_PAYLOAD_MAX];
	uint16_t n;

	if (u == NULL || len > JM_PAYLOAD_MAX)
	{
		return;
	}
	frame[0] = cmd;
	if (body != NULL && len > 0)
	{
		uint16_t i;
		for (i = 0; i < len; i++)
		{
			frame[1 + i] = body[i];
		}
	}
	n = (uint16_t)(1 + len);

	g_active = u;
	upacker_pack(&u->packer, frame, n);
}
