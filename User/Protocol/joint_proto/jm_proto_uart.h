/**
 * @file        jm_proto_uart.h
 * @brief       关节电机协议-串口绑定层: 用 packer_parser 完成组帧/拆帧
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
 * @note        串口编解码: A5 5A + 长度 + 头校验 + (CMD+DATA) + CRC16, 见 packer_parser。
 *              收到整帧 -> jm_proto_dispatch -> 若有应答再用 packer 封帧发回。
 *              CAN 走另一套绑定层(jm_proto_can), 与本层互不影响。
 */
#ifndef __JM_PROTO_UART_H__
#define __JM_PROTO_UART_H__

#include <stdint.h>
#include "jm_proto.h"
#include "packer_parser.h"

#ifdef __cplusplus
extern "C"
{
#endif

	/* 串口底层字节发送函数(由应用注入, 通常包裹 drv_usart DMA 发送) */
	typedef void (*jm_uart_tx_fn)(uint8_t *data, uint16_t len);

	typedef struct
	{
		jm_proto_t proto;	 /* 协议核心实例 */
		upacker_inst packer; /* packer_parser 解/封包器 */
		jm_uart_tx_fn tx;	 /* 字节流发送 */
	} jm_proto_uart_t;

	/**
	 * @brief  初始化串口协议绑定
	 * @param  u         实例
	 * @param  ops       业务回调
	 * @param  motor_id  本机地址(串口可填0)
	 * @param  tx        串口字节发送函数(必填)
	 * @return 0 成功, -1 参数错误
	 */
	int jm_proto_uart_init(jm_proto_uart_t *u, const jm_proto_ops_t *ops,
						   uint8_t motor_id, jm_uart_tx_fn tx);

	/**
	 * @brief  喂入串口收到的原始字节(可在中断/线程中分批调用)
	 *         内部逐字节过状态机, 收齐整帧自动分发并回送应答。
	 */
	void jm_proto_uart_feed(jm_proto_uart_t *u, uint8_t *buf, uint16_t len);

	/**
	 * @brief  主动下发一帧(电机->上位机方向, 如周期上报反馈)
	 * @param  u     实例
	 * @param  cmd   命令码
	 * @param  body  载荷(不含CMD), 可为 NULL
	 * @param  len   载荷字节数
	 */
	void jm_proto_uart_send(jm_proto_uart_t *u, uint8_t cmd, const uint8_t *body, uint16_t len);

#ifdef __cplusplus
}
#endif
#endif /* __JM_PROTO_UART_H__ */
