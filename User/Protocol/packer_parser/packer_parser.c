/**
 * @file        packer_parser.c
 * @brief       数据解包和封包器
 * 
 * @author      name (name@robot.com)
 * @version     1.0
 * @date        2026-06-18
 * 
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 * 
 * 
 * @par 修改日志:
 * | 日期       | 版本 | 作者   | 修改内容   |
 * |------------|------|--------|------------|
 * | 2026-06-18     | 1.0  | yangsl | 初始创建   |
 * 
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 */
#include <string.h>
#include "packer_parser.h"
#include "crc16.h"
// 内部状态机状态定义(顺序对齐实际接收流程)
typedef enum
{
	STATE_HEADER_HIGH, // 等待帧头高位 A5
	STATE_HEADER_LOW,  // 等待帧头低位 5A
	STATE_LEN_HIGH,	   // 长度高字节
	STATE_LEN_LOW,	   // 长度低字节
	STATE_HEADER_CHK,  // 头部校验检查
	STATE_DATA_RECV,   // 接收数据区
	STATE_CRC_LOW,	   // CRC低字节
	STATE_CRC_HIGH	   // CRC高字节
} DecoderState;

static uint8_t frame_decode(upacker_inst_t packer, uint8_t d);
static int frame_encode(upacker_inst_t packer, uint8_t *data, uint16_t size);

/*---------------------------- 解码状态机 ----------------------------*/
static uint8_t frame_decode(upacker_inst_t packer, uint8_t d)
{
	switch (packer->state)
	{
		case STATE_HEADER_HIGH:
			if (d == STX_H)
			{
				packer->calc = STX_H;
				packer->state = STATE_HEADER_LOW;
			}
			break;

		case STATE_HEADER_LOW:
			if (d == STX_L)
			{
				packer->calc += STX_L;
				packer->state = STATE_LEN_HIGH;
			}
			else
			{
				packer->state = STATE_HEADER_HIGH;
			}
			break;

		case STATE_LEN_HIGH:
			/* 保留帧头累加值(calc 已含 STX_H+STX_L), 与 encode 头校验口径一致 */
			packer->flen = (uint16_t)(d << 8);
			packer->calc += d;
			packer->state = STATE_LEN_LOW;
			break;

		case STATE_LEN_LOW:
			packer->flen |= d;
			packer->calc += d;
			if (packer->flen > MAX_PACK_SIZE)
			{
				packer->state = STATE_HEADER_HIGH; // 长度越界, 丢弃重新同步
			}
			else
			{
				packer->state = STATE_HEADER_CHK;
			}
			break;

		case STATE_HEADER_CHK:
#if USE_DATA_CHECK
			if (d != (packer->calc & 0xFF))
			{
				packer->state = STATE_HEADER_HIGH;
				return 0;
			}
#endif
			packer->cnt = 0;
			/* 无数据区(flen==0)时直接进入 CRC 接收, 避免误吞一字节 */
			packer->state = (packer->flen == 0) ? STATE_CRC_LOW : STATE_DATA_RECV;
			break;

		case STATE_DATA_RECV:
			packer->data[packer->cnt++] = d;
			if (packer->cnt >= packer->flen)
			{
				packer->state = STATE_CRC_LOW;
			}
			break;

		case STATE_CRC_LOW:
			packer->rx_crc16 = d;
			packer->state = STATE_CRC_HIGH;
			break;

		case STATE_CRC_HIGH:
		{
			packer->rx_crc16 |= (d << 8);
			packer->state = STATE_HEADER_HIGH;

#if USE_DATA_CHECK
			const uint16_t calc_crc = crc16_calc(packer->data, packer->flen);
#else
			const uint16_t calc_crc = packer->rx_crc16;
#endif
			return (calc_crc == packer->rx_crc16);
		}

		default:
			packer->state = STATE_HEADER_HIGH;
			break;
	}
	return 0;
}

/*---------------------------- 编码打包 ----------------------------*/
static uint8_t send_buffer[SEND_BUF_SIZE];
static int frame_encode(upacker_inst_t packer, uint8_t *data, uint16_t size)
{
	uint16_t crc;
	uint16_t n = 0;

	/* 入参与发送回调检查; size 越界直接拒绝, 防止冲掉 send_buffer */
	if (packer == NULL || packer->send == NULL || size > MAX_PACK_SIZE)
	{
		return -1;
	}
	if (size != 0 && data == NULL)
	{
		return -1;
	}

	/* 头部: 帧头 + 长度(大端) + 头校验和 */
	send_buffer[n++] = STX_H;
	send_buffer[n++] = STX_L;
	send_buffer[n++] = (uint8_t)((size >> 8) & 0xFF);
	send_buffer[n++] = (uint8_t)(size & 0xFF);
	send_buffer[n++] = (uint8_t)((STX_H + STX_L + send_buffer[2] + send_buffer[3]) & 0xFF);

	/* 数据区 */
	if (size != 0)
	{
		memcpy(&send_buffer[n], data, size);
		n += size;
	}

	/* CRC16(覆盖数据区), 显式小端落帧, 与 decode 收序一致 */
	crc = crc16_calc(data, size);
	send_buffer[n++] = (uint8_t)(crc & 0xFF);
	send_buffer[n++] = (uint8_t)((crc >> 8) & 0xFF);

	packer->send(send_buffer, n);
	return 0;
}

/*---------------------------- 初始化函数 ----------------------------*/
/**
 * @brief  数据包解析器初始化函数
 * 
 * @param  packer           : 解包器实例
 * @param  size             : 设置可以接受数据包大小 （只有动态内存时才需要设置）
 * 
 * @return int 
 */
int upacker_init(upacker_inst_t packer, uint16_t size)
{
	if (packer == NULL)
	{
		return -1;
	}
#if USE_DYNAMIC_MEM
	/* 动态模式: 申请数据缓冲, 上限不超过协议最大包长 */
	if (size == 0 || size > MAX_PACK_SIZE)
	{
		return -1;
	}
	packer->data = (uint8_t *)malloc(size);
	if (!packer->data)
		return -1;
#else
	(void)size; /* 静态模式忽略 size, 缓冲为定长 MAX_PACK_SIZE */
#endif
	packer->cb = NULL;
	packer->send = NULL;
	packer->flen = 0;
	packer->cnt = 0;
	packer->state = STATE_HEADER_HIGH; // 初始状态
	return 0;
}

/**
 * @brief  设置接收/发送回调
 *
 * @param  packer           : 解包器实例
 * @param  cb               : 整帧接收完成回调(收到合法帧时触发, 可为 NULL)
 * @param  send             : 底层发送回调(封包后输出字节流, 封包前须设置)
 */
void upacker_set_cb(upacker_inst_t packer, PACKER_CB cb, PACKER_CB send)
{
	if (packer == NULL)
	{
		return;
	}
	packer->cb = cb;
	packer->send = send;
}

/*---------------------------- 数据解包 ----------------------------*/

/**
 * @brief 数据解包函数
 * 
 * @param  packer           : 解包器实例
 * @param  buff             : 解包数据缓冲区
 * @param  size             : 解包数据长度
 * 
 */
void upacker_unpack(upacker_inst_t packer, uint8_t *buff, uint16_t size)
{
	if (packer == NULL || buff == NULL)
	{
		return;
	}
	for (uint16_t i = 0; i < size; i++)
	{
		if (frame_decode(packer, buff[i]) && packer->cb != NULL)
		{
			packer->cb(packer->data, packer->flen);
		}
	}
}

/*---------------------------- 数据封包 ----------------------------*/
/**
 * @brief 数据封包函数
 * 
 * @param  packer           : 封包器实例
 * @param  buff             : 封包数据缓冲区
 * @param  size             : 封包数据长度
 */
void upacker_pack(upacker_inst_t packer, uint8_t *buff, uint16_t size)
{
	frame_encode(packer, buff, size);
}
