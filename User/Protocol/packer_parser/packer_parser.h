/**
 * @file        packer_parser.h
 * @brief       数据包解析器头文件
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

#ifndef __PACKER_PARSER_H__
#define __PACKER_PARSER_H__

#include "stdint.h"

#ifdef __cplusplus
extern "C"
{
#endif

// 配置宏
#define USE_DYNAMIC_MEM 0  // 是否使用动态内存
#define USE_DATA_CHECK 1   // 是否启用校验检查
#define MAX_PACK_SIZE 1024 // 最大数据包长度(数据区 CMD+DATA)
#define STX_L 0x5A		   // 帧头低位
#define STX_H 0xA5		   // 帧头高位

/* 整帧开销: 帧头2 + 长度2 + 头校验1 + CRC2 = 7 字节 */
#define PACK_OVERHEAD 7
/* 发送缓冲区大小: 数据区上限 + 帧开销 */
#define SEND_BUF_SIZE (MAX_PACK_SIZE + PACK_OVERHEAD)

	// 类型定义
	typedef void (*PACKER_CB)(uint8_t *d, uint16_t s);

	typedef struct
	{
#if !USE_DYNAMIC_MEM
		uint8_t data[MAX_PACK_SIZE]; // 数据缓冲区
#else
	uint8_t *data;
#endif
		uint16_t flen;	   // 数据区实际长度(CMD+DATA)
		uint16_t rx_crc16; // 接收的CRC值
		uint16_t crc16;	   // 计算的CRC值
		uint8_t calc;	   // 头部校验计算值
		uint8_t state;	   // 状态机状态(0-7)
		uint16_t cnt;	   // 数据接收计数器
		PACKER_CB cb;	   // 数据包接收回调
		PACKER_CB send;	   // 数据发送回调
	} upacker_inst;
	typedef upacker_inst *upacker_inst_t;

	// 函数声明
	int upacker_init(upacker_inst_t packer, uint16_t size);
	void upacker_set_cb(upacker_inst_t packer, PACKER_CB cb, PACKER_CB send);
	void upacker_pack(upacker_inst_t packer, uint8_t *buff, uint16_t size);
	void upacker_unpack(upacker_inst_t packer, uint8_t *buff, uint16_t size);

#ifdef __cplusplus
}
#endif
#endif //__PACKER_PARSER_H__
