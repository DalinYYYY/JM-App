/**
 * @file        jm_proto_can.h
 * @brief       关节电机协议-CAN绑定层: 扩展帧 ID=(CMD<<8)|电机ID, 独立编解码
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
 * @note        与串口层(jm_proto_uart)共用 jm_proto_dispatch 命令处理,
 *              但编解码不同: CAN 受单帧8字节限制, MIT/反馈帧用定点压缩,
 *              >8字节载荷用多帧分包(首字节带分包序号+末帧标志)。
 * @note        本层不直接依赖 drv_can, 通过收发回调注入(同 UART 层的解耦风格);
 *              桥接 drv_can 只需在应用层把 drvCanMsg_t 与本层的 jm_can_frame_t 互转。
 */
#ifndef __JM_PROTO_CAN_H__
#define __JM_PROTO_CAN_H__

#include <stdint.h>
#include "jm_proto.h"

#ifdef __cplusplus
extern "C"
{
#endif

	/* 本层自定义的最小CAN帧(不含HAL/drv_can类型, 保证可独立编译) */
	typedef struct
	{
		uint32_t id;     /* 仲裁ID(扩展帧29位): (CMD<<8)|电机ID */
		uint8_t len;     /* 数据长度 0~8 */
		uint8_t data[8]; /* 数据区 */
	} jm_can_frame_t;

	/* CAN底层发送函数(由应用注入, 通常包裹 drv_can_send) */
	typedef void (*jm_can_tx_fn)(const jm_can_frame_t *frame);

/* ---------------- 多帧分包协议(>8字节载荷) ----------------
 * 单帧: 载荷直接放 data[0..len-1]。
 * 多帧: data[0] = 控制字 = (末帧标志<<7) | (分包序号&0x7F),
 *       data[1..] = 该分包的载荷片段(每帧最多7字节)。
 *       序号从0递增, 末帧置bit7。接收方按序号拼接, 收到末帧后整体分发。*/
#define JM_CAN_SEG_LAST     0x80u
#define JM_CAN_SEG_SEQ_MASK 0x7Fu
#define JM_CAN_SEG_PAYLOAD  7u /* 多帧时每帧有效载荷字节数 */
#define JM_CAN_SINGLE_MAX   8u /* 单帧可直接承载的载荷上限 */

/* ---------------- MIT 控制帧定点压缩范围(可按电机改) ----------------
 * 与达妙/CubeMars习惯一致: pos16 vel12 kp12 kd12 tff12 = 64bit。*/
#define JM_MIT_POS_MIN (-12.5f)
#define JM_MIT_POS_MAX (12.5f)
#define JM_MIT_VEL_MIN (-65.0f)
#define JM_MIT_VEL_MAX (65.0f)
#define JM_MIT_KP_MIN  (0.0f)
#define JM_MIT_KP_MAX  (500.0f)
#define JM_MIT_KD_MIN  (0.0f)
#define JM_MIT_KD_MAX  (5.0f)
#define JM_MIT_TFF_MIN (-50.0f)
#define JM_MIT_TFF_MAX (50.0f)

/* 反馈帧压缩范围(温度线性映射) */
#define JM_FB_TEMP_MIN (-40.0f)
#define JM_FB_TEMP_MAX (215.0f)

	/* ---------------- CAN协议实例 ---------------- */
	typedef struct
	{
		jm_proto_t proto; /* 协议核心实例(与UART层同一套dispatch) */
		jm_can_tx_fn tx;  /* CAN帧发送 */
		uint8_t motor_id; /* 本机地址(1~127); 仅接收匹配本机或广播(0)的帧 */

		/* 多帧接收重组缓冲 */
		uint8_t rx_buf[1 + JM_PAYLOAD_MAX]; /* 重组区: [0]=CMD, 其后载荷 */
		uint16_t rx_len;                    /* 已重组字节数 */
		uint8_t rx_cmd;                     /* 当前重组的CMD */
		uint8_t rx_seq;                     /* 期望的下一分包序号 */
		uint8_t rx_active;                  /* 是否正在重组多帧 */
	} jm_proto_can_t;

	/**
	 * @brief  初始化CAN协议绑定
	 * @param  c         实例
	 * @param  ops       业务回调(与UART层可共用同一套ops)
	 * @param  motor_id  本机CAN地址(1~127)
	 * @param  tx        CAN帧发送函数(必填)
	 * @return 0 成功, -1 参数错误
	 */
	int jm_proto_can_init(jm_proto_can_t *c, const jm_proto_ops_t *ops,
	                      uint8_t motor_id, jm_can_tx_fn tx);

	/**
	 * @brief  喂入收到的一帧CAN报文(在CAN接收中断/线程中调用)
	 *         内部完成: 地址过滤 -> 拆ID取CMD -> (MIT)定点解压/多帧重组 ->
	 *         jm_proto_dispatch -> 应答按需压缩/分包发回。
	 * @param  c      实例
	 * @param  frame  收到的CAN帧
	 */
	void jm_proto_can_feed(jm_proto_can_t *c, const jm_can_frame_t *frame);

	/**
	 * @brief  主动下发一帧逻辑命令(电机->上位机, 如周期上报)
	 *         body为"逻辑载荷"(全精度), 本层按cmd决定是否压缩, >8字节自动分包。
	 * @param  c     实例
	 * @param  cmd   命令码
	 * @param  body  载荷(不含CMD), 可为NULL
	 * @param  len   载荷字节数
	 */
	void jm_proto_can_send(jm_proto_can_t *c, uint8_t cmd, const uint8_t *body, uint16_t len);

	/* ---------------- 定点压缩助手(供测试/上位机参考实现) ---------------- */
	/** @brief 把5个float的MIT指令压缩进8字节(pos16 vel12 kp12 kd12 tff12) */
	void jm_mit_pack(uint8_t out[8], float pos, float vel, float kp, float kd, float tff);
	/** @brief 从8字节解出5个float的MIT指令 */
	void jm_mit_unpack(const uint8_t in[8], float *pos, float *vel, float *kp, float *kd, float *tff);
	/** @brief 反馈帧压缩进8字节: pos16 vel16 tq16 temp8 err8 */
	uint8_t jm_fb_pack(uint8_t out[8], const jm_feedback_t *fb);

#ifdef __cplusplus
}
#endif
#endif /* __JM_PROTO_CAN_H__ */
