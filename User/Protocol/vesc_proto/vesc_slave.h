/**
 * @file    vesc_slave.h
 * @brief   VESC 从机层：让 MCU 被 VESC Tool 识别为一个 VESC 设备。
 *
 * @details 基于 vesc_proto 帧层，实现 VESC Tool 连接所需的最小回复集：
 *          - COMM_FW_VERSION：握手识别（必需，否则 VESC Tool 不认设备）。
 *          - COMM_GET_VALUES / _SELECTIVE：实时数据（仪表盘显示）。
 *          其余请求经 user_packet 回调透传给应用，可自行扩展（如 MCCONF）。
 *
 * 用法：
 * @code
 *   vesc_slave_t sl;
 *   vesc_slave_cfg_t cfg = {0};
 *   cfg.send_bytes = my_uart_send;       // 必填
 *   cfg.hw_name    = "MyBoard";          // 显示在 VESC Tool 上的硬件名
 *   cfg.fw_major   = 6;  cfg.fw_minor = 0;
 *   cfg.get_values = my_fill_values;     // 仪表盘取值回调
 *   vesc_slave_init(&sl, &cfg);
 *
 *   // 串口收到字节：
 *   vesc_slave_recv(&sl, rxbuf, n);      // 内部自动回复 FW_VERSION / GET_VALUES
 * @endcode
 */
#ifndef VESC_SLAVE_H_
#define VESC_SLAVE_H_

#include "vesc_proto.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 从机配置与设备身份。
 */
typedef struct {
	/** @brief 【必填】发送回调，把字节写到 USART。 */
	void (*send_bytes)(const uint8_t *data, uint16_t len, void *ctx);

	/** @brief 【必填】填充实时值回调，VESC Tool 请求 GET_VALUES 时触发。 */
	void (*get_values)(vesc_values_t *v, void *ctx);

	/** @brief 【选填】未识别命令透传给应用（用于扩展 MCCONF 等）。 */
	void (*user_packet)(vesc_proto_t *vp, uint8_t cmd_id,
			const uint8_t *payload, uint16_t len, void *ctx);

	const char *hw_name;   /**< 硬件名称（显示在 VESC Tool），NULL 则用 "VESC_PROTO" */
	const char *fw_name;   /**< 固件名称字符串，NULL 则用空串 */
	uint8_t     fw_major;  /**< 固件主版本号 */
	uint8_t     fw_minor;  /**< 固件次版本号 */
	uint8_t     hw_type;   /**< 硬件类型（0 = HW_TYPE_VESC，通常保持 0） */
	uint8_t     uuid[12];  /**< 设备 UUID（用于配置备份/恢复，可填芯片唯一 ID） */

	void *ctx;             /**< 透传给所有回调的用户上下文 */
} vesc_slave_cfg_t;

/**
 * @brief 从机实例。字段视为私有。
 */
typedef struct {
	vesc_proto_t     proto;  /**< 底层帧层实例 */
	vesc_slave_cfg_t cfg;    /**< 配置副本 */
} vesc_slave_t;

/**
 * @brief  初始化从机实例。
 * @param  sl   从机实例。
 * @param  cfg  配置；send_bytes 与 get_values 必填。
 */
void vesc_slave_init(vesc_slave_t *sl, const vesc_slave_cfg_t *cfg);

/**
 * @brief  把 USART 收到的一批字节喂入从机（自动拆帧并回复请求）。
 * @param  sl    从机实例。
 * @param  data  接收字节缓冲。
 * @param  len   字节数。
 */
void vesc_slave_recv(vesc_slave_t *sl, const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* VESC_SLAVE_H_ */
