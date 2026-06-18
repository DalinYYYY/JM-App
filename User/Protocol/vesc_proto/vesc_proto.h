/**
 * @file    vesc_proto.h
 * @brief   可移植的 VESC 串行通信协议模块（帧层 + 常用命令封装）。
 *
 * @details 特点：
 *          - 平台无关：不依赖任何 MCU 头文件，仅通过回调注入发送函数。
 *          - 可多实例：每条 USART 链路持有一个 vesc_proto_t。
 *          - 批量接收：DMA/中断收到一批数据后调用 vesc_proto_recv()。
 *
 *          移植只需 3 步：
 *          1. 定义一个 vesc_proto_t 实例和一份配置。
 *          2. 实现 send_bytes 回调（把字节写到你的 USART）。
 *          3. 在串口接收处调用 vesc_proto_recv()。
 */
#ifndef VESC_PROTO_H_
#define VESC_PROTO_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "vesc_comm_ids.h" /* 完整 COMM_PACKET_ID 枚举（0~159） */

/** @brief 最大 payload 长度，可在编译期覆盖。默认 512 与上位机 VESC Tool 一致。 */
#ifndef VESC_PROTO_MAX_PL
#define VESC_PROTO_MAX_PL 512u
#endif

/** @brief 收发缓冲区大小：payload + 起止/长度/CRC 余量。 */
#define VESC_PROTO_BUF_LEN (VESC_PROTO_MAX_PL + 8u)

#ifdef __cplusplus
extern "C"
{
#endif

	/* 命令 ID 枚举见 vesc_comm_ids.h（COMM_PACKET_ID，完整 0~159）。 */

	/**
 * @brief COMM_GET_VALUES 全字段解析结果（请求时 mask=0xFFFFFFFF）。
 */
	typedef struct
	{
		float temp_fet;		   /**< 控制器温度 ℃ */
		float temp_motor;	   /**< 电机温度 ℃ */
		float current_motor;   /**< 电机电流 A */
		float current_in;	   /**< 输入电流 A */
		float id;			   /**< 直轴电流 A */
		float iq;			   /**< 交轴电流 A */
		float duty;			   /**< 占空比 -1..1 */
		float rpm;			   /**< 电气转速 ERPM */
		float v_in;			   /**< 输入电压 V */
		float amp_hours;	   /**< 已用安时 Ah */
		float amp_hours_chg;   /**< 回充安时 Ah */
		float watt_hours;	   /**< 已用瓦时 Wh */
		float watt_hours_chg;  /**< 回充瓦时 Wh */
		int32_t tacho;		   /**< 里程计 */
		int32_t tacho_abs;	   /**< 绝对里程计 */
		uint8_t fault_code;	   /**< 故障码 */
		float pid_pos;		   /**< PID 位置 ° */
		uint8_t controller_id; /**< 控制器 ID */
	} vesc_values_t;

	/**
 * @brief COMM_FW_VERSION 解析结果（仅取常用字段）。
 */
	typedef struct
	{
		uint8_t major;	  /**< 固件主版本号 */
		uint8_t minor;	  /**< 固件次版本号 */
		char hw_name[32]; /**< 硬件名称（C 字符串） */
		uint8_t uuid[12]; /**< STM32 UUID */
	} vesc_fw_t;

	/**
 * @brief 用户回调集合。不需要的回调置 NULL 即可。
 */
	typedef struct
	{
		/** @brief 【必填】把 len 个字节发到 USART（阻塞或入队均可）。 */
		void (*send_bytes)(const uint8_t *data, uint16_t len, void *ctx);

		/** @brief 【选填】收到 COMM_GET_VALUES 回复且字段齐全时触发。 */
		void (*on_values)(const vesc_values_t *v, void *ctx);

		/** @brief 【选填】收到 COMM_FW_VERSION 回复时触发。 */
		void (*on_fw_version)(const vesc_fw_t *fw, void *ctx);

		/** @brief 【选填】收到任意一帧合法 payload 时触发（便于处理自定义命令）。 */
		void (*on_packet)(uint8_t cmd_id, const uint8_t *payload, uint16_t len, void *ctx);

		/** @brief 透传给所有回调的用户上下文指针。 */
		void *ctx;
	} vesc_proto_cfg_t;

	/**
 * @brief 协议实例。所有字段视为私有，请勿手改。
 */
	typedef struct
	{
		vesc_proto_cfg_t cfg;				/**< 用户配置 */
		uint16_t rx_rd;						/**< 接收缓冲读指针 */
		uint16_t rx_wr;						/**< 接收缓冲写指针 */
		int32_t bytes_left;					/**< 拆帧快进计数 */
		uint8_t rx_buf[VESC_PROTO_BUF_LEN]; /**< 接收拆帧缓冲 */
		uint8_t tx_buf[VESC_PROTO_BUF_LEN]; /**< 发送组帧缓冲 */
	} vesc_proto_t;

	/* ---- 初始化 ---- */

	/**
 * @brief  用配置初始化协议实例。
 * @param  vp   协议实例。
 * @param  cfg  配置；send_bytes 必填，否则所有发送将静默返回。为 NULL 时仅清零。
 */
	void vesc_proto_init(vesc_proto_t *vp, const vesc_proto_cfg_t *cfg);

	/**
 * @brief  复位接收拆帧状态（链路重连或同步丢失时调用）。
 * @param  vp  协议实例。
 */
	void vesc_proto_reset(vesc_proto_t *vp);

	/* ---- 接收 ---- */

	/**
 * @brief  把 USART 收到的一批字节喂入协议层（自动拆帧并触发回调）。
 * @param  vp    协议实例。
 * @param  data  接收字节缓冲。
 * @param  len   字节数。
 */
	void vesc_proto_recv(vesc_proto_t *vp, const uint8_t *data, uint16_t len);

	/* ---- 底层发送 ---- */

	/**
 * @brief  发送一帧原始 payload（自动组帧 + CRC）。
 * @param  vp       协议实例。
 * @param  payload  payload 数据，payload[0] 须为命令 ID。
 * @param  len      payload 长度，须为 1..VESC_PROTO_MAX_PL。
 */
	void vesc_proto_send(vesc_proto_t *vp, const uint8_t *payload, uint16_t len);

	/* ---- 高层命令封装（自动组帧 + 编码）---- */

	/** @brief 设置占空比。@param vp 实例。@param duty 占空比 -1..1。 */
	void vesc_set_duty(vesc_proto_t *vp, float duty);
	/** @brief 设置电机电流。@param vp 实例。@param current_a 电流（A）。 */
	void vesc_set_current(vesc_proto_t *vp, float current_a);
	/** @brief 设置制动电流。@param vp 实例。@param current_a 电流（A）。 */
	void vesc_set_current_brake(vesc_proto_t *vp, float current_a);
	/** @brief 设置相对电流。@param vp 实例。@param rel 相对值 -1..1。 */
	void vesc_set_current_rel(vesc_proto_t *vp, float rel);
	/** @brief 设置目标转速。@param vp 实例。@param erpm 电气转速（ERPM）。 */
	void vesc_set_rpm(vesc_proto_t *vp, float erpm);
	/** @brief 设置目标位置。@param vp 实例。@param deg 角度（°）。 */
	void vesc_set_pos(vesc_proto_t *vp, float deg);
	/** @brief 设置手刹电流。@param vp 实例。@param current_a 电流（A）。 */
	void vesc_set_handbrake(vesc_proto_t *vp, float current_a);
	/** @brief 发送心跳，维持控制不超时（需周期调用，建议 ~50ms）。@param vp 实例。 */
	void vesc_send_alive(vesc_proto_t *vp);
	/** @brief 请求设备重启。@param vp 实例。 */
	void vesc_reboot(vesc_proto_t *vp);
	/** @brief 请求实时值，结果经 on_values 回调返回。@param vp 实例。 */
	void vesc_request_values(vesc_proto_t *vp);
	/** @brief 请求固件版本，结果经 on_fw_version 回调返回。@param vp 实例。 */
	void vesc_request_fw_version(vesc_proto_t *vp);

	/**
 * @brief  通过本机 CAN 把一条命令转发给指定从机。
 * @param  vp       协议实例。
 * @param  can_id   目标从机 CAN ID。
 * @param  payload  被转发命令的 payload（payload[0] 为命令 ID）。
 * @param  len      payload 长度。
 */
	void vesc_forward_can(vesc_proto_t *vp, uint8_t can_id,
						  const uint8_t *payload, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* VESC_PROTO_H_ */
