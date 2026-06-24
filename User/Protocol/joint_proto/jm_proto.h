/**
 * @file        jm_proto.h
 * @brief       关节电机通信协议核心(传输无关): CMD分发 + 小端编解码助手
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
 * @note        本层与传输介质无关: 串口(jm_proto_uart)与CAN(jm_proto_can)
 *              各自完成组帧/拆帧后, 都调用 jm_proto_dispatch() 走同一套命令处理。
 *              业务动作(真正控制电机/读参数)通过 jm_proto_ops_t 回调注入, 不在本层实现。
 */
#ifndef __JM_PROTO_H__
#define __JM_PROTO_H__

#include <stdint.h>
#include <stdbool.h>
#include "jm_cmd_def.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* 单帧载荷上限(数据区, 不含CMD) */
#define JM_PAYLOAD_MAX 256

	/* ---------------- 实时反馈数据(读命令的数据源) ---------------- */
	typedef struct
	{
		float pos;			 /* 输出端位置 rad */
		float vel;			 /* 输出端速度 rad/s */
		float torque;		 /* 输出端力矩 Nm */
		float id;			 /* d轴电流 A */
		float iq;			 /* q轴电流 A */
		float ia, ib, ic;	 /* 三相电流 A */
		float vbus;			 /* 母线电压 V */
		float ibus;			 /* 母线电流 A */
		float temp_fet;		 /* 功率管温度 ℃ */
		float temp_motor;	 /* 电机温度 ℃ */
		int32_t multiturn;	 /* 多圈计数 */
		float single;		 /* 单圈位置 rad */
		uint32_t fault_mask; /* 故障位掩码 */
		uint32_t warn_mask;	 /* 警告位掩码 */
		uint8_t top_fsm;	 /* 顶层状态 top_fsm_e */
		uint8_t run_state;	 /* 运行子状态 run_state_e */
		uint8_t ctrl_mode;	 /* 当前控制模式 ctrl_mode_e */
		uint8_t enable;		 /* 是否使能 */
	} jm_feedback_t;

	/* ---------------- 业务回调(由应用层实现, 注入到协议) ----------------
	 * 协议层只负责"解析CMD/载荷"和"组织应答", 真正动作由这些回调完成。
	 * 返回 jm_err_e: JM_ERR_OK 表示成功, 其余值会触发 NACK 应答。
	 */
	typedef struct jm_proto_ops
	{
		/* 设置控制模式并下发目标(CMD 0x00~0xB8)。payload/len 为该命令的原始载荷,
		 * 协议层已用小端助手解析的责任留给应用; 也可在此直接按 cmd 解析。*/
		jm_err_e (*set_mode)(uint8_t cmd, const uint8_t *payload, uint16_t len);

		/* 读实时反馈: 应用填充 fb。 */
		jm_err_e (*get_feedback)(jm_feedback_t *fb);

		/* 读单个参数: 输出到 value(小端), 写回 *out_type 与 *out_len(字节数)。*/
		jm_err_e (*param_read)(uint16_t param_id, uint8_t *value, uint8_t *out_type, uint8_t *out_len);

		/* 写单个参数: value 为小端原始字节, len 为字节数(由参数类型决定)。*/
		jm_err_e (*param_write)(uint16_t param_id, const uint8_t *value, uint8_t len);

		/* 参数固化到Flash / 恢复默认(param_id=0xFFFF 表示全部)。可为 NULL。*/
		jm_err_e (*param_save)(void);
		jm_err_e (*param_reset)(uint16_t param_id);

		/* 读设备信息。可为 NULL(不支持则回 NACK)。*/
		jm_err_e (*get_dev_info)(uint32_t *hw_ver, uint32_t *fw_ver, uint8_t uid[12]);
		const char *(*get_dev_name)(void); /* 返回名称字符串, 可为 NULL */

		/* 设置同步遥测订阅(CMD 0xCB)。enable 为周期上报总开关(1=启动周期上报,
		 * 0=停止); mask 为 jm_telemetry_bit_e 位或, 选择上报哪些数据组; period_ms 为
		 * 上报周期(0 表示沿用默认/不改)。绑定层据此周期主动推送 0xCA 数据帧(无应答)。
		 * 可为 NULL(不支持订阅则回 NACK)。*/
		jm_err_e (*set_telemetry)(uint8_t enable, uint16_t mask, uint16_t period_ms);
	} jm_proto_ops_t;

	/* ---------------- 协议实例 ---------------- */
	typedef struct jm_proto
	{
		const jm_proto_ops_t *ops; /* 业务回调 */
		uint8_t motor_id;		   /* 本机地址(CAN用; 串口可忽略) */
		/* 应答输出缓冲(由 dispatch 填充, 调用方取走发送) */
		uint8_t reply[1 + JM_PAYLOAD_MAX]; /* reply[0]=CMD, 其后为载荷 */
		uint16_t reply_len;				   /* 含CMD的总长度; 0 表示无需应答 */
	} jm_proto_t;

	/* ---------------- 小端编解码助手(串口/CAN共用) ---------------- */
	static inline uint16_t jm_rd_u16(const uint8_t *p)
	{
		return (uint16_t)(p[0] | (p[1] << 8));
	}
	static inline uint32_t jm_rd_u32(const uint8_t *p)
	{
		return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
	}
	static inline float jm_rd_f32(const uint8_t *p)
	{
		union
		{
			uint32_t u;
			float f;
		} v;
		v.u = jm_rd_u32(p);
		return v.f;
	}
	static inline void jm_wr_u16(uint8_t *p, uint16_t v)
	{
		p[0] = (uint8_t)v;
		p[1] = (uint8_t)(v >> 8);
	}
	static inline void jm_wr_u32(uint8_t *p, uint32_t v)
	{
		p[0] = (uint8_t)v;
		p[1] = (uint8_t)(v >> 8);
		p[2] = (uint8_t)(v >> 16);
		p[3] = (uint8_t)(v >> 24);
	}
	static inline void jm_wr_f32(uint8_t *p, float f)
	{
		union
		{
			uint32_t u;
			float f;
		} v;
		v.f = f;
		jm_wr_u32(p, v.u);
	}

	/* ---------------- 核心接口 ---------------- */

	/**
	 * @brief  初始化协议实例
	 * @param  proto     实例
	 * @param  ops       业务回调(必填)
	 * @param  motor_id  本机地址(串口可填0)
	 */
	void jm_proto_init(jm_proto_t *proto, const jm_proto_ops_t *ops, uint8_t motor_id);

	/**
	 * @brief  传输无关的命令分发(串口/CAN 解出 cmd+payload 后都调它)
	 * @param  proto    实例
	 * @param  cmd      命令码(jm_cmd_e)
	 * @param  payload  载荷(不含CMD), 可为 NULL
	 * @param  len      载荷字节数
	 * @return 处理结果错误码; 应答数据放在 proto->reply / proto->reply_len。
	 *         reply_len>0 时调用方需把 reply 经各自传输层发回。
	 */
	jm_err_e jm_proto_dispatch(jm_proto_t *proto, uint8_t cmd, const uint8_t *payload, uint16_t len);

#ifdef __cplusplus
}
#endif
#endif /* __JM_PROTO_H__ */
