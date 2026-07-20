/**
 * @file        jm_proto.h
 * @brief       关节电机通信协议核心(传输无关): CMD分发 + 小端编解码助手
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
 * | 2026-06-25 | 1.1  | Dalin  | 补全 0xC9/0xE2/0xE3/0xF0/0xF1 ops 回调 |
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
		float pos;           /* 电机端多圈位置 θ_m rad (带符号, ±∞) */
		float vel;           /* 电机端机械角速度 rad/s */
		float torque;        /* 输出端力矩 Nm */
		float id;            /* d轴电流 A */
		float iq;            /* q轴电流 A */
		float ia, ib, ic;    /* 三相电流 A */
		float vbus;          /* 母线电压 V */
		float ibus;          /* 母线电流 A */
		float temp_fet;      /* 功率管温度 ℃ */
		float temp_motor;    /* 电机温度 ℃ */
		int32_t multiturn;   /* 多圈计数 (整圈, 带符号) */
		float single;        /* 单圈机械角 rad [0,2π) */
		uint32_t fault_mask; /* 故障位掩码 */
		uint32_t warn_mask;  /* 警告位掩码 */
		uint8_t top_fsm;     /* 顶层状态 top_fsm_e */
		uint8_t run_state;   /* 运行子状态 run_state_e */
		uint8_t ctrl_mode;   /* 当前控制模式 ctrl_mode_e */
		uint8_t enable;      /* 是否使能 */
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

		/* 读调试通道(CMD 0xC9): 把任意观测 float[] 写入 out, 返回 *out_count 个
		 * (每个元素为小端 f32, 帧内按位序拼接, 与 0xCA 遥测 DEBUG 组同构, 主机按
		 * (reply_len-1)/4 解析)。max_count 为 out 容量。可为 NULL(则回通用 ACK)。
		 * 用途: 免改协议加观测点, 任意处写 jm_dbg[i]=变量 即可上位机查看。*/
		jm_err_e (*get_debug)(float *out, uint8_t *out_count, uint8_t max_count);

		/* 批量读参数(CMD 0xE2): 从 start_id 起连续读 count 个, 应答体由实现层组织为
		 * [start_id:u16][count:u8][[type:u8][value]...] 并写入 out, 写回 *out_len。
		 * out 容量为 JM_PAYLOAD_MAX。可为 NULL(回 NACK)。*/
		jm_err_e (*param_read_bulk)(uint16_t start_id, uint16_t count,
		                            uint8_t *out, uint16_t *out_len);

		/* 批量写参数(CMD 0xE3): 从 start_id 起连续写 count 个, values 为按参数表类型
		 * 逐个拼接的原始字节(每个值长度由 (start_id+i) 的类型决定, 字符串亦按完整 size)。
		 * 可为 NULL(回 NACK)。*/
		jm_err_e (*param_write_bulk)(uint16_t start_id, uint16_t count,
		                             const uint8_t *values, uint16_t len);

		/* 电机配置(motor_info)读写 0xE6~0xE8: 与0xE0-0xE5的运行时参数独立, 面向
		 * Flash/EEPROM持久化的硬件配置/校准数据。帧内 value 固定4字节, 固件按字段类型
		 * (u8/i8/u16/i16/u32/i32/f32)自动转换。可为 NULL(回 NACK)。*/
		jm_err_e (*motor_info_read)(uint16_t param_id, uint8_t *value4,
		                            uint8_t *out_type, uint8_t *out_len);
		jm_err_e (*motor_info_write)(uint16_t param_id, const uint8_t *value4, uint8_t len);
		jm_err_e (*motor_info_save)(void);
		/* 批量读/写(CMD 0xE9/0xEA): 从 start_id 起连续读/写 count 个, 每值固定4B。
		 * read 把结果写入 out: [start_id:u16][count:u8][value:4B]..., 写回 *out_len;
		 * write 的 values 为 count*4B 拼接。块内连续ID有效, 跨块间隔返回 BAD_PARAM_ID。
		 * 可为 NULL(回 NACK)。*/
		jm_err_e (*motor_info_read_bulk)(uint16_t start_id, uint16_t count,
		                                 uint8_t *out, uint16_t *out_len);
		jm_err_e (*motor_info_write_bulk)(uint16_t start_id, uint16_t count,
		                                  const uint8_t *values, uint16_t len);
		/* 恢复默认(CMD 0xEB): param_id=0xFFFF 表示全部恢复默认。可为 NULL(回 NACK)。*/
		jm_err_e (*motor_info_reset)(uint16_t param_id);

		/* 重新标定复位(CMD 0xEC): 清除 is_calibrated + 编码器字段，保留电气字段和限幅字段。
		 * 仅清 RAM，不自动落盘，需随后发 0xEA 固化。可为 NULL(回 NACK)。*/
		jm_err_e (*motor_info_recalib_reset)(void);

		/* 设置本机 CAN 地址(CMD 0xF0): new_id 范围 1~127, 需由实现层持久化。
		 * 协议层在成功后同步更新 proto->motor_id; CAN 滤波地址重启后由绑定层重新加载生效。
		 * 可为 NULL(回 NACK)。*/
		jm_err_e (*set_can_id)(uint8_t new_id);

		/* 设置 CAN 波特率(CMD 0xF1): baud_code 0=1M 1=500K 2=250K 3=125K。
		 * 重启后由 CAN 绑定层加载生效。可为 NULL(回 NACK)。*/
		jm_err_e (*set_baudrate)(uint8_t baud_code);

		/* PID 理论估计(CMD 0xA0): 基于辨识参数计算三环PID写入ControlParam_t,
		 * 自动设 source=AUTOTUNE 并 reload。仅IDLE态可执行。
		 * ring_select: 0=电流环 1=速度环 2=位置环 3=全部三环
		 * cur_bw/vel_bw/pos_bw: 各环带宽Hz, <=0用推荐默认值
		 * out_fail_reason: 失败原因输出(0=无,1=辨识未就绪,2=非IDLE态,3=参数无效)
		 * 返回 JM_ERR_OK 成功, 其余失败。可为 NULL(回 NACK)。*/
		jm_err_e (*pid_autotune)(uint8_t ring_select, float cur_bw, float vel_bw, float pos_bw,
		                         uint8_t *out_fail_reason);

		/* PID 来源切换(CMD 0xA1): 独立设置某环参数来源, 立即 reload。仅IDLE态可执行。
		 * ring_select: 0=电流环 1=速度环 2=位置环
		 * source: 0=默认 1=Flash工程值 2=理论估计 3=调试
		 * 返回 JM_ERR_OK 成功, 其余失败。可为 NULL(回 NACK)。*/
		jm_err_e (*pid_source_set)(uint8_t ring_select, uint8_t source);

		/* PID 来源查询(CMD 0xA2): 读取三环当前 source 状态。
		 * 输出 3 字节: cur_src/vel_src/pos_src (0=默认 1=Flash 2=理论估计 3=调试)
		 * 返回 JM_ERR_OK 成功, 其余失败。可为 NULL(回 NACK)。*/
		jm_err_e (*pid_source_get)(uint8_t *out_cur, uint8_t *out_vel, uint8_t *out_pos);

		/* PID 参数实时写(CMD 0xA5): 仅 DEBUG source 下允许写, 直接写 profile。
		 * ring: 0=D轴 1=Q轴 2=速度 3=位置; param_type: 1=kp 2=ki 3=kd
		 * 4=output_limit 5=integral_limit 6=output_filter_alpha 7=flags
		 * value4: 4字节小端值。可为 NULL(回 NACK)。*/
		jm_err_e (*pid_param_set)(uint8_t ring, uint8_t param_type, const uint8_t *value4);

		/* PID 参数实时读(CMD 0xA6): 随时可读, 返回当前 profile 中的值(4字节)。
		 * ring/param_type 同 0xA5。可为 NULL(回 NACK)。*/
		jm_err_e (*pid_param_get)(uint8_t ring, uint8_t param_type, uint8_t *out_value4);
	} jm_proto_ops_t;

	/* ---------------- 协议实例 ---------------- */
	typedef struct jm_proto
	{
		const jm_proto_ops_t *ops; /* 业务回调 */
		uint8_t motor_id;          /* 本机地址(CAN用; 串口可忽略) */
		/* 应答输出缓冲(由 dispatch 填充, 调用方取走发送) */
		uint8_t reply[1 + JM_PAYLOAD_MAX]; /* reply[0]=CMD, 其后为载荷 */
		uint16_t reply_len;                /* 含CMD的总长度; 0 表示无需应答 */
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
