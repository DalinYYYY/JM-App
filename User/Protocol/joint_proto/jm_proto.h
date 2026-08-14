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

/* TRACE 通道位序与 motor_observer_channel_e 对齐。 */
#define JM_TRACE_CH_ID_REF  (1u << 0)
#define JM_TRACE_CH_ID      (1u << 1)
#define JM_TRACE_CH_IQ_REF  (1u << 2)
#define JM_TRACE_CH_IQ      (1u << 3)
#define JM_TRACE_CH_VEL_REF (1u << 4)
#define JM_TRACE_CH_VEL     (1u << 5)
#define JM_TRACE_CH_POS_REF (1u << 6)
#define JM_TRACE_CH_POS     (1u << 7)
#define JM_TRACE_CH_ALL     0xFFu
#define JM_TRACE_FLAG_OVERFLOW       0x01u
#define JM_TRACE_FLAG_DISCONTINUITY  0x02u
#define JM_TRACE_FLAG_LAST            0x04u

	/* ---------------- 实时反馈数据(读命令的数据源) ---------------- */
	typedef struct
	{
		float pos;           /* 电机端多圈位置 θ_m rad (带符号, ±∞) */
		float vel;           /* 电机端机械角速度 rad/s */
		float pos_ref;       /* 位置目标 rad */
		float vel_ref;       /* 速度目标 rad/s */
		float torque;        /* 输出端力矩 Nm */
		float id;            /* d轴电流 A */
		float iq;            /* q轴电流 A */
		float id_ref;        /* d轴电流目标 A */
		float iq_ref;        /* q轴电流目标 A */
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
		/* TRACE 配置(CMD 0xB9): enable=0 停止; enable=1 开始独立批量波形流。
		 * 成功时返回实际采样率、打包点数和缓冲容量。 */
		jm_err_e (*trace_config)(uint8_t enable, uint16_t session_id,
		                         uint32_t ch_mask, uint32_t rate_hz,
		                         uint8_t packet_samples, uint8_t flags,
		                         uint8_t *out, uint16_t *out_len);
		/* 通信任务取一批已采样数据; 不在控制ISR内调用。 */
		jm_err_e (*trace_pop)(uint8_t *out, uint16_t *out_len);

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
		/* 固化(CMD 0xEA): flags 位见 JM_MOTOR_INFO_SAVE_FLAG_*;
		 * bit0=1 追加写 Flash 备份, 默认(0)仅写 EEPROM。 */
		jm_err_e (*motor_info_save)(uint8_t flags);
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

	/* 切换 CAN FD 运行期模式(CMD 0xF3): enable 1=切FD长帧 0=切回经典。
	 * out_ack_enable 输出当前实际模式(0=经典/1=FD), out_cap 输出硬件能力(0/1)。
	 * UART 模式或不支持 FD 的板级配置应返回 cap=0。
	 * 可为 NULL(回 NACK)。*/
	jm_err_e (*set_fd_mode)(uint8_t enable, uint8_t *out_ack_enable, uint8_t *out_cap);

	/* CAN-DI物理设备识别(F6): duration_100ms=0停止，否则让目标设备指示灯闪烁。
	 * 可为NULL，此时CAN绑定层返回UNSUPPORTED。*/
	jm_err_e (*identify_can_device)(uint8_t duration_100ms);

		/* PID 理论估计(CMD 0xA0): 基于辨识参数计算三环PID写入ControlParam_t,
		 * 自动设 source=AUTOTUNE 并 reload。仅IDLE态可执行。
		 * ring_mask: 位掩码 bit0=电流环 bit1=速度环 bit2=位置环(可组合, 如0x05=电流+位置)
		 * cur_bw/vel_bw/pos_bw: 各环带宽Hz, <=0用推荐默认值
		 * out_fail_reason: 失败原因输出(0=无,1=辨识未就绪,2=非IDLE态,3=参数无效)
		 * 返回 JM_ERR_OK 成功, 其余失败。可为 NULL(回 NACK)。*/
		jm_err_e (*pid_autotune)(uint8_t ring_mask, float cur_bw, float vel_bw, float pos_bw,
		                         uint8_t *out_fail_reason);

		/* PID 来源切换(CMD 0xA1): 独立设置某环参数来源, 立即 reload。
		 * 仅 IDLE/READY 态可执行（READY 下 PWM 关闭）；RUN/CALIB/FAULT 拒绝。
		 * ring_select: 0=电流环 1=速度环 2=位置环
		 * source: 0=默认 1=Flash工程值 2=理论估计 3=调试；调试来源由 A2 心跳续租
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
		/* value_len=4 为旧版单字段写；param_type=0 时 value 为原子批量载荷。 */
		jm_err_e (*pid_param_set)(uint8_t ring, uint8_t param_type,
		                          const uint8_t *value, uint16_t value_len);

		/* PID 参数实时读(CMD 0xA6): 随时可读, 返回当前 profile 中的值(4字节)。
		 * ring/param_type 同 0xA5。可为 NULL(回 NACK)。*/
		jm_err_e (*pid_param_get)(uint8_t ring, uint8_t param_type, uint8_t *out_value4);
	} jm_proto_ops_t;

	/* ---------------- 协议实例 ---------------- */
	typedef struct jm_proto
	{
		const jm_proto_ops_t *ops; /* 业务回调 */
		uint8_t motor_id;          /* 本机地址(CAN用; 串口可忽略) */
		/* 异步命令序列号: 0=同步命令(无 seq), 1~255=异步命令递增循环。
		 * 由 jm_proto_async_alloc_seq() 分配; PENDING 即时 NACK 与最终 ACK/NACK 携带同一 seq。
		 * 上位机据此匹配请求与最终应答, 防止应答丢失导致重试重复执行(如 Flash 擦写)。*/
		uint8_t async_seq;
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

	/* ---------------- 载荷长度校验规范 ----------------
	 * dispatch 内部统一用 JM_CHECK_LEN 宏(jm_proto.c 内定义, 因依赖 static reply_nack)
	 * 应用层 ops 回调中需自行校验 len, 不足时返回 JM_ERR_LENGTH, dispatch 会自动转 NACK。
	 * 严禁用裸 if(len<x) return ERR_LENGTH 而不回 NACK, 否则上位机收不到应答误以为丢帧。 */

	/* ---------------- 字段演进四铁律(编码规范) ----------------
	 * 1. 新增字段必须追加到结构体末尾, 禁止中间插入或重排已有字段顺序
	 * 2. 已有字段的语义和类型不可变更; 若需变更, 必须新增命令码或新增字段
	 * 3. 新增字段必须有"默认值"(0 或合理初值), 老固件读到新命令的扩展部分自动忽略
	 * 4. 命令码一旦定义并发布不可重用; 已废弃命令码标记 reserved, 不可重新分配
	 * 例外: 协议主版本号(MAJOR)递增时允许破坏性变更, 但需同步更新 JM_PROTO_VERSION_MAJOR */

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

	/* ---------------- 异步命令序列号机制 ----------------
	 * 异步命令双时序协议:
	 *   1. 收到命令立即校验, 失败 -> 即时 NACK(err_code, seq=0)
	 *   2. 校验通过, 排队异步执行 -> 即时 NACK(PENDING, seq=N)
	 *   3. 异步任务完成 -> 最终 ACK(seq=N) 或 NACK(err_code, seq=N)
	 * seq 由 jm_proto_async_alloc_seq() 分配, 1~255 循环(0 保留给同步命令)。
	 * 上位机通过 seq 匹配 PENDING 与最终应答; 若最终应答丢失, 重试时下位机 seq 已递增,
	 * 上位机可据此识别"上次已执行"避免重复 Flash 擦写。
	 *
	 * 开发期简化: 仅引入 seq 基础设施 + API, 不强制所有耗时命令改异步。
	 *   - 标定命令(0x90~0x96): 已是异步(切 CALIB 态由状态机 poll), 暂不回 PENDING,
	 *     保留同步 ACK + 0x97 进度查询机制。
	 *   - Flash 保存(0xE4/0xEA): 开发期通信负载低, 暂保持同步执行;
	 *     量产期若性能问题再改异步 flag + PENDING。
	 *   - 预留命令(0x80~0x82/0xCC~0xCF): 回 NACK(NOT_SUPPORTED, seq=0)。 */

	/**
	 * @brief  分配新的异步序列号 (1~255 循环, 0 保留给同步命令)
	 * @param  proto  实例
	 * @return 新 seq (1~255); 同时写入 proto->async_seq
	 * @note   线程安全: 仅在通信线程上下文调用; ISR 中不可调用。
	 *         典型用法: ops 回调排队异步任务时调用, 把返回 seq 存入异步上下文,
	 *         最终应答时携带同一 seq。
	 */
	uint8_t jm_proto_async_alloc_seq(jm_proto_t *proto);

	/**
	 * @brief  组织异步命令的即时 PENDING 应答 (NACK with err_code=PENDING + seq)
	 * @param  proto  实例
	 * @param  cmd    原命令码
	 * @param  seq    异步序列号(由 jm_proto_async_alloc_seq 分配)
	 * @return JM_ERR_PENDING (同时填充 proto->reply)
	 * @note   应答格式: [0xFE][cmd][0x0B][seq] (4B)
	 */
	jm_err_e jm_proto_reply_pending(jm_proto_t *proto, uint8_t cmd, uint8_t seq);

	/**
	 * @brief  组织异步命令的最终 ACK (携带 seq, 由通信线程在异步任务完成后调用)
	 * @param  proto    实例
	 * @param  cmd      原命令码
	 * @param  status   状态字节(0=成功, 其余由命令定义; 写入 ACK 载荷首字节)
	 * @param  seq      异步序列号(与 PENDING 携带的 seq 一致)
	 * @return JM_ERR_OK (同时填充 proto->reply)
	 * @note   应答格式: [cmd][seq][status] (3B); 调用方需主动发送 proto->reply。
	 *         与同步 ACK(2B: [cmd][status]) 区别: 异步 ACK 多 1 字节 seq。
	 */
	jm_err_e jm_proto_reply_ack_async(jm_proto_t *proto, uint8_t cmd, uint8_t status, uint8_t seq);

	/**
	 * @brief  组织异步命令的最终 NACK (携带 seq, 由通信线程在异步任务完成后调用)
	 * @param  proto    实例
	 * @param  cmd      原命令码
	 * @param  err      错误码(具体失败原因)
	 * @param  seq      异步序列号(与 PENDING 携带的 seq 一致)
	 * @return err (同时填充 proto->reply)
	 * @note   应答格式: [0xFE][cmd][err_code][seq] (4B); 与同步 NACK(3B, seq=0) 区别: 多 1 字节 seq。
	 */
	jm_err_e jm_proto_reply_nack_async(jm_proto_t *proto, uint8_t cmd, jm_err_e err, uint8_t seq);

#ifdef __cplusplus
}
#endif
#endif /* __JM_PROTO_H__ */
