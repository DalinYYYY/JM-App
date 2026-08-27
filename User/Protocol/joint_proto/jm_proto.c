/**
 * @file        jm_proto.c
 * @brief       关节电机通信协议核心: 传输无关的 CMD 分发与应答组织
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
 * | 2026-06-25 | 1.1  | Dalin  | 补全 0xC9/0xE2/0xE3/0xF0/0xF1 分发 |
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 */
#include <string.h>
#include "jm_proto.h"
#include "calib_mgr.h"

/* 组织应答: reply[0]=cmd, 其后拷贝 body(可空), 设置 reply_len */
static void reply_set(jm_proto_t *p, uint8_t cmd, const uint8_t *body, uint16_t body_len)
{
	if (body_len > JM_PAYLOAD_MAX)
	{
		body_len = JM_PAYLOAD_MAX;
	}
	p->reply[0] = cmd;
	if (body != NULL && body_len > 0)
	{
		memcpy(&p->reply[1], body, body_len);
	}
	p->reply_len = (uint16_t)(1 + body_len);
}

/* 载荷长度校验宏 (内部使用, 依赖 static reply_nack)
 * 用法: if (JM_CHECK_LEN(len, 8, cmd, p)) return err; 或直接 JM_CHECK_LEN(len, 8, cmd, p);
 * 长度不足时回 NACK(LENGTH) 并返回 err (err 已是 reply_nack 的返回值) */
#define JM_CHECK_LEN(len_var, expected_min, cmd_var, p_var) \
	do { \
		if ((len_var) < (expected_min)) { \
			return reply_nack((p_var), (cmd_var), JM_ERR_LENGTH); \
		} \
	} while (0)

/* 组织 NACK 应答: [0xFE][失败的cmd][err_code][seq] (4B 格式)
 * seq: 异步命令的序列号, 同步命令填 0 */
static jm_err_e reply_nack(jm_proto_t *p, uint8_t cmd, jm_err_e err)
{
	uint8_t body[3] = {cmd, (uint8_t)err, 0u}; /* seq=0 (同步命令) */
	reply_set(p, JM_CMD_NACK, body, sizeof(body));
	return err;
}

/* 组织最简 ACK: [cmd][状态/0] */
static jm_err_e reply_ack(jm_proto_t *p, uint8_t cmd, uint8_t status)
{
	reply_set(p, cmd, &status, 1);
	return JM_ERR_OK;
}

/* 把实时反馈打包成应答载荷(串口全精度 f32) */
static uint16_t pack_feedback(const jm_feedback_t *fb, uint8_t *o)
{
	uint16_t n = 0;
	jm_wr_f32(&o[n], fb->pos);
	n += 4;
	jm_wr_f32(&o[n], fb->vel);
	n += 4;
	jm_wr_f32(&o[n], fb->torque);
	n += 4;
	jm_wr_f32(&o[n], fb->temp_motor);
	n += 4;
	jm_wr_f32(&o[n], fb->vbus);
	n += 4;
	jm_wr_u16(&o[n], (uint16_t)fb->fault_mask);
	n += 2;
	return n; /* 22 字节 */
}

void jm_proto_init(jm_proto_t *proto, const jm_proto_ops_t *ops, uint8_t motor_id)
{
	if (proto == NULL)
	{
		return;
	}
	memset(proto, 0, sizeof(*proto));
	proto->ops = ops;
	proto->motor_id = motor_id;
	/* async_seq 初始为 0(同步命令); 首次 alloc 返回 1 */
}

/* ===================== 异步命令序列号机制 ===================== */
jm_err_e jm_proto_reply_nack_async(jm_proto_t *p, uint8_t cmd, jm_err_e err, uint8_t seq)
{
	uint8_t body[3] = {cmd, (uint8_t)err, seq}; /* [orig_cmd][err_code][seq] */
	reply_set(p, JM_CMD_NACK, body, sizeof(body));
	return err;
}

jm_err_e jm_proto_reply_pending(jm_proto_t *p, uint8_t cmd, uint8_t seq)
{
	/* PENDING 即时应答 = NACK with err_code=PENDING + seq */
	return jm_proto_reply_nack_async(p, cmd, JM_ERR_PENDING, seq);
}

jm_err_e jm_proto_reply_ack_async(jm_proto_t *p, uint8_t cmd, uint8_t status, uint8_t seq)
{
	/* 异步最终 ACK 载荷: [seq][status] (2B), 加上 reply[0]=cmd 共 3B
	 * 与同步 ACK(2B: [cmd][status]) 区别: 多 1 字节 seq, 上位机据此匹配 PENDING */
	uint8_t body[2] = {seq, status};
	reply_set(p, cmd, body, sizeof(body));
	return JM_ERR_OK;
}

uint8_t jm_proto_async_alloc_seq(jm_proto_t *proto)
{
	uint8_t seq;
	if (proto == NULL)
	{
		return 0;
	}
	/* 1~255 循环递增; 0 保留给同步命令 */
	seq = (uint8_t)(proto->async_seq + 1u);
	if (seq == 0u)
	{
		seq = 1u;
	}
	proto->async_seq = seq;
	return seq;
}

/* ---- 反馈查询类 0xC0~0xC8 ---- */
static jm_err_e handle_read(jm_proto_t *p, uint8_t cmd)
{
	jm_feedback_t fb;
	uint8_t o[32];
	uint16_t n = 0;
	jm_err_e e;

	if (p->ops == NULL || p->ops->get_feedback == NULL)
	{
		return reply_nack(p, cmd, JM_ERR_UNSUPPORTED);
	}
	memset(&fb, 0, sizeof(fb));
	e = p->ops->get_feedback(&fb);
	if (e != JM_ERR_OK)
	{
		return reply_nack(p, cmd, e);
	}

	switch (cmd)
	{
		case JM_CMD_READ_FEEDBACK:
			n = pack_feedback(&fb, o);
			break;
		case JM_CMD_READ_STATE:
			o[0] = fb.top_fsm;
			o[1] = fb.run_state;
			o[2] = fb.ctrl_mode;
			o[3] = fb.enable;
			n = 4;
			break;
		case JM_CMD_READ_PHASE_CURRENT:
			jm_wr_f32(&o[0], fb.ia);
			jm_wr_f32(&o[4], fb.ib);
			jm_wr_f32(&o[8], fb.ic);
			n = 12;
			break;
		case JM_CMD_READ_DQ_CURRENT:
			jm_wr_f32(&o[0], fb.id);
			jm_wr_f32(&o[4], fb.iq);
			n = 8;
			break;
		case JM_CMD_READ_BUS:
			jm_wr_f32(&o[0], fb.vbus);
			jm_wr_f32(&o[4], fb.ibus);
			jm_wr_f32(&o[8], fb.vbus * fb.ibus);
			n = 12;
			break;
		case JM_CMD_READ_TEMPERATURE:
			jm_wr_f32(&o[0], fb.temp_fet);
			jm_wr_f32(&o[4], fb.temp_motor);
			n = 8;
			break;
		case JM_CMD_READ_POS_VEL:
			jm_wr_f32(&o[0], fb.pos);
			jm_wr_f32(&o[4], fb.vel);
			n = 8;
			break;
		case JM_CMD_READ_MULTITURN:
			jm_wr_u32(&o[0], (uint32_t)fb.multiturn);
			jm_wr_f32(&o[4], fb.single);
			n = 8;
			break;
		case JM_CMD_READ_FAULT:
			jm_wr_u32(&o[0], fb.fault_mask);
			jm_wr_u32(&o[4], fb.warn_mask);
			n = 8;
			break;
		default:
			return reply_nack(p, cmd, JM_ERR_UNSUPPORTED);
	}
	reply_set(p, cmd, o, n);
	return JM_ERR_OK;
}

/* ---- 参数读写类 0xE0~0xE5 ---- */
static jm_err_e handle_param(jm_proto_t *p, uint8_t cmd, const uint8_t *pl, uint16_t len)
{
	const jm_proto_ops_t *ops = p->ops;
	uint16_t pid;
	jm_err_e e;

	switch (cmd)
	{
		case JM_CMD_PARAM_READ:
		{
			uint8_t o[3 + 16];
			uint8_t type = 0, vlen = 0;
			if (len < 2)
				return reply_nack(p, cmd, JM_ERR_LENGTH);
			if (ops == NULL || ops->param_read == NULL)
				return reply_nack(p, cmd, JM_ERR_UNSUPPORTED);
			pid = jm_rd_u16(pl);
			e = ops->param_read(pid, &o[3], &type, &vlen);
			if (e != JM_ERR_OK)
				return reply_nack(p, cmd, e);
			jm_wr_u16(&o[0], pid);
			o[2] = type;
			reply_set(p, cmd, o, (uint16_t)(3 + vlen));
			return JM_ERR_OK;
		}
		case JM_CMD_PARAM_WRITE:
		{
			uint8_t o[3];
			if (len < 3)
				return reply_nack(p, cmd, JM_ERR_LENGTH);
			if (ops == NULL || ops->param_write == NULL)
				return reply_nack(p, cmd, JM_ERR_UNSUPPORTED);
			pid = jm_rd_u16(pl);
			e = ops->param_write(pid, &pl[2], (uint8_t)(len - 2));
			jm_wr_u16(&o[0], pid);
			o[2] = (uint8_t)e;
			reply_set(p, cmd, o, 3);
			return e;
		}
		case JM_CMD_PARAM_SAVE:
			if (ops == NULL || ops->param_save == NULL)
				return reply_nack(p, cmd, JM_ERR_UNSUPPORTED);
			e = ops->param_save();
			return (e == JM_ERR_OK) ? reply_ack(p, cmd, 0) : reply_nack(p, cmd, e);
		case JM_CMD_PARAM_RESET:
			if (len < 2)
				return reply_nack(p, cmd, JM_ERR_LENGTH);
			if (ops == NULL || ops->param_reset == NULL)
				return reply_nack(p, cmd, JM_ERR_UNSUPPORTED);
			e = ops->param_reset(jm_rd_u16(pl));
			return (e == JM_ERR_OK) ? reply_ack(p, cmd, 0) : reply_nack(p, cmd, e);
		/* 批量读参数 0xE2: {start_id:u16;count:u16} -> {start_id:u16;count:u8;[type:u8;value]...} */
		case JM_CMD_PARAM_READ_BULK:
		{
			uint16_t start_id, count, n = 0;
			uint8_t o[JM_PAYLOAD_MAX];
			if (len < 4)
				return reply_nack(p, cmd, JM_ERR_LENGTH);
			if (ops == NULL || ops->param_read_bulk == NULL)
				return reply_nack(p, cmd, JM_ERR_UNSUPPORTED);
			start_id = jm_rd_u16(&pl[0]);
			count = jm_rd_u16(&pl[2]);
			e = ops->param_read_bulk(start_id, count, o, &n);
			if (e != JM_ERR_OK)
				return reply_nack(p, cmd, e);
			reply_set(p, cmd, o, n);
			return JM_ERR_OK;
		}
		/* 批量写参数 0xE3: {start_id:u16;count:u16;values:bytes} -> ACK{status:u8} */
		case JM_CMD_PARAM_WRITE_BULK:
		{
			uint16_t start_id, count;
			uint8_t status;
			if (len < 4)
				return reply_nack(p, cmd, JM_ERR_LENGTH);
			if (ops == NULL || ops->param_write_bulk == NULL)
				return reply_nack(p, cmd, JM_ERR_UNSUPPORTED);
			start_id = jm_rd_u16(&pl[0]);
			count = jm_rd_u16(&pl[2]);
			e = ops->param_write_bulk(start_id, count, &pl[4], (uint16_t)(len - 4));
			status = (uint8_t)e;
			reply_set(p, cmd, &status, 1);
			return e;
		}
		/* 电机配置读 0xE6: {param_id:u16} -> {param_id:u16;type:u8;value:4B} (固定4字节值) */
		case JM_CMD_MOTOR_INFO_READ:
		{
			uint8_t o[3 + 4];
			uint8_t type = 0, vlen = 0;
			if (len < 2)
				return reply_nack(p, cmd, JM_ERR_LENGTH);
			if (ops == NULL || ops->motor_info_read == NULL)
				return reply_nack(p, cmd, JM_ERR_UNSUPPORTED);
			pid = jm_rd_u16(pl);
			e = ops->motor_info_read(pid, &o[3], &type, &vlen);
			if (e != JM_ERR_OK)
				return reply_nack(p, cmd, e);
			jm_wr_u16(&o[0], pid);
			o[2] = type;
			reply_set(p, cmd, o, (uint16_t)(3 + vlen));
			return JM_ERR_OK;
		}
		/* 电机配置写 0xE7: {param_id:u16;value:4B} -> ACK{param_id:u16;status:u8} (固定4字节值) */
		case JM_CMD_MOTOR_INFO_WRITE:
		{
			uint8_t o[3];
			if (len < 6)
				return reply_nack(p, cmd, JM_ERR_LENGTH);
			if (ops == NULL || ops->motor_info_write == NULL)
				return reply_nack(p, cmd, JM_ERR_UNSUPPORTED);
			pid = jm_rd_u16(pl);
			e = ops->motor_info_write(pid, &pl[2], (uint8_t)(len - 2));
			jm_wr_u16(&o[0], pid);
			o[2] = (uint8_t)e;
			reply_set(p, cmd, o, 3);
			return e;
		}
		/* 批量读电机配置 0xE8: {start_id:u16;count:u16} -> {start_id:u16;count:u8;[value:4B]...} */
		case JM_CMD_MOTOR_INFO_READ_BULK:
		{
			uint16_t start_id, count, n = 0;
			uint8_t o[JM_PAYLOAD_MAX];
			if (len < 4)
				return reply_nack(p, cmd, JM_ERR_LENGTH);
			if (ops == NULL || ops->motor_info_read_bulk == NULL)
				return reply_nack(p, cmd, JM_ERR_UNSUPPORTED);
			start_id = jm_rd_u16(&pl[0]);
			count = jm_rd_u16(&pl[2]);
			e = ops->motor_info_read_bulk(start_id, count, o, &n);
			if (e != JM_ERR_OK)
				return reply_nack(p, cmd, e);
			reply_set(p, cmd, o, n);
			return JM_ERR_OK;
		}
		/* 批量写电机配置 0xE9: {start_id:u16;count:u16;[value:4B]...} -> ACK{status:u8} */
		case JM_CMD_MOTOR_INFO_WRITE_BULK:
		{
			uint16_t start_id, count;
			uint8_t status;
			if (len < 4)
				return reply_nack(p, cmd, JM_ERR_LENGTH);
			if (ops == NULL || ops->motor_info_write_bulk == NULL)
				return reply_nack(p, cmd, JM_ERR_UNSUPPORTED);
			start_id = jm_rd_u16(&pl[0]);
			count = jm_rd_u16(&pl[2]);
			e = ops->motor_info_write_bulk(start_id, count, &pl[4], (uint16_t)(len - 4));
			status = (uint8_t)e;
			reply_set(p, cmd, &status, 1);
			return e;
		}
		/* 电机配置固化 0xEA: {flags:u8 可选} -> ACK{status:u8}
		 * flags bit0=1 追加写 Flash 备份, 默认(无载荷/bit0=0)仅写 EEPROM */
		case JM_CMD_MOTOR_INFO_SAVE:
			if (ops == NULL || ops->motor_info_save == NULL)
				return reply_nack(p, cmd, JM_ERR_UNSUPPORTED);
			if (len > 1)
				return reply_nack(p, cmd, JM_ERR_LENGTH);
			e = ops->motor_info_save((len >= 1) ? pl[0] : 0u);
			return (e == JM_ERR_OK) ? reply_ack(p, cmd, 0) : reply_nack(p, cmd, e);
		/* 电机配置恢复默认 0xEB: {param_id:u16=0xFFFF全部} -> ACK{status:u8} */
	case JM_CMD_MOTOR_INFO_RESET:
		if (len < 2)
			return reply_nack(p, cmd, JM_ERR_LENGTH);
		if (ops == NULL || ops->motor_info_reset == NULL)
			return reply_nack(p, cmd, JM_ERR_UNSUPPORTED);
		e = ops->motor_info_reset(jm_rd_u16(pl));
		return (e == JM_ERR_OK) ? reply_ack(p, cmd, 0) : reply_nack(p, cmd, e);
	/* 重新标定复位 0xEC: 无载荷 -> ACK{status:u8}
	 * 清除 is_calibrated + 编码器字段，保留电气字段和限幅字段。
	 * 仅清 RAM，需随后发 0xEA 固化。*/
	case JM_CMD_MOTOR_INFO_RECALIB_RESET:
		if (ops == NULL || ops->motor_info_recalib_reset == NULL)
			return reply_nack(p, cmd, JM_ERR_UNSUPPORTED);
		e = ops->motor_info_recalib_reset();
		return (e == JM_ERR_OK) ? reply_ack(p, cmd, 0) : reply_nack(p, cmd, e);
		default:
			return reply_nack(p, cmd, JM_ERR_UNSUPPORTED);
	}
}

/* ---- 设备信息类 0xD0~0xD2 ----
 * 0xD0 READ_DEV_INFO 应答格式 (向后兼容追加, 协议层不依赖板级宏):
 *   原始格式 (20B, 老上位机): [0-3]hw [4-7]fw [8-19]uid[12]
 *   扩展格式 (28B, 新上位机): 追加 [20]motor_id_default [21]proto_major [22]proto_minor
 *                                  [23]feat_lo [24]feat_hi [25-27]reserved
 * 协议层始终返回 28B 扩展格式; 老上位机读前 20B 即可, 后 8B 自动忽略。
 * JM_FEATURE_FLAGS_LO 在 jm_cmd_def.h 中根据 USE_CAN_FD_MODE 编译期决定,
 * 若板级未启用 FD, bit0(CAN_FD)=0, 上位机据此关闭 FD 模式适配。 */
static jm_err_e handle_dev(jm_proto_t *p, uint8_t cmd)
{
	const jm_proto_ops_t *ops = p->ops;
	switch (cmd)
	{
		case JM_CMD_READ_DEV_INFO:
		{
			uint8_t o[28];
			uint32_t hw = 0, fw = 0;
			uint8_t uid[12] = {0};
			if (ops == NULL || ops->get_dev_info == NULL)
				return reply_nack(p, cmd, JM_ERR_UNSUPPORTED);
			if (ops->get_dev_info(&hw, &fw, uid) != JM_ERR_OK)
				return reply_nack(p, cmd, JM_ERR_STATE_DENY);
			jm_wr_u32(&o[0], hw);
			jm_wr_u32(&o[4], fw);
			memcpy(&o[8], uid, 12);
			/* 扩展字段 (motor_id_default + proto_version + feature_flags) */
			o[20] = p->motor_id;
			o[21] = JM_PROTO_VERSION_MAJOR;
			o[22] = JM_PROTO_VERSION_MINOR;
			o[23] = (uint8_t)(JM_FEATURE_FLAGS_LO & 0xFF);
			o[24] = (uint8_t)((JM_FEATURE_FLAGS_LO >> 8) & 0xFF);
			o[25] = 0; /* reserved */
			o[26] = 0;
			o[27] = 0;
			reply_set(p, cmd, o, 28); /* 始终 28B (向后兼容追加) */
			return JM_ERR_OK;
		}
		case JM_CMD_READ_DEV_NAME:
		{
			const char *name = (ops && ops->get_dev_name) ? ops->get_dev_name() : NULL;
			uint8_t o[16] = {0};
			if (name == NULL)
				return reply_nack(p, cmd, JM_ERR_UNSUPPORTED);
			strncpy((char *)o, name, sizeof(o) - 1);
			reply_set(p, cmd, o, sizeof(o));
			return JM_ERR_OK;
		}
		case JM_CMD_HEARTBEAT:
		{
			jm_feedback_t fb;
			uint8_t o[7];
			memset(&fb, 0, sizeof(fb));
			if (ops && ops->get_feedback)
				ops->get_feedback(&fb);
			o[0] = fb.top_fsm;
			jm_wr_u16(&o[1], (uint16_t)fb.fault_mask);
			jm_wr_u32(&o[3], 0);
			reply_set(p, cmd, o, sizeof(o));
			return JM_ERR_OK;
		}
		default:
			return reply_nack(p, cmd, JM_ERR_UNSUPPORTED);
	}
}

/* ---- 调试通道 0xC9: 通用 float[] 观测点, 无专用回调时回通用 ACK ---- */
static jm_err_e handle_read_debug(jm_proto_t *p, uint8_t cmd)
{
	/* 无专用返回时按 CSV #88 约定回通用 ACK */
	if (p->ops == NULL || p->ops->get_debug == NULL)
	{
		return reply_ack(p, cmd, 0);
	}
	{
		float dbg[16]; /* 容量 16; 若 JM_DBG_CH 增大需同步扩容 */
		uint8_t o[sizeof(dbg)];
		uint8_t cnt = 0;
		uint8_t cap = (uint8_t)(sizeof(dbg) / sizeof(float));
		uint16_t n = 0;
		uint8_t i;
		jm_err_e e;
		e = p->ops->get_debug(dbg, &cnt, cap);
		if (e != JM_ERR_OK)
		{
			return reply_nack(p, cmd, e);
		}
		if (cnt > cap)
		{
			cnt = cap;
		}
		/* 帧内仅 raw f32 拼接(无计数字节), 与 0xCA 遥测 DEBUG 组同构 */
		for (i = 0; i < cnt; i++)
		{
			jm_wr_f32(&o[n], dbg[i]);
			n += 4;
		}
		reply_set(p, cmd, o, n);
		return JM_ERR_OK;
	}
}

/* ---- 主分发: 串口/CAN 解出 cmd+payload 后统一进这里 ---- */
jm_err_e jm_proto_dispatch(jm_proto_t *proto, uint8_t cmd, const uint8_t *payload, uint16_t len)
{
	if (proto == NULL)
	{
		return JM_ERR_STATE_DENY;
	}
	proto->reply_len = 0; /* 默认无应答 */

	/* 软件复位 0x07: {magic:u32=0x5E7E7E5E} -> ACK, 随后延迟约 200ms 执行复位。
	 * 回调内已安全停机; 魔数不匹配回 NACK(UNAUTHORIZED)。
	 * CAN 广播(motor_id=0)对本命令不开放(见 jm_proto_can_broadcast_allowed)。 */
	if (cmd == JM_CMD_SOFT_RESET)
	{
		uint32_t magic;
		jm_err_e e;
		if (len < 4u)
		{
			return reply_nack(proto, cmd, JM_ERR_LENGTH);
		}
		if (proto->ops == NULL || proto->ops->soft_reset == NULL)
		{
			return reply_nack(proto, cmd, JM_ERR_UNSUPPORTED);
		}
		magic = jm_rd_u32(&payload[0]);
		e = proto->ops->soft_reset(magic);
		return (e == JM_ERR_OK) ? reply_ack(proto, cmd, 0) : reply_nack(proto, cmd, e);
	}

	/* 订阅同步遥测 0xCB: payload = enable(u8) + mask(u16) [+ period_ms(u16)], 小端。
	 * enable=1 启动周期上报, enable=0 停止; 仅本订阅命令回单次 ACK 供上位机确认开关,
	 * 之后的周期性 0xCA 数据帧由绑定层主动推送, 不要求逐帧应答。*/
	if (cmd == JM_CMD_SET_TELEMETRY)
	{
		uint8_t enable;
		uint16_t mask, period = 0;
		if (len < 3)
		{
			return reply_nack(proto, cmd, JM_ERR_LENGTH);
		}
		if (proto->ops == NULL || proto->ops->set_telemetry == NULL)
		{
			return reply_nack(proto, cmd, JM_ERR_UNSUPPORTED);
		}
		enable = payload[0];
		mask = jm_rd_u16(&payload[1]);
		if (len >= 5)
		{
			period = jm_rd_u16(&payload[3]);
		}
		{
			jm_err_e e = proto->ops->set_telemetry(enable, mask, period);
			return (e == JM_ERR_OK) ? reply_ack(proto, cmd, 0) : reply_nack(proto, cmd, e);
		}
	}

	/* 反馈查询 0xC0~0xC8 */
	if (cmd >= JM_CMD_READ_FEEDBACK && cmd <= JM_CMD_READ_FAULT)
	{
		if (len != 0u)
			return reply_nack(proto, cmd, JM_ERR_LENGTH);
		return handle_read(proto, cmd);
	}
	/* C9 仅保留通用 jm_dbg 查询，不再承载高速波形读取。 */
	if (cmd == JM_CMD_READ_DEBUG)
	{
		if (len != 0u)
			return reply_nack(proto, cmd, JM_ERR_LENGTH);
		return handle_read_debug(proto, cmd);
	}
	/* TEST_SWEEP_FREQ 0x76: 专用 ACK 与严格长度分支，避免通用 set_mode
	 * 只能返回单字节状态而丢失 session/点数/时长。 */
	if (cmd == JM_CMD_TEST_SWEEP_FREQ)
	{
		uint8_t out[8];
		uint16_t out_len = 0u;
		jm_err_e e;
		uint8_t enable = (len > 0u) ? (payload[0] & JM_SWEEP_CONTROL_ENABLE) : 0u;
		uint8_t length_ok = 0u;

		if (len == JM_SWEEP_PAYLOAD_LEN)
			length_ok = 1u;
		else if (len == JM_SWEEP_STOP_PAYLOAD_LEN && enable == 0u)
			length_ok = 1u;
		else if (len == JM_SWEEP_BIAS_PAYLOAD_LEN)
			length_ok = 1u;
		if (!length_ok)
			return reply_nack(proto, cmd, JM_ERR_LENGTH);
#if (JM_ENABLE_BODE_SWEEP != 1)
		return reply_nack(proto, cmd, JM_ERR_NOT_SUPPORTED);
#else
		if (proto->ops == NULL || proto->ops->test_sweep == NULL)
			return reply_nack(proto, cmd, JM_ERR_NOT_SUPPORTED);
		e = proto->ops->test_sweep(payload, len, out, &out_len);
		if (e != JM_ERR_OK)
			return reply_nack(proto, cmd, e);
		if (out_len > sizeof(out))
			return reply_nack(proto, cmd, JM_ERR_LENGTH);
		reply_set(proto, cmd, out, out_len);
		return JM_ERR_OK;
#endif
	}
	/* TRACE_CONFIG 0xB9: enable + session + mask + rate + packet + flags。 */
	if (cmd == JM_CMD_TRACE_CONFIG)
	{
		uint8_t out[JM_PAYLOAD_MAX];
		uint16_t out_len = 0u;
		jm_err_e e;
		if (len != 13u)
			return reply_nack(proto, cmd, JM_ERR_LENGTH);
		if (proto->ops == NULL || proto->ops->trace_config == NULL)
			return reply_nack(proto, cmd, JM_ERR_UNSUPPORTED);
		e = proto->ops->trace_config(payload[0], jm_rd_u16(&payload[1]),
			jm_rd_u32(&payload[3]), jm_rd_u32(&payload[7]), payload[11],
			payload[12], out, &out_len);
		if (e != JM_ERR_OK)
			return reply_nack(proto, cmd, e);
		reply_set(proto, cmd, out, out_len);
		return JM_ERR_OK;
	}
	/* B5~B7 旧版采集命令不再支持, B8 单步调试仍由 set_mode 处理。 */
	if (cmd == JM_CMD_START_LOG || cmd == JM_CMD_STOP_LOG ||
		cmd == JM_CMD_HIGH_SPEED_DAQ)
		return reply_nack(proto, cmd, JM_ERR_NOT_SUPPORTED);
	/* 设备信息 0xD0~0xDF */
	if (cmd >= JM_CMD_READ_DEV_INFO && cmd <= JM_CMD_HEARTBEAT)
	{
		if (len != 0u)
			return reply_nack(proto, cmd, JM_ERR_LENGTH);
		return handle_dev(proto, cmd);
	}
	/* 参数读写 0xE0~0xEF */
	if (cmd >= JM_CMD_PARAM_READ && cmd <= JM_CMD_MOTOR_INFO_RECALIB_RESET)
	{
		return handle_param(proto, cmd, payload, len);
	}
	/* 广播同步 0xF2: 不应答 */
	if (cmd == JM_CMD_BROADCAST_SYNC)
	{
		if (proto->ops && proto->ops->set_mode)
		{
			proto->ops->set_mode(cmd, payload, len);
		}
		return JM_ERR_OK;
	}
	/* 设置 CAN_ID 0xF0: {new_id:u8} -> ACK{new_id:u8,restart_required:u8}。
	 * 回调原子写入 motor_info Flash。运行期地址保持不变, 重启后加载新地址。 */
	if (cmd == JM_CMD_SET_CAN_ID)
	{
		uint8_t new_id;
		jm_err_e e;
		if (len < 1)
		{
			return reply_nack(proto, cmd, JM_ERR_LENGTH);
		}
		if (proto->ops == NULL || proto->ops->set_can_id == NULL)
		{
			return reply_nack(proto, cmd, JM_ERR_UNSUPPORTED);
		}
		new_id = payload[0];
		if (new_id < 1u || new_id > 127u)
		{
			return reply_nack(proto, cmd, JM_ERR_OUT_OF_RANGE);
		}
		e = proto->ops->set_can_id(new_id);
		if (e != JM_ERR_OK)
		{
			return reply_nack(proto, cmd, e);
		}
		{
			uint8_t o[2] = {new_id, 1u};
			reply_set(proto, cmd, o, sizeof(o));
			return JM_ERR_OK;
		}
	}
	/* 设置波特率 0xF1: {baud_code:u8} -> ACK; 0=1M 1=500K 2=250K 3=125K, 重启生效 */
	if (cmd == JM_CMD_SET_BAUDRATE)
	{
		uint8_t baud;
		jm_err_e e;
		if (len < 1)
		{
			return reply_nack(proto, cmd, JM_ERR_LENGTH);
		}
		if (proto->ops == NULL || proto->ops->set_baudrate == NULL)
		{
			return reply_nack(proto, cmd, JM_ERR_UNSUPPORTED);
		}
		baud = payload[0];
		if (baud > 3u)
		{
			return reply_nack(proto, cmd, JM_ERR_OUT_OF_RANGE);
		}
		e = proto->ops->set_baudrate(baud);
		return (e == JM_ERR_OK) ? reply_ack(proto, cmd, 0) : reply_nack(proto, cmd, e);
	}
	/* 切换 CAN FD 模式 0xF3: {enable:u8} -> ACK{ack_enable:u8, cap:u8}
	 * 下位机在 ACK 后才切换模式(ACK 用旧模式发出); UART 模式返回 cap=0 */
	if (cmd == JM_CMD_SET_FD_MODE)
	{
		uint8_t enable;
		uint8_t ack_enable = 0u;
		uint8_t cap = 0u;
		jm_err_e e;
		if (len < 1)
		{
			return reply_nack(proto, cmd, JM_ERR_LENGTH);
		}
		if (proto->ops == NULL || proto->ops->set_fd_mode == NULL)
		{
			return reply_nack(proto, cmd, JM_ERR_UNSUPPORTED);
		}
		enable = payload[0] ? 1u : 0u;
		e = proto->ops->set_fd_mode(enable, &ack_enable, &cap);
		if (e != JM_ERR_OK)
		{
			return reply_nack(proto, cmd, e);
		}
		/* ACK{ack_enable, cap} */
		{
			uint8_t ack[2] = {ack_enable, cap};
			reply_set(proto, cmd, ack, sizeof(ack));
		}
		return JM_ERR_OK;
	}

	/* 标定进度查询 0x97: 直接返回 8 字节详细状态 ACK, 不走 ops->set_mode。
	 * 字段: state/fail_reason/progress/level/submode/step/step_total/reserved。
	 * 无论 state 为何(空闲/进行/完成/失败)都回 ACK, 由上位机解读。*/
	if (cmd == JM_CMD_CALIB_QUERY)
	{
		if (len != 0u)
		{
			return reply_nack(proto, cmd, JM_ERR_LENGTH);
		}
		calib_status_t st = calib_mgr_get_status();
		uint8_t body[8] = {
			(uint8_t)st.state,
			(uint8_t)st.fail_reason,
			st.progress,
			st.level,
			st.submode,
			st.step,
			st.step_total,
			0u /* reserved */
		};
		reply_set(proto, cmd, body, sizeof(body));
		return JM_ERR_OK;
	}

	/* PID 理论估计 0xA0: 触发 autotune 计算 + 自动设 source=2 + reload。
	 * ring_select 位掩码: bit0=电流 bit1=速度 bit2=位置 (可组合)
	 * ACK: 8字节 {status, fail_reason, ring_mask_done, reserved[5]} */
	if (cmd == JM_CMD_PID_AUTOTUNE)
	{
		uint8_t ring_select;
		float cur_bw, vel_bw, pos_bw;
		uint8_t fail_reason = 0;
		jm_err_e e;

		if (len < 13) /* ring(1) + 3*float(12) */
		{
			return reply_nack(proto, cmd, JM_ERR_LENGTH);
		}
		if (proto->ops == NULL || proto->ops->pid_autotune == NULL)
		{
			return reply_nack(proto, cmd, JM_ERR_UNSUPPORTED);
		}
		ring_select = payload[0];
		memcpy(&cur_bw, &payload[1], 4);
		memcpy(&vel_bw, &payload[5], 4);
		memcpy(&pos_bw, &payload[9], 4);
		e = proto->ops->pid_autotune(ring_select, cur_bw, vel_bw, pos_bw, &fail_reason);
		/* 0xA0 总是回 8字节 ACK(成功/失败均回), 返回 JM_ERR_OK 避免调用方覆盖 reply。
		 * 成败信息编码在 body[0](status) 和 body[1](fail_reason) 中, 同 0x97 先例。*/
		{
			uint8_t body[8] = {(e == JM_ERR_OK) ? 0u : 1u, fail_reason,
			                   (e == JM_ERR_OK) ? ring_select : 0u, 0, 0, 0, 0, 0};
			reply_set(proto, cmd, body, sizeof(body));
			return JM_ERR_OK;
		}
	}

	/* PID 来源切换 0xA1: 独立设置某环 source, 立即 reload。简单 ACK。*/
	if (cmd == JM_CMD_PID_SOURCE_SET)
	{
		uint8_t ring_select, source;
		jm_err_e e;

		if (len < 2)
		{
			return reply_nack(proto, cmd, JM_ERR_LENGTH);
		}
		if (proto->ops == NULL || proto->ops->pid_source_set == NULL)
		{
			return reply_nack(proto, cmd, JM_ERR_UNSUPPORTED);
		}
		ring_select = payload[0];
		source = payload[1];
		e = proto->ops->pid_source_set(ring_select, source);
		return (e == JM_ERR_OK) ? reply_ack(proto, cmd, 0) : reply_nack(proto, cmd, e);
	}

	/* PID 来源查询 0xA2: 返回三环当前 source (3字节: cur/vel/pos) */
	if (cmd == JM_CMD_PID_SOURCE_GET)
	{
		uint8_t o[3];
		jm_err_e e;
		if (len != 0u)
		{
			return reply_nack(proto, cmd, JM_ERR_LENGTH);
		}
		if (proto->ops == NULL || proto->ops->pid_source_get == NULL)
		{
			return reply_nack(proto, cmd, JM_ERR_UNSUPPORTED);
		}
		e = proto->ops->pid_source_get(&o[0], &o[1], &o[2]);
		if (e != JM_ERR_OK)
		{
			return reply_nack(proto, cmd, e);
		}
		reply_set(proto, cmd, o, 3);
		return JM_ERR_OK;
	}

	/* PID 参数实时写 0xA5: 仅 DEBUG source 下允许, 直接写 profile, ISR 下一拍生效。
	 * 载荷: ring(1) + param_type(1) + value(4)  = 6 字节
	 * ACK: {status:u8}  0=成功, 失败走 NACK */
	if (cmd == JM_CMD_PID_PARAM_SET)
	{
		uint8_t ring, param_type;
		jm_err_e e;

		if (len < 6) /* ring(1) + param_type(1) + value(4); type=0 may extend */
		{
			return reply_nack(proto, cmd, JM_ERR_LENGTH);
		}
		if (proto->ops == NULL || proto->ops->pid_param_set == NULL)
		{
			return reply_nack(proto, cmd, JM_ERR_UNSUPPORTED);
		}
		ring = payload[0];
		param_type = payload[1];
		e = proto->ops->pid_param_set(ring, param_type, &payload[2], (uint16_t)(len - 2u));
		return (e == JM_ERR_OK) ? reply_ack(proto, cmd, 0) : reply_nack(proto, cmd, e);
	}

	/* PID 参数实时读 0xA6: 随时可读, 返回当前 profile 中的值。
	 * 载荷: ring(1) + param_type(1) = 2 字节
	 * 应答: {ring:u8, param_type:u8, value:4B} = 6 字节 */
	if (cmd == JM_CMD_PID_PARAM_GET)
	{
		uint8_t ring, param_type;
		uint8_t value4[4];
		uint8_t body[6];
		jm_err_e e;

		if (len < 2) /* ring(1) + param_type(1) */
		{
			return reply_nack(proto, cmd, JM_ERR_LENGTH);
		}
		if (proto->ops == NULL || proto->ops->pid_param_get == NULL)
		{
			return reply_nack(proto, cmd, JM_ERR_UNSUPPORTED);
		}
		ring = payload[0];
		param_type = payload[1];
		e = proto->ops->pid_param_get(ring, param_type, value4);
		if (e != JM_ERR_OK)
		{
			return reply_nack(proto, cmd, e);
		}
		body[0] = ring;
		body[1] = param_type;
		memcpy(&body[2], value4, 4);
		reply_set(proto, cmd, body, 6);
		return JM_ERR_OK;
	}

	/* 预留命令拦截: 0x80~0x82(多电机同步) / 0xCC~0xCF(OTA)
	 * 这些命令码已定义但当前固件未实现 handler, 统一回 NACK(NOT_SUPPORTED, seq=0)。
	 * 区别于 UNSUPPORTED(0x01, 命令码区间不识别): NOT_SUPPORTED 表示命令码已知但未实现,
	 * 上位机据此区分"老固件不识别新命令"与"新固件预留未实现"。*/
	if (cmd == JM_CMD_SYNC || cmd == JM_CMD_PRESET_AND_TRIGGER || cmd == JM_CMD_TRIGGER ||
	    cmd == JM_CMD_OTA_START || cmd == JM_CMD_OTA_DATA ||
	    cmd == JM_CMD_OTA_END || cmd == JM_CMD_OTA_RESUME)
	{
		return reply_nack(proto, cmd, JM_ERR_NOT_SUPPORTED);
	}

	/* 其余 0x00~0xB8 控制/校准/诊断类: 统一交给 set_mode 回调,
	 * CMD 值即 ctrl_mode_e, 由应用层按模式解析 payload 并执行。*/
	if (cmd <= JM_CMD_SINGLE_STEP)
	{
		jm_err_e e;
		if (proto->ops == NULL || proto->ops->set_mode == NULL)
		{
			return reply_nack(proto, cmd, JM_ERR_UNSUPPORTED);
		}
		e = proto->ops->set_mode(cmd, payload, len);
		return (e == JM_ERR_OK) ? reply_ack(proto, cmd, 0) : reply_nack(proto, cmd, e);
	}

	return reply_nack(proto, cmd, JM_ERR_UNSUPPORTED);
}
