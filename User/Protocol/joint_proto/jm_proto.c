/**
 * @file        jm_proto.c
 * @brief       关节电机通信协议核心: 传输无关的 CMD 分发与应答组织
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
 */
#include <string.h>
#include "jm_proto.h"

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

/* 组织 NACK 应答: [0xFE][失败的cmd][err_code] */
static jm_err_e reply_nack(jm_proto_t *p, uint8_t cmd, jm_err_e err)
{
	uint8_t body[2] = {cmd, (uint8_t)err};
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
		default:
			return reply_nack(p, cmd, JM_ERR_UNSUPPORTED);
	}
}

/* ---- 设备信息类 0xD0~0xD2 ---- */
static jm_err_e handle_dev(jm_proto_t *p, uint8_t cmd)
{
	const jm_proto_ops_t *ops = p->ops;
	switch (cmd)
	{
		case JM_CMD_READ_DEV_INFO:
		{
			uint8_t o[20];
			uint32_t hw = 0, fw = 0;
			uint8_t uid[12] = {0};
			if (ops == NULL || ops->get_dev_info == NULL)
				return reply_nack(p, cmd, JM_ERR_UNSUPPORTED);
			if (ops->get_dev_info(&hw, &fw, uid) != JM_ERR_OK)
				return reply_nack(p, cmd, JM_ERR_STATE_DENY);
			jm_wr_u32(&o[0], hw);
			jm_wr_u32(&o[4], fw);
			memcpy(&o[8], uid, 12);
			reply_set(p, cmd, o, 20);
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

/* ---- 主分发: 串口/CAN 解出 cmd+payload 后统一进这里 ---- */
jm_err_e jm_proto_dispatch(jm_proto_t *proto, uint8_t cmd, const uint8_t *payload, uint16_t len)
{
	if (proto == NULL)
	{
		return JM_ERR_STATE_DENY;
	}
	proto->reply_len = 0; /* 默认无应答 */

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

	/* 反馈查询 0xC0~0xCF */
	if (cmd >= JM_CMD_READ_FEEDBACK && cmd <= JM_CMD_READ_FAULT)
	{
		return handle_read(proto, cmd);
	}
	/* 设备信息 0xD0~0xDF */
	if (cmd >= JM_CMD_READ_DEV_INFO && cmd <= JM_CMD_HEARTBEAT)
	{
		return handle_dev(proto, cmd);
	}
	/* 参数读写 0xE0~0xEF */
	if (cmd >= JM_CMD_PARAM_READ && cmd <= JM_CMD_PARAM_RESET)
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
