/**
 * @file        jm_proto_can.c
 * @brief       关节电机协议-CAN绑定层实现: 定点压缩 + 多帧分包 + 共用dispatch
 *
 * @author      Dalin (dalinyy@163.com)
 * @version     1.1
 * @date        2026-06-18
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @par 修改日志:
 * | 日期       | 版本 | 作者   | 修改内容   |
 * |------------|------|--------|------------|
 * | 2026-06-18 | 1.0  | Dalin  | 初始创建   |
 * | 2026-07-29 | 1.1  | Dalin  | 运行期FD模式切换 + 多帧CRC16 + 超时清理 + 首帧冲突检测 |
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 */
#include <string.h>
#include "jm_proto_can.h"
#include "crc16.h" /* 与UART层共用 crc16_calc(CCITT/XMODEM, 初值0) */

/* ---------------- 定点压缩助手 ---------------- */

/* float -> 无符号定点(bits位), 截断到范围内 */
static uint32_t float_to_uint(float x, float xmin, float xmax, uint8_t bits)
{
	float span = xmax - xmin;
	uint32_t fullscale = (bits >= 32) ? 0xFFFFFFFFu : ((1u << bits) - 1u);
	if (x < xmin)
		x = xmin;
	if (x > xmax)
		x = xmax;
	if (span <= 0.0f)
		return 0;
	return (uint32_t)((x - xmin) * (float)fullscale / span + 0.5f);
}

/* 无符号定点(bits位) -> float */
static float uint_to_float(uint32_t v, float xmin, float xmax, uint8_t bits)
{
	uint32_t fullscale = (bits >= 32) ? 0xFFFFFFFFu : ((1u << bits) - 1u);
	if (fullscale == 0)
		return xmin;
	return (float)v * (xmax - xmin) / (float)fullscale + xmin;
}

void jm_mit_pack(uint8_t out[8], float pos, float vel, float kp, float kd, float tff)
{
	uint32_t p = float_to_uint(pos, JM_MIT_POS_MIN, JM_MIT_POS_MAX, 16);
	uint32_t v = float_to_uint(vel, JM_MIT_VEL_MIN, JM_MIT_VEL_MAX, 12);
	uint32_t kpv = float_to_uint(kp, JM_MIT_KP_MIN, JM_MIT_KP_MAX, 12);
	uint32_t kdv = float_to_uint(kd, JM_MIT_KD_MIN, JM_MIT_KD_MAX, 12);
	uint32_t t = float_to_uint(tff, JM_MIT_TFF_MIN, JM_MIT_TFF_MAX, 12);

	out[0] = (uint8_t)(p >> 8);
	out[1] = (uint8_t)(p & 0xFF);
	out[2] = (uint8_t)(v >> 4);
	out[3] = (uint8_t)(((v & 0xF) << 4) | ((kpv >> 8) & 0xF));
	out[4] = (uint8_t)(kpv & 0xFF);
	out[5] = (uint8_t)(kdv >> 4);
	out[6] = (uint8_t)(((kdv & 0xF) << 4) | ((t >> 8) & 0xF));
	out[7] = (uint8_t)(t & 0xFF);
}

void jm_mit_unpack(const uint8_t in[8], float *pos, float *vel, float *kp, float *kd, float *tff)
{
	uint32_t p = ((uint32_t)in[0] << 8) | in[1];
	uint32_t v = ((uint32_t)in[2] << 4) | (in[3] >> 4);
	uint32_t kpv = ((uint32_t)(in[3] & 0xF) << 8) | in[4];
	uint32_t kdv = ((uint32_t)in[5] << 4) | (in[6] >> 4);
	uint32_t t = ((uint32_t)(in[6] & 0xF) << 8) | in[7];

	if (pos)
		*pos = uint_to_float(p, JM_MIT_POS_MIN, JM_MIT_POS_MAX, 16);
	if (vel)
		*vel = uint_to_float(v, JM_MIT_VEL_MIN, JM_MIT_VEL_MAX, 12);
	if (kp)
		*kp = uint_to_float(kpv, JM_MIT_KP_MIN, JM_MIT_KP_MAX, 12);
	if (kd)
		*kd = uint_to_float(kdv, JM_MIT_KD_MIN, JM_MIT_KD_MAX, 12);
	if (tff)
		*tff = uint_to_float(t, JM_MIT_TFF_MIN, JM_MIT_TFF_MAX, 12);
}

/* 反馈帧压缩: pos16 vel16 tq16 temp8 err8 = 8字节 */
uint8_t jm_fb_pack(uint8_t out[8], const jm_feedback_t *fb)
{
	uint32_t p = float_to_uint(fb->pos, JM_MIT_POS_MIN, JM_MIT_POS_MAX, 16);
	uint32_t v = float_to_uint(fb->vel, JM_MIT_VEL_MIN, JM_MIT_VEL_MAX, 16);
	uint32_t tq = float_to_uint(fb->torque, JM_MIT_TFF_MIN, JM_MIT_TFF_MAX, 16);
	uint32_t tp = float_to_uint(fb->temp_motor, JM_FB_TEMP_MIN, JM_FB_TEMP_MAX, 8);

	out[0] = (uint8_t)(p >> 8);
	out[1] = (uint8_t)(p & 0xFF);
	out[2] = (uint8_t)(v >> 8);
	out[3] = (uint8_t)(v & 0xFF);
	out[4] = (uint8_t)(tq >> 8);
	out[5] = (uint8_t)(tq & 0xFF);
	out[6] = (uint8_t)tp;
	out[7] = (uint8_t)(fb->fault_mask & 0xFF);
	return 8;
}

/* ---------------- 运行期模式 ---------------- */

/* 运行期单帧上限: FD=64B, 经典=8B */
static inline uint8_t jm_can_single_max(const jm_proto_can_t *c)
{
	return c->use_fd_runtime ? JM_CAN_SINGLE_MAX_FD : JM_CAN_SINGLE_MAX_CLS;
}

/* 运行期多帧片段载荷: FD=63B, 经典=7B (留 1B 给控制字) */
static uint8_t jm_can_fd_len_is_valid(uint16_t len)
{
	return len <= 8u || len == 12u || len == 16u || len == 20u ||
	       len == 24u || len == 32u || len == 48u || len == 64u;
}

static uint16_t jm_can_next_chunk(const jm_proto_can_t *c, uint16_t remaining)
{
	static const uint8_t fd_chunks[] = {63u, 47u, 31u, 23u, 19u, 15u, 11u, 7u, 6u, 5u, 4u, 3u, 2u, 1u};
	uint8_t i;

	if (!c->use_fd_runtime)
		return (remaining > JM_CAN_SEG_PAYLOAD_CLS) ? JM_CAN_SEG_PAYLOAD_CLS : remaining;

	for (i = 0u; i < sizeof(fd_chunks); i++)
	{
		if (remaining >= fd_chunks[i])
			return fd_chunks[i];
	}
	return 0u;
}

uint8_t jm_proto_can_set_fd_mode(jm_proto_can_t *c, uint8_t enable)
{
	if (c == NULL)
	{
		return 0;
	}
#if defined(JM_PERIPH_CAN_FD) && defined(USE_CAN_FD_MODE) && (USE_CAN_FD_MODE == 1)
	c->use_fd_runtime = enable ? 1u : 0u; /* 硬件支持, 按 enable 切换 */
	return 1;                              /* 硬件能力: 支持 */
#else
	c->use_fd_runtime = 0u; /* 硬件不支持, 强制经典 */
	return 0;                /* 硬件能力: 不支持 */
#endif
}

uint8_t jm_proto_can_get_fd_mode(const jm_proto_can_t *c)
{
	return (c != NULL) ? c->use_fd_runtime : 0u;
}

/* ---------------- TX 底层 ---------------- */

/* 发一帧原始CAN(ID已含CMD+电机ID). is_fd 由运行期模式决定 */
static void can_send_raw(jm_proto_can_t *c, uint32_t id, const uint8_t *d, uint8_t len)
{
	jm_can_frame_t f;
	uint8_t max_len = jm_can_single_max(c);
	if (c->tx == NULL || len > max_len)
	{
		return;
	}
	f.id = id;
	f.len = len;
	f.is_fd = c->use_fd_runtime;
	memset(f.data, 0, sizeof(f.data));
	if (d != NULL && len > 0)
	{
		memcpy(f.data, d, len);
	}
	c->tx(&f);
}

/* 发送一段已确定编码的载荷: <=single_max 单帧, 否则多帧分包(末帧追加 CRC16) */
static void can_copy_stream(uint8_t *dst, const uint8_t *body, uint16_t len,
	                         uint16_t crc, uint16_t offset, uint16_t count)
{
	uint16_t i;
	for (i = 0; i < count; i++)
	{
		uint16_t pos = (uint16_t)(offset + i);
		if (pos < len)
			dst[i] = body[pos];
		else if (pos == len)
			dst[i] = (uint8_t)(crc & 0xFFu);
		else
			dst[i] = (uint8_t)(crc >> 8);
	}
}

static void can_emit_payload(jm_proto_can_t *c, uint8_t cmd, const uint8_t *body, uint16_t len)
{
	uint32_t id = JM_CAN_MAKE_ID(cmd, c->motor_id);
	uint8_t single_max = jm_can_single_max(c);

	if (len <= single_max && (!c->use_fd_runtime || jm_can_fd_len_is_valid(len)))
	{
		can_send_raw(c, id, body, (uint8_t)len);
		return;
	}

	/* 多帧: data[0]=控制字(序号+末帧标志), data[1..]=片段;
	 *       末帧末尾追加 2 字节 CRC16(小端, 覆盖整个载荷) */
	uint16_t crc = crc16_calc((uint8_t *)body, (int)len);
	uint16_t off = 0;
	uint16_t total = (uint16_t)(len + 2u);
	uint8_t seq = 0;
	id |= JM_CAN_MULTI_FLAG;
	while (off < total)
	{
		uint8_t frame[64];
		uint16_t chunk = jm_can_next_chunk(c, (uint16_t)(total - off));
		frame[0] = (uint8_t)(seq & JM_CAN_SEG_SEQ_MASK);
		if (off + chunk >= total)
		{
			frame[0] |= JM_CAN_SEG_LAST; /* 末帧 */
			can_copy_stream(&frame[1], body, len, crc, off, chunk);
			can_send_raw(c, id, frame, (uint8_t)(1u + chunk));
		}
		else
		{
			can_copy_stream(&frame[1], body, len, crc, off, chunk);
			can_send_raw(c, id, frame, (uint8_t)(1 + chunk));
		}
		off = (uint16_t)(off + chunk);
		seq++;
	}
}

/* 把"逻辑载荷"按cmd编码后发出: 反馈/MIT压缩, 其余原样 */
static void can_emit_logical(jm_proto_can_t *c, uint8_t cmd, const uint8_t *body, uint16_t len)
{
	uint8_t packed[8];

	/* 反馈帧: 逻辑载荷是全精度结构, CAN压缩成8字节 */
	if (cmd == JM_CMD_READ_FEEDBACK && c->proto.ops && c->proto.ops->get_feedback)
	{
		jm_feedback_t fb;
		memset(&fb, 0, sizeof(fb));
		if (c->proto.ops->get_feedback(&fb) == JM_ERR_OK)
		{
			jm_fb_pack(packed, &fb);
			can_emit_payload(c, cmd, packed, 8);
			return;
		}
	}
	/* 其余命令: 载荷原样(<=8单帧, >8分包) */
	can_emit_payload(c, cmd, body, len);
}

void jm_proto_can_send(jm_proto_can_t *c, uint8_t cmd, const uint8_t *body, uint16_t len)
{
	if (c == NULL)
	{
		return;
	}
	can_emit_logical(c, cmd, body, len);
}

/* ---------------- RX: 分发并回送应答 ---------------- */

/* 对已重组好的(cmd + payload)做分发, 应答经CAN压缩/分包发回 */
static void can_dispatch_and_reply(jm_proto_can_t *c, uint8_t cmd,
                                   const uint8_t *payload, uint16_t plen,
                                   uint8_t reply_enabled)
{
	uint8_t norm[20]; /* MIT归一化缓冲: 5*f32 */

	/* MIT/阻抗: CAN是8字节压缩, 解压成与串口一致的5*f32再分发,
	 * 使应用层 set_mode 回调对两种传输完全一致(传输无关)。*/
	if ((cmd == JM_CMD_MIT || cmd == JM_CMD_IMPEDANCE) && plen >= 8)
	{
		float pos, vel, kp, kd, tff;
		jm_mit_unpack(payload, &pos, &vel, &kp, &kd, &tff);
		jm_wr_f32(&norm[0], pos);
		jm_wr_f32(&norm[4], vel);
		jm_wr_f32(&norm[8], kp);
		jm_wr_f32(&norm[12], kd);
		jm_wr_f32(&norm[16], tff);
		payload = norm;
		plen = 20;
	}

	jm_proto_dispatch(&c->proto, cmd, payload, plen);

	/* 广播命令只执行不应答, 避免多节点同时发送造成总线冲突。 */
	if (reply_enabled && c->proto.reply_len > 0)
	{
		uint8_t rcmd = c->proto.reply[0];
		const uint8_t *rbody = (c->proto.reply_len > 1) ? &c->proto.reply[1] : NULL;
		uint16_t rlen = (uint16_t)(c->proto.reply_len - 1);
		/* 反馈类应答在CAN上压缩(can_emit_logical内部按rcmd处理) */
		can_emit_logical(c, rcmd, rbody, rlen);
	}
}

/* 广播仅开放不会修改持久化配置的全局安全命令。 */
uint8_t jm_proto_can_broadcast_allowed(uint8_t cmd)
{
	return (cmd == JM_CMD_BROADCAST_SYNC || cmd == JM_CMD_ESTOP) ? 1u : 0u;
}

/* 复位多帧重组状态 */
static void rx_reset(jm_proto_can_t *c)
{
	c->rx_active = 0;
	c->rx_len = 0;
	c->rx_seq = 0;
	c->rx_cmd = 0;
	c->rx_start_tick = 0;
}

void jm_proto_can_feed(jm_proto_can_t *c, const jm_can_frame_t *frame, uint32_t now_tick)
{
	uint8_t cmd, dst, is_broadcast;

	if (c == NULL || frame == NULL)
	{
		return;
	}

	cmd = JM_CAN_GET_CMD(frame->id);
	dst = JM_CAN_GET_MOTOR_ID(frame->id);

	/* 地址过滤: 只收发给本机或广播(0)的帧 */
	if (dst != c->motor_id && dst != JM_CAN_BROADCAST_ID)
	{
		return;
	}
	is_broadcast = (dst == JM_CAN_BROADCAST_ID) ? 1u : 0u;
	if (is_broadcast && !jm_proto_can_broadcast_allowed(cmd))
	{
		return;
	}

	/* 多帧重组超时清理: 200ms 未收到末帧则丢弃当前重组, 避免永久占用缓冲 */
	if (c->rx_active && (now_tick - c->rx_start_tick) > JM_CAN_MULTI_TIMEOUT_MS)
	{
		c->rx_timeout_count++;
		rx_reset(c);
	}

	/* 判断是否多帧分包: 载荷上限内的命令走单帧;
	 * 多帧重组仅用于本层 can_emit_payload 产生的帧, 即 data[0] 为控制字。
	 * 为避免与单帧载荷歧义, 多帧仅对"已知大载荷CMD"启用(与CSV"CAN需分包"一致)。*/
	uint8_t is_multi = JM_CAN_IS_MULTI_ID(frame->id) ? 1u : 0u;

	if (!is_multi)
	{
		/* 单帧: data 即载荷 */
		can_dispatch_and_reply(c, cmd, frame->data, frame->len,
		                       is_broadcast ? 0u : 1u);
		return;
	}
	/* 白名单广播命令均为单帧。拒绝广播分包, 避免跨节点重组状态冲突。 */
	if (is_broadcast)
	{
		return;
	}

	/* 多帧重组: data[0]=控制字, data[1..]=片段 */
	if (frame->len < 1)
	{
		return;
	}
	{
		uint8_t ctrl = frame->data[0];
		uint8_t seq = ctrl & JM_CAN_SEG_SEQ_MASK;
		uint8_t last = (ctrl & JM_CAN_SEG_LAST) ? 1 : 0;
		uint8_t frag = (uint8_t)(frame->len - 1);

		if (seq == 0)
		{
			/* 新一轮重组; 若旧重组未完成则计为冲突 */
			if (c->rx_active)
			{
				c->rx_conflict_count++;
			}
			rx_reset(c);
			c->rx_active = 1;
			c->rx_cmd = cmd;
			c->rx_buf[0] = cmd; /* 重组区[0]存CMD, 与dispatch约定一致 */
			c->rx_len = 1;
			c->rx_start_tick = now_tick;
		}
		else if (!c->rx_active || cmd != c->rx_cmd || seq != c->rx_seq)
		{
			rx_reset(c); /* 序号错乱, 丢弃 */
			return;
		}

		if ((uint16_t)(c->rx_len + frag) > sizeof(c->rx_buf))
		{
			rx_reset(c);
			return; /* 溢出保护 */
		}
		memcpy(&c->rx_buf[c->rx_len], &frame->data[1], frag);
		c->rx_len = (uint16_t)(c->rx_len + frag);
		c->rx_seq++;

		if (last)
		{
			/* 重组完成: rx_buf[0]=CMD, [1..]=载荷+CRC16(末2字节小端) */
			if (c->rx_len >= 3u) /* 至少 1B CMD + 2B CRC */
			{
				uint16_t plen = (uint16_t)(c->rx_len - 1u); /* 含 CRC 的载荷长度 */
				uint16_t recv_crc = (uint16_t)(c->rx_buf[c->rx_len - 2u]
				                          | (c->rx_buf[c->rx_len - 1u] << 8));
				uint16_t calc_crc = crc16_calc(&c->rx_buf[1], (int)(plen - 2u));
				if (recv_crc != calc_crc)
				{
					c->rx_crc_fail_count++;
					rx_reset(c);
					return; /* CRC 校验失败, 丢弃 */
				}
				/* CRC 校验通过, 分发载荷(去除末2字节CRC) */
				can_dispatch_and_reply(c, c->rx_cmd, &c->rx_buf[1],
				                       (uint16_t)(plen - 2u), 1u);
			}
			rx_reset(c);
		}
	}
}

/* ---------------- 初始化 ---------------- */
int jm_proto_can_init(jm_proto_can_t *c, const jm_proto_ops_t *ops,
                      uint8_t motor_id, jm_can_tx_fn tx)
{
	if (c == NULL || tx == NULL)
	{
		return -1;
	}
	memset(c, 0, sizeof(*c));
	jm_proto_init(&c->proto, ops, motor_id);
	c->tx = tx;
	c->motor_id = motor_id;
	c->use_fd_runtime = 0; /* 上电默认经典模式(兼容所有上位机硬件) */
	rx_reset(c);
	return 0;
}
