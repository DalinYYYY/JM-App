/**
 * @file    vesc_proto.c
 * @brief   可移植 VESC 串行通信协议模块实现（帧层 + 常用命令封装）。
 * @details 包含 CRC-16 校验、大端编解码、组帧/拆帧状态机及命令编解码。
 *          平台无关，移植说明见 vesc_proto.h。
 */
#include "vesc_proto.h"
#include <string.h>

/* ====================== CRC-16/XMODEM ====================== */

/**
 * @brief  计算 CRC-16/XMODEM 校验值。
 * @details 多项式 0x1021，初值 0，输入/输出均不反转。采用按位算法，
 *          省去 512 字节查表，与固件 util/crc.c 的查表实现数学等价。
 * @param  buf  待校验数据。
 * @param  len  数据字节数。
 * @return 16 位校验值。
 */
static uint16_t vesc_crc16(const uint8_t *buf, uint32_t len)
{
	uint16_t crc = 0;
	for (uint32_t i = 0; i < len; i++)
	{
		crc ^= (uint16_t)buf[i] << 8;
		for (uint8_t b = 0; b < 8; b++)
		{
			crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
								  : (uint16_t)(crc << 1);
		}
	}
	return crc;
}

/* ====================== buffer 编解码（大端） ====================== */

/**
 * @brief  以大端序追加一个 int32 到缓冲区，并前移索引。
 * @param  b  目标缓冲区。
 * @param  v  待写入的值。
 * @param  i  读写索引（入/出参，写入后自增 4）。
 */
static void buf_append_i32(uint8_t *b, int32_t v, int32_t *i)
{
	b[(*i)++] = (uint8_t)(v >> 24);
	b[(*i)++] = (uint8_t)(v >> 16);
	b[(*i)++] = (uint8_t)(v >> 8);
	b[(*i)++] = (uint8_t)v;
}

/**
 * @brief  从缓冲区按大端序读取一个 int16，并前移索引。
 * @param  b  源缓冲区。
 * @param  i  读写索引（入/出参，读取后自增 2）。
 * @return 读取到的有符号 16 位值。
 */
static int16_t buf_get_i16(const uint8_t *b, int32_t *i)
{
	int16_t r = (int16_t)(((uint16_t)b[*i] << 8) | (uint16_t)b[*i + 1]);
	*i += 2;
	return r;
}

/**
 * @brief  从缓冲区按大端序读取一个 int32，并前移索引。
 * @param  b  源缓冲区。
 * @param  i  读写索引（入/出参，读取后自增 4）。
 * @return 读取到的有符号 32 位值。
 */
static int32_t buf_get_i32(const uint8_t *b, int32_t *i)
{
	int32_t r = ((int32_t)b[*i] << 24) | ((int32_t)b[*i + 1] << 16) | ((int32_t)b[*i + 2] << 8) | (int32_t)b[*i + 3];
	*i += 4;
	return r;
}

/**
 * @brief  读取定点编码的 float16（值 = int16 / scale）。
 * @param  b     源缓冲区。
 * @param  scale 缩放系数。
 * @param  i     读写索引（入/出参，读取后自增 2）。
 * @return 还原后的浮点值。
 */
static float buf_get_f16(const uint8_t *b, float scale, int32_t *i)
{
	return (float)buf_get_i16(b, i) / scale;
}

/**
 * @brief  读取定点编码的 float32（值 = int32 / scale）。
 * @param  b     源缓冲区。
 * @param  scale 缩放系数。
 * @param  i     读写索引（入/出参，读取后自增 4）。
 * @return 还原后的浮点值。
 */
static float buf_get_f32(const uint8_t *b, float scale, int32_t *i)
{
	return (float)buf_get_i32(b, i) / scale;
}

/* ====================== 初始化 ====================== */

/**
 * @brief  初始化协议实例。
 * @param  vp   协议实例。
 * @param  cfg  配置（含发送回调与可选的接收回调）；为 NULL 时仅清零实例。
 */
void vesc_proto_init(vesc_proto_t *vp, const vesc_proto_cfg_t *cfg)
{
	memset(vp, 0, sizeof(*vp));
	if (cfg)
	{
		vp->cfg = *cfg;
	}
}

/**
 * @brief  复位接收拆帧状态（链路重连或同步丢失时调用）。
 * @param  vp  协议实例。
 */
void vesc_proto_reset(vesc_proto_t *vp)
{
	vp->rx_rd = 0;
	vp->rx_wr = 0;
	vp->bytes_left = 0;
}

/* ====================== 发送：组帧 ====================== */

/**
 * @brief  将一段 payload 组帧并通过发送回调发出。
 * @details 帧结构：起始字节(2/3) + 长度(1/2B,大端) + payload + CRC16(大端) + 停止字节(3)。
 *          长度字段宽度按 payload 长度自适应。
 * @param  vp       协议实例。
 * @param  payload  payload 数据，payload[0] 应为命令 ID。
 * @param  len      payload 长度，须为 1..VESC_PROTO_MAX_PL，否则静默丢弃。
 */
void vesc_proto_send(vesc_proto_t *vp, const uint8_t *payload, uint16_t len)
{
	if (len == 0 || len > VESC_PROTO_MAX_PL || !vp->cfg.send_bytes)
	{
		return;
	}

	int32_t n = 0;
	uint8_t *tx = vp->tx_buf;

	/* 起始字节 + 长度字段（按长度自适应宽度，大端） */
	if (len <= 255)
	{
		tx[n++] = 2;
		tx[n++] = (uint8_t)len;
	}
	else
	{
		tx[n++] = 3;
		tx[n++] = (uint8_t)(len >> 8);
		tx[n++] = (uint8_t)(len & 0xFF);
	}

	memcpy(tx + n, payload, len);
	n += len;

	uint16_t crc = vesc_crc16(payload, len);
	tx[n++] = (uint8_t)(crc >> 8);
	tx[n++] = (uint8_t)(crc & 0xFF);
	tx[n++] = 3; /* 停止字节 */

	vp->cfg.send_bytes(tx, (uint16_t)n, vp->cfg.ctx);
}

/* ====================== 高层命令封装 ====================== */

/**
 * @brief  发送仅含命令 ID、无参数的帧。
 * @param  vp  协议实例。
 * @param  id  命令 ID。
 */
static void send_cmd_only(vesc_proto_t *vp, uint8_t id)
{
	uint8_t p = id;
	vesc_proto_send(vp, &p, 1);
}

/**
 * @brief  发送 "命令 ID + 一个 int32 参数" 的帧。
 * @param  vp  协议实例。
 * @param  id  命令 ID。
 * @param  v   int32 参数（已完成定点缩放）。
 */
static void send_cmd_i32(vesc_proto_t *vp, uint8_t id, int32_t v)
{
	uint8_t p[5];
	int32_t n = 0;
	p[n++] = id;
	buf_append_i32(p, v, &n);
	vesc_proto_send(vp, p, (uint16_t)n);
}

void vesc_set_duty(vesc_proto_t *vp, float duty)
{
	send_cmd_i32(vp, COMM_SET_DUTY, (int32_t)(duty * 100000.0f));
}
void vesc_set_current(vesc_proto_t *vp, float current_a)
{
	send_cmd_i32(vp, COMM_SET_CURRENT, (int32_t)(current_a * 1000.0f));
}
void vesc_set_current_brake(vesc_proto_t *vp, float current_a)
{
	send_cmd_i32(vp, COMM_SET_CURRENT_BRAKE, (int32_t)(current_a * 1000.0f));
}
void vesc_set_current_rel(vesc_proto_t *vp, float rel)
{
	send_cmd_i32(vp, COMM_SET_CURRENT_REL, (int32_t)(rel * 100000.0f));
}
void vesc_set_rpm(vesc_proto_t *vp, float erpm)
{
	send_cmd_i32(vp, COMM_SET_RPM, (int32_t)erpm);
}
void vesc_set_pos(vesc_proto_t *vp, float deg)
{
	send_cmd_i32(vp, COMM_SET_POS, (int32_t)(deg * 1000000.0f));
}
void vesc_set_handbrake(vesc_proto_t *vp, float current_a)
{
	send_cmd_i32(vp, COMM_SET_HANDBRAKE, (int32_t)(current_a * 1000.0f));
}
void vesc_send_alive(vesc_proto_t *vp)
{
	send_cmd_only(vp, COMM_ALIVE);
}
void vesc_reboot(vesc_proto_t *vp)
{
	send_cmd_only(vp, COMM_REBOOT);
}
void vesc_request_values(vesc_proto_t *vp)
{
	send_cmd_only(vp, COMM_GET_VALUES);
}
void vesc_request_fw_version(vesc_proto_t *vp)
{
	send_cmd_only(vp, COMM_FW_VERSION);
}

/**
 * @brief  通过本机 CAN 总线把一条命令转发给指定从机。
 * @param  vp       协议实例。
 * @param  can_id   目标从机的 CAN ID。
 * @param  payload  被转发命令的完整 payload（payload[0] 为命令 ID）。
 * @param  len      被转发 payload 的长度；len 与目标头部之和超限时静默丢弃。
 */
void vesc_forward_can(vesc_proto_t *vp, uint8_t can_id,
					  const uint8_t *payload, uint16_t len)
{
	if (len == 0 || (uint32_t)len + 2 > VESC_PROTO_MAX_PL)
	{
		return;
	}
	/* 拼装：[FORWARD_CAN][can_id][原命令...] */
	uint8_t tmp[VESC_PROTO_MAX_PL];
	int32_t n = 0;
	tmp[n++] = COMM_FORWARD_CAN;
	tmp[n++] = can_id;
	memcpy(tmp + n, payload, len);
	n += len;
	vesc_proto_send(vp, tmp, (uint16_t)n);
}

/* ====================== 接收：回复解析 ====================== */

/**
 * @brief  解析 COMM_GET_VALUES 回复的全字段。
 * @param  d    payload 主体（不含命令 ID）。
 * @param  len  主体长度；不足全字段所需长度时直接返回清零结果。
 * @param  v    输出结构体。
 */
static void parse_values(const uint8_t *d, uint16_t len, vesc_values_t *v)
{
	int32_t i = 0;
	memset(v, 0, sizeof(*v));
	/* 全字段需要约 71 字节，长度不足则放弃 */
	if (len < 71)
	{
		return;
	}
	v->temp_fet = buf_get_f16(d, 1e1f, &i);
	v->temp_motor = buf_get_f16(d, 1e1f, &i);
	v->current_motor = buf_get_f32(d, 1e2f, &i);
	v->current_in = buf_get_f32(d, 1e2f, &i);
	v->id = buf_get_f32(d, 1e2f, &i);
	v->iq = buf_get_f32(d, 1e2f, &i);
	v->duty = buf_get_f16(d, 1e3f, &i);
	v->rpm = buf_get_f32(d, 1e0f, &i);
	v->v_in = buf_get_f16(d, 1e1f, &i);
	v->amp_hours = buf_get_f32(d, 1e4f, &i);
	v->amp_hours_chg = buf_get_f32(d, 1e4f, &i);
	v->watt_hours = buf_get_f32(d, 1e4f, &i);
	v->watt_hours_chg = buf_get_f32(d, 1e4f, &i);
	v->tacho = buf_get_i32(d, &i);
	v->tacho_abs = buf_get_i32(d, &i);
	v->fault_code = d[i++];
	v->pid_pos = buf_get_f32(d, 1e6f, &i);
	v->controller_id = d[i++];
}

/**
 * @brief  解析 COMM_FW_VERSION 回复（仅取常用字段）。
 * @param  d    payload 主体（不含命令 ID）。
 * @param  len  主体长度。
 * @param  fw   输出结构体（主/次版本号、硬件名、UUID）。
 */
static void parse_fw(const uint8_t *d, uint16_t len, vesc_fw_t *fw)
{
	int32_t i = 0;
	memset(fw, 0, sizeof(*fw));
	if (len < 2)
	{
		return;
	}
	fw->major = d[i++];
	fw->minor = d[i++];
	/* 硬件名：C 字符串，截断防溢出 */
	uint32_t k = 0;
	while (i < len && d[i] != 0 && k < sizeof(fw->hw_name) - 1)
	{
		fw->hw_name[k++] = (char)d[i++];
	}
	fw->hw_name[k] = 0;
	while (i < len && d[i] != 0)
		i++; /* 跳过剩余名字 */
	if (i < len)
		i++; /* 跳过 \0 */
	if (i + 12 <= len)
	{
		memcpy(fw->uuid, d + i, 12);
	}
}

/**
 * @brief  一帧合法 payload 解码成功后的回调分发。
 * @details 先调用通用 on_packet，再按命令 ID 解析并调用 on_values / on_fw_version。
 * @param  vp       协议实例。
 * @param  payload  完整 payload（payload[0] 为命令 ID）。
 * @param  len      payload 长度。
 */
static void dispatch(vesc_proto_t *vp, const uint8_t *payload, uint16_t len)
{
	if (len == 0)
	{
		return;
	}
	uint8_t id = payload[0];
	const uint8_t *body = payload + 1;
	uint16_t blen = len - 1;

	if (vp->cfg.on_packet)
	{
		vp->cfg.on_packet(id, body, blen, vp->cfg.ctx);
	}
	if (id == COMM_GET_VALUES && vp->cfg.on_values)
	{
		vesc_values_t v;
		parse_values(body, blen, &v);
		vp->cfg.on_values(&v, vp->cfg.ctx);
	}
	else if (id == COMM_FW_VERSION && vp->cfg.on_fw_version)
	{
		vesc_fw_t fw;
		parse_fw(body, blen, &fw);
		vp->cfg.on_fw_version(&fw, vp->cfg.ctx);
	}
}

/**
 * @brief  尝试从缓冲区起始处解码一帧。
 * @details 校验起始字节、长度编码、停止字节与 CRC，成功则调用 dispatch()。
 * @param  vp          协议实例。
 * @param  buf         待解码缓冲区。
 * @param  in_len      缓冲区可用字节数。
 * @param  bytes_left  出参：还需多少字节才能继续判断（仅返回 -2 时有意义）。
 * @return >0 = 成功，返回整帧消耗的字节数；-1 = 结构非法；-2 = 数据不足。
 */
static int32_t try_decode(vesc_proto_t *vp, const uint8_t *buf,
						  uint32_t in_len, int32_t *bytes_left)
{
	*bytes_left = 0;
	if (in_len == 0)
	{
		*bytes_left = 1;
		return -2;
	}

	bool len_8b = (buf[0] == 2);
	bool len_16b = (buf[0] == 3);
	uint32_t data_start = buf[0];

	if (!len_8b && !len_16b)
	{
		return -1; /* 无效起始字节 */
	}
	if (in_len < data_start)
	{
		*bytes_left = data_start - in_len;
		return -2;
	}

	uint32_t len;
	if (len_8b)
	{
		len = buf[1];
		if (len < 1)
			return -1;
	}
	else
	{
		len = ((uint32_t)buf[1] << 8) | buf[2];
		if (len < 255)
			return -1; /* 应使用更短的长度编码 */
	}
	if (len > VESC_PROTO_MAX_PL)
	{
		return -1;
	}

	/* 完整帧需要 data_start + len + 2(CRC) + 1(stop) */
	if (in_len < (len + data_start + 3))
	{
		*bytes_left = (len + data_start + 3) - in_len;
		return -2;
	}
	if (buf[data_start + len + 2] != 3)
	{
		return -1; /* 无效停止字节 */
	}

	uint16_t crc_calc = vesc_crc16(buf + data_start, len);
	uint16_t crc_rx = ((uint16_t)buf[data_start + len] << 8) | buf[data_start + len + 1];
	if (crc_calc != crc_rx)
	{
		return -1;
	}

	dispatch(vp, buf + data_start, (uint16_t)len);
	return (int32_t)(len + data_start + 3);
}

/**
 * @brief  处理单个接收字节，驱动拆帧状态机。
 * @details 使用线性缓冲（非环形），满时左移对齐；借助 bytes_left 快进，
 *          解码失败时丢弃 1 字节重新对齐。
 * @param  vp  协议实例。
 * @param  rx  接收到的字节。
 */
static void process_byte(vesc_proto_t *vp, uint8_t rx)
{
	uint32_t data_len = vp->rx_wr - vp->rx_rd;

	/* 缓冲溢出保护（正常不会发生） */
	if (data_len >= VESC_PROTO_BUF_LEN)
	{
		vp->rx_rd = 0;
		vp->rx_wr = 0;
		vp->bytes_left = 0;
		vp->rx_buf[vp->rx_wr++] = rx;
		return;
	}

	/* 写指针到顶则左移数据（线性缓冲，非环形） */
	if (vp->rx_wr >= VESC_PROTO_BUF_LEN)
	{
		memmove(vp->rx_buf, vp->rx_buf + vp->rx_rd, data_len);
		vp->rx_rd = 0;
		vp->rx_wr = (uint16_t)data_len;
	}

	vp->rx_buf[vp->rx_wr++] = rx;
	data_len++;

	/* 已知还差多个字节，先快进累计 */
	if (vp->bytes_left > 1)
	{
		vp->bytes_left--;
		return;
	}

	for (;;)
	{
		int32_t left;
		int32_t res = try_decode(vp, vp->rx_buf + vp->rx_rd, data_len, &left);
		vp->bytes_left = left;

		if (res == -2)
		{
			break; /* 需要更多数据 */
		}
		if (res > 0)
		{
			data_len -= (uint32_t)res;
			vp->rx_rd += (uint16_t)res;
		}
		else
		{ /* res == -1，丢弃 1 字节重新对齐 */
			vp->rx_rd++;
			data_len--;
		}
		if (data_len == 0)
		{
			break;
		}
	}

	if (data_len == 0)
	{
		vp->rx_rd = 0;
		vp->rx_wr = 0;
	}
}

/**
 * @brief  批量喂入接收字节（DMA / 中断收到一批后调用）。
 * @details 逐字节驱动拆帧状态机，凑齐合法帧后在本函数调用栈内同步触发回调。
 * @param  vp    协议实例。
 * @param  data  接收到的字节缓冲。
 * @param  len   字节数。
 */
void vesc_proto_recv(vesc_proto_t *vp, const uint8_t *data, uint16_t len)
{
	for (uint16_t i = 0; i < len; i++)
	{
		process_byte(vp, data[i]);
	}
}
