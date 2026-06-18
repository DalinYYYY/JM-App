/**
 * @file    vesc_slave.c
 * @brief   VESC 从机层实现：响应 VESC Tool 的握手与实时值请求。
 */
#include "vesc_slave.h"
#include <string.h>

/* ---- 大端编码助手 ---- */
static void put_u32(uint8_t *b, uint32_t v, int32_t *i) {
	b[(*i)++] = (uint8_t)(v >> 24);
	b[(*i)++] = (uint8_t)(v >> 16);
	b[(*i)++] = (uint8_t)(v >> 8);
	b[(*i)++] = (uint8_t)v;
}
static void put_i32(uint8_t *b, int32_t v, int32_t *i) { put_u32(b, (uint32_t)v, i); }
static void put_i16(uint8_t *b, int16_t v, int32_t *i) {
	b[(*i)++] = (uint8_t)(v >> 8);
	b[(*i)++] = (uint8_t)v;
}
/* 定点浮点编码 */
static void put_f16(uint8_t *b, float val, float scale, int32_t *i) {
	put_i16(b, (int16_t)(val * scale), i);
}
static void put_f32(uint8_t *b, float val, float scale, int32_t *i) {
	put_i32(b, (int32_t)(val * scale), i);
}
static uint32_t get_u32(const uint8_t *b) {
	return ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) |
	       ((uint32_t)b[2] << 8) | (uint32_t)b[3];
}

/* ==================================================================== */
/*  COMM_FW_VERSION 回复 —— VESC Tool 握手识别的关键                      */
/* ==================================================================== */
static void reply_fw_version(vesc_slave_t *sl) {
	uint8_t buf[80];
	int32_t n = 0;
	const vesc_slave_cfg_t *c = &sl->cfg;

	buf[n++] = COMM_FW_VERSION;
	buf[n++] = c->fw_major;
	buf[n++] = c->fw_minor;

	/* 硬件名称字符串（含结尾 \0） */
	const char *hw = c->hw_name ? c->hw_name : "VESC_PROTO";
	size_t hw_len = strlen(hw);
	memcpy(buf + n, hw, hw_len + 1);
	n += (int32_t)hw_len + 1;

	/* 12 字节 UUID */
	memcpy(buf + n, c->uuid, 12);
	n += 12;

	buf[n++] = 0;            /* pairing_done */
	buf[n++] = 0;            /* 测试版本号 */
	buf[n++] = c->hw_type;   /* HW_TYPE（0 = HW_TYPE_VESC） */
	buf[n++] = 0;            /* 自定义配置数量 */
	buf[n++] = 0;            /* 相位滤波器标志 */
	buf[n++] = 0;            /* QMLUI_HW 标志 */
	buf[n++] = 0;            /* QMLUI_APP 标志 */
	buf[n++] = 0;            /* nrf_flags */

	/* 固件名称字符串（含结尾 \0） */
	const char *fw = c->fw_name ? c->fw_name : "";
	size_t fw_len = strlen(fw);
	memcpy(buf + n, fw, fw_len + 1);
	n += (int32_t)fw_len + 1;

	put_u32(buf, 0, &n);     /* 硬件配置 CRC（无可填 0） */

	vesc_proto_send(&sl->proto, buf, (uint16_t)n);
}

/* ==================================================================== */
/*  COMM_GET_VALUES / _SELECTIVE 回复 —— 仪表盘实时数据                   */
/* ==================================================================== */
/* 按 mask 逐位编码字段。GET_VALUES 等价 mask=0xFFFFFFFF。
 * 字段顺序与缩放须与固件 commands.c 完全一致。 */
static void reply_get_values(vesc_slave_t *sl, uint32_t mask, bool selective) {
	vesc_values_t v;
	memset(&v, 0, sizeof(v));
	if (sl->cfg.get_values) {
		sl->cfg.get_values(&v, sl->cfg.ctx);
	}

	uint8_t buf[80];
	int32_t n = 0;
	buf[n++] = selective ? COMM_GET_VALUES_SELECTIVE : COMM_GET_VALUES;
	if (selective) {
		put_u32(buf, mask, &n);  /* selective 回复回显 mask */
	}

	if (mask & ((uint32_t)1 << 0))  put_f16(buf, v.temp_fet, 1e1f, &n);
	if (mask & ((uint32_t)1 << 1))  put_f16(buf, v.temp_motor, 1e1f, &n);
	if (mask & ((uint32_t)1 << 2))  put_f32(buf, v.current_motor, 1e2f, &n);
	if (mask & ((uint32_t)1 << 3))  put_f32(buf, v.current_in, 1e2f, &n);
	if (mask & ((uint32_t)1 << 4))  put_f32(buf, v.id, 1e2f, &n);
	if (mask & ((uint32_t)1 << 5))  put_f32(buf, v.iq, 1e2f, &n);
	if (mask & ((uint32_t)1 << 6))  put_f16(buf, v.duty, 1e3f, &n);
	if (mask & ((uint32_t)1 << 7))  put_f32(buf, v.rpm, 1e0f, &n);
	if (mask & ((uint32_t)1 << 8))  put_f16(buf, v.v_in, 1e1f, &n);
	if (mask & ((uint32_t)1 << 9))  put_f32(buf, v.amp_hours, 1e4f, &n);
	if (mask & ((uint32_t)1 << 10)) put_f32(buf, v.amp_hours_chg, 1e4f, &n);
	if (mask & ((uint32_t)1 << 11)) put_f32(buf, v.watt_hours, 1e4f, &n);
	if (mask & ((uint32_t)1 << 12)) put_f32(buf, v.watt_hours_chg, 1e4f, &n);
	if (mask & ((uint32_t)1 << 13)) put_i32(buf, v.tacho, &n);
	if (mask & ((uint32_t)1 << 14)) put_i32(buf, v.tacho_abs, &n);
	if (mask & ((uint32_t)1 << 15)) buf[n++] = v.fault_code;
	if (mask & ((uint32_t)1 << 16)) put_f32(buf, v.pid_pos, 1e6f, &n);
	if (mask & ((uint32_t)1 << 17)) buf[n++] = v.controller_id;
	if (mask & ((uint32_t)1 << 18)) {       /* 3 路 MOS 温度，无数据填 FET 温度 */
		put_f16(buf, v.temp_fet, 1e1f, &n);
		put_f16(buf, v.temp_fet, 1e1f, &n);
		put_f16(buf, v.temp_fet, 1e1f, &n);
	}
	if (mask & ((uint32_t)1 << 19)) put_f32(buf, 0.0f, 1e3f, &n);  /* vd（无数据） */
	if (mask & ((uint32_t)1 << 20)) put_f32(buf, 0.0f, 1e3f, &n);  /* vq（无数据） */
	if (mask & ((uint32_t)1 << 21)) buf[n++] = 0;                  /* 状态位 */

	vesc_proto_send(&sl->proto, buf, (uint16_t)n);
}

/* ==================================================================== */
/*  请求分发                                                            */
/* ==================================================================== */
/* 帧层每收到一帧合法 payload 都会触发此回调（cmd_id + 主体）。 */
static void on_request(uint8_t cmd_id, const uint8_t *body, uint16_t len, void *ctx) {
	vesc_slave_t *sl = (vesc_slave_t *)ctx;

	switch (cmd_id) {
	case COMM_FW_VERSION:
		reply_fw_version(sl);
		break;

	case COMM_GET_VALUES:
		reply_get_values(sl, 0xFFFFFFFFu, false);
		break;

	case COMM_GET_VALUES_SELECTIVE: {
		uint32_t mask = (len >= 4) ? get_u32(body) : 0xFFFFFFFFu;
		reply_get_values(sl, mask, true);
		break;
	}

	case COMM_ALIVE:
		/* 心跳无需回复 */
		break;

	default:
		/* 其余请求交给应用扩展（如 MCCONF / APPCONF） */
		if (sl->cfg.user_packet) {
			sl->cfg.user_packet(&sl->proto, cmd_id, body, len, sl->cfg.ctx);
		}
		break;
	}
}

/* ==================================================================== */
/*  公共接口                                                            */
/* ==================================================================== */
void vesc_slave_init(vesc_slave_t *sl, const vesc_slave_cfg_t *cfg) {
	memset(sl, 0, sizeof(*sl));
	if (cfg) {
		sl->cfg = *cfg;
	}

	/* 配置底层帧层：发送回调透传，接收回调路由到 on_request。
	 * ctx 指向从机实例本身，供 on_request 取回。 */
	vesc_proto_cfg_t pcfg;
	memset(&pcfg, 0, sizeof(pcfg));
	pcfg.send_bytes = sl->cfg.send_bytes;
	pcfg.on_packet  = on_request;
	pcfg.ctx        = sl;
	vesc_proto_init(&sl->proto, &pcfg);
}

void vesc_slave_recv(vesc_slave_t *sl, const uint8_t *data, uint16_t len) {
	vesc_proto_recv(&sl->proto, data, len);
}

