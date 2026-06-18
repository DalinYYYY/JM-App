/*
 * example_jm_proto_uart.c - 关节电机串口协议接入示例(仅参考, 不参与编译)
 *
 * 展示三步: 1) 实现业务回调 ops  2) 初始化绑定串口发送  3) 喂收到的字节 / 周期上报
 */
#include <stddef.h>
#include "jm_proto_uart.h"

/* ---- 1) 业务回调: 把协议命令对接到真正的电机控制/参数系统 ---- */
static jm_err_e app_set_mode(uint8_t cmd, const uint8_t *pl, uint16_t len)
{
	/* cmd 即 ctrl_mode_e。按模式解析载荷(小端助手), 调用 motor_loop/foc 下发。*/
	switch (cmd)
	{
		case JM_CMD_ENABLE: /* 上使能 */ break;
		case JM_CMD_DISABLE: /* 下使能 */ break;
		case JM_CMD_POSITION: /* 位置环: pos=jm_rd_f32(pl) */
			if (len < 4)
				return JM_ERR_LENGTH;
			/* float pos = jm_rd_f32(pl); motor_set_position(pos); */
			break;
		case JM_CMD_VELOCITY:
			if (len < 4)
				return JM_ERR_LENGTH;
			/* float vel = jm_rd_f32(pl); ... */
			break;
		/* 其余模式按 CSV 载荷定义解析... */
		default: return JM_ERR_UNSUPPORTED;
	}
	return JM_ERR_OK;
}

static jm_err_e app_get_feedback(jm_feedback_t *fb)
{
	/* 从 usr.motor_state[M1].ctrl_data / dev_power_monitor 填充 */
	fb->pos = 0.0f;
	fb->vel = 0.0f;
	fb->torque = 0.0f;
	fb->vbus = 24.0f;
	fb->temp_motor = 28.0f;
	fb->top_fsm = 4;
	fb->run_state = 0;
	fb->ctrl_mode = 0;
	fb->enable = 1;
	return JM_ERR_OK;
}

static jm_err_e app_param_read(uint16_t id, uint8_t *val, uint8_t *type, uint8_t *vlen)
{
	/* 按 joint_motor_param_index.csv 的 param_id 取 motor_param 字段, 小端写入 val */
	(void)id;
	jm_wr_f32(val, 1.234f);
	*type = JM_PT_F32;
	*vlen = 4;
	return JM_ERR_OK;
}

static jm_err_e app_param_write(uint16_t id, const uint8_t *val, uint8_t len)
{
	(void)id;
	(void)val;
	(void)len;
	/* motor_param_set_xxx(...) */
	return JM_ERR_OK;
}

static const jm_proto_ops_t app_ops = {
	.set_mode = app_set_mode,
	.get_feedback = app_get_feedback,
	.param_read = app_param_read,
	.param_write = app_param_write,
	.param_save = NULL,
	.param_reset = NULL,
	.get_dev_info = NULL,
	.get_dev_name = NULL,
};

/* ---- 2) 初始化: 注入串口发送(包裹你的 drv_usart 发送) ---- */
static jm_proto_uart_t s_jm_uart;

extern void my_uart_send(uint8_t *data, uint16_t len); /* 例: drv_usart DMA 发送 */

void app_jm_uart_setup(void)
{
	jm_proto_uart_init(&s_jm_uart, &app_ops, 0, my_uart_send); /*motor_id,串口可填0*/
}

/* ---- 3a) 在串口空闲中断/接收线程里喂入收到的字节 ---- */
void app_jm_uart_on_rx(uint8_t *buf, uint16_t len)
{
	jm_proto_uart_feed(&s_jm_uart, buf, len); /* 收齐整帧自动分发+应答 */
}

/* ---- 3b) (可选)周期主动上报实时反馈, 无需上位机轮询 ---- */
void app_jm_uart_report(void)
{
	jm_feedback_t fb;
	uint8_t body[22];
	uint16_t n = 0;
	app_get_feedback(&fb);
	jm_wr_f32(&body[0], fb.pos);
	jm_wr_f32(&body[4], fb.vel);
	jm_wr_f32(&body[8], fb.torque);
	jm_wr_f32(&body[12], fb.temp_motor);
	jm_wr_f32(&body[16], fb.vbus);
	jm_wr_u16(&body[20], (uint16_t)fb.fault_mask);
	n = 22;
	jm_proto_uart_send(&s_jm_uart, JM_CMD_READ_FEEDBACK, body, n);
}
