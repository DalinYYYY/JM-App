/*
 * example_slave.c - 把 MCU 做成可被 VESC Tool 识别的 VESC 设备（场景 A）
 *
 * 本文件仅作参考，不参与固件编译。把 HAL_* 换成你平台的 USART API 即可。
 *
 * 连接流程：
 *   1. VESC Tool 打开串口 → 发 COMM_FW_VERSION → 本模块回复设备身份 → 被识别。
 *   2. VESC Tool 周期性发 COMM_GET_VALUES → 本模块回填实时数据 → 仪表盘显示。
 */
#include "vesc_slave.h"
#include <stdint.h>

/* 假设的平台 USART API（请替换） */
extern void     HAL_UART_Transmit_App(const uint8_t *data, uint16_t len);
extern uint16_t HAL_UART_ReadBatch(uint8_t *buf, uint16_t max);

static vesc_slave_t s_slave;

/* ---- 回调：发送字节到 USART ---- */
static void cb_send(const uint8_t *data, uint16_t len, void *ctx) {
	(void)ctx;
	HAL_UART_Transmit_App(data, len);
}

/* ---- 回调：填充仪表盘实时值（用你自己的传感器数据替换）---- */
static void cb_get_values(vesc_values_t *v, void *ctx) {
	(void)ctx;
	v->temp_fet      = 32.5f;   /* 控制器温度 ℃ */
	v->temp_motor    = 28.0f;   /* 电机温度 ℃ */
	v->current_motor = 1.2f;    /* 电机电流 A */
	v->current_in    = 0.8f;    /* 输入电流 A */
	v->duty          = 0.15f;   /* 占空比 */
	v->rpm           = 1500.0f; /* 电气转速 */
	v->v_in          = 24.3f;   /* 输入电压 V */
	v->fault_code    = 0;       /* 无故障 */
	v->controller_id = 0;
}

void slave_init(void) {
	vesc_slave_cfg_t cfg = {0};
	cfg.send_bytes = cb_send;        /* 必填 */
	cfg.get_values = cb_get_values;  /* 必填 */
	cfg.hw_name    = "MyBoard";      /* VESC Tool 上显示的硬件名 */
	cfg.fw_name    = "myfw";
	cfg.fw_major   = 6;
	cfg.fw_minor   = 0;
	cfg.hw_type    = 0;              /* HW_TYPE_VESC */
	/* cfg.uuid 可填芯片唯一 ID，留空则为全 0 */
	vesc_slave_init(&s_slave, &cfg);
}

/* 主循环 / 串口任务：把收到的字节喂进来，回复自动完成 */
void slave_poll(void) {
	uint8_t rxbuf[128];
	uint16_t n = HAL_UART_ReadBatch(rxbuf, sizeof(rxbuf));
	if (n) {
		vesc_slave_recv(&s_slave, rxbuf, n);
	}
}
