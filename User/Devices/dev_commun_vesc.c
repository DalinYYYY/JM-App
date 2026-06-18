/**
 * @file        dev_commun_vesc.c
 * @brief       VESC Tool 串口通信设备(USART+DMA空闲中断, 把本机伪装成VESC从机)
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
 * @note        分层: 本设备(粘合层) → vesc_slave(从机命令) → vesc_proto(帧/CRC)
 *                                  ↘ drv_usart(USART+DMA空闲中断)
 *              slave 为对象首成员: 协议栈回调的 ctx 即指向内嵌 slave,
 *              其地址与设备对象首地址相同, 故所有回调可直接 cast 回设备对象。
 */
#include "dev_commun_vesc.h"
#if defined(USE_DEV_COMMUN_VESC)

#include "assert_report.h"
#include <string.h>

dev_commun_vesc_t dev_commun_vesc;

/* ==================================================================== */
/*  协议栈回调(ctx 均解析为设备对象, 见文件头说明)                        */
/* ==================================================================== */

/* 发送回调: 协议栈组好整帧后, 经串口 DMA 一次性推出 */
static void vesc_cb_send_bytes(const uint8_t *data, uint16_t len, void *ctx)
{
	dev_commun_vesc_t *pobj = (dev_commun_vesc_t *)ctx;
	assert_report(pobj != NULL);
	/* tx_buf 驻留在协议栈实例内, DMA 期间不被覆盖(回复短且低频) */
	drv_uart_dma_send(pobj->uart, (uint8_t *)data, len);
}

/* 取值回调: VESC Tool 请求实时值时触发, 转交应用注入的数据源 */
static void vesc_cb_get_values(vesc_values_t *v, void *ctx)
{
	dev_commun_vesc_t *pobj = (dev_commun_vesc_t *)ctx;
	assert_report(pobj != NULL);
	if (pobj->fill_values != NULL)
	{
		pobj->fill_values(v); /* 未注入则维持协议栈清零默认值 */
	}
}

/* ==================================================================== */
/*  设备接口                                                            */
/* ==================================================================== */

/* 启动空闲中断+DMA不定长接收(幂等: 重复调用只生效一次) */
static int dev_commun_vesc_start(struct dev_commun_vesc *pobj)
{
	assert_report(pobj != NULL);
	if (pobj->started)
	{
		return DEV_EOK;
	}
	if (usart_idle_init(pobj->uart, DEV_VESC_RX_BUF_SIZE) != DRV_EOK)
	{
		return DEV_ERROR;
	}
	pobj->started = 1;
	return DEV_EOK;
}

/* 串口 IRQ 中调用: 检测 IDLE 标志, 停 DMA 并锁存本帧长度 */
static void dev_commun_vesc_on_rx_idle(struct dev_commun_vesc *pobj)
{
	assert_report(pobj != NULL);
	drv_uart_idle(pobj->uart);
}

/* 主循环/线程周期调用: 取出空闲突发数据喂协议栈, 自动完成识别与回复 */
static void dev_commun_vesc_poll(struct dev_commun_vesc *pobj)
{
	uint16_t rx_len = 0;
	assert_report(pobj != NULL);
	if (!pobj->started)
	{
		return;
	}

	usart_idle_get_data(pobj->uart, pobj->rx_tmp, &rx_len);
	if (rx_len != 0)
	{
		vesc_slave_recv(&pobj->slave, pobj->rx_tmp, rx_len);
	}
}

/* 注入仪表盘实时值数据源(应用把传感器/控制量填进 vesc_values_t) */
static void dev_commun_vesc_set_values_cb(struct dev_commun_vesc *pobj,
										  void (*cb)(vesc_values_t *v))
{
	assert_report(pobj != NULL);
	pobj->fill_values = cb;
}

void dev_commun_vesc_init(dev_commun_vesc_t *pobj, vesc_comm_id_e id)
{
	assert_report(pobj != NULL);
	assert_report(id < VESC_COMM_ID_MAX);
	memset(pobj, 0, sizeof(dev_commun_vesc_t));

	const dev_commun_vesc_config_t *cfg = &commun_vesc_list[id];
	pobj->uart = cfg->uart;

	/* 组装从机协议栈: send/get_values 必填; ctx 指向设备对象本身。
	 * 由于 slave 是设备对象首成员, &slave == pobj, 各回调 ctx 一致。 */
	vesc_slave_cfg_t scfg;
	memset(&scfg, 0, sizeof(scfg));
	scfg.send_bytes = vesc_cb_send_bytes;
	scfg.get_values = vesc_cb_get_values;
	scfg.hw_name = cfg->hw_name;
	scfg.fw_name = cfg->fw_name;
	scfg.fw_major = cfg->fw_major;
	scfg.fw_minor = cfg->fw_minor;
	scfg.hw_type = 0; /* HW_TYPE_VESC */
	scfg.ctx = pobj;
	vesc_slave_init(&pobj->slave, &scfg);

	/* public */
	pobj->start = dev_commun_vesc_start;
	pobj->on_rx_idle = dev_commun_vesc_on_rx_idle;
	pobj->poll = dev_commun_vesc_poll;
	pobj->set_values_cb = dev_commun_vesc_set_values_cb;
}

#endif /* USE_DEV_COMMUN_VESC */
