/**
 * @file        serialstudio_commun.c
 * @brief       SerialStudio 上位机通信(承载 joint_proto 协议)接入层实现
 *
 * @author      Dalin (dalin@robot.com)
 * @version     1.0
 * @date        2026-06-22
 *
 * @copyright   Copyright (c) 2026 Robot Tech.co, Ltd. All rights reserved.
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 * @note        从 thread_commun 抽出, 收敛 SerialStudio/joint_proto 相关逻辑:
 *              - joint_proto 业务回调(反馈/参数/控制模式)
 *              - 同步遥测订阅(0xCB)与单帧打包上报(0xCA)
 *              通信线程仅需 ss_commun_init() + 每周期 ss_commun_process()。
 */
#include "serialstudio_commun.h"
#if defined(USE_DEV_COMMUN_UART)

#include "runtime_param.h"
#include "thread_config.h"
#include "dev_commun_uart.h"
#include "jm_proto_ops.h" /* 传输无关业务回调集(串口/CAN 共用) */

/* ---------------- joint_proto 业务回调 ----------------
 * 反馈/控制/参数/设备信息/遥测的真正实现统一收敛在 jm_proto_ops.c,
 * 串口与 CAN 注入同一份 ops。本文件只负责串口绑定与遥测帧打包上报。
 * 反馈读取复用 ops 同源接口, 保证遥测打包与命令应答口径一致。*/

/* ---------------- 同步遥测周期状态 ----------------
 * 约束沿用: packer 单全局 send_buffer + 无发送忙查询, 故每个上报节拍只发一帧。
 * 改为固定全量帧后, 不再按 mask 选组; 0xCB 仅用于调节上报周期(period_ms)。
 * 订阅(0xCB)由共用 ops 统一接收并记录; 本文件每拍读其 period_ms 换算成 tick。*/
#define COMMUN_TELEMETRY_TICK 5u /* 默认上报节拍: 每 5 个通信 tick(≈5ms) 发一帧 */

/* 当前上报周期(单位: 通信 tick): 由 ops 记录的 period_ms 换算, 0 表示沿用默认 */
static uint16_t commun_uart_telemetry_tick(void)
{
	uint16_t period_ms = jm_app_telemetry_period_ms();
	uint16_t tick;
	if (period_ms == 0u)
	{
		return COMMUN_TELEMETRY_TICK;
	}
	/* 通信线程周期为 THREAD_DELAY_COMMUN ms, 换算成 tick 数, 至少 1 */
	tick = (uint16_t)(period_ms / THREAD_DELAY_COMMUN);
	return (tick == 0u) ? 1u : tick;
}

/* ---------------- 同步遥测(固定全量帧上传) ----------------
 * 每拍发送【固定长度、固定字段、固定顺序】的全量帧, 不再用 mask 变长。
 * 帧体严格按上位机 Frame Index 1..21 顺序排列, 每槽 4 字节(统一 f32, 状态/计数量
 * 也转 f32), 共 84 字节。这样上位机解析器只需按固定偏移读 21 个 f32 原样返回,
 * dataset.index=i 自然对应 array[i-1], 映射零歧义; 每帧每字段都有新值, 无"保留上次"。
 *
 * 顺序: 1 pos 2 vel 3 torque 4 tempFet 5 tempMotor 6 vbus 7 ibus 8 power
 *       9 id 10 iq 11 ia 12 ib 13 ic 14 multiturn 15 single
 *       16 fault 17 warn 18 topFsm 19 runState 20 ctrlMode 21 enable */
#define SS_TLM_SLOTS 21u /* 固定槽位数(Frame Index 1..21) */

static uint16_t commun_uart_pack_telemetry(const jm_feedback_t *fb, uint8_t *o)
{
	uint16_t n = 0;
	jm_wr_f32(&o[n], fb->pos);
	n += 4; /* 1  */
	jm_wr_f32(&o[n], fb->vel);
	n += 4; /* 2  */
	jm_wr_f32(&o[n], fb->torque);
	n += 4; /* 3  */
	jm_wr_f32(&o[n], fb->temp_fet);
	n += 4; /* 4  */
	jm_wr_f32(&o[n], fb->temp_motor);
	n += 4; /* 5  */
	jm_wr_f32(&o[n], fb->vbus);
	n += 4; /* 6  */
	jm_wr_f32(&o[n], fb->ibus);
	n += 4; /* 7  */
	jm_wr_f32(&o[n], fb->vbus * fb->ibus);
	n += 4; /* 8 power */
	jm_wr_f32(&o[n], fb->id);
	n += 4; /* 9  */
	jm_wr_f32(&o[n], fb->iq);
	n += 4; /* 10 */
	jm_wr_f32(&o[n], fb->ia);
	n += 4; /* 11 */
	jm_wr_f32(&o[n], fb->ib);
	n += 4; /* 12 */
	jm_wr_f32(&o[n], fb->ic);
	n += 4; /* 13 */
	jm_wr_f32(&o[n], (float)fb->multiturn);
	n += 4; /* 14 */
	jm_wr_f32(&o[n], fb->single);
	n += 4; /* 15 */
	jm_wr_f32(&o[n], (float)fb->fault_mask);
	n += 4; /* 16 */
	jm_wr_f32(&o[n], (float)fb->warn_mask);
	n += 4; /* 17 */
	jm_wr_f32(&o[n], (float)fb->top_fsm);
	n += 4; /* 18 */
	jm_wr_f32(&o[n], (float)fb->run_state);
	n += 4; /* 19 */
	jm_wr_f32(&o[n], (float)fb->ctrl_mode);
	n += 4; /* 20 */
	jm_wr_f32(&o[n], (float)fb->enable);
	n += 4; /* 21 */
	return n;
}

static void commun_uart_push_telemetry(dev_commun_uart_t *dev)
{
	jm_feedback_t fb;
	uint8_t o[SS_TLM_SLOTS * 4]; /* 固定 84 字节全量帧体 */
	uint16_t n;

	if (jm_app_get_feedback(&fb) != JM_ERR_OK)
	{
		return;
	}

	n = commun_uart_pack_telemetry(&fb, o);
	dev->report(dev, JM_CMD_TELEMETRY, o, n);
}

/* ---------------- 对外接口 ---------------- */

void ss_commun_init(void)
{
	/* 关节电机串口通信(USART+DMA空闲中断): 初始化→注入业务回调→启动接收 */
	dev_commun_uart_init(&dev_commun_uart, JM_UART_COMM_ID_1);
	dev_commun_uart.set_ops(&dev_commun_uart, jm_app_ops_get()); /* 串口/CAN 共用同一份回调 */
	dev_commun_uart.start(&dev_commun_uart);
}

void ss_commun_process(void)
{
	static uint8_t telemetry_tick = 0; /* 遥测上报分频计数 */

	/* 取空闲突发数据喂协议栈, 自动完成命令分发与应答 */
	dev_commun_uart.poll(&dev_commun_uart);

	/* 周期主动上报遥测: 按订阅周期分频推一帧, 错峰避免 DMA 撞车 */
	// if (++telemetry_tick >= commun_uart_telemetry_tick())
	// {
	// 	telemetry_tick = 0;
	// 	commun_uart_push_telemetry(&dev_commun_uart);
	// }
}

#endif /* USE_DEV_COMMUN_UART */
