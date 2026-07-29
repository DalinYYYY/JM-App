/**
 * @file        jm_host_commun.c
 * @brief       关节电机上位机通信(承载 joint_proto 协议)接入层实现
 *
 * @author      Dalin (dalinyy@163.com)
 * @version     1.0
 * @date        2026-06-24
 *
 * @copyright   Copyright (c) 2026 Robot Tech.co, Ltd. All rights reserved.
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 * @note        面向自研 PyQt 上位机, 取代已废弃的 serialstudio_commun。
 *              joint_proto 业务回调(反馈/控制/参数/设备信息)统一收敛在 jm_proto_ops.c,
 *              串口与 CAN 注入同一份 ops; 本文件只负责串口绑定与遥测帧打包上报。
 * @note        遥控模式(周期无应答上报):
 *              - 上位机用 SET_TELEMETRY(0xCB)=enable+mask+period 配置开关/种类/周期;
 *                ops 记录后, 本模块每拍读 jm_app_telemetry_enabled() 决定是否上报。
 *              - 使能时按 mask 变长打包 TELEMETRY(0xCA): 帧体 = mask(u16) + 按位序拼接
 *                所选数据组, 位序严格对齐 jm_telemetry_bit_e 与上位机 parse_telemetry。
 *              - 数据帧不要求上位机逐帧应答; 仅 0xCB 开关回单次 ACK 供上位机确认。
 */
#include "jm_host_commun.h"

/* ---- 公共头文件(UART/CAN 共用遥测打包) ---- */
#include "runtime_param.h" /* JM_DBG_CH, jm_dbg[] */
#include "thread_config.h" /* THREAD_DELAY_COMMUN */
#include "jm_proto_ops.h" /* jm_app_telemetry_*/ + 传递引入 jm_proto.h / jm_cmd_def.h * /

#if defined(USE_DEV_COMMUN_UART) || defined(USE_DEV_COMMUN_CAN)
#define COMMUN_TELEMETRY_TICK 5u /* 默认上报节拍: 每 5 个通信 tick 发一帧 */
#endif

/* ---------------- 公共遥测打包函数(UART/CAN 共用) ----------------
 * pack_telemetry: 传输无关, 仅把 mask+feedback 打包成字节流
 * telemetry_tick:  把 period_ms 换算成通信 tick 数 */
static uint16_t jm_host_commun_telemetry_tick(void)
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

static uint16_t jm_host_commun_pack_telemetry(uint16_t mask, const jm_feedback_t *fb, uint8_t *o)
{
	uint16_t n = 0;

	jm_wr_u16(&o[n], mask);
	n += 2;                    /* 帧头: 订阅掩码 */

	if (mask & JM_TLM_POS_VEL) /* pos,vel  8B */
	{
		jm_wr_f32(&o[n], fb->pos);
		n += 4;
		jm_wr_f32(&o[n], fb->vel);
		n += 4;
	}
	if (mask & JM_TLM_DQ) /* id,iq  8B */
	{
		jm_wr_f32(&o[n], fb->id);
		n += 4;
		jm_wr_f32(&o[n], fb->iq);
		n += 4;
	}
	if (mask & JM_TLM_PHASE) /* ia,ib,ic  12B */
	{
		jm_wr_f32(&o[n], fb->ia);
		n += 4;
		jm_wr_f32(&o[n], fb->ib);
		n += 4;
		jm_wr_f32(&o[n], fb->ic);
		n += 4;
	}
	if (mask & JM_TLM_BUS) /* vbus,ibus,power  12B */
	{
		jm_wr_f32(&o[n], fb->vbus);
		n += 4;
		jm_wr_f32(&o[n], fb->ibus);
		n += 4;
		jm_wr_f32(&o[n], fb->vbus * fb->ibus);
		n += 4;
	}
	if (mask & JM_TLM_TEMP) /* tempFet,tempMotor  8B */
	{
		jm_wr_f32(&o[n], fb->temp_fet);
		n += 4;
		jm_wr_f32(&o[n], fb->temp_motor);
		n += 4;
	}
	if (mask & JM_TLM_MULTITURN) /* multiturn(u32),single(f32)  8B */
	{
		jm_wr_u32(&o[n], (uint32_t)fb->multiturn);
		n += 4;
		jm_wr_f32(&o[n], fb->single);
		n += 4;
	}
	if (mask & JM_TLM_TORQUE) /* torque  4B */
	{
		jm_wr_f32(&o[n], fb->torque);
		n += 4;
	}
	if (mask & JM_TLM_FAULT) /* fault(u32),warn(u32)  8B */
	{
		jm_wr_u32(&o[n], fb->fault_mask);
		n += 4;
		jm_wr_u32(&o[n], fb->warn_mask);
		n += 4;
	}
	if (mask & JM_TLM_STATE) /* topFsm,runState,ctrlMode,enable(u8)  4B */
	{
		o[n++] = fb->top_fsm;
		o[n++] = fb->run_state;
		o[n++] = fb->ctrl_mode;
		o[n++] = fb->enable;
	}
	if (mask & JM_TLM_DEBUG) /* jm_dbg[JM_DBG_CH](f32)  N*4B */
	{
		uint8_t i;
		for (i = 0; i < JM_DBG_CH; i++)
		{
			jm_wr_f32(&o[n], jm_dbg[i]);
			n += 4;
		}
	}
	return n;
}

#if defined(USE_DEV_COMMUN_UART)

#include "drv_rtos.h"
#include "dev_commun_uart.h"
#include "motor_loop.h" /* motor_loop_t: 调试通道通过 p_motor_loop 读 ADC 原始值 */

/* UART 专用的接收信号量与统计计数器 */
static drv_rtos_sem_handle_t s_rx_sem = NULL;
static volatile uint32_t s_rx_notify_count = 0;
static volatile uint32_t s_rx_wakeup_count = 0;
static volatile uint32_t s_rx_timeout_count = 0;

static void commun_uart_push_telemetry(dev_commun_uart_t *dev)
{
	jm_feedback_t fb;
	uint8_t o[2 + 64 + JM_DBG_CH * 4]; /* mask(2) + 最大固定组(<=64) + 调试通道 */
	uint16_t mask;
	uint16_t n;

	mask = jm_app_telemetry_mask();
	if (mask == 0u)
	{
		return; /* 未选任何组, 无需上报 */
	}
	if (jm_app_get_feedback(&fb) != JM_ERR_OK)
	{
		return;
	}

	n = jm_host_commun_pack_telemetry(mask, &fb, o);
	dev->report(dev, JM_CMD_TELEMETRY, o, n);
}

/* ---------------- 调试通道绑定 ----------------*/
/* [诊断] 电角度换相核对: 开环电压 Ud=0/Uq>0 时 Id/Iq 低频反相大摆动。
 * poles=20 已确认正确, 故疑点在 d轴对齐(enc_offset)或编码器方向(enc_direction)。
 * 重点判读 ch4(enc_dir):
 *   本架构 offset 在标定时强制 dir=+1 采样, 若真实 enc_dir=-1, 运行时 get_raw 反转 raw,
 *   offset 在"正向坐标系"标、"反向坐标系"用 → 机械零点错位 → d轴对齐失效 → Id/Iq大摆动。
 *   (calib_level3_encoder.c 已注释此隐患) 若 ch4=-1 且 Id 大 → 高度怀疑此错位, 需重标零位。
 * 判读:
 *   ch3(poles)=20; ch4(enc_dir)=±1; ch5(ele_deg)匀速转线性锯齿0~360;
 *   ch6(mech_deg)一圈内 ele 循环 poles 次; ch7(enc_offset)标定机械零位(deg)。
 * 核对完恢复电流/dq 遥测。*/
static void jm_host_commun_update_debug(void)
{
	const foc_t *fc = &usr.p_motor_loop->motor.foc;
	const motor_loop_t *ml = usr.p_motor_loop;
	const motion_param_t *mp = &usr.p_motor_loop->motor.motor_param;
	const motor_param_t *param = &usr.motor_param[M1];

	jm_dbg[0] = (float)ml->out.id_ref;                  /* d轴电流参考(A) */
	jm_dbg[1] = (float)ml->out.iq_ref;                  /* q轴电流参考(A) */
	jm_dbg[2] = 0.0f;                                   /* 保留 */
	jm_dbg[3] = (float)fc->i_dq.d;                      /* Id */
	jm_dbg[4] = (float)fc->i_dq.q;                      /* Iq */
	jm_dbg[5] = (float)fc->Theta;                       /* park 用电角度(rad) */
	jm_dbg[6] = (float)mp->mechanical_angle;            /* 机械角(deg) */
	jm_dbg[7] = (float)param->encoder_param.enc_offset; /* 编码器机械零位偏移(deg) */
}

/* ---------------- 对外接口 ---------------- */

void jm_host_commun_init(void)
{
	/* 关节电机串口通信(USART+DMA空闲中断): 初始化→注入业务回调→启动接收 */
	if (s_rx_sem == NULL)
	{
		s_rx_sem = drv_rtos_sem_create(1u, 1u);
	}
	if (s_rx_sem != NULL)
	{
		(void)drv_rtos_sem_wait(s_rx_sem, 0u);
	}
	dev_commun_uart_init(&dev_commun_uart, JM_UART_COMM_ID_1);
	dev_commun_uart.set_ops(&dev_commun_uart, jm_app_ops_get()); /* 串口/CAN 共用同一份回调 */
	dev_commun_uart.start(&dev_commun_uart);
}

void jm_host_commun_notify_rx(void)
{
	s_rx_notify_count++;
	if (s_rx_sem != NULL)
	{
		(void)drv_rtos_sem_release(s_rx_sem);
	}
}

void jm_host_commun_wait(uint32_t timeout_ms)
{
	if (s_rx_sem == NULL)
	{
		drv_rtos_delay_ms(timeout_ms);
		return;
	}
	if (drv_rtos_sem_wait(s_rx_sem, timeout_ms) == DRV_EOK)
	{
		s_rx_wakeup_count++;
	}
	else
	{
		s_rx_timeout_count++;
	}
}

void jm_host_commun_process(void)
{
	static uint8_t telemetry_tick = 0; /* 遥测上报分频计数 */

	/* 刷新调试通道: 从 motor_state 快照填充 jm_dbg[] */
	jm_host_commun_update_debug();

	/* 取空闲突发数据喂协议栈, 自动完成命令分发与应答 */
	dev_commun_uart.poll(&dev_commun_uart);

	/* 遥控模式: 仅当上位机用 0xCB 使能后才按订阅周期分频主动推送遥测帧(无应答)。
	 * 停止时不发, 且复位分频计数, 使下次使能后第一帧及时发出。*/
	if (!jm_app_telemetry_enabled())
	{
		telemetry_tick = 0;
		return;
	}
	if (++telemetry_tick >= jm_host_commun_telemetry_tick())
	{
		telemetry_tick = 0;
		commun_uart_push_telemetry(&dev_commun_uart);
	}
}

#endif /* USE_DEV_COMMUN_UART */

/* ====================================================================== */
/* CAN/CAN-FD 通信接入层 (与 UART 路径并存, 业务回调共用 jm_proto_ops) */
/* ====================================================================== */
#if defined(USE_DEV_COMMUN_CAN)
#include "dev_commun_can.h"
#include "main.h" /* HAL_GetTick (降级检查) */

/* 双通道并发: 开发期关闭主控仲裁(JM_DUAL_CHANNEL_ARB_ENABLE=0),
 * UART 和 CAN 可同时下发命令, motor_state 临界区(__disable_irq)保证原子性。
 * 两路各自独立 init/process, 互不干扰。*/

void jm_host_commun_can_init(void)
{
	/* CAN 通信: init → 注入业务回调(与 UART 共用) → 启动 CAN */
	dev_commun_can_init(&dev_commun_can, JM_CAN_COMM_ID_1);
	dev_commun_can.set_ops(&dev_commun_can, jm_app_ops_get());
	dev_commun_can.start(&dev_commun_can);
}

void jm_host_commun_can_process(void)
{
	static uint8_t telemetry_tick = 0; /* 遥测上报分频计数 */

	/* 刷新诊断统计(供应用读取总线负载/通信质量) */
	dev_commun_can.poll(&dev_commun_can);

	/* 通信中断降级: 500ms 内 HOLD, 超时强制切 IDLE 停机 */
	// if (dev_commun_can.check_loss(&dev_commun_can, HAL_GetTick()))
	// {
	// 	__disable_irq();
	// 	motor_loop_set_cmd(CONTROL_MODE_IDLE);
	// 	__enable_irq();
	// }

	/* 遥控模式: 仅当上位机用 0xCB 使能后才按订阅周期分频主动推送遥测帧(无应答)。
	 * 停止时不发, 且复位分频计数, 使下次使能后第一帧及时发出。
	 * 与 UART 侧对称, 共用 jm_host_commun_pack_telemetry 打包, report 走 CAN 多帧发送。*/
	if (!jm_app_telemetry_enabled())
	{
		telemetry_tick = 0;
		return;
	}
	if (++telemetry_tick >= jm_host_commun_telemetry_tick())
	{
		jm_feedback_t fb;
		uint8_t o[2 + 64 + JM_DBG_CH * 4]; /* mask(2) + 最大固定组(<=64) + 调试通道 */
		uint16_t mask = jm_app_telemetry_mask();
		if (mask != 0u && jm_app_get_feedback(&fb) == JM_ERR_OK)
		{
			uint16_t n = jm_host_commun_pack_telemetry(mask, &fb, o);
			dev_commun_can.report(&dev_commun_can, JM_CMD_TELEMETRY, o, n);
		}
		telemetry_tick = 0;
	}
}

#endif /* USE_DEV_COMMUN_CAN */
