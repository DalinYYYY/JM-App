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

/* ---------------- joint_proto 业务回调: 上位机命令的真正动作落点 ---------------- */

/* 设置控制模式并下发目标(CMD 0x00~0xB8) */
static jm_err_e commun_uart_set_mode(uint8_t cmd, const uint8_t *payload, uint16_t len)
{
	(void)cmd;
	(void)payload;
	(void)len;
	/* TODO: 按 cmd 解析载荷并下发到电机控制层 */
	return JM_ERR_OK;
}

/* 读实时反馈: 从运行参数填充 */
static jm_err_e commun_uart_get_feedback(jm_feedback_t *fb)
{
	const motor_state_t *m = &usr.motor_state[M1];

	fb->pos = m->motion.position_rad;		/* 输出端多圈位置 rad */
	fb->vel = m->motion.velocity_rad_s;		/* 输出端速度 rad/s */
	fb->torque = m->power.torque_est;		/* 输出端力矩 Nm (估算) */
	fb->id = m->electrical.id_meas;			/* d轴电流 A */
	fb->iq = m->electrical.iq_meas;			/* q轴电流 A */
	fb->ia = m->electrical.ia;				/* A 相电流 A */
	fb->ib = m->electrical.ib;				/* B 相电流 A */
	fb->ic = m->electrical.ic;				/* C 相电流 A */
	fb->vbus = m->power.v_bus;				/* 母线电压 V */
	fb->ibus = m->power.i_bus;				/* 母线电流 A */
	fb->temp_fet = m->thermal.temp_fet;		/* 功率管温度 ℃ */
	fb->temp_motor = m->thermal.temp_motor; /* 电机温度 ℃ */
	fb->multiturn = m->motion.multiturn;	/* 多圈计数 */
	fb->single = m->motion.single_turn_rad; /* 单圈位置 rad */
	fb->fault_mask = m->fault.fault_mask;	/* 故障掩码 */
	fb->warn_mask = m->fault.warn_mask;		/* 警告掩码 */
	fb->top_fsm = (uint8_t)usr.fsm.motor_fsm[M1];
	fb->run_state = (uint8_t)m->run_mode;
	fb->ctrl_mode = (uint8_t)usr.fsm.motor_mode[M1];
	fb->enable = m->enable_motor ? 1 : 0;
	return JM_ERR_OK;
}

/* 读单个参数 */
static jm_err_e commun_uart_param_read(uint16_t param_id, uint8_t *value,
									   uint8_t *out_type, uint8_t *out_len)
{
	(void)param_id;
	(void)value;
	(void)out_type;
	(void)out_len;
	/* TODO: 按 param_id 查参数表并填充 value/out_type/out_len */
	return JM_ERR_BAD_PARAM_ID;
}

/* 写单个参数 */
static jm_err_e commun_uart_param_write(uint16_t param_id, const uint8_t *value, uint8_t len)
{
	(void)param_id;
	(void)value;
	(void)len;
	/* TODO: 按 param_id 写入参数表 */
	return JM_ERR_BAD_PARAM_ID;
}

/* ---------------- 同步遥测周期状态 ----------------
 * 约束沿用: packer 单全局 send_buffer + 无发送忙查询, 故每个上报节拍只发一帧。
 * 改为固定全量帧后, 不再按 mask 选组; 0xCB 仅用于调节上报周期(period_ms)。*/
#define COMMUN_TELEMETRY_TICK 5u /* 默认上报节拍: 每 5 个通信 tick(≈5ms) 发一帧 */

static uint16_t s_tlm_period_tick = COMMUN_TELEMETRY_TICK; /* 上报周期(单位: 通信 tick) */

/* 设置同步遥测(0xCB): 固定全量帧下 mask 忽略, 仅用 period_ms 调上报周期(0 表示不改)。 */
static jm_err_e commun_uart_set_telemetry(uint16_t mask, uint16_t period_ms)
{
	(void)mask; /* 固定全量帧: 不再按掩码选组 */
	if (period_ms != 0u)
	{
		/* 通信线程周期为 THREAD_DELAY_COMMUN ms, 换算成 tick 数, 至少 1 */
		uint16_t tick = (uint16_t)(period_ms / THREAD_DELAY_COMMUN);
		s_tlm_period_tick = (tick == 0u) ? 1u : tick;
	}
	return JM_ERR_OK;
}

static const jm_proto_ops_t commun_uart_ops = {
	.set_mode = commun_uart_set_mode,
	.get_feedback = commun_uart_get_feedback,
	.param_read = commun_uart_param_read,
	.param_write = commun_uart_param_write,
	.param_save = NULL,
	.param_reset = NULL,
	.get_dev_info = NULL,
	.get_dev_name = NULL,
	.set_telemetry = commun_uart_set_telemetry,
};

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

	if (commun_uart_get_feedback(&fb) != JM_ERR_OK)
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
	dev_commun_uart.set_ops(&dev_commun_uart, &commun_uart_ops);
	dev_commun_uart.start(&dev_commun_uart);
}

void ss_commun_process(void)
{
	static uint8_t telemetry_tick = 0; /* 遥测上报分频计数 */

	/* 取空闲突发数据喂协议栈, 自动完成命令分发与应答 */
	dev_commun_uart.poll(&dev_commun_uart);

	/* 周期主动上报遥测: 按订阅周期 s_tlm_period_tick 分频推一帧, 错峰避免 DMA 撞车 */
	// if (++telemetry_tick >= s_tlm_period_tick)
	// {
	// 	telemetry_tick = 0;
	// 	commun_uart_push_telemetry(&dev_commun_uart);
	// }
}

#endif /* USE_DEV_COMMUN_UART */
