/**
 * @file        dev_commun_can.c
 * @brief       关节电机 CAN/CAN-FD 通信设备(对接 drv_can + jm_proto_can)
 *
 * @author      Dalin (dalinyy@163.com)
 * @version     1.0
 * @date        2026-07-27
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @par 修改日志:
 * | 日期       | 版本 | 作者   | 修改内容   |
 * |------------|------|--------|------------|
 * | 2026-07-27 | 1.0  | Dalin  | 初始创建 (CAN 设备粘合层) |
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 * @note        分层: 本设备(粘合层) → jm_proto_can(编解码) → jm_proto(分发)
 *                                  ↘ drv_can(FDCAN/经典 CAN 收发)
 *              jm_proto_can 的 tx 回调签名无上下文参数, 故用模块级活动设备指针
 *              s_active 路由到正确 CAN 通道(典型单路上位机链路, 与 UART 模式一致)。
 * @note        通信中断降级: poll() 内基于 HAL_GetTick() 检查 last_rx_tick,
 *              超 JM_CAN_LOSS_TIMEOUT_MS 触发应用层 IDLE 停机。
 *              降级动作不在本层执行(本层无电机控制权), 由应用层 check_loss() 返回 1
 *              后自行切 IDLE, 保持分层清晰。
 */
#include "dev_commun_can.h"
#if defined(USE_DEV_COMMUN_CAN)

#include "assert_report.h"
#include "main.h" /* HAL_GetTick (降级计时) */
#if defined(USE_DEV_FLASH)
#include "motor_info_storage.h"
#endif
#include <string.h>

dev_commun_can_t dev_commun_can;

/* 当前活动设备(支撑无上下文的 tx/rx 回调; 每次 start/poll/report/on_rx_msg 前刷新) */
static dev_commun_can_t *s_active = NULL;

/* 前向声明: trampoline 用于 drv_can 回调注册(无上下文参数, 通过 s_active 路由) */
static void s_rx_trampoline(canNumber_e can, drvCanMsg_t *msg);
static void s_err_trampoline(canNumber_e can, uint8_t error_type);

/* ==================================================================== */
/*  协议栈发送回调: jm_proto_can 组好整帧后, 经 drv_can 推出              */
/* ==================================================================== */
static void jm_can_tx(const jm_can_frame_t *frame)
{
	drvCanMsg_t msg;
	uint8_t use_fd;
	uint8_t max_len;

	if (s_active == NULL || frame == NULL)
	{
		return;
	}

	/* 运行期 FD 模式优先: 由 0xF3 SET_FD_MODE 命令设置(与 jm_proto_can_t.use_fd_runtime 同步);
	 * 板级 USE_CAN_FD_MODE=0 时强制经典(无 FD 硬件能力) */
	use_fd = s_active->use_fd_runtime;
#if !defined(USE_CAN_FD_MODE) || (USE_CAN_FD_MODE == 0)
	use_fd = 0u;
#endif
	max_len = use_fd ? 64u : 8u;
	if (frame->len > max_len)
	{
		s_active->tx_fail_count++;
		s_active->last_error = -5;
		return;
	}

	memset(&msg, 0, sizeof(msg));
	msg.id = frame->id;
	msg.ide = s_active->ide;
	msg.rtr = 0u;
	msg.len = frame->len;
	msg.is_fd = use_fd; /* drv_can 按 is_fd 决定 FDFormat/BRS */
	memcpy(msg.data, frame->data, frame->len);

	if (drv_can_send(s_active->can, &msg) == DRV_EOK)
	{
		s_active->tx_count++;
	}
	else
	{
		s_active->tx_fail_count++;
		s_active->last_error = -4;
	}
}

/* ==================================================================== */
/*  CAN 接收回调 trampoline: drv_can 回调签名是 void(can, drvCanMsg_t*)  */
/*  无上下文, 通过 s_active 路由到活动设备实例                             */
/* ==================================================================== */
static void s_rx_trampoline(canNumber_e can, drvCanMsg_t *msg)
{
	if (s_active != NULL)
	{
		s_active->on_rx_msg(s_active, can, msg);
	}
}

static void s_err_trampoline(canNumber_e can, uint8_t error_type)
{
	if (s_active != NULL)
	{
		s_active->on_err(s_active, can, error_type);
	}
}

/* ==================================================================== */
/*  设备接口                                                            */
/* ==================================================================== */

/* 注入业务回调数据源(电机控制/参数读写); 须在 start 前调用 */
static void dev_commun_can_set_ops(struct dev_commun_can *pobj, const jm_proto_ops_t *ops)
{
	assert_report(pobj != NULL);
	pobj->ops = ops;
}

/* 启动 CAN(配置过滤器、注册回调、使能中断); 幂等: 重复调用只生效一次 */
static int dev_commun_can_start(struct dev_commun_can *pobj)
{
	drvCanDualFilter_t filter;

	assert_report(pobj != NULL);
	pobj->start_count++;
	if (pobj->started)
	{
		return DEV_EOK;
	}

	/* 组装 CAN 协议: 注入业务回调与发送函数 */
	s_active = pobj;
	if (jm_proto_can_init(&pobj->jm, pobj->ops, pobj->motor_id, jm_can_tx) != 0)
	{
		pobj->start_fail_count++;
		pobj->last_error = -1;
		return DEV_ERROR;
	}

	/* 仲裁ID低8位是节点地址。CMD和多帧标志不参与比较。 */
	filter.unicast_id = pobj->motor_id;
	filter.broadcast_id = JM_CAN_BROADCAST_ID;
	filter.mask = 0xFFu;
	filter.ide = pobj->ide;
	if (drv_can_init_dual_filter(pobj->can, &filter) != DRV_EOK)
	{
		pobj->start_fail_count++;
		pobj->last_error = -2;
		return DEV_ERROR;
	}

	/* 注册接收/错误回调(trampoline 路由到 s_active) */
	if (drv_can_register_rx_callback(pobj->can, s_rx_trampoline) != DRV_EOK)
	{
		pobj->start_fail_count++;
		pobj->last_error = -3;
		return DEV_ERROR;
	}
	if (drv_can_register_err_callback(pobj->can, s_err_trampoline) != DRV_EOK)
	{
		pobj->start_fail_count++;
		pobj->last_error = -3;
		return DEV_ERROR;
	}

	pobj->started = 1;
	pobj->last_error = DEV_EOK;
	pobj->last_rx_tick = HAL_GetTick(); /* 初始化为启动时刻, 避免启动即超时 */
	return DEV_EOK;
}

/* CAN RX ISR: validate and enqueue only; protocol dispatch runs in poll(). */
static void dev_commun_can_on_rx_msg(struct dev_commun_can *pobj, canNumber_e can, drvCanMsg_t *msg)
{
	jm_can_frame_t frame;
	uint8_t use_fd;
	uint8_t max_len;
	uint8_t head;
	uint8_t next;
	uint8_t dst;
	uint32_t now_tick;

	assert_report(pobj != NULL);
	(void)can; /* 单实例时 can 与 pobj->can 一致, 多实例时由 s_active 路由 */
	pobj->rx_irq_count++;

	/* 长度守卫: 经典模式≤8B, FD模式≤64B; 超长帧丢弃 */
	use_fd = pobj->use_fd_runtime;
#if !defined(USE_CAN_FD_MODE) || (USE_CAN_FD_MODE == 0)
	use_fd = 0u;
#endif
	max_len = use_fd ? 64u : 8u;
	if (msg->len > max_len)
	{
		pobj->last_error = -6;
		return;
	}

	/* The hardware filter currently accepts the whole protocol ID range. Ignore
	 * traffic for other motors before it consumes queue space or refreshes this
	 * motor's communication-loss watchdog. */
	dst = JM_CAN_GET_MOTOR_ID(msg->id);
	if (dst != pobj->motor_id && dst != JM_CAN_BROADCAST_ID)
	{
		return;
	}
	if (dst == JM_CAN_BROADCAST_ID &&
	    !jm_proto_can_broadcast_allowed(JM_CAN_GET_CMD(msg->id)))
	{
		return;
	}

	/* Copy the validated frame into the fixed-size ISR-to-thread queue. */
	frame.id = msg->id;
	frame.len = msg->len;
	frame.is_fd = msg->is_fd;
	memset(frame.data, 0, sizeof(frame.data));
	if (msg->len > 0u)
	{
		memcpy(frame.data, msg->data, msg->len);
	}

	/* 刷新最近接收 tick (ISR 上下文, HAL_GetTick 是原子读); 同时作为多帧重组超时基准 */
	now_tick = HAL_GetTick();
	pobj->last_rx_tick = now_tick;

	/* 锁定当前实例, 保证协议层应答 tx 回调指向正确 CAN */
	head = pobj->rx_queue_head;
	next = (uint8_t)(head + 1u);
	if (next >= DEV_COMMUN_CAN_RX_QUEUE_SIZE)
		next = 0u;
	if (next == pobj->rx_queue_tail)
	{
		pobj->rx_queue_overflow_count++;
		pobj->last_error = -7;
		return;
	}
	pobj->rx_queue[head].frame = frame;
	pobj->rx_queue[head].tick = now_tick;
	__DMB();
	pobj->rx_queue_head = next;
}

/* CAN 错误 ISR: drv_can 回调经 trampoline 进入, 仅记录统计, Bus-Off 恢复由 drv_can 内部处理 */
static void dev_commun_can_on_err(struct dev_commun_can *pobj, canNumber_e can, uint8_t error_type)
{
	assert_report(pobj != NULL);
	(void)can;
	(void)error_type;
	pobj->err_irq_count++;
}

/* 主循环/线程周期调用: 诊断刷新 + 降级超时检查(由应用调用 check_loss) */
static void dev_commun_can_poll(struct dev_commun_can *pobj)
{
	dev_commun_can_rx_item_t item;
	uint8_t tail;
	uint8_t next;

	assert_report(pobj != NULL);
	pobj->poll_count++;
	if (!pobj->started)
	{
		pobj->poll_not_started_count++;
		return;
	}

	/* 刷新 drv_can 诊断快照(供应用读取总线负载/通信质量) */
	s_active = pobj;
	while (pobj->rx_queue_tail != pobj->rx_queue_head)
	{
		tail = pobj->rx_queue_tail;
		item = pobj->rx_queue[tail];
		next = (uint8_t)(tail + 1u);
		if (next >= DEV_COMMUN_CAN_RX_QUEUE_SIZE)
			next = 0u;
		__DMB();
		pobj->rx_queue_tail = next;
		jm_proto_can_feed(&pobj->jm, &item.frame, item.tick);
	}
	(void)drv_can_get_diag(pobj->can, &pobj->diag);
}

/* 主动上报一帧(电机->上位机方向, 如周期反馈), 无需上位机轮询 */
static void dev_commun_can_report(struct dev_commun_can *pobj, uint8_t cmd,
                                  const uint8_t *body, uint16_t len)
{
	assert_report(pobj != NULL);
	if (!pobj->started)
	{
		return;
	}
	s_active = pobj;
	jm_proto_can_send(&pobj->jm, cmd, body, len);
}

/* 检查通信中断降级: 返回 1=已超时需 IDLE, 0=正常
 * 应用在主循环中调用, 返回 1 时自行切 IDLE 停机 */
static int dev_commun_can_check_loss(struct dev_commun_can *pobj, uint32_t now_tick)
{
	uint32_t elapsed;

	assert_report(pobj != NULL);
	if (!pobj->started)
	{
		return 0;
	}

#if (JM_CAN_LOSS_ACTION == 2) /* TIMER 模式 */
	elapsed = now_tick - pobj->last_rx_tick; /* tick 32 位回绕自然处理 */
	if (elapsed >= (uint32_t)JM_CAN_LOSS_TIMEOUT_MS)
	{
		pobj->loss_timeout_count++;
		/* 重置 last_rx_tick 避免重复触发, 应用切 IDLE 后由 enable=0 自然停止 */
		pobj->last_rx_tick = now_tick;
		return 1;
	}
#else
	(void)elapsed;
#endif
	return 0;
}

/* ==================================================================== */
/*  初始化                                                               */
/* ==================================================================== */
void dev_commun_can_init(dev_commun_can_t *pobj, jm_can_comm_id_e id)
{
	const dev_commun_can_config_t *cfg;

	assert_report(pobj != NULL);
	assert_report(id < JM_CAN_COMM_ID_MAX);
	memset(pobj, 0, sizeof(dev_commun_can_t));

	cfg = &commun_can_list[id];
	pobj->can = cfg->can;
	pobj->motor_id = cfg->motor_id;
#if defined(USE_DEV_FLASH)
	{
		motor_info_t *info = motor_info_storage_get();
		uint32_t stored_id = (info != NULL) ? motor_info_get_can_id(info) : 0u;
		if (stored_id >= 1u && stored_id <= 127u)
		{
			pobj->motor_id = (uint8_t)stored_id;
		}
	}
#endif
	pobj->ide = cfg->ide;
	pobj->use_fd = cfg->use_fd;
	pobj->use_fd_runtime = 0u; /* 上电默认经典模式(兼容所有上位机硬件), 由 0xF3 命令切换 */
	pobj->ops = NULL;          /* 由应用 set_ops 注入 */

	pobj->init_count = 1;

	/* public */
	pobj->set_ops = dev_commun_can_set_ops;
	pobj->start = dev_commun_can_start;
	pobj->on_rx_msg = dev_commun_can_on_rx_msg;
	pobj->on_err = dev_commun_can_on_err;
	pobj->poll = dev_commun_can_poll;
	pobj->report = dev_commun_can_report;
	pobj->check_loss = dev_commun_can_check_loss;
}

#endif /* USE_DEV_COMMUN_CAN */
