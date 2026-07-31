/**
 * @file        drv_can.c
 * @brief       CAN/FDCAN驱动实现，跨F4(CAN)/G4·H7(FDCAN)统一收发与回调
 *
 * @author      Dalin (dalinyy@163.com)
 * @version     1.1
 * @date        2026-06-16
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @par 修改日志:
 * | 日期       | 版本 | 作者   | 修改内容   |
 * |------------|------|--------|------------|
 * | 2026-06-16 | 1.0  | Dalin  | 初始创建   |
 * | 2026-07-27 | 1.1  | Dalin  | FIFO循环读 + Bus-Off恢复/错误回调 + TxFull检测
 *                              | FD模式收发 + 双过滤器 + 诊断统计 |
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 */
#include "drv_can.h"
#include "main.h"
#include "mcu_compat.h"
#include "drv_delay.h" /* drv_delay_us: TxFIFO 满时轮询等待 */
#include <string.h>
#include <stddef.h> /* offsetof */

#ifdef USE_CAN_DRIVER

/* 弱声明句柄(实际由CubeMX生成) */
#if defined(JM_PERIPH_CAN_CLASSIC)
__weak CAN_HandleTypeDef hcan1;
__weak CAN_HandleTypeDef hcan2;
#elif defined(JM_PERIPH_CAN_FD)
__weak FDCAN_HandleTypeDef hfdcan1;
__weak FDCAN_HandleTypeDef hfdcan2;
#endif

/* 接收回调表 + 错误回调表 + 诊断统计表 */
static can_rx_callback_t user_can_rx_callback[DRV_CAN_NUMBER_MAX] = {NULL};
static can_err_callback_t user_can_err_callback[DRV_CAN_NUMBER_MAX] = {NULL};
static drvCanDiag_t s_can_diag[DRV_CAN_NUMBER_MAX] = {0};

/* 句柄查找表：以canNumber_e为索引(DRV_CAN1=0)，O(1)定位HAL句柄 */
#if defined(JM_PERIPH_CAN_CLASSIC)
static CAN_HandleTypeDef *const s_can_map[DRV_CAN_NUMBER_MAX] = {
	[DRV_CAN1] = &hcan1,
	[DRV_CAN2] = &hcan2,
};
#elif defined(JM_PERIPH_CAN_FD)
static FDCAN_HandleTypeDef *const s_can_map[DRV_CAN_NUMBER_MAX] = {
	[DRV_CAN1] = &hfdcan1,
	[DRV_CAN2] = &hfdcan2,
};
#endif

static inline void *get_can_handle(canNumber_e can)
{
	if (can < DRV_CAN1 || can >= DRV_CAN_NUMBER_MAX)
		return NULL;

	return s_can_map[can];
}

/* 诊断计数原子自增(关中断保护, 适用于 ISR 和线程上下文)
 * 用字段偏移量代替 C++ 成员指针, 保持 C99 兼容 */
static inline void diag_inc(canNumber_e can, size_t field_offset)
{
	uint32_t *p;
	if (can < DRV_CAN1 || can >= DRV_CAN_NUMBER_MAX)
		return;
	p = (uint32_t *)((uint8_t *)&s_can_diag[can] + field_offset);
	__disable_irq();
	*p += 1u;
	__enable_irq();
}

/* 字段偏移量宏, 避免重复 offsetof 计算 */
#define DIAG_OFF(field)  offsetof(drvCanDiag_t, field)

#if defined(JM_PERIPH_CAN_CLASSIC)
/* 生成 bxCAN 32 位掩码过滤器。IDE/RTR 也参与比较, 只接受指定帧类型的数据帧。 */
static void classic_filter_fill(CAN_FilterTypeDef *fcfg, uint32_t bank,
                                uint32_t id, uint32_t mask, uint8_t ide)
{
	memset(fcfg, 0, sizeof(*fcfg));
	if (ide)
	{
		fcfg->FilterIdHigh = (uint16_t)(id >> 13);
		fcfg->FilterIdLow = (uint16_t)((id << 3) | CAN_ID_EXT | CAN_RTR_DATA);
		fcfg->FilterMaskIdHigh = (uint16_t)(mask >> 13);
		fcfg->FilterMaskIdLow = (uint16_t)((mask << 3) | CAN_ID_EXT | CAN_RTR_REMOTE);
	}
	else
	{
		fcfg->FilterIdHigh = (uint16_t)(id << 5);
		fcfg->FilterIdLow = CAN_ID_STD | CAN_RTR_DATA;
		fcfg->FilterMaskIdHigh = (uint16_t)(mask << 5);
		fcfg->FilterMaskIdLow = CAN_ID_EXT | CAN_RTR_REMOTE;
	}
	fcfg->FilterFIFOAssignment = CAN_FILTER_FIFO0;
	fcfg->FilterBank = bank;
	fcfg->FilterMode = CAN_FILTERMODE_IDMASK;
	fcfg->FilterScale = CAN_FILTERSCALE_32BIT;
	fcfg->FilterActivation = CAN_FILTER_ENABLE;
	fcfg->SlaveStartFilterBank = 14;
}
#endif

#if defined(JM_PERIPH_CAN_FD)
/* 数据长度(字节)转FDCAN DLC宏：经典CAN仅0~8，FD支持到64。
 * 本HAL中 FDCAN_DLC_BYTES_0~8 == 0~8，12/16/.../64 == 0x9~0xF */
static uint32_t fdcan_len_to_dlc(uint8_t len)
{
	if (len <= 8)
		return (uint32_t)len; /* FDCAN_DLC_BYTES_0~8 数值即字节数 */
	else if (len <= 12)
		return FDCAN_DLC_BYTES_12;
	else if (len <= 16)
		return FDCAN_DLC_BYTES_16;
	else if (len <= 20)
		return FDCAN_DLC_BYTES_20;
	else if (len <= 24)
		return FDCAN_DLC_BYTES_24;
	else if (len <= 32)
		return FDCAN_DLC_BYTES_32;
	else if (len <= 48)
		return FDCAN_DLC_BYTES_48;
	else
		return FDCAN_DLC_BYTES_64;
}

/* FDCAN DLC宏转数据长度(字节) */
static uint8_t fdcan_dlc_to_len(uint32_t dlc)
{
	switch (dlc)
	{
		case FDCAN_DLC_BYTES_12: return 12;
		case FDCAN_DLC_BYTES_16: return 16;
		case FDCAN_DLC_BYTES_20: return 20;
		case FDCAN_DLC_BYTES_24: return 24;
		case FDCAN_DLC_BYTES_32: return 32;
		case FDCAN_DLC_BYTES_48: return 48;
		case FDCAN_DLC_BYTES_64: return 64;
		default: return (uint8_t)dlc; /* 0~8字节，DLC数值即字节数 */
	}
}
#endif

/**
 * @brief       初始化CAN(配置过滤器、启动、使能接收中断)
 */
int drv_can_init(canNumber_e can, drvCanFilter_t *filter)
{
	void *h = get_can_handle(can);
	if (h == NULL)
		return DRV_ERROR;

#if defined(JM_PERIPH_CAN_CLASSIC)
	CAN_HandleTypeDef *handle = (CAN_HandleTypeDef *)h;
	CAN_FilterTypeDef fcfg = {0};

	/* 默认接收所有ID(掩码模式、掩码为0) */
	fcfg.FilterIdHigh = (uint16_t)((filter ? filter->id : 0u) >> 13);
	fcfg.FilterIdLow = (uint16_t)((filter ? filter->id : 0u) << 3);
	fcfg.FilterMaskIdHigh = (uint16_t)((filter ? filter->mask : 0u) >> 13);
	fcfg.FilterMaskIdLow = (uint16_t)((filter ? filter->mask : 0u) << 3);
	fcfg.FilterFIFOAssignment = (filter && filter->fifo) ? CAN_FILTER_FIFO1 : CAN_FILTER_FIFO0;
	fcfg.FilterBank = 0;
	fcfg.FilterMode = CAN_FILTERMODE_IDMASK;
	fcfg.FilterScale = CAN_FILTERSCALE_32BIT;
	fcfg.FilterActivation = CAN_FILTER_ENABLE;
	fcfg.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(handle, &fcfg) != HAL_OK)
		return DRV_ERROR;
	if (HAL_CAN_Start(handle) != HAL_OK)
		return DRV_ERROR;
	/* 使能错误中断(Bus-Off/错误被动/警告) */
	if (HAL_CAN_ActivateNotification(handle, CAN_IT_RX_FIFO0_MSG_PENDING |
	                                    CAN_IT_BUSOFF | CAN_IT_ERROR |
	                                    CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE) != HAL_OK)
		return DRV_ERROR;

#elif defined(JM_PERIPH_CAN_FD)
	FDCAN_HandleTypeDef *handle = (FDCAN_HandleTypeDef *)h;
	FDCAN_FilterTypeDef fcfg = {0};

	fcfg.IdType = (filter && filter->ide) ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
	fcfg.FilterIndex = 0;
	fcfg.FilterType = FDCAN_FILTER_MASK;
	fcfg.FilterConfig = (filter && filter->fifo) ? FDCAN_FILTER_TO_RXFIFO1 : FDCAN_FILTER_TO_RXFIFO0;
	fcfg.FilterID1 = filter ? filter->id : 0u;
	fcfg.FilterID2 = filter ? filter->mask : 0u;

	if (HAL_FDCAN_ConfigFilter(handle, &fcfg) != HAL_OK)
		return DRV_ERROR;
	if (HAL_FDCAN_Start(handle) != HAL_OK)
		return DRV_ERROR;
	/* 使能接收新消息 + 错误中断(Bus-Off/错误被动/警告)
	 * 注: STM32G4 HAL 无 FDCAN_IT_ERROR 宏, 错误中断由 BUS_OFF/ERROR_PASSIVE/ERROR_WARNING 分别使能 */
	if (HAL_FDCAN_ActivateNotification(handle,
	                                    FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
	                                    FDCAN_IT_BUS_OFF |
	                                    FDCAN_IT_ERROR_WARNING | FDCAN_IT_ERROR_PASSIVE,
	                                    0) != HAL_OK)
		return DRV_ERROR;
#endif

	return DRV_EOK;
}

/**
 * @brief       初始化CAN并配置双掩码过滤器(单播+广播)
 */
int drv_can_init_dual_filter(canNumber_e can, drvCanDualFilter_t *dual_filter)
{
	void *h = get_can_handle(can);
	if (h == NULL || dual_filter == NULL)
		return DRV_ERROR;

#if defined(JM_PERIPH_CAN_CLASSIC)
	CAN_HandleTypeDef *handle = (CAN_HandleTypeDef *)h;
	CAN_FilterTypeDef fcfg = {0};
	uint32_t first_bank = (can == DRV_CAN2) ? 14u : 0u;

	classic_filter_fill(&fcfg, first_bank, dual_filter->unicast_id,
	                    dual_filter->mask, dual_filter->ide);
	if (HAL_CAN_ConfigFilter(handle, &fcfg) != HAL_OK)
		return DRV_ERROR;
	classic_filter_fill(&fcfg, first_bank + 1u, dual_filter->broadcast_id,
	                    dual_filter->mask, dual_filter->ide);
	if (HAL_CAN_ConfigFilter(handle, &fcfg) != HAL_OK)
		return DRV_ERROR;
	if (HAL_CAN_Start(handle) != HAL_OK)
		return DRV_ERROR;
	if (HAL_CAN_ActivateNotification(handle, CAN_IT_RX_FIFO0_MSG_PENDING |
	                                    CAN_IT_BUSOFF | CAN_IT_ERROR |
	                                    CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE) != HAL_OK)
		return DRV_ERROR;

#elif defined(JM_PERIPH_CAN_FD)
	FDCAN_HandleTypeDef *handle = (FDCAN_HandleTypeDef *)h;
	FDCAN_FilterTypeDef fcfg = {0};

	/* 未命中过滤器的帧和所有远程帧必须拒绝, 否则全局默认策略仍会收帧。 */
	if (HAL_FDCAN_ConfigGlobalFilter(handle,
	                                 FDCAN_REJECT, FDCAN_REJECT,
	                                 FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
		return DRV_ERROR;

	/* 过滤器0: 单播节点ID */
	fcfg.IdType = dual_filter->ide ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
	fcfg.FilterIndex = 0;
	fcfg.FilterType = FDCAN_FILTER_MASK;
	fcfg.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	fcfg.FilterID1 = dual_filter->unicast_id;
	fcfg.FilterID2 = dual_filter->mask;
	if (HAL_FDCAN_ConfigFilter(handle, &fcfg) != HAL_OK)
		return DRV_ERROR;

	/* 过滤器1: 广播节点ID */
	fcfg.FilterIndex = 1;
	fcfg.FilterID1 = dual_filter->broadcast_id;
	fcfg.FilterID2 = dual_filter->mask;
	if (HAL_FDCAN_ConfigFilter(handle, &fcfg) != HAL_OK)
		return DRV_ERROR;

	if (HAL_FDCAN_Start(handle) != HAL_OK)
		return DRV_ERROR;
	if (HAL_FDCAN_ActivateNotification(handle,
	                                    FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
	                                    FDCAN_IT_BUS_OFF |
	                                    FDCAN_IT_ERROR_WARNING | FDCAN_IT_ERROR_PASSIVE,
	                                    0) != HAL_OK)
		return DRV_ERROR;
#endif

	return DRV_EOK;
}

/**
 * @brief       发送一帧CAN报文
 */
int drv_can_send(canNumber_e can, drvCanMsg_t *msg)
{
	void *h = get_can_handle(can);
	if (h == NULL || msg == NULL)
		return DRV_ERROR;

#if defined(JM_PERIPH_CAN_CLASSIC)
	CAN_HandleTypeDef *handle = (CAN_HandleTypeDef *)h;
	CAN_TxHeaderTypeDef header = {0};
	uint32_t mailbox;

	if (msg->len > 8)
		return DRV_ERROR;

	/* TxFull 等待: 3 个邮箱全满时轮询等待(最多 1ms), 避免多帧连续发送丢帧 */
	{
		uint32_t wait = 0u;
		while (HAL_CAN_GetTxMailboxesFreeLevel(handle) == 0u)
		{
			if (++wait > 100u) /* 100 * 10us = 1ms 超时 */
			{
				diag_inc(can, DIAG_OFF(tx_fail_count));
				return DRV_ERROR;
			}
			drv_delay_us(10);
		}
	}

	header.IDE = msg->ide ? CAN_ID_EXT : CAN_ID_STD;
	if (msg->ide)
		header.ExtId = msg->id;
	else
		header.StdId = msg->id;
	header.RTR = msg->rtr ? CAN_RTR_REMOTE : CAN_RTR_DATA;
	header.DLC = msg->len;
	header.TransmitGlobalTime = DISABLE;

	if (HAL_CAN_AddTxMessage(handle, &header, msg->data, &mailbox) != HAL_OK)
	{
		diag_inc(can, DIAG_OFF(tx_fail_count));
		return DRV_ERROR;
	}
	diag_inc(can, DIAG_OFF(tx_frame_count));

#elif defined(JM_PERIPH_CAN_FD)
	FDCAN_HandleTypeDef *handle = (FDCAN_HandleTypeDef *)h;
	FDCAN_TxHeaderTypeDef header = {0};
	uint8_t use_fd = msg->is_fd;

	/* USE_CAN_FD_MODE 关闭时强制经典帧; 开启时按 msg->is_fd 选择 */
#if !defined(USE_CAN_FD_MODE) || (USE_CAN_FD_MODE == 0)
	use_fd = 0u;
#endif
	if (use_fd)
	{
		if (msg->len > 64)
			return DRV_ERROR;
	}
	else
	{
		if (msg->len > 8)
			return DRV_ERROR;
	}

	/* TxFull 等待: TxFIFO 满时轮询等待(最多 1ms), 避免多帧连续发送丢帧 */
	{
		uint32_t wait = 0u;
		while (HAL_FDCAN_GetTxFifoFreeLevel(handle) == 0u)
		{
			if (++wait > 100u) /* 100 * 10us = 1ms 超时 */
			{
				diag_inc(can, DIAG_OFF(tx_fail_count));
				return DRV_ERROR;
			}
			drv_delay_us(10);
		}
	}

	header.Identifier = msg->id;
	header.IdType = msg->ide ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
	header.TxFrameType = msg->rtr ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
	header.DataLength = fdcan_len_to_dlc(msg->len);
	header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	header.FDFormat = use_fd ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN;
	/* BRS 仅在 FD 帧时启用(数据段 8Mbps), 经典帧关闭 */
	header.BitRateSwitch = use_fd ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
	header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	header.MessageMarker = 0;

	if (HAL_FDCAN_AddMessageToTxFifoQ(handle, &header, msg->data) != HAL_OK)
	{
		diag_inc(can, DIAG_OFF(tx_fail_count));
		return DRV_ERROR;
	}
	diag_inc(can, DIAG_OFF(tx_frame_count));
#endif

	return DRV_EOK;
}

/**
 * @brief       接收一帧CAN报文(从FIFO0读取)
 */
int drv_can_recv(canNumber_e can, drvCanMsg_t *msg)
{
	void *h = get_can_handle(can);
	if (h == NULL || msg == NULL)
		return DRV_ERROR;

#if defined(JM_PERIPH_CAN_CLASSIC)
	CAN_HandleTypeDef *handle = (CAN_HandleTypeDef *)h;
	CAN_RxHeaderTypeDef header = {0};

	if (HAL_CAN_GetRxMessage(handle, CAN_RX_FIFO0, &header, msg->data) != HAL_OK)
		return DRV_ERROR;

	msg->ide = (header.IDE == CAN_ID_EXT) ? 1u : 0u;
	msg->id = msg->ide ? header.ExtId : header.StdId;
	msg->rtr = (header.RTR == CAN_RTR_REMOTE) ? 1u : 0u;
	msg->len = (uint8_t)header.DLC;
	msg->is_fd = 0u; /* 经典CAN 无 FD 帧 */

#elif defined(JM_PERIPH_CAN_FD)
	FDCAN_HandleTypeDef *handle = (FDCAN_HandleTypeDef *)h;
	FDCAN_RxHeaderTypeDef header = {0};

	if (HAL_FDCAN_GetRxMessage(handle, FDCAN_RX_FIFO0, &header, msg->data) != HAL_OK)
		return DRV_ERROR;

	msg->ide = (header.IdType == FDCAN_EXTENDED_ID) ? 1u : 0u;
	msg->id = header.Identifier;
	msg->rtr = (header.RxFrameType == FDCAN_REMOTE_FRAME) ? 1u : 0u;
	msg->len = fdcan_dlc_to_len(header.DataLength);
	msg->is_fd = (header.FDFormat == FDCAN_FD_CAN) ? 1u : 0u; /* 标识 FD 帧 */
#endif

	return DRV_EOK;
}

/**
 * @brief       注册接收完成回调
 */
int drv_can_register_rx_callback(canNumber_e can, can_rx_callback_t callback)
{
	if (can < DRV_CAN1 || can >= DRV_CAN_NUMBER_MAX)
		return DRV_ERROR;

	user_can_rx_callback[can] = callback;
	return DRV_EOK;
}

/**
 * @brief       注册错误回调
 */
int drv_can_register_err_callback(canNumber_e can, can_err_callback_t callback)
{
	if (can < DRV_CAN1 || can >= DRV_CAN_NUMBER_MAX)
		return DRV_ERROR;

	user_can_err_callback[can] = callback;
	return DRV_EOK;
}

/**
 * @brief       获取诊断统计
 */
int drv_can_get_diag(canNumber_e can, drvCanDiag_t *diag)
{
	if (can < DRV_CAN1 || can >= DRV_CAN_NUMBER_MAX || diag == NULL)
		return DRV_ERROR;

	__disable_irq();
	*diag = s_can_diag[can];
	__enable_irq();
	return DRV_EOK;
}

/* ==================== 接收中断: FIFO 循环读 + 错误回调 ==================== */
#if defined(JM_PERIPH_CAN_CLASSIC)
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	canNumber_e can_id;

	if (hcan->Instance == CAN1)
		can_id = DRV_CAN1;
	else if (hcan->Instance == CAN2)
		can_id = DRV_CAN2;
	else
		return;

	/* 循环读直到 FIFO0 空, 避免高负载下 FIFO 溢出丢帧 */
	while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0u)
	{
		drvCanMsg_t msg = {0};
		if (drv_can_recv(can_id, &msg) != DRV_EOK)
		{
			diag_inc(can_id, DIAG_OFF(rx_overflow_count));
			break;
		}
		diag_inc(can_id, DIAG_OFF(rx_frame_count));
		if (user_can_rx_callback[can_id])
			user_can_rx_callback[can_id](can_id, &msg);
	}
}

/* 经典CAN错误回调: Bus-Off 自动恢复 + 错误统计 */
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	canNumber_e can_id;
	if (hcan->Instance == CAN1)
		can_id = DRV_CAN1;
	else if (hcan->Instance == CAN2)
		can_id = DRV_CAN2;
	else
		return;

	uint8_t err_type = 3u; /* 默认其他 */
	if ((hcan->ErrorCode & HAL_CAN_ERROR_BOF) != 0u)
	{
		err_type = 0u; /* Bus-Off */
		diag_inc(can_id, DIAG_OFF(busoff_count));
		/* Bus-Off 自动恢复: 停止->重启->重新使能中断
		 * 注: ISR 中不可调 HAL_Delay; CAN 控制器硬件在 Start 后自动等待 128 个空闲位才退出 Bus-Off */
		HAL_CAN_Stop(hcan);
		HAL_CAN_ResetError(hcan);
		HAL_CAN_Start(hcan);
		HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING |
		                                    CAN_IT_BUSOFF | CAN_IT_ERROR |
		                                    CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE);
	}
	else if ((hcan->ErrorCode & HAL_CAN_ERROR_EPV) != 0u)
	{
		err_type = 1u; /* 错误被动 */
		diag_inc(can_id, DIAG_OFF(error_passive_count));
	}
	else if ((hcan->ErrorCode & HAL_CAN_ERROR_EWG) != 0u)
	{
		err_type = 2u; /* 警告 */
	}
	HAL_CAN_ResetError(hcan);

	if (user_can_err_callback[can_id])
		user_can_err_callback[can_id](can_id, err_type);
}

#elif defined(JM_PERIPH_CAN_FD)
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	canNumber_e can_id;

	if ((RxFifo0ITs & (FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_FULL | FDCAN_IT_RX_FIFO0_MESSAGE_LOST)) == 0u)
		return;

	if (hfdcan->Instance == FDCAN1)
		can_id = DRV_CAN1;
	else if (hfdcan->Instance == FDCAN2)
		can_id = DRV_CAN2;
	else
		return;

	/* 循环读直到 FIFO0 空, 避免高负载下 FIFO 溢出丢帧 */
	while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0u)
	{
		drvCanMsg_t msg = {0};
		if (drv_can_recv(can_id, &msg) != DRV_EOK)
		{
			diag_inc(can_id, DIAG_OFF(rx_overflow_count));
			break;
		}
		diag_inc(can_id, DIAG_OFF(rx_frame_count));
		if (user_can_rx_callback[can_id])
			user_can_rx_callback[can_id](can_id, &msg);
	}

	/* FIFO 溢出计数 */
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != 0u)
	{
		diag_inc(can_id, DIAG_OFF(rx_overflow_count));
	}

	/* 重新激活接收中断 (HAL 库要求) */
	HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

/* FDCAN 错误回调: Bus-Off 自动恢复 + 错误统计
 * HAL 库回调签名 HAL_FDCAN_ErrorCallback(hfdcan), 通过 HAL_FDCAN_GetProtocolStatus
 * 查询协议状态(BusOff/ErrorPassive/Warning) 判断错误类型。
 * 注: STM32G4 HAL 无 HAL_FDCAN_GetActivatedITs/GetLastError 函数,
 *     用 GetProtocolStatus 替代, 用 GetError 读取 HAL 层错误码。 */
void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
	canNumber_e can_id;
	FDCAN_ProtocolStatusTypeDef pstatus = {0};

	if (hfdcan->Instance == FDCAN1)
		can_id = DRV_CAN1;
	else if (hfdcan->Instance == FDCAN2)
		can_id = DRV_CAN2;
	else
		return;

	/* 读取协议状态(BusOff/ErrorPassive/Warning) */
	HAL_FDCAN_GetProtocolStatus(hfdcan, &pstatus);
	uint8_t err_type = 3u;
	uint32_t err = HAL_FDCAN_GetError(hfdcan);

	if (pstatus.BusOff != 0u)
	{
		err_type = 0u; /* Bus-Off */
		diag_inc(can_id, DIAG_OFF(busoff_count));
		/* Bus-Off 自动恢复: 停止->重启->重新使能中断 (CAN 规范要求 128 个空闲位) */
		HAL_FDCAN_Stop(hfdcan);
		HAL_FDCAN_Start(hfdcan);
		HAL_FDCAN_ActivateNotification(hfdcan,
		                                FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
		                                FDCAN_IT_BUS_OFF |
		                                FDCAN_IT_ERROR_WARNING | FDCAN_IT_ERROR_PASSIVE,
		                                0);
	}
	else if (pstatus.ErrorPassive != 0u)
	{
		err_type = 1u; /* 错误被动 */
		diag_inc(can_id, DIAG_OFF(error_passive_count));
	}
	else
	{
		err_type = 2u; /* 警告(协议状态中 Warning 由 ErrorWarning 字段表示, 简化处理) */
	}
	(void)err;

	if (user_can_err_callback[can_id])
		user_can_err_callback[can_id](can_id, err_type);
}
#endif

#endif /* USE_CAN_DRIVER */
