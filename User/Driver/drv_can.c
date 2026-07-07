/**
 * @file        drv_can.c
 * @brief       CAN/FDCAN驱动实现，跨F4(CAN)/G4·H7(FDCAN)统一收发与回调
 *
 * @author      Dalin (dalinyy@163.com)
 * @version     1.0
 * @date        2026-06-16
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @par 修改日志:
 * | 日期       | 版本 | 作者   | 修改内容   |
 * |------------|------|--------|------------|
 * | 2026-06-16 | 1.0  | Dalin  | 初始创建   |
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 */
#include "drv_can.h"
#include "main.h"
#include <string.h>

#ifdef USE_CAN_DRIVER

/* 弱声明句柄(实际由CubeMX生成) */
#if defined(STM32F4)
__weak CAN_HandleTypeDef hcan1;
__weak CAN_HandleTypeDef hcan2;
#elif defined(STM32G4) || defined(STM32H7)
__weak FDCAN_HandleTypeDef hfdcan1;
__weak FDCAN_HandleTypeDef hfdcan2;
#endif

/* 接收回调表 */
static can_rx_callback_t user_can_rx_callback[DRV_CAN_NUMBER_MAX] = {NULL};

/* 句柄查找表：以canNumber_e为索引(DRV_CAN1=0)，O(1)定位HAL句柄 */
#if defined(STM32F4)
static CAN_HandleTypeDef *const s_can_map[DRV_CAN_NUMBER_MAX] = {
	[DRV_CAN1] = &hcan1,
	[DRV_CAN2] = &hcan2,
};
#elif defined(STM32G4) || defined(STM32H7)
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

#if defined(STM32G4) || defined(STM32H7)
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

#if defined(STM32F4)
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
	if (HAL_CAN_ActivateNotification(handle, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
		return DRV_ERROR;

#elif defined(STM32G4) || defined(STM32H7)
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
	if (HAL_FDCAN_ActivateNotification(handle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
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

#if defined(STM32F4)
	CAN_HandleTypeDef *handle = (CAN_HandleTypeDef *)h;
	CAN_TxHeaderTypeDef header = {0};
	uint32_t mailbox;

	if (msg->len > 8)
		return DRV_ERROR;

	header.IDE = msg->ide ? CAN_ID_EXT : CAN_ID_STD;
	if (msg->ide)
		header.ExtId = msg->id;
	else
		header.StdId = msg->id;
	header.RTR = msg->rtr ? CAN_RTR_REMOTE : CAN_RTR_DATA;
	header.DLC = msg->len;
	header.TransmitGlobalTime = DISABLE;

	if (HAL_CAN_AddTxMessage(handle, &header, msg->data, &mailbox) != HAL_OK)
		return DRV_ERROR;

#elif defined(STM32G4) || defined(STM32H7)
	FDCAN_HandleTypeDef *handle = (FDCAN_HandleTypeDef *)h;
	FDCAN_TxHeaderTypeDef header = {0};

	if (msg->len > 64)
		return DRV_ERROR;

	header.Identifier = msg->id;
	header.IdType = msg->ide ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
	header.TxFrameType = msg->rtr ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
	header.DataLength = fdcan_len_to_dlc(msg->len);
	header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	header.BitRateSwitch = FDCAN_BRS_OFF;
	header.FDFormat = FDCAN_CLASSIC_CAN;
	header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	header.MessageMarker = 0;

	if (HAL_FDCAN_AddMessageToTxFifoQ(handle, &header, msg->data) != HAL_OK)
		return DRV_ERROR;
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

#if defined(STM32F4)
	CAN_HandleTypeDef *handle = (CAN_HandleTypeDef *)h;
	CAN_RxHeaderTypeDef header = {0};

	if (HAL_CAN_GetRxMessage(handle, CAN_RX_FIFO0, &header, msg->data) != HAL_OK)
		return DRV_ERROR;

	msg->ide = (header.IDE == CAN_ID_EXT) ? 1u : 0u;
	msg->id = msg->ide ? header.ExtId : header.StdId;
	msg->rtr = (header.RTR == CAN_RTR_REMOTE) ? 1u : 0u;
	msg->len = (uint8_t)header.DLC;

#elif defined(STM32G4) || defined(STM32H7)
	FDCAN_HandleTypeDef *handle = (FDCAN_HandleTypeDef *)h;
	FDCAN_RxHeaderTypeDef header = {0};

	if (HAL_FDCAN_GetRxMessage(handle, FDCAN_RX_FIFO0, &header, msg->data) != HAL_OK)
		return DRV_ERROR;

	msg->ide = (header.IdType == FDCAN_EXTENDED_ID) ? 1u : 0u;
	msg->id = header.Identifier;
	msg->rtr = (header.RxFrameType == FDCAN_REMOTE_FRAME) ? 1u : 0u;
	msg->len = fdcan_dlc_to_len(header.DataLength);
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

/* 接收中断回调：读取报文并转发给用户回调 */
#if defined(STM32F4)
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	canNumber_e can_id;

	if (hcan->Instance == CAN1)
		can_id = DRV_CAN1;
	else if (hcan->Instance == CAN2)
		can_id = DRV_CAN2;
	else
		return;

	drvCanMsg_t msg = {0};
	if (drv_can_recv(can_id, &msg) != DRV_EOK)
		return;

	if (user_can_rx_callback[can_id])
		user_can_rx_callback[can_id](can_id, &msg);
}
#elif defined(STM32G4) || defined(STM32H7)
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	canNumber_e can_id;

	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0)
		return;

	if (hfdcan->Instance == FDCAN1)
		can_id = DRV_CAN1;
	else if (hfdcan->Instance == FDCAN2)
		can_id = DRV_CAN2;
	else
		return;

	drvCanMsg_t msg = {0};
	if (drv_can_recv(can_id, &msg) != DRV_EOK)
		return;

	if (user_can_rx_callback[can_id])
		user_can_rx_callback[can_id](can_id, &msg);

	/* 重新激活接收中断 */
	HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}
#endif

#endif /* USE_CAN_DRIVER */
