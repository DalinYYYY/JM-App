/**
 * @file        drv_can.h
 * @brief       CAN/FDCAN驱动接口，封装收发与接收回调，跨F4(CAN)/G4·H7(FDCAN)统一
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
 * @note        CAN初始化由CubeMX的MX_xxCAN_Init完成，此层封装运行期收发
 * @note        对外接口不暴露HAL类型，报文用统一结构drvCanMsg_t；
 *              F4走CAN、G4/H7走FDCAN，差异在drv_can.c内屏蔽
 */
#ifndef __DRV_CAN_H
#define __DRV_CAN_H

#include "drv_config.h"
#include <stdint.h>

#ifdef USE_CAN_DRIVER

/**
 * @brief CAN编号
 * @param  DRV_CAN_INIT          : 无效占位(-1)
 * @param  DRV_CAN1              : CAN1/FDCAN1
 * @param  DRV_CAN2              : CAN2/FDCAN2
 * @param  DRV_CAN_NUMBER_MAX    : CAN数量边界
 */
typedef enum
{
	DRV_CAN_INIT = -1,
	DRV_CAN1 = 0,
	DRV_CAN2,
	DRV_CAN_NUMBER_MAX
} canNumber_e;

/**
 * @brief 统一CAN报文(屏蔽CAN/FDCAN报文头差异)
 * @param  id                    : 报文ID(标准11位或扩展29位)
 * @param  ide                   : 0标准帧，1扩展帧
 * @param  rtr                   : 0数据帧，1远程帧
 * @param  len                   : 数据长度(经典CAN 0~8，FD 0~64)
 * @param  data                  : 数据缓冲区
 */
typedef struct
{
	uint32_t id;
	uint8_t ide;
	uint8_t rtr;
	uint8_t len;
	uint8_t data[64];
} drvCanMsg_t;

/**
 * @brief 简化CAN过滤器配置(屏蔽CAN/FDCAN过滤器差异)
 * @param  id                    : 验收ID
 * @param  mask                  : 验收掩码(为0时接收所有ID)
 * @param  ide                   : 0过滤标准帧，1过滤扩展帧
 * @param  fifo                  : 接收FIFO编号(0或1)
 */
typedef struct
{
	uint32_t id;
	uint32_t mask;
	uint8_t ide;
	uint8_t fifo;
} drvCanFilter_t;

/**
 * @brief CAN接收回调函数类型
 * @param  can                   : CAN编号
 * @param  msg                   : 接收到的报文
 */
typedef void (*can_rx_callback_t)(canNumber_e can, drvCanMsg_t *msg);

/**
 * @brief       初始化CAN(配置过滤器、启动、使能接收中断)
 * @param        can               : CAN编号
 * @param        filter            : 过滤器配置，NULL时接收所有ID
 * @return       : DRV_EOK成功，DRV_ERROR失败
 */
int drv_can_init(canNumber_e can, drvCanFilter_t *filter);

/**
 * @brief       发送一帧CAN报文
 * @param        can               : CAN编号
 * @param        msg               : 待发送报文
 * @return       : DRV_EOK成功，DRV_ERROR失败
 */
int drv_can_send(canNumber_e can, drvCanMsg_t *msg);

/**
 * @brief       接收一帧CAN报文(从FIFO0读取)
 * @param        can               : CAN编号
 * @param        msg               : 输出报文
 * @return       : DRV_EOK成功，DRV_ERROR失败
 */
int drv_can_recv(canNumber_e can, drvCanMsg_t *msg);

/**
 * @brief       注册接收完成回调(在接收中断中被调用)
 * @param        can               : CAN编号
 * @param        callback          : 回调函数
 * @return       : DRV_EOK成功，DRV_ERROR失败
 */
int drv_can_register_rx_callback(canNumber_e can, can_rx_callback_t callback);

#endif /* USE_CAN_DRIVER */
#endif /* __DRV_CAN_H */
