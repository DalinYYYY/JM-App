/**
 * @file        drv_can.h
 * @brief       CAN/FDCAN驱动接口，封装收发与接收回调，跨F4(CAN)/G4·H7(FDCAN)统一
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
 * | 2026-07-27 | 1.1  | Dalin  | FIFO循环读+错误回调+TxFull+FD模式+双过滤器+诊断字段 |
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 *              初始化由 CubeMX 完成，本层封装运行期收发，不暴露 HAL 类型。
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
 * @param  is_fd                 : 0=经典帧, 1=FD帧(决定FDFormat/BRS, 仅G4/H7有效)
 * @param  data                  : 数据缓冲区
 */
typedef struct
{
	uint32_t id;
	uint8_t ide;
	uint8_t rtr;
	uint8_t len;
	uint8_t is_fd; /* FD 帧标识(由 drv_can_send/recv 根据 USE_CAN_FD_MODE 和长度自动设置) */
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
 * @brief 双过滤器配置(单播+广播同时接收)
 *   过滤器0: 按 mask 匹配本机单播地址
 *   过滤器1: 按 mask 匹配广播地址
 *   两个过滤器都路由到 FIFO0, 上层无需区分来源。
 * @param  unicast_id            : 单播验收ID
 * @param  broadcast_id          : 广播验收ID
 * @param  mask                  : 验收掩码(协议地址过滤使用0xFF, 仅比较低8位节点ID)
 * @param  ide                   : 0标准帧, 1扩展帧
 */
typedef struct
{
	uint32_t unicast_id;
	uint32_t broadcast_id;
	uint32_t mask;
	uint8_t ide;
} drvCanDualFilter_t;

/**
 * @brief CAN接收回调函数类型
 * @param  can                   : CAN编号
 * @param  msg                   : 接收到的报文
 */
typedef void (*can_rx_callback_t)(canNumber_e can, drvCanMsg_t *msg);

/**
 * @brief CAN错误回调函数类型(Bus-Off/错误被动等)
 * @param  can                   : CAN编号
 * @param  error_type            : 错误类型(0=Bus-Off恢复完成, 1=错误被动, 2=警告, 3=其他)
 */
typedef void (*can_err_callback_t)(canNumber_e can, uint8_t error_type);

/**
 * @brief CAN诊断统计(运行期累计, 上层可读取用于总线负载/通信质量监控)
 */
typedef struct
{
	uint32_t rx_frame_count;     /* 接收帧数(累计) */
	uint32_t tx_frame_count;     /* 发送帧数(累计) */
	uint32_t rx_overflow_count;  /* FIFO溢出次数(读取出错或来不及处理) */
	uint32_t tx_fail_count;      /* 发送失败次数(TxFIFO满或总线错误) */
	uint32_t busoff_count;       /* Bus-Off 恢复次数 */
	uint32_t error_passive_count;/* 进入错误被动次数 */
} drvCanDiag_t;

/**
 * @brief       初始化CAN(配置过滤器、启动、使能接收中断)
 * @param        can               : CAN编号
 * @param        filter            : 过滤器配置，NULL时接收所有ID
 * @return       : DRV_EOK成功，DRV_ERROR失败
 */
int drv_can_init(canNumber_e can, drvCanFilter_t *filter);

/**
 * @brief       初始化CAN并配置双掩码过滤器(单播+广播)
 *               FDCAN 使用两个过滤器元素, 经典CAN使用两个 Filter Bank。
 * @param        can               : CAN编号
 * @param        dual_filter       : 双过滤器配置
 * @return       : DRV_EOK成功，DRV_ERROR失败
 */
int drv_can_init_dual_filter(canNumber_e can, drvCanDualFilter_t *dual_filter);

/**
 * @brief       发送一帧CAN报文
 * @param        can               : CAN编号
 * @param        msg               : 待发送报文(msg->is_fd 决定 FD 格式)
 * @return       : DRV_EOK成功，DRV_ERROR失败(TxFIFO满也返回失败)
 */
int drv_can_send(canNumber_e can, drvCanMsg_t *msg);

/**
 * @brief       等待所有已排队报文完成发送
 * @note        仅在线程/主循环调用，禁止在ISR中调用
 * @return      DRV_EOK=发送队列已空，DRV_ERROR=超时或句柄无效
 */
int drv_can_wait_tx_idle(canNumber_e can, uint32_t timeout_ms);

/**
 * @brief       运行期停止CAN并重新配置单播+广播双过滤器
 * @note        仅在线程/主循环调用，禁止在ISR中调用
 */
int drv_can_reconfigure_dual_filter(canNumber_e can,
                                    drvCanDualFilter_t *dual_filter);

/**
 * @brief       接收一帧CAN报文(从FIFO0读取)
 * @param        can               : CAN编号
 * @param        msg               : 输出报文(msg->is_fd 标识 FD 帧)
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

/**
 * @brief       注册错误回调(Bus-Off恢复/错误被动等事件)
 * @param        can               : CAN编号
 * @param        callback          : 回调函数, NULL取消注册
 * @return       : DRV_EOK成功，DRV_ERROR失败
 */
int drv_can_register_err_callback(canNumber_e can, can_err_callback_t callback);

/**
 * @brief       获取诊断统计(复制到调用方, 线程安全: 内部关中断)
 * @param        can               : CAN编号
 * @param        diag              : 输出诊断数据
 * @return       : DRV_EOK成功，DRV_ERROR失败
 */
int drv_can_get_diag(canNumber_e can, drvCanDiag_t *diag);

#endif /* USE_CAN_DRIVER */
#endif /* __DRV_CAN_H__ */
