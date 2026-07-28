/**
 * @file        jm_host_commun.h
 * @brief       关节电机上位机通信(承载 joint_proto 协议)接入层
 *
 * @author      Dalin (dalinyy@163.com)
 * @version     1.0
 * @date        2026-06-24
 *
 * @copyright   Copyright (c) 2026 Robot Tech.co, Ltd. All rights reserved.
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 * @note        面向自研 PyQt 上位机的串口通信模块, 取代已废弃的 serialstudio_commun:
 *              - 复用 jm_proto_ops 业务回调(反馈/参数/控制, 串口/CAN 共用)
 *              - 遥控模式: 上位机用 SET_TELEMETRY(0xCB) 配置 enable/mask/period,
 *                本模块按 mask 变长打包 TELEMETRY(0xCA) 周期主动上报, 不要求逐帧应答。
 *              通信线程只需 init() 一次, 每周期调 process()。
 */
#ifndef __JM_HOST_COMMUN_H__
#define __JM_HOST_COMMUN_H__

#include "dev_config.h"
#if defined(USE_DEV_COMMUN_UART) || defined(USE_DEV_COMMUN_CAN)

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#if defined(USE_DEV_COMMUN_UART)
	/**
	 * @brief 初始化关节电机上位机串口通信(UART+DMA空闲中断)
	 * @note  内部完成: 设备 init → 注入 joint_proto 业务回调 → 启动不定长接收。
	 *        须在通信线程进入主循环前调用一次。
	 */
	void jm_host_commun_init(void);

	/**
	 * @brief UART RX IDLE 中断唤醒通信线程
	 * @note  仅置通信信号量，不在中断中解析协议。
	 */
	void jm_host_commun_notify_rx(void);

	/**
	 * @brief 通信周期处理(在通信线程主循环每拍调用)
	 * @note  取空闲突发数据喂协议栈(命令分发+应答); 当遥控开关使能时, 按订阅周期
	 *        分频按 mask 变长打包主动推送 TELEMETRY(0xCA), 全程无需上位机应答。
	 */
	void jm_host_commun_process(void);

	/**
	 * @brief 等待通信事件或周期超时
	 * @param timeout_ms 最大等待时间，超时后用于维持遥测周期处理。
	 */
	void jm_host_commun_wait(uint32_t timeout_ms);
#endif /* USE_DEV_COMMUN_UART */

#if defined(USE_DEV_COMMUN_CAN)
	/**
	 * @brief 初始化关节电机上位机 CAN/CAN-FD 通信
	 * @note  内部完成: 设备 init → 注入 joint_proto 业务回调 → 启动 CAN(过滤器+中断)。
	 *        须在通信线程进入主循环前调用一次。与 UART 路径相互独立, 可并存。
	 *        业务回调(jm_proto_ops_t)与 UART 共用同一份, 两路传输行为一致。
	 */
	void jm_host_commun_can_init(void);

	/**
	 * @brief CAN 通信周期处理(在通信线程主循环每拍调用)
	 * @note  CAN 接收在 ISR 中直接 feed 协议层(零拷贝), 本函数仅做:
	 *        - 诊断统计刷新(poll);
	 *        - 通信中断降级检查(超 500ms 无帧则通知应用切 IDLE)。
	 */
	void jm_host_commun_can_process(void);
#endif /* USE_DEV_COMMUN_CAN */

#ifdef __cplusplus
}
#endif

#endif /* USE_DEV_COMMUN_UART || USE_DEV_COMMUN_CAN */
#endif /* __JM_HOST_COMMUN_H__ */
