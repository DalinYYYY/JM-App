/**
 * @file        jm_host_commun.h
 * @brief       关节电机上位机通信(承载 joint_proto 协议)接入层
 *
 * @author      Dalin (dalin@robot.com)
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
#if defined(USE_DEV_COMMUN_UART)

#ifdef __cplusplus
extern "C"
{
#endif

	/**
	 * @brief 初始化关节电机上位机串口通信(UART+DMA空闲中断)
	 * @note  内部完成: 设备 init → 注入 joint_proto 业务回调 → 启动不定长接收。
	 *        须在通信线程进入主循环前调用一次。
	 */
	void jm_host_commun_init(void);

	/**
	 * @brief 通信周期处理(在通信线程主循环每拍调用)
	 * @note  取空闲突发数据喂协议栈(命令分发+应答); 当遥控开关使能时, 按订阅周期
	 *        分频按 mask 变长打包主动推送 TELEMETRY(0xCA), 全程无需上位机应答。
	 */
	void jm_host_commun_process(void);

#ifdef __cplusplus
}
#endif

#endif /* USE_DEV_COMMUN_UART */
#endif /* __JM_HOST_COMMUN_H__ */
