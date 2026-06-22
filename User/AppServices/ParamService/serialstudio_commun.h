/**
 * @file        serialstudio_commun.h
 * @brief       SerialStudio 上位机通信(承载 joint_proto 协议)接入层
 *
 * @author      Dalin (dalin@robot.com)
 * @version     1.0
 * @date        2026-06-22
 *
 * @copyright   Copyright (c) 2026 Robot Tech.co, Ltd. All rights reserved.
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 * @note        从 thread_commun 抽出, 收敛 SerialStudio/joint_proto 相关的:
 *              业务回调(反馈/参数/控制) + 同步遥测订阅与单帧打包上报。
 *              通信线程只需 init() 一次, 每周期调 process()。
 */
#ifndef __SERIALSTUDIO_COMMUN_H__
#define __SERIALSTUDIO_COMMUN_H__

#include "dev_config.h"
#if defined(USE_DEV_COMMUN_UART)

#ifdef __cplusplus
extern "C"
{
#endif

	/**
	 * @brief 初始化 SerialStudio 串口通信(UART+DMA空闲中断)
	 * @note  内部完成: 设备 init → 注入 joint_proto 业务回调 → 启动不定长接收。
	 *        须在通信线程进入主循环前调用一次。
	 */
	void ss_commun_init(void);

	/**
	 * @brief 通信周期处理(在通信线程主循环每拍调用)
	 * @note  取空闲突发数据喂协议栈(命令分发+应答), 并按订阅周期分频主动推送遥测帧。
	 */
	void ss_commun_process(void);

#ifdef __cplusplus
}
#endif

#endif /* USE_DEV_COMMUN_UART */
#endif /* __SERIALSTUDIO_COMMUN_H__ */
