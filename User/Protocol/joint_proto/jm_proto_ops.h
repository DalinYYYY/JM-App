/**
 * @file        jm_proto_ops.h
 * @brief       关节电机协议-业务回调实现(传输无关, 串口/CAN 共用)
 *
 * @author      Dalin (dalinyy@163.com)
 * @version     1.0
 * @date        2026-06-23
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @par 修改日志:
 * | 日期       | 版本 | 作者   | 修改内容   |
 * |------------|------|--------|------------|
 * | 2026-06-23 | 1.0  | Dalin  | 初始创建   |
 * | 2026-06-25 | 1.1  | Dalin  | 增补 jm_app_can_baudrate 声明 |
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 * @note        把 jm_proto_ops_t 的全部回调集中实现在一处, 对接固件应用层:
 *              - 反馈/状态     ← usr.motor_state[M1]   (runtime_param)
 *              - 控制/模式     → motor_loop (sys.motor.cmd + motor_loop_set_cmd)
 *              - 参数读写      ← usr.motor_param[M1]    (motor_param, 按 param_id 表寻址)
 *              - 设备信息      ← version.h + usr.sys.info
 *              本层只依赖应用层(DataHub/MotorControl), 与具体传输介质无关,
 *              因此 jm_proto_uart / jm_proto_can 两条绑定层可注入同一套 ops。
 */
#ifndef __JM_PROTO_OPS_H__
#define __JM_PROTO_OPS_H__

#include "jm_proto.h"

#ifdef __cplusplus
extern "C"
{
#endif

	/* 前向声明: 仅 jm_app_param_storage_save 用到指针, 不在协议头里拉入应用层
	 * motor_param.h, 保持本头与传输/应用解耦(实现文件再包含完整定义)。*/
	typedef struct motor_param motor_param_t;

	/**
	 * @brief  获取应用层业务回调集(单例)
	 * @return 指向静态 jm_proto_ops_t 的指针, 可直接注入 jm_proto_uart_init /
	 *         jm_proto_can_init / dev_commun_uart.set_ops。
	 */
	const jm_proto_ops_t *jm_app_ops_get(void);

	/**
	 * @brief  读取实时反馈(供绑定层主动上报遥测复用, 与 ops.get_feedback 同源)
	 * @param  fb 输出反馈结构
	 * @return JM_ERR_OK 成功
	 */
	jm_err_e jm_app_get_feedback(jm_feedback_t *fb);

	/**
	 * @brief  周期遥测上报总开关(SET_TELEMETRY 0xCB 的 enable 字段写入)
	 * @return 1=已使能周期上报, 0=已停止。绑定层每拍据此决定是否推送 0xCA。
	 */
	uint8_t jm_app_telemetry_enabled(void);

	/**
	 * @brief  当前遥测订阅掩码(SET_TELEMETRY 0xCB 写入), 0 表示订阅全部
	 */
	uint16_t jm_app_telemetry_mask(void);

	/**
	 * @brief  当前遥测上报周期(ms, SET_TELEMETRY 0xCB 写入), 0 表示沿用默认
	 */
	uint16_t jm_app_telemetry_period_ms(void);

	/* PID DEBUG 会话租约检查: 通信线程每拍调用，超时自动回滚。 */
	void jm_app_pid_debug_poll(void);

	/**
	 * @brief  当前 CAN 波特率码(SET_BAUDRATE 0xF1 写入)
	 * @return 0=1M(默认) 1=500K 2=250K 3=125K; 由 CAN 绑定层初始化时读取
	 */
	uint8_t jm_app_can_baudrate(void);

	/**
	 * @brief  参数持久化钩子(可由 Flash 驱动层实现以提供真正掉电保存)
	 * @param  cfg 待保存的电机参数
	 * @return 0 成功, 非 0 失败(映射为 JM_ERR_FLASH)
	 * @note   本模块提供默认弱实现(仅做范围校验, 不落 Flash); 接入 Flash 后,
	 *         在驱动层提供同名强符号即可覆盖, 无需改动协议回调。
	 */
	int jm_app_param_storage_save(const motor_param_t *cfg);

#ifdef __cplusplus
}
#endif
#endif /* __JM_PROTO_OPS_H__ */
