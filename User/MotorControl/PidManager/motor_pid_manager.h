/**
 * @file    motor_pid_manager.h
 * @brief   PID 管理模块统一门面
 * @date    2026-07-08
 *
 * @details 本头文件汇总 PidManager 模块的子头文件，外部模块只需 #include "motor_pid_manager.h"
 *          即可使用全部 PID 管理功能：
 *          - motor_pid_profile:       运行时 PID 参数管理（迁移自 ControlProcess）
 *          - motor_pid_load:    三环独立 source 加载 + source 管理 API
 *                               （定义 pid_source_e / pid_ring_e 枚举）
 *          - motor_pid_autotune: 零极点对消法理论估计
 *
 *          source 标志存于 PidManager 内部 static，不持久化到 Flash。
 *          每次上电默认 source=0（用 motor_param.c 默认值），行为向后兼容。
 *          若需持久化，后续让 motor_info_generate.py 脚本支持新增字段。
 */
#ifndef __MOTOR_PID_MANAGER_H__
#define __MOTOR_PID_MANAGER_H__

#include "motor_pid_profile.h"
#include "motor_pid_load.h" /* pid_source_e / pid_ring_e 定义于此 */
#include "motor_pid_autotune.h"

#endif /* __MOTOR_PID_MANAGER_H__ */
