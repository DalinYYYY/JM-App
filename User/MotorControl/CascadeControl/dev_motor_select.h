/**
 * @file        dev_motor_select.h
 * @brief       dev_motor 头文件选择器（真实驱动 / 虚拟电机 二选一）
 * @details     根据 motor_loop_config.h 中的 MOTOR_LOOP_ENABLE_DEV_DRIVER：
 *                - 1：包含真实 dev_motor.h（硬件驱动就绪）
 *                - 0：包含 dev_motor_virtual.h（虚拟电机，dq 物理模型在环仿真）
 *
 *              三环控制层（current_loop.h / motor_loop.h）统一包含本头，而非直接
 *              包含 dev_motor.h，使设备层接的是真实硬件还是虚拟电机对上层透明，
 *              API 签名（dev_motor_t / dev_motor_init）不变。
 */

#ifndef __DEV_MOTOR_SELECT_H__
#define __DEV_MOTOR_SELECT_H__

#include "motor_loop_config.h"

#if (MOTOR_LOOP_ENABLE_DEV_DRIVER)
#include "dev_motor.h"
#else
#include "dev_motor_virtual.h"
#endif

#endif /* __DEV_MOTOR_SELECT_H__ */
