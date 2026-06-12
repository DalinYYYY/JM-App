/**
 * @file        motor_loop_config.h
 * @brief       电机三环控制全局配置
 * @details     电流环运行在中断基频（由触发电流环中断的硬件定时器/ADC 决定，
 *              本层不配置）。速度环、位置环通过分频在同一中断内执行——此处仅
 *              配置两者相对电流环的分频系数。
 *
 *              分频判断在中断中执行，为兼顾执行效率，采用自增计数器与阈值比较，
 *              避免在中断里做取模/除法运算。
 *
 *              示例（电流环 20kHz 时）：
 *                VEL_DIV = 5  → 速度环 4kHz
 *                POS_DIV = 10 → 位置环 2kHz
 *
 * @author      yangsl
 * @version     1.0
 * @date        2026-06-11
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 */

#ifndef __MOTOR_LOOP_CONFIG_H__
#define __MOTOR_LOOP_CONFIG_H__

/*============================================================================
 * 分频系数配置（用户可修改）
 *   - 电流环频率由中断决定，不在此配置
 *   - 速度/位置环 = 电流环频率 / 对应分频系数
 *   - 约束：POS_DIV 必须为 VEL_DIV 的整数倍，保证两环节拍对齐
 *==========================================================================*/

/** @brief 速度环分频：每 N 个电流环中断执行一次速度环 */
#ifndef MOTOR_LOOP_VEL_DIV
#define MOTOR_LOOP_VEL_DIV 5u
#endif

/** @brief 位置环分频：每 N 个电流环中断执行一次位置环 */
#ifndef MOTOR_LOOP_POS_DIV
#define MOTOR_LOOP_POS_DIV 10u
#endif

#if (MOTOR_LOOP_POS_DIV % MOTOR_LOOP_VEL_DIV) != 0
#error "MOTOR_LOOP_POS_DIV 必须为 MOTOR_LOOP_VEL_DIV 的整数倍"
#endif

#endif /* __MOTOR_LOOP_CONFIG_H__ */
