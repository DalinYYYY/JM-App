/**
 * @file        current_loop.h
 * @brief       FOC 电流环控制模块（dq 电流闭环 + SVPWM + PWM 输出）
 * @details     封装 FOC 电流环完整链路：
 *                编码器电角度 → 三相电流采样 → Clarke → Park
 *                → d/q 轴 PI（pid_profile 统一封装）→ 反 Park → SVPWM → PWM
 *
 *              PID 参数通过 pid_profile 管理（PID_PROFILE_CURRENT_D / _Q），
 *              可在运行时按需调参，与位置/速度环共用同一套参数体系。
 *
 *              底层接口调用方式参考 foc_current_control.c。
 *
 * @author      yangsl
 * @version     1.0
 * @date        2026-06-11
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 */

#ifndef __CURRENT_LOOP_H__
#define __CURRENT_LOOP_H__

#include <stdint.h>
#include "pid_core.h"
#include "dev_motor.h"

/**
 * @brief 电流环控制器
 * @note  命名为 cur_loop_t 以避免与 motor_param 的 current_loop 参数子结构混淆。
 */
typedef struct
{
	dev_motor_t *motor; // 关联的底层电机设备

	pid_state_t pid_id; // d轴电流环 PID 运行状态
	pid_state_t pid_iq; // q轴电流环 PID 运行状态

	float dt; // 电流环控制周期(s)，由中断频率决定
} cur_loop_t;

/**
 * @brief 初始化电流环
 * @param cl 电流环控制器指针
 * @param motor 底层电机设备指针
 * @param dt 电流环控制周期(s)
 */
void cur_loop_init(cur_loop_t *cl, dev_motor_t *motor, float dt);

/**
 * @brief 复位电流环 PID 状态
 * @param cl 电流环控制器指针
 */
void cur_loop_reset(cur_loop_t *cl);

/**
 * @brief 运行一次电流环（在电流环中断中按基频调用）
 * @param cl 电流环控制器指针
 * @param id_ref d轴电流参考(A)
 * @param iq_ref q轴电流参考(A)
 * @note 内部完成编码器电角度与三相电流刷新
 */
void cur_loop_run(cur_loop_t *cl, float id_ref, float iq_ref);

/**
 * @brief 电流采样零位校准
 * @param cl 电流环控制器指针
 */
void cur_loop_calibrate_offset(cur_loop_t *cl);

#endif /* __CURRENT_LOOP_H__ */
