/**
 * @file        current_loop.h
 * @brief       FOC 电流环控制模块（dq 电流闭环 + SVPWM + PWM 输出）
 * @details     封装 FOC 电流环完整链路：
 *                编码器电角度 → 三相电流采样 → Clarke → Park
 *                → d/q 轴 PI（motor_pid_profile 统一封装）→ 反 Park → SVPWM → PWM
 *
 *              PID 参数通过 motor_pid_profile 管理（MOTOR_PID_PROFILE_CURRENT_D / _Q），
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
#include "dev_motor_select.h"
#include "motor_control.h"   /* for motor_ref_t */
#include "cascade_control.h" /* for cascade_out_t */
#include "motor_param.h"
#include "foc_decoupling.h"

/**
 * @brief 电流环控制器
 * @note  命名为 cur_loop_t 以避免与 motor_param 的 current_loop 参数子结构混淆。
 */
typedef struct
{
	dev_motor_t *motor; // 关联的底层电机设备
	motor_param_t *param; // 电机参数（解耦算法依赖 R/L/flux/pole_pairs）
	foc_decoupling_config_t decoupling; // 解耦运行时配置（由 param 构建）

	pid_state_t pid_id; // d轴电流环 PID 运行状态
	pid_state_t pid_iq; // q轴电流环 PID 运行状态

	float dt; // 电流环控制周期(s)，由中断频率决定

	/* 电流环补偿诊断快照: ISR 写入, 通信任务读取。保存 V 域量，
	 * 避免真实硬件 SVPWM 归一化后无法判断补偿是否进入电压指令。 */
	volatile float diag_ud_pi;
	volatile float diag_uq_pi;
	volatile float diag_ud_cross;
	volatile float diag_uq_bemf;
	volatile float diag_ud;
	volatile float diag_uq;
	volatile float diag_omega_mech;
	volatile float diag_vbus;
	volatile float diag_config;
} cur_loop_t;

/**
 * @brief 初始化电流环
 * @param cl 电流环控制器指针
 * @param motor 底层电机设备指针
 * @param param 电机参数指针（解耦算法依赖）
 * @param dt 电流环控制周期(s)
 */
void cur_loop_init(cur_loop_t *cl, dev_motor_t *motor, motor_param_t *param, float dt);

/**
 * @brief 复位电流环 PID 状态
 * @param cl 电流环控制器指针
 */
void cur_loop_reset(cur_loop_t *cl);

/**
 * @brief 运行一次电流环（在电流环中断中按基频调用）
 * @param cl 电流环控制器指针
 * @param ref 电机控制参考（含 ctrl_type / voltage / duty）
 * @param out 级联外环输出的电流参考（id_ref/iq_ref），仅闭环模式使用
 * @note 内部按 ref->ctrl_type 分流：
 *       VOLTAGE — 旁路 PI，直接用 ref->voltage 设 udq
 *       DUTY    — 旁路整个 FOC，直接驱动 half_bridge
 *       IDLE    — PWM 置零
 *       其余    — 正常 PI 电流环（使用 out->id_ref / out->iq_ref）
 */
void cur_loop_run(cur_loop_t *cl, const motor_ref_t *ref, const cascade_out_t *out);

/**
 * @brief 电流采样零位校准
 * @param cl 电流环控制器指针
 */
void cur_loop_calibrate_offset(cur_loop_t *cl);

/**
 * @brief 设置解耦运行时配置（由 motor_param 字段构建）
 * @param cl 电流环控制器指针
 * @note  在 motor_param 的 decouple_algo/bemf_ff_enable/deadtime_comp_enable 变更后调用
 */
void cur_loop_set_decoupling_config(cur_loop_t *cl);

#endif /* __CURRENT_LOOP_H__ */
