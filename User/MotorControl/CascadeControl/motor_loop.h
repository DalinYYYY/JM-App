/**
 * @file        motor_loop.h
 * @brief       电机三环控制集成/编排层
 * @details     连接电流环、运动反馈、上游状态机与级联外环，提供统一中断入口。
 *              三环在单一中断内按分频执行，频率由 motor_loop_config.h 全局配置：
 *                - 电流环 : 中断基频（current_loop：FOC + PI + SVPWM + PWM）
 *                - 速度环 : 分频执行（级联速度环）
 *                - 位置环 : 分频执行（级联位置环）
 *
 *              数据流：
 *                运动反馈(pos/vel) → 状态机生成 motor_ref → 级联外环
 *                → iq_ref/id_ref → 电流环 → PWM
 *
 * @author      yangsl
 * @version     1.0
 * @date        2026-06-11
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 */

#ifndef __MOTOR_LOOP_H__
#define __MOTOR_LOOP_H__

#include <stdint.h>
#include "dev_motor_select.h"
#include "system_state.h"
#include "cascade_control.h"
#include "current_loop.h"
#include "motor_loop_config.h"
#include "state_define.h"

/**
 * @brief 电机三环控制运行上下文
 */
typedef struct motor_loop_s
{
	dev_motor_t motor;		// 底层电机设备（FOC/编码器/半桥/ADC）
	system_state_t sys;		// 上层状态机（模式管理 + 参考生成）
	cascade_ctrl_t cascade; // 三环级联外环（位置 + 速度）
	cur_loop_t current;		// 电流环
	cascade_out_t out;		// 级联输出（dq电流参考）

	uint32_t sched_cnt; /* 超周期错峰调度计数器(0..POS_DIV-1): 速度拍0/VEL_DIV, 故障拍2, 位置拍4, 遥测拍7, 见 motor_loop_config.h */
} motor_loop_t;

/**
 * @brief 初始化电机三环控制
 * @param current_freq_hz 电流环中断频率(Hz)，由触发中断的硬件决定
 * @note 内部完成 motor_pid_profile、dev_motor、状态机、级联外环、电流环的初始化。
 *       速度/位置环周期 = 电流环周期 × 对应分频系数。
 *       FOC 三相电流/电弧度回调由本模块内部实现（读取全局电机对象）。
 */
void motor_loop_init(float current_freq_hz);

/**
 * @brief 三环控制中断入口（电流环基频）
 * @note 在电流环中断（ADC注入转换完成）中调用
 */
void motor_loop_isr(void);

/**
 * @brief 下发上层控制指令
 * @param cmd 控制指令（ctrl_mode_e）
 */
void motor_loop_set_cmd(ctrl_mode_e cmd);

/**
 * @brief 获取全局电机三环控制上下文
 * @return 上下文指针
 */
motor_loop_t *motor_loop_get(void);

/**
 * @brief 运行时翻转编码器方向(换电机/换安装后调试用)
 * @param dir 方向: 1=CW(正向), -1=CCW(反向)
 * @note  应在 IDLE 状态下调用; 立即生效, 无需重新初始化。
 *        切换后原 enc_offset 失效, 需重新做编码器零位标定。
 */
void motor_loop_set_encoder_dir(int8_t dir);

#endif /* __MOTOR_LOOP_H__ */
