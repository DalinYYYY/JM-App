/**
 * @file        cascade_control.h
 * @brief       三环级联控制算法层（位置环 → 速度环 → 电流参考）
 * @details     纯算法模块，不依赖任何硬件。输入上游状态机生成的 motor_ref_t
 *              与电机反馈，输出 dq 轴电流参考（id_ref/iq_ref）交由电流环执行。
 *
 *              环路结构（级联）：
 *                位置环 ──vel_sp──► 速度环 ──iq_ref──► [电流环]
 *
 *              PID 全部通过 motor_pid_profile 统一封装：位置环与速度环使用的参数
 *              配置文件由 motor_ref_t.pos_profile / vel_profile 指定，使不同
 *              运行模式（标准/点动/回零/阻抗）可加载不同 PID 参数。
 *
 *              入环层级由 ref.ctrl_type 决定：
 *                POSITION : 位置环→速度环→电流参考
 *                VELOCITY : 速度环→电流参考
 *                TORQUE   : 力矩÷kt 直接得电流参考（跳过外环）
 *                CURRENT  : 直接使用 id/iq 参考（跳过全部外环）
 *                IDLE     : 电流参考置零
 *
 *              无扰切换：ctrl_type 或 profile 变化时对新入环 PID 做积分预装载，
 *              使入环瞬间输出连续，无冲击（bumpless transfer）。
 *
 * @author      yangsl
 * @version     1.0
 * @date        2026-06-11
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 */

#ifndef __CASCADE_CONTROL_H__
#define __CASCADE_CONTROL_H__

#include <stdint.h>
#include "pid_core.h"
#include "motor_pid_profile.h"
#include "motor_param.h"
#include "motor_control.h"

/**
 * @brief 级联控制反馈输入
 */
typedef struct
{
	float pos; // 实际位置(rad)
	float vel; // 实际速度(rad/s)
	float id;  // 实际d轴电流(A)
	float iq;  // 实际q轴电流(A)
} cascade_fb_t;

/**
 * @brief 级联控制输出（dq轴电流参考）
 */
typedef struct
{
	float id_ref; // d轴电流参考(A)
	float iq_ref; // q轴电流参考(A)
} cascade_out_t;

/**
 * @brief 三环级联控制器
 * @note PID 参数由 motor_pid_profile 统一管理，本结构仅持有外环运行时状态。
 */
typedef struct
{
	motor_param_t *param; // 电机参数（限幅来源）

	// 外环 PID 运行时状态（电流环在 current_loop 模块内）
	pid_state_t pid_pos; // 位置环
	pid_state_t pid_vel; // 速度环

	float vel_setpoint; // 位置环输出的速度设定（调试可观测）

	ref_ctrl_type_e last_ctrl_type;	   // 上一拍入环层级（无扰切换检测）
	motor_pid_profile_id_e last_pos_profile; // 上一拍位置环配置文件
	motor_pid_profile_id_e last_vel_profile; // 上一拍速度环配置文件

	float dt_pos; // 位置环周期(s)
	float dt_vel; // 速度环周期(s)
} cascade_ctrl_t;

/**
 * @brief 初始化级联控制器
 * @param c 级联控制器指针
 * @param param 电机参数指针
 * @param dt_pos 位置环控制周期(s)
 * @param dt_vel 速度环控制周期(s)
 */
void cascade_control_init(cascade_ctrl_t *c, motor_param_t *param, float dt_pos, float dt_vel);

/**
 * @brief 复位级联控制器（清零所有外环积分与状态）
 * @param c 级联控制器指针
 */
void cascade_control_reset(cascade_ctrl_t *c);

/**
 * @brief 运行位置环（输出速度设定，写入 c->vel_setpoint）
 * @param c 级联控制器指针
 * @param ref 参考输入
 * @param fb 反馈输入
 * @note 仅在 ctrl_type==POSITION 时需要调用，按位置环节拍执行
 */
void cascade_control_run_position(cascade_ctrl_t *c, const motor_ref_t *ref, const cascade_fb_t *fb);

/**
 * @brief 运行速度环 + 入环分发，输出 dq 电流参考
 * @param c 级联控制器指针
 * @param ref 参考输入
 * @param fb 反馈输入
 * @param out 输出电流参考
 * @note 按速度环节拍执行；内部根据 ref.ctrl_type 决定是否使用 vel_setpoint。
 *       检测到 ctrl_type/profile 变化时自动做积分预装载实现无扰切换。
 */
void cascade_control_run(cascade_ctrl_t *c, const motor_ref_t *ref, const cascade_fb_t *fb, cascade_out_t *out);

#endif /* __CASCADE_CONTROL_H__ */
