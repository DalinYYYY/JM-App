/**
 * @file        system_state.h
 * @brief 		系统状态机核心头文件
 * 
 * @author      name (name@robot.com)
 * @version     1.0
 * @date        2026-06-11
 * 
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 * 
 * 
 * @par 修改日志:
 * | 日期       | 版本 | 作者   | 修改内容   |
 * |------------|------|--------|------------|
 * | 2026-06-11     | 1.0  | yangsl | 初始创建   |
 * 
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 * 
 * @details 定义系统状态机的核心数据结构、状态切换接口、控制循环入口等，
 *          负责电机控制系统的顶层状态管理（包括故障处理、运行模式切换、控制指令处理）
 */

#ifndef __SYSTEM_STATE_H__
#define __SYSTEM_STATE_H__

#include "state_define.h"
#include "motor_control.h"
#include "ctrl_transition.h"
#include "calib_types.h"

/* 前向声明，避免 system_state.h 直接依赖 dev_motor.h */
struct dev_motor;

typedef struct
{
	top_fsm_e top_state;		  /*!< 顶层有限状态机状态 */
	ctrl_mode_e ctrl_mode;	  /*!< 当前控制模式 */
	uint32_t fault_code;		  /*!< 系统故障码 */
	motor_ctrl_t motor;			  /*!< 电机控制核心上下文 */
	transition_t transition;	  /*!< 状态过渡器 */
	run_state_e target_run_state; /*!< 目标运行状态 */
	calib_state_e calib_state;	/*!< 标定子状态（仅 CALIB 态有效）*/
} system_state_t;

/**
 * @brief 运行模式平滑过渡的调用次数（可配置）
 * @details 过渡时长以 motor_control_loop（建议在电流环中调用）的调用次数计，
 *          而非软件定时器。配置值 = 期望过渡时长 / 电流环周期。
 */
extern uint32_t g_run_state_trans_count;

/**
 * @brief 初始化系统状态机
 * @param[in,out] sys 系统状态机实例指针（非NULL）
 * @param[in] motor 底层电机设备指针（供标定模块操作 FOC/编码器/半桥）
 * @param[in] param 电机参数配置指针（非NULL，含电机额定参数、PID参数等）
 * @param[in] dt 控制周期（单位：秒，如1e-4表示100us）
 * @retval 无
 * @note 会初始化电机控制上下文、状态过渡器，并将顶层状态初始化为IDLE
 */
void system_state_init(system_state_t *sys, struct dev_motor *motor, motor_param_t *param, float dt);

/**
 * @brief 切换顶层有限状态机状态
 * @param[in,out] sys 系统状态机实例指针（非NULL）
 * @param[in] new_state 目标顶层状态（如TOP_FSM_IDLE/TOP_FSM_RUN/TOP_FSM_FAULT等）
 * @retval 无
 * @note 1. 若目标状态超出范围或与当前状态一致，不执行任何操作；
 *       2. 切换前会处理原状态的收尾（如RUN切其他状态时清零电压、复位PID）；
 *       3. 切换后会初始化新状态的核心参数（如FAULT状态清零电压）
 */
void top_fsm_switch(system_state_t *sys, top_fsm_e new_state);

/**
 * @brief 切换电机运行状态（带平滑过渡）
 * @param[in,out] sys 系统状态机实例指针（非NULL）
 * @param[in] new_state 目标运行状态（如RUN_STATE_POSITION/RUN_STATE_TORQUE等）
 * @param[in] trans_count 过渡时长（以 motor_control_loop 的调用次数计，非毫秒）
 * @retval 无
 * @note 1. 若目标状态超出范围或与当前状态一致，不执行任何操作；
 *       2. 会启动参考层过渡器；同量纲模式做参考线性混合，异量纲模式
 *          直接切换并依赖下游三环预装载积分实现无扰切换
 */
void run_state_switch(system_state_t *sys, run_state_e new_state, uint32_t trans_count);

/**
 * @brief 电机控制主循环（中断级执行）
 * @param[in,out] sys 系统状态机实例指针（非NULL）
 * @retval 无
 * @note 1. 先执行故障检测，故障/安全状态下参考置为IDLE；
 *       2. 处理运行状态的平滑过渡（若有），生成 motor.ref 参考输出；
 *       3. 下游三环模块读取 sys->motor.ref，按 ref.ctrl_type 入环并自行分频；
 *       4. 需在硬件中断（如定时器）中调用，保证执行周期稳定
 */
void motor_control_loop(system_state_t *sys);

/**
 * @brief 系统故障检测
 * @param[in,out] sys 系统状态机实例指针（非NULL）
 * @retval 无
 * @note 1. 检测编码器故障等核心故障，映射到fault_code；
 *       2. 检测到故障时自动切换顶层状态为FAULT；
 *       3. 可扩展其他故障类型（如过流、过压、过温等）
 */
void fault_check(system_state_t *sys);

/**
 * @brief 处理上层控制指令
 * @param[in,out] sys 系统状态机实例指针（非NULL）
 * @param[in] cmd 控制指令（如CONTROL_MODE_POSITION/CONTROL_MODE_CLEAR_FAULT等）
 * @retval 无
 * @note 1. 故障状态下仅响应CLEAR_FAULT指令；
 *       2. 处理IDLE/ESTOP/BOOTLOADER等顶层指令，以及各类运行模式指令；
 *       3. 运行模式指令会映射为对应运行状态，并触发平滑切换
 */
void process_ctrl_cmd(system_state_t *sys, ctrl_mode_e cmd);

#endif /* __SYSTEM_STATE_H__ */
