/**
 * @file    motor_pid_autotune.h
 * @brief   PID 理论估计模块（零极点对消法）
 * @date    2026-07-08
 *
 * @details 基于电机辨识参数（R/L/J/Kt）用零极点对消法自动计算三环 PID：
 *          - 电流环: Kp = ωc·L, Ki = ωc·R （零极点对消，带宽 ≈ ωc）
 *          - 速度环: Kp = J·ωc/Kt, Ki = J·ωc²/(4·Kt) （二阶最佳阻尼 ξ=0.707）
 *          - 位置环: Kp = 2π·f （纯比例，带宽 = Kp）
 *
 *          前置条件：电机辨识已完成（is_calibrated == 1 且 R/L 非默认值）。
 *          事务语义：motor_pid_autotune_apply 任一环失败则全部不写入。
 *          带宽参数 = 0 时用推荐默认值（电流环 1kHz / 速度环 100Hz / 位置环 20Hz）。
 */
#ifndef __MOTOR_PID_AUTOTUNE_H__
#define __MOTOR_PID_AUTOTUNE_H__

#include <stdint.h>
#include "motor_info.h"

/**
 * @brief  单环理论估计结果
 */
typedef struct
{
	float kp;              /* 比例增益 */
	float ki;              /* 积分增益 */
	float integral_limit;  /* 积分限幅 */
} autotune_result_t;

/**
 * @brief  电流环理论估计（d/q 双轴，零极点对消法）
 * @param  info         motor_info 指针（读取辨识参数 R/L）
 * @param  bandwidth_hz 期望电流环带宽 (Hz)，≤0 时用默认 1000Hz
 * @param  out_d        d轴结果输出
 * @param  out_q        q轴结果输出
 * @retval 0=成功, -1=参数无效, -2=辨识数据未就绪
 */
int motor_pid_autotune_current(const motor_info_t *info, float bandwidth_hz,
                               autotune_result_t *out_d, autotune_result_t *out_q);

/**
 * @brief  速度环理论估计（二阶最佳阻尼 ξ=0.707）
 * @param  info         motor_info 指针（读取 J/Kt）
 * @param  bandwidth_hz 期望速度环带宽 (Hz)，≤0 时用默认 100Hz
 * @param  out          结果输出
 * @retval 0=成功, -1=参数无效, -2=辨识数据未就绪
 */
int motor_pid_autotune_velocity(const motor_info_t *info, float bandwidth_hz,
                                autotune_result_t *out);

/**
 * @brief  位置环理论估计（纯比例，带宽 = Kp）
 * @param  info         motor_info 指针
 * @param  bandwidth_hz 期望位置环带宽 (Hz)，≤0 时用默认 20Hz
 * @param  out          结果输出
 * @retval 0=成功, -1=参数无效
 */
int motor_pid_autotune_position(const motor_info_t *info, float bandwidth_hz,
                                autotune_result_t *out);

/**
 * @brief  事务性应用：计算三环并写入 ControlParam_t
 * @param  info          motor_info 指针（可写，理论值写入 control 段）
 * @param  current_bw_hz 电流环带宽 (Hz)，≤0 用默认
 * @param  velocity_bw_hz 速度环带宽 (Hz)，≤0 用默认
 * @param  position_bw_hz 位置环带宽 (Hz)，≤0 用默认
 * @retval 0=成功, -1=参数无效, -2=辨识数据未就绪
 * @note   事务语义：任一环计算失败则全部不写入，返回错误码。
 *         写入后需调用 motor_pid_reload() 生效，由上位机显式发 0xEA 固化。
 */
int motor_pid_autotune_apply(motor_info_t *info, float current_bw_hz,
                             float velocity_bw_hz, float position_bw_hz);

#endif /* __MOTOR_PID_AUTOTUNE_H__ */
