#ifndef __MOTOR_MODE_H__
#define __MOTOR_MODE_H__

#include "motor_control.h"

/**
 * @brief 模式处理函数统一签名
 * @details 每个运动模式导出一个此签名函数，由 motor_ctrl_dispatch 查表调用。
 *          函数内仅填充 ctrl->ref（参考输出），不执行任何环路计算。
 *          实时约束：可在电流环中断调用，禁止动态内存/阻塞/字符串格式化。
 * @param ctrl 电机控制上下文（含 run_state/cmd/fb/param/ref）
 */
typedef void (*motor_mode_fn)(motor_ctrl_t *ctrl);

/* 限幅辅助（公共，供各模式文件复用）*/
static inline float motor_mode_clamp(float v, float lo, float hi)
{
	return (v < lo) ? lo : (v > hi) ? hi : v;
}

/* ===== 前期实现的模式（每个模式一个文件）===== */
void motor_mode_idle_run(motor_ctrl_t *ctrl);            /* IDLE */
void motor_mode_hold_run(motor_ctrl_t *ctrl);            /* HOLD */
void motor_mode_open_loop_run(motor_ctrl_t *ctrl);       /* OPEN_LOOP / VOLTAGE_VECTOR */
void motor_mode_duty_run(motor_ctrl_t *ctrl);            /* DUTY_CYCLE */
void motor_mode_current_run(motor_ctrl_t *ctrl);         /* CURRENT / FIELD_WEAKENING / SENSORLESS */
void motor_mode_torque_run(motor_ctrl_t *ctrl);          /* TORQUE */
void motor_mode_mit_run(motor_ctrl_t *ctrl);             /* MIT */
void motor_mode_velocity_run(motor_ctrl_t *ctrl);        /* VELOCITY / VELOCITY_TORQUE */
void motor_mode_position_run(motor_ctrl_t *ctrl);        /* POSITION / POSITION_VELOCITY / POSITION_TORQUE / PP */
void motor_mode_profile_velocity_run(motor_ctrl_t *ctrl);/* PV */
void motor_mode_profile_torque_run(motor_ctrl_t *ctrl);  /* PT */
void motor_mode_test_sweep_run(motor_ctrl_t *ctrl);      /* TEST_SWEEP_FREQ */

/* ===== Legacy fallback（未拆分模式的统一入口）===== */
void motor_mode_legacy_run(motor_ctrl_t *ctrl);          /* 力控/轨迹/特殊/其他测试模式 */

#endif /* __MOTOR_MODE_H__ */
