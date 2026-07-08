#ifndef __MOTOR_PID_PROFILE_H__
#define __MOTOR_PID_PROFILE_H__

#include <stdint.h>
#include <stdbool.h>
#include "pid_core.h"
#include "motor_param.h"

/**
 * @brief PID参数配置文件ID
 */
typedef enum
{
	MOTOR_PID_PROFILE_CURRENT_D = 0, // d轴电流环
	MOTOR_PID_PROFILE_CURRENT_Q,	   // q轴电流环
	MOTOR_PID_PROFILE_VELOCITY,	   // 速度环
	MOTOR_PID_PROFILE_POSITION,	   // 位置环
	MOTOR_PID_PROFILE_IMPEDANCE,	   // 阻抗控制
	MOTOR_PID_PROFILE_HOMING,		   // 回零模式
	MOTOR_PID_PROFILE_JOG,		   // 点动模式
	MOTOR_PID_PROFILE_TEST,		   // 测试模式
	MOTOR_PID_PROFILE_MAX
} motor_pid_profile_id_e;

/**
 * @brief PID参数配置文件
 */
typedef struct
{
	pid_param_t param;
	char name[16];
} motor_pid_profile_t;

/**
 * @brief 初始化PID参数管理器
 * @param motor_param 电机参数指针
 */
void motor_pid_profile_init(const motor_param_t *motor_param);

/**
 * @brief 获取指定ID的PID参数配置文件
 * @param id 配置文件ID
 * @return PID参数指针，失败返回NULL
 */
const pid_param_t *motor_pid_profile_get(motor_pid_profile_id_e id);

/**
 * @brief 设置指定ID的PID参数
 * @param id 配置文件ID
 * @param param 新的PID参数
 * @return 0成功，-1失败
 */
int motor_pid_profile_set(motor_pid_profile_id_e id, const pid_param_t *param);

/**
 * @brief 从电机参数加载PID配置
 * @param motor_param 电机参数指针
 */
void motor_pid_profile_load_from_motor_param(const motor_param_t *motor_param);

/**
 * @brief 保存PID配置到电机参数
 * @param motor_param 电机参数指针
 */
void motor_pid_profile_save_to_motor_param(motor_param_t *motor_param);

/**
 * @brief 恢复指定配置文件的默认值
 * @param id 配置文件ID
 */
void motor_pid_profile_restore_default(motor_pid_profile_id_e id);

/**
 * @brief 初始化PID运行时状态
 * @param state PID状态指针
 */
void motor_pid_profile_init_state(pid_state_t *state);

/**
 * @brief 复位PID运行时状态
 * @param state PID状态指针
 */
void motor_pid_profile_reset_state(pid_state_t *state);

/**
 * @brief 无扰预装载PID积分项
 * @param state PID状态指针
 * @param id 配置文件ID（用于读取积分限幅）
 * @param output_now 期望的当前输出值（通常为入环瞬间的实际反馈）
 * @param actual 当前实际值（用于初始化微分项基准）
 * @details 令PID入环瞬间输出≈output_now，避免积分从0起步造成的输出突变，
 *          实现无扰切换（bumpless transfer）。
 */
void motor_pid_profile_preload(pid_state_t *state, motor_pid_profile_id_e id, float output_now, float actual);

/**
 * @brief 使用指定配置文件执行PID计算
 * @param state PID状态指针
 * @param id 配置文件ID
 * @param target 目标值
 * @param actual 实际值
 * @param dt 控制周期(s)
 * @return PID输出值
 */
float motor_pid_profile_calculate(pid_state_t *state,
							motor_pid_profile_id_e id,
							float target,
							float actual,
							float dt);

/**
 * @brief 使用指定配置文件执行带前馈的PID计算
 * @param state PID状态指针
 * @param id 配置文件ID
 * @param target 目标值
 * @param actual 实际值
 * @param feedforward 前馈值
 * @param dt 控制周期(s)
 * @return PID输出值
 */
float motor_pid_profile_calculate_with_ff(pid_state_t *state,
									motor_pid_profile_id_e id,
									float target,
									float actual,
									float feedforward,
									float dt);

#endif /* __MOTOR_PID_PROFILE_H__ */
