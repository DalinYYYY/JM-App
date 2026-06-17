#ifndef __PID_CORE_H__
#define __PID_CORE_H__

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief PID控制器运行时状态
 * @note 仅存储计算过程中的状态，不存储参数
 */
typedef struct
{
	float integral;
	float prev_error;
	float prev_output;
	float prev_actual;
} pid_state_t;

/**
 * @brief PID算法配置标志位
 */
typedef enum
{
	PID_FLAG_NONE = 0,
	PID_FLAG_ANTI_WINDUP = (1 << 0),				 // 启用抗积分饱和
	PID_FLAG_DIFFERENTIAL_ON_MEASUREMENT = (1 << 1), // 微分先行（对测量值微分）
	PID_FLAG_INCREMENTAL = (1 << 2),				 // 增量式PID
	PID_FLAG_OUTPUT_FILTER = (1 << 3),				 // 输出滤波
} pid_flag_e;

/**
 * @brief PID计算参数
 * @note 每次计算时传入，支持动态参数调整
 */
typedef struct
{
	float kp;
	float ki;
	float kd;
	float output_limit;
	float integral_limit;
	float output_filter_alpha; // 输出滤波系数(0.0~1.0)
	uint32_t flags;			   // PID算法标志位
} pid_param_t;

/**
 * @brief 初始化PID运行时状态
 * @param state PID状态指针
 */
void pid_core_init(pid_state_t *state);

/**
 * @brief 复位PID运行时状态
 * @param state PID状态指针
 */
void pid_core_reset(pid_state_t *state);

/**
 * @brief 执行PID计算
 * @param state PID状态指针
 * @param param PID参数指针
 * @param target 目标值
 * @param actual 实际值
 * @param dt 控制周期(s)
 * @return PID输出值
 */
float pid_core_calculate(pid_state_t *state,
						 const pid_param_t *param,
						 float target,
						 float actual,
						 float dt);

/**
 * @brief 带前馈的PID计算
 * @param state PID状态指针
 * @param param PID参数指针
 * @param target 目标值
 * @param actual 实际值
 * @param feedforward 前馈值
 * @param dt 控制周期(s)
 * @return PID输出值
 */
float pid_core_calculate_with_ff(pid_state_t *state,
								 const pid_param_t *param,
								 float target,
								 float actual,
								 float feedforward,
								 float dt);

#endif /* __PID_CORE_H__ */
