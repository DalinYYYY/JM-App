#include "pid_core.h"
#include <math.h>

static float clamp(float value, float min, float max)
{
	if (value < min)
		return min;
	if (value > max)
		return max;
	return value;
}

void pid_core_init(pid_state_t *state)
{
	pid_core_reset(state);
}

void pid_core_reset(pid_state_t *state)
{
	state->integral = 0.0f;
	state->prev_error = 0.0f;
	state->prev_output = 0.0f;
	state->prev_actual = 0.0f;
}

float pid_core_calculate(pid_state_t *state,
						 const pid_param_t *param,
						 float target,
						 float actual,
						 float dt)
{
	return pid_core_calculate_with_ff(state, param, target, actual, 0.0f, dt);
}

float pid_core_calculate_with_ff(pid_state_t *state,
								 const pid_param_t *param,
								 float target,
								 float actual,
								 float feedforward,
								 float dt)
{
	float error = target - actual;
	float output = feedforward;

	// 比例项
	float p_term = param->kp * error;
	output += p_term;

	// 积分项（带抗积分饱和）
	float i_term = 0.0f;
	if (param->ki != 0.0f && dt > 0.0f)
	{
		bool enable_integral = true;
		if ((param->flags & PID_FLAG_ANTI_WINDUP) && (fabsf(state->prev_output) >= param->output_limit))
		{
			// 输出饱和时停止积分
			enable_integral = false;
		}

		if (enable_integral)
		{
			state->integral += param->ki * error * dt;
			state->integral = clamp(state->integral,
									-param->integral_limit,
									param->integral_limit);
		}
		i_term = state->integral;
		output += i_term;
	}

	// 微分项
	float d_term = 0.0f;
	if (param->kd != 0.0f && dt > 0.0f)
	{
		if (param->flags & PID_FLAG_DIFFERENTIAL_ON_MEASUREMENT)
		{
			// 微分先行：对实际值微分，避免目标阶跃冲击
			d_term = param->kd * (state->prev_actual - actual) / dt;
		}
		else
		{
			// 标准微分：对误差微分
			d_term = param->kd * (error - state->prev_error) / dt;
		}
		output += d_term;
	}

	// 增量式PID处理
	if (param->flags & PID_FLAG_INCREMENTAL)
	{
		output += state->prev_output;
	}

	// 输出限幅
	output = clamp(output, -param->output_limit, param->output_limit);

	// 输出滤波
	if (param->flags & PID_FLAG_OUTPUT_FILTER)
	{
		output = param->output_filter_alpha * output + (1.0f - param->output_filter_alpha) * state->prev_output;
	}

	// 更新状态
	state->prev_error = error;
	state->prev_actual = actual;
	state->prev_output = output;

	return output;
}
