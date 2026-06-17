#include "pid_profile.h"
#include <string.h>
#include <math.h>

static pid_profile_t s_pid_profiles[PID_PROFILE_MAX];
static const motor_param_t *s_motor_param;

/**
 * @brief 默认PID参数配置
 */
static const pid_profile_t s_default_profiles[PID_PROFILE_MAX] = {
	[PID_PROFILE_CURRENT_D] = {.param = {.kp = 0.3f,
										 .ki = 10.0f,
										 .kd = 0.0f,
										 .output_limit = 48.0f,
										 .integral_limit = 10.0f,
										 .output_filter_alpha = 1.0f,
										 .flags = PID_FLAG_ANTI_WINDUP},
							   .name = "current_d"},
	[PID_PROFILE_CURRENT_Q] = {.param = {.kp = 0.3f,
										 .ki = 10.0f,
										 .kd = 0.0f,
										 .output_limit = 48.0f,
										 .integral_limit = 10.0f,
										 .output_filter_alpha = 1.0f,
										 .flags = PID_FLAG_ANTI_WINDUP},
							   .name = "current_q"},
	[PID_PROFILE_VELOCITY] = {.param = {.kp = 0.5f,
										.ki = 0.1f,
										.kd = 0.005f,
										.output_limit = 20.0f,
										.integral_limit = 5.0f,
										.output_filter_alpha = 1.0f,
										.flags = PID_FLAG_ANTI_WINDUP | PID_FLAG_DIFFERENTIAL_ON_MEASUREMENT},
							   .name = "velocity" },
	[PID_PROFILE_POSITION] = {.param = {.kp = 100.0f,
										.ki = 0.0f,
										.kd = 5.0f,
										.output_limit = 30.0f,
										.integral_limit = 0.0f,
										.output_filter_alpha = 1.0f,
										.flags = PID_FLAG_NONE},
							   .name = "position" },
	[PID_PROFILE_IMPEDANCE] = {.param = {.kp = 50.0f,
										 .ki = 0.0f,
										 .kd = 2.0f,
										 .output_limit = 10.0f,
										 .integral_limit = 0.0f,
										 .output_filter_alpha = 0.8f,
										 .flags = PID_FLAG_OUTPUT_FILTER},
							   .name = "impedance"},
	[PID_PROFILE_HOMING] = {.param = {.kp = 50.0f,
									  .ki = 0.0f,
									  .kd = 2.0f,
									  .output_limit = 10.0f,
									  .integral_limit = 0.0f,
									  .output_filter_alpha = 1.0f,
									  .flags = PID_FLAG_NONE},
							   .name = "homing"	},
	[PID_PROFILE_JOG] = {.param = {.kp = 0.3f,
								   .ki = 0.05f,
								   .kd = 0.002f,
								   .output_limit = 15.0f,
								   .integral_limit = 3.0f,
								   .output_filter_alpha = 1.0f,
								   .flags = PID_FLAG_ANTI_WINDUP},
							   .name = "jog"	  },
	[PID_PROFILE_TEST] = {
							   .param =
			{.kp = 0.0f, .ki = 0.0f, .kd = 0.0f, .output_limit = 20.0f, .integral_limit = 0.0f, .output_filter_alpha = 1.0f, .flags = PID_FLAG_NONE},
							   .name = "test"	  }
};

void pid_profile_init(const motor_param_t *motor_param)
{
	s_motor_param = motor_param;

	// 加载默认参数
	for (int i = 0; i < PID_PROFILE_MAX; i++)
	{
		memcpy(&s_pid_profiles[i], &s_default_profiles[i], sizeof(pid_profile_t));
	}

	// 从电机参数覆盖默认值
	pid_profile_load_from_motor_param(motor_param);
}

const pid_param_t *pid_profile_get(pid_profile_id_e id)
{
	if (id >= PID_PROFILE_MAX)
		return NULL;

	return &s_pid_profiles[id].param;
}

int pid_profile_set(pid_profile_id_e id, const pid_param_t *param)
{
	if (id >= PID_PROFILE_MAX || param == NULL)
		return -1;

	// 参数校验
	if (param->output_limit < 0.0f || param->integral_limit < 0.0f)
		return -1;

	if (param->output_filter_alpha < 0.0f || param->output_filter_alpha > 1.0f)
		return -1;

	memcpy(&s_pid_profiles[id].param, param, sizeof(pid_param_t));
	return 0;
}

void pid_profile_load_from_motor_param(const motor_param_t *p)
{
	// 电流环参数
	s_pid_profiles[PID_PROFILE_CURRENT_D].param.kp = motor_param_get_current_kp_d(p);
	s_pid_profiles[PID_PROFILE_CURRENT_D].param.ki = motor_param_get_current_ki_d(p);
	s_pid_profiles[PID_PROFILE_CURRENT_D].param.integral_limit = motor_param_get_current_integral_limit(p);
	s_pid_profiles[PID_PROFILE_CURRENT_D].param.output_limit = motor_param_get_rated_voltage(p);

	s_pid_profiles[PID_PROFILE_CURRENT_Q].param.kp = motor_param_get_current_kp_q(p);
	s_pid_profiles[PID_PROFILE_CURRENT_Q].param.ki = motor_param_get_current_ki_q(p);
	s_pid_profiles[PID_PROFILE_CURRENT_Q].param.integral_limit = motor_param_get_current_integral_limit(p);
	s_pid_profiles[PID_PROFILE_CURRENT_Q].param.output_limit = motor_param_get_rated_voltage(p);

	// 速度环参数
	s_pid_profiles[PID_PROFILE_VELOCITY].param.kp = motor_param_get_speed_kp(p);
	s_pid_profiles[PID_PROFILE_VELOCITY].param.ki = motor_param_get_speed_ki(p);
	s_pid_profiles[PID_PROFILE_VELOCITY].param.integral_limit = motor_param_get_speed_integral_limit(p);
	s_pid_profiles[PID_PROFILE_VELOCITY].param.output_limit = motor_param_get_peak_current(p);

	// 位置环参数
	s_pid_profiles[PID_PROFILE_POSITION].param.kp = motor_param_get_position_kp(p);
	s_pid_profiles[PID_PROFILE_POSITION].param.integral_limit = motor_param_get_position_integral_limit(p);
	s_pid_profiles[PID_PROFILE_POSITION].param.output_limit = motor_param_get_max_speed(p);
}

void pid_profile_save_to_motor_param(motor_param_t *p)
{
	// 电流环参数
	motor_param_set_current_kp_d(p, s_pid_profiles[PID_PROFILE_CURRENT_D].param.kp);
	motor_param_set_current_ki_d(p, s_pid_profiles[PID_PROFILE_CURRENT_D].param.ki);
	motor_param_set_current_integral_limit(p, s_pid_profiles[PID_PROFILE_CURRENT_D].param.integral_limit);

	motor_param_set_current_kp_q(p, s_pid_profiles[PID_PROFILE_CURRENT_Q].param.kp);
	motor_param_set_current_ki_q(p, s_pid_profiles[PID_PROFILE_CURRENT_Q].param.ki);

	// 速度环参数
	motor_param_set_speed_kp(p, s_pid_profiles[PID_PROFILE_VELOCITY].param.kp);
	motor_param_set_speed_ki(p, s_pid_profiles[PID_PROFILE_VELOCITY].param.ki);
	motor_param_set_speed_integral_limit(p, s_pid_profiles[PID_PROFILE_VELOCITY].param.integral_limit);

	// 位置环参数
	motor_param_set_position_kp(p, s_pid_profiles[PID_PROFILE_POSITION].param.kp);
	motor_param_set_position_integral_limit(p, s_pid_profiles[PID_PROFILE_POSITION].param.integral_limit);
}

void pid_profile_restore_default(pid_profile_id_e id)
{
	if (id >= PID_PROFILE_MAX)
		return;

	memcpy(&s_pid_profiles[id], &s_default_profiles[id], sizeof(pid_profile_t));
}

void pid_profile_init_state(pid_state_t *state)
{
	pid_core_init(state);
}

void pid_profile_reset_state(pid_state_t *state)
{
	pid_core_reset(state);
}

void pid_profile_preload(pid_state_t *state, pid_profile_id_e id, float output_now, float actual)
{
	const pid_param_t *param = pid_profile_get(id);

	pid_core_reset(state);

	if (param == NULL)
		return;

	// 将积分项预装载为期望输出，并限制在积分限幅内
	float integral = output_now;
	if (integral > param->integral_limit)
		integral = param->integral_limit;
	else if (integral < -param->integral_limit)
		integral = -param->integral_limit;

	state->integral = integral;
	state->prev_output = output_now;
	state->prev_actual = actual;
}

float pid_profile_calculate(pid_state_t *state, pid_profile_id_e id, float target, float actual, float dt)
{
	const pid_param_t *param = pid_profile_get(id);
	if (param == NULL)
		return 0.0f;

	return pid_core_calculate(state, param, target, actual, dt);
}

float pid_profile_calculate_with_ff(pid_state_t *state, pid_profile_id_e id, float target, float actual, float feedforward, float dt)
{
	const pid_param_t *param = pid_profile_get(id);
	if (param == NULL)
		return 0.0f;

	return pid_core_calculate_with_ff(state, param, target, actual, feedforward, dt);
}
