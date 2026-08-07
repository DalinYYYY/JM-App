#include "motor_pid_profile.h"
#include <string.h>
#include <math.h>

motor_pid_profile_t s_motor_pid_profiles[MOTOR_PID_PROFILE_MAX];
static const motor_param_t *s_motor_param;

/**
 * @brief 默认PID参数配置
 */
static const motor_pid_profile_t s_default_profiles[MOTOR_PID_PROFILE_MAX] = {
	[MOTOR_PID_PROFILE_CURRENT_D] = {.param = {.kp = 5.5f,
                                               .ki = 1500.0f,
                                               .kd = 0.0f,
                                               .output_limit = 24.0f,
                                               .integral_limit = 200.0f,
                                               .output_filter_alpha = 1.0f,
                                               .flags = PID_FLAG_ANTI_WINDUP},
                                     .name = "current_d"},
	[MOTOR_PID_PROFILE_CURRENT_Q] = {.param = {.kp = 5.5f,
                                               .ki = 1500.0f,
                                               .kd = 0.0f,
                                               .output_limit = 48.0f,
                                               .integral_limit = 10.0f,
                                               .output_filter_alpha = 1.0f,
                                               .flags = PID_FLAG_ANTI_WINDUP},
                                     .name = "current_q"},
	[MOTOR_PID_PROFILE_VELOCITY] = {.param = {.kp = 0.08f,
                                              .ki = 0.8f,
                                              .kd = 0.0f,
                                              .output_limit = 20.0f,
                                              .integral_limit = 200.0f,
                                              .output_filter_alpha = 1.0f,
                                              .flags = PID_FLAG_ANTI_WINDUP | PID_FLAG_DIFFERENTIAL_ON_MEASUREMENT},
                                     .name = "velocity" },
	[MOTOR_PID_PROFILE_POSITION] = {.param = {.kp = 10.0f,
                                              .ki = 0.0f,
                                              .kd = 0.0f,
                                              .output_limit = 30.0f,
                                              .integral_limit = 0.0f,
                                              .output_filter_alpha = 1.0f,
                                              .flags = PID_FLAG_NONE},
                                     .name = "position" },
	[MOTOR_PID_PROFILE_IMPEDANCE] = {.param = {.kp = 50.0f,
                                               .ki = 0.0f,
                                               .kd = 2.0f,
                                               .output_limit = 10.0f,
                                               .integral_limit = 0.0f,
                                               .output_filter_alpha = 0.8f,
                                               .flags = PID_FLAG_OUTPUT_FILTER},
                                     .name = "impedance"},
	[MOTOR_PID_PROFILE_HOMING] = {.param = {.kp = 50.0f,
                                            .ki = 0.0f,
                                            .kd = 2.0f,
                                            .output_limit = 10.0f,
                                            .integral_limit = 0.0f,
                                            .output_filter_alpha = 1.0f,
                                            .flags = PID_FLAG_NONE},
                                     .name = "homing"   },
	[MOTOR_PID_PROFILE_JOG] = {.param = {.kp = 0.3f,
                                         .ki = 0.05f,
                                         .kd = 0.002f,
                                         .output_limit = 15.0f,
                                         .integral_limit = 3.0f,
                                         .output_filter_alpha = 1.0f,
                                         .flags = PID_FLAG_ANTI_WINDUP},
                                     .name = "jog"      },
	[MOTOR_PID_PROFILE_TEST] = {
									 .param =
			{.kp = 0.0f, .ki = 0.0f, .kd = 0.0f, .output_limit = 20.0f, .integral_limit = 0.0f, .output_filter_alpha = 1.0f, .flags = PID_FLAG_NONE},
									 .name = "test"     }
};

void motor_pid_profile_init(const motor_param_t *motor_param)
{
	s_motor_param = motor_param;

	// 加载默认参数
	for (int i = 0; i < MOTOR_PID_PROFILE_MAX; i++)
	{
		memcpy(&s_motor_pid_profiles[i], &s_default_profiles[i], sizeof(motor_pid_profile_t));
	}

	// 从电机参数覆盖默认值
	motor_pid_profile_load_from_motor_param(motor_param);
}

const pid_param_t *motor_pid_profile_get(motor_pid_profile_id_e id)
{
	if (id >= MOTOR_PID_PROFILE_MAX)
		return NULL;

	return &s_motor_pid_profiles[id].param;
}

int motor_pid_profile_set(motor_pid_profile_id_e id, const pid_param_t *param)
{
	if (id >= MOTOR_PID_PROFILE_MAX || param == NULL)
		return -1;

	if (motor_pid_profile_validate(param, 0.0f) != 0)
		return -1;

	memcpy(&s_motor_pid_profiles[id].param, param, sizeof(pid_param_t));
	return 0;
}

int motor_pid_profile_validate(const pid_param_t *param, float output_limit_max)
{
	if (param == NULL || !isfinite(param->kp) || !isfinite(param->ki) ||
		!isfinite(param->kd) || !isfinite(param->output_limit) ||
		!isfinite(param->integral_limit) || !isfinite(param->output_filter_alpha))
		return -1;
	if (param->kp < 0.0f || param->ki < 0.0f || param->kd < 0.0f ||
		param->kp > 1000000.0f || param->ki > 10000000.0f || param->kd > 1000000.0f)
		return -1;
	if (param->output_limit <= 0.0f || param->integral_limit < 0.0f ||
		param->output_filter_alpha < 0.0f || param->output_filter_alpha > 1.0f)
		return -1;
	if ((param->flags & ~(uint32_t)(PID_FLAG_ANTI_WINDUP |
		PID_FLAG_DIFFERENTIAL_ON_MEASUREMENT | PID_FLAG_INCREMENTAL |
		PID_FLAG_OUTPUT_FILTER)) != 0u)
		return -1;
	if (output_limit_max > 0.0f && param->output_limit > output_limit_max)
		return -1;
	return 0;
}

static int profile_apply_field(pid_param_t *p, uint8_t type, const uint8_t *v)
{
	float fv;
	if (p == NULL || v == NULL) return -1;
	memcpy(&fv, v, sizeof(fv));
	switch (type)
	{
		case 1: p->kp = fv; break;
		case 2: p->ki = fv; break;
		case 3: p->kd = fv; break;
		case 4: p->output_limit = fv; break;
		case 5: p->integral_limit = fv; break;
		case 6: p->output_filter_alpha = fv; break;
		case 7: memcpy(&p->flags, v, 4); break;
		default: return -1;
	}
	return 0;
}

int motor_pid_profile_apply_batch(uint8_t id, const uint8_t *entries,
	                                  uint16_t entries_len, float output_limit_max)
{
	pid_param_t shadow;
	uint8_t count, i;
	if (id >= MOTOR_PID_PROFILE_MAX || entries == NULL || entries_len < 1u)
		return -1;
	count = entries[0];
	if (count == 0u || count > 7u || entries_len != (uint16_t)(1u + count * 5u))
		return -1;
	shadow = s_motor_pid_profiles[id].param;
	for (i = 0; i < count; i++)
	{
		const uint8_t *e = &entries[1u + i * 5u];
		if (profile_apply_field(&shadow, e[0], &e[1]) != 0)
			return -1;
	}
	if (motor_pid_profile_validate(&shadow, output_limit_max) != 0)
		return -1;
	memcpy(&s_motor_pid_profiles[id].param, &shadow, sizeof(shadow));
	return 0;
}

void motor_pid_profile_load_from_motor_param(const motor_param_t *p)
{
	// 电流环参数
	s_motor_pid_profiles[MOTOR_PID_PROFILE_CURRENT_D].param.kp = (p)->current_loop.current_kp_d;
	s_motor_pid_profiles[MOTOR_PID_PROFILE_CURRENT_D].param.ki = (p)->current_loop.current_ki_d;
	s_motor_pid_profiles[MOTOR_PID_PROFILE_CURRENT_D].param.integral_limit = (p)->current_loop.current_integral_limit;
	s_motor_pid_profiles[MOTOR_PID_PROFILE_CURRENT_D].param.output_limit = (p)->motor_base.rated_voltage;

	s_motor_pid_profiles[MOTOR_PID_PROFILE_CURRENT_Q].param.kp = (p)->current_loop.current_kp_q;
	s_motor_pid_profiles[MOTOR_PID_PROFILE_CURRENT_Q].param.ki = (p)->current_loop.current_ki_q;
	s_motor_pid_profiles[MOTOR_PID_PROFILE_CURRENT_Q].param.integral_limit = (p)->current_loop.current_integral_limit;
	s_motor_pid_profiles[MOTOR_PID_PROFILE_CURRENT_Q].param.output_limit = (p)->motor_base.rated_voltage;

	// 速度环参数
	s_motor_pid_profiles[MOTOR_PID_PROFILE_VELOCITY].param.kp = (p)->position_loop.speed_kp;
	s_motor_pid_profiles[MOTOR_PID_PROFILE_VELOCITY].param.ki = (p)->position_loop.speed_ki;
	s_motor_pid_profiles[MOTOR_PID_PROFILE_VELOCITY].param.integral_limit = (p)->position_loop.speed_integral_limit;
	s_motor_pid_profiles[MOTOR_PID_PROFILE_VELOCITY].param.output_limit = (p)->motor_base.peak_current;

	// 位置环参数
	s_motor_pid_profiles[MOTOR_PID_PROFILE_POSITION].param.kp = (p)->position_loop.position_kp;
	s_motor_pid_profiles[MOTOR_PID_PROFILE_POSITION].param.integral_limit = (p)->position_loop.position_integral_limit;
	s_motor_pid_profiles[MOTOR_PID_PROFILE_POSITION].param.output_limit = (p)->motor_base.max_speed;
}

void motor_pid_profile_save_to_motor_param(motor_param_t *p)
{
	// 电流环参数
	(p)->current_loop.current_kp_d = s_motor_pid_profiles[MOTOR_PID_PROFILE_CURRENT_D].param.kp;
	(p)->current_loop.current_ki_d = s_motor_pid_profiles[MOTOR_PID_PROFILE_CURRENT_D].param.ki;
	(p)->current_loop.current_integral_limit = s_motor_pid_profiles[MOTOR_PID_PROFILE_CURRENT_D].param.integral_limit;

	(p)->current_loop.current_kp_q = s_motor_pid_profiles[MOTOR_PID_PROFILE_CURRENT_Q].param.kp;
	(p)->current_loop.current_ki_q = s_motor_pid_profiles[MOTOR_PID_PROFILE_CURRENT_Q].param.ki;

	// 速度环参数
	(p)->position_loop.speed_kp = s_motor_pid_profiles[MOTOR_PID_PROFILE_VELOCITY].param.kp;
	(p)->position_loop.speed_ki = s_motor_pid_profiles[MOTOR_PID_PROFILE_VELOCITY].param.ki;
	(p)->position_loop.speed_integral_limit = s_motor_pid_profiles[MOTOR_PID_PROFILE_VELOCITY].param.integral_limit;

	// 位置环参数
	(p)->position_loop.position_kp = s_motor_pid_profiles[MOTOR_PID_PROFILE_POSITION].param.kp;
	(p)->position_loop.position_integral_limit = s_motor_pid_profiles[MOTOR_PID_PROFILE_POSITION].param.integral_limit;
}

void motor_pid_profile_restore_default(motor_pid_profile_id_e id)
{
	if (id >= MOTOR_PID_PROFILE_MAX)
		return;

	memcpy(&s_motor_pid_profiles[id], &s_default_profiles[id], sizeof(motor_pid_profile_t));
}

void motor_pid_profile_init_state(pid_state_t *state)
{
	pid_core_init(state);
}

void motor_pid_profile_reset_state(pid_state_t *state)
{
	pid_core_reset(state);
}

void motor_pid_profile_preload(pid_state_t *state, motor_pid_profile_id_e id, float output_now, float actual)
{
	const pid_param_t *param = motor_pid_profile_get(id);

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

float motor_pid_profile_calculate(pid_state_t *state, motor_pid_profile_id_e id, float target, float actual, float dt)
{
	const pid_param_t *param = motor_pid_profile_get(id);
	if (param == NULL)
		return 0.0f;

	return pid_core_calculate(state, param, target, actual, dt);
}

float motor_pid_profile_calculate_with_ff(pid_state_t *state, motor_pid_profile_id_e id, float target, float actual, float feedforward, float dt)
{
	const pid_param_t *param = motor_pid_profile_get(id);
	if (param == NULL)
		return 0.0f;

	return pid_core_calculate_with_ff(state, param, target, actual, feedforward, dt);
}

/* ==================== 0xA5/0xA6 单字段读写 API ==================== */

int motor_pid_profile_set_param(uint8_t id, uint8_t param_type, const uint8_t *value4)
{
	if (id >= MOTOR_PID_PROFILE_MAX || value4 == NULL)
		return -1;

	pid_param_t shadow = s_motor_pid_profiles[id].param;
	if (profile_apply_field(&shadow, param_type, value4) != 0 ||
		motor_pid_profile_validate(&shadow, 0.0f) != 0)
		return -1;
	memcpy(&s_motor_pid_profiles[id].param, &shadow, sizeof(shadow));
	return 0;
}

int motor_pid_profile_get_param(uint8_t id, uint8_t param_type, uint8_t *out_value4)
{
	if (id >= MOTOR_PID_PROFILE_MAX || out_value4 == NULL)
		return -1;

	const pid_param_t *p = &s_motor_pid_profiles[id].param;
	float fv;

	switch (param_type)
	{
		case 1: /* kp */
			fv = p->kp;
			memcpy(out_value4, &fv, 4);
			break;
		case 2: /* ki */
			fv = p->ki;
			memcpy(out_value4, &fv, 4);
			break;
		case 3: /* kd */
			fv = p->kd;
			memcpy(out_value4, &fv, 4);
			break;
		case 4: /* output_limit */
			fv = p->output_limit;
			memcpy(out_value4, &fv, 4);
			break;
		case 5: /* integral_limit */
			fv = p->integral_limit;
			memcpy(out_value4, &fv, 4);
			break;
		case 6: /* output_filter_alpha */
			fv = p->output_filter_alpha;
			memcpy(out_value4, &fv, 4);
			break;
		case 7: /* flags */
			memcpy(out_value4, &p->flags, 4);
			break;
		default:
			return -1;
	}
	return 0;
}
