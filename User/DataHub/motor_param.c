/**
 * @file    motor_param.c
 * @brief   关节电机配置参数API实现
 * @date    2026-06-12
 *
 * @warning 【自动生成文件，请勿手动修改】
 *          本文件由脚本 generate_config_header_v9.py 根据配置表自动生成，
 *          任何手动改动都会在下次运行脚本时被覆盖。
 *          如需修改参数定义，请编辑源 CSV 配置表后重新生成。
 */

#include "motor_param.h"
#include <stdio.h>
#include <string.h>
#include <errno.h>

/* 默认值常量 */
static const motor_param_t g_default_config =
	{
		.motor_instance =
			{
							 .motor_id = 0,
							 .motor_name = {0},
							 },
		.motor_base =
			{
							 .r = 0.1f,
							 .ld = 0.0001f,
							 .lq = 0.00012f,
							 .flux = 0.001f,
							 .kt = 0.1f,
							 .pole_pairs = 7,
							 .rated_current = 0.6f,
							 .peak_current = 1.50f,
							 .max_speed = 2000.0f,
							 .dead_time_ns = 500.0f,
							 .rated_voltage = 12.0f,
							 .rated_speed_rpm = 1000.0f,
							 .rated_torque = 2.0f,
							 .peak_torque = 6.0f,
							 .inertia = 1e-05f,
							 .ke = 0.01f,
							 .pwm_freq_hz = 10000,
							 .foc_freq_hz = 10000,
							 },
		.gearbox_param =
			{
							 .gear_ratio = 100.0f,
							 .gear_efficiency = 0.85f,
							 .output_torque_const = 10.0f,
							 .gear_backlash = 0.01f,
							 },
		.encoder_param =
			{
							 .enc_lines = 4000,
							 .enc_direction = 1,
							 .enc_offset = 43.77f,  /* deg */
							 .elec_angle_bias = 0.0f,
							 .pos_filter_alpha = 0.1f,
							 .enc_type = 0,
							 .enc_auto_calib = 1,
							 .speed_obs_gain = 100.0f,
							 },
		.position_limit =
			{
							 .multiturn_enable = 1,
							 .pos_min_limit = -12.566f,
							 .pos_max_limit = 12.566f,
							 .limit_sw_enable = 0,
							 },
		.homing_param =
			{
							 .homing_method = 0,
							 .homing_speed_fast = 5.0f,
							 .homing_speed_slow = 0.5f,
							 .homing_offset = 0.0f,
							 .homing_current = 2.0f,
							 },
		.current_loop =
			{
							 .current_kp_d = 0.5f,
							 .current_ki_d = 10.0f,
							 .current_kp_q = 0.5f,
							 .current_ki_q = 10.0f,
							 .current_integral_limit = 12.0f,
							 .decoupling_gain = 1.0f,
							 .deadtime_comp_v = 0.0f,
							 .pwm_max_duty = 0.9f,
							 .current_bandwidth_hz = 1000.0f,
							 .current_filter_alpha = 0.1f,
							 .d_feedforward_gain = 1.0f,
							 .q_feedforward_gain = 1.0f,
							 },
		.position_loop =
			{
							 .speed_kp = 0.1f,
							 .speed_ki = 1.0f,
							 .speed_integral_limit = 10.0f,
							 .velocity_ff_gain = 1.0f,
							 .accel_ff_gain = 0.0f,
							 .position_kp = 10.0f,
							 .position_integral_limit = 0.0f,
							 .friction_coulomb = 0.0f,
							 .friction_viscous = 0.0f,
							 .notch_freq_hz = 200.0f,
							 .notch_width_hz = 20.0f,
							 .notch_depth_db = 40.0f,
							 .notch_enable = 0,
							 .speed_bandwidth_hz = 100.0f,
							 .speed_filter_alpha = 0.05f,
							 .position_bandwidth_hz = 20.0f,
							 },
		.impedance_ctrl =
			{
							 .impedance_kp = 10.0f,
							 .impedance_kd = 0.1f,
							 .iq_max = 10.0f,
							 },
		.thermal_model =
			{
							 .thermal_resistance = 1.0f,
							 .thermal_time_const = 60.0f,
							 .derating_temp_start = 70.0f,
							 },
		.protection_param =
			{
							 .protect_over_current = 20.0f,
							 .protect_over_voltage = 58.0f,
							 .protect_under_voltage = 15.0f,
							 .protect_over_speed = 400.0f,
							 .protect_over_temp = 85.0f,
							 .protect_under_temp = -20.0f,
							 .protect_pos_error = 1000,
							 .protect_enable_mask = 4294967295,
							 },
};

int motor_param_init(motor_param_t *cfg)
{
	if (cfg == NULL)
		return -EINVAL;
	memcpy(cfg, &g_default_config, sizeof(motor_param_t));
	return 0;
}

int motor_param_validate(const motor_param_t *cfg)
{
	if (cfg == NULL)
		return -EINVAL;

	if (cfg->motor_instance.motor_id > (uint8_t)31)
		return 1;
	if (cfg->motor_base.r < 0.001f || cfg->motor_base.r > 10.0f)
		return 3;
	if (cfg->motor_base.ld < 1e-05f || cfg->motor_base.ld > 0.01f)
		return 4;
	if (cfg->motor_base.lq < 1e-05f || cfg->motor_base.lq > 0.01f)
		return 5;
	if (cfg->motor_base.flux < 0.0001f || cfg->motor_base.flux > 0.1f)
		return 6;
	if (cfg->motor_base.kt < 0.01f || cfg->motor_base.kt > 10.0f)
		return 7;
	if (cfg->motor_base.pole_pairs < (uint8_t)1 || cfg->motor_base.pole_pairs > (uint8_t)64)
		return 8;
	if (cfg->motor_base.rated_current < 0.1f || cfg->motor_base.rated_current > 100.0f)
		return 9;
	if (cfg->motor_base.peak_current < 0.1f || cfg->motor_base.peak_current > 200.0f)
		return 10;
	if (cfg->motor_base.max_speed < 10.0f || cfg->motor_base.max_speed > 1000.0f)
		return 11;
	if (cfg->motor_base.dead_time_ns < 100.0f || cfg->motor_base.dead_time_ns > 2000.0f)
		return 12;
	if (cfg->motor_base.rated_voltage < 12.0f || cfg->motor_base.rated_voltage > 100.0f)
		return 13;
	if (cfg->motor_base.rated_speed_rpm < 100.0f || cfg->motor_base.rated_speed_rpm > 20000.0f)
		return 14;
	if (cfg->motor_base.rated_torque < 0.1f || cfg->motor_base.rated_torque > 100.0f)
		return 15;
	if (cfg->motor_base.peak_torque < 0.1f || cfg->motor_base.peak_torque > 300.0f)
		return 16;
	if (cfg->motor_base.inertia < 1e-07f || cfg->motor_base.inertia > 0.01f)
		return 17;
	if (cfg->motor_base.ke < 0.001f || cfg->motor_base.ke > 1.0f)
		return 18;
	if (cfg->motor_base.pwm_freq_hz < (uint32_t)5000 || cfg->motor_base.pwm_freq_hz > (uint32_t)100000)
		return 19;
	if (cfg->motor_base.foc_freq_hz < (uint32_t)1000 || cfg->motor_base.foc_freq_hz > (uint32_t)100000)
		return 20;
	if (cfg->gearbox_param.gear_ratio < 1.0f || cfg->gearbox_param.gear_ratio > 1000.0f)
		return 21;
	if (cfg->gearbox_param.gear_efficiency < 0.1f || cfg->gearbox_param.gear_efficiency > 1.0f)
		return 22;
	if (cfg->gearbox_param.output_torque_const < 0.1f || cfg->gearbox_param.output_torque_const > 1000.0f)
		return 23;
	if (cfg->gearbox_param.gear_backlash < 0.0f || cfg->gearbox_param.gear_backlash > 0.5f)
		return 24;
	if (cfg->encoder_param.enc_lines < (uint32_t)100 || cfg->encoder_param.enc_lines > (uint32_t)1000000)
		return 25;
	if (cfg->encoder_param.enc_direction < (int8_t)-1 || cfg->encoder_param.enc_direction > (int8_t)1)
		return 26;
	if (cfg->encoder_param.enc_offset < -360.0f || cfg->encoder_param.enc_offset > 360.0f)
		return 27;
	if (cfg->encoder_param.elec_angle_bias < -3.1416f || cfg->encoder_param.elec_angle_bias > 3.1416f)
		return 28;
	if (cfg->encoder_param.pos_filter_alpha < 0.0f || cfg->encoder_param.pos_filter_alpha > 1.0f)
		return 29;
	if (cfg->encoder_param.enc_type > (uint8_t)10)
		return 30;
	if (cfg->encoder_param.enc_auto_calib > (uint8_t)1)
		return 31;
	if (cfg->encoder_param.speed_obs_gain < 1.0f || cfg->encoder_param.speed_obs_gain > 1000.0f)
		return 32;
	if (cfg->position_limit.multiturn_enable > (uint8_t)1)
		return 33;
	if (cfg->position_limit.pos_min_limit < -1000.0f || cfg->position_limit.pos_min_limit > 0.0f)
		return 34;
	if (cfg->position_limit.pos_max_limit < 0.0f || cfg->position_limit.pos_max_limit > 1000.0f)
		return 35;
	if (cfg->position_limit.limit_sw_enable > (uint8_t)1)
		return 36;
	if (cfg->homing_param.homing_method > (uint8_t)10)
		return 37;
	if (cfg->homing_param.homing_speed_fast < 0.1f || cfg->homing_param.homing_speed_fast > 50.0f)
		return 38;
	if (cfg->homing_param.homing_speed_slow < 0.01f || cfg->homing_param.homing_speed_slow > 5.0f)
		return 39;
	if (cfg->homing_param.homing_offset < -6.283f || cfg->homing_param.homing_offset > 6.283f)
		return 40;
	if (cfg->homing_param.homing_current < 0.1f || cfg->homing_param.homing_current > 20.0f)
		return 41;
	if (cfg->current_loop.current_kp_d < 0.0f || cfg->current_loop.current_kp_d > 100.0f)
		return 42;
	if (cfg->current_loop.current_ki_d < 0.0f || cfg->current_loop.current_ki_d > 1000.0f)
		return 43;
	if (cfg->current_loop.current_kp_q < 0.0f || cfg->current_loop.current_kp_q > 100.0f)
		return 44;
	if (cfg->current_loop.current_ki_q < 0.0f || cfg->current_loop.current_ki_q > 1000.0f)
		return 45;
	if (cfg->current_loop.current_integral_limit < 0.0f || cfg->current_loop.current_integral_limit > 100.0f)
		return 46;
	if (cfg->current_loop.decoupling_gain < 0.0f || cfg->current_loop.decoupling_gain > 1.0f)
		return 47;
	if (cfg->current_loop.deadtime_comp_v < 0.0f || cfg->current_loop.deadtime_comp_v > 5.0f)
		return 48;
	if (cfg->current_loop.pwm_max_duty < 0.5f || cfg->current_loop.pwm_max_duty > 0.95f)
		return 49;
	if (cfg->current_loop.current_bandwidth_hz < 100.0f || cfg->current_loop.current_bandwidth_hz > 5000.0f)
		return 50;
	if (cfg->current_loop.current_filter_alpha < 0.0f || cfg->current_loop.current_filter_alpha > 1.0f)
		return 51;
	if (cfg->current_loop.d_feedforward_gain < 0.0f || cfg->current_loop.d_feedforward_gain > 2.0f)
		return 52;
	if (cfg->current_loop.q_feedforward_gain < 0.0f || cfg->current_loop.q_feedforward_gain > 2.0f)
		return 53;
	if (cfg->position_loop.speed_kp < 0.0f || cfg->position_loop.speed_kp > 100.0f)
		return 54;
	if (cfg->position_loop.speed_ki < 0.0f || cfg->position_loop.speed_ki > 1000.0f)
		return 55;
	if (cfg->position_loop.speed_integral_limit < 0.0f || cfg->position_loop.speed_integral_limit > 100.0f)
		return 56;
	if (cfg->position_loop.velocity_ff_gain < 0.0f || cfg->position_loop.velocity_ff_gain > 1.0f)
		return 57;
	if (cfg->position_loop.accel_ff_gain < 0.0f || cfg->position_loop.accel_ff_gain > 1.0f)
		return 58;
	if (cfg->position_loop.position_kp < 0.0f || cfg->position_loop.position_kp > 1000.0f)
		return 59;
	if (cfg->position_loop.position_integral_limit < 0.0f || cfg->position_loop.position_integral_limit > 100.0f)
		return 60;
	if (cfg->position_loop.friction_coulomb < 0.0f || cfg->position_loop.friction_coulomb > 100.0f)
		return 61;
	if (cfg->position_loop.friction_viscous < 0.0f || cfg->position_loop.friction_viscous > 10.0f)
		return 62;
	if (cfg->position_loop.notch_freq_hz < 10.0f || cfg->position_loop.notch_freq_hz > 1000.0f)
		return 63;
	if (cfg->position_loop.notch_width_hz < 1.0f || cfg->position_loop.notch_width_hz > 100.0f)
		return 64;
	if (cfg->position_loop.notch_depth_db < 10.0f || cfg->position_loop.notch_depth_db > 100.0f)
		return 65;
	if (cfg->position_loop.notch_enable > (uint8_t)1)
		return 66;
	if (cfg->position_loop.speed_bandwidth_hz < 10.0f || cfg->position_loop.speed_bandwidth_hz > 1000.0f)
		return 67;
	if (cfg->position_loop.speed_filter_alpha < 0.0f || cfg->position_loop.speed_filter_alpha > 1.0f)
		return 68;
	if (cfg->position_loop.position_bandwidth_hz < 1.0f || cfg->position_loop.position_bandwidth_hz > 200.0f)
		return 69;
	if (cfg->impedance_ctrl.impedance_kp < 0.0f || cfg->impedance_ctrl.impedance_kp > 1000.0f)
		return 70;
	if (cfg->impedance_ctrl.impedance_kd < 0.0f || cfg->impedance_ctrl.impedance_kd > 100.0f)
		return 71;
	if (cfg->impedance_ctrl.iq_max < 0.0f || cfg->impedance_ctrl.iq_max > 100.0f)
		return 72;
	if (cfg->thermal_model.thermal_resistance < 0.1f || cfg->thermal_model.thermal_resistance > 10.0f)
		return 73;
	if (cfg->thermal_model.thermal_time_const < 1.0f || cfg->thermal_model.thermal_time_const > 600.0f)
		return 74;
	if (cfg->thermal_model.derating_temp_start < 40.0f || cfg->thermal_model.derating_temp_start > 100.0f)
		return 75;
	if (cfg->protection_param.protect_over_current < 1.0f || cfg->protection_param.protect_over_current > 200.0f)
		return 76;
	if (cfg->protection_param.protect_over_voltage < 20.0f || cfg->protection_param.protect_over_voltage > 100.0f)
		return 77;
	if (cfg->protection_param.protect_under_voltage < 5.0f || cfg->protection_param.protect_under_voltage > 30.0f)
		return 78;
	if (cfg->protection_param.protect_over_speed < 10.0f || cfg->protection_param.protect_over_speed > 2000.0f)
		return 79;
	if (cfg->protection_param.protect_over_temp < 50.0f || cfg->protection_param.protect_over_temp > 120.0f)
		return 80;
	if (cfg->protection_param.protect_under_temp < -40.0f || cfg->protection_param.protect_under_temp > 0.0f)
		return 81;
	if (cfg->protection_param.protect_pos_error < (int32_t)100 || cfg->protection_param.protect_pos_error > (int32_t)100000)
		return 82;
	if (cfg->protection_param.protect_enable_mask > (uint32_t)4294967295)
		return 83;

	return 0;
}

void motor_param_print(const motor_param_t *cfg)
{
	if (cfg == NULL)
		return;
	printf("========== Motor Config ==========\n");

	printf("\n--- Motor Instance ---\n");
	printf("motor_id: %u\n", (unsigned)cfg->motor_instance.motor_id);
	printf("motor_name: %s\n", cfg->motor_instance.motor_name);

	printf("\n--- Motor Base Parameters ---\n");
	printf("r: %f ohm\n", cfg->motor_base.r);
	printf("ld: %f H\n", cfg->motor_base.ld);
	printf("lq: %f H\n", cfg->motor_base.lq);
	printf("flux: %f Wb\n", cfg->motor_base.flux);
	printf("kt: %f Nm/A\n", cfg->motor_base.kt);
	printf("pole_pairs: %u\n", (unsigned)cfg->motor_base.pole_pairs);
	printf("rated_current: %f A\n", cfg->motor_base.rated_current);
	printf("peak_current: %f A\n", cfg->motor_base.peak_current);
	printf("max_speed: %f rad/s\n", cfg->motor_base.max_speed);
	printf("dead_time_ns: %f ns\n", cfg->motor_base.dead_time_ns);
	printf("rated_voltage: %f V\n", cfg->motor_base.rated_voltage);
	printf("rated_speed_rpm: %f rpm\n", cfg->motor_base.rated_speed_rpm);
	printf("rated_torque: %f Nm\n", cfg->motor_base.rated_torque);
	printf("peak_torque: %f Nm\n", cfg->motor_base.peak_torque);
	printf("inertia: %f kg*m2\n", cfg->motor_base.inertia);
	printf("ke: %f V/(rad/s)\n", cfg->motor_base.ke);
	printf("pwm_freq_hz: %u Hz\n", (unsigned)cfg->motor_base.pwm_freq_hz);
	printf("foc_freq_hz: %u Hz\n", (unsigned)cfg->motor_base.foc_freq_hz);

	printf("\n--- Gearbox Parameters ---\n");
	printf("gear_ratio: %f\n", cfg->gearbox_param.gear_ratio);
	printf("gear_efficiency: %f\n", cfg->gearbox_param.gear_efficiency);
	printf("output_torque_const: %f Nm/A\n", cfg->gearbox_param.output_torque_const);
	printf("gear_backlash: %f rad\n", cfg->gearbox_param.gear_backlash);

	printf("\n--- Encoder Parameters ---\n");
	printf("enc_lines: %u CPR\n", (unsigned)cfg->encoder_param.enc_lines);
	printf("enc_direction: %d\n", (int)cfg->encoder_param.enc_direction);
	printf("enc_offset: %.4f deg\n", cfg->encoder_param.enc_offset);
	printf("elec_angle_bias: %f rad\n", cfg->encoder_param.elec_angle_bias);
	printf("pos_filter_alpha: %f\n", cfg->encoder_param.pos_filter_alpha);
	printf("enc_type: %u\n", (unsigned)cfg->encoder_param.enc_type);
	printf("enc_auto_calib: %u\n", (unsigned)cfg->encoder_param.enc_auto_calib);
	printf("speed_obs_gain: %f\n", cfg->encoder_param.speed_obs_gain);

	printf("\n--- Position Limit Config ---\n");
	printf("multiturn_enable: %u\n", (unsigned)cfg->position_limit.multiturn_enable);
	printf("pos_min_limit: %f rad\n", cfg->position_limit.pos_min_limit);
	printf("pos_max_limit: %f rad\n", cfg->position_limit.pos_max_limit);
	printf("limit_sw_enable: %u\n", (unsigned)cfg->position_limit.limit_sw_enable);

	printf("\n--- Homing Config ---\n");
	printf("homing_method: %u\n", (unsigned)cfg->homing_param.homing_method);
	printf("homing_speed_fast: %f rad/s\n", cfg->homing_param.homing_speed_fast);
	printf("homing_speed_slow: %f rad/s\n", cfg->homing_param.homing_speed_slow);
	printf("homing_offset: %f rad\n", cfg->homing_param.homing_offset);
	printf("homing_current: %f A\n", cfg->homing_param.homing_current);

	printf("\n--- Current Loop Control ---\n");
	printf("current_kp_d: %f V/A\n", cfg->current_loop.current_kp_d);
	printf("current_ki_d: %f V/(A*s)\n", cfg->current_loop.current_ki_d);
	printf("current_kp_q: %f V/A\n", cfg->current_loop.current_kp_q);
	printf("current_ki_q: %f V/(A*s)\n", cfg->current_loop.current_ki_q);
	printf("current_integral_limit: %f V\n", cfg->current_loop.current_integral_limit);
	printf("decoupling_gain: %f\n", cfg->current_loop.decoupling_gain);
	printf("deadtime_comp_v: %f V\n", cfg->current_loop.deadtime_comp_v);
	printf("pwm_max_duty: %f\n", cfg->current_loop.pwm_max_duty);
	printf("current_bandwidth_hz: %f Hz\n", cfg->current_loop.current_bandwidth_hz);
	printf("current_filter_alpha: %f\n", cfg->current_loop.current_filter_alpha);
	printf("d_feedforward_gain: %f\n", cfg->current_loop.d_feedforward_gain);
	printf("q_feedforward_gain: %f\n", cfg->current_loop.q_feedforward_gain);

	printf("\n--- Position/Velocity Loop ---\n");
	printf("speed_kp: %f A/(rad/s)\n", cfg->position_loop.speed_kp);
	printf("speed_ki: %f A/rad\n", cfg->position_loop.speed_ki);
	printf("speed_integral_limit: %f A\n", cfg->position_loop.speed_integral_limit);
	printf("velocity_ff_gain: %f\n", cfg->position_loop.velocity_ff_gain);
	printf("accel_ff_gain: %f\n", cfg->position_loop.accel_ff_gain);
	printf("position_kp: %f Hz\n", cfg->position_loop.position_kp);
	printf("position_integral_limit: %f rad\n", cfg->position_loop.position_integral_limit);
	printf("friction_coulomb: %f Nm\n", cfg->position_loop.friction_coulomb);
	printf("friction_viscous: %f Nm/(rad/s)\n", cfg->position_loop.friction_viscous);
	printf("notch_freq_hz: %f Hz\n", cfg->position_loop.notch_freq_hz);
	printf("notch_width_hz: %f Hz\n", cfg->position_loop.notch_width_hz);
	printf("notch_depth_db: %f dB\n", cfg->position_loop.notch_depth_db);
	printf("notch_enable: %u\n", (unsigned)cfg->position_loop.notch_enable);
	printf("speed_bandwidth_hz: %f Hz\n", cfg->position_loop.speed_bandwidth_hz);
	printf("speed_filter_alpha: %f\n", cfg->position_loop.speed_filter_alpha);
	printf("position_bandwidth_hz: %f Hz\n", cfg->position_loop.position_bandwidth_hz);

	printf("\n--- Impedance Control ---\n");
	printf("impedance_kp: %f Nm/rad\n", cfg->impedance_ctrl.impedance_kp);
	printf("impedance_kd: %f Nm/(rad/s)\n", cfg->impedance_ctrl.impedance_kd);
	printf("iq_max: %f A\n", cfg->impedance_ctrl.iq_max);

	printf("\n--- Thermal Model ---\n");
	printf("thermal_resistance: %f K/W\n", cfg->thermal_model.thermal_resistance);
	printf("thermal_time_const: %f s\n", cfg->thermal_model.thermal_time_const);
	printf("derating_temp_start: %f C\n", cfg->thermal_model.derating_temp_start);

	printf("\n--- Protection Config ---\n");
	printf("protect_over_current: %f A\n", cfg->protection_param.protect_over_current);
	printf("protect_over_voltage: %f V\n", cfg->protection_param.protect_over_voltage);
	printf("protect_under_voltage: %f V\n", cfg->protection_param.protect_under_voltage);
	printf("protect_over_speed: %f rad/s\n", cfg->protection_param.protect_over_speed);
	printf("protect_over_temp: %f C\n", cfg->protection_param.protect_over_temp);
	printf("protect_under_temp: %f C\n", cfg->protection_param.protect_under_temp);
	printf("protect_pos_error: %d counts\n", (int)cfg->protection_param.protect_pos_error);
	printf("protect_enable_mask: %u\n", (unsigned)cfg->protection_param.protect_enable_mask);

	printf("\n==================================\n");
}

uint8_t motor_param_get_motor_id(const motor_param_t *cfg)
{
	return cfg->motor_instance.motor_id;
}

int motor_param_set_motor_id(motor_param_t *cfg, uint8_t value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value > (uint8_t)31)
		return -EINVAL;

	cfg->motor_instance.motor_id = value;
	return 0;
}

const char *motor_param_get_motor_name(const motor_param_t *cfg)
{
	return cfg->motor_instance.motor_name;
}

int motor_param_set_motor_name(motor_param_t *cfg, const char *value)
{
	if (cfg == NULL || value == NULL)
		return -EINVAL;
	strncpy(cfg->motor_instance.motor_name, value, sizeof(cfg->motor_instance.motor_name) - 1);
	cfg->motor_instance.motor_name[sizeof(cfg->motor_instance.motor_name) - 1] = '\0';
	return 0;
}

float motor_param_get_r(const motor_param_t *cfg)
{
	return cfg->motor_base.r;
}

int motor_param_set_r(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 0.001f || value > 10.0f)
		return -EINVAL;

	cfg->motor_base.r = value;
	return 0;
}

float motor_param_get_ld(const motor_param_t *cfg)
{
	return cfg->motor_base.ld;
}

int motor_param_set_ld(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 1e-05f || value > 0.01f)
		return -EINVAL;

	cfg->motor_base.ld = value;
	return 0;
}

float motor_param_get_lq(const motor_param_t *cfg)
{
	return cfg->motor_base.lq;
}

int motor_param_set_lq(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 1e-05f || value > 0.01f)
		return -EINVAL;

	cfg->motor_base.lq = value;
	return 0;
}

float motor_param_get_flux(const motor_param_t *cfg)
{
	return cfg->motor_base.flux;
}

int motor_param_set_flux(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 0.0001f || value > 0.1f)
		return -EINVAL;

	cfg->motor_base.flux = value;
	return 0;
}

float motor_param_get_kt(const motor_param_t *cfg)
{
	return cfg->motor_base.kt;
}

int motor_param_set_kt(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 0.01f || value > 10.0f)
		return -EINVAL;

	cfg->motor_base.kt = value;
	return 0;
}

uint8_t motor_param_get_pole_pairs(const motor_param_t *cfg)
{
	return cfg->motor_base.pole_pairs;
}

int motor_param_set_pole_pairs(motor_param_t *cfg, uint8_t value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < (uint8_t)1 || value > (uint8_t)64)
		return -EINVAL;

	cfg->motor_base.pole_pairs = value;
	return 0;
}

float motor_param_get_rated_current(const motor_param_t *cfg)
{
	return cfg->motor_base.rated_current;
}

int motor_param_set_rated_current(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 0.1f || value > 100.0f)
		return -EINVAL;

	cfg->motor_base.rated_current = value;
	return 0;
}

float motor_param_get_peak_current(const motor_param_t *cfg)
{
	return cfg->motor_base.peak_current;
}

int motor_param_set_peak_current(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 0.1f || value > 200.0f)
		return -EINVAL;

	cfg->motor_base.peak_current = value;
	return 0;
}

float motor_param_get_max_speed(const motor_param_t *cfg)
{
	return cfg->motor_base.max_speed;
}

int motor_param_set_max_speed(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 10.0f || value > 1000.0f)
		return -EINVAL;

	cfg->motor_base.max_speed = value;
	return 0;
}

float motor_param_get_dead_time_ns(const motor_param_t *cfg)
{
	return cfg->motor_base.dead_time_ns;
}

int motor_param_set_dead_time_ns(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 100.0f || value > 2000.0f)
		return -EINVAL;

	cfg->motor_base.dead_time_ns = value;
	return 0;
}

float motor_param_get_rated_voltage(const motor_param_t *cfg)
{
	return cfg->motor_base.rated_voltage;
}

int motor_param_set_rated_voltage(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 12.0f || value > 100.0f)
		return -EINVAL;

	cfg->motor_base.rated_voltage = value;
	return 0;
}

float motor_param_get_rated_speed_rpm(const motor_param_t *cfg)
{
	return cfg->motor_base.rated_speed_rpm;
}

int motor_param_set_rated_speed_rpm(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 100.0f || value > 20000.0f)
		return -EINVAL;

	cfg->motor_base.rated_speed_rpm = value;
	return 0;
}

float motor_param_get_rated_torque(const motor_param_t *cfg)
{
	return cfg->motor_base.rated_torque;
}

int motor_param_set_rated_torque(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 0.1f || value > 100.0f)
		return -EINVAL;

	cfg->motor_base.rated_torque = value;
	return 0;
}

float motor_param_get_peak_torque(const motor_param_t *cfg)
{
	return cfg->motor_base.peak_torque;
}

int motor_param_set_peak_torque(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 0.1f || value > 300.0f)
		return -EINVAL;

	cfg->motor_base.peak_torque = value;
	return 0;
}

float motor_param_get_inertia(const motor_param_t *cfg)
{
	return cfg->motor_base.inertia;
}

int motor_param_set_inertia(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 1e-07f || value > 0.01f)
		return -EINVAL;

	cfg->motor_base.inertia = value;
	return 0;
}

float motor_param_get_ke(const motor_param_t *cfg)
{
	return cfg->motor_base.ke;
}

int motor_param_set_ke(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 0.001f || value > 1.0f)
		return -EINVAL;

	cfg->motor_base.ke = value;
	return 0;
}

uint32_t motor_param_get_pwm_freq_hz(const motor_param_t *cfg)
{
	return cfg->motor_base.pwm_freq_hz;
}

int motor_param_set_pwm_freq_hz(motor_param_t *cfg, uint32_t value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < (uint32_t)5000 || value > (uint32_t)100000)
		return -EINVAL;

	cfg->motor_base.pwm_freq_hz = value;
	return 0;
}

uint32_t motor_param_get_foc_freq_hz(const motor_param_t *cfg)
{
	return cfg->motor_base.foc_freq_hz;
}

int motor_param_set_foc_freq_hz(motor_param_t *cfg, uint32_t value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < (uint32_t)1000 || value > (uint32_t)100000)
		return -EINVAL;

	cfg->motor_base.foc_freq_hz = value;
	return 0;
}

float motor_param_get_gear_ratio(const motor_param_t *cfg)
{
	return cfg->gearbox_param.gear_ratio;
}

int motor_param_set_gear_ratio(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 1.0f || value > 1000.0f)
		return -EINVAL;

	cfg->gearbox_param.gear_ratio = value;
	return 0;
}

float motor_param_get_gear_efficiency(const motor_param_t *cfg)
{
	return cfg->gearbox_param.gear_efficiency;
}

int motor_param_set_gear_efficiency(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 0.1f || value > 1.0f)
		return -EINVAL;

	cfg->gearbox_param.gear_efficiency = value;
	return 0;
}

float motor_param_get_output_torque_const(const motor_param_t *cfg)
{
	return cfg->gearbox_param.output_torque_const;
}

int motor_param_set_output_torque_const(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 0.1f || value > 1000.0f)
		return -EINVAL;

	cfg->gearbox_param.output_torque_const = value;
	return 0;
}

float motor_param_get_gear_backlash(const motor_param_t *cfg)
{
	return cfg->gearbox_param.gear_backlash;
}

int motor_param_set_gear_backlash(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 0.0f || value > 0.5f)
		return -EINVAL;

	cfg->gearbox_param.gear_backlash = value;
	return 0;
}

uint32_t motor_param_get_enc_lines(const motor_param_t *cfg)
{
	return cfg->encoder_param.enc_lines;
}

int motor_param_set_enc_lines(motor_param_t *cfg, uint32_t value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < (uint32_t)100 || value > (uint32_t)1000000)
		return -EINVAL;

	cfg->encoder_param.enc_lines = value;
	return 0;
}

int8_t motor_param_get_enc_direction(const motor_param_t *cfg)
{
	return cfg->encoder_param.enc_direction;
}

int motor_param_set_enc_direction(motor_param_t *cfg, int8_t value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < (int8_t)-1 || value > (int8_t)1)
		return -EINVAL;

	cfg->encoder_param.enc_direction = value;
	return 0;
}

float motor_param_get_enc_offset(const motor_param_t *cfg)
{
	return cfg->encoder_param.enc_offset;
}

int motor_param_set_enc_offset(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < -360.0f || value > 360.0f)
		return -EINVAL;

	cfg->encoder_param.enc_offset = value;
	return 0;
}

float motor_param_get_elec_angle_bias(const motor_param_t *cfg)
{
	return cfg->encoder_param.elec_angle_bias;
}

int motor_param_set_elec_angle_bias(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < -3.1416f || value > 3.1416f)
		return -EINVAL;

	cfg->encoder_param.elec_angle_bias = value;
	return 0;
}

float motor_param_get_pos_filter_alpha(const motor_param_t *cfg)
{
	return cfg->encoder_param.pos_filter_alpha;
}

int motor_param_set_pos_filter_alpha(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 0.0f || value > 1.0f)
		return -EINVAL;

	cfg->encoder_param.pos_filter_alpha = value;
	return 0;
}

uint8_t motor_param_get_enc_type(const motor_param_t *cfg)
{
	return cfg->encoder_param.enc_type;
}

int motor_param_set_enc_type(motor_param_t *cfg, uint8_t value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value > (uint8_t)10)
		return -EINVAL;

	cfg->encoder_param.enc_type = value;
	return 0;
}

uint8_t motor_param_get_enc_auto_calib(const motor_param_t *cfg)
{
	return cfg->encoder_param.enc_auto_calib;
}

int motor_param_set_enc_auto_calib(motor_param_t *cfg, uint8_t value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value > (uint8_t)1)
		return -EINVAL;

	cfg->encoder_param.enc_auto_calib = value;
	return 0;
}

float motor_param_get_speed_obs_gain(const motor_param_t *cfg)
{
	return cfg->encoder_param.speed_obs_gain;
}

int motor_param_set_speed_obs_gain(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 1.0f || value > 1000.0f)
		return -EINVAL;

	cfg->encoder_param.speed_obs_gain = value;
	return 0;
}

uint8_t motor_param_get_multiturn_enable(const motor_param_t *cfg)
{
	return cfg->position_limit.multiturn_enable;
}

int motor_param_set_multiturn_enable(motor_param_t *cfg, uint8_t value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value > (uint8_t)1)
		return -EINVAL;

	cfg->position_limit.multiturn_enable = value;
	return 0;
}

float motor_param_get_pos_min_limit(const motor_param_t *cfg)
{
	return cfg->position_limit.pos_min_limit;
}

int motor_param_set_pos_min_limit(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < -1000.0f || value > 0.0f)
		return -EINVAL;

	cfg->position_limit.pos_min_limit = value;
	return 0;
}

float motor_param_get_pos_max_limit(const motor_param_t *cfg)
{
	return cfg->position_limit.pos_max_limit;
}

int motor_param_set_pos_max_limit(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 0.0f || value > 1000.0f)
		return -EINVAL;

	cfg->position_limit.pos_max_limit = value;
	return 0;
}

uint8_t motor_param_get_limit_sw_enable(const motor_param_t *cfg)
{
	return cfg->position_limit.limit_sw_enable;
}

int motor_param_set_limit_sw_enable(motor_param_t *cfg, uint8_t value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value > (uint8_t)1)
		return -EINVAL;

	cfg->position_limit.limit_sw_enable = value;
	return 0;
}

uint8_t motor_param_get_homing_method(const motor_param_t *cfg)
{
	return cfg->homing_param.homing_method;
}

int motor_param_set_homing_method(motor_param_t *cfg, uint8_t value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value > (uint8_t)10)
		return -EINVAL;

	cfg->homing_param.homing_method = value;
	return 0;
}

float motor_param_get_homing_speed_fast(const motor_param_t *cfg)
{
	return cfg->homing_param.homing_speed_fast;
}

int motor_param_set_homing_speed_fast(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 0.1f || value > 50.0f)
		return -EINVAL;

	cfg->homing_param.homing_speed_fast = value;
	return 0;
}

float motor_param_get_homing_speed_slow(const motor_param_t *cfg)
{
	return cfg->homing_param.homing_speed_slow;
}

int motor_param_set_homing_speed_slow(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 0.01f || value > 5.0f)
		return -EINVAL;

	cfg->homing_param.homing_speed_slow = value;
	return 0;
}

float motor_param_get_homing_offset(const motor_param_t *cfg)
{
	return cfg->homing_param.homing_offset;
}

int motor_param_set_homing_offset(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < -6.283f || value > 6.283f)
		return -EINVAL;

	cfg->homing_param.homing_offset = value;
	return 0;
}

float motor_param_get_homing_current(const motor_param_t *cfg)
{
	return cfg->homing_param.homing_current;
}

int motor_param_set_homing_current(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 0.1f || value > 20.0f)
		return -EINVAL;

	cfg->homing_param.homing_current = value;
	return 0;
}

float motor_param_get_current_kp_d(const motor_param_t *cfg)
{
	return cfg->current_loop.current_kp_d;
}

int motor_param_set_current_kp_d(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 0.0f || value > 100.0f)
		return -EINVAL;

	cfg->current_loop.current_kp_d = value;
	return 0;
}

float motor_param_get_current_ki_d(const motor_param_t *cfg)
{
	return cfg->current_loop.current_ki_d;
}

int motor_param_set_current_ki_d(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 0.0f || value > 1000.0f)
		return -EINVAL;

	cfg->current_loop.current_ki_d = value;
	return 0;
}

float motor_param_get_current_kp_q(const motor_param_t *cfg)
{
	return cfg->current_loop.current_kp_q;
}

int motor_param_set_current_kp_q(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 0.0f || value > 100.0f)
		return -EINVAL;

	cfg->current_loop.current_kp_q = value;
	return 0;
}

float motor_param_get_current_ki_q(const motor_param_t *cfg)
{
	return cfg->current_loop.current_ki_q;
}

int motor_param_set_current_ki_q(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 0.0f || value > 1000.0f)
		return -EINVAL;

	cfg->current_loop.current_ki_q = value;
	return 0;
}

float motor_param_get_current_integral_limit(const motor_param_t *cfg)
{
	return cfg->current_loop.current_integral_limit;
}

int motor_param_set_current_integral_limit(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 0.0f || value > 100.0f)
		return -EINVAL;

	cfg->current_loop.current_integral_limit = value;
	return 0;
}

float motor_param_get_decoupling_gain(const motor_param_t *cfg)
{
	return cfg->current_loop.decoupling_gain;
}

int motor_param_set_decoupling_gain(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 0.0f || value > 1.0f)
		return -EINVAL;

	cfg->current_loop.decoupling_gain = value;
	return 0;
}

float motor_param_get_deadtime_comp_v(const motor_param_t *cfg)
{
	return cfg->current_loop.deadtime_comp_v;
}

int motor_param_set_deadtime_comp_v(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 0.0f || value > 5.0f)
		return -EINVAL;

	cfg->current_loop.deadtime_comp_v = value;
	return 0;
}

float motor_param_get_pwm_max_duty(const motor_param_t *cfg)
{
	return cfg->current_loop.pwm_max_duty;
}

int motor_param_set_pwm_max_duty(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 0.5f || value > 0.95f)
		return -EINVAL;

	cfg->current_loop.pwm_max_duty = value;
	return 0;
}

float motor_param_get_current_bandwidth_hz(const motor_param_t *cfg)
{
	return cfg->current_loop.current_bandwidth_hz;
}

int motor_param_set_current_bandwidth_hz(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 100.0f || value > 5000.0f)
		return -EINVAL;

	cfg->current_loop.current_bandwidth_hz = value;
	return 0;
}

float motor_param_get_current_filter_alpha(const motor_param_t *cfg)
{
	return cfg->current_loop.current_filter_alpha;
}

int motor_param_set_current_filter_alpha(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 0.0f || value > 1.0f)
		return -EINVAL;

	cfg->current_loop.current_filter_alpha = value;
	return 0;
}

float motor_param_get_d_feedforward_gain(const motor_param_t *cfg)
{
	return cfg->current_loop.d_feedforward_gain;
}

int motor_param_set_d_feedforward_gain(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 0.0f || value > 2.0f)
		return -EINVAL;

	cfg->current_loop.d_feedforward_gain = value;
	return 0;
}

float motor_param_get_q_feedforward_gain(const motor_param_t *cfg)
{
	return cfg->current_loop.q_feedforward_gain;
}

int motor_param_set_q_feedforward_gain(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 0.0f || value > 2.0f)
		return -EINVAL;

	cfg->current_loop.q_feedforward_gain = value;
	return 0;
}

float motor_param_get_speed_kp(const motor_param_t *cfg)
{
	return cfg->position_loop.speed_kp;
}

int motor_param_set_speed_kp(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 0.0f || value > 100.0f)
		return -EINVAL;

	cfg->position_loop.speed_kp = value;
	return 0;
}

float motor_param_get_speed_ki(const motor_param_t *cfg)
{
	return cfg->position_loop.speed_ki;
}

int motor_param_set_speed_ki(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 0.0f || value > 1000.0f)
		return -EINVAL;

	cfg->position_loop.speed_ki = value;
	return 0;
}

float motor_param_get_speed_integral_limit(const motor_param_t *cfg)
{
	return cfg->position_loop.speed_integral_limit;
}

int motor_param_set_speed_integral_limit(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 0.0f || value > 100.0f)
		return -EINVAL;

	cfg->position_loop.speed_integral_limit = value;
	return 0;
}

float motor_param_get_velocity_ff_gain(const motor_param_t *cfg)
{
	return cfg->position_loop.velocity_ff_gain;
}

int motor_param_set_velocity_ff_gain(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 0.0f || value > 1.0f)
		return -EINVAL;

	cfg->position_loop.velocity_ff_gain = value;
	return 0;
}

float motor_param_get_accel_ff_gain(const motor_param_t *cfg)
{
	return cfg->position_loop.accel_ff_gain;
}

int motor_param_set_accel_ff_gain(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 0.0f || value > 1.0f)
		return -EINVAL;

	cfg->position_loop.accel_ff_gain = value;
	return 0;
}

float motor_param_get_position_kp(const motor_param_t *cfg)
{
	return cfg->position_loop.position_kp;
}

int motor_param_set_position_kp(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 0.0f || value > 1000.0f)
		return -EINVAL;

	cfg->position_loop.position_kp = value;
	return 0;
}

float motor_param_get_position_integral_limit(const motor_param_t *cfg)
{
	return cfg->position_loop.position_integral_limit;
}

int motor_param_set_position_integral_limit(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 0.0f || value > 100.0f)
		return -EINVAL;

	cfg->position_loop.position_integral_limit = value;
	return 0;
}

float motor_param_get_friction_coulomb(const motor_param_t *cfg)
{
	return cfg->position_loop.friction_coulomb;
}

int motor_param_set_friction_coulomb(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 0.0f || value > 100.0f)
		return -EINVAL;

	cfg->position_loop.friction_coulomb = value;
	return 0;
}

float motor_param_get_friction_viscous(const motor_param_t *cfg)
{
	return cfg->position_loop.friction_viscous;
}

int motor_param_set_friction_viscous(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 0.0f || value > 10.0f)
		return -EINVAL;

	cfg->position_loop.friction_viscous = value;
	return 0;
}

float motor_param_get_notch_freq_hz(const motor_param_t *cfg)
{
	return cfg->position_loop.notch_freq_hz;
}

int motor_param_set_notch_freq_hz(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 10.0f || value > 1000.0f)
		return -EINVAL;

	cfg->position_loop.notch_freq_hz = value;
	return 0;
}

float motor_param_get_notch_width_hz(const motor_param_t *cfg)
{
	return cfg->position_loop.notch_width_hz;
}

int motor_param_set_notch_width_hz(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 1.0f || value > 100.0f)
		return -EINVAL;

	cfg->position_loop.notch_width_hz = value;
	return 0;
}

float motor_param_get_notch_depth_db(const motor_param_t *cfg)
{
	return cfg->position_loop.notch_depth_db;
}

int motor_param_set_notch_depth_db(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 10.0f || value > 100.0f)
		return -EINVAL;

	cfg->position_loop.notch_depth_db = value;
	return 0;
}

uint8_t motor_param_get_notch_enable(const motor_param_t *cfg)
{
	return cfg->position_loop.notch_enable;
}

int motor_param_set_notch_enable(motor_param_t *cfg, uint8_t value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value > (uint8_t)1)
		return -EINVAL;

	cfg->position_loop.notch_enable = value;
	return 0;
}

float motor_param_get_speed_bandwidth_hz(const motor_param_t *cfg)
{
	return cfg->position_loop.speed_bandwidth_hz;
}

int motor_param_set_speed_bandwidth_hz(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 10.0f || value > 1000.0f)
		return -EINVAL;

	cfg->position_loop.speed_bandwidth_hz = value;
	return 0;
}

float motor_param_get_speed_filter_alpha(const motor_param_t *cfg)
{
	return cfg->position_loop.speed_filter_alpha;
}

int motor_param_set_speed_filter_alpha(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 0.0f || value > 1.0f)
		return -EINVAL;

	cfg->position_loop.speed_filter_alpha = value;
	return 0;
}

float motor_param_get_position_bandwidth_hz(const motor_param_t *cfg)
{
	return cfg->position_loop.position_bandwidth_hz;
}

int motor_param_set_position_bandwidth_hz(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 1.0f || value > 200.0f)
		return -EINVAL;

	cfg->position_loop.position_bandwidth_hz = value;
	return 0;
}

float motor_param_get_impedance_kp(const motor_param_t *cfg)
{
	return cfg->impedance_ctrl.impedance_kp;
}

int motor_param_set_impedance_kp(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 0.0f || value > 1000.0f)
		return -EINVAL;

	cfg->impedance_ctrl.impedance_kp = value;
	return 0;
}

float motor_param_get_impedance_kd(const motor_param_t *cfg)
{
	return cfg->impedance_ctrl.impedance_kd;
}

int motor_param_set_impedance_kd(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 0.0f || value > 100.0f)
		return -EINVAL;

	cfg->impedance_ctrl.impedance_kd = value;
	return 0;
}

float motor_param_get_iq_max(const motor_param_t *cfg)
{
	return cfg->impedance_ctrl.iq_max;
}

int motor_param_set_iq_max(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 0.0f || value > 100.0f)
		return -EINVAL;

	cfg->impedance_ctrl.iq_max = value;
	return 0;
}

float motor_param_get_thermal_resistance(const motor_param_t *cfg)
{
	return cfg->thermal_model.thermal_resistance;
}

int motor_param_set_thermal_resistance(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 0.1f || value > 10.0f)
		return -EINVAL;

	cfg->thermal_model.thermal_resistance = value;
	return 0;
}

float motor_param_get_thermal_time_const(const motor_param_t *cfg)
{
	return cfg->thermal_model.thermal_time_const;
}

int motor_param_set_thermal_time_const(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 1.0f || value > 600.0f)
		return -EINVAL;

	cfg->thermal_model.thermal_time_const = value;
	return 0;
}

float motor_param_get_derating_temp_start(const motor_param_t *cfg)
{
	return cfg->thermal_model.derating_temp_start;
}

int motor_param_set_derating_temp_start(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 40.0f || value > 100.0f)
		return -EINVAL;

	cfg->thermal_model.derating_temp_start = value;
	return 0;
}

float motor_param_get_protect_over_current(const motor_param_t *cfg)
{
	return cfg->protection_param.protect_over_current;
}

int motor_param_set_protect_over_current(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 1.0f || value > 200.0f)
		return -EINVAL;

	cfg->protection_param.protect_over_current = value;
	return 0;
}

float motor_param_get_protect_over_voltage(const motor_param_t *cfg)
{
	return cfg->protection_param.protect_over_voltage;
}

int motor_param_set_protect_over_voltage(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 20.0f || value > 100.0f)
		return -EINVAL;

	cfg->protection_param.protect_over_voltage = value;
	return 0;
}

float motor_param_get_protect_under_voltage(const motor_param_t *cfg)
{
	return cfg->protection_param.protect_under_voltage;
}

int motor_param_set_protect_under_voltage(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 5.0f || value > 30.0f)
		return -EINVAL;

	cfg->protection_param.protect_under_voltage = value;
	return 0;
}

float motor_param_get_protect_over_speed(const motor_param_t *cfg)
{
	return cfg->protection_param.protect_over_speed;
}

int motor_param_set_protect_over_speed(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 10.0f || value > 2000.0f)
		return -EINVAL;

	cfg->protection_param.protect_over_speed = value;
	return 0;
}

float motor_param_get_protect_over_temp(const motor_param_t *cfg)
{
	return cfg->protection_param.protect_over_temp;
}

int motor_param_set_protect_over_temp(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < 50.0f || value > 120.0f)
		return -EINVAL;

	cfg->protection_param.protect_over_temp = value;
	return 0;
}

float motor_param_get_protect_under_temp(const motor_param_t *cfg)
{
	return cfg->protection_param.protect_under_temp;
}

int motor_param_set_protect_under_temp(motor_param_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < -40.0f || value > 0.0f)
		return -EINVAL;

	cfg->protection_param.protect_under_temp = value;
	return 0;
}

int32_t motor_param_get_protect_pos_error(const motor_param_t *cfg)
{
	return cfg->protection_param.protect_pos_error;
}

int motor_param_set_protect_pos_error(motor_param_t *cfg, int32_t value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value < (int32_t)100 || value > (int32_t)100000)
		return -EINVAL;

	cfg->protection_param.protect_pos_error = value;
	return 0;
}

uint32_t motor_param_get_protect_enable_mask(const motor_param_t *cfg)
{
	return cfg->protection_param.protect_enable_mask;
}

int motor_param_set_protect_enable_mask(motor_param_t *cfg, uint32_t value)
{
	if (cfg == NULL)
		return -EINVAL;

	if (value > (uint32_t)4294967295)
		return -EINVAL;

	cfg->protection_param.protect_enable_mask = value;
	return 0;
}
