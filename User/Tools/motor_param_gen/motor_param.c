/**
 * @file    motor_param.c
 * @brief   关节电机配置参数API实现
 * @date    2026-09-09
 *
 * @warning 【自动生成文件，请勿手动修改】
 *          本文件由脚本 motor_param_generate_v9.py 根据配置表自动生成，
 *          任何手动改动都会在下次运行脚本时被覆盖。
 *          如需修改参数定义，请编辑源 CSV 配置表后重新生成。
 */

#include "motor_param.h"
#if MOTOR_PARAM_EN_PRINT
#include <stdio.h>
#endif
#include <string.h>
#include <errno.h>
#if MOTOR_PARAM_EN_VALIDATE
#include <math.h>
#endif

#if MOTOR_PARAM_EN_INIT_DEFAULTS /* 默认值常量(裁剪时不生成) */
static const motor_param_t g_default_config = {
	.motor_instance =
		{
						 .motor_id = 1,
						 .motor_name = {0},
						 },
	.motor_base =
		{
						 .r = 3.6f,
						 .ld = 0.0048f,
						 .lq = 0.0048f,
						 .flux = 0.02f,
						 .kt = 0.21f,
						 .pole_pairs = 7,
						 .rated_current = 0.6f,
						 .peak_current = 3.7f,
						 .max_speed = 200.0f,
						 .dead_time_ns = 500.0f,
						 .rated_voltage = 24.0f,
						 .rated_speed_rpm = 1550.0f,
						 .rated_torque = 0.2f,
						 .peak_torque = 0.5f,
						 .inertia = 1e-05f,
						 .ke = 0.14f,
						 .pwm_freq_hz = 20000,
						 .foc_freq_hz = 20000,
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
						 .enc_offset = 0.0f,
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
						 .current_integral_limit = 10.0f,
						 .decoupling_gain = 1.0f,
						 .deadtime_comp_v = 0.0f,
						 .pwm_max_duty = 0.9f,
						 .current_bandwidth_hz = 500.0f,
						 .current_filter_alpha = 0.1f,
						 .d_feedforward_gain = 1.0f,
						 .q_feedforward_gain = 1.0f,
						 .decouple_algo = 1,
						 .bemf_ff_enable = 1,
						 .deadtime_comp_enable = 0,
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
						 .protect_pos_error = 1000,
						 .protect_enable = 1,
						 },
	.load_sim_param =
		{
						 .load_sim_dead_zone_rad_s = 0.5f,
						 },
};
#endif

int motor_param_init(motor_param_t *cfg)
{
	if (cfg == NULL)
		return -EINVAL;
#if MOTOR_PARAM_EN_INIT_DEFAULTS
	memcpy(cfg, &g_default_config, sizeof(motor_param_t)); /* 完整默认值 */
#else
	memset(cfg, 0, sizeof(motor_param_t)); /* 裁剪: 仅全零 */
#endif
	return 0;
}

int motor_param_validate(const motor_param_t *cfg)
{
#if MOTOR_PARAM_EN_VALIDATE /* 完整范围校验 */
	if (cfg == NULL)
		return -EINVAL;

	if (cfg->motor_instance.motor_id > (uint8_t)127)
		return 1;
	if (!isfinite(cfg->motor_base.r) || cfg->motor_base.r < 0.001f || cfg->motor_base.r > 10.0f)
		return 3;
	if (!isfinite(cfg->motor_base.ld) || cfg->motor_base.ld < 1e-05f || cfg->motor_base.ld > 0.01f)
		return 4;
	if (!isfinite(cfg->motor_base.lq) || cfg->motor_base.lq < 1e-05f || cfg->motor_base.lq > 0.01f)
		return 5;
	if (!isfinite(cfg->motor_base.flux) || cfg->motor_base.flux < 0.0001f || cfg->motor_base.flux > 0.1f)
		return 6;
	if (!isfinite(cfg->motor_base.kt) || cfg->motor_base.kt < 0.01f || cfg->motor_base.kt > 10.0f)
		return 7;
	if (cfg->motor_base.pole_pairs < (uint8_t)1 || cfg->motor_base.pole_pairs > (uint8_t)64)
		return 8;
	if (!isfinite(cfg->motor_base.rated_current) || cfg->motor_base.rated_current < 0.1f || cfg->motor_base.rated_current > 100.0f)
		return 9;
	if (!isfinite(cfg->motor_base.peak_current) || cfg->motor_base.peak_current < 0.1f || cfg->motor_base.peak_current > 200.0f)
		return 10;
	if (!isfinite(cfg->motor_base.max_speed) || cfg->motor_base.max_speed < 10.0f || cfg->motor_base.max_speed > 1000.0f)
		return 11;
	if (!isfinite(cfg->motor_base.dead_time_ns) || cfg->motor_base.dead_time_ns < 100.0f || cfg->motor_base.dead_time_ns > 2000.0f)
		return 12;
	if (!isfinite(cfg->motor_base.rated_voltage) || cfg->motor_base.rated_voltage < 12.0f || cfg->motor_base.rated_voltage > 100.0f)
		return 13;
	if (!isfinite(cfg->motor_base.rated_speed_rpm) || cfg->motor_base.rated_speed_rpm < 100.0f || cfg->motor_base.rated_speed_rpm > 20000.0f)
		return 14;
	if (!isfinite(cfg->motor_base.rated_torque) || cfg->motor_base.rated_torque < 0.1f || cfg->motor_base.rated_torque > 100.0f)
		return 15;
	if (!isfinite(cfg->motor_base.peak_torque) || cfg->motor_base.peak_torque < 0.1f || cfg->motor_base.peak_torque > 300.0f)
		return 16;
	if (!isfinite(cfg->motor_base.inertia) || cfg->motor_base.inertia < 1e-07f || cfg->motor_base.inertia > 0.01f)
		return 17;
	if (!isfinite(cfg->motor_base.ke) || cfg->motor_base.ke < 0.001f || cfg->motor_base.ke > 1.0f)
		return 18;
	if (cfg->motor_base.pwm_freq_hz < (uint32_t)5000 || cfg->motor_base.pwm_freq_hz > (uint32_t)100000)
		return 19;
	if (cfg->motor_base.foc_freq_hz < (uint32_t)1000 || cfg->motor_base.foc_freq_hz > (uint32_t)100000)
		return 20;
	if (!isfinite(cfg->gearbox_param.gear_ratio) || cfg->gearbox_param.gear_ratio < 1.0f || cfg->gearbox_param.gear_ratio > 1000.0f)
		return 21;
	if (!isfinite(cfg->gearbox_param.gear_efficiency) || cfg->gearbox_param.gear_efficiency < 0.1f || cfg->gearbox_param.gear_efficiency > 1.0f)
		return 22;
	if (!isfinite(cfg->gearbox_param.output_torque_const) || cfg->gearbox_param.output_torque_const < 0.1f || cfg->gearbox_param.output_torque_const > 1000.0f)
		return 23;
	if (!isfinite(cfg->gearbox_param.gear_backlash) || cfg->gearbox_param.gear_backlash < 0.0f || cfg->gearbox_param.gear_backlash > 0.5f)
		return 24;
	if (cfg->encoder_param.enc_lines < (uint32_t)100 || cfg->encoder_param.enc_lines > (uint32_t)1000000)
		return 25;
	if (cfg->encoder_param.enc_direction < (int8_t)-1 || cfg->encoder_param.enc_direction > (int8_t)1)
		return 26;
	if (!isfinite(cfg->encoder_param.enc_offset) || cfg->encoder_param.enc_offset < -360.0f || cfg->encoder_param.enc_offset > 360.0f)
		return 27;
	if (!isfinite(cfg->encoder_param.elec_angle_bias) || cfg->encoder_param.elec_angle_bias < -3.1416f || cfg->encoder_param.elec_angle_bias > 3.1416f)
		return 28;
	if (!isfinite(cfg->encoder_param.pos_filter_alpha) || cfg->encoder_param.pos_filter_alpha < 0.0f || cfg->encoder_param.pos_filter_alpha > 1.0f)
		return 29;
	if (cfg->encoder_param.enc_type > (uint8_t)10)
		return 30;
	if (cfg->encoder_param.enc_auto_calib > (uint8_t)1)
		return 31;
	if (!isfinite(cfg->encoder_param.speed_obs_gain) || cfg->encoder_param.speed_obs_gain < 1.0f || cfg->encoder_param.speed_obs_gain > 1000.0f)
		return 32;
	if (cfg->position_limit.multiturn_enable > (uint8_t)1)
		return 33;
	if (!isfinite(cfg->position_limit.pos_min_limit) || cfg->position_limit.pos_min_limit < -1000.0f || cfg->position_limit.pos_min_limit > 0.0f)
		return 34;
	if (!isfinite(cfg->position_limit.pos_max_limit) || cfg->position_limit.pos_max_limit < 0.0f || cfg->position_limit.pos_max_limit > 1000.0f)
		return 35;
	if (cfg->position_limit.limit_sw_enable > (uint8_t)1)
		return 36;
	if (cfg->homing_param.homing_method > (uint8_t)10)
		return 37;
	if (!isfinite(cfg->homing_param.homing_speed_fast) || cfg->homing_param.homing_speed_fast < 0.1f || cfg->homing_param.homing_speed_fast > 50.0f)
		return 38;
	if (!isfinite(cfg->homing_param.homing_speed_slow) || cfg->homing_param.homing_speed_slow < 0.01f || cfg->homing_param.homing_speed_slow > 5.0f)
		return 39;
	if (!isfinite(cfg->homing_param.homing_offset) || cfg->homing_param.homing_offset < -6.283f || cfg->homing_param.homing_offset > 6.283f)
		return 40;
	if (!isfinite(cfg->homing_param.homing_current) || cfg->homing_param.homing_current < 0.1f || cfg->homing_param.homing_current > 20.0f)
		return 41;
	if (!isfinite(cfg->current_loop.current_kp_d) || cfg->current_loop.current_kp_d < 0.0f || cfg->current_loop.current_kp_d > 100.0f)
		return 42;
	if (!isfinite(cfg->current_loop.current_ki_d) || cfg->current_loop.current_ki_d < 0.0f || cfg->current_loop.current_ki_d > 1000.0f)
		return 43;
	if (!isfinite(cfg->current_loop.current_kp_q) || cfg->current_loop.current_kp_q < 0.0f || cfg->current_loop.current_kp_q > 100.0f)
		return 44;
	if (!isfinite(cfg->current_loop.current_ki_q) || cfg->current_loop.current_ki_q < 0.0f || cfg->current_loop.current_ki_q > 1000.0f)
		return 45;
	if (!isfinite(cfg->current_loop.current_integral_limit) || cfg->current_loop.current_integral_limit < 0.0f || cfg->current_loop.current_integral_limit > 100.0f)
		return 46;
	if (!isfinite(cfg->current_loop.decoupling_gain) || cfg->current_loop.decoupling_gain < 0.0f || cfg->current_loop.decoupling_gain > 1.0f)
		return 47;
	if (!isfinite(cfg->current_loop.deadtime_comp_v) || cfg->current_loop.deadtime_comp_v < 0.0f || cfg->current_loop.deadtime_comp_v > 5.0f)
		return 48;
	if (!isfinite(cfg->current_loop.pwm_max_duty) || cfg->current_loop.pwm_max_duty < 0.5f || cfg->current_loop.pwm_max_duty > 0.95f)
		return 49;
	if (!isfinite(cfg->current_loop.current_bandwidth_hz) || cfg->current_loop.current_bandwidth_hz < 100.0f || cfg->current_loop.current_bandwidth_hz > 5000.0f)
		return 50;
	if (!isfinite(cfg->current_loop.current_filter_alpha) || cfg->current_loop.current_filter_alpha < 0.0f || cfg->current_loop.current_filter_alpha > 1.0f)
		return 51;
	if (!isfinite(cfg->current_loop.d_feedforward_gain) || cfg->current_loop.d_feedforward_gain < 0.0f || cfg->current_loop.d_feedforward_gain > 2.0f)
		return 52;
	if (!isfinite(cfg->current_loop.q_feedforward_gain) || cfg->current_loop.q_feedforward_gain < 0.0f || cfg->current_loop.q_feedforward_gain > 2.0f)
		return 53;
	if (cfg->current_loop.decouple_algo > (uint8_t)2)
		return 54;
	if (cfg->current_loop.bemf_ff_enable > (uint8_t)1)
		return 55;
	if (cfg->current_loop.deadtime_comp_enable > (uint8_t)1)
		return 56;
	if (!isfinite(cfg->position_loop.speed_kp) || cfg->position_loop.speed_kp < 0.0f || cfg->position_loop.speed_kp > 100.0f)
		return 57;
	if (!isfinite(cfg->position_loop.speed_ki) || cfg->position_loop.speed_ki < 0.0f || cfg->position_loop.speed_ki > 1000.0f)
		return 58;
	if (!isfinite(cfg->position_loop.speed_integral_limit) || cfg->position_loop.speed_integral_limit < 0.0f || cfg->position_loop.speed_integral_limit > 100.0f)
		return 59;
	if (!isfinite(cfg->position_loop.velocity_ff_gain) || cfg->position_loop.velocity_ff_gain < 0.0f || cfg->position_loop.velocity_ff_gain > 1.0f)
		return 60;
	if (!isfinite(cfg->position_loop.accel_ff_gain) || cfg->position_loop.accel_ff_gain < 0.0f || cfg->position_loop.accel_ff_gain > 1.0f)
		return 61;
	if (!isfinite(cfg->position_loop.position_kp) || cfg->position_loop.position_kp < 0.0f || cfg->position_loop.position_kp > 1000.0f)
		return 62;
	if (!isfinite(cfg->position_loop.position_integral_limit) || cfg->position_loop.position_integral_limit < 0.0f || cfg->position_loop.position_integral_limit > 100.0f)
		return 63;
	if (!isfinite(cfg->position_loop.friction_coulomb) || cfg->position_loop.friction_coulomb < 0.0f || cfg->position_loop.friction_coulomb > 100.0f)
		return 64;
	if (!isfinite(cfg->position_loop.friction_viscous) || cfg->position_loop.friction_viscous < 0.0f || cfg->position_loop.friction_viscous > 10.0f)
		return 65;
	if (!isfinite(cfg->position_loop.notch_freq_hz) || cfg->position_loop.notch_freq_hz < 10.0f || cfg->position_loop.notch_freq_hz > 1000.0f)
		return 66;
	if (!isfinite(cfg->position_loop.notch_width_hz) || cfg->position_loop.notch_width_hz < 1.0f || cfg->position_loop.notch_width_hz > 100.0f)
		return 67;
	if (!isfinite(cfg->position_loop.notch_depth_db) || cfg->position_loop.notch_depth_db < 10.0f || cfg->position_loop.notch_depth_db > 100.0f)
		return 68;
	if (cfg->position_loop.notch_enable > (uint8_t)1)
		return 69;
	if (!isfinite(cfg->position_loop.speed_bandwidth_hz) || cfg->position_loop.speed_bandwidth_hz < 10.0f || cfg->position_loop.speed_bandwidth_hz > 1000.0f)
		return 70;
	if (!isfinite(cfg->position_loop.speed_filter_alpha) || cfg->position_loop.speed_filter_alpha < 0.0f || cfg->position_loop.speed_filter_alpha > 1.0f)
		return 71;
	if (!isfinite(cfg->position_loop.position_bandwidth_hz) || cfg->position_loop.position_bandwidth_hz < 1.0f || cfg->position_loop.position_bandwidth_hz > 200.0f)
		return 72;
	if (!isfinite(cfg->impedance_ctrl.impedance_kp) || cfg->impedance_ctrl.impedance_kp < 0.0f || cfg->impedance_ctrl.impedance_kp > 1000.0f)
		return 73;
	if (!isfinite(cfg->impedance_ctrl.impedance_kd) || cfg->impedance_ctrl.impedance_kd < 0.0f || cfg->impedance_ctrl.impedance_kd > 100.0f)
		return 74;
	if (!isfinite(cfg->impedance_ctrl.iq_max) || cfg->impedance_ctrl.iq_max < 0.0f || cfg->impedance_ctrl.iq_max > 100.0f)
		return 75;
	if (!isfinite(cfg->thermal_model.thermal_resistance) || cfg->thermal_model.thermal_resistance < 0.1f || cfg->thermal_model.thermal_resistance > 10.0f)
		return 76;
	if (!isfinite(cfg->thermal_model.thermal_time_const) || cfg->thermal_model.thermal_time_const < 1.0f || cfg->thermal_model.thermal_time_const > 600.0f)
		return 77;
	if (!isfinite(cfg->thermal_model.derating_temp_start) || cfg->thermal_model.derating_temp_start < 40.0f || cfg->thermal_model.derating_temp_start > 100.0f)
		return 78;
	if (!isfinite(cfg->protection_param.protect_over_current) || cfg->protection_param.protect_over_current < 1.0f || cfg->protection_param.protect_over_current > 200.0f)
		return 79;
	if (!isfinite(cfg->protection_param.protect_over_voltage) || cfg->protection_param.protect_over_voltage < 20.0f || cfg->protection_param.protect_over_voltage > 100.0f)
		return 80;
	if (!isfinite(cfg->protection_param.protect_under_voltage) || cfg->protection_param.protect_under_voltage < 5.0f || cfg->protection_param.protect_under_voltage > 30.0f)
		return 81;
	if (!isfinite(cfg->protection_param.protect_over_speed) || cfg->protection_param.protect_over_speed < 10.0f || cfg->protection_param.protect_over_speed > 2000.0f)
		return 82;
	if (cfg->protection_param.protect_pos_error < (int32_t)100 || cfg->protection_param.protect_pos_error > (int32_t)100000)
		return 85;
	if (cfg->protection_param.protect_enable > (uint32_t)1)
		return 86;
	if (!isfinite(cfg->load_sim_param.load_sim_dead_zone_rad_s) || cfg->load_sim_param.load_sim_dead_zone_rad_s < 0.05f || cfg->load_sim_param.load_sim_dead_zone_rad_s > 5.0f)
		return 87;

	return 0;
#else
	(void)cfg; /* 范围校验已裁剪, 恒通过 */
	return 0;
#endif
}

void motor_param_print(const motor_param_t *cfg)
{
#if MOTOR_PARAM_EN_PRINT /* 打印全部参数 */
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
	printf("enc_offset: %f deg\n", cfg->encoder_param.enc_offset);
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
	printf("decouple_algo: %u\n", (unsigned)cfg->current_loop.decouple_algo);
	printf("bemf_ff_enable: %u\n", (unsigned)cfg->current_loop.bemf_ff_enable);
	printf("deadtime_comp_enable: %u\n", (unsigned)cfg->current_loop.deadtime_comp_enable);

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
	printf("protect_pos_error: %d counts\n", (int)cfg->protection_param.protect_pos_error);
	printf("protect_enable: %u\n", (unsigned)cfg->protection_param.protect_enable);

	printf("\n--- Load Simulation Config ---\n");
	printf("load_sim_dead_zone_rad_s: %f rad/s\n", cfg->load_sim_param.load_sim_dead_zone_rad_s);

	printf("\n==================================\n");
#else
	(void)cfg; /* 参数打印已裁剪 */
#endif
}
