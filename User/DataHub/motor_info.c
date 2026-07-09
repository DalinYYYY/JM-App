/**
 * @file    motor_info.c
 * @brief   MotorInfo 配置参数 API 实现
 * @date    2026-07-09
 *
 * @warning 【自动生成文件，请勿手动修改】
 *          本文件由脚本 motor_info_generate.py 根据 motor_info.csv 自动生成，
 *          任何手动改动都会在下次运行脚本时被覆盖。
 *          如需修改参数定义，请编辑源 CSV 配置表后重新生成。
 */

#include "motor_info.h"
#include <stdio.h>
#include <string.h>
#include <errno.h>

int motor_info_init(motor_info_t *cfg)
{
	if (cfg == NULL)
		return -EINVAL;
	memset(cfg->raw, 0, PARAM_AREA_SIZE);

	/* ---- 头部 ---- */
	cfg->blocks.header.magic = PARAM_MAGIC;
	cfg->blocks.header.version_major = MOTOR_INFO_VERSION_MAJOR;
	cfg->blocks.header.version_minor = MOTOR_INFO_VERSION_MINOR;
	cfg->blocks.header.crc32 = 0;
	cfg->blocks.header.blocks[0].offset = 0x0040u;
	cfg->blocks.header.blocks[0].size = 64u;
	cfg->blocks.header.blocks[1].offset = 0x0080u;
	cfg->blocks.header.blocks[1].size = 128u;
	cfg->blocks.header.blocks[2].offset = 0x0100u;
	cfg->blocks.header.blocks[2].size = 64u;
	cfg->blocks.header.blocks[3].offset = 0x0140u;
	cfg->blocks.header.blocks[3].size = 320u;
	cfg->blocks.header.blocks[4].offset = 0x0280u;
	cfg->blocks.header.blocks[4].size = 128u;
	cfg->blocks.header.blocks[5].offset = 0x0300u;
	cfg->blocks.header.blocks[5].size = 64u;
	cfg->blocks.header.reserved = 0;

	/* ---- 系统级参数 ---- */
	cfg->blocks.system.config_version = 65536U;
	cfg->blocks.system.enable_uart = 0U;
	cfg->blocks.system.enable_bus_sensor = 1U;
	cfg->blocks.system.safety_limit = 1U;
	cfg->blocks.system.total_runtime_s = 0U;

	/* ---- 电机标定参数（含减速器/编码器/功率级/电流采样） ---- */
	cfg->blocks.motor_calib.is_calibrated = 0U;
	cfg->blocks.motor_calib.pole_pairs = 7U;
	cfg->blocks.motor_calib.motor_type = 0U;
	cfg->blocks.motor_calib.direction = 0U;
	cfg->blocks.motor_calib.phase_resistance = 0.1f;
	cfg->blocks.motor_calib.phase_inductance_d = 0.0001f;
	cfg->blocks.motor_calib.phase_inductance_q = 0.00012f;
	cfg->blocks.motor_calib.flux_linkage = 0.001f;
	cfg->blocks.motor_calib.torque_constant = 0.1f;
	cfg->blocks.motor_calib.rotor_inertia = 0.00001f;
	cfg->blocks.motor_calib.friction_coulomb = 0.0f;
	cfg->blocks.motor_calib.friction_viscous = 0.0f;
	cfg->blocks.motor_calib.gear_ratio = 100.0f;
	cfg->blocks.motor_calib.gear_efficiency = 0.85f;
	cfg->blocks.motor_calib.calibration_current = 5.0f;
	cfg->blocks.motor_calib.resistance_calib_max_voltage = 1.0f;
	cfg->blocks.motor_calib.current_lim = 15.0f;
	cfg->blocks.motor_calib.current_control_bandwidth = 1000.0f;
	cfg->blocks.motor_calib.enc_type = 1U;
	cfg->blocks.motor_calib.enc_lines = 16384U;
	cfg->blocks.motor_calib.enc_direction = 1;
	cfg->blocks.motor_calib.enc_offset = 0.0f;
	cfg->blocks.motor_calib.elec_angle_bias = 0.0f;
	cfg->blocks.motor_calib.pwm_freq_hz = 20000U;
	cfg->blocks.motor_calib.dead_time_ns = 500.0f;
	cfg->blocks.motor_calib.shunt_resistance = 0.01f;
	cfg->blocks.motor_calib.current_amp_gain = 50.0f;

	/* ---- 设备参数（CAN/UART） ---- */
	cfg->blocks.device.device_zero = 0.0f;
	cfg->blocks.device.device_time = 0U;
	cfg->blocks.device.can_id = 1U;
	cfg->blocks.device.can_baudrate = 1000000U;
	cfg->blocks.device.can_timeout_s = 0.5f;
	cfg->blocks.device.can_fd_enable = 0U;
	cfg->blocks.device.can_fd_baudrate = 8000000U;
	cfg->blocks.device.uart_baudrate = 115200U;

	/* ---- 控制参数（三环PID+前馈+滤波） ---- */
	cfg->blocks.control.kp_ld = 0.5f;
	cfg->blocks.control.ki_ld = 10.0f;
	cfg->blocks.control.kp_lq = 0.5f;
	cfg->blocks.control.ki_lq = 10.0f;
	cfg->blocks.control.integral_limit = 12.0f;
	cfg->blocks.control.decoupling_gain = 1.0f;
	cfg->blocks.control.comp_du_V = 0.0f;
	cfg->blocks.control.pwm_duty_max = 0.9f;
	cfg->blocks.control.kp_s = 0.1f;
	cfg->blocks.control.ki_s = 1.0f;
	cfg->blocks.control.speed_integral_limit = 10.0f;
	cfg->blocks.control.vff = 1.0f;
	cfg->blocks.control.aff = 0.0f;
	cfg->blocks.control.jerk_ff = 0.0f;
	cfg->blocks.control.speed_filter_alpha = 0.1f;
	cfg->blocks.control.speed_filter_enable = 0U;
	cfg->blocks.control.kp_p = 10.0f;
	cfg->blocks.control.ki_p = 0.0f;
	cfg->blocks.control.position_integral_limit = 0.0f;
	cfg->blocks.control.position_filter_alpha = 0.1f;
	cfg->blocks.control.position_filter_enable = 0U;
	cfg->blocks.control.following_error_limit = 1000.0f;

	/* ---- 保护与通信参数 ---- */
	cfg->blocks.protect_comm.over_current_A = 20.0f;
	cfg->blocks.protect_comm.over_voltage_V = 58.0f;
	cfg->blocks.protect_comm.under_voltage_V = 15.0f;
	cfg->blocks.protect_comm.over_temp_drive = 85.0f;
	cfg->blocks.protect_comm.over_temp_motor = 85.0f;
	cfg->blocks.protect_comm.under_temp_d = -20.0f;
	cfg->blocks.protect_comm.over_speed_rad_s = 400.0f;
	cfg->blocks.protect_comm.position_following_error_p = 1000;
	cfg->blocks.protect_comm.pos_limit_min = -1000000;
	cfg->blocks.protect_comm.pos_limit_max = 1000000;
	cfg->blocks.protect_comm.error_enable_mask = 4294967295U;

	/* ---- 高级算法参数（MIT/力控/回零） ---- */
	cfg->blocks.advanced.mit_kp = 10.0f;
	cfg->blocks.advanced.mit_kd = 0.1f;
	cfg->blocks.advanced.mit_max_current = 10.0f;
	cfg->blocks.advanced.mit_feedforward_torque = 0.0f;
	cfg->blocks.advanced.force_kp = 1.0f;
	cfg->blocks.advanced.force_ki = 0.1f;
	cfg->blocks.advanced.force_limit = 10.0f;
	cfg->blocks.advanced.force_control_enable = 0U;
	cfg->blocks.advanced.homing_method = 0U;
	cfg->blocks.advanced.homing_speed = 10.0f;
	cfg->blocks.advanced.homing_offset = 0.0f;

	return 0;
}

int motor_info_validate(const motor_info_t *cfg)
{
	if (cfg == NULL)
		return -EINVAL;

	/* 头部魔数校验 */
	if (cfg->blocks.header.magic != PARAM_MAGIC)
		return -1;

	/* 系统级参数 */
	if (cfg->blocks.system.config_version > (uint32_t)4294967295)
		return 0; /* config_version */
	if (cfg->blocks.system.enable_uart > (uint32_t)15)
		return 1; /* enable_uart */
	if (cfg->blocks.system.enable_bus_sensor > (uint32_t)1)
		return 2; /* enable_bus_sensor */
	if (cfg->blocks.system.safety_limit > (uint32_t)1)
		return 3; /* safety_limit */
	if (cfg->blocks.system.total_runtime_s > (uint32_t)4294967295)
		return 4; /* total_runtime_s */

	/* 电机标定参数（含减速器/编码器/功率级/电流采样） */
	if (cfg->blocks.motor_calib.is_calibrated > (uint32_t)1)
		return 16; /* is_calibrated */
	if (cfg->blocks.motor_calib.pole_pairs < (uint32_t)1 || cfg->blocks.motor_calib.pole_pairs > (uint32_t)128)
		return 17; /* pole_pairs */
	if (cfg->blocks.motor_calib.motor_type > (uint32_t)2)
		return 18; /* motor_type */
	if (cfg->blocks.motor_calib.direction > (uint32_t)1)
		return 19; /* direction */
	if (cfg->blocks.motor_calib.phase_resistance < 0.001f || cfg->blocks.motor_calib.phase_resistance > 50.0f)
		return 20; /* phase_resistance */
	if (cfg->blocks.motor_calib.phase_inductance_d < 0.000001f || cfg->blocks.motor_calib.phase_inductance_d > 0.1f)
		return 21; /* phase_inductance_d */
	if (cfg->blocks.motor_calib.phase_inductance_q < 0.000001f || cfg->blocks.motor_calib.phase_inductance_q > 0.1f)
		return 22; /* phase_inductance_q */
	if (cfg->blocks.motor_calib.flux_linkage < 0.00001f || cfg->blocks.motor_calib.flux_linkage > 1.0f)
		return 23; /* flux_linkage */
	if (cfg->blocks.motor_calib.torque_constant < 0.001f || cfg->blocks.motor_calib.torque_constant > 50.0f)
		return 24; /* torque_constant */
	if (cfg->blocks.motor_calib.rotor_inertia < 0.0000001f || cfg->blocks.motor_calib.rotor_inertia > 1.0f)
		return 25; /* rotor_inertia */
	if (cfg->blocks.motor_calib.friction_coulomb < 0.0f || cfg->blocks.motor_calib.friction_coulomb > 500.0f)
		return 26; /* friction_coulomb */
	if (cfg->blocks.motor_calib.friction_viscous < 0.0f || cfg->blocks.motor_calib.friction_viscous > 100.0f)
		return 27; /* friction_viscous */
	if (cfg->blocks.motor_calib.gear_ratio < 1.0f || cfg->blocks.motor_calib.gear_ratio > 10000.0f)
		return 28; /* gear_ratio */
	if (cfg->blocks.motor_calib.gear_efficiency < 0.1f || cfg->blocks.motor_calib.gear_efficiency > 1.0f)
		return 29; /* gear_efficiency */
	if (cfg->blocks.motor_calib.calibration_current < 0.1f || cfg->blocks.motor_calib.calibration_current > 50.0f)
		return 30; /* calibration_current */
	if (cfg->blocks.motor_calib.resistance_calib_max_voltage < 0.1f || cfg->blocks.motor_calib.resistance_calib_max_voltage > 60.0f)
		return 31; /* resistance_calib_max_voltage */
	if (cfg->blocks.motor_calib.current_lim < 0.1f || cfg->blocks.motor_calib.current_lim > 500.0f)
		return 32; /* current_lim */
	if (cfg->blocks.motor_calib.current_control_bandwidth < 50.0f || cfg->blocks.motor_calib.current_control_bandwidth > 20000.0f)
		return 33; /* current_control_bandwidth */
	if (cfg->blocks.motor_calib.enc_type > (uint32_t)10)
		return 34; /* enc_type */
	if (cfg->blocks.motor_calib.enc_lines < (uint32_t)100 || cfg->blocks.motor_calib.enc_lines > (uint32_t)10000000)
		return 35; /* enc_lines */
	if (cfg->blocks.motor_calib.enc_direction < (int32_t)-1 || cfg->blocks.motor_calib.enc_direction > (int32_t)1)
		return 36; /* enc_direction */
	if (cfg->blocks.motor_calib.enc_offset < -360.0f || cfg->blocks.motor_calib.enc_offset > 360.0f)
		return 37; /* enc_offset */
	if (cfg->blocks.motor_calib.elec_angle_bias < -3.1416f || cfg->blocks.motor_calib.elec_angle_bias > 3.1416f)
		return 38; /* elec_angle_bias */
	if (cfg->blocks.motor_calib.pwm_freq_hz < (uint32_t)1000 || cfg->blocks.motor_calib.pwm_freq_hz > (uint32_t)200000)
		return 39; /* pwm_freq_hz */
	if (cfg->blocks.motor_calib.dead_time_ns < 50.0f || cfg->blocks.motor_calib.dead_time_ns > 5000.0f)
		return 40; /* dead_time_ns */
	if (cfg->blocks.motor_calib.shunt_resistance < 0.0001f || cfg->blocks.motor_calib.shunt_resistance > 1.0f)
		return 41; /* shunt_resistance */
	if (cfg->blocks.motor_calib.current_amp_gain < 1.0f || cfg->blocks.motor_calib.current_amp_gain > 10000.0f)
		return 42; /* current_amp_gain */

	/* 设备参数（CAN/UART） */
	if (cfg->blocks.device.device_zero < -12.566f || cfg->blocks.device.device_zero > 12.566f)
		return 48; /* device_zero */
	if (cfg->blocks.device.device_time > (uint32_t)99999999)
		return 49; /* device_time */
	if (cfg->blocks.device.can_id > (uint32_t)2047)
		return 50; /* can_id */
	if (cfg->blocks.device.can_baudrate < (uint32_t)10000 || cfg->blocks.device.can_baudrate > (uint32_t)8000000)
		return 51; /* can_baudrate */
	if (cfg->blocks.device.can_timeout_s < 0.0f || cfg->blocks.device.can_timeout_s > 60.0f)
		return 52; /* can_timeout_s */
	if (cfg->blocks.device.can_fd_enable > (uint32_t)1)
		return 53; /* can_fd_enable */
	if (cfg->blocks.device.can_fd_baudrate < (uint32_t)100000 || cfg->blocks.device.can_fd_baudrate > (uint32_t)8000000)
		return 54; /* can_fd_baudrate */
	if (cfg->blocks.device.uart_baudrate < (uint32_t)1200 || cfg->blocks.device.uart_baudrate > (uint32_t)8000000)
		return 55; /* uart_baudrate */

	/* 控制参数（三环PID+前馈+滤波） */
	if (cfg->blocks.control.kp_ld < 0.0f || cfg->blocks.control.kp_ld > 1000.0f)
		return 64; /* kp_ld */
	if (cfg->blocks.control.ki_ld < 0.0f || cfg->blocks.control.ki_ld > 100000.0f)
		return 65; /* ki_ld */
	if (cfg->blocks.control.kp_lq < 0.0f || cfg->blocks.control.kp_lq > 1000.0f)
		return 66; /* kp_lq */
	if (cfg->blocks.control.ki_lq < 0.0f || cfg->blocks.control.ki_lq > 100000.0f)
		return 67; /* ki_lq */
	if (cfg->blocks.control.integral_limit < 0.0f || cfg->blocks.control.integral_limit > 1000.0f)
		return 68; /* integral_limit */
	if (cfg->blocks.control.decoupling_gain < 0.0f || cfg->blocks.control.decoupling_gain > 5.0f)
		return 69; /* decoupling_gain */
	if (cfg->blocks.control.comp_du_V < 0.0f || cfg->blocks.control.comp_du_V > 20.0f)
		return 70; /* comp_du_V */
	if (cfg->blocks.control.pwm_duty_max < 0.1f || cfg->blocks.control.pwm_duty_max > 0.99f)
		return 71; /* pwm_duty_max */
	if (cfg->blocks.control.kp_s < 0.0f || cfg->blocks.control.kp_s > 10000.0f)
		return 72; /* kp_s */
	if (cfg->blocks.control.ki_s < 0.0f || cfg->blocks.control.ki_s > 100000.0f)
		return 73; /* ki_s */
	if (cfg->blocks.control.speed_integral_limit < 0.0f || cfg->blocks.control.speed_integral_limit > 1000.0f)
		return 74; /* speed_integral_limit */
	if (cfg->blocks.control.vff < 0.0f || cfg->blocks.control.vff > 5.0f)
		return 75; /* vff */
	if (cfg->blocks.control.aff < 0.0f || cfg->blocks.control.aff > 5.0f)
		return 76; /* aff */
	if (cfg->blocks.control.jerk_ff < 0.0f || cfg->blocks.control.jerk_ff > 5.0f)
		return 77; /* jerk_ff */
	if (cfg->blocks.control.speed_filter_alpha < 0.0f || cfg->blocks.control.speed_filter_alpha > 1.0f)
		return 78; /* speed_filter_alpha */
	if (cfg->blocks.control.speed_filter_enable > (uint32_t)1)
		return 79; /* speed_filter_enable */
	if (cfg->blocks.control.kp_p < 0.0f || cfg->blocks.control.kp_p > 10000.0f)
		return 80; /* kp_p */
	if (cfg->blocks.control.ki_p < 0.0f || cfg->blocks.control.ki_p > 10000.0f)
		return 81; /* ki_p */
	if (cfg->blocks.control.position_integral_limit < 0.0f || cfg->blocks.control.position_integral_limit > 1000.0f)
		return 82; /* position_integral_limit */
	if (cfg->blocks.control.position_filter_alpha < 0.0f || cfg->blocks.control.position_filter_alpha > 1.0f)
		return 83; /* position_filter_alpha */
	if (cfg->blocks.control.position_filter_enable > (uint32_t)1)
		return 84; /* position_filter_enable */
	if (cfg->blocks.control.following_error_limit < 0.0f || cfg->blocks.control.following_error_limit > 1000000.0f)
		return 85; /* following_error_limit */

	/* 保护与通信参数 */
	if (cfg->blocks.protect_comm.over_current_A < 0.1f || cfg->blocks.protect_comm.over_current_A > 1000.0f)
		return 128; /* over_current_A */
	if (cfg->blocks.protect_comm.over_voltage_V < 5.0f || cfg->blocks.protect_comm.over_voltage_V > 120.0f)
		return 129; /* over_voltage_V */
	if (cfg->blocks.protect_comm.under_voltage_V < 0.0f || cfg->blocks.protect_comm.under_voltage_V > 60.0f)
		return 130; /* under_voltage_V */
	if (cfg->blocks.protect_comm.over_temp_drive < 20.0f || cfg->blocks.protect_comm.over_temp_drive > 150.0f)
		return 131; /* over_temp_drive */
	if (cfg->blocks.protect_comm.over_temp_motor < 20.0f || cfg->blocks.protect_comm.over_temp_motor > 200.0f)
		return 132; /* over_temp_motor */
	if (cfg->blocks.protect_comm.under_temp_d < -80.0f || cfg->blocks.protect_comm.under_temp_d > 0.0f)
		return 133; /* under_temp_d */
	if (cfg->blocks.protect_comm.over_speed_rad_s < 1.0f || cfg->blocks.protect_comm.over_speed_rad_s > 20000.0f)
		return 134; /* over_speed_rad_s */
	if (cfg->blocks.protect_comm.position_following_error_p < (int32_t)1 || cfg->blocks.protect_comm.position_following_error_p > (int32_t)1000000)
		return 135; /* position_following_error_p */
	if (cfg->blocks.protect_comm.pos_limit_min < (int32_t)-10000000 || cfg->blocks.protect_comm.pos_limit_min > (int32_t)0)
		return 136; /* pos_limit_min */
	if (cfg->blocks.protect_comm.pos_limit_max < (int32_t)0 || cfg->blocks.protect_comm.pos_limit_max > (int32_t)10000000)
		return 137; /* pos_limit_max */
	if (cfg->blocks.protect_comm.error_enable_mask > (uint32_t)4294967295)
		return 138; /* error_enable_mask */

	/* 高级算法参数（MIT/力控/回零） */
	if (cfg->blocks.advanced.mit_kp < 0.0f || cfg->blocks.advanced.mit_kp > 10000.0f)
		return 160; /* mit_kp */
	if (cfg->blocks.advanced.mit_kd < 0.0f || cfg->blocks.advanced.mit_kd > 1000.0f)
		return 161; /* mit_kd */
	if (cfg->blocks.advanced.mit_max_current < 0.0f || cfg->blocks.advanced.mit_max_current > 500.0f)
		return 162; /* mit_max_current */
	if (cfg->blocks.advanced.mit_feedforward_torque < 0.0f || cfg->blocks.advanced.mit_feedforward_torque > 500.0f)
		return 163; /* mit_feedforward_torque */
	if (cfg->blocks.advanced.force_kp < 0.0f || cfg->blocks.advanced.force_kp > 10000.0f)
		return 164; /* force_kp */
	if (cfg->blocks.advanced.force_ki < 0.0f || cfg->blocks.advanced.force_ki > 10000.0f)
		return 165; /* force_ki */
	if (cfg->blocks.advanced.force_limit < 0.0f || cfg->blocks.advanced.force_limit > 1000.0f)
		return 166; /* force_limit */
	if (cfg->blocks.advanced.force_control_enable > (uint32_t)1)
		return 167; /* force_control_enable */
	if (cfg->blocks.advanced.homing_method > (uint32_t)10)
		return 168; /* homing_method */
	if (cfg->blocks.advanced.homing_speed < 0.001f || cfg->blocks.advanced.homing_speed > 500.0f)
		return 169; /* homing_speed */
	if (cfg->blocks.advanced.homing_offset < -6.283f || cfg->blocks.advanced.homing_offset > 6.283f)
		return 170; /* homing_offset */

	return 0;
}

void motor_info_print(const motor_info_t *cfg)
{
	if (cfg == NULL)
		return;
	printf("========== MotorInfo Config (%d params) ==========\n", MOTOR_INFO_PARAM_COUNT);
	printf("magic=0x%08X  v%d.%d  crc32=0x%08X\n",
	       (unsigned)cfg->blocks.header.magic,
	       cfg->blocks.header.version_major,
	       cfg->blocks.header.version_minor,
	       (unsigned)cfg->blocks.header.crc32);

	printf("\n--- System Parameters ---\n");
	printf("config_version: %u\n", (unsigned)cfg->blocks.system.config_version);
	printf("enable_uart: %u\n", (unsigned)cfg->blocks.system.enable_uart);
	printf("enable_bus_sensor: %u\n", (unsigned)cfg->blocks.system.enable_bus_sensor);
	printf("safety_limit: %u\n", (unsigned)cfg->blocks.system.safety_limit);
	printf("total_runtime_s: %u s\n", (unsigned)cfg->blocks.system.total_runtime_s);

	printf("\n--- Motor Calibration Parameters ---\n");
	printf("is_calibrated: %u\n", (unsigned)cfg->blocks.motor_calib.is_calibrated);
	printf("pole_pairs: %u pairs\n", (unsigned)cfg->blocks.motor_calib.pole_pairs);
	printf("motor_type: %u\n", (unsigned)cfg->blocks.motor_calib.motor_type);
	printf("direction: %u\n", (unsigned)cfg->blocks.motor_calib.direction);
	printf("phase_resistance: %f ohm\n", cfg->blocks.motor_calib.phase_resistance);
	printf("phase_inductance_d: %f H\n", cfg->blocks.motor_calib.phase_inductance_d);
	printf("phase_inductance_q: %f H\n", cfg->blocks.motor_calib.phase_inductance_q);
	printf("flux_linkage: %f Wb\n", cfg->blocks.motor_calib.flux_linkage);
	printf("torque_constant: %f Nm/A\n", cfg->blocks.motor_calib.torque_constant);
	printf("rotor_inertia: %f kg·m²\n", cfg->blocks.motor_calib.rotor_inertia);
	printf("friction_coulomb: %f Nm\n", cfg->blocks.motor_calib.friction_coulomb);
	printf("friction_viscous: %f Nm/(rad/s)\n", cfg->blocks.motor_calib.friction_viscous);
	printf("gear_ratio: %f\n", cfg->blocks.motor_calib.gear_ratio);
	printf("gear_efficiency: %f\n", cfg->blocks.motor_calib.gear_efficiency);
	printf("calibration_current: %f A\n", cfg->blocks.motor_calib.calibration_current);
	printf("resistance_calib_max_voltage: %f V\n", cfg->blocks.motor_calib.resistance_calib_max_voltage);
	printf("current_lim: %f A\n", cfg->blocks.motor_calib.current_lim);
	printf("current_control_bandwidth: %f Hz\n", cfg->blocks.motor_calib.current_control_bandwidth);
	printf("enc_type: %u\n", (unsigned)cfg->blocks.motor_calib.enc_type);
	printf("enc_lines: %u CPR\n", (unsigned)cfg->blocks.motor_calib.enc_lines);
	printf("enc_direction: %d\n", (int)cfg->blocks.motor_calib.enc_direction);
	printf("enc_offset: %f deg\n", cfg->blocks.motor_calib.enc_offset);
	printf("elec_angle_bias: %f rad\n", cfg->blocks.motor_calib.elec_angle_bias);
	printf("pwm_freq_hz: %u Hz\n", (unsigned)cfg->blocks.motor_calib.pwm_freq_hz);
	printf("dead_time_ns: %f ns\n", cfg->blocks.motor_calib.dead_time_ns);
	printf("shunt_resistance: %f ohm\n", cfg->blocks.motor_calib.shunt_resistance);
	printf("current_amp_gain: %f\n", cfg->blocks.motor_calib.current_amp_gain);

	printf("\n--- Device Parameters ---\n");
	printf("device_zero: %f rad\n", cfg->blocks.device.device_zero);
	printf("device_time: %u\n", (unsigned)cfg->blocks.device.device_time);
	printf("can_id: %u\n", (unsigned)cfg->blocks.device.can_id);
	printf("can_baudrate: %u bps\n", (unsigned)cfg->blocks.device.can_baudrate);
	printf("can_timeout_s: %f s\n", cfg->blocks.device.can_timeout_s);
	printf("can_fd_enable: %u\n", (unsigned)cfg->blocks.device.can_fd_enable);
	printf("can_fd_baudrate: %u bps\n", (unsigned)cfg->blocks.device.can_fd_baudrate);
	printf("uart_baudrate: %u bps\n", (unsigned)cfg->blocks.device.uart_baudrate);

	printf("\n--- Control Parameters ---\n");
	printf("kp_ld: %f V/A\n", cfg->blocks.control.kp_ld);
	printf("ki_ld: %f V/(A·s)\n", cfg->blocks.control.ki_ld);
	printf("kp_lq: %f V/A\n", cfg->blocks.control.kp_lq);
	printf("ki_lq: %f V/(A·s)\n", cfg->blocks.control.ki_lq);
	printf("integral_limit: %f V\n", cfg->blocks.control.integral_limit);
	printf("decoupling_gain: %f\n", cfg->blocks.control.decoupling_gain);
	printf("comp_du_V: %f V\n", cfg->blocks.control.comp_du_V);
	printf("pwm_duty_max: %f\n", cfg->blocks.control.pwm_duty_max);
	printf("kp_s: %f A/(rad/s)\n", cfg->blocks.control.kp_s);
	printf("ki_s: %f A/rad\n", cfg->blocks.control.ki_s);
	printf("speed_integral_limit: %f A\n", cfg->blocks.control.speed_integral_limit);
	printf("vff: %f\n", cfg->blocks.control.vff);
	printf("aff: %f\n", cfg->blocks.control.aff);
	printf("jerk_ff: %f\n", cfg->blocks.control.jerk_ff);
	printf("speed_filter_alpha: %f\n", cfg->blocks.control.speed_filter_alpha);
	printf("speed_filter_enable: %u\n", (unsigned)cfg->blocks.control.speed_filter_enable);
	printf("kp_p: %f Hz\n", cfg->blocks.control.kp_p);
	printf("ki_p: %f 1/s\n", cfg->blocks.control.ki_p);
	printf("position_integral_limit: %f rad\n", cfg->blocks.control.position_integral_limit);
	printf("position_filter_alpha: %f\n", cfg->blocks.control.position_filter_alpha);
	printf("position_filter_enable: %u\n", (unsigned)cfg->blocks.control.position_filter_enable);
	printf("following_error_limit: %f P\n", cfg->blocks.control.following_error_limit);

	printf("\n--- Protection & Comm Parameters ---\n");
	printf("over_current_A: %f A\n", cfg->blocks.protect_comm.over_current_A);
	printf("over_voltage_V: %f V\n", cfg->blocks.protect_comm.over_voltage_V);
	printf("under_voltage_V: %f V\n", cfg->blocks.protect_comm.under_voltage_V);
	printf("over_temp_drive: %f ℃\n", cfg->blocks.protect_comm.over_temp_drive);
	printf("over_temp_motor: %f ℃\n", cfg->blocks.protect_comm.over_temp_motor);
	printf("under_temp_d: %f ℃\n", cfg->blocks.protect_comm.under_temp_d);
	printf("over_speed_rad_s: %f rad/s\n", cfg->blocks.protect_comm.over_speed_rad_s);
	printf("position_following_error_p: %d P\n", (int)cfg->blocks.protect_comm.position_following_error_p);
	printf("pos_limit_min: %d P\n", (int)cfg->blocks.protect_comm.pos_limit_min);
	printf("pos_limit_max: %d P\n", (int)cfg->blocks.protect_comm.pos_limit_max);
	printf("error_enable_mask: %u\n", (unsigned)cfg->blocks.protect_comm.error_enable_mask);

	printf("\n--- Advanced Algorithm Parameters ---\n");
	printf("mit_kp: %f Nm/rad\n", cfg->blocks.advanced.mit_kp);
	printf("mit_kd: %f Nm/(rad/s)\n", cfg->blocks.advanced.mit_kd);
	printf("mit_max_current: %f A\n", cfg->blocks.advanced.mit_max_current);
	printf("mit_feedforward_torque: %f Nm\n", cfg->blocks.advanced.mit_feedforward_torque);
	printf("force_kp: %f A/Nm\n", cfg->blocks.advanced.force_kp);
	printf("force_ki: %f A/(Nm·s)\n", cfg->blocks.advanced.force_ki);
	printf("force_limit: %f Nm\n", cfg->blocks.advanced.force_limit);
	printf("force_control_enable: %u\n", (unsigned)cfg->blocks.advanced.force_control_enable);
	printf("homing_method: %u\n", (unsigned)cfg->blocks.advanced.homing_method);
	printf("homing_speed: %f rad/s\n", cfg->blocks.advanced.homing_speed);
	printf("homing_offset: %f rad\n", cfg->blocks.advanced.homing_offset);

	printf("\n=============================================\n");
}

/******************************************************************************
 * 系统级参数
 ******************************************************************************/
uint32_t motor_info_get_config_version(const motor_info_t *cfg)
{
	return cfg->blocks.system.config_version;
}

int motor_info_set_config_version(motor_info_t *cfg, uint32_t value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value > (uint32_t)4294967295)
		return -EINVAL; /* 越界 */
	cfg->blocks.system.config_version = value;
	return 0;
}

uint32_t motor_info_get_enable_uart(const motor_info_t *cfg)
{
	return cfg->blocks.system.enable_uart;
}

int motor_info_set_enable_uart(motor_info_t *cfg, uint32_t value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value > (uint32_t)15)
		return -EINVAL; /* 越界 */
	cfg->blocks.system.enable_uart = value;
	return 0;
}

uint32_t motor_info_get_enable_bus_sensor(const motor_info_t *cfg)
{
	return cfg->blocks.system.enable_bus_sensor;
}

int motor_info_set_enable_bus_sensor(motor_info_t *cfg, uint32_t value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value > (uint32_t)1)
		return -EINVAL; /* 越界 */
	cfg->blocks.system.enable_bus_sensor = value;
	return 0;
}

uint32_t motor_info_get_safety_limit(const motor_info_t *cfg)
{
	return cfg->blocks.system.safety_limit;
}

int motor_info_set_safety_limit(motor_info_t *cfg, uint32_t value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value > (uint32_t)1)
		return -EINVAL; /* 越界 */
	cfg->blocks.system.safety_limit = value;
	return 0;
}

uint32_t motor_info_get_total_runtime_s(const motor_info_t *cfg)
{
	return cfg->blocks.system.total_runtime_s;
}

int motor_info_set_total_runtime_s(motor_info_t *cfg, uint32_t value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value > (uint32_t)4294967295)
		return -EINVAL; /* 越界 */
	cfg->blocks.system.total_runtime_s = value;
	return 0;
}

/******************************************************************************
 * 电机标定参数（含减速器/编码器/功率级/电流采样）
 ******************************************************************************/
uint32_t motor_info_get_is_calibrated(const motor_info_t *cfg)
{
	return cfg->blocks.motor_calib.is_calibrated;
}

int motor_info_set_is_calibrated(motor_info_t *cfg, uint32_t value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value > (uint32_t)1)
		return -EINVAL; /* 越界 */
	cfg->blocks.motor_calib.is_calibrated = value;
	return 0;
}

uint32_t motor_info_get_pole_pairs(const motor_info_t *cfg)
{
	return cfg->blocks.motor_calib.pole_pairs;
}

int motor_info_set_pole_pairs(motor_info_t *cfg, uint32_t value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < (uint32_t)1 || value > (uint32_t)128)
		return -EINVAL; /* 越界 */
	cfg->blocks.motor_calib.pole_pairs = value;
	return 0;
}

uint32_t motor_info_get_motor_type(const motor_info_t *cfg)
{
	return cfg->blocks.motor_calib.motor_type;
}

int motor_info_set_motor_type(motor_info_t *cfg, uint32_t value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value > (uint32_t)2)
		return -EINVAL; /* 越界 */
	cfg->blocks.motor_calib.motor_type = value;
	return 0;
}

uint32_t motor_info_get_direction(const motor_info_t *cfg)
{
	return cfg->blocks.motor_calib.direction;
}

int motor_info_set_direction(motor_info_t *cfg, uint32_t value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value > (uint32_t)1)
		return -EINVAL; /* 越界 */
	cfg->blocks.motor_calib.direction = value;
	return 0;
}

float motor_info_get_phase_resistance(const motor_info_t *cfg)
{
	return cfg->blocks.motor_calib.phase_resistance;
}

int motor_info_set_phase_resistance(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < 0.001f || value > 50.0f)
		return -EINVAL; /* 越界 */
	cfg->blocks.motor_calib.phase_resistance = value;
	return 0;
}

float motor_info_get_phase_inductance_d(const motor_info_t *cfg)
{
	return cfg->blocks.motor_calib.phase_inductance_d;
}

int motor_info_set_phase_inductance_d(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < 0.000001f || value > 0.1f)
		return -EINVAL; /* 越界 */
	cfg->blocks.motor_calib.phase_inductance_d = value;
	return 0;
}

float motor_info_get_phase_inductance_q(const motor_info_t *cfg)
{
	return cfg->blocks.motor_calib.phase_inductance_q;
}

int motor_info_set_phase_inductance_q(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < 0.000001f || value > 0.1f)
		return -EINVAL; /* 越界 */
	cfg->blocks.motor_calib.phase_inductance_q = value;
	return 0;
}

float motor_info_get_flux_linkage(const motor_info_t *cfg)
{
	return cfg->blocks.motor_calib.flux_linkage;
}

int motor_info_set_flux_linkage(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < 0.00001f || value > 1.0f)
		return -EINVAL; /* 越界 */
	cfg->blocks.motor_calib.flux_linkage = value;
	return 0;
}

float motor_info_get_torque_constant(const motor_info_t *cfg)
{
	return cfg->blocks.motor_calib.torque_constant;
}

int motor_info_set_torque_constant(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < 0.001f || value > 50.0f)
		return -EINVAL; /* 越界 */
	cfg->blocks.motor_calib.torque_constant = value;
	return 0;
}

float motor_info_get_rotor_inertia(const motor_info_t *cfg)
{
	return cfg->blocks.motor_calib.rotor_inertia;
}

int motor_info_set_rotor_inertia(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < 0.0000001f || value > 1.0f)
		return -EINVAL; /* 越界 */
	cfg->blocks.motor_calib.rotor_inertia = value;
	return 0;
}

float motor_info_get_friction_coulomb(const motor_info_t *cfg)
{
	return cfg->blocks.motor_calib.friction_coulomb;
}

int motor_info_set_friction_coulomb(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < 0.0f || value > 500.0f)
		return -EINVAL; /* 越界 */
	cfg->blocks.motor_calib.friction_coulomb = value;
	return 0;
}

float motor_info_get_friction_viscous(const motor_info_t *cfg)
{
	return cfg->blocks.motor_calib.friction_viscous;
}

int motor_info_set_friction_viscous(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < 0.0f || value > 100.0f)
		return -EINVAL; /* 越界 */
	cfg->blocks.motor_calib.friction_viscous = value;
	return 0;
}

float motor_info_get_gear_ratio(const motor_info_t *cfg)
{
	return cfg->blocks.motor_calib.gear_ratio;
}

int motor_info_set_gear_ratio(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < 1.0f || value > 10000.0f)
		return -EINVAL; /* 越界 */
	cfg->blocks.motor_calib.gear_ratio = value;
	return 0;
}

float motor_info_get_gear_efficiency(const motor_info_t *cfg)
{
	return cfg->blocks.motor_calib.gear_efficiency;
}

int motor_info_set_gear_efficiency(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < 0.1f || value > 1.0f)
		return -EINVAL; /* 越界 */
	cfg->blocks.motor_calib.gear_efficiency = value;
	return 0;
}

float motor_info_get_calibration_current(const motor_info_t *cfg)
{
	return cfg->blocks.motor_calib.calibration_current;
}

int motor_info_set_calibration_current(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < 0.1f || value > 50.0f)
		return -EINVAL; /* 越界 */
	cfg->blocks.motor_calib.calibration_current = value;
	return 0;
}

float motor_info_get_resistance_calib_max_voltage(const motor_info_t *cfg)
{
	return cfg->blocks.motor_calib.resistance_calib_max_voltage;
}

int motor_info_set_resistance_calib_max_voltage(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < 0.1f || value > 60.0f)
		return -EINVAL; /* 越界 */
	cfg->blocks.motor_calib.resistance_calib_max_voltage = value;
	return 0;
}

float motor_info_get_current_lim(const motor_info_t *cfg)
{
	return cfg->blocks.motor_calib.current_lim;
}

int motor_info_set_current_lim(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < 0.1f || value > 500.0f)
		return -EINVAL; /* 越界 */
	cfg->blocks.motor_calib.current_lim = value;
	return 0;
}

float motor_info_get_current_control_bandwidth(const motor_info_t *cfg)
{
	return cfg->blocks.motor_calib.current_control_bandwidth;
}

int motor_info_set_current_control_bandwidth(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < 50.0f || value > 20000.0f)
		return -EINVAL; /* 越界 */
	cfg->blocks.motor_calib.current_control_bandwidth = value;
	return 0;
}

uint32_t motor_info_get_enc_type(const motor_info_t *cfg)
{
	return cfg->blocks.motor_calib.enc_type;
}

int motor_info_set_enc_type(motor_info_t *cfg, uint32_t value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value > (uint32_t)10)
		return -EINVAL; /* 越界 */
	cfg->blocks.motor_calib.enc_type = value;
	return 0;
}

uint32_t motor_info_get_enc_lines(const motor_info_t *cfg)
{
	return cfg->blocks.motor_calib.enc_lines;
}

int motor_info_set_enc_lines(motor_info_t *cfg, uint32_t value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < (uint32_t)100 || value > (uint32_t)10000000)
		return -EINVAL; /* 越界 */
	cfg->blocks.motor_calib.enc_lines = value;
	return 0;
}

int32_t motor_info_get_enc_direction(const motor_info_t *cfg)
{
	return cfg->blocks.motor_calib.enc_direction;
}

int motor_info_set_enc_direction(motor_info_t *cfg, int32_t value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < (int32_t)-1 || value > (int32_t)1)
		return -EINVAL; /* 越界 */
	cfg->blocks.motor_calib.enc_direction = value;
	return 0;
}

float motor_info_get_enc_offset(const motor_info_t *cfg)
{
	return cfg->blocks.motor_calib.enc_offset;
}

int motor_info_set_enc_offset(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < -360.0f || value > 360.0f)
		return -EINVAL; /* 越界 */
	cfg->blocks.motor_calib.enc_offset = value;
	return 0;
}

float motor_info_get_elec_angle_bias(const motor_info_t *cfg)
{
	return cfg->blocks.motor_calib.elec_angle_bias;
}

int motor_info_set_elec_angle_bias(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < -3.1416f || value > 3.1416f)
		return -EINVAL; /* 越界 */
	cfg->blocks.motor_calib.elec_angle_bias = value;
	return 0;
}

uint32_t motor_info_get_pwm_freq_hz(const motor_info_t *cfg)
{
	return cfg->blocks.motor_calib.pwm_freq_hz;
}

int motor_info_set_pwm_freq_hz(motor_info_t *cfg, uint32_t value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < (uint32_t)1000 || value > (uint32_t)200000)
		return -EINVAL; /* 越界 */
	cfg->blocks.motor_calib.pwm_freq_hz = value;
	return 0;
}

float motor_info_get_dead_time_ns(const motor_info_t *cfg)
{
	return cfg->blocks.motor_calib.dead_time_ns;
}

int motor_info_set_dead_time_ns(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < 50.0f || value > 5000.0f)
		return -EINVAL; /* 越界 */
	cfg->blocks.motor_calib.dead_time_ns = value;
	return 0;
}

float motor_info_get_shunt_resistance(const motor_info_t *cfg)
{
	return cfg->blocks.motor_calib.shunt_resistance;
}

int motor_info_set_shunt_resistance(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < 0.0001f || value > 1.0f)
		return -EINVAL; /* 越界 */
	cfg->blocks.motor_calib.shunt_resistance = value;
	return 0;
}

float motor_info_get_current_amp_gain(const motor_info_t *cfg)
{
	return cfg->blocks.motor_calib.current_amp_gain;
}

int motor_info_set_current_amp_gain(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < 1.0f || value > 10000.0f)
		return -EINVAL; /* 越界 */
	cfg->blocks.motor_calib.current_amp_gain = value;
	return 0;
}

/******************************************************************************
 * 设备参数（CAN/UART）
 ******************************************************************************/
float motor_info_get_device_zero(const motor_info_t *cfg)
{
	return cfg->blocks.device.device_zero;
}

int motor_info_set_device_zero(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < -12.566f || value > 12.566f)
		return -EINVAL; /* 越界 */
	cfg->blocks.device.device_zero = value;
	return 0;
}

uint32_t motor_info_get_device_time(const motor_info_t *cfg)
{
	return cfg->blocks.device.device_time;
}

int motor_info_set_device_time(motor_info_t *cfg, uint32_t value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value > (uint32_t)99999999)
		return -EINVAL; /* 越界 */
	cfg->blocks.device.device_time = value;
	return 0;
}

uint32_t motor_info_get_can_id(const motor_info_t *cfg)
{
	return cfg->blocks.device.can_id;
}

int motor_info_set_can_id(motor_info_t *cfg, uint32_t value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value > (uint32_t)2047)
		return -EINVAL; /* 越界 */
	cfg->blocks.device.can_id = value;
	return 0;
}

uint32_t motor_info_get_can_baudrate(const motor_info_t *cfg)
{
	return cfg->blocks.device.can_baudrate;
}

int motor_info_set_can_baudrate(motor_info_t *cfg, uint32_t value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < (uint32_t)10000 || value > (uint32_t)8000000)
		return -EINVAL; /* 越界 */
	cfg->blocks.device.can_baudrate = value;
	return 0;
}

float motor_info_get_can_timeout_s(const motor_info_t *cfg)
{
	return cfg->blocks.device.can_timeout_s;
}

int motor_info_set_can_timeout_s(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < 0.0f || value > 60.0f)
		return -EINVAL; /* 越界 */
	cfg->blocks.device.can_timeout_s = value;
	return 0;
}

uint32_t motor_info_get_can_fd_enable(const motor_info_t *cfg)
{
	return cfg->blocks.device.can_fd_enable;
}

int motor_info_set_can_fd_enable(motor_info_t *cfg, uint32_t value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value > (uint32_t)1)
		return -EINVAL; /* 越界 */
	cfg->blocks.device.can_fd_enable = value;
	return 0;
}

uint32_t motor_info_get_can_fd_baudrate(const motor_info_t *cfg)
{
	return cfg->blocks.device.can_fd_baudrate;
}

int motor_info_set_can_fd_baudrate(motor_info_t *cfg, uint32_t value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < (uint32_t)100000 || value > (uint32_t)8000000)
		return -EINVAL; /* 越界 */
	cfg->blocks.device.can_fd_baudrate = value;
	return 0;
}

uint32_t motor_info_get_uart_baudrate(const motor_info_t *cfg)
{
	return cfg->blocks.device.uart_baudrate;
}

int motor_info_set_uart_baudrate(motor_info_t *cfg, uint32_t value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < (uint32_t)1200 || value > (uint32_t)8000000)
		return -EINVAL; /* 越界 */
	cfg->blocks.device.uart_baudrate = value;
	return 0;
}

/******************************************************************************
 * 控制参数（三环PID+前馈+滤波）
 ******************************************************************************/
float motor_info_get_kp_ld(const motor_info_t *cfg)
{
	return cfg->blocks.control.kp_ld;
}

int motor_info_set_kp_ld(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < 0.0f || value > 1000.0f)
		return -EINVAL; /* 越界 */
	cfg->blocks.control.kp_ld = value;
	return 0;
}

float motor_info_get_ki_ld(const motor_info_t *cfg)
{
	return cfg->blocks.control.ki_ld;
}

int motor_info_set_ki_ld(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < 0.0f || value > 100000.0f)
		return -EINVAL; /* 越界 */
	cfg->blocks.control.ki_ld = value;
	return 0;
}

float motor_info_get_kp_lq(const motor_info_t *cfg)
{
	return cfg->blocks.control.kp_lq;
}

int motor_info_set_kp_lq(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < 0.0f || value > 1000.0f)
		return -EINVAL; /* 越界 */
	cfg->blocks.control.kp_lq = value;
	return 0;
}

float motor_info_get_ki_lq(const motor_info_t *cfg)
{
	return cfg->blocks.control.ki_lq;
}

int motor_info_set_ki_lq(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < 0.0f || value > 100000.0f)
		return -EINVAL; /* 越界 */
	cfg->blocks.control.ki_lq = value;
	return 0;
}

float motor_info_get_integral_limit(const motor_info_t *cfg)
{
	return cfg->blocks.control.integral_limit;
}

int motor_info_set_integral_limit(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < 0.0f || value > 1000.0f)
		return -EINVAL; /* 越界 */
	cfg->blocks.control.integral_limit = value;
	return 0;
}

float motor_info_get_decoupling_gain(const motor_info_t *cfg)
{
	return cfg->blocks.control.decoupling_gain;
}

int motor_info_set_decoupling_gain(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < 0.0f || value > 5.0f)
		return -EINVAL; /* 越界 */
	cfg->blocks.control.decoupling_gain = value;
	return 0;
}

float motor_info_get_comp_du_V(const motor_info_t *cfg)
{
	return cfg->blocks.control.comp_du_V;
}

int motor_info_set_comp_du_V(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < 0.0f || value > 20.0f)
		return -EINVAL; /* 越界 */
	cfg->blocks.control.comp_du_V = value;
	return 0;
}

float motor_info_get_pwm_duty_max(const motor_info_t *cfg)
{
	return cfg->blocks.control.pwm_duty_max;
}

int motor_info_set_pwm_duty_max(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < 0.1f || value > 0.99f)
		return -EINVAL; /* 越界 */
	cfg->blocks.control.pwm_duty_max = value;
	return 0;
}

float motor_info_get_kp_s(const motor_info_t *cfg)
{
	return cfg->blocks.control.kp_s;
}

int motor_info_set_kp_s(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < 0.0f || value > 10000.0f)
		return -EINVAL; /* 越界 */
	cfg->blocks.control.kp_s = value;
	return 0;
}

float motor_info_get_ki_s(const motor_info_t *cfg)
{
	return cfg->blocks.control.ki_s;
}

int motor_info_set_ki_s(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < 0.0f || value > 100000.0f)
		return -EINVAL; /* 越界 */
	cfg->blocks.control.ki_s = value;
	return 0;
}

float motor_info_get_speed_integral_limit(const motor_info_t *cfg)
{
	return cfg->blocks.control.speed_integral_limit;
}

int motor_info_set_speed_integral_limit(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < 0.0f || value > 1000.0f)
		return -EINVAL; /* 越界 */
	cfg->blocks.control.speed_integral_limit = value;
	return 0;
}

float motor_info_get_vff(const motor_info_t *cfg)
{
	return cfg->blocks.control.vff;
}

int motor_info_set_vff(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < 0.0f || value > 5.0f)
		return -EINVAL; /* 越界 */
	cfg->blocks.control.vff = value;
	return 0;
}

float motor_info_get_aff(const motor_info_t *cfg)
{
	return cfg->blocks.control.aff;
}

int motor_info_set_aff(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < 0.0f || value > 5.0f)
		return -EINVAL; /* 越界 */
	cfg->blocks.control.aff = value;
	return 0;
}

float motor_info_get_jerk_ff(const motor_info_t *cfg)
{
	return cfg->blocks.control.jerk_ff;
}

int motor_info_set_jerk_ff(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < 0.0f || value > 5.0f)
		return -EINVAL; /* 越界 */
	cfg->blocks.control.jerk_ff = value;
	return 0;
}

float motor_info_get_speed_filter_alpha(const motor_info_t *cfg)
{
	return cfg->blocks.control.speed_filter_alpha;
}

int motor_info_set_speed_filter_alpha(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < 0.0f || value > 1.0f)
		return -EINVAL; /* 越界 */
	cfg->blocks.control.speed_filter_alpha = value;
	return 0;
}

uint32_t motor_info_get_speed_filter_enable(const motor_info_t *cfg)
{
	return cfg->blocks.control.speed_filter_enable;
}

int motor_info_set_speed_filter_enable(motor_info_t *cfg, uint32_t value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value > (uint32_t)1)
		return -EINVAL; /* 越界 */
	cfg->blocks.control.speed_filter_enable = value;
	return 0;
}

float motor_info_get_kp_p(const motor_info_t *cfg)
{
	return cfg->blocks.control.kp_p;
}

int motor_info_set_kp_p(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < 0.0f || value > 10000.0f)
		return -EINVAL; /* 越界 */
	cfg->blocks.control.kp_p = value;
	return 0;
}

float motor_info_get_ki_p(const motor_info_t *cfg)
{
	return cfg->blocks.control.ki_p;
}

int motor_info_set_ki_p(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < 0.0f || value > 10000.0f)
		return -EINVAL; /* 越界 */
	cfg->blocks.control.ki_p = value;
	return 0;
}

float motor_info_get_position_integral_limit(const motor_info_t *cfg)
{
	return cfg->blocks.control.position_integral_limit;
}

int motor_info_set_position_integral_limit(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < 0.0f || value > 1000.0f)
		return -EINVAL; /* 越界 */
	cfg->blocks.control.position_integral_limit = value;
	return 0;
}

float motor_info_get_position_filter_alpha(const motor_info_t *cfg)
{
	return cfg->blocks.control.position_filter_alpha;
}

int motor_info_set_position_filter_alpha(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < 0.0f || value > 1.0f)
		return -EINVAL; /* 越界 */
	cfg->blocks.control.position_filter_alpha = value;
	return 0;
}

uint32_t motor_info_get_position_filter_enable(const motor_info_t *cfg)
{
	return cfg->blocks.control.position_filter_enable;
}

int motor_info_set_position_filter_enable(motor_info_t *cfg, uint32_t value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value > (uint32_t)1)
		return -EINVAL; /* 越界 */
	cfg->blocks.control.position_filter_enable = value;
	return 0;
}

float motor_info_get_following_error_limit(const motor_info_t *cfg)
{
	return cfg->blocks.control.following_error_limit;
}

int motor_info_set_following_error_limit(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < 0.0f || value > 1000000.0f)
		return -EINVAL; /* 越界 */
	cfg->blocks.control.following_error_limit = value;
	return 0;
}

/******************************************************************************
 * 保护与通信参数
 ******************************************************************************/
float motor_info_get_over_current_A(const motor_info_t *cfg)
{
	return cfg->blocks.protect_comm.over_current_A;
}

int motor_info_set_over_current_A(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < 0.1f || value > 1000.0f)
		return -EINVAL; /* 越界 */
	cfg->blocks.protect_comm.over_current_A = value;
	return 0;
}

float motor_info_get_over_voltage_V(const motor_info_t *cfg)
{
	return cfg->blocks.protect_comm.over_voltage_V;
}

int motor_info_set_over_voltage_V(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < 5.0f || value > 120.0f)
		return -EINVAL; /* 越界 */
	cfg->blocks.protect_comm.over_voltage_V = value;
	return 0;
}

float motor_info_get_under_voltage_V(const motor_info_t *cfg)
{
	return cfg->blocks.protect_comm.under_voltage_V;
}

int motor_info_set_under_voltage_V(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < 0.0f || value > 60.0f)
		return -EINVAL; /* 越界 */
	cfg->blocks.protect_comm.under_voltage_V = value;
	return 0;
}

float motor_info_get_over_temp_drive(const motor_info_t *cfg)
{
	return cfg->blocks.protect_comm.over_temp_drive;
}

int motor_info_set_over_temp_drive(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < 20.0f || value > 150.0f)
		return -EINVAL; /* 越界 */
	cfg->blocks.protect_comm.over_temp_drive = value;
	return 0;
}

float motor_info_get_over_temp_motor(const motor_info_t *cfg)
{
	return cfg->blocks.protect_comm.over_temp_motor;
}

int motor_info_set_over_temp_motor(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < 20.0f || value > 200.0f)
		return -EINVAL; /* 越界 */
	cfg->blocks.protect_comm.over_temp_motor = value;
	return 0;
}

float motor_info_get_under_temp_d(const motor_info_t *cfg)
{
	return cfg->blocks.protect_comm.under_temp_d;
}

int motor_info_set_under_temp_d(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < -80.0f || value > 0.0f)
		return -EINVAL; /* 越界 */
	cfg->blocks.protect_comm.under_temp_d = value;
	return 0;
}

float motor_info_get_over_speed_rad_s(const motor_info_t *cfg)
{
	return cfg->blocks.protect_comm.over_speed_rad_s;
}

int motor_info_set_over_speed_rad_s(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < 1.0f || value > 20000.0f)
		return -EINVAL; /* 越界 */
	cfg->blocks.protect_comm.over_speed_rad_s = value;
	return 0;
}

int32_t motor_info_get_position_following_error_p(const motor_info_t *cfg)
{
	return cfg->blocks.protect_comm.position_following_error_p;
}

int motor_info_set_position_following_error_p(motor_info_t *cfg, int32_t value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < (int32_t)1 || value > (int32_t)1000000)
		return -EINVAL; /* 越界 */
	cfg->blocks.protect_comm.position_following_error_p = value;
	return 0;
}

int32_t motor_info_get_pos_limit_min(const motor_info_t *cfg)
{
	return cfg->blocks.protect_comm.pos_limit_min;
}

int motor_info_set_pos_limit_min(motor_info_t *cfg, int32_t value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < (int32_t)-10000000 || value > (int32_t)0)
		return -EINVAL; /* 越界 */
	cfg->blocks.protect_comm.pos_limit_min = value;
	return 0;
}

int32_t motor_info_get_pos_limit_max(const motor_info_t *cfg)
{
	return cfg->blocks.protect_comm.pos_limit_max;
}

int motor_info_set_pos_limit_max(motor_info_t *cfg, int32_t value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < (int32_t)0 || value > (int32_t)10000000)
		return -EINVAL; /* 越界 */
	cfg->blocks.protect_comm.pos_limit_max = value;
	return 0;
}

uint32_t motor_info_get_error_enable_mask(const motor_info_t *cfg)
{
	return cfg->blocks.protect_comm.error_enable_mask;
}

int motor_info_set_error_enable_mask(motor_info_t *cfg, uint32_t value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value > (uint32_t)4294967295)
		return -EINVAL; /* 越界 */
	cfg->blocks.protect_comm.error_enable_mask = value;
	return 0;
}

/******************************************************************************
 * 高级算法参数（MIT/力控/回零）
 ******************************************************************************/
float motor_info_get_mit_kp(const motor_info_t *cfg)
{
	return cfg->blocks.advanced.mit_kp;
}

int motor_info_set_mit_kp(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < 0.0f || value > 10000.0f)
		return -EINVAL; /* 越界 */
	cfg->blocks.advanced.mit_kp = value;
	return 0;
}

float motor_info_get_mit_kd(const motor_info_t *cfg)
{
	return cfg->blocks.advanced.mit_kd;
}

int motor_info_set_mit_kd(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < 0.0f || value > 1000.0f)
		return -EINVAL; /* 越界 */
	cfg->blocks.advanced.mit_kd = value;
	return 0;
}

float motor_info_get_mit_max_current(const motor_info_t *cfg)
{
	return cfg->blocks.advanced.mit_max_current;
}

int motor_info_set_mit_max_current(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < 0.0f || value > 500.0f)
		return -EINVAL; /* 越界 */
	cfg->blocks.advanced.mit_max_current = value;
	return 0;
}

float motor_info_get_mit_feedforward_torque(const motor_info_t *cfg)
{
	return cfg->blocks.advanced.mit_feedforward_torque;
}

int motor_info_set_mit_feedforward_torque(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < 0.0f || value > 500.0f)
		return -EINVAL; /* 越界 */
	cfg->blocks.advanced.mit_feedforward_torque = value;
	return 0;
}

float motor_info_get_force_kp(const motor_info_t *cfg)
{
	return cfg->blocks.advanced.force_kp;
}

int motor_info_set_force_kp(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < 0.0f || value > 10000.0f)
		return -EINVAL; /* 越界 */
	cfg->blocks.advanced.force_kp = value;
	return 0;
}

float motor_info_get_force_ki(const motor_info_t *cfg)
{
	return cfg->blocks.advanced.force_ki;
}

int motor_info_set_force_ki(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < 0.0f || value > 10000.0f)
		return -EINVAL; /* 越界 */
	cfg->blocks.advanced.force_ki = value;
	return 0;
}

float motor_info_get_force_limit(const motor_info_t *cfg)
{
	return cfg->blocks.advanced.force_limit;
}

int motor_info_set_force_limit(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < 0.0f || value > 1000.0f)
		return -EINVAL; /* 越界 */
	cfg->blocks.advanced.force_limit = value;
	return 0;
}

uint32_t motor_info_get_force_control_enable(const motor_info_t *cfg)
{
	return cfg->blocks.advanced.force_control_enable;
}

int motor_info_set_force_control_enable(motor_info_t *cfg, uint32_t value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value > (uint32_t)1)
		return -EINVAL; /* 越界 */
	cfg->blocks.advanced.force_control_enable = value;
	return 0;
}

uint32_t motor_info_get_homing_method(const motor_info_t *cfg)
{
	return cfg->blocks.advanced.homing_method;
}

int motor_info_set_homing_method(motor_info_t *cfg, uint32_t value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value > (uint32_t)10)
		return -EINVAL; /* 越界 */
	cfg->blocks.advanced.homing_method = value;
	return 0;
}

float motor_info_get_homing_speed(const motor_info_t *cfg)
{
	return cfg->blocks.advanced.homing_speed;
}

int motor_info_set_homing_speed(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < 0.001f || value > 500.0f)
		return -EINVAL; /* 越界 */
	cfg->blocks.advanced.homing_speed = value;
	return 0;
}

float motor_info_get_homing_offset(const motor_info_t *cfg)
{
	return cfg->blocks.advanced.homing_offset;
}

int motor_info_set_homing_offset(motor_info_t *cfg, float value)
{
	if (cfg == NULL)
		return -EINVAL;
	if (value < -6.283f || value > 6.283f)
		return -EINVAL; /* 越界 */
	cfg->blocks.advanced.homing_offset = value;
	return 0;
}

/******************************************************************************
 * 协议分发表实现 (param_id -> get/set), 供 0xE6-0xEB 单参读写
 ******************************************************************************/

int motor_info_dispatch_read(uint16_t pid, const motor_info_t *cfg, uint8_t out4[4], uint8_t *out_type, uint8_t *out_len)
{
	if (cfg == NULL || out4 == NULL || out_type == NULL || out_len == NULL)
		return -1;
	/* 固定4B, 先清零(短类型高字节补零) */
	out4[0] = out4[1] = out4[2] = out4[3] = 0;
	switch (pid)
	{
		case 0:
		{ /* config_version (uint32_t) */
			uint32_t v = motor_info_get_config_version(cfg);
			memcpy(out4, &v, 4);
			*out_type = 4;
			*out_len = 4;
			return 0;
		}
		case 1:
		{ /* enable_uart (uint32_t) */
			uint32_t v = motor_info_get_enable_uart(cfg);
			memcpy(out4, &v, 4);
			*out_type = 4;
			*out_len = 4;
			return 0;
		}
		case 2:
		{ /* enable_bus_sensor (uint32_t) */
			uint32_t v = motor_info_get_enable_bus_sensor(cfg);
			memcpy(out4, &v, 4);
			*out_type = 4;
			*out_len = 4;
			return 0;
		}
		case 3:
		{ /* safety_limit (uint32_t) */
			uint32_t v = motor_info_get_safety_limit(cfg);
			memcpy(out4, &v, 4);
			*out_type = 4;
			*out_len = 4;
			return 0;
		}
		case 4:
		{ /* total_runtime_s (uint32_t) */
			uint32_t v = motor_info_get_total_runtime_s(cfg);
			memcpy(out4, &v, 4);
			*out_type = 4;
			*out_len = 4;
			return 0;
		}
		case 16:
		{ /* is_calibrated (uint32_t) */
			uint32_t v = motor_info_get_is_calibrated(cfg);
			memcpy(out4, &v, 4);
			*out_type = 4;
			*out_len = 4;
			return 0;
		}
		case 17:
		{ /* pole_pairs (uint32_t) */
			uint32_t v = motor_info_get_pole_pairs(cfg);
			memcpy(out4, &v, 4);
			*out_type = 4;
			*out_len = 4;
			return 0;
		}
		case 18:
		{ /* motor_type (uint32_t) */
			uint32_t v = motor_info_get_motor_type(cfg);
			memcpy(out4, &v, 4);
			*out_type = 4;
			*out_len = 4;
			return 0;
		}
		case 19:
		{ /* direction (uint32_t) */
			uint32_t v = motor_info_get_direction(cfg);
			memcpy(out4, &v, 4);
			*out_type = 4;
			*out_len = 4;
			return 0;
		}
		case 20:
		{ /* phase_resistance (float) */
			float v = motor_info_get_phase_resistance(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 21:
		{ /* phase_inductance_d (float) */
			float v = motor_info_get_phase_inductance_d(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 22:
		{ /* phase_inductance_q (float) */
			float v = motor_info_get_phase_inductance_q(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 23:
		{ /* flux_linkage (float) */
			float v = motor_info_get_flux_linkage(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 24:
		{ /* torque_constant (float) */
			float v = motor_info_get_torque_constant(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 25:
		{ /* rotor_inertia (float) */
			float v = motor_info_get_rotor_inertia(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 26:
		{ /* friction_coulomb (float) */
			float v = motor_info_get_friction_coulomb(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 27:
		{ /* friction_viscous (float) */
			float v = motor_info_get_friction_viscous(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 28:
		{ /* gear_ratio (float) */
			float v = motor_info_get_gear_ratio(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 29:
		{ /* gear_efficiency (float) */
			float v = motor_info_get_gear_efficiency(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 30:
		{ /* calibration_current (float) */
			float v = motor_info_get_calibration_current(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 31:
		{ /* resistance_calib_max_voltage (float) */
			float v = motor_info_get_resistance_calib_max_voltage(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 32:
		{ /* current_lim (float) */
			float v = motor_info_get_current_lim(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 33:
		{ /* current_control_bandwidth (float) */
			float v = motor_info_get_current_control_bandwidth(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 34:
		{ /* enc_type (uint32_t) */
			uint32_t v = motor_info_get_enc_type(cfg);
			memcpy(out4, &v, 4);
			*out_type = 4;
			*out_len = 4;
			return 0;
		}
		case 35:
		{ /* enc_lines (uint32_t) */
			uint32_t v = motor_info_get_enc_lines(cfg);
			memcpy(out4, &v, 4);
			*out_type = 4;
			*out_len = 4;
			return 0;
		}
		case 36:
		{ /* enc_direction (int32_t) */
			int32_t v = motor_info_get_enc_direction(cfg);
			memcpy(out4, &v, 4);
			*out_type = 5;
			*out_len = 4;
			return 0;
		}
		case 37:
		{ /* enc_offset (float) */
			float v = motor_info_get_enc_offset(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 38:
		{ /* elec_angle_bias (float) */
			float v = motor_info_get_elec_angle_bias(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 39:
		{ /* pwm_freq_hz (uint32_t) */
			uint32_t v = motor_info_get_pwm_freq_hz(cfg);
			memcpy(out4, &v, 4);
			*out_type = 4;
			*out_len = 4;
			return 0;
		}
		case 40:
		{ /* dead_time_ns (float) */
			float v = motor_info_get_dead_time_ns(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 41:
		{ /* shunt_resistance (float) */
			float v = motor_info_get_shunt_resistance(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 42:
		{ /* current_amp_gain (float) */
			float v = motor_info_get_current_amp_gain(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 48:
		{ /* device_zero (float) */
			float v = motor_info_get_device_zero(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 49:
		{ /* device_time (uint32_t) */
			uint32_t v = motor_info_get_device_time(cfg);
			memcpy(out4, &v, 4);
			*out_type = 4;
			*out_len = 4;
			return 0;
		}
		case 50:
		{ /* can_id (uint32_t) */
			uint32_t v = motor_info_get_can_id(cfg);
			memcpy(out4, &v, 4);
			*out_type = 4;
			*out_len = 4;
			return 0;
		}
		case 51:
		{ /* can_baudrate (uint32_t) */
			uint32_t v = motor_info_get_can_baudrate(cfg);
			memcpy(out4, &v, 4);
			*out_type = 4;
			*out_len = 4;
			return 0;
		}
		case 52:
		{ /* can_timeout_s (float) */
			float v = motor_info_get_can_timeout_s(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 53:
		{ /* can_fd_enable (uint32_t) */
			uint32_t v = motor_info_get_can_fd_enable(cfg);
			memcpy(out4, &v, 4);
			*out_type = 4;
			*out_len = 4;
			return 0;
		}
		case 54:
		{ /* can_fd_baudrate (uint32_t) */
			uint32_t v = motor_info_get_can_fd_baudrate(cfg);
			memcpy(out4, &v, 4);
			*out_type = 4;
			*out_len = 4;
			return 0;
		}
		case 55:
		{ /* uart_baudrate (uint32_t) */
			uint32_t v = motor_info_get_uart_baudrate(cfg);
			memcpy(out4, &v, 4);
			*out_type = 4;
			*out_len = 4;
			return 0;
		}
		case 64:
		{ /* kp_ld (float) */
			float v = motor_info_get_kp_ld(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 65:
		{ /* ki_ld (float) */
			float v = motor_info_get_ki_ld(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 66:
		{ /* kp_lq (float) */
			float v = motor_info_get_kp_lq(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 67:
		{ /* ki_lq (float) */
			float v = motor_info_get_ki_lq(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 68:
		{ /* integral_limit (float) */
			float v = motor_info_get_integral_limit(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 69:
		{ /* decoupling_gain (float) */
			float v = motor_info_get_decoupling_gain(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 70:
		{ /* comp_du_V (float) */
			float v = motor_info_get_comp_du_V(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 71:
		{ /* pwm_duty_max (float) */
			float v = motor_info_get_pwm_duty_max(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 72:
		{ /* kp_s (float) */
			float v = motor_info_get_kp_s(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 73:
		{ /* ki_s (float) */
			float v = motor_info_get_ki_s(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 74:
		{ /* speed_integral_limit (float) */
			float v = motor_info_get_speed_integral_limit(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 75:
		{ /* vff (float) */
			float v = motor_info_get_vff(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 76:
		{ /* aff (float) */
			float v = motor_info_get_aff(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 77:
		{ /* jerk_ff (float) */
			float v = motor_info_get_jerk_ff(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 78:
		{ /* speed_filter_alpha (float) */
			float v = motor_info_get_speed_filter_alpha(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 79:
		{ /* speed_filter_enable (uint32_t) */
			uint32_t v = motor_info_get_speed_filter_enable(cfg);
			memcpy(out4, &v, 4);
			*out_type = 4;
			*out_len = 4;
			return 0;
		}
		case 80:
		{ /* kp_p (float) */
			float v = motor_info_get_kp_p(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 81:
		{ /* ki_p (float) */
			float v = motor_info_get_ki_p(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 82:
		{ /* position_integral_limit (float) */
			float v = motor_info_get_position_integral_limit(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 83:
		{ /* position_filter_alpha (float) */
			float v = motor_info_get_position_filter_alpha(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 84:
		{ /* position_filter_enable (uint32_t) */
			uint32_t v = motor_info_get_position_filter_enable(cfg);
			memcpy(out4, &v, 4);
			*out_type = 4;
			*out_len = 4;
			return 0;
		}
		case 85:
		{ /* following_error_limit (float) */
			float v = motor_info_get_following_error_limit(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 128:
		{ /* over_current_A (float) */
			float v = motor_info_get_over_current_A(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 129:
		{ /* over_voltage_V (float) */
			float v = motor_info_get_over_voltage_V(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 130:
		{ /* under_voltage_V (float) */
			float v = motor_info_get_under_voltage_V(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 131:
		{ /* over_temp_drive (float) */
			float v = motor_info_get_over_temp_drive(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 132:
		{ /* over_temp_motor (float) */
			float v = motor_info_get_over_temp_motor(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 133:
		{ /* under_temp_d (float) */
			float v = motor_info_get_under_temp_d(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 134:
		{ /* over_speed_rad_s (float) */
			float v = motor_info_get_over_speed_rad_s(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 135:
		{ /* position_following_error_p (int32_t) */
			int32_t v = motor_info_get_position_following_error_p(cfg);
			memcpy(out4, &v, 4);
			*out_type = 5;
			*out_len = 4;
			return 0;
		}
		case 136:
		{ /* pos_limit_min (int32_t) */
			int32_t v = motor_info_get_pos_limit_min(cfg);
			memcpy(out4, &v, 4);
			*out_type = 5;
			*out_len = 4;
			return 0;
		}
		case 137:
		{ /* pos_limit_max (int32_t) */
			int32_t v = motor_info_get_pos_limit_max(cfg);
			memcpy(out4, &v, 4);
			*out_type = 5;
			*out_len = 4;
			return 0;
		}
		case 138:
		{ /* error_enable_mask (uint32_t) */
			uint32_t v = motor_info_get_error_enable_mask(cfg);
			memcpy(out4, &v, 4);
			*out_type = 4;
			*out_len = 4;
			return 0;
		}
		case 160:
		{ /* mit_kp (float) */
			float v = motor_info_get_mit_kp(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 161:
		{ /* mit_kd (float) */
			float v = motor_info_get_mit_kd(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 162:
		{ /* mit_max_current (float) */
			float v = motor_info_get_mit_max_current(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 163:
		{ /* mit_feedforward_torque (float) */
			float v = motor_info_get_mit_feedforward_torque(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 164:
		{ /* force_kp (float) */
			float v = motor_info_get_force_kp(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 165:
		{ /* force_ki (float) */
			float v = motor_info_get_force_ki(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 166:
		{ /* force_limit (float) */
			float v = motor_info_get_force_limit(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 167:
		{ /* force_control_enable (uint32_t) */
			uint32_t v = motor_info_get_force_control_enable(cfg);
			memcpy(out4, &v, 4);
			*out_type = 4;
			*out_len = 4;
			return 0;
		}
		case 168:
		{ /* homing_method (uint32_t) */
			uint32_t v = motor_info_get_homing_method(cfg);
			memcpy(out4, &v, 4);
			*out_type = 4;
			*out_len = 4;
			return 0;
		}
		case 169:
		{ /* homing_speed (float) */
			float v = motor_info_get_homing_speed(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		case 170:
		{ /* homing_offset (float) */
			float v = motor_info_get_homing_offset(cfg);
			memcpy(out4, &v, 4);
			*out_type = 6;
			*out_len = 4;
			return 0;
		}
		default:
			return -1;
	}
}

int motor_info_dispatch_write(uint16_t pid, motor_info_t *cfg, const uint8_t in4[4], uint8_t len)
{
	(void)len; /* 帧内固定4B, 仅取低4B */
	if (cfg == NULL || in4 == NULL)
		return -1;
	switch (pid)
	{
		case 0:
		{ /* config_version (uint32_t) */
			uint32_t v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_config_version(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 1:
		{ /* enable_uart (uint32_t) */
			uint32_t v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_enable_uart(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 2:
		{ /* enable_bus_sensor (uint32_t) */
			uint32_t v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_enable_bus_sensor(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 3:
		{ /* safety_limit (uint32_t) */
			uint32_t v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_safety_limit(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 4:
		{ /* total_runtime_s (uint32_t) */
			uint32_t v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_total_runtime_s(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 16:
		{ /* is_calibrated (uint32_t) */
			uint32_t v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_is_calibrated(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 17:
		{ /* pole_pairs (uint32_t) */
			uint32_t v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_pole_pairs(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 18:
		{ /* motor_type (uint32_t) */
			uint32_t v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_motor_type(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 19:
		{ /* direction (uint32_t) */
			uint32_t v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_direction(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 20:
		{ /* phase_resistance (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_phase_resistance(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 21:
		{ /* phase_inductance_d (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_phase_inductance_d(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 22:
		{ /* phase_inductance_q (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_phase_inductance_q(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 23:
		{ /* flux_linkage (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_flux_linkage(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 24:
		{ /* torque_constant (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_torque_constant(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 25:
		{ /* rotor_inertia (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_rotor_inertia(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 26:
		{ /* friction_coulomb (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_friction_coulomb(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 27:
		{ /* friction_viscous (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_friction_viscous(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 28:
		{ /* gear_ratio (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_gear_ratio(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 29:
		{ /* gear_efficiency (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_gear_efficiency(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 30:
		{ /* calibration_current (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_calibration_current(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 31:
		{ /* resistance_calib_max_voltage (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_resistance_calib_max_voltage(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 32:
		{ /* current_lim (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_current_lim(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 33:
		{ /* current_control_bandwidth (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_current_control_bandwidth(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 34:
		{ /* enc_type (uint32_t) */
			uint32_t v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_enc_type(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 35:
		{ /* enc_lines (uint32_t) */
			uint32_t v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_enc_lines(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 36:
		{ /* enc_direction (int32_t) */
			int32_t v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_enc_direction(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 37:
		{ /* enc_offset (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_enc_offset(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 38:
		{ /* elec_angle_bias (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_elec_angle_bias(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 39:
		{ /* pwm_freq_hz (uint32_t) */
			uint32_t v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_pwm_freq_hz(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 40:
		{ /* dead_time_ns (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_dead_time_ns(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 41:
		{ /* shunt_resistance (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_shunt_resistance(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 42:
		{ /* current_amp_gain (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_current_amp_gain(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 48:
		{ /* device_zero (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_device_zero(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 49:
		{ /* device_time (uint32_t) */
			uint32_t v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_device_time(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 50:
		{ /* can_id (uint32_t) */
			uint32_t v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_can_id(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 51:
		{ /* can_baudrate (uint32_t) */
			uint32_t v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_can_baudrate(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 52:
		{ /* can_timeout_s (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_can_timeout_s(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 53:
		{ /* can_fd_enable (uint32_t) */
			uint32_t v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_can_fd_enable(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 54:
		{ /* can_fd_baudrate (uint32_t) */
			uint32_t v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_can_fd_baudrate(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 55:
		{ /* uart_baudrate (uint32_t) */
			uint32_t v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_uart_baudrate(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 64:
		{ /* kp_ld (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_kp_ld(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 65:
		{ /* ki_ld (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_ki_ld(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 66:
		{ /* kp_lq (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_kp_lq(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 67:
		{ /* ki_lq (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_ki_lq(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 68:
		{ /* integral_limit (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_integral_limit(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 69:
		{ /* decoupling_gain (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_decoupling_gain(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 70:
		{ /* comp_du_V (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_comp_du_V(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 71:
		{ /* pwm_duty_max (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_pwm_duty_max(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 72:
		{ /* kp_s (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_kp_s(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 73:
		{ /* ki_s (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_ki_s(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 74:
		{ /* speed_integral_limit (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_speed_integral_limit(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 75:
		{ /* vff (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_vff(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 76:
		{ /* aff (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_aff(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 77:
		{ /* jerk_ff (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_jerk_ff(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 78:
		{ /* speed_filter_alpha (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_speed_filter_alpha(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 79:
		{ /* speed_filter_enable (uint32_t) */
			uint32_t v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_speed_filter_enable(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 80:
		{ /* kp_p (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_kp_p(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 81:
		{ /* ki_p (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_ki_p(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 82:
		{ /* position_integral_limit (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_position_integral_limit(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 83:
		{ /* position_filter_alpha (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_position_filter_alpha(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 84:
		{ /* position_filter_enable (uint32_t) */
			uint32_t v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_position_filter_enable(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 85:
		{ /* following_error_limit (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_following_error_limit(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 128:
		{ /* over_current_A (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_over_current_A(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 129:
		{ /* over_voltage_V (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_over_voltage_V(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 130:
		{ /* under_voltage_V (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_under_voltage_V(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 131:
		{ /* over_temp_drive (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_over_temp_drive(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 132:
		{ /* over_temp_motor (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_over_temp_motor(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 133:
		{ /* under_temp_d (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_under_temp_d(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 134:
		{ /* over_speed_rad_s (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_over_speed_rad_s(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 135:
		{ /* position_following_error_p (int32_t) */
			int32_t v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_position_following_error_p(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 136:
		{ /* pos_limit_min (int32_t) */
			int32_t v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_pos_limit_min(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 137:
		{ /* pos_limit_max (int32_t) */
			int32_t v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_pos_limit_max(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 138:
		{ /* error_enable_mask (uint32_t) */
			uint32_t v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_error_enable_mask(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 160:
		{ /* mit_kp (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_mit_kp(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 161:
		{ /* mit_kd (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_mit_kd(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 162:
		{ /* mit_max_current (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_mit_max_current(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 163:
		{ /* mit_feedforward_torque (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_mit_feedforward_torque(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 164:
		{ /* force_kp (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_force_kp(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 165:
		{ /* force_ki (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_force_ki(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 166:
		{ /* force_limit (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_force_limit(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 167:
		{ /* force_control_enable (uint32_t) */
			uint32_t v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_force_control_enable(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 168:
		{ /* homing_method (uint32_t) */
			uint32_t v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_homing_method(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 169:
		{ /* homing_speed (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_homing_speed(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		case 170:
		{ /* homing_offset (float) */
			float v;
			memcpy(&v, in4, 4);
			int rc = motor_info_set_homing_offset(cfg, v);
			return (rc == 0) ? 0 : -2;
		}
		default:
			return -1;
	}
}
