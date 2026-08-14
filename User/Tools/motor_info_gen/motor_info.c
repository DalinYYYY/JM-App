/**
 * @file    motor_info.c
 * @brief   MotorInfo 配置参数 API 实现
 * @date    2026-08-14
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
#include <stddef.h>

int motor_info_init(motor_info_t *cfg)
{
    if (cfg == NULL) return -EINVAL;
    memset(cfg->raw, 0, PARAM_AREA_SIZE);

    /* ---- 头部 ---- */
    cfg->blocks.header.version_major = MOTOR_INFO_VERSION_MAJOR;
    cfg->blocks.header.version_minor = MOTOR_INFO_VERSION_MINOR;
    cfg->blocks.header.crc32 = 0;
    cfg->blocks.header.blocks[0].offset = 0x0040u;
    cfg->blocks.header.blocks[0].size   = 64u;
    cfg->blocks.header.blocks[1].offset = 0x0080u;
    cfg->blocks.header.blocks[1].size   = 128u;
    cfg->blocks.header.blocks[2].offset = 0x0100u;
    cfg->blocks.header.blocks[2].size   = 64u;
    cfg->blocks.header.blocks[3].offset = 0x0140u;
    cfg->blocks.header.blocks[3].size   = 320u;
    cfg->blocks.header.blocks[4].offset = 0x0280u;
    cfg->blocks.header.blocks[4].size   = 128u;
    cfg->blocks.header.blocks[5].offset = 0x0300u;
    cfg->blocks.header.blocks[5].size   = 64u;
    cfg->blocks.header.reserved[0] = 0;
    cfg->blocks.header.reserved[1] = 0;

    /* ---- 系统级参数 ---- */
    cfg->blocks.system.config_version = 65536U;
    cfg->blocks.system.enable_uart = 0U;
    cfg->blocks.system.enable_bus_sensor = 1U;
    cfg->blocks.system.safety_limit = 1U;
    cfg->blocks.system.total_runtime_s = 0U;
    cfg->blocks.system.flash_write_count = 0U;

    /* ---- 电机标定参数（含减速器/编码器/功率级/电流采样） ---- */
    cfg->blocks.motor_calib.is_calibrated = 0U;
    cfg->blocks.motor_calib.pole_pairs = 7U;
    cfg->blocks.motor_calib.motor_type = 0U;
    cfg->blocks.motor_calib.direction = 0U;
    cfg->blocks.motor_calib.phase_resistance = 3.6f;
    cfg->blocks.motor_calib.phase_inductance_d = 0.0048f;
    cfg->blocks.motor_calib.phase_inductance_q = 0.0048f;
    cfg->blocks.motor_calib.flux_linkage = 0.02f;
    cfg->blocks.motor_calib.torque_constant = 0.21f;
    cfg->blocks.motor_calib.rotor_inertia = 0.00001f;
    cfg->blocks.motor_calib.friction_coulomb = 0.0f;
    cfg->blocks.motor_calib.friction_viscous = 0.0f;
    cfg->blocks.motor_calib.gear_ratio = 100.0f;
    cfg->blocks.motor_calib.gear_efficiency = 0.85f;
    cfg->blocks.motor_calib.calibration_current = 0.5f;
    cfg->blocks.motor_calib.resistance_calib_max_voltage = 3.0f;
    cfg->blocks.motor_calib.current_lim = 3.7f;
    cfg->blocks.motor_calib.current_control_bandwidth = 500.0f;
    cfg->blocks.motor_calib.enc_type = 1U;
    cfg->blocks.motor_calib.enc_lines = 16384U;
    cfg->blocks.motor_calib.enc_direction = 1;
    cfg->blocks.motor_calib.enc_offset = 0.0f;
    cfg->blocks.motor_calib.elec_angle_bias = 0.0f;
    cfg->blocks.motor_calib.pwm_freq_hz = 20000U;
    cfg->blocks.motor_calib.dead_time_ns = 500.0f;
    cfg->blocks.motor_calib.shunt_resistance = 0.01f;
    cfg->blocks.motor_calib.current_amp_gain = 50.0f;
    cfg->blocks.motor_calib.peak_current = 3.7f;
    cfg->blocks.motor_calib.max_speed = 200.0f;

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
    cfg->blocks.control.decouple_algo = 1U;
    cfg->blocks.control.bemf_ff_enable = 1U;
    cfg->blocks.control.deadtime_comp_enable = 1U;
    cfg->blocks.control.pid_source_mask = 0U;

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
    if (cfg == NULL) return -EINVAL;

    /* 系统级参数 */
    if (cfg->blocks.system.config_version > (uint32_t)4294967295) return 0;  /* config_version */
    if (cfg->blocks.system.enable_uart > (uint32_t)15) return 1;  /* enable_uart */
    if (cfg->blocks.system.enable_bus_sensor > (uint32_t)1) return 2;  /* enable_bus_sensor */
    if (cfg->blocks.system.safety_limit > (uint32_t)1) return 3;  /* safety_limit */
    if (cfg->blocks.system.total_runtime_s > (uint32_t)4294967295) return 4;  /* total_runtime_s */
    if (cfg->blocks.system.flash_write_count > (uint32_t)4294967295) return 5;  /* flash_write_count */

    /* 电机标定参数（含减速器/编码器/功率级/电流采样） */
    if (cfg->blocks.motor_calib.is_calibrated > (uint32_t)1) return 16;  /* is_calibrated */
    if (cfg->blocks.motor_calib.pole_pairs < (uint32_t)1 || cfg->blocks.motor_calib.pole_pairs > (uint32_t)128) return 17;  /* pole_pairs */
    if (cfg->blocks.motor_calib.motor_type > (uint32_t)2) return 18;  /* motor_type */
    if (cfg->blocks.motor_calib.direction > (uint32_t)1) return 19;  /* direction */
    if (cfg->blocks.motor_calib.phase_resistance < 0.001f || cfg->blocks.motor_calib.phase_resistance > 50.0f) return 20;  /* phase_resistance */
    if (cfg->blocks.motor_calib.phase_inductance_d < 0.000001f || cfg->blocks.motor_calib.phase_inductance_d > 0.1f) return 21;  /* phase_inductance_d */
    if (cfg->blocks.motor_calib.phase_inductance_q < 0.000001f || cfg->blocks.motor_calib.phase_inductance_q > 0.1f) return 22;  /* phase_inductance_q */
    if (cfg->blocks.motor_calib.flux_linkage < 0.00001f || cfg->blocks.motor_calib.flux_linkage > 1.0f) return 23;  /* flux_linkage */
    if (cfg->blocks.motor_calib.torque_constant < 0.00001f || cfg->blocks.motor_calib.torque_constant > 50.0f) return 24;  /* torque_constant */
    if (cfg->blocks.motor_calib.rotor_inertia < 0.0000001f || cfg->blocks.motor_calib.rotor_inertia > 1.0f) return 25;  /* rotor_inertia */
    if (cfg->blocks.motor_calib.friction_coulomb < 0.0f || cfg->blocks.motor_calib.friction_coulomb > 500.0f) return 26;  /* friction_coulomb */
    if (cfg->blocks.motor_calib.friction_viscous < 0.0f || cfg->blocks.motor_calib.friction_viscous > 100.0f) return 27;  /* friction_viscous */
    if (cfg->blocks.motor_calib.gear_ratio < 1.0f || cfg->blocks.motor_calib.gear_ratio > 10000.0f) return 28;  /* gear_ratio */
    if (cfg->blocks.motor_calib.gear_efficiency < 0.1f || cfg->blocks.motor_calib.gear_efficiency > 1.0f) return 29;  /* gear_efficiency */
    if (cfg->blocks.motor_calib.calibration_current < 0.1f || cfg->blocks.motor_calib.calibration_current > 50.0f) return 30;  /* calibration_current */
    if (cfg->blocks.motor_calib.resistance_calib_max_voltage < 0.1f || cfg->blocks.motor_calib.resistance_calib_max_voltage > 60.0f) return 31;  /* resistance_calib_max_voltage */
    if (cfg->blocks.motor_calib.current_lim < 0.1f || cfg->blocks.motor_calib.current_lim > 500.0f) return 32;  /* current_lim */
    if (cfg->blocks.motor_calib.current_control_bandwidth < 50.0f || cfg->blocks.motor_calib.current_control_bandwidth > 20000.0f) return 33;  /* current_control_bandwidth */
    if (cfg->blocks.motor_calib.enc_type > (uint32_t)10) return 34;  /* enc_type */
    if (cfg->blocks.motor_calib.enc_lines < (uint32_t)100 || cfg->blocks.motor_calib.enc_lines > (uint32_t)10000000) return 35;  /* enc_lines */
    if (cfg->blocks.motor_calib.enc_direction < (int32_t)-1 || cfg->blocks.motor_calib.enc_direction > (int32_t)1) return 36;  /* enc_direction */
    if (cfg->blocks.motor_calib.enc_offset < -360.0f || cfg->blocks.motor_calib.enc_offset > 360.0f) return 37;  /* enc_offset */
    if (cfg->blocks.motor_calib.elec_angle_bias < -3.1416f || cfg->blocks.motor_calib.elec_angle_bias > 3.1416f) return 38;  /* elec_angle_bias */
    if (cfg->blocks.motor_calib.pwm_freq_hz < (uint32_t)1000 || cfg->blocks.motor_calib.pwm_freq_hz > (uint32_t)200000) return 39;  /* pwm_freq_hz */
    if (cfg->blocks.motor_calib.dead_time_ns < 50.0f || cfg->blocks.motor_calib.dead_time_ns > 5000.0f) return 40;  /* dead_time_ns */
    if (cfg->blocks.motor_calib.shunt_resistance < 0.0001f || cfg->blocks.motor_calib.shunt_resistance > 1.0f) return 41;  /* shunt_resistance */
    if (cfg->blocks.motor_calib.current_amp_gain < 1.0f || cfg->blocks.motor_calib.current_amp_gain > 10000.0f) return 42;  /* current_amp_gain */
    if (cfg->blocks.motor_calib.peak_current < 0.1f || cfg->blocks.motor_calib.peak_current > 200.0f) return 43;  /* peak_current */
    if (cfg->blocks.motor_calib.max_speed < 10.0f || cfg->blocks.motor_calib.max_speed > 1000.0f) return 44;  /* max_speed */

    /* 设备参数（CAN/UART） */
    if (cfg->blocks.device.device_zero < -12.566f || cfg->blocks.device.device_zero > 12.566f) return 48;  /* device_zero */
    if (cfg->blocks.device.device_time > (uint32_t)99999999) return 49;  /* device_time */
    if (cfg->blocks.device.can_id < (uint32_t)1 || cfg->blocks.device.can_id > (uint32_t)127) return 50;  /* can_id */
    if (cfg->blocks.device.can_baudrate < (uint32_t)10000 || cfg->blocks.device.can_baudrate > (uint32_t)8000000) return 51;  /* can_baudrate */
    if (cfg->blocks.device.can_timeout_s < 0.0f || cfg->blocks.device.can_timeout_s > 60.0f) return 52;  /* can_timeout_s */
    if (cfg->blocks.device.can_fd_enable > (uint32_t)1) return 53;  /* can_fd_enable */
    if (cfg->blocks.device.can_fd_baudrate < (uint32_t)100000 || cfg->blocks.device.can_fd_baudrate > (uint32_t)8000000) return 54;  /* can_fd_baudrate */
    if (cfg->blocks.device.uart_baudrate < (uint32_t)1200 || cfg->blocks.device.uart_baudrate > (uint32_t)8000000) return 55;  /* uart_baudrate */

    /* 控制参数（三环PID+前馈+滤波） */
    if (cfg->blocks.control.kp_ld < 0.0f || cfg->blocks.control.kp_ld > 1000.0f) return 64;  /* kp_ld */
    if (cfg->blocks.control.ki_ld < 0.0f || cfg->blocks.control.ki_ld > 100000.0f) return 65;  /* ki_ld */
    if (cfg->blocks.control.kp_lq < 0.0f || cfg->blocks.control.kp_lq > 1000.0f) return 66;  /* kp_lq */
    if (cfg->blocks.control.ki_lq < 0.0f || cfg->blocks.control.ki_lq > 100000.0f) return 67;  /* ki_lq */
    if (cfg->blocks.control.integral_limit < 0.0f || cfg->blocks.control.integral_limit > 1000.0f) return 68;  /* integral_limit */
    if (cfg->blocks.control.decoupling_gain < 0.0f || cfg->blocks.control.decoupling_gain > 5.0f) return 69;  /* decoupling_gain */
    if (cfg->blocks.control.comp_du_V < 0.0f || cfg->blocks.control.comp_du_V > 20.0f) return 70;  /* comp_du_V */
    if (cfg->blocks.control.pwm_duty_max < 0.1f || cfg->blocks.control.pwm_duty_max > 0.99f) return 71;  /* pwm_duty_max */
    if (cfg->blocks.control.kp_s < 0.0f || cfg->blocks.control.kp_s > 10000.0f) return 72;  /* kp_s */
    if (cfg->blocks.control.ki_s < 0.0f || cfg->blocks.control.ki_s > 100000.0f) return 73;  /* ki_s */
    if (cfg->blocks.control.speed_integral_limit < 0.0f || cfg->blocks.control.speed_integral_limit > 1000.0f) return 74;  /* speed_integral_limit */
    if (cfg->blocks.control.vff < 0.0f || cfg->blocks.control.vff > 5.0f) return 75;  /* vff */
    if (cfg->blocks.control.aff < 0.0f || cfg->blocks.control.aff > 5.0f) return 76;  /* aff */
    if (cfg->blocks.control.jerk_ff < 0.0f || cfg->blocks.control.jerk_ff > 5.0f) return 77;  /* jerk_ff */
    if (cfg->blocks.control.speed_filter_alpha < 0.0f || cfg->blocks.control.speed_filter_alpha > 1.0f) return 78;  /* speed_filter_alpha */
    if (cfg->blocks.control.speed_filter_enable > (uint32_t)1) return 79;  /* speed_filter_enable */
    if (cfg->blocks.control.kp_p < 0.0f || cfg->blocks.control.kp_p > 10000.0f) return 80;  /* kp_p */
    if (cfg->blocks.control.ki_p < 0.0f || cfg->blocks.control.ki_p > 10000.0f) return 81;  /* ki_p */
    if (cfg->blocks.control.position_integral_limit < 0.0f || cfg->blocks.control.position_integral_limit > 1000.0f) return 82;  /* position_integral_limit */
    if (cfg->blocks.control.position_filter_alpha < 0.0f || cfg->blocks.control.position_filter_alpha > 1.0f) return 83;  /* position_filter_alpha */
    if (cfg->blocks.control.position_filter_enable > (uint32_t)1) return 84;  /* position_filter_enable */
    if (cfg->blocks.control.following_error_limit < 0.0f || cfg->blocks.control.following_error_limit > 1000000.0f) return 85;  /* following_error_limit */
    if (cfg->blocks.control.decouple_algo > (uint32_t)2) return 86;  /* decouple_algo */
    if (cfg->blocks.control.bemf_ff_enable > (uint32_t)1) return 87;  /* bemf_ff_enable */
    if (cfg->blocks.control.deadtime_comp_enable > (uint32_t)1) return 88;  /* deadtime_comp_enable */
    if (cfg->blocks.control.pid_source_mask > (uint32_t)4294967295) return 127;  /* pid_source_mask */

    /* 保护与通信参数 */
    if (cfg->blocks.protect_comm.over_current_A < 0.1f || cfg->blocks.protect_comm.over_current_A > 1000.0f) return 128;  /* over_current_A */
    if (cfg->blocks.protect_comm.over_voltage_V < 5.0f || cfg->blocks.protect_comm.over_voltage_V > 120.0f) return 129;  /* over_voltage_V */
    if (cfg->blocks.protect_comm.under_voltage_V < 0.0f || cfg->blocks.protect_comm.under_voltage_V > 60.0f) return 130;  /* under_voltage_V */
    if (cfg->blocks.protect_comm.over_temp_drive < 20.0f || cfg->blocks.protect_comm.over_temp_drive > 150.0f) return 131;  /* over_temp_drive */
    if (cfg->blocks.protect_comm.over_temp_motor < 20.0f || cfg->blocks.protect_comm.over_temp_motor > 200.0f) return 132;  /* over_temp_motor */
    if (cfg->blocks.protect_comm.under_temp_d < -80.0f || cfg->blocks.protect_comm.under_temp_d > 0.0f) return 133;  /* under_temp_d */
    if (cfg->blocks.protect_comm.over_speed_rad_s < 1.0f || cfg->blocks.protect_comm.over_speed_rad_s > 20000.0f) return 134;  /* over_speed_rad_s */
    if (cfg->blocks.protect_comm.position_following_error_p < (int32_t)1 || cfg->blocks.protect_comm.position_following_error_p > (int32_t)1000000) return 135;  /* position_following_error_p */
    if (cfg->blocks.protect_comm.pos_limit_min < (int32_t)-10000000 || cfg->blocks.protect_comm.pos_limit_min > (int32_t)0) return 136;  /* pos_limit_min */
    if (cfg->blocks.protect_comm.pos_limit_max < (int32_t)0 || cfg->blocks.protect_comm.pos_limit_max > (int32_t)10000000) return 137;  /* pos_limit_max */
    if (cfg->blocks.protect_comm.error_enable_mask > (uint32_t)4294967295) return 138;  /* error_enable_mask */

    /* 高级算法参数（MIT/力控/回零） */
    if (cfg->blocks.advanced.mit_kp < 0.0f || cfg->blocks.advanced.mit_kp > 10000.0f) return 160;  /* mit_kp */
    if (cfg->blocks.advanced.mit_kd < 0.0f || cfg->blocks.advanced.mit_kd > 1000.0f) return 161;  /* mit_kd */
    if (cfg->blocks.advanced.mit_max_current < 0.0f || cfg->blocks.advanced.mit_max_current > 500.0f) return 162;  /* mit_max_current */
    if (cfg->blocks.advanced.mit_feedforward_torque < 0.0f || cfg->blocks.advanced.mit_feedforward_torque > 500.0f) return 163;  /* mit_feedforward_torque */
    if (cfg->blocks.advanced.force_kp < 0.0f || cfg->blocks.advanced.force_kp > 10000.0f) return 164;  /* force_kp */
    if (cfg->blocks.advanced.force_ki < 0.0f || cfg->blocks.advanced.force_ki > 10000.0f) return 165;  /* force_ki */
    if (cfg->blocks.advanced.force_limit < 0.0f || cfg->blocks.advanced.force_limit > 1000.0f) return 166;  /* force_limit */
    if (cfg->blocks.advanced.force_control_enable > (uint32_t)1) return 167;  /* force_control_enable */
    if (cfg->blocks.advanced.homing_method > (uint32_t)10) return 168;  /* homing_method */
    if (cfg->blocks.advanced.homing_speed < 0.001f || cfg->blocks.advanced.homing_speed > 500.0f) return 169;  /* homing_speed */
    if (cfg->blocks.advanced.homing_offset < -6.283f || cfg->blocks.advanced.homing_offset > 6.283f) return 170;  /* homing_offset */

    return 0;
}

void motor_info_print(const motor_info_t *cfg)
{
    if (cfg == NULL) return;
    printf("========== MotorInfo Config (%d params) ==========\n", MOTOR_INFO_PARAM_COUNT);
    printf("v%d.%d  crc32=0x%08X\n",
           cfg->blocks.header.version_major,
           cfg->blocks.header.version_minor,
           (unsigned)cfg->blocks.header.crc32);

    printf("\n--- System Parameters ---\n");
    printf("config_version: %u\n", (unsigned)cfg->blocks.system.config_version);
    printf("enable_uart: %u\n", (unsigned)cfg->blocks.system.enable_uart);
    printf("enable_bus_sensor: %u\n", (unsigned)cfg->blocks.system.enable_bus_sensor);
    printf("safety_limit: %u\n", (unsigned)cfg->blocks.system.safety_limit);
    printf("total_runtime_s: %u s\n", (unsigned)cfg->blocks.system.total_runtime_s);
    printf("flash_write_count: %u\n", (unsigned)cfg->blocks.system.flash_write_count);

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
    printf("peak_current: %f A\n", cfg->blocks.motor_calib.peak_current);
    printf("max_speed: %f rad/s\n", cfg->blocks.motor_calib.max_speed);

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
    printf("decouple_algo: %u\n", (unsigned)cfg->blocks.control.decouple_algo);
    printf("bemf_ff_enable: %u\n", (unsigned)cfg->blocks.control.bemf_ff_enable);
    printf("deadtime_comp_enable: %u\n", (unsigned)cfg->blocks.control.deadtime_comp_enable);
    printf("pid_source_mask: %u\n", (unsigned)cfg->blocks.control.pid_source_mask);

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
 * Table-driven param_id dispatch. Field-level get/set functions are not generated.
 ******************************************************************************/

#define MOTOR_INFO_PARAM_IDX_INVALID 0xFFu
#define MOTOR_INFO_ACCESS_RO 0u
#define MOTOR_INFO_ACCESS_RW 1u
#define MOTOR_INFO_DESC_HAS_RANGE 0x01u

typedef union
{
    uint32_t u32;
    int32_t i32;
    float f32;
} motor_info_word_t;

typedef struct
{
    uint16_t pid;
    uint16_t offset;
    uint8_t type;
    uint8_t access;
    uint8_t size;
    uint8_t flags;
    motor_info_word_t min;
    motor_info_word_t max;
} motor_info_param_desc_t;

static const uint8_t s_pid_to_desc_index[MOTOR_INFO_MAX_PID + 1u] =
{
    0u, 1u, 2u, 3u, 4u, 5u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u,
    6u, 7u, 8u, 9u, 10u, 11u, 12u, 13u, 14u, 15u, 16u, 17u, 18u, 19u, 20u, 21u,
    22u, 23u, 24u, 25u, 26u, 27u, 28u, 29u, 30u, 31u, 32u, 33u, 34u, 255u, 255u, 255u,
    35u, 36u, 37u, 38u, 39u, 40u, 41u, 42u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u,
    43u, 44u, 45u, 46u, 47u, 48u, 49u, 50u, 51u, 52u, 53u, 54u, 55u, 56u, 57u, 58u,
    59u, 60u, 61u, 62u, 63u, 64u, 65u, 66u, 67u, 255u, 255u, 255u, 255u, 255u, 255u, 255u,
    255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u,
    255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 68u,
    69u, 70u, 71u, 72u, 73u, 74u, 75u, 76u, 77u, 78u, 79u, 255u, 255u, 255u, 255u, 255u,
    255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u, 255u,
    80u, 81u, 82u, 83u, 84u, 85u, 86u, 87u, 88u, 89u, 90u,
};

static const motor_info_param_desc_t s_motor_info_desc[MOTOR_INFO_PARAM_COUNT] =
{
    /* config_version */
    { 0u, (uint16_t)offsetof(motor_info_t, blocks.system.config_version), MOTOR_INFO_TYPE_U32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .u32 = 0U }, { .u32 = 4294967295U } },
    /* enable_uart */
    { 1u, (uint16_t)offsetof(motor_info_t, blocks.system.enable_uart), MOTOR_INFO_TYPE_U32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .u32 = 0U }, { .u32 = 15U } },
    /* enable_bus_sensor */
    { 2u, (uint16_t)offsetof(motor_info_t, blocks.system.enable_bus_sensor), MOTOR_INFO_TYPE_U32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .u32 = 0U }, { .u32 = 1U } },
    /* safety_limit */
    { 3u, (uint16_t)offsetof(motor_info_t, blocks.system.safety_limit), MOTOR_INFO_TYPE_U32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .u32 = 0U }, { .u32 = 1U } },
    /* total_runtime_s */
    { 4u, (uint16_t)offsetof(motor_info_t, blocks.system.total_runtime_s), MOTOR_INFO_TYPE_U32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .u32 = 0U }, { .u32 = 4294967295U } },
    /* flash_write_count */
    { 5u, (uint16_t)offsetof(motor_info_t, blocks.system.flash_write_count), MOTOR_INFO_TYPE_U32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .u32 = 0U }, { .u32 = 4294967295U } },
    /* is_calibrated */
    { 16u, (uint16_t)offsetof(motor_info_t, blocks.motor_calib.is_calibrated), MOTOR_INFO_TYPE_U32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .u32 = 0U }, { .u32 = 1U } },
    /* pole_pairs */
    { 17u, (uint16_t)offsetof(motor_info_t, blocks.motor_calib.pole_pairs), MOTOR_INFO_TYPE_U32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .u32 = 1U }, { .u32 = 128U } },
    /* motor_type */
    { 18u, (uint16_t)offsetof(motor_info_t, blocks.motor_calib.motor_type), MOTOR_INFO_TYPE_U32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .u32 = 0U }, { .u32 = 2U } },
    /* direction */
    { 19u, (uint16_t)offsetof(motor_info_t, blocks.motor_calib.direction), MOTOR_INFO_TYPE_U32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .u32 = 0U }, { .u32 = 1U } },
    /* phase_resistance */
    { 20u, (uint16_t)offsetof(motor_info_t, blocks.motor_calib.phase_resistance), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = 0.001f }, { .f32 = 50.0f } },
    /* phase_inductance_d */
    { 21u, (uint16_t)offsetof(motor_info_t, blocks.motor_calib.phase_inductance_d), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = 0.000001f }, { .f32 = 0.1f } },
    /* phase_inductance_q */
    { 22u, (uint16_t)offsetof(motor_info_t, blocks.motor_calib.phase_inductance_q), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = 0.000001f }, { .f32 = 0.1f } },
    /* flux_linkage */
    { 23u, (uint16_t)offsetof(motor_info_t, blocks.motor_calib.flux_linkage), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = 0.00001f }, { .f32 = 1.0f } },
    /* torque_constant */
    { 24u, (uint16_t)offsetof(motor_info_t, blocks.motor_calib.torque_constant), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = 0.00001f }, { .f32 = 50.0f } },
    /* rotor_inertia */
    { 25u, (uint16_t)offsetof(motor_info_t, blocks.motor_calib.rotor_inertia), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = 0.0000001f }, { .f32 = 1.0f } },
    /* friction_coulomb */
    { 26u, (uint16_t)offsetof(motor_info_t, blocks.motor_calib.friction_coulomb), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = 0.0f }, { .f32 = 500.0f } },
    /* friction_viscous */
    { 27u, (uint16_t)offsetof(motor_info_t, blocks.motor_calib.friction_viscous), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = 0.0f }, { .f32 = 100.0f } },
    /* gear_ratio */
    { 28u, (uint16_t)offsetof(motor_info_t, blocks.motor_calib.gear_ratio), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = 1.0f }, { .f32 = 10000.0f } },
    /* gear_efficiency */
    { 29u, (uint16_t)offsetof(motor_info_t, blocks.motor_calib.gear_efficiency), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = 0.1f }, { .f32 = 1.0f } },
    /* calibration_current */
    { 30u, (uint16_t)offsetof(motor_info_t, blocks.motor_calib.calibration_current), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = 0.1f }, { .f32 = 50.0f } },
    /* resistance_calib_max_voltage */
    { 31u, (uint16_t)offsetof(motor_info_t, blocks.motor_calib.resistance_calib_max_voltage), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = 0.1f }, { .f32 = 60.0f } },
    /* current_lim */
    { 32u, (uint16_t)offsetof(motor_info_t, blocks.motor_calib.current_lim), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = 0.1f }, { .f32 = 500.0f } },
    /* current_control_bandwidth */
    { 33u, (uint16_t)offsetof(motor_info_t, blocks.motor_calib.current_control_bandwidth), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = 50.0f }, { .f32 = 20000.0f } },
    /* enc_type */
    { 34u, (uint16_t)offsetof(motor_info_t, blocks.motor_calib.enc_type), MOTOR_INFO_TYPE_U32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .u32 = 0U }, { .u32 = 10U } },
    /* enc_lines */
    { 35u, (uint16_t)offsetof(motor_info_t, blocks.motor_calib.enc_lines), MOTOR_INFO_TYPE_U32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .u32 = 100U }, { .u32 = 10000000U } },
    /* enc_direction */
    { 36u, (uint16_t)offsetof(motor_info_t, blocks.motor_calib.enc_direction), MOTOR_INFO_TYPE_I32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .i32 = -1 }, { .i32 = 1 } },
    /* enc_offset */
    { 37u, (uint16_t)offsetof(motor_info_t, blocks.motor_calib.enc_offset), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = -360.0f }, { .f32 = 360.0f } },
    /* elec_angle_bias */
    { 38u, (uint16_t)offsetof(motor_info_t, blocks.motor_calib.elec_angle_bias), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = -3.1416f }, { .f32 = 3.1416f } },
    /* pwm_freq_hz */
    { 39u, (uint16_t)offsetof(motor_info_t, blocks.motor_calib.pwm_freq_hz), MOTOR_INFO_TYPE_U32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .u32 = 1000U }, { .u32 = 200000U } },
    /* dead_time_ns */
    { 40u, (uint16_t)offsetof(motor_info_t, blocks.motor_calib.dead_time_ns), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = 50.0f }, { .f32 = 5000.0f } },
    /* shunt_resistance */
    { 41u, (uint16_t)offsetof(motor_info_t, blocks.motor_calib.shunt_resistance), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = 0.0001f }, { .f32 = 1.0f } },
    /* current_amp_gain */
    { 42u, (uint16_t)offsetof(motor_info_t, blocks.motor_calib.current_amp_gain), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = 1.0f }, { .f32 = 10000.0f } },
    /* peak_current */
    { 43u, (uint16_t)offsetof(motor_info_t, blocks.motor_calib.peak_current), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = 0.1f }, { .f32 = 200.0f } },
    /* max_speed */
    { 44u, (uint16_t)offsetof(motor_info_t, blocks.motor_calib.max_speed), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = 10.0f }, { .f32 = 1000.0f } },
    /* device_zero */
    { 48u, (uint16_t)offsetof(motor_info_t, blocks.device.device_zero), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = -12.566f }, { .f32 = 12.566f } },
    /* device_time */
    { 49u, (uint16_t)offsetof(motor_info_t, blocks.device.device_time), MOTOR_INFO_TYPE_U32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .u32 = 0U }, { .u32 = 99999999U } },
    /* can_id */
    { 50u, (uint16_t)offsetof(motor_info_t, blocks.device.can_id), MOTOR_INFO_TYPE_U32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .u32 = 1U }, { .u32 = 127U } },
    /* can_baudrate */
    { 51u, (uint16_t)offsetof(motor_info_t, blocks.device.can_baudrate), MOTOR_INFO_TYPE_U32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .u32 = 10000U }, { .u32 = 8000000U } },
    /* can_timeout_s */
    { 52u, (uint16_t)offsetof(motor_info_t, blocks.device.can_timeout_s), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = 0.0f }, { .f32 = 60.0f } },
    /* can_fd_enable */
    { 53u, (uint16_t)offsetof(motor_info_t, blocks.device.can_fd_enable), MOTOR_INFO_TYPE_U32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .u32 = 0U }, { .u32 = 1U } },
    /* can_fd_baudrate */
    { 54u, (uint16_t)offsetof(motor_info_t, blocks.device.can_fd_baudrate), MOTOR_INFO_TYPE_U32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .u32 = 100000U }, { .u32 = 8000000U } },
    /* uart_baudrate */
    { 55u, (uint16_t)offsetof(motor_info_t, blocks.device.uart_baudrate), MOTOR_INFO_TYPE_U32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .u32 = 1200U }, { .u32 = 8000000U } },
    /* kp_ld */
    { 64u, (uint16_t)offsetof(motor_info_t, blocks.control.kp_ld), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = 0.0f }, { .f32 = 1000.0f } },
    /* ki_ld */
    { 65u, (uint16_t)offsetof(motor_info_t, blocks.control.ki_ld), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = 0.0f }, { .f32 = 100000.0f } },
    /* kp_lq */
    { 66u, (uint16_t)offsetof(motor_info_t, blocks.control.kp_lq), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = 0.0f }, { .f32 = 1000.0f } },
    /* ki_lq */
    { 67u, (uint16_t)offsetof(motor_info_t, blocks.control.ki_lq), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = 0.0f }, { .f32 = 100000.0f } },
    /* integral_limit */
    { 68u, (uint16_t)offsetof(motor_info_t, blocks.control.integral_limit), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = 0.0f }, { .f32 = 1000.0f } },
    /* decoupling_gain */
    { 69u, (uint16_t)offsetof(motor_info_t, blocks.control.decoupling_gain), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = 0.0f }, { .f32 = 5.0f } },
    /* comp_du_V */
    { 70u, (uint16_t)offsetof(motor_info_t, blocks.control.comp_du_V), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = 0.0f }, { .f32 = 20.0f } },
    /* pwm_duty_max */
    { 71u, (uint16_t)offsetof(motor_info_t, blocks.control.pwm_duty_max), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = 0.1f }, { .f32 = 0.99f } },
    /* kp_s */
    { 72u, (uint16_t)offsetof(motor_info_t, blocks.control.kp_s), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = 0.0f }, { .f32 = 10000.0f } },
    /* ki_s */
    { 73u, (uint16_t)offsetof(motor_info_t, blocks.control.ki_s), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = 0.0f }, { .f32 = 100000.0f } },
    /* speed_integral_limit */
    { 74u, (uint16_t)offsetof(motor_info_t, blocks.control.speed_integral_limit), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = 0.0f }, { .f32 = 1000.0f } },
    /* vff */
    { 75u, (uint16_t)offsetof(motor_info_t, blocks.control.vff), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = 0.0f }, { .f32 = 5.0f } },
    /* aff */
    { 76u, (uint16_t)offsetof(motor_info_t, blocks.control.aff), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = 0.0f }, { .f32 = 5.0f } },
    /* jerk_ff */
    { 77u, (uint16_t)offsetof(motor_info_t, blocks.control.jerk_ff), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = 0.0f }, { .f32 = 5.0f } },
    /* speed_filter_alpha */
    { 78u, (uint16_t)offsetof(motor_info_t, blocks.control.speed_filter_alpha), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = 0.0f }, { .f32 = 1.0f } },
    /* speed_filter_enable */
    { 79u, (uint16_t)offsetof(motor_info_t, blocks.control.speed_filter_enable), MOTOR_INFO_TYPE_U32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .u32 = 0U }, { .u32 = 1U } },
    /* kp_p */
    { 80u, (uint16_t)offsetof(motor_info_t, blocks.control.kp_p), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = 0.0f }, { .f32 = 10000.0f } },
    /* ki_p */
    { 81u, (uint16_t)offsetof(motor_info_t, blocks.control.ki_p), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = 0.0f }, { .f32 = 10000.0f } },
    /* position_integral_limit */
    { 82u, (uint16_t)offsetof(motor_info_t, blocks.control.position_integral_limit), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = 0.0f }, { .f32 = 1000.0f } },
    /* position_filter_alpha */
    { 83u, (uint16_t)offsetof(motor_info_t, blocks.control.position_filter_alpha), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = 0.0f }, { .f32 = 1.0f } },
    /* position_filter_enable */
    { 84u, (uint16_t)offsetof(motor_info_t, blocks.control.position_filter_enable), MOTOR_INFO_TYPE_U32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .u32 = 0U }, { .u32 = 1U } },
    /* following_error_limit */
    { 85u, (uint16_t)offsetof(motor_info_t, blocks.control.following_error_limit), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = 0.0f }, { .f32 = 1000000.0f } },
    /* decouple_algo */
    { 86u, (uint16_t)offsetof(motor_info_t, blocks.control.decouple_algo), MOTOR_INFO_TYPE_U32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .u32 = 0U }, { .u32 = 2U } },
    /* bemf_ff_enable */
    { 87u, (uint16_t)offsetof(motor_info_t, blocks.control.bemf_ff_enable), MOTOR_INFO_TYPE_U32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .u32 = 0U }, { .u32 = 1U } },
    /* deadtime_comp_enable */
    { 88u, (uint16_t)offsetof(motor_info_t, blocks.control.deadtime_comp_enable), MOTOR_INFO_TYPE_U32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .u32 = 0U }, { .u32 = 1U } },
    /* pid_source_mask */
    { 127u, (uint16_t)offsetof(motor_info_t, blocks.control.pid_source_mask), MOTOR_INFO_TYPE_U32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .u32 = 0U }, { .u32 = 4294967295U } },
    /* over_current_A */
    { 128u, (uint16_t)offsetof(motor_info_t, blocks.protect_comm.over_current_A), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = 0.1f }, { .f32 = 1000.0f } },
    /* over_voltage_V */
    { 129u, (uint16_t)offsetof(motor_info_t, blocks.protect_comm.over_voltage_V), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = 5.0f }, { .f32 = 120.0f } },
    /* under_voltage_V */
    { 130u, (uint16_t)offsetof(motor_info_t, blocks.protect_comm.under_voltage_V), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = 0.0f }, { .f32 = 60.0f } },
    /* over_temp_drive */
    { 131u, (uint16_t)offsetof(motor_info_t, blocks.protect_comm.over_temp_drive), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = 20.0f }, { .f32 = 150.0f } },
    /* over_temp_motor */
    { 132u, (uint16_t)offsetof(motor_info_t, blocks.protect_comm.over_temp_motor), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = 20.0f }, { .f32 = 200.0f } },
    /* under_temp_d */
    { 133u, (uint16_t)offsetof(motor_info_t, blocks.protect_comm.under_temp_d), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = -80.0f }, { .f32 = 0.0f } },
    /* over_speed_rad_s */
    { 134u, (uint16_t)offsetof(motor_info_t, blocks.protect_comm.over_speed_rad_s), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = 1.0f }, { .f32 = 20000.0f } },
    /* position_following_error_p */
    { 135u, (uint16_t)offsetof(motor_info_t, blocks.protect_comm.position_following_error_p), MOTOR_INFO_TYPE_I32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .i32 = 1 }, { .i32 = 1000000 } },
    /* pos_limit_min */
    { 136u, (uint16_t)offsetof(motor_info_t, blocks.protect_comm.pos_limit_min), MOTOR_INFO_TYPE_I32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .i32 = -10000000 }, { .i32 = 0 } },
    /* pos_limit_max */
    { 137u, (uint16_t)offsetof(motor_info_t, blocks.protect_comm.pos_limit_max), MOTOR_INFO_TYPE_I32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .i32 = 0 }, { .i32 = 10000000 } },
    /* error_enable_mask */
    { 138u, (uint16_t)offsetof(motor_info_t, blocks.protect_comm.error_enable_mask), MOTOR_INFO_TYPE_U32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .u32 = 0U }, { .u32 = 4294967295U } },
    /* mit_kp */
    { 160u, (uint16_t)offsetof(motor_info_t, blocks.advanced.mit_kp), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = 0.0f }, { .f32 = 10000.0f } },
    /* mit_kd */
    { 161u, (uint16_t)offsetof(motor_info_t, blocks.advanced.mit_kd), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = 0.0f }, { .f32 = 1000.0f } },
    /* mit_max_current */
    { 162u, (uint16_t)offsetof(motor_info_t, blocks.advanced.mit_max_current), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = 0.0f }, { .f32 = 500.0f } },
    /* mit_feedforward_torque */
    { 163u, (uint16_t)offsetof(motor_info_t, blocks.advanced.mit_feedforward_torque), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = 0.0f }, { .f32 = 500.0f } },
    /* force_kp */
    { 164u, (uint16_t)offsetof(motor_info_t, blocks.advanced.force_kp), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = 0.0f }, { .f32 = 10000.0f } },
    /* force_ki */
    { 165u, (uint16_t)offsetof(motor_info_t, blocks.advanced.force_ki), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = 0.0f }, { .f32 = 10000.0f } },
    /* force_limit */
    { 166u, (uint16_t)offsetof(motor_info_t, blocks.advanced.force_limit), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = 0.0f }, { .f32 = 1000.0f } },
    /* force_control_enable */
    { 167u, (uint16_t)offsetof(motor_info_t, blocks.advanced.force_control_enable), MOTOR_INFO_TYPE_U32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .u32 = 0U }, { .u32 = 1U } },
    /* homing_method */
    { 168u, (uint16_t)offsetof(motor_info_t, blocks.advanced.homing_method), MOTOR_INFO_TYPE_U32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .u32 = 0U }, { .u32 = 10U } },
    /* homing_speed */
    { 169u, (uint16_t)offsetof(motor_info_t, blocks.advanced.homing_speed), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = 0.001f }, { .f32 = 500.0f } },
    /* homing_offset */
    { 170u, (uint16_t)offsetof(motor_info_t, blocks.advanced.homing_offset), MOTOR_INFO_TYPE_F32, MOTOR_INFO_ACCESS_RW, 4u, MOTOR_INFO_DESC_HAS_RANGE, { .f32 = -6.283f }, { .f32 = 6.283f } },
};

static const motor_info_param_desc_t *motor_info_find_desc(uint16_t pid)
{
    if (pid > MOTOR_INFO_MAX_PID)
        return NULL;
    uint8_t idx = s_pid_to_desc_index[pid];
    if (idx == MOTOR_INFO_PARAM_IDX_INVALID || idx >= MOTOR_INFO_PARAM_COUNT)
        return NULL;
    return &s_motor_info_desc[idx];
}

static int motor_info_raw_in_range(const motor_info_param_desc_t *desc, const uint8_t raw[4])
{
    if ((desc->flags & MOTOR_INFO_DESC_HAS_RANGE) == 0u)
        return 1;
    switch (desc->type)
    {
    case MOTOR_INFO_TYPE_U8: { uint8_t v; memcpy(&v, raw, sizeof(v)); return (v >= (uint8_t)desc->min.u32 && v <= (uint8_t)desc->max.u32); }
    case MOTOR_INFO_TYPE_I8: { int8_t v; memcpy(&v, raw, sizeof(v)); return (v >= (int8_t)desc->min.i32 && v <= (int8_t)desc->max.i32); }
    case MOTOR_INFO_TYPE_U16: { uint16_t v; memcpy(&v, raw, sizeof(v)); return (v >= (uint16_t)desc->min.u32 && v <= (uint16_t)desc->max.u32); }
    case MOTOR_INFO_TYPE_I16: { int16_t v; memcpy(&v, raw, sizeof(v)); return (v >= (int16_t)desc->min.i32 && v <= (int16_t)desc->max.i32); }
    case MOTOR_INFO_TYPE_U32: { uint32_t v; memcpy(&v, raw, sizeof(v)); return (v >= desc->min.u32 && v <= desc->max.u32); }
    case MOTOR_INFO_TYPE_I32: { int32_t v; memcpy(&v, raw, sizeof(v)); return (v >= desc->min.i32 && v <= desc->max.i32); }
    case MOTOR_INFO_TYPE_F32: { float v; memcpy(&v, raw, sizeof(v)); return (v >= desc->min.f32 && v <= desc->max.f32); }
    default:
        return 0;
    }
}

int motor_info_dispatch_read(uint16_t pid, const motor_info_t *cfg, uint8_t out4[4], uint8_t *out_type, uint8_t *out_len)
{
    if (cfg == NULL || out4 == NULL || out_type == NULL || out_len == NULL)
        return -1;
    const motor_info_param_desc_t *desc = motor_info_find_desc(pid);
    if (desc == NULL)
        return -1;
    out4[0] = out4[1] = out4[2] = out4[3] = 0;
    memcpy(out4, &cfg->raw[desc->offset], desc->size);
    *out_type = desc->type;
    *out_len = desc->size;
    return 0;
}

int motor_info_dispatch_write(uint16_t pid, motor_info_t *cfg, const uint8_t in4[4], uint8_t len)
{
    (void)len;
    if (cfg == NULL || in4 == NULL)
        return -1;
    const motor_info_param_desc_t *desc = motor_info_find_desc(pid);
    if (desc == NULL)
        return -1;
    if (desc->access == MOTOR_INFO_ACCESS_RO)
        return -3;
    if (!motor_info_raw_in_range(desc, in4))
        return -2;
    memcpy(&cfg->raw[desc->offset], in4, desc->size);
    return 0;
}

int motor_info_read_u32(const motor_info_t *cfg, uint16_t pid, uint32_t *value)
{
    if (value == NULL)
        return -1;
    uint8_t raw[4], type, len;
    int rc = motor_info_dispatch_read(pid, cfg, raw, &type, &len);
    if (rc != 0)
        return rc;
    if (type != MOTOR_INFO_TYPE_U8 && type != MOTOR_INFO_TYPE_U16 && type != MOTOR_INFO_TYPE_U32)
        return -1;
    uint32_t v = 0;
    memcpy(&v, raw, len);
    *value = v;
    return 0;
}

int motor_info_write_u32(motor_info_t *cfg, uint16_t pid, uint32_t value)
{
    uint8_t raw[4];
    memcpy(raw, &value, sizeof(value));
    return motor_info_dispatch_write(pid, cfg, raw, sizeof(value));
}

int motor_info_read_i32(const motor_info_t *cfg, uint16_t pid, int32_t *value)
{
    if (value == NULL)
        return -1;
    uint8_t raw[4], type, len;
    int rc = motor_info_dispatch_read(pid, cfg, raw, &type, &len);
    if (rc != 0)
        return rc;
    if (type != MOTOR_INFO_TYPE_I8 && type != MOTOR_INFO_TYPE_I16 && type != MOTOR_INFO_TYPE_I32)
        return -1;
    int32_t v = 0;
    memcpy(&v, raw, len);
    *value = v;
    return 0;
}

int motor_info_write_i32(motor_info_t *cfg, uint16_t pid, int32_t value)
{
    uint8_t raw[4];
    memcpy(raw, &value, sizeof(value));
    return motor_info_dispatch_write(pid, cfg, raw, sizeof(value));
}

int motor_info_read_f32(const motor_info_t *cfg, uint16_t pid, float *value)
{
    if (value == NULL)
        return -1;
    uint8_t raw[4], type, len;
    int rc = motor_info_dispatch_read(pid, cfg, raw, &type, &len);
    if (rc != 0)
        return rc;
    if (type != MOTOR_INFO_TYPE_F32 || len != sizeof(float))
        return -1;
    memcpy(value, raw, sizeof(float));
    return 0;
}

int motor_info_write_f32(motor_info_t *cfg, uint16_t pid, float value)
{
    uint8_t raw[4];
    memcpy(raw, &value, sizeof(value));
    return motor_info_dispatch_write(pid, cfg, raw, sizeof(value));
}
