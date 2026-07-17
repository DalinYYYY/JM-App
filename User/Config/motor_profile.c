/**
 * @file    motor_profile.c
 * @brief   电机参数配置 apply 接口实现
 * @date    2026-07-02
 *
 * @details 在 motor_param_init() / motor_info_init() 之后调用本文件接口，
 *          用 motor_profile.h 的 MOTOR_* 宏覆盖电机电气身份字段。
 *          这样自动生成的 motor_param.c / motor_info.c 无需手动修改，
 *          切换电机型号只需改 motor_profile.h 的 MOTOR_PROFILE 宏。
 */
#include "motor_profile.h"
#include "motor_param.h"
#include "motor_info.h"
#include <stddef.h> /* NULL */

/* 电机时间常数 τ = L/R（秒），用于派生标定时间参数 */
#define MOTOR_TAU_S (MOTOR_LD / MOTOR_R)

void motor_profile_apply_param(void *cfg)
{
	motor_param_t *p = (motor_param_t *)cfg;
	if (p == NULL)
		return;

	/* 电机本体电气身份参数（motor_base 段）*/
	p->motor_base.r = MOTOR_R;
	p->motor_base.ld = MOTOR_LD;
	p->motor_base.lq = MOTOR_LQ;
	p->motor_base.flux = MOTOR_FLUX;
	p->motor_base.kt = MOTOR_KT;
	p->motor_base.ke = MOTOR_KE;
	p->motor_base.pole_pairs = MOTOR_POLE_PAIRS;
	p->motor_base.rated_current = MOTOR_RATED_CURRENT;
	p->motor_base.peak_current = MOTOR_PEAK_CURRENT;
	p->motor_base.max_speed = MOTOR_MAX_SPEED;
	p->motor_base.rated_voltage = MOTOR_RATED_VOLTAGE;
	p->motor_base.rated_speed_rpm = MOTOR_RATED_SPEED_RPM;
	p->motor_base.rated_torque = MOTOR_RATED_TORQUE;
	p->motor_base.peak_torque = MOTOR_PEAK_TORQUE;
	p->motor_base.inertia = MOTOR_INERTIA;

	/* 板级参数（pwm_freq/foc_freq/dead_time）与减速器、编码器、PID 等
	 * 不在此覆盖，保留 motor_param_init() 的默认值。*/
	(void)MOTOR_TAU_S; /* 预留：未来可用于运行时派生控制环增益 */
}

void motor_profile_apply_info(void *cfg)
{
	motor_info_t *p = (motor_info_t *)cfg;
	if (p == NULL)
		return;

	/* 逐字段零值 fallback：零值视为未设置，用 profile 默认值覆盖；
	 * 非零保留 motor_info 中的标定值。
	 *   - 首次上电（Flash 无数据）：所有字段为 0，全部用默认值
	 *   - 已标定后上电：标定字段非零保留，未标定字段仍为 0 用默认值
	 *   - 部分标定：已标定字段生效，未标定字段用默认值兜底
	 * 注意：is_calibrated 不参与 fallback 判断，仅作状态标志。*/
	if (p->blocks.motor_calib.pole_pairs == 0U)
		p->blocks.motor_calib.pole_pairs = (uint32_t)MOTOR_POLE_PAIRS;
	if (p->blocks.motor_calib.phase_resistance == 0.0f)
		p->blocks.motor_calib.phase_resistance = MOTOR_R;
	if (p->blocks.motor_calib.phase_inductance_d == 0.0f)
		p->blocks.motor_calib.phase_inductance_d = MOTOR_LD;
	if (p->blocks.motor_calib.phase_inductance_q == 0.0f)
		p->blocks.motor_calib.phase_inductance_q = MOTOR_LQ;
	if (p->blocks.motor_calib.flux_linkage == 0.0f)
		p->blocks.motor_calib.flux_linkage = MOTOR_FLUX;
	if (p->blocks.motor_calib.torque_constant == 0.0f)
		p->blocks.motor_calib.torque_constant = MOTOR_KT;
	if (p->blocks.motor_calib.rotor_inertia == 0.0f)
		p->blocks.motor_calib.rotor_inertia = MOTOR_INERTIA;
	/* enc_direction: 0 视为未标定，默认 CW(1) */
	if (p->blocks.motor_calib.enc_direction == 0)
		p->blocks.motor_calib.enc_direction = 1;
	/* enc_offset / elec_angle_bias 不做零值 fallback：
	 *   - profile 没有合理的默认零点（每台电机安装位置不同）
	 *   - 零值表示"未标定，需重新做编码器零位标定"
	 *   - sync_to_param 会把零值同步到运行期，控制环读到 0 即知未标定 */

	/* is_calibrated / motor_type / direction / 减速器 / 编码器 / 功率级 /
	 * 电流采样 / PID 等不在此覆盖，保留 motor_info_init() 的默认值。*/
}

void motor_profile_apply_info_default(void *cfg)
{
	motor_info_t *p = (motor_info_t *)cfg;
	if (p == NULL)
		return;

	/* 无条件覆盖：无视 motor_info_init 的非零通用默认值，
	 * 强制用 profile 的电机型号特定默认值覆盖。
	 * 仅在首次上电（Flash 无数据 或 profile 版本不匹配）时调用。*/
	p->blocks.motor_calib.pole_pairs = (uint32_t)MOTOR_POLE_PAIRS;
	p->blocks.motor_calib.phase_resistance = MOTOR_R;
	p->blocks.motor_calib.phase_inductance_d = MOTOR_LD;
	p->blocks.motor_calib.phase_inductance_q = MOTOR_LQ;
	p->blocks.motor_calib.flux_linkage = MOTOR_FLUX;
	p->blocks.motor_calib.torque_constant = MOTOR_KT;
	p->blocks.motor_calib.rotor_inertia = MOTOR_INERTIA;
	p->blocks.motor_calib.enc_direction = 1; /* 默认 CW(正向) */

	/* 同时设置 config_version，标记当前 profile 版本 */
	p->blocks.system.config_version = MOTOR_PROFILE_CONFIG_VERSION;
}

void motor_profile_sync_to_param(motor_param_t *param, const motor_info_t *info)
{
	if (param == NULL || info == NULL)
		return;

	const MotorCalibParam_t *c = &info->blocks.motor_calib;

	/* 电气参数：仅当 is_calibrated==1 时用 Flash 标定值覆盖 profile 默认值
	 * （apply_param 已写入 profile 默认值，此处按标定状态决定是否覆盖）*/
	if (c->is_calibrated == 1U)
	{
		param->motor_base.pole_pairs = (uint8_t)c->pole_pairs;
		param->motor_base.r = c->phase_resistance;
		param->motor_base.ld = c->phase_inductance_d;
		param->motor_base.lq = c->phase_inductance_q;
		param->motor_base.flux = c->flux_linkage;
		param->motor_base.kt = c->torque_constant;
		param->motor_base.inertia = c->rotor_inertia;
	}

	/* 编码器参数：始终同步（apply_info 已对 enc_direction 做零值 fallback=1；
	 * enc_offset/elec_angle_bias 保留 0 表示需标定，控制环据此判断未标定状态）*/
	param->encoder_param.enc_direction = (int8_t)c->enc_direction;
	param->encoder_param.enc_offset = c->enc_offset;
	param->encoder_param.elec_angle_bias = c->elec_angle_bias;

	/* 解耦配置：始终同步（默认值已在 motor_info_init 中设置，不依赖 is_calibrated）*/
	param->current_loop.decouple_algo = (uint8_t)motor_info_get_decouple_algo(info);
	param->current_loop.bemf_ff_enable = (uint8_t)motor_info_get_bemf_ff_enable(info);
	param->current_loop.deadtime_comp_enable = (uint8_t)motor_info_get_deadtime_comp_enable(info);
}
