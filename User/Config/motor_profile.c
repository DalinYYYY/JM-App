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
#if defined(JM_BOARD_V1) || defined(JM_BOARD_SFOC_V2)
#include "fault_manager.h" /* fault_mgr_get_cfg: ProtectComm 故障段加载 */
#endif
#include <stddef.h>        /* NULL */

/* 电机时间常数 τ = L/R（秒），用于派生标定时间参数 */
#define MOTOR_TAU_S (MOTOR_LD / MOTOR_R)

#if defined(JM_BOARD_V1) || defined(JM_BOARD_SFOC_V2)
static void motor_profile_sync_fault_cfg(const ProtectCommParam_t *f);
#endif

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
	/* MOTOR_PARAM_EN_INIT_DEFAULTS=0 时这两个板级字段也是零，自动死区
	 * 补偿会按 t_dead*f_pwm*Vbus 算出 0。 */
	p->motor_base.pwm_freq_hz = MOTOR_PROFILE_PWM_FREQ_HZ;
	p->motor_base.foc_freq_hz = MOTOR_PROFILE_PWM_FREQ_HZ;
	p->motor_base.dead_time_ns = MOTOR_PROFILE_DEAD_TIME_NS;
	/* motor_param_init 默认采用全零裁剪；补偿链路必须显式给出非零增益，
	 * 否则即使使能位为 1，实际补偿项仍会被增益 0 乘掉。 */
	p->current_loop.decoupling_gain = 1.0f;
	p->current_loop.d_feedforward_gain = 1.0f;
	p->current_loop.q_feedforward_gain = 1.0f;
	p->current_loop.decouple_algo = 1u;
	p->current_loop.bemf_ff_enable = 1u;

	/* 其余板级参数（foc_freq）、减速器、编码器、PID 等仍由各自配置链路提供。 */
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
	if (p->blocks.motor_calib.peak_current == 0.0f)
		p->blocks.motor_calib.peak_current = MOTOR_PEAK_CURRENT;
	if (p->blocks.motor_calib.max_speed == 0.0f)
		p->blocks.motor_calib.max_speed = MOTOR_MAX_SPEED;
	if (p->blocks.motor_calib.pwm_freq_hz == 0u)
		p->blocks.motor_calib.pwm_freq_hz = MOTOR_PROFILE_PWM_FREQ_HZ;
	if (p->blocks.motor_calib.dead_time_ns == 0.0f)
		p->blocks.motor_calib.dead_time_ns = MOTOR_PROFILE_DEAD_TIME_NS;
	/* enc_direction: 0 视为未标定，默认 CW(1) */
	if (p->blocks.motor_calib.enc_direction == 0)
		p->blocks.motor_calib.enc_direction = 1;
	/* enc_offset / elec_angle_bias 不做零值 fallback：
	 *   - profile 没有合理的默认零点（每台电机安装位置不同）
	 *   - 零值表示"未标定，需重新做编码器零位标定"
	 *   - sync_to_param 会把零值同步到运行期，控制环读到 0 即知未标定 */

	/* is_calibrated / motor_type / direction / 减速器 / 编码器 / 功率级 /
	 * 电流采样 / PID 等不在此覆盖，保留 motor_info_init() 的默认值。*/

	/* 标记当前 profile 版本：无论数据来自加载还是恢复默认,
	 * 经本函数整理后的数据均应视为当前布局版本, 否则 0xEA 固化
	 * 会在回读校验中因版本不符失败(0xD0[27]=1) */
	p->blocks.system.config_version = MOTOR_PROFILE_CONFIG_VERSION;
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
	p->blocks.motor_calib.peak_current = MOTOR_PEAK_CURRENT;
	p->blocks.motor_calib.max_speed = MOTOR_MAX_SPEED;
	p->blocks.motor_calib.pwm_freq_hz = MOTOR_PROFILE_PWM_FREQ_HZ;
	p->blocks.motor_calib.dead_time_ns = MOTOR_PROFILE_DEAD_TIME_NS;
	p->blocks.motor_calib.enc_direction = 1; /* 默认 CW(正向) */

	/* motor_info_init 同样采用裁剪初始化；首次上电必须给控制补偿字段
	 * 明确的默认值，否则同步到 motor_param 后所有补偿都会是 0。 */
	p->blocks.control.decoupling_gain = 1.0f;
	p->blocks.control.decouple_algo = 1U;
	p->blocks.control.bemf_ff_enable = 1U;

	/* 保护总使能默认开启；三级故障使能掩码默认全开(低/高32位对)；0 表示明确关闭。 */
	p->blocks.protect_comm.protect_enable = 1u;
	p->blocks.protect_comm.mask_critical1 = 0xFFFFFFFFu;
	p->blocks.protect_comm.mask_critical2 = 0xFFFFFFFFu;
	p->blocks.protect_comm.mask_exception1 = 0xFFFFFFFFu;
	p->blocks.protect_comm.mask_exception2 = 0xFFFFFFFFu;
	p->blocks.protect_comm.mask_warning1 = 0xFFFFFFFFu;
	p->blocks.protect_comm.mask_warning2 = 0xFFFFFFFFu;

	/* 同时设置 config_version，标记当前 profile 版本 */
	p->blocks.system.config_version = MOTOR_PROFILE_CONFIG_VERSION;
}

void motor_profile_sync_to_param(motor_param_t *param, const motor_info_t *info)
{
	if (param == NULL || info == NULL)
		return;

	const MotorCalibParam_t *c = &info->blocks.motor_calib;
	const ProtectCommParam_t *pc = &info->blocks.protect_comm;
	uint32_t can_id = info->blocks.device.can_id;

	/* 节点地址由 motor_info 持久化配置统一提供, motor_param 仅保留运行期镜像。 */
	if (can_id >= 1u && can_id <= 127u)
		param->motor_instance.motor_id = (uint8_t)can_id;

	/* 电气参数：始终从 motor_info 同步到运行期，不再受 is_calibrated 门控。
	 * 【关键修复】原逻辑仅 is_calibrated==1 才同步，导致上位机 0xE7/0xEA 写入的
	 * RS03 参数(pole_pairs=21 等)存进 Flash 后，因未跑 L7 全流程(is_calibrated=0)
	 * 而不被同步，运行期保留 profile 默认值(pole_pairs=7)。
	 * motor_info 的电气字段要么是 Flash 标定值，要么是 apply_info 已做的逐字段
	 * 零值 fallback(profile 默认值)，两者都是有效值，无需 is_calibrated 门控。*/
	param->motor_base.pole_pairs = (uint8_t)c->pole_pairs;
	param->motor_base.r = c->phase_resistance;
	param->motor_base.ld = c->phase_inductance_d;
	param->motor_base.lq = c->phase_inductance_q;
	param->motor_base.flux = c->flux_linkage;
	param->motor_base.kt = c->torque_constant;
	param->motor_base.inertia = c->rotor_inertia;
	/* 摩擦模型参数（PID 26/27）: 供级联控制层摩擦前馈(friction_comp)使用,
	 * 缺此同步时上位机下发/固化的 Tf 永远到不了运行期(前馈恒 0)。 */
	param->position_loop.friction_coulomb = c->friction_coulomb;
	param->position_loop.friction_viscous = c->friction_viscous;
	param->motor_base.peak_current = c->peak_current;
	param->motor_base.max_speed = c->max_speed;
	param->motor_base.pwm_freq_hz = (c->pwm_freq_hz != 0U)
		? c->pwm_freq_hz : MOTOR_PROFILE_PWM_FREQ_HZ;
	param->motor_base.dead_time_ns = (c->dead_time_ns > 0.0f)
		? c->dead_time_ns : MOTOR_PROFILE_DEAD_TIME_NS;
	/* 编码器参数：始终同步（apply_info 已对 enc_direction 做零值 fallback=1；
	 * enc_offset/elec_angle_bias 保留 0 表示需标定，控制环据此判断未标定状态）*/
	param->encoder_param.enc_direction = (int8_t)c->enc_direction;
	param->encoder_param.enc_offset = c->enc_offset;
	param->encoder_param.elec_angle_bias = c->elec_angle_bias;

	/* 解耦配置：始终同步（默认值已在 motor_info_init 中设置，不依赖 is_calibrated）*/
	motor_profile_sync_control_to_param(param, info);

	/* Protection parameters are stored in motor_info Flash, but runtime fault
	 * checking reads motor_param_t.protection_param. Keep the runtime copy in
	 * sync during boot so 0xE7/0xEA changes take effect after restart. */
	param->protection_param.protect_over_current = pc->over_current_A;
	param->protection_param.protect_over_voltage = pc->over_voltage_V;
	param->protection_param.protect_under_voltage = pc->under_voltage_V;
	param->protection_param.protect_over_speed = pc->over_speed_rad_s;
	param->protection_param.protect_pos_error = pc->position_following_error_p;
	param->protection_param.protect_enable = pc->protect_enable;

#if defined(JM_BOARD_V1) || defined(JM_BOARD_SFOC_V2)
	/* 故障管理参数块(0x0380): 逐字段加载到 fault_mgr 运行配置。
	 * 旧参数区该块为 0xFF/随机值, 经范围校验失败的回落 fault_mgr 编译期默认值,
	 * 与缓启动 softstart_valid 同思路但粒度到字段(阈值类参数逐项独立生效)。 */
	motor_profile_sync_fault_cfg(&info->blocks.protect_comm);
#endif
}

void motor_profile_sync_control_to_param(motor_param_t *param, const motor_info_t *info)
{
	if (param == NULL || info == NULL)
		return;

	const ControlParam_t *control = &info->blocks.control;
	param->current_loop.decoupling_gain = control->decoupling_gain;
	param->current_loop.decouple_algo = (uint8_t)control->decouple_algo;
	param->current_loop.bemf_ff_enable = (uint8_t)control->bemf_ff_enable;
	param->current_loop.deadtime_comp_enable = (uint8_t)control->deadtime_comp_enable;
	param->current_loop.deadtime_comp_v = control->comp_du_V;
	/* 齿槽补偿使能/增益（PID 184/185, advanced 块）: 表数据存独立 Flash 扇区,
	 * 由 cogging_comp_reload 上电加载; 此处仅同步开关与增益供控制层每拍直读 */
	param->position_loop.cogging_comp_enable = (info->blocks.advanced.cogging_comp_enable != 0u) ? 1u : 0u;
	param->position_loop.cogging_comp_gain = info->blocks.advanced.cogging_comp_gain;
}

#if defined(JM_BOARD_V1) || defined(JM_BOARD_SFOC_V2)
/* ProtectCommParam 故障段 -> fault_mgr 运行配置(逐字段校验, 失败保持默认) */
static void motor_profile_sync_fault_cfg(const ProtectCommParam_t *f)
{
	fault_cfg_t *cfg = fault_mgr_get_cfg();

	if (cfg == NULL || f == NULL)
		return;

	/* 三级使能掩码：低/高32位对合成 u64；0=关闭该级别，非0=启用该级别。 */
	{
		uint64_t mask[3] = {
			((uint64_t)f->mask_critical2 << 32) | f->mask_critical1,
			((uint64_t)f->mask_exception2 << 32) | f->mask_exception1,
			((uint64_t)f->mask_warning2 << 32) | f->mask_warning1,
		};
		fault_mgr_set_enable_mask(mask);
	}

	/* 中度电压阈值: 须在故障阈值之间的合理区间 */
	if (f->ov_mid_v > 20.0f && f->ov_mid_v < 100.0f)
		cfg->ov_mid_v = f->ov_mid_v;
	if (f->uv_mid_v > 5.0f && f->uv_mid_v < 30.0f)
		cfg->uv_mid_v = f->uv_mid_v;

	/* 母线电流 */
	if (f->ibus_over_A > 1.0f && f->ibus_over_A < 200.0f)
		cfg->ibus_over_A = f->ibus_over_A;
	if (f->ibus_cont_A > 1.0f && f->ibus_cont_A < 200.0f)
		cfg->ibus_cont_A = f->ibus_cont_A;
	if (f->ibus_cont_time_s > 0.5f && f->ibus_cont_time_s < 60.0f)
		cfg->ibus_cont_time_s = f->ibus_cont_time_s;

	/* 温度阈值
	 * 注: 降功率系数/温度预警偏移/堵转/编码器跳变/坏帧/跟随误差/软限位/
	 * VBUS 失效时长等参数已精简(协议 v1.12), 相关检测回落
	 * fault_mgr 编译期默认(s_default_cfg), 仅影响调节粒度不影响保护功能。 */
	if (f->temp_fet_over_d > 50.0f && f->temp_fet_over_d < 120.0f)
		cfg->temp_fet_over_d = f->temp_fet_over_d;
	if (f->temp_motor_over_d > 50.0f && f->temp_motor_over_d < 200.0f)
		cfg->temp_motor_over_d = f->temp_motor_over_d;
	if (f->temp_motor_hot_d > 50.0f && f->temp_motor_hot_d < 200.0f)
		cfg->temp_motor_hot_d = f->temp_motor_hot_d;
	if (f->temp_under_d > -40.0f && f->temp_under_d < 10.0f)
		cfg->temp_under_d = f->temp_under_d;

}

/**
 * @brief 重新加载故障管理配置（补偿 fault_mgr_init 的 memset 覆盖）
 * @param info motor_info 持久化数据
 * @note  在 motor_loop_init 中调用，确保 Flash 持久化的故障配置生效
 */
void motor_profile_sync_fault_cfg_reload(const motor_info_t *info)
{
	if (info == NULL)
		return;

	motor_profile_sync_fault_cfg(&info->blocks.protect_comm);
}
#endif /* JM_BOARD_V1 || JM_BOARD_SFOC_V2 */
