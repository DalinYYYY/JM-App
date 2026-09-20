/**
 * @file calib_config_runtime.c
 * @brief 标定参数运行期派生实现
 *
 * @details 从 motor_info 读取电机电气身份参数，运行期派生标定电压/时间/范围。
 *          算法公式与改造前 calib_config.h 编译期派生完全一致，仅改为函数调用。
 *
 *          启用 USE_DEV_FLASH: 从 motor_info_storage_get() 读取 Flash 加载值
 *          未启用: 从 MOTOR_* 编译期宏构造静态实例返回（V1 等板回退行为）
 */
#include "calib_config_runtime.h"
#include "calib_config.h"  /* CALIB_TICKS_PER_SEC, CALIB_LOW_R_THRESHOLD 等 */
#include "motor_profile.h" /* MOTOR_* 宏（fallback 路径用） */

#if defined(USE_DEV_FLASH)
#include "motor_info_storage.h"
#include "motor_info.h"
#endif
#if defined(USE_DEV_POWER_MONITOR)
#include "dev_power_monitor.h" /* Vbus 实测（死区电压派生用） */
#endif

#ifndef PI
#define PI 3.14159265358979f
#endif

/* ===================== 电机身份快照获取 ===================== */
const calib_motor_ident_t *calib_motor_ident_get(void)
{
#if defined(USE_DEV_FLASH)
	/* 启用 Flash: 从 motor_info 读取运行期值（上位机可通过 0xE7 修改）*/
	static calib_motor_ident_t ident;
	const motor_info_t *info = motor_info_storage_get();
	if (info != NULL)
	{
		const MotorCalibParam_t *c = &info->blocks.motor_calib;
		ident.r = c->phase_resistance;
		ident.ld = c->phase_inductance_d;
		ident.lq = c->phase_inductance_q;
		ident.flux = c->flux_linkage;
		ident.pole_pairs = c->pole_pairs;
		ident.peak_current = c->peak_current; /* 从 motor_info 读取（Index 43）*/
		ident.max_speed = c->max_speed;       /* 从 motor_info 读取（Index 44）*/
		ident.dead_time_s = c->dead_time_ns * 1e-9f;  /* Index 40 */
		ident.pwm_freq_hz = (float)c->pwm_freq_hz;    /* Index 39 */
		return &ident;
	}
	/* motor_info 不可用时回退到编译期值（不应发生，仅防御性处理）*/
#endif
	/* 未启用 Flash: 用 MOTOR_* 编译期宏构造静态实例 */
	{
		static const calib_motor_ident_t s_fallback = {
			.r = MOTOR_R,
			.ld = MOTOR_LD,
			.lq = MOTOR_LQ,
			.flux = MOTOR_FLUX,
			.pole_pairs = MOTOR_POLE_PAIRS,
			.peak_current = MOTOR_PEAK_CURRENT,
			.max_speed = MOTOR_MAX_SPEED,
			.dead_time_s = 500.0f * 1e-9f,  /* 500ns 板级通用值(无 Flash 板兜底) */
			.pwm_freq_hz = 10000.0f,        /* 10kHz 板级通用值(无 Flash 板兜底) */
		};
		return &s_fallback;
	}
}

/* ===================== 基础派生 ===================== */
float calib_tau_s(void)
{
	const calib_motor_ident_t *id = calib_motor_ident_get();
	return id->ld / id->r;
}

bool calib_is_low_r(void)
{
	const calib_motor_ident_t *id = calib_motor_ident_get();
	return id->r < CALIB_LOW_R_THRESHOLD;
}

float calib_test_current_a(void)
{
	const calib_motor_ident_t *id = calib_motor_ident_get();
	float cur = id->peak_current * 0.3f;
	/* 低阻电机(R<0.5Ω): 突破 1.5A 默认上限, 用 6A 保证足够对齐力矩
	 * 实际标定电压 = I×R + 2×死区(见 calib_voltage_floor_v()),
	 * 标定电流收敛到本函数设计值附近(QH8919≈12A, RS03≈8A)
	 * 普通电机: 保持原 1.5A 上限不变 */
	if (calib_is_low_r())
	{
		return (cur < CALIB_CFG_ALIGN_CURRENT_MAX_A) ? cur : CALIB_CFG_ALIGN_CURRENT_MAX_A;
	}
	return (cur < CALIB_CFG_MAX_TEST_CURRENT_A) ? cur : CALIB_CFG_MAX_TEST_CURRENT_A;
}

float calib_test_current_actual_a(void)
{
	if (calib_is_low_r())
	{
		const calib_motor_ident_t *id = calib_motor_ident_get();
		float cur = id->peak_current * 0.5f;
		/* 低阻电机对齐电流上限与 test_current 一致(3A), 避免 actual 与 test 脱节 */
		return (cur < CALIB_CFG_ALIGN_CURRENT_MAX_A) ? cur : CALIB_CFG_ALIGN_CURRENT_MAX_A;
	}
	return calib_test_current_a();
}

/* ===================== L2 R ===================== */
float calib_cfg_l2_r_test_voltage_v(void)
{
	return calib_test_current_actual_a() * calib_motor_ident_get()->r;
}

float calib_cfg_l2_r_test_voltage_lo_v(void)
{
	float v_r = calib_cfg_l2_r_test_voltage_v();
	float v_dt_x2 = CALIB_CFG_L2_R_V_DT_ESTIMATE_V * 2.0f;
	float v_r_x04 = v_r * 0.4f;
	return (v_r_x04 > v_dt_x2) ? v_r_x04 : v_dt_x2;
}

uint32_t calib_cfg_l2_r_test_ticks(void)
{
	return (uint32_t)(calib_tau_s() * 750.0f * CALIB_TICKS_PER_SEC);
}

uint32_t calib_cfg_l2_r_sample_count(void)
{
	return CALIB_CFG_L2_R_SAMPLE_COUNT;
}

/* ===================== L2 Ld ===================== */
float calib_cfg_l2_ld_test_voltage_v(void)
{
	return calib_cfg_l2_r_test_voltage_v() * 1.3f;
}

uint32_t calib_cfg_l2_ld_test_ticks(void)
{
	return (uint32_t)(calib_tau_s() * 3.0f * CALIB_TICKS_PER_SEC);
}

uint32_t calib_cfg_l2_ld_sample_count(void)
{
	return CALIB_CFG_L2_LD_SAMPLE_COUNT;
}

uint32_t calib_cfg_l2_ld_skip_ticks(void)
{
	return CALIB_CFG_L2_LD_SKIP_TICKS;
}

/* ===================== L2 Lq ===================== */
float calib_cfg_l2_lq_test_voltage_v(void)
{
	return calib_cfg_l2_r_test_voltage_v() * 1.3f;
}

uint32_t calib_cfg_l2_lq_test_ticks(void)
{
	return (uint32_t)(calib_tau_s() * 3.0f * CALIB_TICKS_PER_SEC);
}

uint32_t calib_cfg_l2_lq_sample_count(void)
{
	return CALIB_CFG_L2_LQ_SAMPLE_COUNT;
}

uint32_t calib_cfg_l2_lq_skip_ticks(void)
{
	return CALIB_CFG_L2_LQ_SKIP_TICKS;
}

/* ===================== L2 flux ===================== */
/* 实测派生死区电压: V_dt = t_dt × f_pwm × Vbus
 * 旧固定估计 0.5V(×1.5=0.75V) 按 24V 母线设计, 48V 母线下实际仅 0.24V,
 * flux 标定多扣 0.51V 直接压低磁链读数(实测 0.006→0.0033)。
 * t_dt/f_pwm 从 motor_info 读取(id 39/40), Vbus 运行期实测, 母线无关自适应。*/
float calib_deadtime_voltage_v(void)
{
	const calib_motor_ident_t *id = calib_motor_ident_get();
	float vbus = 24.0f; /* 无电源监控时回退标称值 */
#if defined(USE_DEV_POWER_MONITOR)
	if (dev_power_monitor.vbus > 1.0f)
		vbus = dev_power_monitor.vbus;
#endif
	return id->dead_time_s * id->pwm_freq_hz * vbus;
}

float calib_cfg_l2_flux_target_speed_clamped_rad_s(void)
{
	const calib_motor_ident_t *id = calib_motor_ident_get();
	/* 目标转速: 逼近台架机械上限(50rad/s), 最大化 ωe 信噪比——
	 * flux = (uq_eff - R·iq)/ωe 中死区/电阻项误差占比 ∝ 1/ωe,
	 * 16.7rad/s(ωe=234)时死区残余误差占 ±6%, 50rad/s(ωe=700)时仅 ±2%。
	 * spin 电压按 Flash flux 派生, 实际转速由真实 flux 自限,
	 * 不会超压失控(转速 < uq/(flux_true×pp) 恒成立)。*/
	float max_clamp = 50.0f; /* 台架机械转速上限 */
	if (id->max_speed * 0.3f < max_clamp)
		max_clamp = id->max_speed * 0.3f; /* 电机能力防护 */
	float speed = max_clamp;
	if (speed < 5.0f)
		speed = 5.0f;
	return speed;
}

float calib_cfg_l2_flux_spin_voltage_v(void)
{
	const calib_motor_ident_t *id = calib_motor_ident_get();
	return calib_test_current_actual_a() * id->r
	       + id->flux * (float)id->pole_pairs * calib_cfg_l2_flux_target_speed_clamped_rad_s();
}

uint32_t calib_cfg_l2_flux_spin_ticks(void)
{
	return (uint32_t)(CALIB_CFG_L2_FLUX_SPIN_TIME_S * CALIB_TICKS_PER_SEC);
}

uint32_t calib_cfg_l2_flux_sample_count(void)
{
	return CALIB_CFG_L2_FLUX_SAMPLE_COUNT;
}

/* ===================== 交流注入 ===================== */
float calib_cfg_ac_inject_freq_hz(void)
{
	/* 注入频率取 ω·L = R（φ=45°）：相敏分解中感性分量 i_rea = I·sinφ 达到
	 * 最大（0.707），Ld 信噪比最优；阻性分量 cosφ=0.707 仅降 26%（R 本来
	 * 信噪比就富余）。旧值 0.3/(2πτ) 时 φ=16.7°，感性信号只占 29%，
	 * Ld 重复测量方差大（R 稳定而 Ld 抖的根因之一）。
	 * 限幅: 上限 200Hz 保证每周期 ≥50 个采样点；下限 5Hz 防过慢。*/
	float f = 1.0f / (2.0f * PI * calib_tau_s());
	if (f > 200.0f)
		f = 200.0f;
	if (f < 5.0f)
		f = 5.0f;
	return f;
}

float calib_cfg_ac_inject_amp_v(void)
{
	/* AC 幅值: 低阻电机放大到死区以上, 提升信噪比避免死区非线性污染。
	 * 正常阻值电机保持原公式 (test_current*0.5*r)。*/
	if (calib_is_low_r())
	{
		/* ud_ac = max(0.5*test_current*r, 0.3V)
		 * 下限 0.3V 保证 AC 分量幅值明显高于死区, 相敏检测能稳定提取基波 */
		float v_ac = calib_test_current_actual_a() * calib_motor_ident_get()->r * 0.5f;
		return (v_ac > 0.3f) ? v_ac : 0.3f;
	}
	return calib_test_current_actual_a() * calib_motor_ident_get()->r * 0.5f;
}

float calib_cfg_ac_inject_dc_v(void)
{
	/* DC 偏置: 低阻电机强制拉到死区以上, 避免 ud_dc < V_dead 导致实际电压
	 * 被体二极管/MOSFET 体电阻非线性吞掉。
	 * CALIB_CFG_L2_R_V_DT_ESTIMATE_V=0.5V 是死区压降保守估计,
	 * ud_dc = max(test_current*r, 1.5*V_dead) = max(0.18V, 0.75V) = 0.75V
	 * 这样实际加到电机绕组的电压 = ud_dc - V_dead ≈ 0.25V 仍在线性区。
	 * 注: 标定出的 R 包含 MOSFET Rds(on)·2 + 接线电阻, 略大于电机本体 R,
	 *     此为系统性正向偏置, 属工程可接受范围。*/
	if (calib_is_low_r())
	{
		float v_dc = calib_test_current_actual_a() * calib_motor_ident_get()->r;
		float v_floor = CALIB_CFG_L2_R_V_DT_ESTIMATE_V * 1.5f; /* 0.75V */
		return (v_dc > v_floor) ? v_dc : v_floor;
	}
	return calib_test_current_actual_a() * calib_motor_ident_get()->r;
}

uint32_t calib_cfg_ac_inject_warmup_ticks(void)
{
	return (uint32_t)(calib_tau_s() * 10.0f * CALIB_TICKS_PER_SEC);
}

uint32_t calib_cfg_ac_inject_total_ticks(void)
{
	/* 采样窗口整周期量化: N = round(CYCLES × f_sample/f_inject)。
	 * 非整周期窗口会让 DC 偏置(~8A)通过 Σsin(ωt)≠0 泄漏进相关器，
	 * 直接污染小信号感性分量——Ld 方差的第二个来源。
	 * 量化后一个采样拍以内的残余相位，泄漏可忽略。*/
	float f = calib_cfg_ac_inject_freq_hz();
	if (f <= 0.0f)
		return (uint32_t)CALIB_CFG_AC_INJECT_CYCLES * (uint32_t)CALIB_TICKS_PER_SEC;
	float n = (float)CALIB_CFG_AC_INJECT_CYCLES * CALIB_TICKS_PER_SEC / f;
	uint32_t ticks = (uint32_t)(n + 0.5f);
	return (ticks < 1u) ? 1u : ticks;
}

uint32_t calib_cfg_ac_inject_sample_count(void)
{
	return CALIB_CFG_AC_INJECT_SAMPLE_COUNT;
}

/* ===================== 标定电压自适应保底 ===================== */
/* 相序/极对数/零位/方向四项开环电压标定共用的保底电压。
 * 自适应保底 = 测试电流需求电压(I×R) + 2×死区压降估计：
 *   - 死区余量保证有效绕组电压 ≥ I×R，标定电流收敛到设计值附近
 *   - 力矩 = I×kt(QH8919≈12A/1.5Nm, RS03≈8A/2.6Nm)，拖动转子余量充足 */
static float calib_voltage_floor_v(void)
{
	return calib_test_current_a() * calib_motor_ident_get()->r
	       + CALIB_CFG_L2_R_V_DT_ESTIMATE_V * 2.0f;
}

/* ===================== L2 相序 ===================== */
float calib_cfg_l2_phase_seq_voltage_v(void)
{
	return calib_voltage_floor_v();
}

uint32_t calib_cfg_l2_phase_seq_align_ticks(void)
{
	return (uint32_t)(CALIB_CFG_L2_PHASE_SEQ_ALIGN_S * CALIB_TICKS_PER_SEC);
}

float calib_cfg_l2_phase_seq_scan_target_rad(void)
{
	return 2.0f * PI * (float)CALIB_CFG_L2_PHASE_SEQ_SCAN_ELE_CYCLES;
}

/* ===================== L2 极对数 ===================== */
float calib_cfg_l2_pole_pairs_voltage_v(void)
{
	/* 与相序标定同源：保底见 calib_voltage_floor_v() 说明 */
	return calib_voltage_floor_v();
}

uint32_t calib_cfg_l2_pole_pairs_align_ticks(void)
{
	return (uint32_t)(CALIB_CFG_L2_POLE_PAIRS_ALIGN_S * CALIB_TICKS_PER_SEC);
}

float calib_cfg_l2_pole_pairs_dtheta_rad(void)
{
	return 2.0f * PI * CALIB_CFG_L2_POLE_PAIRS_ELE_FREQ_HZ / CALIB_TICKS_PER_SEC;
}

float calib_cfg_l2_pole_pairs_target_rad(void)
{
	const calib_motor_ident_t *id = calib_motor_ident_get();
	uint16_t cycles = (uint16_t)id->pole_pairs;
	if (cycles < CALIB_CFG_L2_POLE_PAIRS_ELE_CYCLES)
	{
		cycles = CALIB_CFG_L2_POLE_PAIRS_ELE_CYCLES; /* 小极对数电机保底扫描电周期数 */
	}
	return 2.0f * PI * (float)cycles;
}

/* ===================== L3 编码器 ===================== */
float calib_cfg_l3_align_voltage_v(void)
{
	/* 零位对齐电压：保底见 calib_voltage_floor_v() 说明 */
	return calib_voltage_floor_v();
}

uint32_t calib_cfg_l3_align_ticks(void)
{
	return (uint32_t)(CALIB_CFG_L3_ALIGN_TIME_S * CALIB_TICKS_PER_SEC);
}

uint32_t calib_cfg_l3_sample_count(void)
{
	return CALIB_CFG_L3_SAMPLE_COUNT;
}

float calib_cfg_l3_dir_voltage_v(void)
{
	/* 方向标定用 uq 施加力矩, 需克服静摩擦+齿槽转矩,
	 * 保底与零位对齐同源: 见 calib_voltage_floor_v() 说明 */
	return calib_voltage_floor_v();
}

uint32_t calib_cfg_l3_dir_ticks(void)
{
	return (uint32_t)(CALIB_CFG_L3_DIR_TIME_S * CALIB_TICKS_PER_SEC);
}

/* ===================== L6 惯量辨识 ===================== */
/* 测试电流: 转矩 SNR 与绕组发热折中。上限复用对齐电流安全上限 6A
 * (短时标定安全值, 惯量辨识两段合计 <6s 远短于对齐扫描 10s)。*/
float calib_cfg_l6_inertia_test_current_a(void)
{
	const calib_motor_ident_t *id = calib_motor_ident_get();
	float cur = id->peak_current * CALIB_CFG_L6_INERTIA_CURRENT_FRAC;
	if (cur > CALIB_CFG_ALIGN_CURRENT_MAX_A)
		cur = CALIB_CFG_ALIGN_CURRENT_MAX_A;
	if (cur < CALIB_CFG_L6_INERTIA_CURRENT_MIN_A)
		cur = CALIB_CFG_L6_INERTIA_CURRENT_MIN_A;
	return cur;
}

/* 加速目标转速: 上限 50rad/s 与 flux 标定同源(台架机械约束),
 * 换向时速度越低位置漂移越小; 下限 5rad/s 保证回归窗口速度信噪比。*/
float calib_cfg_l6_inertia_speed_limit_rad_s(void)
{
	const calib_motor_ident_t *id = calib_motor_ident_get();
	float v = id->max_speed * CALIB_CFG_L6_INERTIA_SPEED_FRAC;
	if (v > CALIB_CFG_L6_INERTIA_SPEED_MAX_RAD_S)
		v = CALIB_CFG_L6_INERTIA_SPEED_MAX_RAD_S;
	if (v < CALIB_CFG_L6_INERTIA_SPEED_MIN_RAD_S)
		v = CALIB_CFG_L6_INERTIA_SPEED_MIN_RAD_S;
	return v;
}

/* ===================== 合理性范围 ===================== */
float calib_cfg_r_min_ohm(void)
{
	float r_lo = calib_motor_ident_get()->r * 0.2f;
	return (r_lo > CALIB_CFG_R_MIN_OHM_SAFE) ? r_lo : CALIB_CFG_R_MIN_OHM_SAFE;
}

float calib_cfg_r_max_ohm(void)
{
	return calib_motor_ident_get()->r * 5.0f;
}

float calib_cfg_ld_min_h(void)
{
	return calib_motor_ident_get()->ld * 0.2f;
}

float calib_cfg_ld_max_h(void)
{
	return calib_motor_ident_get()->ld * 3.0f;
}

float calib_cfg_lq_min_h(void)
{
	return calib_motor_ident_get()->lq * 0.2f;
}

float calib_cfg_lq_max_h(void)
{
	return calib_motor_ident_get()->lq * 3.0f;
}

float calib_cfg_flux_min_wb(void)
{
	return calib_motor_ident_get()->flux * 0.2f;
}

float calib_cfg_flux_max_wb(void)
{
	return calib_motor_ident_get()->flux * 3.0f;
}
