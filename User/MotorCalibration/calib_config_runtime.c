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
#include "calib_config.h"     /* CALIB_TICKS_PER_SEC, CALIB_LOW_R_THRESHOLD 等 */
#include "motor_profile.h"    /* MOTOR_* 宏（fallback 路径用） */

#if defined(USE_DEV_FLASH)
#include "motor_info_storage.h"
#include "motor_info.h"
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
		ident.peak_current = c->peak_current;   /* 从 motor_info 读取（Index 43）*/
		ident.max_speed = c->max_speed;         /* 从 motor_info 读取（Index 44）*/
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
	return (cur < CALIB_CFG_MAX_TEST_CURRENT_A) ? cur : CALIB_CFG_MAX_TEST_CURRENT_A;
}

float calib_test_current_actual_a(void)
{
	if (calib_is_low_r())
	{
		const calib_motor_ident_t *id = calib_motor_ident_get();
		float cur = id->peak_current * 0.5f;
		return (cur < CALIB_CFG_MAX_TEST_CURRENT_A) ? cur : CALIB_CFG_MAX_TEST_CURRENT_A;
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
float calib_cfg_l2_flux_target_speed_clamped_rad_s(void)
{
	const calib_motor_ident_t *id = calib_motor_ident_get();
	float v_target = (CALIB_CFG_L2_R_V_DT_ESTIMATE_V * 3.0f > 0.5f)
	                   ? CALIB_CFG_L2_R_V_DT_ESTIMATE_V * 3.0f
	                   : 0.5f;
	float speed = v_target / (id->flux * (float)id->pole_pairs);
	float max_clamp = id->max_speed * 0.3f;
	if (speed > max_clamp)
		speed = max_clamp;
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
	return 0.3f / (2.0f * PI * calib_tau_s());
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
	return (uint32_t)((float)CALIB_CFG_AC_INJECT_CYCLES / calib_cfg_ac_inject_freq_hz() * CALIB_TICKS_PER_SEC);
}

uint32_t calib_cfg_ac_inject_sample_count(void)
{
	return CALIB_CFG_AC_INJECT_SAMPLE_COUNT;
}

/* ===================== L2 相序 ===================== */
float calib_cfg_l2_phase_seq_voltage_v(void)
{
	return calib_test_current_a() * calib_motor_ident_get()->r;
}

uint32_t calib_cfg_l2_phase_seq_align_ticks(void)
{
	return (uint32_t)(CALIB_CFG_L2_PHASE_SEQ_ALIGN_S * CALIB_TICKS_PER_SEC);
}

uint32_t calib_cfg_l2_phase_seq_step_ticks(void)
{
	return (uint32_t)(CALIB_CFG_L2_PHASE_SEQ_STEP_S * CALIB_TICKS_PER_SEC);
}

/* ===================== L2 极对数 ===================== */
float calib_cfg_l2_pole_pairs_voltage_v(void)
{
	return calib_test_current_a() * calib_motor_ident_get()->r;
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
	return 2.0f * PI * (float)CALIB_CFG_L2_POLE_PAIRS_ELE_CYCLES;
}

/* ===================== L3 编码器 ===================== */
float calib_cfg_l3_align_voltage_v(void)
{
	return calib_test_current_a() * calib_motor_ident_get()->r;
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
	return calib_test_current_a() * calib_motor_ident_get()->r * 0.7f;
}

uint32_t calib_cfg_l3_dir_ticks(void)
{
	return (uint32_t)(CALIB_CFG_L3_DIR_TIME_S * CALIB_TICKS_PER_SEC);
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
