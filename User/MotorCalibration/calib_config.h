#ifndef __CALIB_CONFIG_H__
#define __CALIB_CONFIG_H__

#include "motor_profile.h" /* 电机电气身份参数（R/Ld/Lq/flux/pole_pairs），用于派生标定结果合理性范围 */

/* ===================== 标定集中配置 =====================
 * 所有标定可调参数（电压/时间/采样数）集中于此。
 * 时间相关 TICK 数约定：控制环频率 10kHz（dt=100us），
 *
 * 所有测试电压与时间均从 motor_profile.h 的 MOTOR_* 参数派生，
 */

#define CALIB_TICKS_PER_SEC 10000.0f

/* 电机时间常数 τ = Ld/R（秒），用于派生标定时间参数 */
#define MOTOR_TAU_S (MOTOR_LD / MOTOR_R)

/* ===================== 电机分档识别（通用性核心）=====================
 * 根据 MOTOR_R 自动判断电机类型，切换标定策略。
 * 低阻电机(R<0.5Ω): 用交流注入法辨识 R/Ld，提高测试电流
 * 高阻电机(R≥0.5Ω): 用 DC 两点差分法辨识 R，标准测试电流 */
#define CALIB_LOW_R_THRESHOLD  0.5f
#define CALIB_IS_LOW_R         (MOTOR_R < CALIB_LOW_R_THRESHOLD)

/* 低阻电机测试电流提高：峰值电流 × 0.5（受 CALIB_CFG_MAX_TEST_CURRENT_A 限制）
 * 低阻电机需要更大电流才能产生可测电压（V=I×R，R 小则 V 小）*/
#define CALIB_CFG_TEST_CURRENT_LOW_R_A \
	((MOTOR_PEAK_CURRENT * 0.5f) < CALIB_CFG_MAX_TEST_CURRENT_A \
	 ? (MOTOR_PEAK_CURRENT * 0.5f) \
	 : CALIB_CFG_MAX_TEST_CURRENT_A)

/* 实际测试电流：低阻电机用大电流，高阻电机用标准电流 */
#define CALIB_CFG_TEST_CURRENT_ACTUAL_A \
	(CALIB_IS_LOW_R ? CALIB_CFG_TEST_CURRENT_LOW_R_A : CALIB_CFG_TEST_CURRENT_A)

/* 标定测试电流基准 = min(峰值电流 × 0.3, 电源限流安全值)
 * 所有"施加电压产生电流"类标定均以此为电流目标
 *
 * 示例:
 *   5010电机(R=0.12Ω, Ipeak=20A): min(6A, 1.5A)=1.5A, V=0.18V
 *   GM4820H(R=3.6Ω, Ipeak=3.7A):  min(1.11A, 1.5A)=1.11A, V=4V */
#define CALIB_CFG_MAX_TEST_CURRENT_A 1.5f
#define CALIB_CFG_TEST_CURRENT_A \
	((MOTOR_PEAK_CURRENT * 0.3f) < CALIB_CFG_MAX_TEST_CURRENT_A \
	 ? (MOTOR_PEAK_CURRENT * 0.3f) \
	 : CALIB_CFG_MAX_TEST_CURRENT_A)

/* ===================== L1 驱动硬件底层参数（预留）===================== */
#define CALIB_CFG_L1_ADC_OFFSET_SAMPLES 1000 /* ADC偏置采样次数（取平均）*/
#define CALIB_CFG_L1_VBUS_SAMPLE_COUNT  500  /* 母线电压采样次数 */

/* ===================== L2 电机电气身份参数（从 MOTOR_* 派生）===================== */

/* R 辨识：R=(V2-V1)/(id2-id1)  死区效应会使 R 偏大, 但两点差分可部分抵消*/
#define CALIB_CFG_L2_R_TEST_VOLTAGE_V    (CALIB_CFG_TEST_CURRENT_ACTUAL_A * MOTOR_R) /* 高档 V2 */
/* V_dt 估计值：无标定时用 0.5V 保守估计（典型死区+MOSFET-Rds压降）*/
#define CALIB_CFG_L2_R_V_DT_ESTIMATE_V   0.5f
/* V1 低档：max(V_R×0.4, V_dt_estimate×2)，确保 V1 > 死区压降
 * 低阻电机 V_R 很小（如 5010: 0.18V），若 V1<V_dt 则低档电流被死区主导 */
#define CALIB_CFG_L2_R_TEST_VOLTAGE_LO_V \
	((CALIB_CFG_L2_R_TEST_VOLTAGE_V * 0.4f) > (CALIB_CFG_L2_R_V_DT_ESTIMATE_V * 2.0f) \
	 ? (CALIB_CFG_L2_R_TEST_VOLTAGE_V * 0.4f) \
	 : (CALIB_CFG_L2_R_V_DT_ESTIMATE_V * 2.0f)) /* 低档 V1 */
#define CALIB_CFG_L2_R_TEST_TIME_S       (MOTOR_TAU_S * 750.0f)
#define CALIB_CFG_L2_R_TEST_TICKS        (uint32_t)(CALIB_CFG_L2_R_TEST_TIME_S * CALIB_TICKS_PER_SEC)
#define CALIB_CFG_L2_R_SAMPLE_COUNT      500 /* 每档稳态采样次数 */

/* Ld 辨识（d 轴阶跃响应）
 * 阶跃电压 = R 测试电压 × 1.3（需更高电压产生 di/dt）
 * 暂态窗口 = 3.0τ（扩展窗口确保低感电机有足够采样点）*/
#define CALIB_CFG_L2_LD_TEST_VOLTAGE_V (CALIB_CFG_L2_R_TEST_VOLTAGE_V * 1.3f)
#define CALIB_CFG_L2_LD_TEST_TIME_S    (MOTOR_TAU_S * 3.0f)
#define CALIB_CFG_L2_LD_TEST_TICKS     (uint32_t)(CALIB_CFG_L2_LD_TEST_TIME_S * CALIB_TICKS_PER_SEC)
#define CALIB_CFG_L2_LD_SAMPLE_COUNT   30 /* 暂态采样点数 */
#define CALIB_CFG_L2_LD_SKIP_TICKS     2u  /* 阶跃后跳过前 2 拍（数值噪声）*/

/* Lq 辨识（q 轴阶跃响应）—— 与 Ld 对称 */
#define CALIB_CFG_L2_LQ_TEST_VOLTAGE_V (CALIB_CFG_L2_R_TEST_VOLTAGE_V * 1.3f)
#define CALIB_CFG_L2_LQ_TEST_TIME_S    (MOTOR_TAU_S * 3.0f)
#define CALIB_CFG_L2_LQ_TEST_TICKS     (uint32_t)(CALIB_CFG_L2_LQ_TEST_TIME_S * CALIB_TICKS_PER_SEC)
#define CALIB_CFG_L2_LQ_SAMPLE_COUNT   30 /* 暂态采样点数 */
#define CALIB_CFG_L2_LQ_SKIP_TICKS     2u  /* 阶跃后跳过前 2 拍 */

/* flux 辨识（反电势法）
 * 目标转速自适应：目标反电势 = max(V_dt×3, 0.5V)，确保信噪比充足
 * 反推转速 omega = V_target / (flux × pp)，低 flux 电机自动提速 */
#define CALIB_CFG_L2_FLUX_TARGET_BEMF_V \
	((CALIB_CFG_L2_R_V_DT_ESTIMATE_V * 3.0f) > 0.5f \
	 ? (CALIB_CFG_L2_R_V_DT_ESTIMATE_V * 3.0f) \
	 : 0.5f)
#define CALIB_CFG_L2_FLUX_TARGET_SPEED_RAD_S \
	(CALIB_CFG_L2_FLUX_TARGET_BEMF_V / (MOTOR_FLUX * MOTOR_POLE_PAIRS))
/* 限幅：转速不超过电机最大转速的 0.3 倍，也不低于 5 rad/s */
#define CALIB_CFG_L2_FLUX_TARGET_SPEED_CLAMPED_RAD_S \
	(CALIB_CFG_L2_FLUX_TARGET_SPEED_RAD_S > (MOTOR_MAX_SPEED * 0.3f) \
	 ? (MOTOR_MAX_SPEED * 0.3f) \
	 : (CALIB_CFG_L2_FLUX_TARGET_SPEED_RAD_S < 5.0f \
	    ? 5.0f \
	    : CALIB_CFG_L2_FLUX_TARGET_SPEED_RAD_S))
#define CALIB_CFG_L2_FLUX_SPIN_VOLTAGE_V \
	(CALIB_CFG_TEST_CURRENT_ACTUAL_A * MOTOR_R + \
	 MOTOR_FLUX * MOTOR_POLE_PAIRS * CALIB_CFG_L2_FLUX_TARGET_SPEED_CLAMPED_RAD_S)
#define CALIB_CFG_L2_FLUX_SPIN_TIME_S        2.0f /* 稳速转动时间(s)，让滤波收敛 */
#define CALIB_CFG_L2_FLUX_SPIN_TICKS         (uint32_t)(CALIB_CFG_L2_FLUX_SPIN_TIME_S * CALIB_TICKS_PER_SEC)
#define CALIB_CFG_L2_FLUX_SAMPLE_COUNT       200  /* 稳态采样次数 */
/* flux 稳速判断：最近 N 个采样转速方差/均值 < 阈值则认为稳速 */
#define CALIB_CFG_L2_FLUX_SPEED_STABLE_WINDOW  50u
#define CALIB_CFG_L2_FLUX_SPEED_STABLE_RATIO   0.05f  /* 方差/均值 < 5% */
#define CALIB_CFG_L2_FLUX_OMEGA_E_MIN_RAD_S    5.0f   /* 电气角速度下限 */

/* ===================== 交流注入法参数（低阻电机 R/Ld 辨识）=====================
 * 在 d 轴施加 ud = U_dc + U_ac·sin(2π·f·t)，采样 id，相敏检测分离：
 *   同相分量 → R = U_ac·cos(φ) / I_ac
 *   正交分量 → Ld = U_ac·sin(φ) / (ω·I_ac)
 * 频率选择：f = 0.3/(2π·τ)，时间常数特征频率的 0.3 倍
 *   保证 ω·Ld 与 R 可比，相敏检测灵敏度最优 */
#define CALIB_CFG_AC_INJECT_FREQ_HZ \
	(0.3f / (2.0f * 3.14159265f * MOTOR_TAU_S))
#define CALIB_CFG_AC_INJECT_AMP_V    (CALIB_CFG_TEST_CURRENT_ACTUAL_A * MOTOR_R * 0.5f)
#define CALIB_CFG_AC_INJECT_DC_V     (CALIB_CFG_TEST_CURRENT_ACTUAL_A * MOTOR_R)
/* 预热时间：10×τ，让暂态衰减完毕 */
#define CALIB_CFG_AC_INJECT_WARMUP_TICKS  (uint32_t)(MOTOR_TAU_S * 10.0f * CALIB_TICKS_PER_SEC)
/* 采样周期数：采 5 个完整交流周期 */
#define CALIB_CFG_AC_INJECT_CYCLES        5u
#define CALIB_CFG_AC_INJECT_TOTAL_TICKS \
	(uint32_t)((float)CALIB_CFG_AC_INJECT_CYCLES / CALIB_CFG_AC_INJECT_FREQ_HZ * CALIB_TICKS_PER_SEC)
#define CALIB_CFG_AC_INJECT_SAMPLE_COUNT  200

/* 相序识别：对齐/步进电压 = 测试电流 × R */
#define CALIB_CFG_L2_PHASE_SEQ_VOLTAGE_V   (CALIB_CFG_TEST_CURRENT_A * MOTOR_R)
#define CALIB_CFG_L2_PHASE_SEQ_ALIGN_S     1.0f /* 对齐等待(s) */
#define CALIB_CFG_L2_PHASE_SEQ_ALIGN_TICKS (uint32_t)(CALIB_CFG_L2_PHASE_SEQ_ALIGN_S * CALIB_TICKS_PER_SEC)
#define CALIB_CFG_L2_PHASE_SEQ_STEP_S      0.5f /* 步进后等待(s) */
#define CALIB_CFG_L2_PHASE_SEQ_STEP_TICKS  (uint32_t)(CALIB_CFG_L2_PHASE_SEQ_STEP_S * CALIB_TICKS_PER_SEC)

/* 极对数辨识：开环强制电角度扫描法
 * 施加 ud 锁定转子跟随"强制电角度"，匀速扫过 N 个完整电周期，
 *   pole_pairs = 命令电角度变化(N·2π，精确已知) / 实测机械角变化*/
#define CALIB_CFG_L2_POLE_PAIRS_VOLTAGE_V   (CALIB_CFG_TEST_CURRENT_A * MOTOR_R)
#define CALIB_CFG_L2_POLE_PAIRS_ALIGN_S     1.0f /* 对齐 d 轴等待(s)，让转子锁到 theta=0 */
#define CALIB_CFG_L2_POLE_PAIRS_ALIGN_TICKS (uint32_t)(CALIB_CFG_L2_POLE_PAIRS_ALIGN_S * CALIB_TICKS_PER_SEC)
#define CALIB_CFG_L2_POLE_PAIRS_ELE_CYCLES  8u   /* 扫描的完整电周期数（越多量化误差越小）*/
#define CALIB_CFG_L2_POLE_PAIRS_ELE_FREQ_HZ 2.0f /* 电角度扫描频率(电周期/秒)，慢速确保转子跟随 */
/* 每 tick 电角度增量(rad) = 2π·f / ticks_per_sec */
#define CALIB_CFG_L2_POLE_PAIRS_DTHETA_RAD (2.0f * 3.14159265F * CALIB_CFG_L2_POLE_PAIRS_ELE_FREQ_HZ / CALIB_TICKS_PER_SEC)
/* 扫描目标：命令电角度累加到 N·2π 即结束 */
#define CALIB_CFG_L2_POLE_PAIRS_TARGET_RAD (2.0f * 3.14159265F * (float)CALIB_CFG_L2_POLE_PAIRS_ELE_CYCLES)

/* ===================== L3 编码器校准参数（从 MOTOR_* 派生）===================== */
/* 零位标定：对齐电压 = 测试电流 × R（d 轴锁定）*/
#define CALIB_CFG_L3_ALIGN_VOLTAGE_V (CALIB_CFG_TEST_CURRENT_A * MOTOR_R)
#define CALIB_CFG_L3_ALIGN_TIME_S    2.0f /* 对齐稳定等待时间(s) */
#define CALIB_CFG_L3_ALIGN_TICKS     (uint32_t)(CALIB_CFG_L3_ALIGN_TIME_S * CALIB_TICKS_PER_SEC)
#define CALIB_CFG_L3_SAMPLE_COUNT    100  /* 零位标定采样次数（取平均滤波）*/
/* 方向标定：uq 电压 = 测试电流 × R × 0.7（略小，可靠克服静摩擦）*/
#define CALIB_CFG_L3_DIR_VOLTAGE_V (CALIB_CFG_TEST_CURRENT_A * MOTOR_R * 0.7f)
#define CALIB_CFG_L3_DIR_TIME_S    1.0f /* 方向测试持续时间(s) */
#define CALIB_CFG_L3_DIR_TICKS     (uint32_t)(CALIB_CFG_L3_DIR_TIME_S * CALIB_TICKS_PER_SEC)

/* ===================== 全局健壮性参数 ===================== */
#define CALIB_CFG_GLOBAL_TIMEOUT_S     30.0f /* 单次标定全局超时(s)，防止电机卡转挂死 */
#define CALIB_CFG_GLOBAL_TIMEOUT_TICKS (uint32_t)(CALIB_CFG_GLOBAL_TIMEOUT_S * CALIB_TICKS_PER_SEC)
#define CALIB_CFG_MAX_VOLTAGE_MAG_V    7.0f  /* 标定施加电压幅值上限(√(ud²+uq²)) */

/* ===================== 结果合理性范围（calib_validate.h 用）=====================
 * 从 motor_profile.h 的 MOTOR_* 参数派生。
 * 切换电机型号时自动适配，无需手动调整。
 * 拦截短路/断路/异常值，兼顾误测拦截与误触发规避。
 *
 * R 范围按标称值派生：0.2×R ~ 5×R，兼顾误差容忍与异常拦截
 * 切换电机型号时自动适配，无需手动调整。*/
#define CALIB_CFG_R_MIN_OHM      (MOTOR_R * 0.2f)    /* R 下限: 标称值的 0.2 倍 */
#define CALIB_CFG_R_MAX_OHM      (MOTOR_R * 5.0f)    /* R 上限: 标称值的 5 倍 */
/* 安全下限：不低于 10mΩ（防短路判 0）*/
#define CALIB_CFG_R_MIN_OHM_SAFE 0.01f
#define CALIB_CFG_R_MIN_OHM_FINAL \
	(CALIB_CFG_R_MIN_OHM > CALIB_CFG_R_MIN_OHM_SAFE \
	 ? CALIB_CFG_R_MIN_OHM : CALIB_CFG_R_MIN_OHM_SAFE)
#define CALIB_CFG_LD_MIN_H       (MOTOR_LD * 0.2f)   /* Ld 下限 */
#define CALIB_CFG_LD_MAX_H       (MOTOR_LD * 3.0f)   /* Ld 上限 */
#define CALIB_CFG_LQ_MIN_H       (MOTOR_LQ * 0.2f)   /* Lq 下限 */
#define CALIB_CFG_LQ_MAX_H       (MOTOR_LQ * 3.0f)   /* Lq 上限 */
#define CALIB_CFG_FLUX_MIN_WB    (MOTOR_FLUX * 0.2f) /* flux 下限 */
#define CALIB_CFG_FLUX_MAX_WB    (MOTOR_FLUX * 3.0f) /* flux 上限 */
#define CALIB_CFG_POLE_PAIRS_MIN 1                   /* 极对数下限（通用）*/
#define CALIB_CFG_POLE_PAIRS_MAX 14                  /* 极对数上限（覆盖 2N-28P）*/

#endif                                               /* __CALIB_CONFIG_H__ */
