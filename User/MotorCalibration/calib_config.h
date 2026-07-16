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
#define CALIB_CFG_L2_R_TEST_VOLTAGE_V    (CALIB_CFG_TEST_CURRENT_A * MOTOR_R)   /* 高档 V2 */
#define CALIB_CFG_L2_R_TEST_VOLTAGE_LO_V (CALIB_CFG_L2_R_TEST_VOLTAGE_V * 0.4f) /* 低档 V1 */
#define CALIB_CFG_L2_R_TEST_TIME_S       (MOTOR_TAU_S * 750.0f)
#define CALIB_CFG_L2_R_TEST_TICKS        (uint32_t)(CALIB_CFG_L2_R_TEST_TIME_S * CALIB_TICKS_PER_SEC)
#define CALIB_CFG_L2_R_SAMPLE_COUNT      500 /* 每档稳态采样次数 */

/* Ld 辨识（d 轴阶跃响应）
 * 阶跃电压 = R 测试电压 × 1.3（需更高电压产生 di/dt）
 * 暂态窗口 = 2.25τ（末端 di/dt 仍远 > 阈值，避免尾部噪声）*/
#define CALIB_CFG_L2_LD_TEST_VOLTAGE_V (CALIB_CFG_L2_R_TEST_VOLTAGE_V * 1.3f)
#define CALIB_CFG_L2_LD_TEST_TIME_S    (MOTOR_TAU_S * 2.25f)
#define CALIB_CFG_L2_LD_TEST_TICKS     (uint32_t)(CALIB_CFG_L2_LD_TEST_TIME_S * CALIB_TICKS_PER_SEC)
#define CALIB_CFG_L2_LD_SAMPLE_COUNT   30 /* 暂态采样点数 */

/* Lq 辨识（q 轴阶跃响应）—— 与 Ld 对称 */
#define CALIB_CFG_L2_LQ_TEST_VOLTAGE_V (CALIB_CFG_L2_R_TEST_VOLTAGE_V * 1.3f)
#define CALIB_CFG_L2_LQ_TEST_TIME_S    (MOTOR_TAU_S * 2.25f)
#define CALIB_CFG_L2_LQ_TEST_TICKS     (uint32_t)(CALIB_CFG_L2_LQ_TEST_TIME_S * CALIB_TICKS_PER_SEC)
#define CALIB_CFG_L2_LQ_SAMPLE_COUNT   30 /* 暂态采样点数 */

/* flux 辨识（反电势法）
 * 驱动电压 = R 压降 + 反电势 = 测试电流×R + flux×pp×目标转速
 * 目标机械转速取 10 rad/s（保守，确保 back-EMF 可测）*/
#define CALIB_CFG_L2_FLUX_TARGET_SPEED_RAD_S 10.0f
#define CALIB_CFG_L2_FLUX_SPIN_VOLTAGE_V     (CALIB_CFG_TEST_CURRENT_A * MOTOR_R + MOTOR_FLUX * MOTOR_POLE_PAIRS * CALIB_CFG_L2_FLUX_TARGET_SPEED_RAD_S)
#define CALIB_CFG_L2_FLUX_SPIN_TIME_S        2.0f /* 稳速转动时间(s)，让滤波收敛 */
#define CALIB_CFG_L2_FLUX_SPIN_TICKS         (uint32_t)(CALIB_CFG_L2_FLUX_SPIN_TIME_S * CALIB_TICKS_PER_SEC)
#define CALIB_CFG_L2_FLUX_SAMPLE_COUNT       200  /* 稳态采样次数 */

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
 * R 范围宽(0.01~50Ω): 低电阻电机(如 5010 R=0.12Ω)死区非线性使两点法
 * 结果可能偏离标称值数倍, 过窄范围会误拦。宽范围仅拦截明显异常(短路<0.01/断路>50)。*/
#define CALIB_CFG_R_MIN_OHM      0.01f               /* R 下限: 10mΩ(防短路判 0) */
#define CALIB_CFG_R_MAX_OHM      50.0f               /* R 上限: 50Ω(防断路判 ∞) */
#define CALIB_CFG_LD_MIN_H       (MOTOR_LD * 0.2f)   /* Ld 下限 */
#define CALIB_CFG_LD_MAX_H       (MOTOR_LD * 3.0f)   /* Ld 上限 */
#define CALIB_CFG_LQ_MIN_H       (MOTOR_LQ * 0.2f)   /* Lq 下限 */
#define CALIB_CFG_LQ_MAX_H       (MOTOR_LQ * 3.0f)   /* Lq 上限 */
#define CALIB_CFG_FLUX_MIN_WB    (MOTOR_FLUX * 0.2f) /* flux 下限 */
#define CALIB_CFG_FLUX_MAX_WB    (MOTOR_FLUX * 3.0f) /* flux 上限 */
#define CALIB_CFG_POLE_PAIRS_MIN 1                   /* 极对数下限（通用）*/
#define CALIB_CFG_POLE_PAIRS_MAX 14                  /* 极对数上限（覆盖 2N-28P）*/

#endif                                               /* __CALIB_CONFIG_H__ */
