#ifndef __CALIB_CONFIG_H__
#define __CALIB_CONFIG_H__

/* ===================== 标定集中配置（与电机型号无关的常量）=====================
 * 所有与电机型号无关的标定可调参数集中于此。
 * 时间相关 TICK 数约定：控制环频率 10kHz（dt=100us）。
 *
 * 与电机型号相关的派生参数（基于 R/Ld/Lq/flux/pole_pairs/peak_current/max_speed）
 * 已迁移至 calib_config_runtime.h/c，运行期从 motor_info 读取派生。
 * 切换电机型号无需重编译固件，通过上位机 0xE7/0xEA 命令修改 motor_info 即可。
 */

#define CALIB_TICKS_PER_SEC 10000.0f

/* 电机分档识别阈值（与电机型号无关的通用阈值）*/
#define CALIB_LOW_R_THRESHOLD 0.5f

/* 测试电流安全上限（电源限流保护，与电机型号无关）*/
#define CALIB_CFG_MAX_TEST_CURRENT_A 1.5f

/* 标定施加电压幅值上限(√(ud²+uq²))，与电机型号无关 */
#define CALIB_CFG_MAX_VOLTAGE_MAG_V 7.0f

/* 全局超时(s)，与电机型号无关 */
#define CALIB_CFG_GLOBAL_TIMEOUT_S     30.0f
#define CALIB_CFG_GLOBAL_TIMEOUT_TICKS (uint32_t)(CALIB_CFG_GLOBAL_TIMEOUT_S * CALIB_TICKS_PER_SEC)

/* ===================== L1 驱动硬件底层参数（与电机无关）===================== */
#define CALIB_CFG_L1_ADC_OFFSET_SAMPLES 1000
#define CALIB_CFG_L1_VBUS_SAMPLE_COUNT  500

/* ===================== L2 采样次数（与电机无关）===================== */
#define CALIB_CFG_L2_R_SAMPLE_COUNT    500
#define CALIB_CFG_L2_LD_SAMPLE_COUNT   30
#define CALIB_CFG_L2_LD_SKIP_TICKS     2u
#define CALIB_CFG_L2_LQ_SAMPLE_COUNT   30
#define CALIB_CFG_L2_LQ_SKIP_TICKS     2u
#define CALIB_CFG_L2_FLUX_SAMPLE_COUNT 200

/* ===================== L2 死区压降估计（与电机无关的保守值）===================== */
#define CALIB_CFG_L2_R_V_DT_ESTIMATE_V 0.5f

/* ===================== L2 flux 稳速判断（与电机无关）===================== */
#define CALIB_CFG_L2_FLUX_SPEED_STABLE_WINDOW 50u
#define CALIB_CFG_L2_FLUX_SPEED_STABLE_RATIO  0.05f
#define CALIB_CFG_L2_FLUX_OMEGA_E_MIN_RAD_S   5.0f
#define CALIB_CFG_L2_FLUX_SPIN_TIME_S         2.0f

/* ===================== 交流注入采样次数（与电机无关）===================== */
#define CALIB_CFG_AC_INJECT_CYCLES       5u
#define CALIB_CFG_AC_INJECT_SAMPLE_COUNT 200

/* ===================== L2 相序/极对数时间（与电机无关）===================== */
#define CALIB_CFG_L2_PHASE_SEQ_ALIGN_S      1.0f
#define CALIB_CFG_L2_PHASE_SEQ_STEP_S       0.5f
#define CALIB_CFG_L2_POLE_PAIRS_ALIGN_S     1.0f
#define CALIB_CFG_L2_POLE_PAIRS_ELE_CYCLES  21u
#define CALIB_CFG_L2_POLE_PAIRS_ELE_FREQ_HZ 2.0f

/* ===================== L3 编码器校准时间（与电机无关）===================== */
#define CALIB_CFG_L3_ALIGN_TIME_S 2.0f
#define CALIB_CFG_L3_SAMPLE_COUNT 100
#define CALIB_CFG_L3_DIR_TIME_S   1.0f

/* ===================== 极对数通用范围（与电机型号无关）===================== */
#define CALIB_CFG_R_MIN_OHM_SAFE 0.01f
#define CALIB_CFG_POLE_PAIRS_MIN 1
#define CALIB_CFG_POLE_PAIRS_MAX 24 /* 24 覆盖多极对电机(RS03=21, 轮毂电机可达23) */

/* 标定对齐电流安全上限(低阻电机突破 1.5A 默认上限, 保证足够对齐力矩)
 * RS03(R=0.1Ω, 额定13A) 用 6A 时电压=0.6V, 占空比 1.25%, 力矩 1.2Nm(电机端)
 * 6A 为 46% 额定, 短时标定(扫描10s)安全, 避免大电流持续导致绕组发热
 * 3A 时力矩不足导致极对数辨识偏低(实测21→20), 提升至 6A 修复 */
#define CALIB_CFG_ALIGN_CURRENT_MAX_A 6.0f

/* 最小标定电压保底(伏特), 确保低阻电机有足够标定电流
 * 开环电压标定: V = I_test × R, 当 motor_info 中 R 值偏小(如 RS03 R=0.1Ω)时
 * 6A×0.1=0.6V 远低于死区压降0.5V, 实际绕组电压几乎为零。
 * 保底3.0V确保: 3.0V - 死区0.5V = 2.5V 实际绕组电压, RS03电流25A(<峰值43A)
 * 24V电源下占空比12.5%, 死区占比从33%(1.5V时)降到17%(3.0V时)
 * 受 CALIB_CFG_MAX_VOLTAGE_MAG_V=7V 限幅保护, 3.0V 远低于上限 */
#define CALIB_CFG_MIN_CALIB_VOLTAGE_V 3.0f

#endif /* __CALIB_CONFIG_H__ */
