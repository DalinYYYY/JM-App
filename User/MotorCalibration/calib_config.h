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
#define CALIB_LOW_R_THRESHOLD  0.5f

/* 测试电流安全上限（电源限流保护，与电机型号无关）*/
#define CALIB_CFG_MAX_TEST_CURRENT_A 1.5f

/* 标定施加电压幅值上限(√(ud²+uq²))，与电机型号无关 */
#define CALIB_CFG_MAX_VOLTAGE_MAG_V    7.0f

/* 全局超时(s)，与电机型号无关 */
#define CALIB_CFG_GLOBAL_TIMEOUT_S     30.0f
#define CALIB_CFG_GLOBAL_TIMEOUT_TICKS (uint32_t)(CALIB_CFG_GLOBAL_TIMEOUT_S * CALIB_TICKS_PER_SEC)

/* ===================== L1 驱动硬件底层参数（与电机无关）===================== */
#define CALIB_CFG_L1_ADC_OFFSET_SAMPLES 1000
#define CALIB_CFG_L1_VBUS_SAMPLE_COUNT  500

/* ===================== L2 采样次数（与电机无关）===================== */
#define CALIB_CFG_L2_R_SAMPLE_COUNT      500
#define CALIB_CFG_L2_LD_SAMPLE_COUNT     30
#define CALIB_CFG_L2_LD_SKIP_TICKS       2u
#define CALIB_CFG_L2_LQ_SAMPLE_COUNT     30
#define CALIB_CFG_L2_LQ_SKIP_TICKS       2u
#define CALIB_CFG_L2_FLUX_SAMPLE_COUNT   200

/* ===================== L2 死区压降估计（与电机无关的保守值）===================== */
#define CALIB_CFG_L2_R_V_DT_ESTIMATE_V   0.5f

/* ===================== L2 flux 稳速判断（与电机无关）===================== */
#define CALIB_CFG_L2_FLUX_SPEED_STABLE_WINDOW  50u
#define CALIB_CFG_L2_FLUX_SPEED_STABLE_RATIO   0.05f
#define CALIB_CFG_L2_FLUX_OMEGA_E_MIN_RAD_S    5.0f
#define CALIB_CFG_L2_FLUX_SPIN_TIME_S          2.0f

/* ===================== 交流注入采样次数（与电机无关）===================== */
#define CALIB_CFG_AC_INJECT_CYCLES        5u
#define CALIB_CFG_AC_INJECT_SAMPLE_COUNT  200

/* ===================== L2 相序/极对数时间（与电机无关）===================== */
#define CALIB_CFG_L2_PHASE_SEQ_ALIGN_S     1.0f
#define CALIB_CFG_L2_PHASE_SEQ_STEP_S      0.5f
#define CALIB_CFG_L2_POLE_PAIRS_ALIGN_S    1.0f
#define CALIB_CFG_L2_POLE_PAIRS_ELE_CYCLES 8u
#define CALIB_CFG_L2_POLE_PAIRS_ELE_FREQ_HZ 2.0f

/* ===================== L3 编码器校准时间（与电机无关）===================== */
#define CALIB_CFG_L3_ALIGN_TIME_S    2.0f
#define CALIB_CFG_L3_SAMPLE_COUNT    100
#define CALIB_CFG_L3_DIR_TIME_S      1.0f

/* ===================== 极对数通用范围（与电机型号无关）===================== */
#define CALIB_CFG_R_MIN_OHM_SAFE 0.01f
#define CALIB_CFG_POLE_PAIRS_MIN 1
#define CALIB_CFG_POLE_PAIRS_MAX 14

#endif /* __CALIB_CONFIG_H__ */
