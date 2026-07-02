#ifndef __CALIB_CONFIG_H__
#define __CALIB_CONFIG_H__

/* ===================== 标定集中配置 =====================
 * 所有标定可调参数（电压/时间/采样数）集中于此。
 * 各 level 模块 #include 引用，避免魔法数散落。
 * 修改参数只需改本文件，无需动算法源码。
 *
 * 时间相关 TICK 数约定：控制环频率 10kHz（dt=100us），
 * 秒数 × 10000 得到 TICK。若控制环频率改动，
 * 仅需修改下面的 CALIB_TICKS_PER_SEC 宏。*/

#define CALIB_TICKS_PER_SEC 10000.0f

/* ===================== L1 驱动硬件底层参数（预留）===================== */
#define CALIB_CFG_L1_ADC_OFFSET_SAMPLES 1000 /* ADC偏置采样次数（取平均）*/
#define CALIB_CFG_L1_VBUS_SAMPLE_COUNT  500  /* 母线电压采样次数 */

/* ===================== L2 电机电气身份参数 ===================== */
/* R 辨识（DC 法）*/
#define CALIB_CFG_L2_R_TEST_VOLTAGE_V 0.5f /* DC 测试电压(V) */
#define CALIB_CFG_L2_R_TEST_TIME_S    1.0f /* 稳态等待(s) */
#define CALIB_CFG_L2_R_TEST_TICKS     (uint32_t)(CALIB_CFG_L2_R_TEST_TIME_S * CALIB_TICKS_PER_SEC)
#define CALIB_CFG_L2_R_SAMPLE_COUNT   200  /* 稳态采样次数 */

/* Ld 辨识（d 轴阶跃响应）*/
#define CALIB_CFG_L2_LD_TEST_VOLTAGE_V 2.0f   /* d 轴阶跃电压(V) */
#define CALIB_CFG_L2_LD_TEST_TIME_S    0.005f /* 暂态采样窗口(s) */
#define CALIB_CFG_L2_LD_TEST_TICKS     (uint32_t)(CALIB_CFG_L2_LD_TEST_TIME_S * CALIB_TICKS_PER_SEC)
#define CALIB_CFG_L2_LD_SAMPLE_COUNT   50     /* 暂态采样点数 */

/* Lq 辨识（q 轴阶跃响应）*/
#define CALIB_CFG_L2_LQ_TEST_VOLTAGE_V 2.0f   /* q 轴阶跃电压(V) */
#define CALIB_CFG_L2_LQ_TEST_TIME_S    0.005f /* 暂态采样窗口(s) */
#define CALIB_CFG_L2_LQ_TEST_TICKS     (uint32_t)(CALIB_CFG_L2_LQ_TEST_TIME_S * CALIB_TICKS_PER_SEC)
#define CALIB_CFG_L2_LQ_SAMPLE_COUNT   50     /* 暂态采样点数 */

/* flux 辨识（反电势法）*/
#define CALIB_CFG_L2_FLUX_SPIN_VOLTAGE_V     3.0f  /* 驱动电压(V) */
#define CALIB_CFG_L2_FLUX_SPIN_TIME_S        2.0f  /* 稳速转动时间(s) */
#define CALIB_CFG_L2_FLUX_SPIN_TICKS         (uint32_t)(CALIB_CFG_L2_FLUX_SPIN_TIME_S * CALIB_TICKS_PER_SEC)
#define CALIB_CFG_L2_FLUX_SAMPLE_COUNT       200   /* 稳态采样次数 */
#define CALIB_CFG_L2_FLUX_TARGET_SPEED_RAD_S 10.0f /* 目标机械转速(rad/s) */

/* 相序识别 */
#define CALIB_CFG_L2_PHASE_SEQ_VOLTAGE_V   1.0f /* 对齐/步进电压(V) */
#define CALIB_CFG_L2_PHASE_SEQ_ALIGN_S     1.0f /* 对齐等待(s) */
#define CALIB_CFG_L2_PHASE_SEQ_ALIGN_TICKS (uint32_t)(CALIB_CFG_L2_PHASE_SEQ_ALIGN_S * CALIB_TICKS_PER_SEC)
#define CALIB_CFG_L2_PHASE_SEQ_STEP_S      0.5f /* 步进后等待(s) */
#define CALIB_CFG_L2_PHASE_SEQ_STEP_TICKS  (uint32_t)(CALIB_CFG_L2_PHASE_SEQ_STEP_S * CALIB_TICKS_PER_SEC)

/* 极对数辨识 */
#define CALIB_CFG_L2_POLE_PAIRS_VOLTAGE_V  1.0f /* 驱动电压(V) */
#define CALIB_CFG_L2_POLE_PAIRS_SPIN_S     1.0f /* 转动时间(s) */
#define CALIB_CFG_L2_POLE_PAIRS_SPIN_TICKS (uint32_t)(CALIB_CFG_L2_POLE_PAIRS_SPIN_S * CALIB_TICKS_PER_SEC)

/* ===================== L3 编码器校准参数（已实现，从 calib_level3_encoder.c 迁移）===================== */
#define CALIB_CFG_L3_ALIGN_VOLTAGE_V 1.5f /* d轴对齐电压(V) */
#define CALIB_CFG_L3_ALIGN_TIME_S    2.0f /* 对齐稳定等待时间(s) */
#define CALIB_CFG_L3_ALIGN_TICKS     (uint32_t)(CALIB_CFG_L3_ALIGN_TIME_S * CALIB_TICKS_PER_SEC)
#define CALIB_CFG_L3_SAMPLE_COUNT    100  /* 零位标定采样次数（取平均滤波）*/
#define CALIB_CFG_L3_DIR_VOLTAGE_V   0.5f /* 方向测试uq电压(V) */
#define CALIB_CFG_L3_DIR_TIME_S      1.0f /* 方向测试持续时间(s) */
#define CALIB_CFG_L3_DIR_TICKS       (uint32_t)(CALIB_CFG_L3_DIR_TIME_S * CALIB_TICKS_PER_SEC)

/* ===================== 全局健壮性参数 ===================== */
#define CALIB_CFG_GLOBAL_TIMEOUT_S     30.0f /* 单次标定全局超时(s)，防止电机卡转挂死 */
#define CALIB_CFG_GLOBAL_TIMEOUT_TICKS (uint32_t)(CALIB_CFG_GLOBAL_TIMEOUT_S * CALIB_TICKS_PER_SEC)
#define CALIB_CFG_MAX_VOLTAGE_MAG_V    5.0f  /* 标定施加电压幅值上限(√(ud²+uq²))，防止烧管子 */

/* ===================== 结果合理性范围（calib_validate.h 用）===================== */
#define CALIB_CFG_R_MIN_OHM      0.001f /* R 相电阻下限(Ω) */
#define CALIB_CFG_R_MAX_OHM      100.0f /* R 相电阻上限(Ω) */
#define CALIB_CFG_LD_MIN_H       1e-6f  /* Ld d轴电感下限(H) */
#define CALIB_CFG_LD_MAX_H       1e-1f  /* Ld d轴电感上限(H) */
#define CALIB_CFG_LQ_MIN_H       1e-6f  /* Lq q轴电感下限(H) */
#define CALIB_CFG_LQ_MAX_H       1e-1f  /* Lq q轴电感上限(H) */
#define CALIB_CFG_FLUX_MIN_WB    1e-4f  /* flux 磁链下限(Wb) */
#define CALIB_CFG_FLUX_MAX_WB    1.0f   /* flux 磁链上限(Wb) */
#define CALIB_CFG_POLE_PAIRS_MIN 1      /* 极对数下限 */
#define CALIB_CFG_POLE_PAIRS_MAX 20     /* 极对数上限 */

#endif                                  /* __CALIB_CONFIG_H__ */
