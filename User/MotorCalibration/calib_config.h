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
#define CALIB_CFG_L1_ADC_OFFSET_SAMPLES  1000  /* ADC偏置采样次数（取平均）*/
#define CALIB_CFG_L1_VBUS_SAMPLE_COUNT   500   /* 母线电压采样次数 */

/* ===================== L2 电机电气身份参数（预留）===================== */
#define CALIB_CFG_L2_R_TEST_VOLTAGE_V    0.5f   /* R辨识：施加的DC测试电压(V) */
#define CALIB_CFG_L2_R_TEST_TIME_S       1.0f   /* R辨识：稳态等待时间(s) */
#define CALIB_CFG_L2_R_SAMPLE_COUNT      200    /* R辨识：电流采样次数 */
#define CALIB_CFG_L2_LD_TEST_VOLTAGE_V   2.0f   /* Ld辨识：d轴阶跃电压(V) */
#define CALIB_CFG_L2_LD_TEST_TIME_S      0.005f /* Ld辨识：阶跃持续时间(s)，观测di/dt */
#define CALIB_CFG_L2_LQ_TEST_VOLTAGE_V   2.0f   /* Lq辨识：q轴阶跃电压(V) */
#define CALIB_CFG_L2_LQ_TEST_TIME_S      0.005f /* Lq辨识：阶跃持续时间(s) */
#define CALIB_CFG_L2_FLUX_SPIN_VOLTAGE_V 3.0f   /* flux辨识：驱动电压(V) */
#define CALIB_CFG_L2_FLUX_SPIN_TIME_S    2.0f   /* flux辨识：稳态转动时间(s) */
#define CALIB_CFG_L2_FLUS_SPEED_RAD_S    10.0f  /* flux辨识：目标转速(rad/s) */

/* ===================== L3 编码器校准参数（已实现，从 calib_level3_encoder.c 迁移）===================== */
#define CALIB_CFG_L3_ALIGN_VOLTAGE_V     1.5f   /* d轴对齐电压(V) */
#define CALIB_CFG_L3_ALIGN_TIME_S        2.0f   /* 对齐稳定等待时间(s) */
#define CALIB_CFG_L3_ALIGN_TICKS         (uint32_t)(CALIB_CFG_L3_ALIGN_TIME_S * CALIB_TICKS_PER_SEC)
#define CALIB_CFG_L3_SAMPLE_COUNT        100    /* 零位标定采样次数（取平均滤波）*/
#define CALIB_CFG_L3_DIR_VOLTAGE_V       0.5f   /* 方向测试uq电压(V) */
#define CALIB_CFG_L3_DIR_TIME_S          1.0f   /* 方向测试持续时间(s) */
#define CALIB_CFG_L3_DIR_TICKS           (uint32_t)(CALIB_CFG_L3_DIR_TIME_S * CALIB_TICKS_PER_SEC)

#endif /* __CALIB_CONFIG_H__ */
