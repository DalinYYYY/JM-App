#ifndef __DEV_CONFIG_BOARD_V1_H__
#define __DEV_CONFIG_BOARD_V1_H__

/* ======================================================================
 * Board: V1 (STM32G474CETx, 参考源工程)
 * 本文件显式声明所有板级参数, 与 dev_config.h 的校验项一一对应
 * 新板子可复制本文件作为起点, 逐项修改参数值
 * ====================================================================== */

/* ===== 1. 设备使能集（按板载硬件勾选）===== */
#define USE_DEV_HALF_BRIDGE     /* 半桥驱动 (TIM1 CH1/2/3 → 栅极) */
#define USE_DEV_MT6701          /* MT6701 磁编码器 (SSI) */
#define USE_DEV_MT6835         /* MT6835 磁编码器 (SPI, 备用) */
#define USE_DEV_POWER_MONITOR   /* 母线电压/电流/温度监控 */
#define USE_DEV_PHASE_CURRENT   /* 三相电流采样 (内置运放 → ADC) */
#define USE_DEV_DWT_COUNTER     /* DWT 周期计数器 (性能剖析) */
#define USE_DEV_COMMUN_UART     /* 上位机 UART 通信 */
/* 不启用: USE_DEV_AS5047 / USE_DEV_DRV8301 / USE_DEV_RGB_LED / USE_DEV_FLASH */

/* ===== 2. 编码器型号 ===== */
#define DEV_MOTOR_ENCODER_TYPE DEV_MOTOR_ENCODER_MT6701

/* ===== 3. PWM 时基 (TIM1 中心对齐) =====
 * HALF_BRIDGE_PWM_PERIOD 必须与 Board/V1/Src/tim.c htim1.Init.Period 一致
 * G4 默认 ARR=8500, CCR 取 ARR-20 */
#define HALF_BRIDGE_PWM_PERIOD (8500u)
#define HALF_BRIDGE_ADC_TRIG_CCR (HALF_BRIDGE_PWM_PERIOD - 20u)

/* ===== 4. 相电流采样链路 (内置运放, GAIN=50V/V, 采样电阻 0.01Ω) =====
 * V1 用 G4 内置运放, 与 ODrive 的 DRV8301 外置运放方案不同
 * 采样链路参数必须保持一致: 改运放/GAIN/分流电阻时同步修改 */
#define PHASE_CURRENT_GAIN  (50.0f)
#define PHASE_CURRENT_SHUNT (0.01f)

/* ===== 5. 母线电压分压 =====
 * V1 分压网络 11:1 */
#define PM_VBUS_RATIO (11.0f)

/* ===== 6. 母线电流来源 =====
 * V1 用 INA199B1 硬件采样母线电流 */
#define PM_IBUS_SOURCE (0)
/* INA199B1: gain=50, shunt=0.01Ω → 1/(gain*shunt) = 1/0.5 = 2.0 */
#define PM_IBUS_RATIO (2.0f)

/* ===== 7. Flash 存储 =====
 * V1 未启用 USE_DEV_FLASH, 无需定义 MOTORINFO_FLASH_* */

#endif /* __DEV_CONFIG_BOARD_V1_H__ */
