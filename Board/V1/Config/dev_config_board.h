#ifndef __DEV_CONFIG_BOARD_V1_H__
#define __DEV_CONFIG_BOARD_V1_H__

/* ======================================================================
 * Board: V1 (STM32G474CETx, 参考源工程)
 * 本文件显式声明所有板级参数, 与 dev_config.h 的校验项一一对应
 * 新板子可复制本文件作为起点, 逐项修改参数值
 * ====================================================================== */

/* ===== 1. 设备使能集（按板载硬件勾选）===== */
#define USE_DEV_HALF_BRIDGE     /* 半桥驱动 (TIM1 CH1/2/3 → 栅极) */
#define USE_DEV_MT6701          /* MT6701 磁编码器 (SSI, 副编码器备料) */
#define USE_DEV_MT6835          /* MT6835 磁编码器 (SPI3, 主编码器) */
#define USE_DEV_POWER_MONITOR   /* 母线电压/电流/温度监控 */
#define USE_DEV_PHASE_CURRENT   /* 三相电流采样 (外置 INA240A2 运放 → ADC) */
#define USE_DEV_DWT_COUNTER     /* DWT 周期计数器 (性能剖析) */
#define USE_DEV_COMMUN_UART     /* 上位机 UART 通信 */
/* 不启用: USE_DEV_AS5047 / USE_DEV_DRV8301 / USE_DEV_RGB_LED / USE_DEV_FLASH */

/* ===== 2. 编码器型号 =====
 * V1 板 MT6835(U18) 为主编码器接 SPI3, MT6701(U19) 为副编码器备料接 SPI1 */
#define DEV_MOTOR_ENCODER_TYPE DEV_MOTOR_ENCODER_MT6835

/* ===== 3. PWM 时基 (TIM1 中心对齐) =====
 * HALF_BRIDGE_PWM_PERIOD 必须与 Board/V1/Src/tim.c htim1.Init.Period 一致
 * G4 默认 ARR=8500, CCR 取 ARR-20 */
#define HALF_BRIDGE_PWM_PERIOD (8500u)
#define HALF_BRIDGE_ADC_TRIG_CCR (HALF_BRIDGE_PWM_PERIOD - 20u)

/* ===== 4. 相电流采样链路 (外置 INA240A2 运放, GAIN=50V/V, 采样电阻 1mΩ) =====
 * V1 用外置 INA240A2 运放 (U26/U27/U28), 与 ODrive 的 DRV8301 内置运放方案不同
 * 采样电阻 R33/R34/R35 = 1mΩ (0.001Ω), 详见原理图 SCH_FOC_SCH_V1.0
 * 采样链路参数必须保持一致: 改运放/GAIN/分流电阻时同步修改 */
#define PHASE_CURRENT_GAIN  (50.0f)
#define PHASE_CURRENT_SHUNT (0.001f)

/* ===== 5. 母线电压分压 =====
 * V1 分压网络 RH=1006kΩ / RL=100kΩ ≈ 11:1, 输出接 PA0 (ADC1_IN1) */
#define PM_VBUS_RATIO (11.0f)

/* ===== 6. 母线电流来源 =====
 * V1 无独立 IBUS 采样硬件 (原理图无 INA199B1), 使用三相电流 + SVPWM 占空比合成
 * 合成公式: Ibus = da*Ia + db*Ib + dc*Ic (功率守恒推导, 详见 dev_power_monitor.c)
 * IBUS 来源由配置表 type=PM_CH_IBUS_SYNTH 表达, 不再用 PM_IBUS_SOURCE 宏 */
/* NTC1(PB0)/NTC2(PB1) 温度采样通道硬件已布线, 驱动层暂返回0占位 (计算逻辑待实现) */

/* ===== 7. 电源监控通道数 =====
 * V1: VBUS + IBUS_SYNTH + NTC1(TEMP_DRIVER) + NTC2(TEMP_MOTOR) = 4 通道 */
#define PM_CH_MAX (4)

/* ===== 8. Flash 存储 =====
 * V1 未启用 USE_DEV_FLASH, 无需定义 MOTORINFO_FLASH_* */

#endif /* __DEV_CONFIG_BOARD_V1_H__ */
