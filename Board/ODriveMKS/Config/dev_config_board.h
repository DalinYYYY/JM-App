#ifndef __DEV_CONFIG_BOARD_ODRIVE_H__
#define __DEV_CONFIG_BOARD_ODRIVE_H__

/* ODrive Mini (STM32F405RGT6 + DRV8301 + AS5047P) 板级使能集与参数覆盖 */

#define USE_DEV_PHASE_CURRENT
#define USE_DEV_HALF_BRIDGE
#define USE_DEV_POWER_MONITOR
#define USE_DEV_FLASH
#define USE_DEV_AS5047
#define USE_DEV_DRV8301
#define USE_DEV_COMMUN_UART
/* 不启用: USE_DEV_MT6701 / USE_DEV_MT6835 / USE_DEV_RGB_LED / USE_DEV_DWT_COUNTER */

/* DRV8301 仅 2 路相电流输出, IC 由基尔霍夫合成 */
#define DRV8301_TWO_PHASE_CURRENT

/* 编码器型号选 AS5047 */
#define DEV_MOTOR_ENCODER_TYPE DEV_MOTOR_ENCODER_AS5047

/* ===== 相电流参数(0.5mΩ + DRV8301 GAIN=40) ===== */
#undef  PHASE_CURRENT_GAIN
#define PHASE_CURRENT_GAIN       (40.0f)
#undef  PHASE_CURRENT_SHUNT
#define PHASE_CURRENT_SHUNT      (0.0005f)

/* ===== 母线电压分压: ODrive R43=18K, R44=1K → 19:1 ===== */
#undef  PM_VBUS_RATIO
#define PM_VBUS_RATIO            (19.0f)

/* ===== 母线电流来源: 三相合成(ODrive 无独立 IBUS 硬件) ===== */
#undef  PM_IBUS_SOURCE
#define PM_IBUS_SOURCE           (1)

/* ===== ADC 触发比较值: F4 TIM1 Period=8400, CCR=8380(ARR-20) =====
 * G4 默认 8480(ARR=8500), F4 必须覆盖否则 CCR>ARR 导致 ADC 永不触发 */
#undef  HALF_BRIDGE_ADC_TRIG_CCR
#define HALF_BRIDGE_ADC_TRIG_CCR (8380u)

/* ===== Flash 存储: Sector 11 (0x080E0000, 128KB), Flash 尾部, 单扇区无磨损均衡 ===== */
#undef  MOTORINFO_FLASH_START_ADDR
#define MOTORINFO_FLASH_START_ADDR 0x080E0000U
#undef  MOTORINFO_FLASH_TOTAL_SIZE
#define MOTORINFO_FLASH_TOTAL_SIZE 0x00020000U /* 128KB */
#undef  MOTORINFO_FLASH_PAGE_SIZE
#define MOTORINFO_FLASH_PAGE_SIZE  0x00020000U /* 128KB, F4 Sector */

#endif /* __DEV_CONFIG_BOARD_ODRIVE_H__ */
