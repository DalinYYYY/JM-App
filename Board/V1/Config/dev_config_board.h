#ifndef __DEV_CONFIG_BOARD_V1_H__
#define __DEV_CONFIG_BOARD_V1_H__

/* ======================================================================
 * Board: V1 (STM32G474CETx, 参考源工程)
 * ====================================================================== */

/* ===== 1. 设备使能集（按板载硬件勾选）===== */
#define USE_DEV_HALF_BRIDGE   /* 半桥驱动 (TIM1 CH1/2/3 → 栅极) */
#define USE_DEV_MT6701        /* MT6701 磁编码器 (SPI3, 副编码器备料) */
#define USE_DEV_MT6835        /* MT6835 磁编码器 (SPI1, 主编码器) */
#define USE_DEV_POWER_MONITOR /* 母线电压/电流/温度监控 */
#define USE_DEV_PHASE_CURRENT /* 三相电流采样 (外置 INA240A2 运放 → ADC) */
#define USE_DEV_DWT_COUNTER   /* DWT 周期计数器 (性能剖析) */
#define USE_DEV_COMMUN_UART   /* 上位机 UART 通信 */
#define USE_DEV_LED           /* LED 状态指示 (LED1红=PC14 故障, LED2绿=PC15 运行状态) */
#define USE_DEV_FLASH         /* Flash 参数存储 (Bank2 末尾 4KB, 供 0xE6-0xEB 命令组) */
/* 不启用: USE_DEV_AS5047 / USE_DEV_DRV8301 / USE_DEV_RGB_LED (V1 用单色 LED) */

/* ===== 2. 编码器型号 =====
 * V1 板 MT6835 为主编码器接 SPI1, MT6701 为副编码器备料接 SPI3 */
#define DEV_MOTOR_ENCODER_TYPE DEV_MOTOR_ENCODER_MT6835

/* ===== 3. PWM 时基 (TIM1 中心对齐) =====
 * HALF_BRIDGE_PWM_PERIOD 必须与 Board/V1/Src/tim.c htim1.Init.Period 一致
 * G4 默认 ARR=8500, CCR 取 ARR-20 */
#define HALF_BRIDGE_PWM_PERIOD   (8500u)
#define HALF_BRIDGE_ADC_TRIG_CCR (HALF_BRIDGE_PWM_PERIOD - 20u)

/* ===== 4. 相电流采样链路 (外置 INA240A2 运放, GAIN=50V/V, 采样电阻 1mΩ) =====
 * V1 用外置 INA240A2 运放 (U26/U27/U28),采样电阻 R33/R34/R35 = 1mΩ (0.001Ω) */
#define PHASE_CURRENT_GAIN  (50.0f)
#define PHASE_CURRENT_SHUNT (0.001f)

/* ===== 5. 母线电压分压 =====
 * V1 分压网络 */
#define PM_VBUS_RATIO (15.70588f)

/* ===== 6. 母线电流来源 =====
 * V1 无独立 IBUS 采样硬件 , 使用三相电流 + SVPWM 占空比合成
 * 合成公式: Ibus = da*Ia + db*Ib + dc*Ic (功率守恒推导, 详见 dev_power_monitor.c)
 * IBUS 来源由配置表 type=PM_CH_IBUS_SYNTH 表达, 不再用 PM_IBUS_SOURCE 宏 */
/* NTC1(PB0)/NTC2(PB1) 温度采样通道硬件已布线, 驱动层暂返回0占位 (计算逻辑待实现) */

/* ===== 7. 电源监控通道数 =====
 * V1: VBUS + IBUS_SYNTH + NTC1(TEMP_DRIVER) + NTC2(TEMP_MOTOR) = 4 通道 */
#define PM_CH_MAX (4)

/* ===== 8. Flash 存储: Bank2 末尾 (0x0804F000, 4KB) =====
 * G474CETx 双 Bank Flash (512KB, 每 Bank 256KB), Bank2 末尾 4KB 区域存储 motor_info
 * Bank1: 0x08000000-0x0803FFFF (256KB, 代码区, 链接脚本 LR_IROM1=0x80000)
 * Bank2: 0x08040000-0x0807FFFF (256KB, 末尾4KB为参数存储区)
 * 注: 与 SFOC 板共用 0x0804F000 地址, 便于三板存储区布局统一
 *     链接脚本 stm32g474xx_flash.sct 已限制代码区为 0x80000(512KB), 不覆盖参数区 */
#define MOTORINFO_FLASH_START_ADDR 0x0804F000U
#define MOTORINFO_FLASH_TOTAL_SIZE 0x00001000U /* 4KB */
#define MOTORINFO_FLASH_PAGE_SIZE  2048U       /* 2KB, G4 双 Bank 页大小 */

#endif                                         /* __DEV_CONFIG_BOARD_V1_H__ */
