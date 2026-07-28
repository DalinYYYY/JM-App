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
#define USE_DEV_COMMUN_CAN    /* 上位机 CAN/CAN-FD 通信 (CA-IS2062A 收发器) */
#define USE_DEV_LED           /* LED 状态指示 (LED1红=PC14 故障, LED2绿=PC15 运行状态) */
#define USE_DEV_FLASH         /* Flash 参数存储 (Bank2 末尾 4KB, 供 0xE6-0xEB 命令组) */
/* 不启用: USE_DEV_AS5047 / USE_DEV_DRV8301 / USE_DEV_RGB_LED (V1 用单色 LED) */

/* ===== 1.1 CAN/FD 通信配置 (CA-IS2062A 收发器, §Q11 已确认支持 FD) =====
 * V1 板收发器为 CA-IS2062A (芯景 ChipAnalog):
 *   - 支持 CAN FD (ISO 11898-2:2015)
 *   - 集成 5kV 数字隔离 (替代光耦)
 *   - 速率 ≤8Mbps (FD 数据段)
 *   - 待机功耗 <15µA
 * 详见方案文档 §AC1 收发器选型矩阵 */
#define USE_CAN_FD_MODE             1   /* 1=启用 CAN FD 模式, 0=仅经典 CAN */

/* FDCAN 位定时 (FDCANCLK = PCLK1/2 = 85MHz)
 * 标称段 1Mbps: Prescaler=5, Sync(1)+TS1(12)+TS2(4)=17Tq, 采样点 76.5%
 * 数据段 8Mbps: Prescaler=1, Sync(1)+TS1(7)+TS2(3)=11Tq, 采样点 72.7% (§AC4 修正) */
#define FDCAN_NOMINAL_BAUDRATE      1000000UL  /* 标称段 1Mbps */
#define FDCAN_DATA_BAUDRATE         8000000UL  /* 数据段 8Mbps */
#define FDCAN_NOMINAL_PRESCALER     5
#define FDCAN_NOMINAL_TS1           12
#define FDCAN_NOMINAL_TS2           4
#define FDCAN_DATA_PRESCALER        1
#define FDCAN_DATA_TS1              7
#define FDCAN_DATA_TS2              3

#define MOTOR_ID_DEFAULT            1   /* 本机默认 CAN 地址 (1~127) */

/* ===== 1.2 通信中断安全降级 (§Q13 已确认 TIMER 模式) =====
 * 收不到任何 CAN 帧 (含广播) 后:
 *   - 500ms 内: 保持最后指令 (HOLD)
 *   - 超过 500ms: 强制 IDLE 停机 */
#define JM_CAN_LOSS_ACTION          2       /* 0=IDLE立即停, 1=HOLD保持, 2=TIMER超时停 */
#define JM_CAN_LOSS_TIMEOUT_MS      500     /* TIMER 模式超时阈值 ms */

/* ===== 1.3 双通道主控仲裁 (§Q12 已确认开发期仅 motor_state 临界区) =====
 * 开发期: UART 和 CAN 可同时下发, 后写覆盖前写 (motor_state 临界区保证原子性)
 * 量产期: 改为 1 启用主控仲裁 (5s 超时释放, 首次控制类命令获取主控) */
#define JM_DUAL_CHANNEL_ARB_ENABLE  0       /* 0=开发期关闭主控仲裁, 1=量产期启用 */
/* 注: motor_state 临界区 (__disable_irq) 始终启用, 与本宏无关 (§AH3) */

/* ===== 1.4 鉴权令牌 (§Q15 已确认开发期关闭) =====
 * 开发期: 关闭鉴权, 便于调试
 * 量产期: 改为 1 启用, 产线工具写入随机令牌到 Flash (§AI1 出厂区扩展后) */
#define JM_CAN_AUTH_ENABLE          0       /* 0=开发期关闭, 1=量产期启用 */
#define JM_CAN_AUTH_TOKEN           0xA5A5A5A5UL  /* 占位, 量产时产线工具写入随机值 */

/* ===== 1.5 命令速率限制 (§Z2, 开发期默认启用防 Flash 损耗) =====
 * 对危险/高开销命令按类别设最小间隔, 超频返回 NACK(RATE_LIMIT)
 * 调试时若觉得间隔过严可临时设 0 关闭 */
#define JM_RATE_LIMIT_ENABLE        1       /* 1=启用速率限制, 0=关闭 */

/* ===== 2. 编码器型号 =====
 * V1 板 MT6835 为主编码器接 SPI1, MT6701 为副编码器备料接 SPI3 */
#define DEV_MOTOR_ENCODER_TYPE DEV_MOTOR_ENCODER_MT6835

/* ===== 3. PWM 时基 (TIM1 中心对齐) =====*/
#define HALF_BRIDGE_PWM_PERIOD (8500u)
// TODO: 目前V1版硬件因为原理图设计问题（PWM HL 高度接反了），导致采样的时机反向
#define HALF_BRIDGE_ADC_TRIG_CCR (200u) /* 谷底前 ~1.18µs 采样(下管导通窗口), 见上 */

/* ===== 4. 相电流采样链路 (外置 INA240A2 运放, GAIN=50V/V, 采样电阻 1mΩ) =====
 * V1 用外置 INA240A2 运放 (U26/U27/U28),采样电阻 R33/R34/R35 = 1mΩ (0.001Ω) */
#define PHASE_CURRENT_GAIN  (50.0f)
#define PHASE_CURRENT_SHUNT (0.001f)
/* V1 板 INA240 输入极性与电流约定相反, 软件取反修正 */
#define PHASE_CURRENT_POLARITY (1.0f)

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
