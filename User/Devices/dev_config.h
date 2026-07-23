/**
 * @file dev_config.h
 * @brief 通用设备配置接口（跨板通用常量 + 编译期校验）
 *
 * 本文件只放跨板通用的常量（PM_VREF、分辨率、LPF 系数等）和编译期校验。
 * 所有板级相关的参数（分压比、相电流增益、PWM 周期、Flash 地址等）
 * 必须在各板的 dev_config_board.h 中显式定义。
 *
 * 新增配置项的工作流：
 *   1. 通用常量 → 直接 #define 在此文件
 *   2. 板级参数 → 在此文件加 #ifndef + #error 校验，在各板 dev_config_board.h 中 #define
 *   3. 仅特定模块用 → 用 #if defined(USE_DEV_xxx) 门控，不影响未启用该模块的板
 */
#ifndef __DEV_CONFIG_H__
#define __DEV_CONFIG_H__

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "board_select.h"

#define DEV_EOK     0
#define DEV_ERROR   1
#define DEV_ENABLE  1
#define DEV_DISABLE 0

/* ===================================================================== */
/* 通用常量（跨板不变，无需板级覆盖）                                     */
/* ===================================================================== */

#if defined(USE_DEV_POWER_MONITOR)
#define PM_VREF          (3.3f)  /* ADC 参考电压 */
#define PM_RESOLUTION    (4096.0f) /* 12-bit ADC */
#define PM_IBUS_OFFSET_V (1.65f) /* INA199B1 REF=VREF/2, 零电流偏置电压 */
#define PM_TEMP_RATIO    (10.0f) /* 温度采样分压比 */
/* 合成母线电流低通滤波系数 (SYNTH 源专用)
 * 抑制小电流时三相采样噪声叠加导致的波动, alpha=0.05 时 10kHz 采样下时间常数约 2ms */
#ifndef PM_IBUS_LPF_ALPHA
#define PM_IBUS_LPF_ALPHA (0.05f)
#endif
#endif

#if defined(USE_DEV_PHASE_CURRENT)
#define PHASE_CURRENT_VREF       (3.3f)    /* ADC 参考电压 */
#define PHASE_CURRENT_RESOLUTION (4096.0f) /* 12-bit ADC */
#define PHASE_CURRENT_LPF_ALPHA  (0.9f)    /* 相电流低通滤波系数 */
/* INA199B1 REF 标称 1.65V (VREF/2), 对应 ADC = 1.65/3.3 * 4096 = 2048
 * 仅作为校准前的兜底默认值; 启动时 cur_loop_calibrate_offset 会用实测均值覆盖 */
#ifndef PHASE_CURRENT_ZERO_ADC
#define PHASE_CURRENT_ZERO_ADC (2048u)
#endif
/* 电流采样极性: +1.0=正向, -1.0=反向(采样电阻/运放输入方向与约定相反时使用)
 * 板级根据 INA240 IN+/IN- 与采样电阻焊接方向决定是否覆盖为 -1.0f */
#ifndef PHASE_CURRENT_POLARITY
#define PHASE_CURRENT_POLARITY (1.0f)
#endif
#endif

#if defined(USE_DEV_COMMUN_VESC)
#define DEV_VESC_RX_BUF_SIZE (256u)
#endif

#if defined(USE_DEV_COMMUN_UART)
#define DEV_JM_UART_RX_BUF_SIZE (256u)
#endif

/* ===================================================================== */
/* 板级必填参数校验（漏配直接报错，指明在哪个板文件补填）                 */
/* ===================================================================== */

/* 电源监控通道数与分压比:
 *   PM_CH_MAX     : 规则组通道数(VBUS+IBUS+NTC等), 由板级配置表决定
 *   PM_VBUS_RATIO : 母线电压分压比(供配置表 scale 字段引用, 避免魔数)
 *   PM_IBUS_RATIO : 母线电流硬件采样比例(仅 IBUS_HW 类型需要, 供 scale 字段引用)
 * 换算参数已挪入配置表 type/scale/offset 字段, 不再用 PM_IBUS_SOURCE 宏分散判断 */
#if defined(USE_DEV_POWER_MONITOR)
#ifndef PM_CH_MAX
#error "PM_CH_MAX (规则组通道数) must be defined in dev_config_board.h"
#endif
#if PM_CH_MAX < 1
#error "PM_CH_MAX must be >= 1"
#endif
#ifndef PM_VBUS_RATIO
#error "PM_VBUS_RATIO (母线电压分压比) must be defined in dev_config_board.h"
#endif
#endif

#if defined(USE_DEV_HALF_BRIDGE)
#ifndef HALF_BRIDGE_PWM_PERIOD
#error "HALF_BRIDGE_PWM_PERIOD (TIM1 ARR) must be defined in dev_config_board.h"
#endif
#ifndef HALF_BRIDGE_ADC_TRIG_CCR
#error "HALF_BRIDGE_ADC_TRIG_CCR (ADC触发比较值) must be defined in dev_config_board.h"
#endif
/* CCR 必须 < ARR, 否则 ADC 永不触发 (中心对齐模式波峰采样) */
#if HALF_BRIDGE_ADC_TRIG_CCR >= HALF_BRIDGE_PWM_PERIOD
#error "HALF_BRIDGE_ADC_TRIG_CCR must be < HALF_BRIDGE_PWM_PERIOD (ADC never triggers!)"
#endif
#endif

#if defined(USE_DEV_PHASE_CURRENT)
#ifndef PHASE_CURRENT_GAIN
#error "PHASE_CURRENT_GAIN (相电流增益) must be defined in dev_config_board.h"
#endif
#ifndef PHASE_CURRENT_SHUNT
#error "PHASE_CURRENT_SHUNT (采样电阻) must be defined in dev_config_board.h"
#endif
#endif

#if defined(USE_DEV_FLASH)
#ifndef MOTORINFO_FLASH_START_ADDR
#error "MOTORINFO_FLASH_START_ADDR must be defined in dev_config_board.h"
#endif
#ifndef MOTORINFO_FLASH_TOTAL_SIZE
#error "MOTORINFO_FLASH_TOTAL_SIZE must be defined in dev_config_board.h"
#endif
#ifndef MOTORINFO_FLASH_PAGE_SIZE
#error "MOTORINFO_FLASH_PAGE_SIZE must be defined in dev_config_board.h"
#endif
/* Flash 地址必须 4 字节对齐 */
#if (MOTORINFO_FLASH_START_ADDR & 0x3U) != 0
#error "MOTORINFO_FLASH_START_ADDR must be 4-byte aligned"
#endif
#endif
#endif /* __DEV_CONFIG_H__ */
