/**
 * @file dev_config.h
 * @brief Shared device configuration interface.
 *
 * Board-specific enable macros and pin/channel tables are selected through
 * `User/Config/board_select.h`.
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
/*                          Device constants                             */
/* ===================================================================== */

#if defined(USE_DEV_POWER_MONITOR)
#define PM_VREF          (3.3f)
#define PM_RESOLUTION    (4096.0f)
#define PM_VBUS_RATIO    (11.0f) /* 板级分压网络 11:1 */
#define PM_IBUS_RATIO    (2.0f)  /* 1/(gain*shunt) = 1/(50*0.01) */
#define PM_IBUS_OFFSET_V (1.65f) /* INA199B1 REF=VREF/2, 零电流偏置电压 */
#define PM_TEMP_RATIO    (10.0f)
/* 母线电流来源选择:
 *   0 = 硬件 ADC 采样 (INA199B1, 旧板子)
 *   1 = 三相电流 + SVPWM 占空比合成 (新板子无母线电流检测)
 * 合成公式: Ibus = da*Ia + db*Ib + dc*Ic (功率守恒推导) */
#ifndef PM_IBUS_SOURCE
#define PM_IBUS_SOURCE (0)
#endif
/* 合成母线电流低通滤波系数 (SYNTH 源专用)
 * 抑制小电流时三相采样噪声叠加导致的波动, alpha=0.05 时 10kHz 采样下时间常数约 2ms, 遥测足够, 响应不迟滞 */
#ifndef PM_IBUS_LPF_ALPHA
#define PM_IBUS_LPF_ALPHA (0.05f)
#endif
#endif

#if defined(USE_DEV_HALF_BRIDGE)
/* CC4 比较匹配触发 ADC 注入组: 中心对齐下采样点落在波峰后约 (ARR-CCR)*5.88ns
 * 8480 距波峰(ARR=8500)约 118ns, 贴近波峰(纹波中点/离开关边沿最远); 留余量避免峰值临界漏触发 */
#define HALF_BRIDGE_ADC_TRIG_CCR (8480u)
#endif

#if defined(USE_DEV_PHASE_CURRENT)
#define PHASE_CURRENT_GAIN       (50.0f)
#define PHASE_CURRENT_SHUNT      (0.01f)
#define PHASE_CURRENT_VREF       (3.3f)
#define PHASE_CURRENT_RESOLUTION (4096.0f)
#define PHASE_CURRENT_LPF_ALPHA  (0.9f)
/* INA199B1 REF 标称 1.65V (VREF/2), 对应 ADC = 1.65/3.3 * 4096 = 2048
 * 仅作为校准前的兜底默认值; 启动时 cur_loop_calibrate_offset 会用实测均值覆盖 */
#define PHASE_CURRENT_ZERO_ADC (2048u)
#endif

#if defined(USE_DEV_COMMUN_VESC)
#define DEV_VESC_RX_BUF_SIZE (256u)
#endif

#if defined(USE_DEV_COMMUN_UART)
#define DEV_JM_UART_RX_BUF_SIZE (256u)
#endif

#endif /* __DEV_CONFIG_H__ */
