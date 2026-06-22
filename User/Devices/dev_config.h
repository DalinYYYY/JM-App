/**
 * @file device_config.h
 * @brief 设备层统一硬件配置：使能开关 + 换算常量
 * @note  本文件只放“是否启用”和“数值常量”两类宏。
 *        真正的引脚/总线/通道映射集中写在 device_config.c 的配置表里 (C99 指定初始化器)，新增/更换硬件改那张表即可。
 */
#ifndef __DEVICE_CONFIG_H__
#define __DEVICE_CONFIG_H__

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define DEV_EOK 0
#define DEV_ERROR 1
#define DEV_ENABLE 1
#define DEV_DISABLE 0

/* ===================================================================== */
/*                          1. 设备使能开关                              */
/* ===================================================================== */
//***<<< Use Configuration Wizard in Context Menu >>>***

// <h> DEVICE ENABLE
// <i> 勾选启用对应设备驱动模块

// <c1> LED
// <i> 板载状态指示灯
// #define USE_DEV_LED
// </c>

// <c1> RGB LED
// <i> RGB 三色指示灯
//#define USE_DEV_RGB_LED
// </c>

// <c1> EEPROM
// <i> 板载 EEPROM 存储
//#define USE_DEV_EEPROM
// </c>

// <c1> HALF BRIDGE
// <i> 三相半桥 PWM 驱动
#define USE_DEV_HALF_BRIDGE
// </c>

// <c1> MT6701 ENCODER
// <i> MT6701 磁编码器 (14bit)
#define USE_DEV_MT6701
// </c>

// <c1> MT6835 ENCODER
// <i> MT6835 磁编码器 (21bit)
#define USE_DEV_MT6835
// </c>

// <c1> POWER MONITOR
// <i> 电源监控 (母线电压/电流/温度, ADC规则组)
#define USE_DEV_POWER_MONITOR
// </c>

// <c1> PHASE CURRENT
// <i> 三相电流采样 (ADC注入组)
#define USE_DEV_PHASE_CURRENT
// </c>

// <c1> DWT COUNTER
// <i> DWT 周期计数器 (微秒计时)
#define USE_DEV_DWT_COUNTER
// </c>

// <c1> COMMUN VESC
// <i> VESC Tool 串口通信 (USART+DMA空闲中断, 伪装成VESC从机)
// #define USE_DEV_COMMUN_VESC
// </c>

// <c1> COMMUN UART
// <i> 关节电机串口通信 (USART+DMA空闲中断, joint_proto 协议)
#define USE_DEV_COMMUN_UART
// </c>
// </h>
/* ===================================================================== */
/*                                                                       */
/* ===================================================================== */
#if defined(USE_DEV_MT6701)
// <o> MT6701 SPI MODE
//   <0=> Hardware SPI
//   <1=> Software SPI
// <i> MT6701 使用硬件或软件 SPI
#define DEV_MT6701_SPI_DRIVER 0
#endif

#if defined(USE_DEV_MT6835)
// <o> MT6835 SPI MODE
//   <0=> Hardware SPI
//   <1=> Software SPI
// <i> MT6835 使用硬件或软件 SPI
#define DEV_MT6835_SPI_DRIVER 0
#endif
/* ===================================================================== */
/*                          2. 换算 / 行为常量                           */
/* ===================================================================== */
/* 引脚/总线/通道映射不在此处，统一见 device_config.c 配置表。 */

#if defined(USE_DEV_POWER_MONITOR)
#define PM_VREF (3.3f)			/* ADC 参考电压, V */
#define PM_RESOLUTION (4096.0f) /* ADC 满量程 (12bit) */
#define PM_VBUS_RATIO (24.0f)	/* 母线电压分压比 */
#define PM_IBUS_RATIO (10.0f)	/* 母线电流采样换算系数 (A/V) */
#define PM_TEMP_RATIO (10.0f)	/* 温度通道换算系数 */
#endif

#if defined(USE_DEV_HALF_BRIDGE)
#define HALF_BRIDGE_ADC_TRIG_CCR (8380u) // <i> 触发ADC注入组的比较值(略大于ARR, 借更新事件触发采样)
#endif

#if defined(USE_DEV_PHASE_CURRENT)
#define PHASE_CURRENT_GAIN (50.0f)		   /* 放大器增益*/
#define PHASE_CURRENT_SHUNT (0.01f)		   /* 采样电阻, 欧姆 */
#define PHASE_CURRENT_VREF (3.3f)		   /* ADC 参考电压, V */
#define PHASE_CURRENT_RESOLUTION (4096.0f) /* ADC 满量程 (12bit) */
#define PHASE_CURRENT_LPF_ALPHA (0.9f)	   /* 一阶低通滤波系数, 0~1 */
#define PHASE_CURRENT_ZERO_ADC (2024u)	   /* 电流零位时 ADC 采样偏移 */
#endif

#if defined(USE_DEV_COMMUN_VESC)
#define DEV_VESC_RX_BUF_SIZE (256u) /* VESC 接收缓冲(空闲中断单帧上限, 含组帧余量) */
#endif

#if defined(USE_DEV_COMMUN_UART)
#define DEV_JM_UART_RX_BUF_SIZE (300u) /* 关节电机串口接收缓冲(空闲中断单帧上限) */
#endif

//***<<< end of configuration section >>>***

#endif /* __DEVICE_CONFIG_H__ */
