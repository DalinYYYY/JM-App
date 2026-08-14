

#ifndef _DRV_CONFIG_H_
#define _DRV_CONFIG_H_

#include <stdint.h>
#include "mcu_compat.h"

#define DRV_EOK (0)	  /**< 无错误 */
#define DRV_ERROR (1) /**< 发生通用错误 */

// -------------------定义可视化配置---------------------
//***<<< Use Configuration Wizard in Context Menu >>>***

//  <h> VERSION INFO
// <s>Driver Config Version
//  <i>Driver configuration file version
#define DRV_CONFIG_VERSION "0.0.1"
//  </h>

// <h>MCU ENABLE DRIVER
// <c1>
// ENABLE DRIVER ---> USART
#define USE_USART_DRIVER
// </c>

// <c1>
// ENABLE DRIVER ---> FLASH
//#define USE_FLASH_DRIVER
// </c>

// <c1>
// ENABLE DRIVER ---> FLASH_G4 (由 FLASH_MCU 自动派生, 勿手动开启)
//#define USE_FLASH_G4_DRIVER
// </c>

// <c1>
// ENABLE DRIVER ---> FLASH_F4 (由 FLASH_MCU 自动派生, 勿手动开启)
//#define USE_FLASH_F4_DRIVER
// </c>

// <c1>
// ENABLE DRIVER ---> SPI
#define USE_SPI_DRIVER
// </c>

// <c1>
// ENABLE DRIVER ---> I2C
#define USE_I2C_DRIVER
// </c>

// <c1>
// ENABLE DRIVER ---> SOFT_I2C
//#define USE_I2C_DRIVER
// </c>

// <c1>
// ENABLE DRIVER ---> CAN
#define USE_CAN_DRIVER
// </c>

// <c1>
// ENABLE DRIVER ---> PWM
#define USE_TIM_PWM_DRIVER
// </c>

// <c1>
// ENABLE DRIVER ---> ENCODER
//#define USE_TIM_ENCODER_DRIVER
// </c>

// <c1>
// ENABLE DRIVER ---> ADC
#define USE_ADC_DRIVER
// </c>

// <c1>
// ENABLE DRIVER ---> USB
//#define USE_USB_DRIVER
// </c>

// <c1>
// ENABLE DRIVER ---> GPIO
#define USE_GPIO_DRIVER
// </c>
// </h>

// <o.0..3>VOFA PORT
//  <i> Default:USERT1
//  <1=>USERT1
//  <2=>USERT2
//  <3=>USERT3
//  <4=>USERT4
//  <5=>USERT5
//  <6=>USERT6
#define VOFA_PORT 0x02

// <h>FLASH MCU (auto-derived from HAL macro, do NOT edit manually)
//  <i> 由 Keil 预定义的 STM32F405xx / STM32G474xx 等自动派生 Flash 驱动选择
//  <i> 切换工程时无需修改此处, 由 board_select.h -> dev_config_board.h 链路保证
//  <i> Flash 驱动按"擦除/编程类型"分组:
//  <i>   - F4: 扇区擦除+word 编程         → drv_flash_f4.c
//  <i>   - G4/L4: 页擦除+doubleword 编程  → drv_flash_g4.c (L4 接口与 G4 一致, 复用)
//  <i>   - F1/F3: 页擦除+halfword 编程    → drv_flash_f1.c (预留, 需新建)
//  <i>   - H7: 扇区擦除+256bit 编程       → drv_flash_h7.c (预留, 需新建)

/* mcu_compat.h 已在文件顶部 include, 此处直接用 JM_MCU_* 派生 */
#if defined(JM_MCU_F4)
#ifndef USE_FLASH_F4_DRIVER
#define USE_FLASH_F4_DRIVER
#endif
#elif defined(JM_MCU_G4) || defined(JM_MCU_L4)
/* L4 的 Flash 接口与 G4 一致(页擦除 + 64-bit doubleword 编程), 直接复用 drv_flash_g4.c
 * drv_flash_g4.h:35-42 的 Flash 几何参数来自 HAL 运行期值, 自动适配各容量后缀 */
#ifndef USE_FLASH_G4_DRIVER
#define USE_FLASH_G4_DRIVER
#endif
#elif defined(JM_MCU_F1) || defined(JM_MCU_F3)
/* F1/F3 Flash: 页擦除 + 16-bit halfword 编程, 接口与 G4 不同, 需新建 drv_flash_f1.c */
#ifndef USE_FLASH_F1_DRIVER
#define USE_FLASH_F1_DRIVER
#endif
#elif defined(JM_MCU_H7)
/* H7 Flash: 扇区擦除 + 256-bit 编程, 双 Bank, 需新建 drv_flash_h7.c */
#ifndef USE_FLASH_H7_DRIVER
#define USE_FLASH_H7_DRIVER
#endif
#endif
// </h>

//***<<< end of configuration section >>>***
#endif /* _DRV_CONFIG_H_ */
