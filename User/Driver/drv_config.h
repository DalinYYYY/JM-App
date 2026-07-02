

#ifndef _DRV_CONFIG_H_
#define _DRV_CONFIG_H_

#include <stdint.h>

#define DRV_EOK (0)	  /**< There is no error */
#define DRV_ERROR (1) /**< A generic error happens */

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
// ENABLE DRIVER ---> FLASH_G4
#define USE_FLASH_G4_DRIVER
// </c>

// <c1>
// ENABLE DRIVER ---> SPI
#define USE_SPI_DRIVER
// </c>

// <c1>
// ENABLE DRIVER ---> I2C
//#define USE_I2C_DRIVER
// </c>

// <c1>
// ENABLE DRIVER ---> SOFT_I2C
//#define USE_I2C_DRIVER
// </c>

// <c1>
// ENABLE DRIVER ---> CAN
//#define USE_CAN_DRIVER
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

// <o.0..3>FLASH MCU
//  <i> Default:STM32F103
//  <1=>STM32F103
//  <2=>STM32F429
//  <3=>STM32F405
//  <4=>STM32G431
//  <5=>STM32G491
//  <6=>STM32G474
#define FLASH_MCU 0x06

#if (FLASH_MCU == 0x01)
#define STM32F103
#elif (FLASH_MCU == 0x02)
#define STM32F429
#elif (FLASH_MCU == 0x03)
#define STM32F405
#elif (FLASH_MCU == 0x04)
#define STM32G431
#elif (FLASH_MCU == 0x05)
#define STM32G491
#elif (FLASH_MCU == 0x06)
#define STM32G474
#endif

//***<<< end of configuration section >>>***
#endif /* _DRV_CONFIG_H_ */
