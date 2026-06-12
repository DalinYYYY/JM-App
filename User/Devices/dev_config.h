/**
 * @file .h
 * @brief   设备配置文件
 *
 * @author LinHui(61900302@qq.com)
 * @version 1.0
 * @date 2023-05-10
 *
 * @see
 * @copyright Copyright (c) 2024 RobotDance Technology.co, Ltd
 *
 * @details 修改日志:
 * Date           Version     Author      Description
 * 2023-06-10     1.0         LinHui       New File
 * 2023-06-02     1.1         LinHui       增加可视化配置
 */

#ifndef _DEV_CONFIG_H_
#define _DEV_CONFIG_H_

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define DEV_EOK     0
#define DEV_ERROR   1

#define DEV_ENABLE  1
#define DEV_DISABLE 0

// -------------------定义可视化配置---------------------
//***<<< Use Configuration Wizard in Context Menu >>>***

//  <h> VERSION INFO
// <s>Device Config Version
//  <i>Device configuration file version
#define DEV_CONFIG_VERSION "0.1.0"
//  </h>

// <h>DEVICE ENABLE
// <c1>
// ENABLE DEVICE ---> LED
// #define USE_DEV_LED
// </c>

// <h>DEVICE ENABLE
// <c1>
// ENABLE DEVICE ---> RGB LED
#define USE_DEV_RGB_LED
// </c>

// <h>DEVICE ENABLE
// <c1>
// ENABLE DEVICE ---> RGB LED
// #define USE_DEV_INDUSTRIAL_LIGHT
// </c>

// <c1>
// ENABLE DEVICE ---> TEST_IO
#define USE_DEV_TESTIO
// </c>

// <c1>
// ENABLE DEVICE ---> DEV_USB
#define USE_DEV_USB
// </c>

// <c1>
// ENABLE DEVICE ---> DRV8313
#define USE_DEV_DRV8313
// </c>

// <c1>
// ENABLE DEVICE ---> HALF_BRIDGE
#define USE_DEV_HALF_BRIDGE
// </c>

// <c1>
// ENABLE DEVICE ---> TB6612
#define USE_DEV_TB6612
// </c>

// <c1>
// ENABLE DEVICE ---> ENCODER
#define USE_DEV_ENCODER
// </c>

// <c1>
// ENABLE DEVICE ---> SERVO
#define USE_DEV_SERVO
// </c>EEPROM

// <c1>
// ENABLE DEVICE ---> EEPROM
#define USE_DEV_EEPROM
// </c>


// <c1>
// ENABLE DEVICE ---> AS5600
#define USE_DEV_AS5600
// </c>

// <h>USE_DEV_MT6701
// <c1>
// ENABLE DEVICE ---> MT6701
#define USE_DEV_MT6701
// </c>

//  <o> MT6701_SPI_Driver_Select
//   <i> Default:USE_HARDWARE_SPI
//   <0=> USE_HARDWARE_SPI
//   <1=> USE_SOFTWARE_SPI
#define DEV_MT6701_SPI_DRIVER 0
//  </h>

// <h>USE_DEV_MT6835
// <c1>
// ENABLE DEVICE ---> MT6835
#define USE_DEV_MT6835
// </c>

//  <o> MT6835_SPI_Driver_Select
//   <i> Default:USE_HARDWARE_SPI
//   <0=> USE_HARDWARE_SPI
//   <1=> USE_SOFTWARE_SPI
#define DEV_MT6835_SPI_DRIVER 0
//  </h>


// <h>USE_DEV_MPU6050
// <c1>
// ENABLE DEVICE ---> MPU6050
//#define USE_DEV_MPU6050
// </c>

//  <o> MPU6050_I2C_Driver_Select
//   <i> Default:USE_HARDWARE_I2C
//   <0=> USE_HARDWARE_I2C
//   <1=> USE_SOFTWARE_I2C
#define DEV_MPU6050_I2C_DRIVER 1
//  </h>

// <h>USE_DEV_LCD
// <c1>
// ENABLE DEVICE ---> LCD
#define USE_DEV_LCD
// </c>

//  <o> LCD_SPI_Driver_Select
//   <i> Default:USE_HARDWARE_SPI
//   <0=> USE_HARDWARE_SPI
//   <1=> USE_SOFTWARE_SPI
#define DEV_LCD_SPI_DRIVER 0
//  </h>

//***<<< end of configuration section >>>***

#endif /* _DEV_CONFIG_H_ */
