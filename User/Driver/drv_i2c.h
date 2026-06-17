/**
 * @file        drv_i2c.h
 * @brief       I2C驱动接口，封装HAL的I2C内存读写/设备就绪检测
 *
 * @author      Dalin (dalin@robot.com)
 * @version     1.0
 * @date        2026-06-16
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @par 修改日志:
 * | 日期       | 版本 | 作者   | 修改内容   |
 * |------------|------|--------|------------|
 * | 2026-06-16 | 1.0  | Dalin  | 初始创建   |
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 * @note        I2C初始化由CubeMX的MX_I2Cx_Init完成，此层仅封装运行期读写
 * @note        对外接口不暴露HAL类型，HAL句柄仅在drv_i2c.c内部使用
 */
#ifndef _DRV_I2_H_
#define _DRV_I2_H_

#include "drv_config.h"
#include "drv_gpio.h"

#ifdef USE_I2C_DRIVER

/**
 * @brief I2C外设编号
 * @param  DRV_I2C_INIT          : 无效占位(0)
 * @param  DRV_I2C1              : I2C1
 * @param  DRV_I2C2              : I2C2
 * @param  DRV_I2C3              : I2C3
 * @param  DRV_I2C_NUMBER_MAX    : 外设数量边界
 */
typedef enum I2CNUMBER_
{
	DRV_I2C_INIT = 0,
	DRV_I2C1,
	DRV_I2C2,
	DRV_I2C3,

	DRV_I2C_NUMBER_MAX
} i2cNumber_e;

/**
 * @brief I2C内存地址宽度
 * @param  DRV_I2C_MEMSIZE_8BIT  : 8位内存地址
 * @param  DRV_I2C_MEMSIZE_16BIT : 16位内存地址
 */
typedef enum I2CMEMSIZE_
{
	DRV_I2C_MEMSIZE_8BIT = 0,
	DRV_I2C_MEMSIZE_16BIT,
} i2cMemSize_e;

/**
 * @brief I2C设备描述
 * @param  hi2c                  : I2C外设编号
 * @param  dev_addr              : 从机设备地址(已含读写位的8bit地址)
 * @param  mem_addr              : 内存/寄存器地址
 * @param  mem_size              : 内存地址宽度(i2cMemSize_e)
 * @param  timeout               : 超时时间 (单位: ms)
 */
typedef struct DRV_I2C_
{
	i2cNumber_e hi2c;
	uint8_t dev_addr;
	uint16_t mem_addr;
	uint8_t mem_size;
	uint32_t timeout;
} i2cDrv_t;

/********************************** drv i2c api **********************************/

/**
 * @brief       向I2C从机内存地址写数据(阻塞)
 * @param        drv               : I2C设备描述
 * @param        data              : 发送数据缓冲区
 * @param        len               : 数据长度
 * @return       : DRV_EOK成功，DRV_ERROR失败
 */
int drv_i2c_send(i2cDrv_t drv, uint8_t *data, uint16_t len);

/**
 * @brief       从I2C从机内存地址读数据(阻塞)
 * @param        drv               : I2C设备描述
 * @param        data              : 接收数据缓冲区
 * @param        len               : 数据长度
 * @return       : DRV_EOK成功，DRV_ERROR失败
 */
int drv_i2c_recv(i2cDrv_t drv, uint8_t *data, uint16_t len);

/**
 * @brief       向I2C从机直接写数据(不带内存地址，主机发送)
 * @param        drv               : I2C设备描述(用hi2c/dev_addr/timeout)
 * @param        data              : 发送数据缓冲区
 * @param        len               : 数据长度
 * @return       : DRV_EOK成功，DRV_ERROR失败
 */
int drv_i2c_master_send(i2cDrv_t drv, uint8_t *data, uint16_t len);

/**
 * @brief       从I2C从机直接读数据(不带内存地址，主机接收)
 * @param        drv               : I2C设备描述(用hi2c/dev_addr/timeout)
 * @param        data              : 接收数据缓冲区
 * @param        len               : 数据长度
 * @return       : DRV_EOK成功，DRV_ERROR失败
 */
int drv_i2c_master_recv(i2cDrv_t drv, uint8_t *data, uint16_t len);

/**
 * @brief       检测I2C从机是否就绪(ACK应答)
 * @param        drv               : I2C设备描述
 * @param        trials            : 每次检测的尝试次数
 * @return       : DRV_EOK就绪，DRV_ERROR未就绪
 */
int drv_i2c_ready(i2cDrv_t drv, uint8_t trials);

#endif /* USE_I2C_DRIVER */
#endif /* _DRV_I2_H_ */
