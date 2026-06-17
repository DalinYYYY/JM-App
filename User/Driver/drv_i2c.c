/**
 * @file        drv_i2c.c
 * @brief       I2C驱动实现，封装HAL的I2C内存读写/设备就绪检测
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
 */
#include "drv_i2c.h"

#ifdef USE_I2C_DRIVER
#include "i2c.h"

__weak I2C_HandleTypeDef hi2c1;
__weak I2C_HandleTypeDef hi2c2;
__weak I2C_HandleTypeDef hi2c3;

/* I2C句柄查找表：以i2cNumber_e为索引，O(1)定位HAL句柄 */
static I2C_HandleTypeDef *const s_i2c_map[DRV_I2C_NUMBER_MAX] = {
	[DRV_I2C1] = &hi2c1,
	[DRV_I2C2] = &hi2c2,
	[DRV_I2C3] = &hi2c3,
};

static inline I2C_HandleTypeDef *get_i2c_handle(i2cDrv_t drv)
{
	if (drv.hi2c >= DRV_I2C_NUMBER_MAX)
		return NULL;

	return s_i2c_map[drv.hi2c];
}

/* 内存地址宽度转HAL宏，非法时返回8bit */
static inline uint16_t get_i2c_memsize(i2cDrv_t drv)
{
	return (drv.mem_size == DRV_I2C_MEMSIZE_16BIT) ? I2C_MEMADD_SIZE_16BIT : I2C_MEMADD_SIZE_8BIT;
}

/**
 * @brief       向I2C从机内存地址写数据(阻塞)
 */
int drv_i2c_send(i2cDrv_t drv, uint8_t *data, uint16_t len)
{
	I2C_HandleTypeDef *hi2c = get_i2c_handle(drv);
	if (hi2c == NULL)
	{
		return DRV_ERROR;
	}

	if (HAL_I2C_Mem_Write(hi2c, drv.dev_addr, drv.mem_addr, get_i2c_memsize(drv), data, len, drv.timeout) != HAL_OK)
		return DRV_ERROR;

	return DRV_EOK;
}

/**
 * @brief       从I2C从机内存地址读数据(阻塞)
 */
int drv_i2c_recv(i2cDrv_t drv, uint8_t *data, uint16_t len)
{
	I2C_HandleTypeDef *hi2c = get_i2c_handle(drv);
	if (hi2c == NULL)
	{
		return DRV_ERROR;
	}

	if (HAL_I2C_Mem_Read(hi2c, drv.dev_addr, drv.mem_addr, get_i2c_memsize(drv), data, len, drv.timeout) != HAL_OK)
		return DRV_ERROR;

	return DRV_EOK;
}

/**
 * @brief       向I2C从机直接写数据(主机发送)
 */
int drv_i2c_master_send(i2cDrv_t drv, uint8_t *data, uint16_t len)
{
	I2C_HandleTypeDef *hi2c = get_i2c_handle(drv);
	if (hi2c == NULL)
	{
		return DRV_ERROR;
	}

	if (HAL_I2C_Master_Transmit(hi2c, drv.dev_addr, data, len, drv.timeout) != HAL_OK)
		return DRV_ERROR;

	return DRV_EOK;
}

/**
 * @brief       从I2C从机直接读数据(主机接收)
 */
int drv_i2c_master_recv(i2cDrv_t drv, uint8_t *data, uint16_t len)
{
	I2C_HandleTypeDef *hi2c = get_i2c_handle(drv);
	if (hi2c == NULL)
	{
		return DRV_ERROR;
	}

	if (HAL_I2C_Master_Receive(hi2c, drv.dev_addr, data, len, drv.timeout) != HAL_OK)
		return DRV_ERROR;

	return DRV_EOK;
}

/**
 * @brief       检测I2C从机是否就绪(ACK应答)
 */
int drv_i2c_ready(i2cDrv_t drv, uint8_t trials)
{
	I2C_HandleTypeDef *hi2c = get_i2c_handle(drv);
	if (hi2c == NULL)
	{
		return DRV_ERROR;
	}

	return (HAL_I2C_IsDeviceReady(hi2c, drv.dev_addr, trials, drv.timeout) == HAL_OK) ? DRV_EOK : DRV_ERROR;
}

#endif /* USE_I2C_DRIVER */
