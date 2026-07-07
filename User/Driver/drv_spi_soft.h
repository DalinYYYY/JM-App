/**
 * @file        drv_spi_soft.h
 * @brief       软件模拟SPI驱动接口，通过GPIO回调实现，与具体MCU/HAL解耦
 *
 * @author      Dalin (dalinyy@163.com)
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
 * @note        MOSI/MISO/SCL的GPIO操作由使用者通过回调注入，不依赖任何HAL类型
 * @note        时序固定为MSB先行、SCL空闲低(CPOL=0/CPHA=0)
 */
#ifndef __DRV_SPI_SOFT_H__
#define __DRV_SPI_SOFT_H__

#include "drv_config.h"
#include <stdint.h>

#ifdef USE_SOFT_SPI_DRIVER

/**
 * @brief 引脚电平
 * @param  SPI_LOW               : 低电平
 * @param  SPI_HIGH              : 高电平
 */
typedef enum
{
	SPI_LOW = 0,
	SPI_HIGH
} spi_state_t;

/**
 * @brief 数据线方向
 * @param  _SPI_IN               : 输入
 * @param  _SPI_OUT              : 输出
 * @param  _SPI_DIR_MAX          : 方向数量边界
 */
typedef enum
{
	_SPI_IN = 0u,
	_SPI_OUT,
	_SPI_DIR_MAX
} spi_sda_dir_t;

/**
 * @brief 软件SPI设备描述
 * @param  miso_read             : 回调-读MISO电平(接收用，可为NULL)
 * @param  mosi                  : 回调-设置MOSI电平
 * @param  scl                   : 回调-设置SCL电平
 * @param  send_byte             : 接口-发送一个字节
 * @param  recv_byte             : 接口-接收一个字节
 * @param  transfer_byte         : 接口-收发一个字节
 */
typedef struct spi_soft_drv
{
	uint8_t (*miso_read)(void);
	void (*mosi)(spi_state_t level);
	void (*scl)(spi_state_t level);

	void (*send_byte)(struct spi_soft_drv *pobj, uint8_t byte);
	uint8_t (*recv_byte)(struct spi_soft_drv *pobj);
	uint8_t (*transfer_byte)(struct spi_soft_drv *pobj, uint8_t byte);
} spi_soft_drv_t;

/**
 * @brief       初始化软件SPI设备(注入GPIO操作回调)
 * @param        spi               : 软件SPI设备对象
 * @param        mosi              : 设置MOSI电平回调
 * @param        scl               : 设置SCL电平回调
 * @param        miso_read         : 读MISO电平回调(仅发送时可传NULL)
 */
void drv_soft_spi_init(spi_soft_drv_t *spi,
                       void (*mosi)(spi_state_t level),
                       void (*scl)(spi_state_t level),
                       uint8_t (*miso_read)(void));

#endif /* USE_SOFT_SPI_DRIVER */
#endif // __DRV_SPI_SOFT_H__
