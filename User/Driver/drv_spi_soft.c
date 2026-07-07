/**
 * @file        drv_spi_soft.c
 * @brief       软件模拟SPI驱动实现，通过GPIO回调实现，与具体MCU/HAL解耦
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
 */
#include "drv_spi_soft.h"

#ifdef USE_SOFT_SPI_DRIVER

/* 发送一个字节(MSB先行，SCL空闲低，上升沿采样) */
static void spi_send_byte(struct spi_soft_drv *pobj, uint8_t byte)
{
	uint8_t i;

	for (i = 0; i < 8; i++)
	{
		pobj->scl(SPI_LOW);
		pobj->mosi((byte & 0x80) ? SPI_HIGH : SPI_LOW);
		pobj->scl(SPI_HIGH);
		byte <<= 1;
	}
}

/* 接收一个字节(MSB先行，上升沿读MISO) */
static uint8_t spi_recv_byte(struct spi_soft_drv *pobj)
{
	uint8_t i;
	uint8_t recv = 0;

	if (pobj->miso_read == NULL)
		return 0;

	for (i = 0; i < 8; i++)
	{
		recv <<= 1;
		pobj->scl(SPI_LOW);
		pobj->scl(SPI_HIGH);
		if (pobj->miso_read())
		{
			recv |= 0x01;
		}
	}
	return recv;
}

/* 全双工收发一个字节 */
static uint8_t spi_transfer_byte(struct spi_soft_drv *pobj, uint8_t byte)
{
	uint8_t i;
	uint8_t recv = 0;

	for (i = 0; i < 8; i++)
	{
		recv <<= 1;
		pobj->scl(SPI_LOW);
		pobj->mosi((byte & 0x80) ? SPI_HIGH : SPI_LOW);
		pobj->scl(SPI_HIGH);
		if (pobj->miso_read && pobj->miso_read())
		{
			recv |= 0x01;
		}
		byte <<= 1;
	}
	return recv;
}

/**
 * @brief       初始化软件SPI设备(注入GPIO操作回调)
 */
void drv_soft_spi_init(spi_soft_drv_t *spi,
                       void (*mosi)(spi_state_t level),
                       void (*scl)(spi_state_t level),
                       uint8_t (*miso_read)(void))
{
	spi->mosi = mosi;
	spi->scl = scl;
	spi->miso_read = miso_read;
	spi->send_byte = spi_send_byte;
	spi->recv_byte = spi_recv_byte;
	spi->transfer_byte = spi_transfer_byte;
}

#endif /* USE_SOFT_SPI_DRIVER */
