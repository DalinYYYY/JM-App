/**
 * @file        drv_spi.c
 * @brief       SPI驱动实现，封装HAL的SPI阻塞/中断/DMA收发及片选控制
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
#include "drv_spi.h"
#include "drv_gpio.h"

#ifdef USE_SPI_DRIVER
#include "spi.h"

__weak SPI_HandleTypeDef hspi1;
__weak SPI_HandleTypeDef hspi2;
__weak SPI_HandleTypeDef hspi3;

/* SPI句柄查找表：以spiNumber_e为索引，O(1)定位HAL句柄 */
static SPI_HandleTypeDef *const s_spi_map[DRV_SPI_NUMBER_MAX] = {
	[DRV_SPI1] = &hspi1,
	[DRV_SPI2] = &hspi2,
	[DRV_SPI3] = &hspi3,
};

static inline SPI_HandleTypeDef *get_spi_handle(spiNumber_e spi)
{
	if (spi >= DRV_SPI_NUMBER_MAX)
		return NULL;

	return s_spi_map[spi];
}

/* cs.gpiox为DRV_GPIO_INIT时视为未配置片选 */
static inline uint8_t spi_cs_valid(spiDrv_t drv)
{
	return (drv.cs.gpiox != DRV_GPIO_INIT) ? 1U : 0U;
}

void drv_spi_cs_select(spiDrv_t drv)
{
	if (spi_cs_valid(drv))
		drv_gpio_write(drv.cs, DRV_PIN_LOW);
}

void drv_spi_cs_release(spiDrv_t drv)
{
	if (spi_cs_valid(drv))
		drv_gpio_write(drv.cs, DRV_PIN_HIGH);
}

/**
 * @brief       SPI发送数据(阻塞，自动管理CS)
 */
int drv_spi_send(spiDrv_t drv, uint8_t *data, uint16_t len, uint32_t timeout)
{
	SPI_HandleTypeDef *hspi;
	HAL_StatusTypeDef st;

	hspi = get_spi_handle(drv.hspi);
	if (hspi == NULL)
	{
		return DRV_ERROR;
	}

	drv_spi_cs_select(drv);
	st = HAL_SPI_Transmit(hspi, data, len, timeout);
	drv_spi_cs_release(drv);

	return (st == HAL_OK) ? DRV_EOK : DRV_ERROR;
}

/**
 * @brief       SPI接收数据(阻塞，自动管理CS)
 */
int drv_spi_read(spiDrv_t drv, uint8_t *data, uint16_t len, uint32_t timeout)
{
	SPI_HandleTypeDef *hspi;
	HAL_StatusTypeDef st;

	hspi = get_spi_handle(drv.hspi);
	if (hspi == NULL)
	{
		return DRV_ERROR;
	}

	drv_spi_cs_select(drv);
	st = HAL_SPI_Receive(hspi, data, len, timeout);
	drv_spi_cs_release(drv);

	return (st == HAL_OK) ? DRV_EOK : DRV_ERROR;
}

/**
 * @brief       SPI全双工收发(阻塞，自动管理CS)
 */
int drv_spi_transfer(spiDrv_t drv, uint8_t *tx_data, uint8_t *rx_data, uint16_t len, uint32_t timeout)
{
	SPI_HandleTypeDef *hspi;
	HAL_StatusTypeDef st;

	hspi = get_spi_handle(drv.hspi);
	if (hspi == NULL)
	{
		return DRV_ERROR;
	}

	drv_spi_cs_select(drv);
	st = HAL_SPI_TransmitReceive(hspi, tx_data, rx_data, len, timeout);
	drv_spi_cs_release(drv);

	return (st == HAL_OK) ? DRV_EOK : DRV_ERROR;
}

/**
 * @brief       SPI中断方式发送(非阻塞)
 */
int drv_spi_send_it(spiDrv_t drv, uint8_t *data, uint16_t len)
{
	SPI_HandleTypeDef *hspi;

	hspi = get_spi_handle(drv.hspi);
	if (hspi == NULL)
	{
		return DRV_ERROR;
	}

	drv_spi_cs_select(drv);
	/* 启动失败需立即释放CS；成功时由用户在完成回调中释放 */
	if (HAL_SPI_Transmit_IT(hspi, data, len) != HAL_OK)
	{
		drv_spi_cs_release(drv);
		return DRV_ERROR;
	}

	return DRV_EOK;
}

/**
 * @brief       SPI中断方式接收(非阻塞)
 */
int drv_spi_read_it(spiDrv_t drv, uint8_t *data, uint16_t len)
{
	SPI_HandleTypeDef *hspi;

	hspi = get_spi_handle(drv.hspi);
	if (hspi == NULL)
	{
		return DRV_ERROR;
	}

	drv_spi_cs_select(drv);
	if (HAL_SPI_Receive_IT(hspi, data, len) != HAL_OK)
	{
		drv_spi_cs_release(drv);
		return DRV_ERROR;
	}

	return DRV_EOK;
}

/**
 * @brief       SPI中断方式全双工收发(非阻塞)
 */
int drv_spi_transfer_it(spiDrv_t drv, uint8_t *tx_data, uint8_t *rx_data, uint16_t len)
{
	SPI_HandleTypeDef *hspi;

	hspi = get_spi_handle(drv.hspi);
	if (hspi == NULL)
	{
		return DRV_ERROR;
	}

	drv_spi_cs_select(drv);
	if (HAL_SPI_TransmitReceive_IT(hspi, tx_data, rx_data, len) != HAL_OK)
	{
		drv_spi_cs_release(drv);
		return DRV_ERROR;
	}

	return DRV_EOK;
}

/**
 * @brief       SPI DMA方式发送(非阻塞)
 */
int drv_spi_send_dma(spiDrv_t drv, uint8_t *data, uint16_t len)
{
	SPI_HandleTypeDef *hspi;

	hspi = get_spi_handle(drv.hspi);
	if (hspi == NULL)
	{
		return DRV_ERROR;
	}

	drv_spi_cs_select(drv);
	if (HAL_SPI_Transmit_DMA(hspi, data, len) != HAL_OK)
	{
		drv_spi_cs_release(drv);
		return DRV_ERROR;
	}

	return DRV_EOK;
}

/**
 * @brief       SPI DMA方式接收(非阻塞)
 */
int drv_spi_read_dma(spiDrv_t drv, uint8_t *data, uint16_t len)
{
	SPI_HandleTypeDef *hspi;

	hspi = get_spi_handle(drv.hspi);
	if (hspi == NULL)
	{
		return DRV_ERROR;
	}

	drv_spi_cs_select(drv);
	if (HAL_SPI_Receive_DMA(hspi, data, len) != HAL_OK)
	{
		drv_spi_cs_release(drv);
		return DRV_ERROR;
	}

	return DRV_EOK;
}

/**
 * @brief       SPI DMA方式全双工收发(非阻塞)
 */
int drv_spi_transfer_dma(spiDrv_t drv, uint8_t *tx_data, uint8_t *rx_data, uint16_t len)
{
	SPI_HandleTypeDef *hspi;

	hspi = get_spi_handle(drv.hspi);
	if (hspi == NULL)
	{
		return DRV_ERROR;
	}

	drv_spi_cs_select(drv);
	if (HAL_SPI_TransmitReceive_DMA(hspi, tx_data, rx_data, len) != HAL_OK)
	{
		drv_spi_cs_release(drv);
		return DRV_ERROR;
	}

	return DRV_EOK;
}

#endif /* USE_SPI_DRIVER */

