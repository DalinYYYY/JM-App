/**
 * @file        drv_usart.c
 * @brief       串口驱动实现，封装HAL的UART阻塞/中断/DMA/空闲中断不定长收发
 *
 * @author      Dalin (dalin@robot.com)
 * @version     1.0
 * @date        2026-06-15
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @par 修改日志:
 * | 日期       | 版本 | 作者   | 修改内容   |
 * |------------|------|--------|------------|
 * | 2026-06-15 | 1.0  | Dalin  | 初始创建   |
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 */
#include "drv_usart.h"

#ifdef USE_USART_DRIVER
#include "dma.h"
#include "usart.h"
#include <stdlib.h>
#include <string.h>

__weak UART_HandleTypeDef huart1;
__weak UART_HandleTypeDef huart2;
__weak UART_HandleTypeDef huart3;
__weak UART_HandleTypeDef huart4;
__weak UART_HandleTypeDef huart5;
__weak UART_HandleTypeDef huart6;

__weak DMA_HandleTypeDef hdma_usart1_rx;
__weak DMA_HandleTypeDef hdma_usart1_tx;
__weak DMA_HandleTypeDef hdma_usart2_rx;
__weak DMA_HandleTypeDef hdma_usart2_tx;
__weak DMA_HandleTypeDef hdma_usart3_rx;
__weak DMA_HandleTypeDef hdma_usart3_tx;
__weak DMA_HandleTypeDef hdma_uart4_rx;
__weak DMA_HandleTypeDef hdma_uart4_tx;
__weak DMA_HandleTypeDef hdma_uart5_rx;
__weak DMA_HandleTypeDef hdma_uart5_tx;
__weak DMA_HandleTypeDef hdma_usart6_rx;
__weak DMA_HandleTypeDef hdma_usart6_tx;

/* 串口句柄查找表：以usartNumber_e为索引，O(1)定位HAL句柄 */
static UART_HandleTypeDef *const s_uart_map[DRV_UART_NUMBER_MAX] = {
	[DRV_UART1] = &huart1,
	[DRV_UART2] = &huart2,
	[DRV_UART3] = &huart3,
	[DRV_UART4] = &huart4,
	[DRV_UART5] = &huart5,
	[DRV_UART6] = &huart6,
};

/* DMA发送/接收通道查找表 */
static DMA_HandleTypeDef *const s_uart_dma_tx_map[DRV_UART_NUMBER_MAX] = {
	[DRV_UART1] = &hdma_usart1_tx,
	[DRV_UART2] = &hdma_usart2_tx,
	[DRV_UART3] = &hdma_usart3_tx,
	[DRV_UART4] = &hdma_uart4_tx,
	[DRV_UART5] = &hdma_uart5_tx,
	[DRV_UART6] = &hdma_usart6_tx,
};

static DMA_HandleTypeDef *const s_uart_dma_rx_map[DRV_UART_NUMBER_MAX] = {
	[DRV_UART1] = &hdma_usart1_rx,
	[DRV_UART2] = &hdma_usart2_rx,
	[DRV_UART3] = &hdma_usart3_rx,
	[DRV_UART4] = &hdma_uart4_rx,
	[DRV_UART5] = &hdma_uart5_rx,
	[DRV_UART6] = &hdma_usart6_rx,
};

/* 打印重定向目标(由drv_config.h的PRINTF_PORT选择) */
#if (PRINTF_PORT == 0x01)
#define PRINTF_API (&huart1)
#elif (PRINTF_PORT == 0x02)
#define PRINTF_API (&huart2)
#elif (PRINTF_PORT == 0x03)
#define PRINTF_API (&huart3)
#elif (PRINTF_PORT == 0x04)
#define PRINTF_API (&huart4)
#elif (PRINTF_PORT == 0x05)
#define PRINTF_API (&huart5)
#elif (PRINTF_PORT == 0x06)
#define PRINTF_API (&huart6)
#endif

static inline UART_HandleTypeDef *get_usart_handle(usartNumber_e usart)
{
	if (usart >= DRV_UART_NUMBER_MAX)
		return NULL;

	return s_uart_map[usart];
}

static inline DMA_HandleTypeDef *get_usart_dma_tx_ch(usartNumber_e uart)
{
	if (uart >= DRV_UART_NUMBER_MAX)
		return NULL;

	return s_uart_dma_tx_map[uart];
}

static inline DMA_HandleTypeDef *get_usart_dma_rx_ch(usartNumber_e uart)
{
	if (uart >= DRV_UART_NUMBER_MAX)
		return NULL;

	return s_uart_dma_rx_map[uart];
}

/* 打印函数(fputc/__io_putchar重定向) */
PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(PRINTF_API, (uint8_t *)&ch, 1, 0xFFFF);
	return ch;
}

/*************************************** 阻塞 ***************************************/

/**
 * @brief       串口阻塞发送
 */
int drv_usart_send(usartNumber_e usart, uint8_t *data, uint16_t len, uint32_t timeout)
{
	UART_HandleTypeDef *handle;

	handle = get_usart_handle(usart);
	if (handle == NULL)
	{
		return DRV_ERROR;
	}

	return (HAL_UART_Transmit(handle, data, len, timeout) == HAL_OK) ? DRV_EOK : DRV_ERROR;
}

/**
 * @brief       串口阻塞接收
 */
int drv_usart_recv(usartNumber_e usart, uint8_t *data, uint16_t len, uint32_t timeout)
{
	UART_HandleTypeDef *handle;

	handle = get_usart_handle(usart);
	if (handle == NULL)
	{
		return DRV_ERROR;
	}

	return (HAL_UART_Receive(handle, data, len, timeout) == HAL_OK) ? DRV_EOK : DRV_ERROR;
}

/*************************************** 中断 ***************************************/

/**
 * @brief       串口中断方式发送
 */
int drv_usart_send_it(usartNumber_e usart, uint8_t *data, uint16_t len)
{
	UART_HandleTypeDef *handle;

	handle = get_usart_handle(usart);
	if (handle == NULL)
	{
		return DRV_ERROR;
	}

	return (HAL_UART_Transmit_IT(handle, data, len) == HAL_OK) ? DRV_EOK : DRV_ERROR;
}

/**
 * @brief       串口中断方式接收
 */
int drv_usart_recv_it(usartNumber_e usart, uint8_t *data, uint16_t len)
{
	UART_HandleTypeDef *handle;

	handle = get_usart_handle(usart);
	if (handle == NULL)
	{
		return DRV_ERROR;
	}

	return (HAL_UART_Receive_IT(handle, data, len) == HAL_OK) ? DRV_EOK : DRV_ERROR;
}

/**
 * @brief       串口接收到空闲方式中断接收
 */
int drv_usart_recv_idle_it(usartNumber_e usart, uint8_t *data, uint16_t len)
{
	UART_HandleTypeDef *handle;

	handle = get_usart_handle(usart);
	if (handle == NULL)
	{
		return DRV_ERROR;
	}

	return (HAL_UARTEx_ReceiveToIdle_IT(handle, data, len) == HAL_OK) ? DRV_EOK : DRV_ERROR;
}

/*************************************** DMA ***************************************/

/**
 * @brief       串口DMA方式发送
 */
int drv_uart_dma_send(usartNumber_e uart, uint8_t *data, uint16_t len)
{
	UART_HandleTypeDef *handle;

	handle = get_usart_handle(uart);
	if (handle == NULL)
	{
		return DRV_ERROR;
	}

	return (HAL_UART_Transmit_DMA(handle, data, len) == HAL_OK) ? DRV_EOK : DRV_ERROR;
}

/**
 * @brief       串口DMA方式接收
 */
int drv_uart_dma_recv(usartNumber_e uart, uint8_t *data, uint16_t len)
{
	UART_HandleTypeDef *handle;

	handle = get_usart_handle(uart);
	if (handle == NULL)
	{
		return DRV_ERROR;
	}

	return (HAL_UART_Receive_DMA(handle, data, len) == HAL_OK) ? DRV_EOK : DRV_ERROR;
}

/*************************************** DMA+IDLE：不定长接收 ***************************************/

static idleData_t dam_rx[DRV_UART_NUMBER_MAX];

/**
 * @brief       创建空闲中断不定长接收并启动(内部malloc缓存)
 */
int usart_idle_init(usartNumber_e uart, uint16_t len)
{
	UART_HandleTypeDef *handle;
	uint8_t *p;

	if (uart >= DRV_UART_NUMBER_MAX)
	{
		return DRV_ERROR;
	}

	p = (uint8_t *)malloc(len);
	if (p == NULL)
	{
		return DRV_ERROR;
	}
	dam_rx[uart].p = p;
	dam_rx[uart].max = len;

	handle = get_usart_handle(uart);
	if (handle == NULL)
	{
		dam_rx[uart].p = NULL;
		dam_rx[uart].max = 0;
		free(p);
		return DRV_ERROR;
	}

	HAL_UART_Receive_DMA(handle, dam_rx[uart].p, dam_rx[uart].max);
	__HAL_UART_ENABLE_IT(handle, UART_IT_IDLE); /* 开启空闲中断 */

	return DRV_EOK;
}

/**
 * @brief       启动一次空闲中断+DMA接收
 */
int uart_start_idle_recv(usartNumber_e uart, uint8_t *data, uint16_t len)
{
	UART_HandleTypeDef *handle;

	handle = get_usart_handle(uart);
	if (handle == NULL)
	{
		return DRV_ERROR;
	}
	HAL_UART_Receive_DMA(handle, data, len);
	__HAL_UART_ENABLE_IT(handle, UART_IT_IDLE);

	return DRV_EOK;
}

/**
 * @brief       空闲中断处理(需在串口中断服务函数中调用)
 * @note         检测IDLE标志，停止DMA并计算已接收长度，置位接收完成标志
 */
int drv_uart_idle(usartNumber_e uart)
{
	UART_HandleTypeDef *handle;
	DMA_HandleTypeDef *dma_ch;
	uint32_t tmp_flag;
	uint32_t temp;

	handle = get_usart_handle(uart);
	if (handle == NULL)
	{
		return DRV_ERROR;
	}
	dma_ch = get_usart_dma_rx_ch(uart);
	if (dma_ch == NULL)
	{
		return DRV_ERROR;
	}

	tmp_flag = __HAL_UART_GET_FLAG(handle, UART_FLAG_IDLE);
	if (tmp_flag != RESET)
	{
		__HAL_UART_CLEAR_IDLEFLAG(handle);
		HAL_UART_DMAStop(handle);

		/* 总计数减去DMA未传输个数，得到已接收个数 */
		temp = __HAL_DMA_GET_COUNTER(dma_ch);
		dam_rx[uart].len = dam_rx[uart].max - temp;
		dam_rx[uart].flag = 1;
	}
	return DRV_EOK;
}

/**
 * @brief       取出空闲中断接收到的数据
 */
int usart_idle_get_data(usartNumber_e uart, uint8_t *data, uint16_t *len)
{
	UART_HandleTypeDef *handle;

	handle = get_usart_handle(uart);
	if (handle == NULL)
	{
		return DRV_ERROR;
	}

	if (dam_rx[uart].flag == 1)
	{
		dam_rx[uart].flag = 0;

		memcpy(data, dam_rx[uart].p, dam_rx[uart].len);
		*len = dam_rx[uart].len;

		memset(dam_rx[uart].p, 0, dam_rx[uart].max);
		dam_rx[uart].len = 0;

		/* 重新使能idle接收 */
		uart_start_idle_recv(uart, dam_rx[uart].p, dam_rx[uart].max);
	}

	return DRV_EOK;
}

/**
 * @brief       空闲中断收发自测示例
 */
void usart_idle_test(void)
{
	static int en = 0;
	uint16_t rx_len = 0;
	uint8_t buf[32] = {0};

	if (en == 0)
	{
		usart_idle_init(DRV_UART1, 32);
		en = 1;
	}
	else
	{
		usart_idle_get_data(DRV_UART1, buf, &rx_len);
	}
}

#endif /* USE_USART_DRIVER */