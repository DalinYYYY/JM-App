/**
 * @file        drv_usart.c
 * @brief       串口驱动实现，封装HAL的UART阻塞/中断/DMA/空闲中断不定长收发
 *
 * @author      Dalin (dalinyy@163.com)
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
#include "main.h"
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
static uint8_t s_idle_rx_buf[DRV_UART_NUMBER_MAX][DMA_IDLE_LEN];

/**
 * @brief       创建空闲中断不定长接收并启动(内部malloc缓存)
 */
int usart_idle_init(usartNumber_e uart, uint16_t len)
{
	UART_HandleTypeDef *handle;
	HAL_StatusTypeDef hal_ret;

	if (uart >= DRV_UART_NUMBER_MAX || len == 0u || len > DMA_IDLE_LEN)
	{
		return DRV_ERROR;
	}

	dam_rx[uart].p = s_idle_rx_buf[uart];
	dam_rx[uart].max = len;
	dam_rx[uart].len = 0;
	dam_rx[uart].read_pos = 0;
	dam_rx[uart].write_pos = 0;
	dam_rx[uart].flag = 0;
	dam_rx[uart].last_error = DRV_EOK;
	memset(dam_rx[uart].p, 0, len);

	handle = get_usart_handle(uart);
	if (handle == NULL)
	{
		dam_rx[uart].p = NULL;
		dam_rx[uart].max = 0;
		dam_rx[uart].last_error = DRV_ERROR;
		return DRV_ERROR;
	}

	hal_ret = HAL_UART_Receive_DMA(handle, dam_rx[uart].p, dam_rx[uart].max);
	if (hal_ret != HAL_OK)
	{
		dam_rx[uart].p = NULL;
		dam_rx[uart].max = 0;
		dam_rx[uart].last_error = (int)hal_ret;
		return DRV_ERROR;
	}
	__HAL_UART_ENABLE_IT(handle, UART_IT_IDLE); /* 开启空闲中断 */

	dam_rx[uart].init_count++;
	return DRV_EOK;
}

/**
 * @brief       启动一次空闲中断+DMA接收
 */
int uart_start_idle_recv(usartNumber_e uart, uint8_t *data, uint16_t len)
{
	UART_HandleTypeDef *handle;
	HAL_StatusTypeDef hal_ret;

	handle = get_usart_handle(uart);
	if (handle == NULL)
	{
		return DRV_ERROR;
	}
	hal_ret = HAL_UART_Receive_DMA(handle, data, len);
	if (hal_ret != HAL_OK)
	{
		return DRV_ERROR;
	}
	__HAL_UART_ENABLE_IT(handle, UART_IT_IDLE);

	return DRV_EOK;
}

/**
 * @brief       空闲中断处理(需在串口中断服务函数中调用)
 * @note         检测IDLE标志并置位接收完成标志；环形DMA长度在 usart_idle_get_data() 中按读写指针计算
 */
int drv_uart_idle(usartNumber_e uart)
{
	UART_HandleTypeDef *handle;
	uint32_t tmp_flag;

	handle = get_usart_handle(uart);
	if (handle == NULL)
	{
		return DRV_ERROR;
	}
	tmp_flag = __HAL_UART_GET_FLAG(handle, UART_FLAG_IDLE);
	if (tmp_flag != RESET)
	{
		__HAL_UART_CLEAR_IDLEFLAG(handle);
		dam_rx[uart].flag = 1;
		dam_rx[uart].idle_count++;
	}
	return DRV_EOK;
}

/**
 * @brief       取出空闲中断接收到的数据
 */
int usart_idle_get_data(usartNumber_e uart, uint8_t *data, uint16_t *len)
{
	UART_HandleTypeDef *handle;
	DMA_HandleTypeDef *dma_ch;
	uint16_t write_pos;
	uint16_t read_pos;
	uint16_t max;
	uint16_t copy_len = 0;

	handle = get_usart_handle(uart);
	if (handle == NULL || data == NULL || len == NULL)
	{
		if (len != NULL)
		{
			*len = 0;
		}
		return DRV_ERROR;
	}
	dma_ch = get_usart_dma_rx_ch(uart);
	if (dma_ch == NULL || dam_rx[uart].p == NULL || dam_rx[uart].max == 0)
	{
		*len = 0;
		return DRV_ERROR;
	}

	*len = 0;
	dam_rx[uart].get_count++;
	dam_rx[uart].flag = 0;
	max = dam_rx[uart].max;
	read_pos = dam_rx[uart].read_pos;
	write_pos = (uint16_t)(max - __HAL_DMA_GET_COUNTER(dma_ch));
	if (write_pos >= max)
	{
		write_pos = 0;
	}
	dam_rx[uart].write_pos = write_pos;

	if (write_pos >= read_pos)
	{
		copy_len = (uint16_t)(write_pos - read_pos);
		if (copy_len != 0)
		{
			memcpy(data, &dam_rx[uart].p[read_pos], copy_len);
		}
	}
	else
	{
		uint16_t first_len = (uint16_t)(max - read_pos);
		memcpy(data, &dam_rx[uart].p[read_pos], first_len);
		if (write_pos != 0)
		{
			memcpy(&data[first_len], dam_rx[uart].p, write_pos);
		}
		copy_len = (uint16_t)(first_len + write_pos);
	}

	dam_rx[uart].read_pos = write_pos;
	dam_rx[uart].len = copy_len;
	dam_rx[uart].rx_bytes += copy_len;
	dam_rx[uart].last_error = DRV_EOK;
	*len = copy_len;

	return DRV_EOK;
}

const idleData_t *usart_idle_get_status(usartNumber_e uart)
{
	if (uart >= DRV_UART_NUMBER_MAX)
	{
		return NULL;
	}
	return &dam_rx[uart];
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

/**
 * @brief       UART 错误回调(覆盖 HAL weak 实现)
 * @note        本回调针对 F4 HAL DMA bug 修复:
 *              F4 HAL 在 DMA 模式下遇到 ORE/NE/FE 错误时会调用 UART_EndRxTransfer
 *              永久禁用 DMAR 位并中止 DMA RX stream, 导致接收彻底瘫痪。
 *              G4 HAL 的 __HAL_UART_CLEAR_IDLEFLAG 写 ICR 不读 DR, 错误概率极低,
 *              本回调在 G4 上虽会执行但属冗余保护, 不影响功能。
 *              (DMA 此时已被 HAL 停止, 读 DR 不再与 DMA 竞争, 安全)
 * @sa          项目记忆 2026-07-15: F4 HAL __HAL_UART_CLEAR_IDLEFLAG 读 SR+DR 引发 ORE
 * @sa          drv_usart.c 第 23 行: HAL 头由 main.h 自动适配各板
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	usartNumber_e i;
	for (i = DRV_UART1; i < DRV_UART_NUMBER_MAX; i++)
	{
		if (s_uart_map[i] == huart && dam_rx[i].p != NULL && dam_rx[i].max > 0)
		{
			/* 清错误标志(F4: 读SR+DR, 此时 DMA 已停, 安全) */
			__HAL_UART_CLEAR_IDLEFLAG(huart);
			/* 停止并重启 DMA 接收 */
			HAL_UART_DMAStop(huart);
			HAL_UART_Receive_DMA(huart, dam_rx[i].p, dam_rx[i].max);
			__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
			dam_rx[i].last_error = -(int)(huart->ErrorCode);
			huart->ErrorCode = HAL_UART_ERROR_NONE;
			break;
		}
	}
}

#endif /* USE_USART_DRIVER */
