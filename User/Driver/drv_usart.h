/**
 * @file        drv_usart.h
 * @brief       串口驱动接口，封装HAL的UART阻塞/中断/DMA/空闲中断不定长收发
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
 * @note        串口初始化由CubeMX的MX_USARTx_UART_Init完成，此层仅封装运行期收发
 * @note        对外接口不暴露HAL类型，HAL句柄仅在drv_usart.c内部使用
 */
#ifndef DRV_USART_H_
#define DRV_USART_H_

#include "drv_config.h"

#ifdef USE_USART_DRIVER
#include <stdint.h>

#if 1 /* printf */
#include "stdio.h"
#if defined(__GNUC__) && !defined(__clang__)
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
#endif /* printf */

/* 定义接收DMA缓存大小，根据实际修改 */
#define DMA_IDLE_LEN 256

/**
 * @brief 串口编号
 * @param  DRV_UART_INIT         : 无效占位(0)
 * @param  DRV_UART1~DRV_UART6   : USART/UART 1~6
 * @param  DRV_UART_NUMBER_MAX   : 串口数量边界
 */
typedef enum USARTNUMBER_
{
	DRV_UART_INIT = 0,
	DRV_UART1,
	DRV_UART2,
	DRV_UART3,
	DRV_UART4,
	DRV_UART5,
	DRV_UART6,

	DRV_UART_NUMBER_MAX
} usartNumber_e;

#define DEV_PRINTF DRV_UART2

/**
 * @brief 空闲中断不定长接收缓存描述
 * @param  flag                  : 接收完成标志(1:有新数据)
 * @param  len                   : 本次接收到的数据长度
 * @param  max                   : 缓存区容量
 * @param  p                     : 缓存区指针
 */
typedef struct
{
	uint8_t flag;
	uint16_t len;
	uint16_t max;
	uint16_t read_pos;
	uint16_t write_pos;
	uint32_t init_count;
	uint32_t idle_count;
	uint32_t get_count;
	uint32_t rx_bytes;
	int last_error;
	uint8_t *p;
} idleData_t;

/*************************************** 阻塞 ***************************************/

/**
 * @brief       串口阻塞发送
 * @param        usart             : 串口编号
 * @param        data              : 发送数据缓冲区
 * @param        len               : 数据长度
 * @param        timeout           : 超时时间 (单位: ms)
 * @return       : DRV_EOK成功，DRV_ERROR失败
 */
int drv_usart_send(usartNumber_e usart, uint8_t *data, uint16_t len, uint32_t timeout);

/**
 * @brief       串口阻塞接收
 * @param        usart             : 串口编号
 * @param        data              : 接收数据缓冲区
 * @param        len               : 数据长度
 * @param        timeout           : 超时时间 (单位: ms)
 * @return       : DRV_EOK成功，DRV_ERROR失败
 */
int drv_usart_recv(usartNumber_e usart, uint8_t *data, uint16_t len, uint32_t timeout);

/*************************************** 中断 ***************************************/

/**
 * @brief       串口中断方式发送
 * @param        usart             : 串口编号
 * @param        data              : 发送数据缓冲区
 * @param        len               : 数据长度
 * @return       : DRV_EOK成功，DRV_ERROR失败
 */
int drv_usart_send_it(usartNumber_e usart, uint8_t *data, uint16_t len);

/**
 * @brief       串口中断方式接收
 * @param        usart             : 串口编号
 * @param        data              : 接收数据缓冲区
 * @param        len               : 数据长度
 * @return       : DRV_EOK成功，DRV_ERROR失败
 */
int drv_usart_recv_it(usartNumber_e usart, uint8_t *data, uint16_t len);

/**
 * @brief       串口接收到空闲方式中断接收(收满len或检测到总线空闲即完成)
 * @param        usart             : 串口编号
 * @param        data              : 接收数据缓冲区
 * @param        len               : 期望最大长度
 * @return       : DRV_EOK成功，DRV_ERROR失败
 */
int drv_usart_recv_idle_it(usartNumber_e usart, uint8_t *data, uint16_t len);

/*************************************** DMA ***************************************/

/**
 * @brief       串口DMA方式发送
 * @param        uart              : 串口编号
 * @param        data              : 发送数据缓冲区
 * @param        len               : 数据长度
 * @return       : DRV_EOK成功，DRV_ERROR失败
 */
int drv_uart_dma_send(usartNumber_e uart, uint8_t *data, uint16_t len);

/**
 * @brief       串口DMA方式接收
 * @param        uart              : 串口编号
 * @param        data              : 接收数据缓冲区
 * @param        len               : 数据长度
 * @return       : DRV_EOK成功，DRV_ERROR失败
 */
int drv_uart_dma_recv(usartNumber_e uart, uint8_t *data, uint16_t len);

/************************************ DMA 空闲中断 **********************************/

/**
 * @brief       创建空闲中断不定长接收并启动(内部malloc缓存)
 * @param        uart              : 串口编号
 * @param        len               : 缓存区大小
 * @return       : DRV_EOK成功，DRV_ERROR失败
 */
int usart_idle_init(usartNumber_e uart, uint16_t len);

/**
 * @brief       启动一次空闲中断+DMA接收
 * @param        uart              : 串口编号
 * @param        data              : 接收数据缓冲区
 * @param        len               : 缓冲区长度
 * @return       : DRV_EOK成功，DRV_ERROR失败
 */
int uart_start_idle_recv(usartNumber_e uart, uint8_t *data, uint16_t len);

/**
 * @brief       空闲中断处理(需在串口中断服务函数中调用)
 * @param        uart              : 串口编号
 * @return       : DRV_EOK成功，DRV_ERROR失败
 */
int drv_uart_idle(usartNumber_e uart);

/**
 * @brief       取出空闲中断接收到的数据
 * @param        uart              : 串口编号
 * @param        data              : 输出数据缓冲区
 * @param        len               : 输出数据长度
 * @return       : DRV_EOK成功，DRV_ERROR失败
 */
int usart_idle_get_data(usartNumber_e uart, uint8_t *data, uint16_t *len);

/**
 * @brief       获取空闲 DMA 接收状态，供调试观察
 * @param        uart              : 串口编号
 * @return       : 内部状态指针，非法编号返回 NULL
 */
const idleData_t *usart_idle_get_status(usartNumber_e uart);

/**
 * @brief       空闲中断收发自测示例
 */
void usart_idle_test(void);

#endif /* USE_USART_DRIVER */
#endif /* DRV_USART_H_ */
