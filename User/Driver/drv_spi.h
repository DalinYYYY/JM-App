/**
 * @file        drv_spi.h
 * @brief       SPI驱动接口，封装HAL的SPI阻塞/中断/DMA收发及片选控制
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
 * @note        SPI初始化由CubeMX的MX_SPIx_Init完成，此层仅封装运行期收发接口
 * @note        对外接口不暴露HAL类型，HAL句柄仅在drv_spi.c内部使用
 */
#ifndef _DRV_SPI_H_
#define _DRV_SPI_H_

#include "drv_config.h"

#ifdef USE_SPI_DRIVER
#include "drv_gpio.h"

/**
 * @brief SPI外设编号
 * @param  DRV_SPI_INIT          : 无效占位(0)
 * @param  DRV_SPI1              : SPI1
 * @param  DRV_SPI2              : SPI2
 * @param  DRV_SPI3              : SPI3
 * @param  DRV_SPI_NUMBER_MAX    : 外设数量边界
 */
typedef enum SPINUMBER_
{
	DRV_SPI_INIT = 0,
	DRV_SPI1,
	DRV_SPI2,
	DRV_SPI3,
	DRV_SPI_NUMBER_MAX
} spiNumber_e;

/**
 * @brief SPI设备描述
 * @param  hspi                  : SPI外设编号
 * @param  cs                    : 片选引脚(低电平有效)，cs.gpiox为DRV_GPIO_INIT时不使用CS
 */
typedef struct DRV_SPI_
{
	spiNumber_e hspi;
	gpioDrv_t cs;
} spiDrv_t;

/********************************** drv spi api **********************************/

/**
 * @brief       片选选中(拉低CS)，cs未配置时为空操作
 * @param        drv               : SPI设备
 */
void drv_spi_cs_select(spiDrv_t drv);

/**
 * @brief       片选释放(拉高CS)，cs未配置时为空操作
 * @param        drv               : SPI设备
 * @note         DMA/中断方式收发后，应在用户的传输完成回调中调用本接口释放CS
 */
void drv_spi_cs_release(spiDrv_t drv);

/**
 * @brief       SPI发送数据(阻塞，自动管理CS)
 * @param        drv               : SPI设备
 * @param        data              : 发送数据缓冲区
 * @param        len               : 数据长度
 * @param        timeout           : 超时时间 (单位: ms)
 * @return       : DRV_EOK成功，DRV_ERROR失败
 */
int drv_spi_send(spiDrv_t drv, uint8_t *data, uint16_t len, uint32_t timeout);

/**
 * @brief       SPI接收数据(阻塞，自动管理CS)
 * @param        drv               : SPI设备
 * @param        data              : 接收数据缓冲区
 * @param        len               : 数据长度
 * @param        timeout           : 超时时间 (单位: ms)
 * @return       : DRV_EOK成功，DRV_ERROR失败
 */
int drv_spi_read(spiDrv_t drv, uint8_t *data, uint16_t len, uint32_t timeout);

/**
 * @brief       SPI全双工收发(阻塞，自动管理CS)
 * @param        drv               : SPI设备
 * @param        tx_data           : 发送数据缓冲区
 * @param        rx_data           : 接收数据缓冲区
 * @param        len               : 数据长度
 * @param        timeout           : 超时时间 (单位: ms)
 * @return       : DRV_EOK成功，DRV_ERROR失败
 */
int drv_spi_transfer(spiDrv_t drv, uint8_t *tx_data, uint8_t *rx_data, uint16_t len, uint32_t timeout);

/**
 * @brief       SPI全双工收发(寄存器直操作轻量版，适用于ISR热路径短帧)
 * @param        drv               : SPI设备
 * @param        tx_data           : 发送数据缓冲区
 * @param        rx_data           : 接收数据缓冲区
 * @param        len               : 数据长度(字节，需与SPI配置的DataSize一致)
 * @return       : DRV_EOK成功，DRV_ERROR失败(非法参数/轮询超上限)
 * @note         直接操作SPI寄存器收发，绕过HAL的状态机/回调/超时框架，
 *               6字节帧可比HAL_SPI_TransmitReceive节省约12us软件开销。
 *               入口幂等补齐SPE使能与FRXTH=1(8bit接收门限)，无需调用方
 *               先发起过HAL事务。CS由调用方自行管理(本函数不碰CS)；
 *               内部有轮询次数上限防止SPI异常时死等，超上限关SPE复位
 *               总线后返回错误，调用方自行容错。
 */
int drv_spi_transfer_fast(spiDrv_t drv, const uint8_t *tx_data, uint8_t *rx_data, uint16_t len);

/**
 * @brief       SPI中断方式发送(非阻塞，启动前拉低CS)
 * @param        drv               : SPI设备
 * @param        data              : 发送数据缓冲区
 * @param        len               : 数据长度
 * @return       : DRV_EOK成功，DRV_ERROR失败
 * @note         传输在中断中完成，需在完成回调中调用drv_spi_cs_release
 */
int drv_spi_send_it(spiDrv_t drv, uint8_t *data, uint16_t len);

/**
 * @brief       SPI中断方式接收(非阻塞，启动前拉低CS)
 * @param        drv               : SPI设备
 * @param        data              : 接收数据缓冲区
 * @param        len               : 数据长度
 * @return       : DRV_EOK成功，DRV_ERROR失败
 * @note         传输在中断中完成，需在完成回调中调用drv_spi_cs_release
 */
int drv_spi_read_it(spiDrv_t drv, uint8_t *data, uint16_t len);

/**
 * @brief       SPI中断方式全双工收发(非阻塞，启动前拉低CS)
 * @param        drv               : SPI设备
 * @param        tx_data           : 发送数据缓冲区
 * @param        rx_data           : 接收数据缓冲区
 * @param        len               : 数据长度
 * @return       : DRV_EOK成功，DRV_ERROR失败
 * @note         传输在中断中完成，需在完成回调中调用drv_spi_cs_release
 */
int drv_spi_transfer_it(spiDrv_t drv, uint8_t *tx_data, uint8_t *rx_data, uint16_t len);

/**
 * @brief       SPI DMA方式发送(非阻塞，启动前拉低CS)
 * @param        drv               : SPI设备
 * @param        data              : 发送数据缓冲区
 * @param        len               : 数据长度
 * @return       : DRV_EOK成功，DRV_ERROR失败
 * @note         需CubeMX已为该SPI配置DMA；完成回调中调用drv_spi_cs_release
 */
int drv_spi_send_dma(spiDrv_t drv, uint8_t *data, uint16_t len);

/**
 * @brief       SPI DMA方式接收(非阻塞，启动前拉低CS)
 * @param        drv               : SPI设备
 * @param        data              : 接收数据缓冲区
 * @param        len               : 数据长度
 * @return       : DRV_EOK成功，DRV_ERROR失败
 * @note         需CubeMX已为该SPI配置DMA；完成回调中调用drv_spi_cs_release
 */
int drv_spi_read_dma(spiDrv_t drv, uint8_t *data, uint16_t len);

/**
 * @brief       SPI DMA方式全双工收发(非阻塞，启动前拉低CS)
 * @param        drv               : SPI设备
 * @param        tx_data           : 发送数据缓冲区
 * @param        rx_data           : 接收数据缓冲区
 * @param        len               : 数据长度
 * @return       : DRV_EOK成功，DRV_ERROR失败
 * @note         需CubeMX已为该SPI配置DMA；完成回调中调用drv_spi_cs_release
 */
int drv_spi_transfer_dma(spiDrv_t drv, uint8_t *tx_data, uint8_t *rx_data, uint16_t len);

#endif /* USE_SPI_DRIVER */
#endif /* _DRV_SPI_H_ */
