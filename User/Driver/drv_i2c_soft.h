/**
 * @file        drv_i2c_soft.h
 * @brief       软件模拟I2C驱动接口，通过GPIO回调实现，与具体MCU/HAL解耦
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
 * @note        SDA/SCL的GPIO操作由使用者通过回调注入，本模块不依赖任何HAL类型
 */
#ifndef _DRV_I2C_SOFT_H_
#define _DRV_I2C_SOFT_H_

#include "drv_config.h"
#include "drv_gpio.h"

#ifdef USE_SOFT_I2C_DRIVER

/**
 * @brief 软件I2C实例编号
 * @param  I2C_ID_1              : 实例1
 * @param  I2C_ID_2              : 实例2
 * @param  I2C_ID_MAX            : 实例数量边界
 */
typedef enum
{
	I2C_ID_1 = 0u,
	I2C_ID_2,
	I2C_ID_MAX,
} i2c_id_e;

/**
 * @brief SDA引脚方向
 * @param  IN                    : 输入
 * @param  OUT                   : 输出
 * @param  DIR_MAX               : 方向数量边界
 */
typedef enum
{
	IN = 0u,
	OUT,
	DIR_MAX
} i2c_sda_dir_e;

/**
 * @brief 引脚电平
 * @param  I2C_LOW               : 低电平
 * @param  I2C_HIGH              : 高电平
 */
typedef enum
{
	I2C_LOW = 0,
	I2C_HIGH
} i2c_state_e;

/**
 * @brief 应答状态
 * @param  NACK                  : 非应答
 * @param  ACK                   : 应答
 */
typedef enum
{
	NACK = 0u,
	ACK
} i2c_ack_state_e;

/**
 * @brief 软件I2C设备描述
 * @param  id                    : 实例编号
 * @param  slave_address         : 从机地址(8bit，含读写位基址)
 * @param  sda_read              : 回调-读SDA电平
 * @param  sda_dir               : 回调-设置SDA方向
 * @param  sda                   : 回调-设置SDA电平
 * @param  scl                   : 回调-设置SCL电平
 * @param  write_nbytes          : 接口-写多字节
 * @param  read_nbytes           : 接口-读多字节
 */
typedef struct i2c_soft_drv
{
	i2c_id_e id;
	uint8_t slave_address;
	uint8_t (*sda_read)(i2c_id_e id);
	void (*sda_dir)(i2c_id_e id, i2c_sda_dir_e dir);
	void (*sda)(i2c_id_e id, i2c_state_e level);
	void (*scl)(i2c_id_e id, i2c_state_e level);

	int (*write_nbytes)(struct i2c_soft_drv *pobj, uint8_t reg, uint8_t *data, uint8_t len);
	int (*read_nbytes)(struct i2c_soft_drv *pobj, uint8_t reg, uint8_t *data, uint8_t len);
} i2c_soft_drv_t;

/**
 * @brief       初始化软件I2C设备(注入GPIO操作回调)
 * @param        pobj              : 软件I2C设备对象
 * @param        id                : 实例编号
 * @param        sda_read          : 读SDA电平回调
 * @param        sda_dir           : 设置SDA方向回调
 * @param        sda               : 设置SDA电平回调
 * @param        scl               : 设置SCL电平回调
 * @param        hw_addr           : 从机地址(8bit基址)
 */
void drv_i2c_init(i2c_soft_drv_t *pobj, i2c_id_e id,
                  uint8_t (*sda_read)(i2c_id_e id),
                  void (*sda_dir)(i2c_id_e id, i2c_sda_dir_e dir),
                  void (*sda)(i2c_id_e id, i2c_state_e level),
                  void (*scl)(i2c_id_e id, i2c_state_e level),
                  uint8_t hw_addr);

/**
 * @brief       向从机寄存器写一个字节
 * @param        i2c               : 软件I2C设备
 * @param        reg               : 寄存器地址
 * @param        data              : 写入数据
 * @return       : DRV_EOK成功，DRV_ERROR失败
 */
int drv_i2c_write_byte(i2c_soft_drv_t *i2c, uint8_t reg, uint8_t data);

/**
 * @brief       向从机寄存器写多字节
 * @param        i2c               : 软件I2C设备
 * @param        reg               : 寄存器地址
 * @param        data              : 写入数据缓冲区
 * @param        len               : 字节数
 * @return       : DRV_EOK成功，DRV_ERROR失败
 */
int drv_i2c_write_nbytes(i2c_soft_drv_t *i2c, uint8_t reg, uint8_t *data, uint8_t len);

/**
 * @brief       从从机寄存器读多字节
 * @param        i2c               : 软件I2C设备
 * @param        reg               : 寄存器地址
 * @param        data              : 读出数据缓冲区
 * @param        len               : 字节数
 * @return       : DRV_EOK成功，DRV_ERROR失败
 */
int drv_i2c_read_nbytes(i2c_soft_drv_t *i2c, uint8_t reg, uint8_t *data, uint8_t len);

#endif /* USE_SOFT_I2C_DRIVER */
#endif /* _DRV_I2C_SOFT_H_ */
