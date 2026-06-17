/**
 * @file        drv_i2c_soft.c
 * @brief       软件模拟I2C驱动实现，通过GPIO回调实现，与具体MCU/HAL解耦
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
#include "drv_i2c_soft.h"

#ifdef USE_SOFT_I2C_DRIVER
#include <stdint.h>
#include <string.h>
#include "assert_report.h"

/* 软件I2C位延时：volatile防止-O优化下空循环被删除，循环次数决定总线速率 */
static void _i2c_delay(void)
{
	volatile uint8_t i;
	for (i = 0; i < 10; i++)
		;
}

/*
 * @brief  起始信号
 * @param  *i2c: 
 */
static void i2c_start(i2c_soft_drv_t *i2c)
{
	i2c->sda_dir(i2c->id, OUT); /* 设置sda为输出 */
	i2c->sda(i2c->id, I2C_HIGH);
	i2c->scl(i2c->id, I2C_HIGH);
	_i2c_delay();
	i2c->sda(i2c->id, I2C_LOW); /* 起始条件：当SCL为高电平时，SDA由高到低的跳变 */
	_i2c_delay();
	i2c->scl(i2c->id, I2C_LOW); /* 钳住I2C总线，准备发送或接收数据 */
}

/*
 * @brief  停止信号
 * @param  *i2c:
 */
static void i2c_stop(i2c_soft_drv_t *i2c)
{
	i2c->sda_dir(i2c->id, OUT); /* 设置sda为输出 */
	i2c->scl(i2c->id, I2C_LOW);
	i2c->sda(i2c->id, I2C_LOW); /* 停止条件： SCL线为高电平期间，SDA线由低电平向高电平的变化 */
	_i2c_delay();
	i2c->scl(i2c->id, I2C_HIGH);
	i2c->sda(i2c->id, I2C_HIGH); /* 发送I2C总线结束信号 */
	_i2c_delay();
}
/*
 * @brief  等待应答信号
 * @param  *i2c:
 * @retval 1接收应答失败，0接收应答成功
 */
static int i2c_wait_ack(i2c_soft_drv_t *i2c)
{
	uint8_t errTime = 0;

	i2c->sda_dir(i2c->id, IN); /* 设置sda为输入 */
	i2c->sda(i2c->id, I2C_HIGH);
	_i2c_delay();
	i2c->scl(i2c->id, I2C_HIGH);
	_i2c_delay();
	while (i2c->sda_read(i2c->id))
	{ /* 低电平则成功应答 */
		errTime++;
		if (errTime > 250)
		{
			i2c_stop(i2c);
			return DRV_ERROR;
		}
	}
	i2c->scl(i2c->id, I2C_LOW);
	return DRV_EOK;
}

/*
 * @brief  回复应答信号
 * @param  *i2c:
 */
static void i2c_ack(i2c_soft_drv_t *i2c)
{
	i2c->scl(i2c->id, I2C_LOW);
	i2c->sda_dir(i2c->id, OUT);
	i2c->sda(i2c->id, I2C_LOW);
	_i2c_delay();
	i2c->scl(i2c->id, I2C_HIGH);
	_i2c_delay();
	i2c->scl(i2c->id, I2C_LOW);
}

/*
 * @brief  回复nack信号
 * @param  *i2c:
 */
static void i2c_nack(i2c_soft_drv_t *i2c)
{
	i2c->scl(i2c->id, I2C_LOW);
	i2c->sda_dir(i2c->id, OUT);
	i2c->sda(i2c->id, I2C_HIGH);
	_i2c_delay();
	i2c->scl(i2c->id, I2C_HIGH);
	_i2c_delay();
	i2c->scl(i2c->id, I2C_LOW);
}

/*
 * @brief  发送一个字节数据
 * @param  *i2c : 
 * @param  byte : 发送的数据
 */
static void i2c_send_byte(i2c_soft_drv_t *i2c, uint8_t byte)
{
	uint8_t i = 0;

	i2c->sda_dir(i2c->id, OUT);
	i2c->scl(i2c->id, I2C_LOW); /* 拉低时钟开始数据传输 */
	for (i = 0; i < 8; i++)
	{
		if ((byte & 0x80) >> 7)
		{ /* 从高到低 */
			i2c->sda(i2c->id, I2C_HIGH);
		}
		else
		{
			i2c->sda(i2c->id, I2C_LOW);
		}
		byte <<= 1;
		_i2c_delay();
		i2c->scl(i2c->id, I2C_HIGH);
		_i2c_delay();
		i2c->scl(i2c->id, I2C_LOW);
		_i2c_delay();
	}
}

/*
 * @brief  读出一个字节数据
 * @param  *i2c     :
 * @param  ack      : ACK读完后发送应答信号，NACK读完后回复不应答
 * @retval receive  ：读出的结果
 */
static uint8_t i2c_read_byte(i2c_soft_drv_t *i2c, i2c_ack_state_e ack)
{
	uint8_t i = 0, receive = 0;

	i2c->sda_dir(i2c->id, IN); /* 输入 */
	for (i = 0; i < 8; i++)
	{
		receive <<= 1;
		i2c->scl(i2c->id, I2C_HIGH);
		_i2c_delay();
		if (i2c->sda_read(i2c->id))
		{
			receive++;
		}
		i2c->scl(i2c->id, I2C_LOW);
		_i2c_delay();
	}
	if (ack)
	{
		i2c_ack(i2c); /*ack*/
	}
	else
	{
		i2c_nack(i2c); /*Nack*/
	}
	return receive;
}

/*
 * @brief  向从机寄存器写一个字节数据
 * @param  *i2c :
 * @param  reg  : 寄存器地址
 * @param  data : 写入的数据
 * @retval 写入结果 ：0正常，其他则错误
 */
int drv_i2c_write_byte(i2c_soft_drv_t *i2c, uint8_t reg, uint8_t data)
{
	int ret = 0;
	i2c_start(i2c);
	i2c_send_byte(i2c, i2c->slave_address | 0); /* 器件地址 + 写命令 */
	ret |= i2c_wait_ack(i2c);
	i2c_send_byte(i2c, reg);
	ret |= i2c_wait_ack(i2c);
	i2c_send_byte(i2c, data);
	ret |= i2c_wait_ack(i2c);
	i2c_stop(i2c);

	return ret;
}

/*
 * @brief  向从机寄存器写多字节数据
 * @param  *i2c :
 * @param  reg  : 寄存器地址
 * @param  data : 写入的数据
 * @param  len  : 写入的字节数
 * @retval 写入结果 ：0正常，其他则错误
 */
int drv_i2c_write_nbytes(i2c_soft_drv_t *i2c, uint8_t reg, uint8_t *data, uint8_t len)
{
	int ret = 0;
	i2c_start(i2c);
	i2c_send_byte(i2c, i2c->slave_address | 0); /* 器件地址 + 写命令 */
	ret |= i2c_wait_ack(i2c);
	i2c_send_byte(i2c, reg);
	ret |= i2c_wait_ack(i2c);
	for (uint8_t i = 0; i < len; i++)
	{
		i2c_send_byte(i2c, data[i]);
		ret |= i2c_wait_ack(i2c);
	}
	i2c_stop(i2c);

	return ret;
}
/*
 * @brief  从从机寄存器读出多字节数据
 * @param  *i2c :
 * @param  reg  : 寄存器地址
 * @param  data : 读出的数据
 * @param  len  : 读出的字节数
 * @retval 读出结果 ：0正常，其他则错误
 */
int drv_i2c_read_nbytes(i2c_soft_drv_t *i2c, uint8_t reg, uint8_t *data, uint8_t len)
{
	int ret = 0;
	uint8_t i;

	if (len == 0)
	{
		return DRV_ERROR;
	}

	i2c_start(i2c);
	i2c_send_byte(i2c, i2c->slave_address | 0); /* 器件地址 + 写命令 */
	ret |= i2c_wait_ack(i2c);
	i2c_send_byte(i2c, reg);
	ret |= i2c_wait_ack(i2c);

	i2c_start(i2c);
	i2c_send_byte(i2c, i2c->slave_address | 1); /* 器件地址 + 读命令 */
	ret |= i2c_wait_ack(i2c);
	for (i = 0; i < len - 1; i++)
	{
		data[i] = i2c_read_byte(i2c, ACK);
	}
	data[i] = i2c_read_byte(i2c, NACK);
	i2c_stop(i2c);

	return ret;
}

void drv_i2c_init(i2c_soft_drv_t *pobj, i2c_id_e id,
				  uint8_t (*sda_read)(i2c_id_e id),
				  void (*sda_dir)(i2c_id_e id, i2c_sda_dir_e dir),
				  void (*sda)(i2c_id_e id, i2c_state_e level),
				  void (*scl)(i2c_id_e id, i2c_state_e level),
				  uint8_t hw_addr)
{
	assert_report(pobj != NULL);
	memset(pobj, 0, sizeof(i2c_soft_drv_t));

	pobj->id = id;
	pobj->sda_read = sda_read;
	pobj->sda_dir = sda_dir;
	pobj->sda = sda;
	pobj->scl = scl;
	pobj->slave_address = hw_addr;
	pobj->read_nbytes = drv_i2c_read_nbytes;
	pobj->write_nbytes = drv_i2c_write_nbytes;
}

#endif /* USE_SOFT_I2C_DRIVER */
