/**
 * @file        dev_mt6835.c
 * @brief       MT6835磁编码器(21bit SPI): 角度/零点读写(寄存器/EEPROM)/方向
 *
 * @author      Dalin (dalin@robot.com)
 * @version     1.1
 * @date        2026-06-17
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @par 修改日志:
 * | 日期       | 版本 | 作者   | 修改内容                                   |
 * |------------|------|--------|--------------------------------------------|
 * | 2026-06-11 | 1.0  | Dalin  | 分离角度获取和角度转化                     |
 * | 2026-06-17 | 1.1  | Dalin  | 复用共享配置表; 去重去死码去魔数; 补init漏绑 |
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 */
#include "dev_mt6835.h"

#if defined(USE_DEV_MT6835)
#include "assert_report.h"
#include "drv_spi.h"
#include "drv_gpio.h"
#include <math.h>

#define MT6835_SPI_TIMEOUT (200u) /* SPI收发超时, ms */

/* 片选控制 */
static void dev_mt6835_csn_ctrl(struct dev_mt6835 *pobj, mt6835State_e state)
{
	drv_gpio_write(mt6835_list[pobj->id].csn, (drvPinState_e)state);
}

/* 读指定寄存器(突发读3字节: 命令字+数据) */
static void mt6835_read_reg(struct dev_mt6835 *pobj, mt6835_reg_enum_t reg, uint8_t *rxdata)
{
	uint16_t tx = MT6835_READ | reg;

	dev_mt6835_csn_ctrl(pobj, MT6835_LOW);
	drv_spi_transfer(mt6835_list[pobj->id].spi_num, (uint8_t *)&tx, rxdata, 3, MT6835_SPI_TIMEOUT);
	dev_mt6835_csn_ctrl(pobj, MT6835_HIGH);
}

/*
 * @brief 写寄存器, 返回true表示芯片应答0x55
 * @note  TODO: 当前命令字 tx=MT6835_WRITE|data 未编入reg地址, 写位置可能不符手册,
 *        需对照MT6835手册的写时序(命令+地址+数据)核实并在硬件上验证
 */
static bool mt6835_write_reg(dev_mt6835_t *pobj, mt6835_reg_enum_t reg, uint8_t data)
{
	uint8_t rxdata[3] = {0, 0, 0xFF};
	uint16_t tx = MT6835_WRITE | data;
	(void)reg;

	dev_mt6835_csn_ctrl(pobj, MT6835_LOW);
	drv_spi_transfer(mt6835_list[pobj->id].spi_num, (uint8_t *)&tx, rxdata, 3, MT6835_SPI_TIMEOUT);
	dev_mt6835_csn_ctrl(pobj, MT6835_HIGH);

	return (rxdata[2] == 0x55);
}

/* 写EEPROM, 返回true表示芯片应答0x55 (同write_reg, 命令字编码待手册核实) */
static bool mt6835_write_eeprom(dev_mt6835_t *pobj, mt6835_reg_enum_t reg, uint8_t data)
{
	uint8_t rxdata[3] = {0, 0, 0xFF};
	uint16_t tx = MT6835_WRITEEEPROM | data;
	(void)reg;

	dev_mt6835_csn_ctrl(pobj, MT6835_LOW);
	drv_spi_transfer(mt6835_list[pobj->id].spi_num, (uint8_t *)&tx, rxdata, 3, MT6835_SPI_TIMEOUT);
	dev_mt6835_csn_ctrl(pobj, MT6835_HIGH);

	return (rxdata[2] == 0x55);
}

/*
 * @brief 读取21bit原始角度值(按running_dir做正反向)
 * @note  TODO: rxdata声明为uint16_t但驱动按字节填充, (rxdata[1]<<5)|(rxdata[2]>>11)
 *        的字节身位疑似有误, 需对照手册ANGLE3..1寄存器布局并在硬件上验证
 */
static uint32_t dev_mt6835_get_raw(struct dev_mt6835 *pobj)
{
	uint16_t rxdata[3] = {0};
	mt6835_read_reg(pobj, MT6835_REG_ANGLE3, (uint8_t *)rxdata);
	pobj->raw = ((uint32_t)(rxdata[1] << 5) | (rxdata[2] >> 11)) & MT6835_ANGLE_MASK;

	if (pobj->running_dir > 1) /* 反向 */
	{
		pobj->raw = MT6835_ANGLE_MASK - pobj->raw;
	}
	return pobj->raw;
}

/* 由21bit原始值算机械角度, 去偏移并归一化到[0,360) */
static float dev_mt6835_get_machAngle(struct dev_mt6835 *pobj)
{
	float angle_org = (float)pobj->raw / MT6835_ANGLE_RESOLUTION * 360.0F;
	float angle = angle_org - pobj->offset;

	pobj->mech_angle_org = angle_org;
	pobj->mech_angle_remove_off = angle;
	pobj->mechanical_angle = (angle >= 0.0F) ? angle : (angle + 360.0F);
	return pobj->mechanical_angle;
}


/* 读零点寄存器原始值(ZERO_POS2:高8位, ZERO_POS1:低4位)
 * 注: read_reg固定写3字节, 故每个接收缓冲须>=3字节, 否则越界破坏栈 */
static uint16_t mt6835_get_raw_zero_angle(dev_mt6835_t *pobj)
{
	uint8_t rx_pos2[3] = {0};
	uint8_t rx_pos1[3] = {0};
	mt6835_read_reg(pobj, MT6835_REG_ZERO_POS2, rx_pos2);
	mt6835_read_reg(pobj, MT6835_REG_ZERO_POS1, rx_pos1);
	return (uint16_t)((rx_pos2[0] << 4) | (rx_pos1[0] >> 4));
}

/* 读零点角度(°) */
static float mt6835_get_zero_angle(dev_mt6835_t *pobj)
{
	return (float)mt6835_get_raw_zero_angle(pobj) * MT6835_ZERO_REG_STEP;
}

/*
 * @brief 写零点角度, 成功返回true(两个寄存器均应答0x55)
 * @note  原实现存在栈溢出: read_reg固定写3字节, 而tx_buf仅2字节, 越界1字节
 *        会破坏栈上相邻内存(如pobj指针); 且该次读取还覆盖了刚算好的高8位。
 *        现去掉可疑的读-改-写, 直接写入12bit零点值。
 *        ZERO_POS1低4位为保留位, 此处写0; 若手册要求保留原值需改回读-改-写,
 *        并使用>=3字节缓冲, 需在硬件上验证。
 */
static bool mt6835_set_zero_angle(dev_mt6835_t *pobj, float rad)
{
	uint16_t angle = (uint16_t)roundf(rad * MT6835_RAD2DEG / MT6835_ZERO_REG_STEP);
	if (angle > 0xFFF)
	{
		return false;
	}

	uint8_t zero_pos2 = (uint8_t)(angle >> 4);		   /* 高8位 */
	uint8_t zero_pos1 = (uint8_t)((angle & 0x0F) << 4); /* 低4位置于bit[7:4], bit[3:0]保留位写0 */

	bool ok = mt6835_write_reg(pobj, MT6835_REG_ZERO_POS2, zero_pos2);
	ok = mt6835_write_reg(pobj, MT6835_REG_ZERO_POS1, zero_pos1) && ok;
	return ok;
}

static void dev_mt6835_set_offset(struct dev_mt6835 *pobj, float offset)
{
	pobj->offset = offset;
}

static void dev_mt6835_set_dir(struct dev_mt6835 *pobj, int dir)
{
	pobj->running_dir = dir;
}

/* 数据处理: 读原始值→算机械角度 */
static void dev_mt6835_handle(struct dev_mt6835 *pobj)
{
	assert_report(pobj != NULL);
	dev_mt6835_get_raw(pobj);
	dev_mt6835_get_machAngle(pobj);
}

void dev_mt6835_init(dev_mt6835_t *pobj, mt6835_id_e dev_id)
{
	assert_report(pobj != NULL);
	memset(pobj, 0, sizeof(dev_mt6835_t));
	pobj->id = dev_id;

	pobj->update = dev_mt6835_handle;
	pobj->get_mechanical_angle = dev_mt6835_get_machAngle;
	pobj->get_mechanical_angle_raw = dev_mt6835_get_raw;
	pobj->set_offset = dev_mt6835_set_offset;
	pobj->set_dir = dev_mt6835_set_dir;
	pobj->set_zero_angle = mt6835_set_zero_angle;
	pobj->get_raw_zero_angle = mt6835_get_zero_angle;
}
#endif /* USE_DEV_MT6835 */

