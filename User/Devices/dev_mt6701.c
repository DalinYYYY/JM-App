/**
 * @file        dev_mt6701.c
 * @brief       MT6701磁编码器(14bit SSI/SPI): 角度/磁场状态/CRC校验/零点/方向
 *
 * @author      Dalin (dalin@robot.com)
 * @version     1.2
 * @date        2026-06-17
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @par 修改日志:
 * | 日期       | 版本 | 作者   | 修改内容                                   |
 * |------------|------|--------|--------------------------------------------|
 * | 2026-06-17 | 1.0  | Dalin  | 初始创建                                   |
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 */
#include "dev_mt6701.h"

#if defined(USE_DEV_MT6701)
#include "drv_spi.h"
#include "drv_gpio.h"
#include "assert_report.h"
#include <math.h>

#define MT6701_SPI DRV_SPI1 /* MT6701挂载的SPI外设 */

/* CRC6校验表(手册多项式 X^6+X+1) */
static const uint8_t tableCRC6[64] = {
	0x00, 0x03, 0x06, 0x05, 0x0C, 0x0F, 0x0A, 0x09,
	0x18, 0x1B, 0x1E, 0x1D, 0x14, 0x17, 0x12, 0x11,
	0x30, 0x33, 0x36, 0x35, 0x3C, 0x3F, 0x3A, 0x39,
	0x28, 0x2B, 0x2E, 0x2D, 0x24, 0x27, 0x22, 0x21,
	0x23, 0x20, 0x25, 0x26, 0x2F, 0x2C, 0x29, 0x2A,
	0x3B, 0x38, 0x3D, 0x3E, 0x37, 0x34, 0x31, 0x32,
	0x13, 0x10, 0x15, 0x16, 0x1F, 0x1C, 0x19, 0x1A,
	0x0B, 0x08, 0x0D, 0x0E, 0x07, 0x04, 0x01, 0x02};

/* 角度归一化到 [0,360) */
static inline float mt6701_norm360(float deg)
{
	deg = fmodf(deg, 360.0F);
	return (deg < 0.0F) ? (deg + 360.0F) : deg;
}

/* SPI读取len字节(CS由调用方手动控制, 故spiDrv不带cs) */
static int8_t mt6701_spi_read(struct dev_mt6701 *pobj, uint8_t *data, uint16_t len)
{
	assert_report(pobj != NULL);
	spiDrv_t dev = {.hspi = MT6701_SPI, .cs = {0}}; /* cs.gpiox=DRV_GPIO_INIT: 驱动不操作CS */
	return (int8_t)drv_spi_read(dev, data, len, 1000);
}

/* 片选控制 */
static void dev_mt6701_csn_ctrl(struct dev_mt6701 *pobj, mt6701State_e state)
{
	drv_gpio_write(mt6701_list[pobj->id].csn, (drvPinState_e)state);
}

/* 设置编码器方向 */
static void mt6701_set_dir(struct dev_mt6701 *pobj, mt6701_dir_e dir)
{
	assert_report(pobj != NULL);
	if (dir == MT6701_DIR_CW || dir == MT6701_DIR_CCW)
	{
		pobj->dir = dir;
	}
}

/* 获取编码器方向 */
static mt6701_dir_e mt6701_get_dir(struct dev_mt6701 *pobj)
{
	assert_report(pobj != NULL);
	return pobj->dir;
}

/* 解析磁场状态 Mg[3:0]: Mg[1:0]基础状态, Mg2按压, Mg3超速(叠加) */
static uint8_t mt6701_parse_mg_state(uint8_t mg_raw)
{
	uint8_t state;
	switch (mg_raw & 0x03)
	{
		case 0: state = MT6701_MG_NORMAL; break;
		case 1: state = MT6701_MG_TOO_STRONG; break;
		case 2: state = MT6701_MG_TOO_WEAK; break;
		default: state = MT6701_MG_INVALID; break;
	}
	if (mg_raw & 0x04)
	{
		state |= MT6701_MG_BUTTON_PRESSED;
	}
	if (mg_raw & 0x08)
	{
		state |= MT6701_MG_OVERSPEED;
	}
	return state;
}

/*
 * @brief CRC6校验(手册多项式 X^6+X+1)
 * @param data_word : 24位字, 布局 [23:10]角度 [9:6]Mg [5:0]CRC
 * @return 0=通过, 1=失败
 */
static uint8_t mt6701_crc6_check(uint32_t data_word)
{
	uint32_t crc_data = (data_word >> 6) & 0x3FFFF; /* 参与校验的18位: 角度14+Mg4 */
	uint8_t crc_calc = 0;

	for (int i = 17; i >= 0; i--)
	{
		crc_calc = tableCRC6[((crc_calc << 1) | ((crc_data >> i) & 0x01)) & 0x3F];
	}
	return (crc_calc == (data_word & 0x3F)) ? 0 : 1;
}

/*
 * @brief  读取mt6701原始数据（角度+磁场+CRC）
 * @param pobj    mt6701对象
 */
/*
 * @brief 读取并解析mt6701原始帧(14bit角度 + 4bit磁场 + 6bit CRC)
 * @note  SSI帧MSB先出: buf[0]=[D13..D6] buf[1]=[D5..D0,Mg3,Mg2] buf[2]=[Mg1,Mg0,CRC5..CRC0]
 */
static void dev_mt6701_get_raw(struct dev_mt6701 *pobj)
{
	int8_t ret;

	dev_mt6701_csn_ctrl(pobj, MT6701_LOW);
	ret = mt6701_spi_read(pobj, pobj->raw_buf, 3);
	dev_mt6701_csn_ctrl(pobj, MT6701_HIGH);

	if (ret != 0)
	{
		pobj->raw = 0;
		pobj->mg_state = MT6701_MG_INVALID;
		pobj->crc_code = 0;
		pobj->crc_check = 1;
		return;
	}

	uint8_t b0 = pobj->raw_buf[0], b1 = pobj->raw_buf[1], b2 = pobj->raw_buf[2];
	uint16_t angle = (uint16_t)((b0 << 6) | (b1 >> 2)) & 0x3FFF; /* 14bit角度 */
	uint8_t mg_raw = (uint8_t)(((b1 & 0x03) << 2) | (b2 >> 6));	 /* 4bit磁场 */
	uint8_t crc = (uint8_t)(b2 & 0x3F);							 /* 6bit CRC */

	pobj->raw = angle;
	pobj->mg_state = mt6701_parse_mg_state(mg_raw);
	pobj->crc_code = crc;
	/* CRC校验字: [23:10]角度 [9:6]Mg [5:0]CRC */
	pobj->crc_check = mt6701_crc6_check(((uint32_t)angle << 10) | ((uint32_t)mg_raw << 6) | crc);
}

/* 由14bit原始值算机械角度: 去偏移→方向→归一化到[0,360) */
static void dev_mt6701_get_machAngle(struct dev_mt6701 *pobj)
{
	float raw_deg = (float)pobj->raw / MT6701_ANGLE_RESOLUTION * 360.0F;
	float angle = mt6701_norm360(raw_deg - pobj->offset);

	if (pobj->dir == MT6701_DIR_CCW)
	{
		angle = mt6701_norm360(360.0F - angle);
	}
	pobj->mech_angle_org = raw_deg; /* 未补偿原始角度 */
	pobj->mechanical_angle = angle; /* 最终机械角度 [0,360) */
}

/* 未补偿的原始机械角度(°), 供零点标定换算 */
static float mt6701_raw_deg(struct dev_mt6701 *pobj)
{
	return (float)pobj->raw / MT6701_ANGLE_RESOLUTION * 360.0F;
}

/* 设置零点角度(°): 令当前位置显示为 angle_deg, 成功返回true */
static bool mt6701_set_zero_angle(struct dev_mt6701 *pobj, float angle_deg)
{
	assert_report(pobj != NULL);
	if (angle_deg < 0.0F || angle_deg > 360.0F)
	{
		return false;
	}
	dev_mt6701_get_raw(pobj);
	pobj->offset = mt6701_norm360(mt6701_raw_deg(pobj) - angle_deg);
	return true;
}

/* 刷新并返回当前机械角度(°) */
static float mt6701_get_zero_angle(struct dev_mt6701 *pobj)
{
	assert_report(pobj != NULL);
	dev_mt6701_get_raw(pobj);
	dev_mt6701_get_machAngle(pobj);
	return pobj->mechanical_angle;
}

/* 将当前角度标定为 0° */
static void mt6701_calibrate_zero(struct dev_mt6701 *pobj)
{
	assert_report(pobj != NULL);
	dev_mt6701_get_raw(pobj);
	pobj->offset = mt6701_raw_deg(pobj);
}

/* 数据处理主函数: 读原始帧→算机械角度 */
static void dev_mt6701_handle(struct dev_mt6701 *pobj)
{
	assert_report(pobj != NULL);
	dev_mt6701_get_raw(pobj);
	dev_mt6701_get_machAngle(pobj);
}

void dev_mt6701_init(dev_mt6701_t *pobj, mt6701_id_e dev_id)
{
	assert_report(pobj != NULL);
	memset(pobj, 0, sizeof(dev_mt6701_t));

	pobj->id = dev_id;
	pobj->offset = 0.0F;
	pobj->dir = MT6701_DIR_CW;

	pobj->update = dev_mt6701_handle;
	pobj->set_zero_angle = mt6701_set_zero_angle;
	pobj->get_zero_angle = mt6701_get_zero_angle;
	pobj->calibrate_zero = mt6701_calibrate_zero;
	pobj->set_dir = mt6701_set_dir;
	pobj->get_dir = mt6701_get_dir;
}

#endif /* USE_DEV_MT6701 */
