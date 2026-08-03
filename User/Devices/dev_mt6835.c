/**
 * @file        dev_mt6835.c
 * @brief       MT6835磁编码器(21bit SPI): 角度/零点读写(寄存器/EEPROM)/方向
 *
 * @author      Dalin (dalinyy@163.com)
 * @version     1.3
 * @date        2026-07-21
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @par 修改日志:
 * | 日期       | 版本 | 作者   | 修改内容                                   |
 * |------------|------|--------|--------------------------------------------|
 * | 2026-06-11 | 1.0  | Dalin  | 分离角度获取和角度转化                     |
 * | 2026-06-17 | 1.1  | Dalin  | 复用共享配置表; 去重去死码去魔数; 补init漏绑 |
 * | 2026-07-21 | 1.2  | Dalin  | 修复SPI字节流时序/21bit角度拼接/写reg命令字/CSN初始电平/方向约定 |
 * | 2026-07-21 | 1.3  | Dalin  | 严格按MT6835_Rev.1.3中文手册: 所有单字节读写改为3字节(24bit/帧); 移除普通写的ACK检查; 修正EEPROM ACK位置; 修正零点寄存器bit分布 |
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 *
 * @note        MT6835 SPI 协议(摘自 MT6835_Rev.1.3 中文手册):
 *              - SPI 模式 3 (CPOL=1, CPHA=1), MSB first, 最高 16MHz
 *              - 数据帧 = 24bit/帧 = 3 字节 (4bit命令 + 12bit地址 + 8bit数据)
 *              - SCK 空闲高电平, CSN 低有效, MISO 空闲 Hi-Z
 *              - CSN 下降沿锁存角度寄存器(0x003~0x006), 并激活 SPI 通信
 *              - T_L (CSN下降沿→首个SCK下降沿) ≥ 100ns
 *
 *              命令编码 (C3~C0):
 *              - 0011 (0x30): 读寄存器 READ
 *              - 0110 (0x60): 写寄存器 WRITE (普通写无 ACK)
 *              - 0101 (0x50): 自动设零点 SetZeroPoint (返回 0x55 ACK)
 *              - 1010 (0xA0): 连续读角度 ContinuousRead
 *              - 1100 (0xC0): 烧录 EEPROM (返回 0x55 ACK)
 *
 *              时序(24bit/帧, 严格 3 字节):
 *              - 读寄存器: MOSI=[0x30|A11~A8][A7~A0][dummy], MISO=[Hi-Z][Hi-Z][DO7~DO0]
 *                数据在第 3 字节 rx[2] 返回
 *              - 写寄存器: MOSI=[0x60|A11~A8][A7~A0][DI7~DI0], MISO 全程 Hi-Z (无 ACK)
 *              - 连续读:   MOSI=[0xA0][0x03][dummy×4], MISO=[?][?][A3][A2][A1][CRC]
 *                数据从第 3 字节开始, 每 4 字节一组(0x003~0x006)
 *
 * @note        21bit 角度寄存器布局(手册 0x003~0x005):
 *              0x003: ANGLE[20:13] (8bit, 全部有效)
 *              0x004: ANGLE[12:5]  (8bit, 全部有效)
 *              0x005: bit[7:3]=ANGLE[4:0], bit[2:0]=STATUS[2:0] (3bit 状态, 需剔除)
 *              拼接公式: raw = (rx[2]<<13) | (rx[3]<<5) | (rx[4]>>3)
 *
 * @note        零点寄存器布局(手册 0x009~0x00A):
 *              0x009 (ZERO_POS2): ZERO_POS[11:4] (高 8bit)
 *              0x00A (ZERO_POS1): bit[7:4]=ZERO_POS[3:0] (低 4bit), bit[3]=Z_EDGE, bit[2:0]=Z_PUL_WID
 *              零点分辨率 = 360°/4096 ≈ 0.0879°/LSB
 */
#include "dev_mt6835.h"

#if defined(USE_DEV_MT6835)
#include "assert_report.h"
#include "drv_spi.h"
#include "drv_gpio.h"
#include <math.h>

#define MT6835_SPI_TIMEOUT (200u) /* SPI收发超时, ms */

/* MT6835 数据帧固定 24bit = 3 字节 (手册规定, 不可改动) */
#define MT6835_FRAME_LEN          (3u)
/* 连续读角度: 命令 2 字节 + 4 字节数据(ANGLE3/2/1/CRC) = 6 字节 */
#define MT6835_CONT_READ_FRAME_LEN (6u)

/* 片选控制 */
static void dev_mt6835_csn_ctrl(struct dev_mt6835 *pobj, mt6835State_e state)
{
	drv_gpio_write(mt6835_list[pobj->id].csn, (drvPinState_e)state);
}

/*
 * @brief 读单字节寄存器 (严格 24bit/帧 = 3 字节传输)
 * @note  手册时序: MOSI 发送 3 字节 = [0x30|A11~A8][A7~A0][dummy]
 *        MISO 返回: 前 2 字节 Hi-Z, 第 3 字节为寄存器数据
 *        MT6835 在收到完整 16bit 命令后, 在第 3 字节(dummy)期间输出数据
 *        注意: 不可多发 dummy 字节, 否则会被 MT6835 当作下一帧命令, 导致状态机错乱
 * @param[out] value 寄存器值
 * @return true=读取成功，false=SPI 传输失败
 */
static bool mt6835_read_reg(struct dev_mt6835 *pobj, mt6835_reg_enum_t reg, uint8_t *value)
{
	uint8_t tx[MT6835_FRAME_LEN] = {
		(uint8_t)((MT6835_READ | reg) >> 8),  /* 0x30 | A[11:8] */
		(uint8_t)(MT6835_READ | reg),          /* A[7:0] */
		0xFFu,                                  /* dummy, MT6835 在此字节输出数据 */
	};
	uint8_t rx[MT6835_FRAME_LEN] = {0};
	int status;

	dev_mt6835_csn_ctrl(pobj, MT6835_LOW);
	status = drv_spi_transfer(mt6835_list[pobj->id].spi_num, tx, rx,
	                          MT6835_FRAME_LEN, MT6835_SPI_TIMEOUT);
	dev_mt6835_csn_ctrl(pobj, MT6835_HIGH);

	if (status != DRV_EOK || value == NULL)
	{
		pobj->read_error_count++;
		return false;
	}
	*value = rx[2]; /* 数据在第 3 字节 */
	return true;
}

/*
 * @brief 写单字节寄存器 (严格 24bit/帧 = 3 字节传输)
 * @note  手册时序: MOSI 发送 3 字节 = [0x60|A11~A8][A7~A0][DI7~DI0]
 *        MISO 全程 Hi-Z, 普通写寄存器不返回 ACK (手册图-20 明确说明)
 *        注意: 不可多发 dummy 字节, 否则会被 MT6835 当作下一帧命令
 *        如需验证写入是否成功, 须另行读回寄存器比较
 */
static bool mt6835_write_reg(dev_mt6835_t *pobj, mt6835_reg_enum_t reg, uint8_t data)
{
	uint8_t tx[MT6835_FRAME_LEN] = {
		(uint8_t)((MT6835_WRITE | reg) >> 8),  /* 0x60 | A[11:8] */
		(uint8_t)(MT6835_WRITE | reg),          /* A[7:0] */
		data,                                    /* DI7~DI0 */
	};
	uint8_t dummy_rx[MT6835_FRAME_LEN] = {0}; /* HAL_SPI_TransmitReceive 不接受 NULL rx, 须提供 dummy 缓冲 */
	int status;

	dev_mt6835_csn_ctrl(pobj, MT6835_LOW);
	status = drv_spi_transfer(mt6835_list[pobj->id].spi_num, tx, dummy_rx,
	                          MT6835_FRAME_LEN, MT6835_SPI_TIMEOUT);
	dev_mt6835_csn_ctrl(pobj, MT6835_HIGH);
	return status == DRV_EOK;
}

/*
 * @brief 烧录 EEPROM (将所有寄存器值永久写入 EEPROM)
 * @note  手册时序: MOSI 发送 3 字节 = [0xC0][0x00][dummy]
 *        (烧录命令地址字段全 0, 不针对单个寄存器)
 *        MISO 在第 3 字节返回 0x55 ACK 表示接收成功
 *        烧录后须等待至少 6 秒再断电, 否则数据可能丢失
 * @return true=ACK 0x55(接收成功), false=未收到 ACK
 */
static bool mt6835_write_eeprom(dev_mt6835_t *pobj)
{
	uint8_t tx[MT6835_FRAME_LEN] = {
		(uint8_t)(MT6835_WRITEEEPROM >> 8), /* 0xC0 */
		0x00u,                               /* 地址字段全 0 */
		0xFFu,                               /* dummy, MT6835 在此字节输出 ACK */
	};
	uint8_t rx[MT6835_FRAME_LEN] = {0};

	dev_mt6835_csn_ctrl(pobj, MT6835_LOW);
	drv_spi_transfer(mt6835_list[pobj->id].spi_num, tx, rx, MT6835_FRAME_LEN, MT6835_SPI_TIMEOUT);
	dev_mt6835_csn_ctrl(pobj, MT6835_HIGH);

	return (rx[2] == 0x55u); /* ACK 在第 3 字节 */
}

/*
 * @brief 自动设置零点 (将当前角度写入零点寄存器)
 * @note  手册时序: MOSI 发送 3 字节 = [0x50][0x00][dummy]
 *        (设零点命令地址字段全 0)
 *        MISO 在第 3 字节返回 0x55 ACK 表示接收成功
 *        注意: 此命令仅写入寄存器 RAM, 断电丢失; 须额外调用 mt6835_write_eeprom 永久保存
 * @return true=ACK 0x55(接收成功), false=未收到 ACK
 */
static bool mt6835_set_zero_point(dev_mt6835_t *pobj)
{
	uint8_t tx[MT6835_FRAME_LEN] = {
		(uint8_t)(MT6835_SETZEROPOINT >> 8), /* 0x50 */
		0x00u,                                /* 地址字段全 0 */
		0xFFu,                                /* dummy, MT6835 在此字节输出 ACK */
	};
	uint8_t rx[MT6835_FRAME_LEN] = {0};

	dev_mt6835_csn_ctrl(pobj, MT6835_LOW);
	drv_spi_transfer(mt6835_list[pobj->id].spi_num, tx, rx, MT6835_FRAME_LEN, MT6835_SPI_TIMEOUT);
	dev_mt6835_csn_ctrl(pobj, MT6835_HIGH);

	return (rx[2] == 0x55u); /* ACK 在第 3 字节 */
}

/*
 * @brief 读取 21bit 原始角度值 (3 次独立单字节读, 按运行方向做正反向)
 * @note  使用 3 次独立的单字节读寄存器操作, 分别读取 ANGLE3/2/1。
 *        相比连续读模式, 单字节读更可靠, 不依赖连续读时序的复杂性。
 *
 *        手册角度寄存器布局:
 *          0x003 (ANGLE3): ANGLE[20:13] (8bit, 全部有效)
 *          0x004 (ANGLE2): ANGLE[12:5]  (8bit, 全部有效)
 *          0x005 (ANGLE1): bit[7:3]=ANGLE[4:0], bit[2:0]=STATUS[2:0] (需剔除)
 *
 *        拼接公式(手册权威):
 *          raw = (angle3 << 13) | (angle2 << 5) | (angle1 >> 3)
 *          其中 angle1>>3 用于剔除低 3bit STATUS, 保留高 5bit ANGLE[4:0]
 *
 *        注意: 3 次读取之间角度可能变化, 但对静止电机无影响;
 *        对运动电机会有微小不一致, 可通过快速连续读取减小误差。
 *
 * @return 21bit 原始角度值 [0, 2097151]
 */
static uint32_t dev_mt6835_get_raw(struct dev_mt6835 *pobj)
{
	uint8_t angle3;
	uint8_t angle2;
	uint8_t angle1;

	if (!mt6835_read_reg(pobj, MT6835_REG_ANGLE3, &angle3) ||
		!mt6835_read_reg(pobj, MT6835_REG_ANGLE2, &angle2) ||
		!mt6835_read_reg(pobj, MT6835_REG_ANGLE1, &angle1))
	{
		return pobj->raw; /* 任一帧失败均保留上一有效角度 */
	}

	/* 21bit 角度拼接 (手册权威公式) */
	uint32_t raw = ((uint32_t)angle3 << 13)
	             | ((uint32_t)angle2 << 5)
	             | ((uint32_t)angle1 >> 3);
	pobj->raw = raw & MT6835_ANGLE_MASK;

	if (pobj->running_dir < 0) /* 反向: -1/1 约定, <0 表示反向 */
	{
		pobj->raw = MT6835_ANGLE_MASK - pobj->raw;
	}
	return pobj->raw;
}

/* 由 21bit 原始值算机械角度, 去偏移并归一化到 [0,360) */
static float dev_mt6835_get_machAngle(struct dev_mt6835 *pobj)
{
	float angle_org = (float)pobj->raw / MT6835_ANGLE_RESOLUTION * 360.0F;
	float angle = angle_org - pobj->offset;

	pobj->mech_angle_org = angle_org;
	pobj->mech_angle_remove_off = angle;
	pobj->mechanical_angle = (angle >= 0.0F) ? angle : (angle + 360.0F);
	return pobj->mechanical_angle;
}

/*
 * @brief 读零点寄存器原始值 (12bit)
 * @note  手册布局:
 *        0x009 (ZERO_POS2): ZERO_POS[11:4] (高 8bit)
 *        0x00A (ZERO_POS1): bit[7:4]=ZERO_POS[3:0] (低 4bit), bit[3:0]=Z_EDGE+Z_PUL_WID
 *        拼接: ZERO_POS[11:0] = (pos2 << 4) | (pos1 >> 4)
 */
static uint16_t mt6835_get_raw_zero_angle(dev_mt6835_t *pobj)
{
	uint8_t pos2;
	uint8_t pos1;
	if (!mt6835_read_reg(pobj, MT6835_REG_ZERO_POS2, &pos2) ||
		!mt6835_read_reg(pobj, MT6835_REG_ZERO_POS1, &pos1))
	{
		return 0u;
	}
	return (uint16_t)(((uint16_t)pos2 << 4) | (pos1 >> 4));
}

/* 读零点角度 (°) */
static float mt6835_get_zero_angle(dev_mt6835_t *pobj)
{
	return (float)mt6835_get_raw_zero_angle(pobj) * MT6835_ZERO_REG_STEP;
}

/*
 * @brief 写零点角度 (写寄存器 RAM, 不烧录 EEPROM)
 * @note  手册: 普通写寄存器(0110)无 ACK, 故采用"写后读回验证"判断成功
 *        如需永久保存, 须另行调用 mt6835_write_eeprom
 *        ZERO_POS[11:0] = (pos2 << 4) | (pos1 >> 4)
 *        pos2 = ZERO_POS[11:4], pos1[7:4] = ZERO_POS[3:0], pos1[3:0] 保留位写 0
 * @return true=读回验证一致, false=验证失败或角度超范围
 */
static bool mt6835_set_zero_angle(dev_mt6835_t *pobj, float rad)
{
	uint16_t angle = (uint16_t)roundf(rad * MT6835_RAD2DEG / MT6835_ZERO_REG_STEP);
	if (angle > 0xFFFu)
	{
		return false;
	}

	uint8_t zero_pos2 = (uint8_t)(angle >> 4);           /* ZERO_POS[11:4] */
	uint8_t zero_pos1 = (uint8_t)((angle & 0x0Fu) << 4); /* ZERO_POS[3:0] 置于 bit[7:4], bit[3:0] 保留位写 0 */

	if (!mt6835_write_reg(pobj, MT6835_REG_ZERO_POS2, zero_pos2) ||
		!mt6835_write_reg(pobj, MT6835_REG_ZERO_POS1, zero_pos1))
	{
		return false;
	}

	/* 写后读回验证 (普通写无 ACK, 只能靠读回判断) */
	uint8_t rb_pos2;
	uint8_t rb_pos1;
	if (!mt6835_read_reg(pobj, MT6835_REG_ZERO_POS2, &rb_pos2) ||
		!mt6835_read_reg(pobj, MT6835_REG_ZERO_POS1, &rb_pos1))
	{
		return false;
	}
	return (rb_pos2 == zero_pos2) && ((rb_pos1 & 0xF0u) == zero_pos1);
}

static void dev_mt6835_set_offset(struct dev_mt6835 *pobj, float offset)
{
	pobj->offset = offset;
}

/*
 * @brief 设置编码器方向
 * @param dir 方向: 1=CW(正向), -1=CCW(反向), 与项目其他编码器约定一致
 */
static void dev_mt6835_set_dir(struct dev_mt6835 *pobj, int dir)
{
	pobj->running_dir = (dir < 0) ? -1 : 1;
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
	pobj->running_dir = 1;   /* 默认正向(CW), 与项目 -1/1 约定一致 */

	/* CSN 上电默认拉高, 避免 MT6835 误以为被选中导致首帧失步
	 * 手册: CSN 下降沿激活 SPI 通信, 空闲时须保持高电平
	 * 原理图未外接上拉电阻, 这里靠 MCU 推挽输出高电平保证总线空闲 */
	dev_mt6835_csn_ctrl(pobj, MT6835_HIGH);

	pobj->update = dev_mt6835_handle;
	pobj->get_mechanical_angle = dev_mt6835_get_machAngle;
	pobj->get_mechanical_angle_raw = dev_mt6835_get_raw;
	pobj->set_offset = dev_mt6835_set_offset;
	pobj->set_dir = dev_mt6835_set_dir;
	pobj->set_zero_angle = mt6835_set_zero_angle;
	pobj->get_raw_zero_angle = mt6835_get_zero_angle;
}
#endif /* USE_DEV_MT6835 */
