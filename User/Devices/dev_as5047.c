/**
 * @file        dev_as5047.c
 * @brief       AS5047P 磁编码器驱动实现
 *
 * @note        SPI 时序: 16-bit data size, len=1 表示一次 16-bit 传输。
 *   帧格式: bit15=parity(偶校验), bit14=EF/RW, bit13:0=data/addr
 *   parity 覆盖 bit14+bit13:0 (15bit, 不含 bit15)
 *
 *   读角度(2-transfer, 每帧独立拉 CSN):
 *     1. CSN LOW → 发 0xFFFF(读ANGLECOM命令) → CSN HIGH(上升沿锁存命令)
 *     2. CSN LOW → 发 0x0000(dummy) → 收 16-bit 角度帧 → CSN HIGH
 *   角度帧: bit15=parity, bit14=EF, bit13:0=14bit角度
 *   注意: AS5047P 靠 CSN 上升沿锁存命令并装载输出寄存器, 两帧之间必须抬 CSN。
 */
#include "dev_as5047.h"

#if defined(USE_DEV_AS5047)

#include <string.h>
#include <math.h>
#include "assert_report.h"

/* 角度归一化到 [0,360) */
static inline float as5047_norm360(float deg)
{
	deg = fmodf(deg, 360.0F);
	return (deg < 0.0F) ? (deg + 360.0F) : deg;
}

/* 偶校验计算: 对 15bit 数据(bit14 + bit13:0)计算, parity 使总数(含 parity)为偶数。
 * 输入 = frame & 0x7FFF (清 bit15, 保留 bit14 + bit13:0) */
static uint8_t calc_even_parity_15bit(uint16_t v15)
{
	uint8_t p = 0;
	for (uint8_t i = 0; i < 15; i++)
		p ^= (uint8_t)((v15 >> i) & 1U);
	return p; /* 期望的 parity 值: 使 15bit 数据 + parity = 偶数个 1 */
}

/* SPI 16-bit 收发(CS 由调用方手动控制, spiDrv_t.cs={0})
 * 返回 0=成功, 非 0=失败 */
static int as5047_spi_xfer(spiDrv_t dev, uint8_t *tx, uint8_t *rx)
{
	return drv_spi_transfer(dev, tx, rx, 1, 1000); /* len=1: 一次 16-bit 传输 */
}

/* 读取角度(2-transfer 协议)
 * ANGLECOM 地址=0x3FFF, 读命令=0xFFFF(bit14=1=read, bit13:0=0x3FFF, parity=1) */
static uint16_t as5047_read_angle(dev_as5047_t *pobj)
{
	const as5047_config_t *cfg = &as5047_list[pobj->id];
	spiDrv_t dev = {.hspi = cfg->spi_num.hspi, .cs = {0}}; /* cs={0}: 驱动不操作 CS */

	/* 用 uint16_t 存储, STM32 小端架构下 HAL 直接用 *((uint16_t*)buf) 访问, 字节序自动正确 */
	uint16_t cmd = 0xFFFF; /* 读 ANGLECOM 命令 */
	uint16_t dummy = 0x0000;
	uint16_t rx1 = 0;
	uint16_t rx2 = 0;

	/* 帧1: CSN LOW → 发读命令 → CSN HIGH(上升沿锁存命令, 装载角度到输出寄存器) */
	drv_gpio_write(cfg->csn, DRV_PIN_LOW);
	int ret1 = as5047_spi_xfer(dev, (uint8_t *)&cmd, (uint8_t *)&rx1); /* 发读命令, 收 garbage */
	drv_gpio_write(cfg->csn, DRV_PIN_HIGH);

	/* 帧2: CSN LOW → 发 dummy → 收角度帧 → CSN HIGH */
	drv_gpio_write(cfg->csn, DRV_PIN_LOW);
	int ret2 = as5047_spi_xfer(dev, (uint8_t *)&dummy, (uint8_t *)&rx2); /* 发 dummy, 收角度帧 */
	drv_gpio_write(cfg->csn, DRV_PIN_HIGH);

	if (ret1 != 0 || ret2 != 0)
	{
		if (pobj->spi_err_cnt < 0xFFFFU)
			pobj->spi_err_cnt++;
	}

	return rx2;
}

/* 读取 ERRFL 寄存器(2-transfer 协议)
 * ERRFL 地址=0x0001, 读命令=0x4001(bit14=1=read, bit13:0=0x0001)
 *   计算 parity: bit14+bit13:0 = 1 + 1 = 2 个 1 (偶), parity=0 → bit15=0 → 命令=0x4001
 * 返回值: 0xFFFF=AS5047P 无响应; 其他=ERRFL 内容(bit0=CORDIC溢出, bit2=SPI帧错误, bit3/4=磁场异常) */
static uint16_t as5047_read_errfl(dev_as5047_t *pobj)
{
	const as5047_config_t *cfg = &as5047_list[pobj->id];
	spiDrv_t dev = {.hspi = cfg->spi_num.hspi, .cs = {0}};

	uint16_t cmd = 0x4001; /* 读 ERRFL(0x0001): bit14=1=read, parity=0 */
	uint16_t dummy = 0x0000;
	uint16_t rx1 = 0;
	uint16_t rx2 = 0;

	/* 帧1: 发读命令, CSN 上升沿锁存 */
	drv_gpio_write(cfg->csn, DRV_PIN_LOW);
	as5047_spi_xfer(dev, (uint8_t *)&cmd, (uint8_t *)&rx1);
	drv_gpio_write(cfg->csn, DRV_PIN_HIGH);

	/* 帧2: 发 dummy 收 ERRFL */
	drv_gpio_write(cfg->csn, DRV_PIN_LOW);
	as5047_spi_xfer(dev, (uint8_t *)&dummy, (uint8_t *)&rx2);
	drv_gpio_write(cfg->csn, DRV_PIN_HIGH);

	return rx2;
}

static void dev_as5047_update(struct dev_as5047 *pobj)
{
	uint16_t frame = as5047_read_angle(pobj);
	pobj->last_frame = frame; /* 诊断: 保存原始帧供调试器观察 */

	/* 角度帧: bit15=parity(偶校验), bit14=EF(错误标志), bit13:0=14bit角度 */
	uint8_t parity_bit = (uint8_t)((frame >> 15) & 1U); /* bit15 = parity */
	uint8_t ef = (uint8_t)((frame >> 14) & 1U);         /* bit14 = EF */
	uint16_t angle14 = frame & 0x3FFFU;                 /* bit13:0 = 角度 */

	/* 偶校验: 计算 bit14 + bit13:0 (15bit, 不含 bit15)
	 * mask = 0x7FFF = bit14 + bit13:0 (bit15 清零) */
	uint8_t expect_p = calc_even_parity_15bit(frame & 0x7FFFU);
	pobj->parity_err = (parity_bit != expect_p) ? 1U : 0U;
	pobj->ef = ef;

	if (pobj->parity_err || ef)
	{
		if (pobj->err_cnt < 0xFFFFU)
			pobj->err_cnt++;
		return; /* 坏帧不更新角度 */
	}
	pobj->err_cnt = 0;
	pobj->raw = angle14;

	/* 原始角度 → 去偏移 → 方向 → 归一化 */
	float deg = (float)angle14 * 360.0F / (float)AS5047_ANGLE_RESOLUTION;
	pobj->mech_angle_org = deg;
	deg = as5047_norm360(deg - pobj->offset);
	if (pobj->dir == AS5047_DIR_CCW)
		deg = as5047_norm360(360.0F - deg);
	pobj->mechanical_angle = deg;
}

static void dev_as5047_set_dir(struct dev_as5047 *pobj, as5047_dir_e dir)
{
	assert_report(pobj != NULL);
	if (dir == AS5047_DIR_CW || dir == AS5047_DIR_CCW)
		pobj->dir = dir;
}

static as5047_dir_e dev_as5047_get_dir(struct dev_as5047 *pobj)
{
	assert_report(pobj != NULL);
	return pobj->dir;
}

void dev_as5047_init(dev_as5047_t *pobj, as5047_id_e dev_id)
{
	assert_report(pobj != NULL);
	assert_report(dev_id < AS5047_ID_MAX);
	memset(pobj, 0, sizeof(dev_as5047_t));
	pobj->id = dev_id;
	pobj->dir = AS5047_DIR_CW;
	pobj->offset = 0.0F;
	pobj->update = dev_as5047_update;
	pobj->set_dir = dev_as5047_set_dir;
	pobj->get_dir = dev_as5047_get_dir;

	/* 上电诊断: 读取 ERRFL 寄存器, 确认 AS5047P 是否响应
	 * errfl=0xFFFF → AS5047P 无响应(MISO 悬空/未上电/虚焊)
	 * errfl=其他值 → AS5047P 正常响应, 检查 bit3/bit4 确认磁场状态 */
	pobj->errfl = as5047_read_errfl(pobj);
}

#endif /* USE_DEV_AS5047 */
