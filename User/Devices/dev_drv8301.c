/**
 * @file        dev_drv8301.c
 * @brief       DRV8301 设备对象实现
 *
 * @note        SPI 16-bit, 2-transfer 读协议(同 AS5047P):
 *   读: CSN LOW → 发读命令(收 garbage) → 发 dummy(收数据) → CSN HIGH
 *   写: CSN LOW → 发写命令(收 status) → CSN HIGH
 *   len=1 表示一次 16-bit 传输(SPI data size=16bit)。
 */
#include "dev_drv8301.h"

#if defined(USE_DEV_DRV8301)

#include <string.h>
#include "assert_report.h"

dev_drv8301_t g_dev_drv8301;

/* 组装命令字
 * DRV8301 SPI 帧: bit15=R/W(0=write), bit14:12=addr, bit11:0=data(12-bit) */
static uint16_t make_write_cmd(uint8_t addr, uint16_t data)
{
	return ((uint16_t)addr << 12) | (data & 0x0FFFU);
}
static uint16_t make_read_cmd(uint8_t addr)
{
	return 0x8000U | ((uint16_t)addr << 12);
}

/* SPI 16-bit 收发(CS 由调用方手动控制)
 * 用 uint16_t 存储, STM32 小端架构下 HAL 用 *((uint16_t*)buf) 访问, 字节序自动正确 */
static void drv8301_spi_xfer(spiDrv_t dev, uint16_t cmd, uint16_t *rx)
{
	uint16_t tx = cmd;
	uint16_t r  = 0;
	drv_spi_transfer(dev, (uint8_t *)&tx, (uint8_t *)&r, 1, 1000); /* len=1: 一次 16-bit */
	*rx = r;
}

static void dev_drv8301_init_impl(struct dev_drv8301 *pobj)
{
	assert_report(pobj != NULL);
	const drv8301_config_t *cfg = &drv8301_list[pobj->id];
	spiDrv_t dev = {.hspi = cfg->spi_num.hspi, .cs = {0}};

	/* 写 CTRL1 */
	pobj->ctrl1 = cfg->ctrl1_value;
	uint16_t cmd = make_write_cmd(DRV8301_REG_CTRL1, pobj->ctrl1);
	uint16_t rx;
	drv_gpio_write(cfg->nscs, DRV_PIN_LOW);
	drv8301_spi_xfer(dev, cmd, &rx);
	drv_gpio_write(cfg->nscs, DRV_PIN_HIGH);

	/* 写 CTRL2 */
	pobj->ctrl2 = cfg->ctrl2_value;
	cmd = make_write_cmd(DRV8301_REG_CTRL2, pobj->ctrl2);
	drv_gpio_write(cfg->nscs, DRV_PIN_LOW);
	drv8301_spi_xfer(dev, cmd, &rx);
	drv_gpio_write(cfg->nscs, DRV_PIN_HIGH);

	/* 回读 CTRL1 验证 SPI 写入成功
	 * DRV8301 上电默认 CTRL1 的 DC_CAL=1(校准模式), SO1/SO2 输出固定电压,
	 * 电流采样恒为 0。若 SPI 写入失败(默认值未覆盖), 电流采样永远为 0。
	 * readback 存入 pobj->ctrl1_readback 供调试器观察:
	 *   0x003C = 写入成功(GAIN=40V/V, DC_CAL=0)
	 *   其他值 = SPI 写入失败, 需检查 nSCS/SPI3 接线 */
	pobj->ctrl1_readback = pobj->read_reg(pobj, DRV8301_REG_CTRL1);
}

static uint16_t dev_drv8301_read_reg(struct dev_drv8301 *pobj, uint8_t addr)
{
	const drv8301_config_t *cfg = &drv8301_list[pobj->id];
	spiDrv_t dev = {.hspi = cfg->spi_num.hspi, .cs = {0}};
	uint16_t rx1, rx2;

	/* 2-transfer 读: 发读命令 → 发 dummy 收数据 */
	drv_gpio_write(cfg->nscs, DRV_PIN_LOW);
	drv8301_spi_xfer(dev, make_read_cmd(addr), &rx1);
	drv8301_spi_xfer(dev, 0x0000U, &rx2);
	drv_gpio_write(cfg->nscs, DRV_PIN_HIGH);

	return rx2 & 0x07FFU;
}

static void dev_drv8301_write_reg(struct dev_drv8301 *pobj, uint8_t addr, uint16_t val)
{
	const drv8301_config_t *cfg = &drv8301_list[pobj->id];
	spiDrv_t dev = {.hspi = cfg->spi_num.hspi, .cs = {0}};
	uint16_t rx;

	drv_gpio_write(cfg->nscs, DRV_PIN_LOW);
	drv8301_spi_xfer(dev, make_write_cmd(addr, val), &rx);
	drv_gpio_write(cfg->nscs, DRV_PIN_HIGH);
}

static uint8_t dev_drv8301_get_fault(struct dev_drv8301 *pobj)
{
	const drv8301_config_t *cfg = &drv8301_list[pobj->id];
	/* nFAULT 低有效: 0=故障, 1=正常 */
	pobj->fault = (drv_gpio_read(cfg->nfault) == DRV_PIN_LOW) ? 1U : 0U;
	return pobj->fault;
}

void dev_drv8301_init(dev_drv8301_t *pobj, drv8301_id_e id)
{
	assert_report(pobj != NULL);
	assert_report(id < DRV8301_ID_MAX);
	memset(pobj, 0, sizeof(dev_drv8301_t));
	pobj->id = id;
	pobj->init      = dev_drv8301_init_impl;
	pobj->read_reg  = dev_drv8301_read_reg;
	pobj->write_reg = dev_drv8301_write_reg;
	pobj->get_fault = dev_drv8301_get_fault;
}

#endif /* USE_DEV_DRV8301 */
