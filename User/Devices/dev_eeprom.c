/**
 * @file        dev_eeprom.c
 * @brief 		读写片外 EEPROM 存储设备 (兼容 AT24C16 / MB85RC16)
 *
 * @author      --
 * @version     1.0
 * @date        2026-06-26
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 *
 * @par 修改日志:
 * | 日期       | 版本 | 作者   | 修改内容   |
 * |------------|------|--------|------------|
 * | 2026-06-26    | 1.0  | -- | 初始创建   |
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 * @note        由 MB85RC16(FRAM) 适配 AT24C16(EEPROM):
 *              - AT24C16 需按 16B 页分页写 + ACK polling 等待写周期 tWR;
 *              - 块选择位(A2A1A0)在器件地址中, 按 256B 块切分自动切换器件地址;
 *              - FRAM 无页限制/写周期, 复用同一驱动(等待立即返回)。
 */

#include "dev_eeprom.h"

#if defined(USE_DEV_EEPROM)
#include "assert_report.h"

/* 默认设备实例 */
dev_eeprom_t dev_eeprom;

/* 块大小: 8bit 内部地址 = 256B/块, 16bit = 65536B/块(单块) */
#define DEV_EEPROM_BLOCK_8BIT       (0x0100U)
#define DEV_EEPROM_BLOCK_16BIT      (0x10000U)
/* ACK polling 最大重试次数 (写周期典型 5ms, 每次重试即一次快速 NACK 检测) */
#define DEV_EEPROM_WRITE_RETRY_MAX  (50U)

/*
 * @brief  构造 I2C 传输描述: 按绝对地址计算块号并合成 8bit 器件地址
 * @param  pobj : eeprom设备句柄
 * @param  addr : 绝对地址
 * @param  word : 输出块内字地址
 * @retval 设备地址已设置的 i2cDrv_t
 */
static i2cDrv_t dev_eeprom_build_drv(const struct dev_eeprom *pobj, uint32_t addr, uint16_t *word)
{
	i2cDrv_t drv = eeprom_list[pobj->id].i2c;
	uint32_t block_size = (eeprom_list[pobj->id].addr_width == 2) ? DEV_EEPROM_BLOCK_16BIT : DEV_EEPROM_BLOCK_8BIT;
	uint32_t block = (block_size == DEV_EEPROM_BLOCK_16BIT) ? 0U : (addr / block_size);
	*word = (uint16_t)(addr % block_size);
	/* 8bit 器件地址 = 7bit基础地址<<1 | 块选择位(A2A1A0) */
	drv.dev_addr = (uint8_t)((eeprom_list[pobj->id].i2c_addr << 1) | (uint8_t)(block << 1));
	drv.mem_addr = *word;
	return drv;
}

/*
 * @brief  eeprom设备读取内存数据(跨块自动切分)
 * @param  *pobj : eeprom设备句柄
 * @param  addr : 绝对地址
 * @param  data : 读取数据的buff
 * @param  len : 读取数据的长度
 * @retval 读取结果 ：DEV_EOK正常，其他则错误
 */
int dev_eeprom_read(struct dev_eeprom *pobj, uint32_t addr, uint8_t *data, uint16_t len)
{
	if ((addr + len) > pobj->total_size) return DEV_ERROR;

	while (len > 0)
	{
		uint16_t word = 0;
		i2cDrv_t drv = dev_eeprom_build_drv(pobj, addr, &word);
		uint32_t block_size = (eeprom_list[pobj->id].addr_width == 2) ? DEV_EEPROM_BLOCK_16BIT : DEV_EEPROM_BLOCK_8BIT;
		uint16_t chunk = (uint16_t)(block_size - (addr % block_size));
		if (chunk > len) chunk = len;

		if (drv_i2c_recv(drv, data, chunk) != DRV_EOK) return DEV_ERROR;

		addr += chunk;
		data += chunk;
		len -= chunk;
	}

	return DEV_EOK;
}

/*
 * @brief  eeprom设备读取内存一字节的数据
 * @param  *pobj : eeprom设备句柄
 * @param  addr : 绝对地址
 * @param  data : 读取数据的buff
 * @retval 读取结果 ：DEV_EOK正常，其他则错误
 */
int dev_eeprom_read_byte(struct dev_eeprom *pobj, uint32_t addr, uint8_t *data)
{
	return dev_eeprom_read(pobj, addr, data, 1);
}

/*
 * @brief  等待上次写周期完成(ACK polling)
 * @param  drv : 已设置器件地址的 I2C 传输描述
 * @retval true完成/就绪, false超时
 */
static bool dev_eeprom_wait_write(i2cDrv_t drv)
{
	uint8_t retry = 0;
	while (retry++ < DEV_EEPROM_WRITE_RETRY_MAX)
	{
		if (drv_i2c_ready(drv, 1) == DRV_EOK) return true;
	}
	return false;
}

/*
 * @brief  eeprom设备写入内存数据(跨页自动分页 + 写周期等待)
 * @param  *pobj : eeprom设备句柄
 * @param  addr : 绝对地址
 * @param  data : 写入数据的data区
 * @param  len : 写入数据的长度
 * @retval 写入结果 ：DEV_EOK正常，其他则错误
 */
int dev_eeprom_write(struct dev_eeprom *pobj, uint32_t addr, const uint8_t *data, uint16_t len)
{
	if ((addr + len) > pobj->total_size) return DEV_ERROR;

	while (len > 0)
	{
		uint16_t word = 0;
		i2cDrv_t drv = dev_eeprom_build_drv(pobj, addr, &word);
		/* 一次写不超过一页, 且不跨页(EEPROM 页内地址自动回绕, 必须页对齐切分) */
		uint16_t chunk = pobj->page_size - (word % pobj->page_size);
		if (chunk > len) chunk = len;

		if (drv_i2c_send(drv, (uint8_t *)data, chunk) != DRV_EOK) return DEV_ERROR;
		if (!dev_eeprom_wait_write(drv)) return DEV_ERROR;

		addr += chunk;
		data += chunk;
		len -= chunk;
	}

	return DEV_EOK;
}

/*
 * @brief  eeprom设备写入内存一字节数据
 * @param  *pobj : eeprom设备句柄
 * @param  addr : 绝对地址
 * @param  data : 写入的数据
 * @retval 写入结果 ：DEV_EOK正常，其他则错误
 */
int dev_eeprom_write_byte(struct dev_eeprom *pobj, uint32_t addr, uint8_t data)
{
	return dev_eeprom_write(pobj, addr, &data, 1);
}

/*
 * @brief  获取eeprom设备状态，判断是否准备完成
 * @param  *pobj : eeprom设备句柄
 * @retval true正常，false错误
 */
bool dev_eeprom_is_ready(struct dev_eeprom *pobj)
{
	uint16_t word = 0;
	i2cDrv_t drv = dev_eeprom_build_drv(pobj, 0, &word);
	return (drv_i2c_ready(drv, 1) == DRV_EOK);
}

/*
 * @brief  擦除设备所有内存区
 * @param  *pobj : eeprom设备句柄
 * @retval 写入结果 ：DEV_EOK正常，其他则错误
 */
int dev_eeprom_erase_all(struct dev_eeprom *pobj)
{
	uint8_t buf[16];
	uint32_t addr = 0;

	memset(buf, 0xFF, sizeof(buf));
	while (addr < pobj->total_size)
	{
		uint16_t chunk = pobj->page_size;
		if (chunk > (pobj->total_size - addr)) chunk = (uint16_t)(pobj->total_size - addr);
		if (dev_eeprom_write(pobj, addr, buf, chunk) != DEV_EOK) return DEV_ERROR;
		addr += chunk;
	}

	return DEV_EOK;
}

/*
 * @brief  芯片自检: 多块写入回读校验(覆盖块0跨页边界/块1/末尾块)
 * @param  *pobj : eeprom设备句柄
 * @retval 校验结果 ：DEV_EOK通过，其他则错误
 */
int dev_eeprom_test(struct dev_eeprom *pobj)
{
	const uint16_t N = 32;
	uint8_t wbuf[32];
	uint8_t rbuf[32];
	uint32_t i, addr;

	/* 1) 块0 跨页写入 (addr 8..39, 跨 16B 页边界, 验证分页写) */
	addr = 8;
	for (i = 0; i < N; i++) wbuf[i] = (uint8_t)(0x10 + i);
	if (dev_eeprom_write(pobj, addr, wbuf, N) != DEV_EOK) return DEV_ERROR;
	if (dev_eeprom_read(pobj, addr, rbuf, N) != DEV_EOK) return DEV_ERROR;
	if (memcmp(wbuf, rbuf, N) != 0) return DEV_ERROR;

	/* 2) 块1 起始 (addr=256, 验证器件地址块选择位) */
	addr = 256;
	for (i = 0; i < N; i++) wbuf[i] = (uint8_t)(0x40 + i);
	if (dev_eeprom_write(pobj, addr, wbuf, N) != DEV_EOK) return DEV_ERROR;
	if (dev_eeprom_read(pobj, addr, rbuf, N) != DEV_EOK) return DEV_ERROR;
	if (memcmp(wbuf, rbuf, N) != 0) return DEV_ERROR;

	/* 3) 末尾块末尾 (addr=total_size-N, 验证最后一块) */
	addr = pobj->total_size - N;
	for (i = 0; i < N; i++) wbuf[i] = (uint8_t)(0x80 + i);
	if (dev_eeprom_write(pobj, addr, wbuf, N) != DEV_EOK) return DEV_ERROR;
	if (dev_eeprom_read(pobj, addr, rbuf, N) != DEV_EOK) return DEV_ERROR;
	if (memcmp(wbuf, rbuf, N) != 0) return DEV_ERROR;

	return DEV_EOK;
}

void dev_eeprom_init(struct dev_eeprom *pobj, eeprom_id_e dev_id)
{
	assert_report(pobj != NULL);
	pobj->id = dev_id;
	pobj->page_size = eeprom_list[pobj->id].page_size;
	pobj->total_size = eeprom_list[pobj->id].total_size;

	pobj->read = dev_eeprom_read;
	pobj->read_byte = dev_eeprom_read_byte;
	pobj->write = dev_eeprom_write;
	pobj->write_byte = dev_eeprom_write_byte;
	pobj->is_ready = dev_eeprom_is_ready;
	pobj->erase_all = dev_eeprom_erase_all;
}
#endif /* USE_DEV_EEPROM */