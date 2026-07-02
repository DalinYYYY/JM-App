
/**
 * @file        dev_eeprom.c
 * @brief 		读写片外eeprom存储设备
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
 */

#include "dev_eeprom.h"
#include "assert_report.h"

#if defined(DEV_EEPROM_USING_SOFT_I2C)
static i2c_soft_drv_t soft_i2c_dev = {0};//当eeprom使用软件I2C时，注册使用的设备句柄
#endif

/*
 * @brief  eeprom设备读取内存数据
 * @param  *pobj : eeprom设备句柄
 * @param  reg  : 寄存器地址(16位 eeprom地址)
 * @param  data : 读取数据的buff
 * @param  len : 读取数据的长度
 * @retval 读取结果 ：DEV_EOK正常，其他则错误
 */
int dev_mb85rc16_read_data(struct dev_eeprom *pobj, uint32_t reg, uint8_t *data, uint16_t len)
{
	int ret = 0;
	uint8_t addrss = reg & 0xFF;

	if (len > eeprom_list[pobj->id].total_size) return DEV_ERROR;

#if defined(DEV_EEPROM_USING_SOFT_I2C)
	soft_i2c_dev.slave_address = eeprom_list[pobj->id].i2c_addr + ((reg / 0xFF) << 1);
	ret = drv_i2c_write_nbytes(&soft_i2c_dev, addrss, data, len);
#else
	eeprom_list[pobj->id].i2c.dev_addr = eeprom_list[pobj->id].i2c_addr + ((reg / 0xFF) << 1);
	eeprom_list[pobj->id].i2c.mem_addr = addrss;
	while(!ret)
	{
		ret = drv_i2c_recv(eeprom_list[pobj->id].i2c, data, len); //阻塞读取返回数据
	}
#endif

	return ret;
}

/*
 * @brief  eeprom设备读取内存一字节的数据
 * @param  *pobj : eeprom设备句柄
 * @param  reg  : 寄存器地址(16位 eeprom地址)
 * @param  data : 读取数据的buff
 * @retval 读取结果 ：DRV_EOK正常，其他则错误
 */
int dev_mb85rc16_read_byte(struct dev_eeprom *pobj, uint32_t addr, uint8_t *data)
{
	return dev_mb85rc16_read_data(pobj, addr, data, 1);
}

/*
 * @brief  eeprom设备写入内存数据
 * @param  *pobj : eeprom设备句柄
 * @param  reg  : 寄存器地址(16位 eeprom地址)
 * @param  data : 写入数据的data区
 * @param  len : 写入数据的长度
 * @retval 写入结果 ：DRV_EOK正常，其他则错误
 */
int dev_mb85rc16_write_data(struct dev_eeprom *pobj, uint32_t addr, uint8_t *data, uint16_t len)
{
	int ret = 0;
	uint8_t addrss = addr & 0xFF;

	if (len > eeprom_list[pobj->id].total_size) return DEV_ERROR;

#if defined(DEV_EEPROM_USING_SOFT_I2C)
	soft_i2c_dev.slave_address = eeprom_list[pobj->id].i2c_addr + ((addr / 0xFF) << 1);
	ret = drv_i2c_write_nbytes(&soft_i2c_dev, addrss, data, len);
#else
	eeprom_list[pobj->id].i2c.dev_addr = eeprom_list[pobj->id].i2c_addr + ((addr / 0xFF) << 1);
	eeprom_list[pobj->id].i2c.mem_addr = addrss;

	ret = drv_i2c_send(eeprom_list[pobj->id].i2c, data, len);
#endif

	return ret;
}

/*
 * @brief  eeprom设备写入内存一字节数据
 * @param  *pobj : eeprom设备句柄
 * @param  reg  : 寄存器地址(16位 eeprom地址)
 * @param  data : 写入数据的data区
 * @retval 写入结果 ：DRV_EOK正常，其他则错误
 */
int dev_mb85rc16_write_byte(struct dev_eeprom *pobj, uint32_t addr, uint8_t data)
{
	return dev_mb85rc16_write_data(pobj, addr, &data, 1);
}

/*
 * @brief  获取eeprom设备状态，判断是否准备完成
 * @param  *pobj : eeprom设备句柄
 * @retval 写入结果 ：true正常，false错误
 */
bool dev_eeprom_get_state(struct dev_eeprom *pobj)
{
	return (bool)drv_i2c_ready(eeprom_list[pobj->id].i2c, 1U);
}

/*
 * @brief  擦除设备所有内存区
 * @param  *pobj : eeprom设备句柄
 * @retval 写入结果 ：DRV_EOK正常，其他则错误
 */
int dev_eeprom_erase_all(struct dev_eeprom *pobj)
{
	int ret = 0;
	uint8_t data = 0xFF;
	uint16_t address = 0;

	for (int i=0; i<eeprom_list[pobj->id].total_size; i++)
	{
		address = i;
		ret = dev_mb85rc16_write_byte(pobj, address, data);
		if (ret != DEV_EOK) break;
	}

	return ret;
}

/********************* 使用软件i2c时的驱动部分 *********************/
#if defined(DEV_EEPROM_USING_SOFT_I2C)
static uint8_t soft_i2c_sda_read(i2c_id_e id)
{
	return (uint8_t)drv_gpio_read(eeprom_list[id].i2c_sda_pin);
}

static void soft_i2c_sda_dir(i2c_id_e id, i2c_sda_dir_e dir)
{
	uint8_t gpio_sta = (uint8_t)drv_gpio_read(eeprom_list[id].i2c_sda_pin);

	gpioInit_t io_init = {
		.mode = (dir == IN) ? DRV_INPUT : DRV_OUTPUT_PP,
		.pull = gpio_sta ? DRV_PULLUP : DRV_PULLDOWN,
		.speed = DRV_HIGH,
		.alternate = 0,
	};
	drv_gpio_init(eeprom_list[id].i2c_sda_pin, io_init);
}

static void soft_i2c_sda(i2c_id_e id, i2c_state_e level)
{
	drv_gpio_write(eeprom_list[id].i2c_sda_pin, (drvPinState_e)level);
}

static void soft_i2c_scl(i2c_id_e id, i2c_state_e level)
{
	drv_gpio_write(eeprom_list[id].i2c_scl_pin, (drvPinState_e)level);
}
#endif
/**********************************************************************/

void dev_eeprom_init(struct dev_eeprom *pobj, eeprom_id_e dev_id)
{
	assert_report(pobj != NULL);
	pobj->id = dev_id;
#if defined(DEV_EEPROM_USING_SOFT_I2C)
	memset(&soft_i2c_dev, 0, sizeof(i2c_soft_drv_t));
	drv_i2c_init(&soft_i2c_dev, pobj->id, soft_i2c_sda_read, soft_i2c_sda_dir, soft_i2c_sda, soft_i2c_scl, eeprom_list[pobj->id].i2c_addr);
#endif

	pobj->read = dev_mb85rc16_read_data;
	pobj->read_byte = dev_mb85rc16_read_byte;
	pobj->write = dev_mb85rc16_write_data;
	pobj->write_byte = dev_mb85rc16_write_byte;
	pobj->is_ready = dev_eeprom_get_state;
	pobj->erase_all = dev_eeprom_erase_all;
}

