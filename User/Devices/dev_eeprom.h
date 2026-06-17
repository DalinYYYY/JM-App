/**
 * @file dev_eeprom.h
 * @brief EEPROM 设备：字节/块读写、跨页写、参数持久化
 * @note  默认按 I2C 接口(AT24Cxx 类)设计; 若为 SPI EEPROM 请替换驱动头与配置成员。
 */
#ifndef __DEV_EEPROM_H
#define __DEV_EEPROM_H

#include "dev_config.h"
#if defined(USE_DEV_EEPROM)

#include "drv_i2c.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

	typedef enum
	{
		EEPROM_ID_1 = 0,
		EEPROM_ID_MAX,
	} eeprom_id_e;

	/* 资源配置 (在 device_config.c 的 eeprom_list 填表) */
	typedef struct
	{
		char name[20];
//		i2cDrv_t i2c;		 /* I2C 驱动句柄 */
		uint8_t i2c_addr;	 /* 7bit 器件地址 */
		uint16_t page_size;	 /* 页大小, 字节 */
		uint32_t total_size; /* 总容量, 字节 */
		uint8_t addr_width;	 /* 内部地址宽度: 1=8bit, 2=16bit */
	} dev_eeprom_config_t;

	/* 配置表定义在 device_config.c */
	extern const dev_eeprom_config_t eeprom_list[EEPROM_ID_MAX];

	typedef struct dev_eeprom
	{
		eeprom_id_e id;
		uint16_t page_size;
		uint32_t total_size;

		/* public, 返回 DEV_EOK / DEV_ERROR */
		int (*read)(struct dev_eeprom *pobj, uint32_t addr, uint8_t *data, uint16_t len);		 /* 任意长度读 */
		int (*write)(struct dev_eeprom *pobj, uint32_t addr, const uint8_t *data, uint16_t len); /* 跨页自动分页写 */
		int (*read_byte)(struct dev_eeprom *pobj, uint32_t addr, uint8_t *data);
		int (*write_byte)(struct dev_eeprom *pobj, uint32_t addr, uint8_t data);
		bool (*is_ready)(struct dev_eeprom *pobj); /* 轮询写完成(ACK polling) */
		int (*erase_all)(struct dev_eeprom *pobj); /* 全片擦除(写 0xFF) */
	} dev_eeprom_t;

	/**
 * @brief 初始化 EEPROM 对象
 */
	void dev_eeprom_init(dev_eeprom_t *pobj, eeprom_id_e id);

#ifdef __cplusplus
}
#endif
#endif /* USE_DEV_EEPROM */
#endif /* __DEV_EEPROM_H */
