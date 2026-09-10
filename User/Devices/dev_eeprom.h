/**
 * @file dev_eeprom.h
 * @brief EEPROM 设备：字节/块读写、跨页写、写周期等待、参数持久化
 * @note  兼容 AT24Cxx I2C EEPROM 与 MB85RC16 FRAM：
 *        两者均为 16Kbit(2048B)、8bit 内部地址、块选择位在器件地址 A2A1A0 中，
 *        寻址时序一致，驱动可复用。
 *        EEPROM 特有: 页写限制(AT24C16=16B) + 写周期 tWR(需要 ACK polling 等待)。
 *        硬件 I2C 走 drv_i2c(USE_I2C_DRIVER)。
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
		i2cDrv_t i2c;		 /* I2C 传输描述(硬I2C: 外设编号/超时) */
		uint8_t i2c_addr;	 /* 7bit 基础器件地址(块0), AT24C16=0x50 */
		uint16_t page_size;	 /* 页大小, 字节 (AT24C16=16) */
		uint32_t total_size; /* 总容量, 字节 (AT24C16=2048) */
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
		int (*read)(struct dev_eeprom *pobj, uint32_t addr, uint8_t *data, uint16_t len);		  /* 任意长度读(跨块自动拆块) */
		int (*write)(struct dev_eeprom *pobj, uint32_t addr, const uint8_t *data, uint16_t len); /* 跨页自动分页写 + 写周期等待 */
		int (*read_byte)(struct dev_eeprom *pobj, uint32_t addr, uint8_t *data);
		int (*write_byte)(struct dev_eeprom *pobj, uint32_t addr, uint8_t data);
		bool (*is_ready)(struct dev_eeprom *pobj); /* 轮询写完成(ACK polling) */
		int (*erase_all)(struct dev_eeprom *pobj); /* 全片擦除(写 0xFF) */
	} dev_eeprom_t;

	/* 默认设备实例 (dev_eeprom.c 定义) */
	extern dev_eeprom_t dev_eeprom;

	/**
 * @brief 初始化 EEPROM 对象
 */
	void dev_eeprom_init(dev_eeprom_t *pobj, eeprom_id_e id);

	/**
 * @brief 芯片自检: 多块写入回读校验(覆盖块0跨页边界/块1/末尾块)
 * @note  会覆盖校验区域内的原数据, 仅用于上电自检/调试
 * @retval DEV_EOK 通过, DEV_ERROR 失败
 */
	int dev_eeprom_test(struct dev_eeprom *pobj);

#ifdef __cplusplus
}
#endif
#endif /* USE_DEV_EEPROM */
#endif /* __DEV_EEPROM_H */
