/**
 * @file        dev_mt6835.h
 * @brief       MT6835磁编码器(21bit SPI): 角度/零点读写(寄存器/EEPROM)/方向
 *
 * @author      Dalin (dalinyy@163.com)
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
#ifndef __DEV_MT6835_H_
#define __DEV_MT6835_H_

#include "dev_config.h"
#if defined(USE_DEV_MT6835)

#include "drv_spi.h"
#include "drv_gpio.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* 操作指令 */
#define MT6835_READ           0x3000            /* 读寄存器 */
#define MT6835_WRITE          0x6000            /* 写寄存器 */
#define MT6835_WRITEEEPROM    0xC000            /* 写 EEPROM */
#define MT6835_SETZEROPOINT   0x5000            /* 自动设零点 */
#define MT6835_CONTINUOUSREAD 0xA000            /* 连续读 */

#define MT6835_ZERO_REG_STEP    (0.088f)        /* 零点寄存器步进, ° */
#define MT6835_ANGLE_RESOLUTION (1 << 21)       /* 2^21 = 2097152 */
#define MT6835_ANGLE_MASK       (0x1FFFFF)      /* 21bit 角度掩码 */
#define MT6835_RAD2DEG          (57.295779513f) /* 弧度转角度 */

	/* 寄存器地址 */
	typedef enum
	{
		MT6835_USER_ID = 0x0001,
		MT6835_REG_ANGLE3 = 0x0003,
		MT6835_REG_ANGLE2 = 0x0004,
		MT6835_REG_ANGLE1 = 0x0005,
		MT6835_REG_CRC = 0x0006,
		MT6835_REG_ABZ_RES2 = 0x0007,
		MT6835_REG_ABZ_RES1 = 0x0008,
		MT6835_REG_ZERO_POS2 = 0x0009,
		MT6835_REG_ZERO_POS1 = 0x000A,
		MT6835_REG_UVW = 0x000B,
		MT6835_REG_PWM = 0x000C,
		MT6835_REG_HYST = 0x000D,
		MT6835_REG_AUTOCAL = 0x000E,
	} mt6835_reg_enum_t;

	typedef enum
	{
		MT6805_ID_1 = 0,
		MT6835_ID_MAX,
	} mt6835_id_e;

	typedef enum
	{
		MT6835_LOW = 0u,
		MT6835_HIGH
	} mt6835State_e;

	/* 资源配置 (在 device_config.c 的 mt6835_list 填表) */
	typedef struct
	{
		char name[20];
		spiDrv_t spi_num;
		gpioDrv_t csn;
		gpioDrv_t cal_en;
	} mt6835_config_t;

	/* 配置表定义在 device_config.c */
	extern const mt6835_config_t mt6835_list[MT6835_ID_MAX];

	typedef struct dev_mt6835
	{
		mt6835_id_e id;
		spiDrv_t spi_num;
		uint32_t raw;                /* 原始 21bit 角度值 */
		float mechanical_angle;      /* 机械角度, ° [0,360) */
		float mech_angle_org;        /* 原始机械角度, ° */
		float mech_angle_remove_off; /* 去偏移角度, ° */
		float offset;                /* 偏移 */
		float foc_offset_static;     /* 静态偏移(电角度对齐用) */
		int running_dir;             /* 运行方向 (<=1 正向, 否则反向) */

		/* public */
		void (*update)(struct dev_mt6835 *pobj);                       /* 读取并刷新机械角度 */
		float (*get_mechanical_angle)(struct dev_mt6835 *pobj);        /* 获取机械角度, ° */
		uint32_t (*get_mechanical_angle_raw)(struct dev_mt6835 *pobj); /* 获取原始角度值 */
		void (*set_offset)(struct dev_mt6835 *pobj, float offset);     /* 设置偏移角度 */
		void (*set_dir)(struct dev_mt6835 *pobj, int dir);             /* 设置运行方向 */
		bool (*set_zero_angle)(struct dev_mt6835 *pobj, float rad);    /* 写零点寄存器, 成功 true */
		float (*get_raw_zero_angle)(struct dev_mt6835 *pobj);          /* 读零点角度, ° */
	} dev_mt6835_t;

	/**
 * @brief 初始化 MT6835 对象
 */
	void dev_mt6835_init(dev_mt6835_t *pobj, mt6835_id_e dev_id);

#ifdef __cplusplus
}
#endif
#endif /* USE_DEV_MT6835 */
#endif /* __DEV_MT6835_H_ */
