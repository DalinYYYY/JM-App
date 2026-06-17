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
#ifndef __DEV_MT6701_H_
#define __DEV_MT6701_H_

#include "dev_config.h"
#if defined(USE_DEV_MT6701)

#include "drv_spi.h"
#include "drv_gpio.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define MT6701_ANGLE_RESOLUTION (1 << 14)						/* 2^14 = 16384 */
#define MT6701_ZERO_REG_STEP (360.0F / MT6701_ANGLE_RESOLUTION) /* 零点步进, ° */

	typedef enum
	{
		MT6701_ID_1 = 0,
		MT6701_ID_MAX,
	} mt6701_id_e;

	typedef enum
	{
		MT6701_LOW = 0u,
		MT6701_HIGH
	} mt6701State_e;

	typedef enum
	{
		MT6701_DIR_CW = 0,	/* 顺时针(正向) */
		MT6701_DIR_CCW = 1, /* 逆时针(反向) */
	} mt6701_dir_e;

	/* 磁场状态 (Mg[3:0] 解析结果, 可叠加) */
	typedef enum
	{
		MT6701_MG_NORMAL = 0,
		MT6701_MG_TOO_STRONG = 1,
		MT6701_MG_TOO_WEAK = 2,
		MT6701_MG_INVALID = 3,
		MT6701_MG_BUTTON_PRESSED = 4,
		MT6701_MG_OVERSPEED = 8,
	} mt6701_mg_state_e;

	/* 资源配置 (在 device_config.c 的 mt6701_list 填表) */
	typedef struct
	{
		char name[20];
		gpioDrv_t csn;
	} mt6701_config_t;

	/* 配置表定义在 device_config.c */
	extern const mt6701_config_t mt6701_list[MT6701_ID_MAX];

	/**
	 * @brief MT6701设备对象
	 * @param  id                     : 设备编号
	 * @param  raw_buf                : SPI读回的3字节原始帧
	 * @param  raw                    : 14bit原始角度值
	 * @param  mech_angle_org         : 去偏移/方向处理后的机械角度, °
	 * @param  mechanical_angle       : 最终机械角度, ° [0,360)
	 * @param  mg_state               : 磁场状态(Mg[3:0]解析, 可叠加)
	 * @param  crc_code               : 帧中6bit CRC
	 * @param  crc_check              : CRC校验结果(0=通过, 1=失败)
	 * @param  offset                 : 角度偏移量(零点), °
	 * @param  dir                    : 旋转方向
	 */
	typedef struct dev_mt6701
	{
		mt6701_id_e id;
		uint8_t raw_buf[3];
		uint32_t raw;
		float mech_angle_org;
		float mechanical_angle;
		uint8_t mg_state;
		uint8_t crc_code;
		uint8_t crc_check;
		float offset;
		mt6701_dir_e dir;

		/* public */
		void (*update)(struct dev_mt6701 *pobj);					/* 读取并刷新机械角度 */
		bool (*set_zero_angle)(struct dev_mt6701 *pobj, float deg); /* 设置零点角度(°), 成功 true */
		float (*get_zero_angle)(struct dev_mt6701 *pobj);			/* 读取当前零点角度(°) */
		void (*calibrate_zero)(struct dev_mt6701 *pobj);			/* 将当前角度标定为 0° */
		void (*set_dir)(struct dev_mt6701 *pobj, mt6701_dir_e dir); /* 设置方向 */
		mt6701_dir_e (*get_dir)(struct dev_mt6701 *pobj);			/* 获取方向 */
	} dev_mt6701_t;

	/**
 * @brief 初始化 MT6701 对象
 */
	void dev_mt6701_init(dev_mt6701_t *pobj, mt6701_id_e dev_id);

#ifdef __cplusplus
}
#endif
#endif /* USE_DEV_MT6701 */
#endif /* __DEV_MT6701_H_ */
