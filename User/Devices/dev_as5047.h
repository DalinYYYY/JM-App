/**
 * @file        dev_as5047.h
 * @brief       AS5047P 磁编码器(14bit SPI): 角度/校验/零点/方向
 *
 * @details     AS5047P 16-bit SPI 帧:
 *   命令帧: bit15=parity(偶校验), bit14=R/W(1=read,0=write), bit13:0=addr
 *   数据帧: bit15=parity(偶校验), bit14=EF(错误标志), bit13:0=DATA(14bit角度)
 *   parity 覆盖范围: bit14 + bit13:0 (15bit, 不含 bit15 本身)
 *   读 ANGLECOM(0x3FFF): 发 0xFFFF(含parity), 再发 dummy, 第二帧收角度。
 *   读 ERRFL(0x0001): 发 0x4001(含parity), 再发 dummy, 第二帧收 ERRFL。
 *   SPI Mode 1(CPOL=0, CPHA=1), MSB first, 16-bit data size。
 *
 * @note        结构对齐 dev_mt6701.h, 手动 CS 控制(spiDrv_t.cs={0})。
 */
#ifndef __DEV_AS5047_H_
#define __DEV_AS5047_H_

#include "dev_config.h"
#if defined(USE_DEV_AS5047)

#include "drv_spi.h"
#include "drv_gpio.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AS5047_ANGLE_RESOLUTION (1 << 14)   /* 2^14 = 16384 */
#define AS5047_READ_ANGLE_CMD   0xFFFFU     /* 读 ANGLECOM(0x3FFF) + parity = 0xFFFF */

typedef enum {
	AS5047_ID_1 = 0,
	AS5047_ID_MAX,
} as5047_id_e;

typedef enum {
	AS5047_DIR_CW  = 1,
	AS5047_DIR_CCW = -1,
} as5047_dir_e;

/* 资源配置(在 dev_config_board.inc 的 as5047_list 填表) */
typedef struct {
	char name[20];
	spiDrv_t spi_num;   /* SPI 外设(cs 字段不使用, 填 {0}) */
	gpioDrv_t csn;      /* 片选引脚(手动控制) */
} as5047_config_t;

extern const as5047_config_t as5047_list[AS5047_ID_MAX];

typedef struct dev_as5047 {
	as5047_id_e id;
	uint16_t raw;            /* 14bit 原始角度 */
	float mech_angle_org;    /* 未补偿原始角度, ° */
	float mechanical_angle;  /* 最终机械角度, ° [0,360) */
	uint8_t  parity_err;     /* 奇偶校验错误标志 */
	uint8_t  ef;             /* AS5047P 错误标志位(EF) */
	uint16_t err_cnt;        /* 连续错误计数 */
	float offset;            /* 零点偏移, ° */
	as5047_dir_e dir;

	/* 诊断字段(调试用, 非 volatile: 中断上下文与调试器观察) */
	uint16_t last_frame;     /* 最近一次原始 16-bit 帧 */
	uint16_t spi_err_cnt;    /* SPI 传输失败计数(HAL 返回非 OK) */
	uint16_t errfl;          /* ERRFL 寄存器值(init 时读取一次, 0xFFFF=无响应) */

	/* public */
	void (*update)(struct dev_as5047 *pobj);
	void (*set_dir)(struct dev_as5047 *pobj, as5047_dir_e dir);
	as5047_dir_e (*get_dir)(struct dev_as5047 *pobj);
} dev_as5047_t;

void dev_as5047_init(dev_as5047_t *pobj, as5047_id_e dev_id);

#ifdef __cplusplus
}
#endif
#endif /* USE_DEV_AS5047 */
#endif /* __DEV_AS5047_H_ */
