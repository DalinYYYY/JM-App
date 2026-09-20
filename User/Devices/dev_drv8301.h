/**
 * @file        dev_drv8301.h
 * @brief       DRV8301 栅极驱动器设备对象(SPI 配置 + 状态读取)
 *
 * @details     DRV8301 内置 2 路电流运放 + Buck + OCP。
 *   SPI 帧 16-bit: bit15=R/W(0=write,1=read), bit14:12=addr, bit11:0=data(12-bit)
 *   寄存器: 0x00=CTRL1, 0x01=CTRL2
 *   读寄存器: 发读命令 → 发 dummy → 第二帧收数据(2-transfer)
 *   SPI Mode 1(CPOL=0, CPHA=1), 16-bit data size, MSB first
 *
 * @note        设备对象模式: 静态单例 + 方法指针。
 *   init 仅做 SPI 寄存器配置, 不操作 EN_GATE(由 motor_enable_list 通过
 *   dev_motor_enable/disable 控制 EN_GATE 引脚电平)。
 *   调用时序: main.c 调 dev_drv8301_init → dev_motor_init 中 dev_motor_disable
 *   (EN_GATE 低) → FOC 初始化 → dev_motor_enable(EN_GATE 高, 复位 fault latch)。
 */
#ifndef __DEV_DRV8301_H_
#define __DEV_DRV8301_H_

#include "dev_config.h"
#if defined(USE_DEV_DRV8301)

#include "drv_spi.h"
#include "drv_gpio.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	DRV8301_ID_1 = 0,
	DRV8301_ID_MAX,
} drv8301_id_e;

#define DRV8301_REG_CTRL1 0x00U
#define DRV8301_REG_CTRL2 0x01U

typedef struct {
	char name[20];
	spiDrv_t spi_num;       /* SPI 外设(cs 字段不使用, 填 {0}) */
	gpioDrv_t nscs;         /* nSCS 片选(手动控制) */
	gpioDrv_t nfault;       /* nFAULT 输入 (PD2), 低有效 */
	uint16_t ctrl1_value;   /* CTRL1 寄存器原始值 */
	uint16_t ctrl2_value;   /* CTRL2 寄存器原始值 */
} drv8301_config_t;

extern const drv8301_config_t drv8301_list[DRV8301_ID_MAX];

typedef struct dev_drv8301 {
	drv8301_id_e id;
	uint16_t ctrl1;
	uint16_t ctrl2;
	uint16_t ctrl1_readback; /* init 后回读 CTRL1, 验证 SPI 写入成功(调试用) */
	uint8_t  fault;

	/* public */
	void     (*init)(struct dev_drv8301 *pobj);
	uint16_t (*read_reg)(struct dev_drv8301 *pobj, uint8_t addr);
	void     (*write_reg)(struct dev_drv8301 *pobj, uint8_t addr, uint16_t val);
	uint8_t  (*get_fault)(struct dev_drv8301 *pobj);
} dev_drv8301_t;

void dev_drv8301_init(dev_drv8301_t *pobj, drv8301_id_e id);
extern dev_drv8301_t g_dev_drv8301;

#ifdef __cplusplus
}
#endif
#endif /* USE_DEV_DRV8301 */
#endif /* __DEV_DRV8301_H_ */
