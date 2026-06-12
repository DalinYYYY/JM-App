/**
 * @file dev_mt6835.h
 * @brief 
 * 
 * @author Eamon (eamon.zhang@hyfoss-tec.com)
 * @version 1.0
 * @date 2025-04-07
 * 
 * doxdocgen.file.copyrightTag
 * 
 * @par 修改日志:
 * <table>
 * <tr><th>Date       <th>Version <th>Author  <th>Description
 * <tr><td>2025-04-07 <td>1.0     <td>Eamon     <td>内容
 * </table>
 */

#ifndef __DEV_MT6835_H_
#define __DEV_MT6835_H_

#include "dev_config.h"
#include "imath.h"
#include "ifilter.h"
#include <stdint.h>
#include <stdbool.h>

#if defined(USE_DEV_MT6835)

#include "drv_spi.h"
#include "drv_gpio.h"


//MT6835芯片操作指令 CMD 指令
#define MT6835_READ			      		0x3000 //读寄存器
#define MT6835_WRITE 			      	0x6000 //写寄存器
#define MT6835_WRITEEEPROM		      	0xC000 //写EPROM寄存器
#define MT6835_SETZEROPOINT	      		0x5000 //自动设置零点寄存器（自动设置当前角度到零点寄存器中）
#define MT6835_CONTINUOUSREAD	      	0xA000 //连续读取寄存器

#define MT6835_ZERO_REG_STEP        (0.088f)
#define MT6835_ANGLE_RESOLUTION     (1 << 21) // 2^21 = 2097152

/*
* 寄存器地址  枚举类型变量
*/
typedef enum mt6835_reg_enum_t
{
	MT6835_USER_ID         =       (0x0001),
	MT6835_REG_ANGLE3      =       (0x0003),  //角度寄存器1地址
	MT6835_REG_ANGLE2      =       (0x0004),  //角度寄存器2地址
	MT6835_REG_ANGLE1      =       (0x0005),  //角度寄存器3地址
	MT6835_REG_CRC         =       (0x0006),  
	MT6835_REG_ABZ_RES2    =       (0x0007),
	MT6835_REG_ABZ_RES1    =       (0x0008),
	MT6835_REG_ZERO_POS2   =       (0x0009),
	MT6835_REG_ZERO_POS1   =       (0x000A),
	MT6835_REG_UVW         =       (0x000B),
	MT6835_REG_PWM         =       (0x000C),
	MT6835_REG_HYST        =       (0x000D),
	MT6835_REG_AUTOCAL     =       (0x000E),
}mt6835_reg_enum_t;

typedef enum
{
	MT6805_ID_1 = 0,
	MT6835_ID_MAX
} mt6835_id_e;

typedef struct
{
	char name[20];//
	spiDrv_t spi_num;
	gpioDrv_t csn;
    gpioDrv_t cal_en;
} mt6835_config_t;

typedef enum
{
	MT6835_LOW = 0u,
	MT6835_HIGH
} mt6835State_e;


typedef struct dev_mt6835
{
	mt6835_id_e id;               // 设备ID
	spiDrv_t spi_num;          	  // SPI设备号	      
	uint32_t raw;                 // 原始值
	float mechanical_angle;       // 机械角度
	float mech_angle_org;         // 机械角度原始值
	float mech_angle_remove_off;  //
    float offset;                 // 偏移
	float foc_offset_static;	 // 静态偏移
	int running_dir;				// 运行方向

	/* public */
	void (*update)(struct dev_mt6835 *pobj);						// 角度更新
	float (*get_mechanical_angle)(struct dev_mt6835 *pobj);			// 获取机械角度
	uint32_t (*get_mechanical_angle_raw)(struct dev_mt6835 *pobj);	// 获取原始角度值
	void (*set_offset)(struct dev_mt6835 *pobj, float offset);		// 设置偏移角度
	void (*set_dir)(struct dev_mt6835 *pobj, int dir);		// 设置偏移角度
	
	bool (*set_zero_angle)(struct dev_mt6835 *mt6835, float rad);	// 设置零点角度
	float (*get_raw_zero_angle)(struct dev_mt6835 *mt6835);		// 获取原始零点角度

} dev_mt6835_t;

void dev_mt6835_init(dev_mt6835_t *pobj, mt6835_id_e dev_id);

#endif //_DEV_MT6835_H_
#endif //USE_DEV_MT6835

