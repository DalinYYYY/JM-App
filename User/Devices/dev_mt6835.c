/**
 * @file dev_mt6835.c
 * @brief MT6835磁编码器驱动程序
 * 
 * @author dalin (dalin@robot.com)
 * @version 1.0
 * @date 2025-04-23
 * 
 * @copyright Copyright (c) 2026  1024 Tech.co, Ltd
 * 
 * @par 修改日志:
 * <table>
 * <tr><th>Date       <th>Version <th>Author  <th>Description
 * <tr><td>2025-04-12 <td>1.0     <td>Dalin     <td>分离角度获取和角度转化
 * <tr><td>2025-04-23 <td>2.0     <td>Dalin     <td>增加零点存储和重新实现SPI读写接口
 * </table>
 */

#include "dev_mt6835.h"

#if defined(USE_DEV_MT6835)
#include "assert_report.h"
#include "drv_spi.h"
#include "drv_gpio.h"
#include <math.h>

#define __SpiDrv1 DRV_SPI1

/* 引脚控制 */
static const mt6835_config_t mt6835_list[MT6835_ID_MAX] = {
{
    "MT6835_1",
    {DRV_SPI1},
    {(gpioType_e)DRV_GPIOB, (gpioPin_e)DRV_PIN_6, (drvPinState_e)0},
    {(gpioType_e)DRV_GPIOB, (gpioPin_e)DRV_PIN_7, (drvPinState_e)0}}
};

/**
 * @brief 设置引脚状态
 * 
 * @param  pobj             : xxx
 * @param  state            : 设置的状态
 * 
 */
static void dev_mt6835_csn_ctrl(struct dev_mt6835 *pobj, mt6835State_e state)
{
    drv_gpio_write(mt6835_list[pobj->id].csn, (drvPinState_e)state);
}

/**
 * @brief 通过spi通信快速地读取给定地址寄存器内的数据
 * 
 * @param  pobj             : xxx
 * @param  rxdata           : xxx
 * 
 */
static void mt6835_burst_read_reg(struct dev_mt6835 *pobj, mt6835_reg_enum_t reg, uint8_t *rxdata)
{
    uint16_t tx;
    tx = MT6835_READ | reg;    

    dev_mt6835_csn_ctrl(pobj, MT6835_LOW);
    drv_spi_transfer(mt6835_list[pobj->id].spi_num, (uint8_t *)&tx, (uint8_t *)rxdata, 3, 200);
    dev_mt6835_csn_ctrl(pobj, MT6835_HIGH);
}

static void mt6835_read_reg(struct dev_mt6835 *pobj, mt6835_reg_enum_t reg, uint8_t *rxdata)
{
    uint16_t tx;
    tx = MT6835_READ | reg;

    dev_mt6835_csn_ctrl(pobj, MT6835_LOW);
    drv_spi_transfer(mt6835_list[pobj->id].spi_num, (uint8_t *)&tx, (uint8_t *)rxdata, 3, 200);
    dev_mt6835_csn_ctrl(pobj, MT6835_HIGH);
}


static bool mt6835_write_reg(dev_mt6835_t *pobj, mt6835_reg_enum_t reg, uint8_t data)
{
    uint8_t rxdata[3] = {0, 0, 0xFF};
    uint16_t tx;
    tx = MT6835_WRITE | data;

    dev_mt6835_csn_ctrl(pobj, MT6835_LOW);
    drv_spi_transfer(mt6835_list[pobj->id].spi_num, (uint8_t *)&tx, (uint8_t *)rxdata, 3, 200);
    dev_mt6835_csn_ctrl(pobj, MT6835_HIGH);

    if (rxdata[2] == 0xFF || rxdata[2] != 0x55) {
        return false;
    }
    return true;
}

static bool mt6835_write_eeprom(dev_mt6835_t *pobj, mt6835_reg_enum_t reg, uint8_t data)
{
    uint8_t rxdata[3] = {0, 0, 0xFF};
    uint16_t tx;
    tx = MT6835_WRITEEEPROM | data;

    dev_mt6835_csn_ctrl(pobj, MT6835_LOW);
    drv_spi_transfer(mt6835_list[pobj->id].spi_num, (uint8_t *)&tx, (uint8_t *)rxdata, 3, 200);
    dev_mt6835_csn_ctrl(pobj, MT6835_HIGH);

    if (rxdata[2] == 0xFF || rxdata[2] != 0x55) {
        return false;
    }
    return true;
}

/*
* @brief 读取mt6835芯片测量的角度原始值
* @param:
*/
static uint32_t dev_mt6835_get_raw(struct dev_mt6835 *pobj)
{
    uint16_t rxdata[3];
    mt6835_burst_read_reg(pobj, MT6835_REG_ANGLE3, (uint8_t *)rxdata);
    pobj->raw = (uint32_t)(rxdata[1] << 5) | (rxdata[2] >> 11);

    if(pobj->running_dir <= 1)
    {
        pobj->raw = pobj->raw;
    }
    else
    {
        pobj->raw = 0x1fffff - pobj->raw;
    }
    
    return pobj->raw;
}

/*
* @brief  读取mt6835机械角度数据
* @param  ：
*/
static float dev_mt6835_get_machAngle(struct dev_mt6835 *pobj)
{
    float _angle_org = (float)(pobj->raw) / 2097152.0F * 360.0F;
    pobj->mech_angle_org = _angle_org;
    float _angle_remove_off = _angle_org - pobj->offset;
    pobj->mech_angle_remove_off = _angle_remove_off;
    pobj->mechanical_angle = ((_angle_remove_off >= 0 ? _angle_remove_off : (_angle_remove_off + 360.0F)));
    
    return pobj->mechanical_angle;
}

static void dev_mt6835_set_offset(struct dev_mt6835 *pobj, float offset)
{
    pobj->offset = offset;
}

static void dev_mt6835_set_dir(struct dev_mt6835 *pobj, int dir)
{
    pobj->running_dir = dir; 
}

/**
 * @brief 在raw中获取mt6835原始零角
 * @param mt6835 mt6835对象
 * @return uint16_t 原始的零角度数据
 */
uint16_t mt6835_get_raw_zero_angle(dev_mt6835_t *pobj)
{
    uint8_t rx_buf[2] = {0};
    mt6835_read_reg(pobj, MT6835_REG_ZERO_POS2, &rx_buf[1]);
    mt6835_read_reg(pobj, MT6835_REG_ZERO_POS1, &rx_buf[0]);
    uint16_t res = (rx_buf[1] << 4) | (rx_buf[0] >> 4);
    return res;
}

/**
 * @brief  获取mt6835零角度
 * 
 * @param  mt6835           : mt6835对象
 * 
 * @return float         : 零角度数据
 */
float mt6835_get_zero_angle(dev_mt6835_t *pobj)
{
    return (float)mt6835_get_raw_zero_angle(pobj) * MT6835_ZERO_REG_STEP;
}

/**
 * @brief  设置mt6835零角度
 * 
 * @param  mt6835           :  mt6835对象
 * @param  rad              :  零角度数据
 * 
 * @return True：成功，false：失败
 */
bool mt6835_set_zero_angle(dev_mt6835_t *pobj, float rad)
{
    uint16_t angle = (uint16_t)roundf(rad * 57.295779513f / MT6835_ZERO_REG_STEP);
    if (angle > 0xFFF)
    {
        return false;
    }

    uint8_t tx_buf[2] = {0};

    tx_buf[1] = angle >> 4;
    tx_buf[0] = (angle & 0x0F) << 4;
    mt6835_read_reg(pobj, MT6835_REG_ZERO_POS2, &tx_buf[0]);
    tx_buf[0] |=  tx_buf[0] & 0x0F;

    mt6835_write_reg(pobj, MT6835_REG_ZERO_POS2, tx_buf[1]);
    mt6835_write_reg(pobj, MT6835_REG_ZERO_POS1, tx_buf[0]);

    return true;
}
 
/**
 * @brief mt6835数据处理
 * @param pobj  mt6835对象
 */
static void dev_mt6835_handle(struct dev_mt6835 *pobj)
{
    static volatile uint32_t speed_tick = 0;
    assert_report(pobj != NULL);
    dev_mt6835_get_raw(pobj);
    dev_mt6835_get_machAngle(pobj); // 机械角度 [0 ~ 360°]
}

/**
  * @brief 初始化mt6835磁编码器对象
  * @param poles  极对数
  * @return dev_mt6835_t* mt6835对象
  */
void dev_mt6835_init(dev_mt6835_t *pobj, mt6835_id_e dev_id)
{
    assert_report(pobj != NULL);
    memset(pobj, 0, sizeof(dev_mt6835_t));
    pobj->id = dev_id;

    // pobj->get_mechanical_angle_raw = dev_mt6835_get_raw;
    pobj->get_mechanical_angle = dev_mt6835_get_machAngle;
    pobj->set_offset = dev_mt6835_set_offset;
    pobj->set_dir = dev_mt6835_set_dir;

    pobj->set_zero_angle = mt6835_set_zero_angle;
    pobj->get_raw_zero_angle = mt6835_get_zero_angle;

    pobj->update = dev_mt6835_handle;
}
#endif
