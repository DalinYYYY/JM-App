/**
 * @file dev_mt6701.c
 * @brief MT6701磁编码器驱动（含角度、磁场状态、CRC解析+零点校准）
 * @author Dalin
 * @version 1.01
 * @date 2024-11-25
 * 
 * Copyright (c) 2024  RobotDance Technology Co., Ltd.
 * 
 * @par 修改日志:
 * <table>
 * <tr><th>Date           <th>Version     <th>Author      <th>Description
 * <tr><td>2024-11-25     <td>1.00        <td>LinHui      <td>Init
 * <tr><td>2024-XX-XX     <td>1.01        <td>YourName    <td>新增磁场/CRC解析+零点设置功能
 * </table>
 */

#include "dev_mt6701.h"

#if defined(USE_DEV_MT6701)
#include "drv_spi.h"
#include "mlog.h"
#include <math.h>
#include "imath.h"
#include "ifilter.h"
#include <stdlib.h>
#include "assert_report.h"

#define __SpiDrv1 DRV_SPI1

/* CRC6校验表（手册多项式：X⁶+X+1），预留扩展CRC校验功能 */
static const uint8_t tableCRC6[64] = {
    0x00, 0x03, 0x06, 0x05, 0x0C, 0x0F, 0x0A, 0x09,
    0x18, 0x1B, 0x1E, 0x1D, 0x14, 0x17, 0x12, 0x11,
    0x30, 0x33, 0x36, 0x35, 0x3C, 0x3F, 0x3A, 0x39,
    0x28, 0x2B, 0x2E, 0x2D, 0x24, 0x27, 0x22, 0x21,
    0x23, 0x20, 0x25, 0x26, 0x2F, 0x2C, 0x29, 0x2A,
    0x3B, 0x38, 0x3D, 0x3E, 0x37, 0x34, 0x31, 0x32,
    0x13, 0x10, 0x15, 0x16, 0x1F, 0x1C, 0x19, 0x1A,
    0x0B, 0x08, 0x0D, 0x0E, 0x07, 0x04, 0x01, 0x02};

/* 引脚配置 */
static const mt6701_config_t mt6701_list[MT6701_ID_MAX] = {
    {"MT6701_1_CSN", {(gpioType_e)DRV_GPIOA, (gpioPin_e)DRV_PIN_4, (drvPinState_e)0}},
};

/*
 * @brief   spi读数据
 * @param data  : 读出的数据
 * @param len   : 读的数据长度
 * @return int8_t   : 0：成功，-1：失败
 */
static int8_t mt6701_spi_read(struct dev_mt6701 *pobj, uint8_t *data, uint16_t len)
{
    spiDrv_t mt6701_dev;

    if (pobj->id != MT6701_ID_1)
    {
        log_error("mt6701_spi_read id error: %d\r\n", pobj->id);
        return -1; // 返回错误码，而非断言
    }
    mt6701_dev.hspi = __SpiDrv1;
    return drv_spi_read(mt6701_dev, data, len, 1000);
}

/*
 * @brief 设置引脚状态
 * @param  id       : 驱动引脚ID
 * @param  state    : 设置的状态
 */
static void dev_mt6701_csn_ctrl(struct dev_mt6701 *pobj, mt6701State_e state)
{
    drv_gpio_write(mt6701_list[pobj->id].csn, (drvPinState_e)state);
}

/**
 * @brief 设置编码器方向（正向/反向）
 * @param pobj  : mt6701对象
 * @param dir   : 目标方向（MT6701_DIR_CW/MT6701_DIR_CCW）
 */
static void mt6701_set_dir(struct dev_mt6701 *pobj, mt6701_dir_e dir)
{
    if (pobj == NULL)
    {
        log_error("mt6701_set_dir: pobj is NULL\n");
        return;
    }
    if (dir != MT6701_DIR_CW && dir != MT6701_DIR_CCW)
    {
        log_error("mt6701_set_dir: invalid dir: %d\n", dir);
        return;
    }
    pobj->dir = dir;
}

/**
 * @brief 获取当前编码器方向
 * @param pobj  : mt6701对象
 * @return mt6701_dir_e : 当前方向（非法则返回默认正向）
 */
static mt6701_dir_e mt6701_get_dir(struct dev_mt6701 *pobj)
{
    if (pobj == NULL)
    {
        log_error("mt6701_get_dir: pobj is NULL\n");
        return MT6701_DIR_CW; // 默认正向
    }
    return pobj->dir;
}

/*
 * @brief 解析磁场状态（Mg[3:0]）
 * @param mg_raw  : 原始4位磁场状态值（bit14-17）
 * @return uint8_t: 组合状态码（便于上层判断）
 */
static uint8_t mt6701_parse_mg_state(uint8_t mg_raw)
{
    uint8_t state = MT6701_MG_INVALID;
    // Mg[1:0]：基础磁场状态
    switch (mg_raw & 0x03)
    {
        case 0: state = MT6701_MG_NORMAL; break;
        case 1: state = MT6701_MG_TOO_STRONG; break;
        case 2: state = MT6701_MG_TOO_WEAK; break;
        default: state = MT6701_MG_INVALID; break;
    }
    // Mg2：旋钮按压状态（叠加）
    if (mg_raw & 0x04)
    {
        state |= MT6701_MG_BUTTON_PRESSED;
    }
    // Mg3：超速状态（叠加）
    if (mg_raw & 0x08)
    {
        state |= MT6701_MG_OVERSPEED;
    }
    return state;
}

/**
 * @brief （扩展功能）CRC6校验函数（可选启用）
 * @param raw_24bit : 24位原始数据
 * @return uint8_t  : 0=校验通过，1=校验失败
 */
uint8_t mt6701_crc6_check(uint32_t raw_24bit)
{
    // 提取需要校验的18位数据（D[13:0]+Mg[3:0]）
    uint32_t crc_data = (raw_24bit & 0x3FFFF); // bit0-17
    uint8_t crc_calc = 0;

    // CRC6计算逻辑（手册多项式：X⁶+X+1）
    for (int i = 17; i >= 0; i--)
    {
        crc_calc = tableCRC6[((crc_calc << 1) | ((crc_data >> i) & 0x01)) & 0x3F];
    }

    // 对比计算出的CRC和数据中的CRC（bit18-23）
    uint8_t crc_recv = (raw_24bit >> 18) & 0x3F;
    return (crc_calc == crc_recv) ? 0 : 1;
}

static uint32_t angle_cnt = 0;
/*
 * @brief  读取mt6701原始数据（角度+磁场+CRC）
 * @param pobj    mt6701对象
 */
static void dev_mt6701_get_raw(struct dev_mt6701 *pobj)
{
    uint8_t buf[6] = {0}; 
    int8_t ret;

    dev_mt6701_csn_ctrl(pobj, MT6701_LOW);
    ret = mt6701_spi_read(pobj, pobj->raw_buf, 3); // 读取24位完整数据
    dev_mt6701_csn_ctrl(pobj, MT6701_HIGH);

    if (ret != 0)
    {
        log_error("MT6701 SPI read failed! ID: %d\n", pobj->id);
        pobj->raw = 0;
        pobj->mg_state = MT6701_MG_INVALID;
        pobj->crc_code = 0;
        return;
    }

    // 24位完整数据
    pobj->raw  = (uint32_t)((pobj->raw_buf[0]<<7) | (pobj->raw_buf[1])>>1)  & 0x3FFF;

    // 1. 解析14位角度数据（bit0-13）
//    pobj->raw = ( raw_24bit  ) & 0x3FFF; // 0x3FFF = 二进制14个1

    // 2. 解析4位磁场状态（bit14-17）
//    uint8_t mg_raw = (raw_24bit >> 14) & 0x0F; // 右移14位，掩码4位
//    pobj->mg_state = mt6701_parse_mg_state(mg_raw);

//    // 3. 解析6位CRC校验码（bit18-23）
//    pobj->crc_code = (raw_24bit >> 18) & 0x3F; // 右移18位，掩码6位

//    // 4. 计算CRC结果
//    pobj->crc_ckeck = mt6701_crc6_check(raw_24bit);

    // 调试日志：输出完整解析结果
//    log_debug("MT6701 ID:%d | 原始角度:%d | 磁场状态:0x%02X | CRC:0x%02X\n",
//              pobj->id, pobj->raw, pobj->mg_state, pobj->crc_code);
}

/**
 * @brief  计算机械角度（严格对齐手册公式：θ = (ΣD[i]*2^i / 16384) * 360°）
 * @param pobj  : mt6701对象
 */
static void dev_mt6701_get_machAngle(struct dev_mt6701 *pobj)
{
    // 分母是16384（14位分辨率）
    float _angle_org = (float)pobj->raw / MT6701_ANGLE_RESOLUTION * 360.0F;
    // pobj->mech_angle_org = _angle_org;

    // 偏移补偿 + 归一化到 0~360°（鲁棒性处理）
    float _angle_remove_off = fmod(_angle_org - pobj->offset, 360.0F);
    if (_angle_remove_off < 0)
    {
        _angle_remove_off += 360.0F;
    }

    // 新增：方向反转处理
    if (pobj->dir == MT6701_DIR_CCW)
    {
        _angle_remove_off = fmod(360.0F - _angle_remove_off, 360.0F); // 反向：360 - 补偿后角度
    }


    pobj->mech_angle_org = _angle_remove_off;
    // pobj->mechanical_angle = _angle_remove_off; // 最终输出0~360°的机械角度
}

/**
 * @brief  设置零点角度（单位：度）
 * @param  pobj       : mt6701对象
 * @param  angle_deg  : 要设置的零点角度（°）
 * @return bool       : true=成功，false=失败
 */
static bool mt6701_set_zero_angle(struct dev_mt6701 *pobj, float angle_deg)
{
    if (pobj == NULL)
    {
        log_error("mt6701_set_zero_angle: pobj is NULL\n");
        return false;
    }

    // 角度范围校验（0~360°）
    if (angle_deg < 0.0F || angle_deg > 360.0F)
    {
        log_error("mt6701_set_zero_angle: invalid angle: %.2f°\n", angle_deg);
        return false;
    }

    // 将角度转换为偏移量（offset = 原始角度 - 目标零点角度）
    // 先更新一次原始角度，保证偏移量准确
    dev_mt6701_get_raw(pobj);
    dev_mt6701_get_machAngle(pobj);
    
    pobj->offset = fmod(pobj->mech_angle_org - angle_deg, 360.0F);
    if (pobj->offset < 0)
    {
        pobj->offset += 360.0F;
    }

    // log_info("mt6701_set_zero_angle: set zero to %.2f°, offset=%.2f°\n", angle_deg, pobj->offset);
    return true;
}

/**
 * @brief  读取当前零点角度（单位：度）
 * @param  pobj       : mt6701对象
 * @return float      : 当前零点角度（°）
 */
static float mt6701_get_zero_angle(struct dev_mt6701 *pobj)
{
    // if (pobj == NULL)
    // {
    //     log_error("mt6701_get_zero_angle: pobj is NULL\n");
    //     return 0.0F;
    // }

    // 零点角度 = 原始角度 - 偏移量（归一化到0~360°）
    dev_mt6701_get_raw(pobj);
    dev_mt6701_get_machAngle(pobj);
    
    float zero_angle = fmod(pobj->mech_angle_org - pobj->offset, 360.0F);
    if (zero_angle < 0)
    {
        zero_angle += 360.0F;
    }

    // log_debug("mt6701_get_zero_angle: current zero angle=%.2f°\n", zero_angle);
    return zero_angle;
}

/**
 * @brief  校准零点（将当前角度设为0°）
 * @param  pobj       : mt6701对象
 */
static void mt6701_calibrate_zero(struct dev_mt6701 *pobj)
{
    if (pobj == NULL)
    {
        log_error("mt6701_calibrate_zero: pobj is NULL\n");
        return;
    }

    // 更新原始角度，将当前角度设为0°
    dev_mt6701_get_raw(pobj);
    dev_mt6701_get_machAngle(pobj);
    
    pobj->offset = pobj->mech_angle_org; // 偏移量 = 当前原始角度
    // log_info("mt6701_calibrate_zero: calibrate success! current angle set to 0°, offset=%.2f°\n", pobj->offset);
}

/**
 * @brief mt6701数据处理主函数
 * @param pobj  mt6701对象
 */
static void dev_mt6701_handle(struct dev_mt6701 *pobj)
{
    assert_report(pobj != NULL);

    // dev_mt6701_get_raw(pobj);       // 解析原始数据（角度+磁场+CRC）
    // dev_mt6701_get_machAngle(pobj); // 计算机械角度 [0 ~ 360°]

    pobj->mechanical_angle = mt6701_get_zero_angle(pobj);
}

/**
 * @brief 初始化mt6701对象
 * @param pobj    : mt6701对象指针
 * @param dev_id  : 设备ID
 */
void dev_mt6701_init(dev_mt6701_t *pobj, mt6701_id_e dev_id)
{
    assert_report(pobj != NULL);
    memset(pobj, 0, sizeof(dev_mt6701_t));

    pobj->id = dev_id;
    pobj->offset = 0.0F;
    pobj->update = dev_mt6701_handle;

    // 绑定零点相关函数（对齐MT6835接口）
    pobj->set_zero_angle = mt6701_set_zero_angle;
    pobj->get_zero_angle = mt6701_get_zero_angle;
    pobj->calibrate_zero = mt6701_calibrate_zero;

    pobj->set_dir = mt6701_set_dir;
    pobj->get_dir = mt6701_get_dir;

    log_info("mt6701_init: ID=%d init success\n", dev_id);
}

#endif /* USE_DEV_MT6701 */