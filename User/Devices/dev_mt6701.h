/**
 * @file dev_mt6701.h
 * @brief 
 * @author Dalin
 * @version 1.00
 * @date 2024-11-25
 * 
 * Copyright (c) 2024  RobotDance Technology Co., Ltd.
 * 
 * @par 修改日志:
 * <table>
 * <tr><th>Date           <th>Version     <th>Author      <th>Description
 * <tr><td>2024-11-25     <td>1.00        <td>LinHui      <td>Init
 * <tr><td>2024-XX-XX     <td>1.01        <td>YourName    <td>新增零点设置/读取功能
 * </table>
 */

#ifndef _DEV_MT6701_H_
#define _DEV_MT6701_H_

#include "dev_config.h"

#if defined(USE_DEV_MT6701)
#include "drv_gpio.h"
#include <stdint.h>
#include <stdbool.h>

// MT6701 分辨率（14位）：2^14 = 16384
#define MT6701_ANGLE_RESOLUTION     (1 << 14)  
// 零点寄存器步进值 (360°/16384 = 0.02197265625°)
#define MT6701_ZERO_REG_STEP        (360.0F / MT6701_ANGLE_RESOLUTION)

typedef enum
{
    MT6701_ID_1 = 0,
//    MT6701_ID_2,
    MT6701_ID_MAX
} mt6701_id_e;

typedef enum
{
    MT6701_LOW = 0u,
    MT6701_HIGH
} mt6701State_e;

typedef struct
{
    char name[20];
    gpioDrv_t csn;
} mt6701_config_t;

// 1. 新增编码器方向枚举（在mt6701_mg_state_e枚举后）
typedef enum {
    MT6701_DIR_CW = 0,   // 顺时针（默认正向）
    MT6701_DIR_CCW = 1   // 逆时针（反向）
} mt6701_dir_e;

// 2. 扩展dev_mt6701结构体（增加方向成员）
typedef struct dev_mt6701
{
    mt6701_id_e id;
    uint8_t raw_buf[6];
    uint32_t raw;                // 原始14位角度值
    float mech_angle_org;        // 原始机械角度
    float mech_angle_remove_off; // 偏移补偿后角度
    float mechanical_angle;      // 最终机械角度
    uint8_t mg_state;            // 4位磁场状态Mg[3:0]
    uint8_t crc_code;            // 6位CRC校验码
    uint8_t crc_ckeck;           // crc计算结果：0=校验通过，1=校验失败
    float offset;                // 角度偏移量
    mt6701_dir_e dir;            // 新增：编码器方向配置
    void (*update)(struct dev_mt6701 *pobj);

    /* 零点相关函数指针 */
    bool (*set_zero_angle)(struct dev_mt6701 *pobj, float angle_deg); 
    float (*get_zero_angle)(struct dev_mt6701 *pobj);                 
    void (*calibrate_zero)(struct dev_mt6701 *pobj);                 

    /* 新增方向配置函数指针 */
    void (*set_dir)(struct dev_mt6701 *pobj, mt6701_dir_e dir); // 设置方向
    mt6701_dir_e (*get_dir)(struct dev_mt6701 *pobj);           // 获取当前方向
} dev_mt6701_t;

// 磁场状态解释枚举
typedef enum {
    MT6701_MG_NORMAL = 0,        // 磁场正常
    MT6701_MG_TOO_STRONG = 1,    // 磁场过强
    MT6701_MG_TOO_WEAK = 2,      // 磁场过弱
    MT6701_MG_INVALID = 3,       // 无效状态
    MT6701_MG_BUTTON_PRESSED = 4,// 旋钮被按压（Mg2=1）
    MT6701_MG_OVERSPEED = 8      // 超速（Mg3=1）
} mt6701_mg_state_e;

void dev_mt6701_init(dev_mt6701_t *pobj, mt6701_id_e dev_id);

#endif // USE_DEV_MT6701
#endif // _DEV_MT6701_H_