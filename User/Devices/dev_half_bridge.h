/**
 * @file dev_half_bridge.h
 * @brief 
 * @author Dalin
 * @version 1.00
 * @date 2024-11-11
 * 
 * @copyright Copyright (c) 2024  RobotDance Technology Co., Ltd.
 * 
 * @par 修改日志:
 * <table>
 * <tr><th>Date           <th>Version     <th>Author      <th>Description
 * <tr><td>2024-11-11     <td>1.00        <td>LinHui      <td>Init
 * </table>
 */

#ifndef __DEV_HALF_BRIDGE_H
 #define __DEV_HALF_BRIDGE_H

#include "dev_config.h"
#if defined(USE_DEV_HALF_BRIDGE)
#include "drv_gpio.h"
#include "drv_tim.h"
#include "drv_tim_pwm.h"
#include <stdint.h>

typedef enum
{
    BRIDGE_LOW = 0u,
    BRIDGE_HIGH
} half_bridge_state_e;

typedef enum
{
    BRIDGE_DEV1 = 0,
    BRIDGE_ID_MAX
} half_bridge_id_e;

typedef struct
{
    char name[20];
    timNumber_e tim;
    timChannel_e channel[4]; 
    // gpioDrv_t   gpio_en;
    // gpioDrv_t   gpio_fault;
} dev_half_bridge_config_t;

/**
 * @brief 接口需求：
 * 1、
 */
typedef struct dev_half_bridge
{
    half_bridge_id_e id;
    timNumber_e tim;
    uint16_t autoreload;
    uint32_t ccr[4];

    /* public */
    void (*start)(struct dev_half_bridge *pobj);
    void (*stop)(struct dev_half_bridge *pobj);
    // void (*enable)(struct dev_half_bridge *pobj);
    // void (*disable)(struct dev_half_bridge *pobj);
    int (*set_3pwm)(struct dev_half_bridge *pobj, uint32_t ccr1, uint32_t ccr2, uint32_t ccr3);
    
    /* 测试接口 */               
    void (*test)(struct dev_half_bridge *pobj);
} dev_half_bridge_t;

void dev_half_bridge_init(dev_half_bridge_t *pobj, half_bridge_id_e id);

#endif // USE_DEV_HALF_BRIDGE
#endif /* __DEV_HALF_BRIDGE_H */
