/**
 * @file dev_motor.h
 * @brief 
 * @author Dalin
 * @version 1.00
 * @date 2024-11-12
 * 
 * @copyright Copyright (c) 2024  RobotDance Technology Co., Ltd.
 * 
 * @par 修改日志:
 * <table>
 * <tr><th>Date           <th>Version     <th>Author      <th>Description
 * <tr><td>2024-11-12     <td>1.00        <td>LinHui      <td>Init
 * </table>
 */

#ifndef __DEV_MOTOR_H__
#define __DEV_MOTOR_H__

#include <stdint.h>
#include "dev_mt6701.h"
#include "foc.h"
#include "dev_adc.h"
#include "dev_half_bridge.h"
#include "dev_mt6835.h"
#include "motion_param.h"
#include "dev_control_signal_acq.h"
#include "dev_motor_phase_current.h"

#include "feedforward_lpf.h"
//#include "notch_filter.h"
#include "lf_notch_filter.h"

typedef enum
{
    DEV_MOTOR_1 = 0,
    // DEV_MOTOR_2,
    DEV_MOTOR_MAX,
} motor_id_e;

typedef struct
{
    char name[20];
    gpioDrv_t gpio;
} dev_motor_enable_config_t;

typedef struct
{
    float position_target; // 位置环目标
    float velocity_target; // 速度环目标
    float current_target;  // 电流环目标
} motor_ctrl_target_t;

typedef struct dev_motor
{
    motor_id_e id; // 电机id
    uint8_t poles; // 极对数

    /* 外部输入回调函数 */
    focCurrent_t (*current_callback)(void);
    float (*ele_radian_callback)(void);

    motor_ctrl_target_t target; // 电机控制目标
    timNumber_e fsm_tim;        // 状态机定时器

    /* public */
    dev_mt6701_t mt6701;               // mt6701
    dev_mt6835_t mt6835;               // mt6835
    
    motion_param_t motor_param;        // motor_param
    dev_control_signal_acq_t acq;      // control signal acquisition
    foc_t foc;                         // foc
    dev_half_bridge_t half_bridge;     // dev_half_bridge
    dev_phase_current_t phase_current; // adc for current
} dev_motor_t;

void dev_motor_init(dev_motor_t *pobj, motor_id_e id,
                    focCurrent_t (*current_callback)(void),
                    float (*ele_radian_callback)(void));

extern ffc_lpf_filter_t ffc_filter; //低通滤波器
// extern notch_filter_t notch_filter;                 //带阻滤波器
extern lf_notch_filter_t lf_notch_filter; //带阻滤波器

#endif /* __DEV_MOTOR_H__ */
