/**
 * @file dev_motor_phase_current.h
 * @brief 
 * 
 * @author dalin (dalin@robot.com)
 * @version 1.0
 * @date 2025-12-16
 * 
 * @copyright Copyright (c) 2025  HYFOOS Tech.co, Ltd
 * 
 * @par 修改日志:
 * <table>
 * <tr><th>Date       <th>Version <th>Author  <th>Description
 * <tr><td>2025-12-16 <td>1.0     <td>Dalin     <td>Init
 * </table>
 */

#ifndef __DEV_MOTOR_PHASE_CURRENT_H
#define __DEV_MOTOR_PHASE_CURRENT_H

#include "dev_config.h"

#include <stdint.h>
#include "drv_adc.h"

// 公式I = U / R ,  U*50 = ADC*(3.3/4096) , R = 0.01Ω , 50为INA240A2增益，0.01为采样电阻
// I = U/R = (ADC*3.3/4096/50) / 0.04 = ADC*0.000402832f (A)

typedef enum
{
    ADCX_IA = 0,
    ADCX_IB,
    ADCX_IC,
    ADCX_INX_INJECTED_MAX,
} dev_injected_obj_e;


typedef struct
{
    char name[20];
    adcNumber_e id;
    adcChannel_e channel;
} dev_phase_current_config_t;

typedef struct
{
    int32_t a;
    int32_t b;
    int32_t c;
} dev_current_i3axis_t;

typedef struct
{
    float a;
    float b;
    float c;
} dev_current_f3axis_t;

/**
 * @brief 注入组需求：
 * 1. 三相电流（单位0.1A）
 */
typedef struct dev_adc_injected
{
    adcNumber_e id[4];    // 注入组总共只有rank1~rank4，故定义最大容量为4
    float gain;           // 每通道的放大器增益
    float shunt_resistor; // 每通道的采样电阻阻值，单位：欧姆
    dev_current_i3axis_t adc;
    dev_current_i3axis_t offset;
    dev_current_f3axis_t current;
    dev_current_f3axis_t voltage;
    ADC_HandleTypeDef *adc_instance;

    /* public */
    ADC_HandleTypeDef *(*get_injected_handle)(struct dev_adc_injected *pobj);
    int (*start)(struct dev_adc_injected *pobj);
    void (*update)(struct dev_adc_injected *pobj);
    void (*set_offset)(struct dev_adc_injected *pobj, dev_current_i3axis_t injected_offset);
} dev_phase_current_t;

void dev_phase_current_init(struct dev_adc_injected *pobj, uint8_t gain, float shunt_resistor);
void dev_adc_regular_init(struct dev_adc_regular *pobj);

#endif
