/**
 * @file dev_adc.h
 * @brief 
 * 
 * @author Dalin
 * @version 1.00
 * @date 2024-10-31
 * 
 * @copyright Copyright (c) 2024 RobotDance Technology Co., Ltd.
 * 
 * @par 修改日志:
 * <table>
 * <tr><th>Date           <th>Version     <th>Author      <th>Description
 * <tr><th>2024-10-31     <td>1.00        <td>Mr.Lin      <td>Init
 * </table>
 */

#ifndef __DEV_AMPLIFIER_H
#define __DEV_AMPLIFIER_H

#include "dev_config.h"

#include <stdint.h>
#include "drv_adc.h"


// 公式I = U / R ,  U*50 = ADC*(3.3/4096) , R = 0.01Ω , 50为INA240A2增益，0.01为采样电阻
// I = U/R = (ADC*3.3/4096/50) / 0.04 = ADC*0.000402832f (A)
typedef enum
{
    ADCX_IBUS,          // 母线电流检测
    ADCX_VBUS ,         // 母线电压检测

    ADCX_REGULAR_MAX,
} dev_regular_obj_e;

typedef struct
{
    char name[20];
    adcNumber_e id;
    adcChannel_e channel;
} dev_adc_regular_config_t;

// 规则组通道数
#define REGULAR_Nbrs            (ADCX_REGULAR_MAX)

// 平均滤波窗口
#define REGULAR_AVERAGE_LPF     (1)

// 采样通道总数（规则组dma采样）
#define REGULAR_SAMPLE_Nbrs     ((REGULAR_Nbrs) * (REGULAR_AVERAGE_LPF))


/**
 * @brief 规则组需求：
 * 1. 母线电压（单位0.1V）[规则组DMA]
 * 2. 母线电流（单位0.1A）[规则组DMA]
 * 3. 温度（单位0.1℃）[规则组DMA]
 */
typedef struct dev_adc_regular
{
    uint8_t adc_nbr[DRV_ADC_MAX];                   // 每个adc的通道数
    uint32_t raw[DRV_ADC_MAX][REGULAR_SAMPLE_Nbrs]; // 规则组dma总采样通道数 
    uint8_t channel_nbr;                            // 规则组adc总通道数
    uint32_t adc[DRV_ADC_MAX][REGULAR_Nbrs];        // 每个adc对应的通道的adc值, 这里给每个adc都定义了已用的通道数，方便计算
    float voltage[DRV_ADC_MAX][REGULAR_Nbrs];       // 每个adc对应的通道的电压值, 这里给每个adc都定义了已用的通道数，方便计算
    int32_t offset[REGULAR_Nbrs];                   // 每通道的adc偏置值

    float enpr;
    float vbus;
    float ibus;
    float temp_driver;
    float temp_motor;
    /* public */
    float (*get_enpr)(struct dev_adc_regular *pobj);                            // 使能信号                          
    float (*get_vbus)(struct dev_adc_regular *pobj);                            // 母线电压
    float (*get_ibus)(struct dev_adc_regular *pobj);                            // 母线电流
    float (*get_temp_driver)(struct dev_adc_regular *pobj);                     // 驱动器温度
    float (*get_temp_motor)(struct dev_adc_regular *pobj);                      // 电机温度      
    int (*start)(struct dev_adc_regular *pobj);   // 启动规则组
    void (*update)(struct dev_adc_regular *pobj); // 更新规则组数据
} dev_adc_regular_t;


void dev_adc_regular_init(struct dev_adc_regular *pobj);

extern dev_adc_regular_t dev_adc_regular;

#endif
