/**
 * @file dev_motor_phase_current.c
 * @brief 电机三相电流采集模块
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
#include "dev_motor_phase_current.h"
#include "assert_report.h"
#include "ifilter.h"

/* -------------------------------------- 注入组通道配置 -------------------------------------- */

static const dev_phase_current_config_t adc_injected_config[ADCX_INX_INJECTED_MAX] = {
    {.name = "ia", .id = DRV_ADC_1, .channel = DRV_ADC_CH1 },
    {.name = "ib", .id = DRV_ADC_1, .channel = DRV_ADC_CH2},
    {.name = "ic", .id = DRV_ADC_1, .channel = DRV_ADC_CH3 },
};

/* ----------------------------------------- 注入组 ----------------------------------------- */

static ADC_HandleTypeDef *dev_phase_current_get_handle(struct dev_adc_injected *pobj)
{
    pobj->adc_instance = get_adc_handle((adcNumber_e)pobj->id[3]); // 此处注入组的通道均是对应ADC3
    return pobj->adc_instance;
}

static int dev_phase_current_start(struct dev_adc_injected *pobj)
{
    assert_report(pobj != NULL);
    int status = 0;

    // 遍历adc配置表，获取注入组的通道id，并记录到injected_id数组中，读取数据时使用
    // for (int i = 0; i < ADCX_INX_INJECTED_MAX; i++)
    // {
    //     drv_phase_current_start(adc_injected_config[i].id); // 启动注入组的adc
    //     pobj->id[i] = adc_injected_config[i].id;
    // }
    drv_adc_injected_start(DRV_ADC_1); // 启动注入组的adc
    pobj->id[0] = DRV_ADC_1;
    pobj->id[1] = DRV_ADC_1;
    pobj->id[2] = DRV_ADC_1;
    pobj->id[3] = DRV_ADC_1;
    return status;
}

/**
 * @brief 读取注入组的通道数据（这里是三相电流数据）
 * @param  pobj 
 */
static void dev_phase_current_get_value(struct dev_adc_injected *pobj)
{
    //    static int init = 1;
    assert_report(pobj != NULL);

    // 获取三相电流的adc数据
    pobj->adc.a = drv_adc_injected_get_value(pobj->id[0], DRV_ADC_RANK1);
    pobj->adc.b = drv_adc_injected_get_value(pobj->id[1], DRV_ADC_RANK2);
    pobj->adc.c = drv_adc_injected_get_value(pobj->id[2], DRV_ADC_RANK3);

    //    if(init)
    //    {
    //        // 初始化偏置值
    //        pobj->offset.a = pobj->adc.a;
    //        pobj->offset.b = pobj->adc.b;
    //        pobj->offset.c = pobj->adc.c;
    //        init = 0;
    //    }
    //
    //	pobj->adc.a = (pobj->adc.a - pobj->offset.a);
    //    pobj->adc.b = (pobj->adc.b - pobj->offset.b);
    //    pobj->adc.c = (pobj->adc.c - pobj->offset.c);
}

/**
 * @brief 设置注入组的adc偏置值（这里是三相电流的偏置值）
 * @param pobj 
 * @param injected_offset ：三相电流的adc偏置值
 */
static void dev_phase_current_set_offset(struct dev_adc_injected *pobj, dev_current_i3axis_t offset)
{
    assert_report(pobj != NULL);
    pobj->offset.a = offset.a;
    pobj->offset.b = offset.b;
    pobj->offset.c = offset.c;
}

/**
 * @brief 获取注入组的电压数据
 * @param pobj 
 */ 
static void dev_phase_current_get_voltage(struct dev_adc_injected *pobj)
{
    float them = 3.3F / 4096.0F;
    assert_report(pobj != NULL);
    pobj->voltage.a = (float)(pobj->adc.a - pobj->offset.a) * them;
    pobj->voltage.b = (float)(pobj->adc.b - pobj->offset.b) * them;
    pobj->voltage.c = (float)(pobj->adc.c - pobj->offset.c) * them;
}
/**
 * @brief 获取三相电流数据
 * @param pobj 
 * @return dev_current_f3axis_t 三相电流数据
 */
static dev_current_f3axis_t dev_adc_get_current(struct dev_adc_injected *pobj)
{
    assert_report(pobj != NULL);
    static dev_current_f3axis_t current = {0};
    float them = (1.0F / pobj->gain / pobj->shunt_resistor);

    current.a = pobj->voltage.a * them;
    current.b = pobj->voltage.b * them;
    current.c = pobj->voltage.c * them;
    return current;
}

/**
 * @brief 电流采样流程
 * @param pobj 
 */
static void dev_phase_current_update(struct dev_adc_injected *pobj)
{
    assert_report(pobj != NULL);
    static dev_current_f3axis_t prev_current = {0};

    // step 1: 读取注入组的adc数据
    dev_phase_current_get_value(pobj);

    // step 2: adc转为电压数据
    dev_phase_current_get_voltage(pobj);

    // step 3: 电压转为电流数据
    dev_current_f3axis_t current = dev_adc_get_current(pobj);

    // step 4: 滤波处理
    pobj->current.a = _lpfilter(0.9F, current.a, prev_current.a);
    pobj->current.b = _lpfilter(0.9F, current.b, prev_current.b);
    pobj->current.c = _lpfilter(0.9F, current.c, prev_current.c);
    //	pobj->current.c = - pobj->current.a - pobj->current.b;

    // step 5: 更新上一次的电流数据
    prev_current = current;
}

/**
 * @brief adc注入组初始化
 * @param  pobj ： adc注入组对象
 * @param  gain ： 放大倍数
 * @param  shunt_resistor ：采样电阻
 */
void dev_phase_current_init(struct dev_adc_injected *pobj, uint8_t gain, float shunt_resistor)
{
    assert_report(pobj != NULL);
    pobj->gain = gain;
    pobj->shunt_resistor = shunt_resistor;

    pobj->get_injected_handle = dev_phase_current_get_handle;
    pobj->start = dev_phase_current_start;
    pobj->set_offset = dev_phase_current_set_offset;
    pobj->update = dev_phase_current_update;
}
