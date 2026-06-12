/**
 * @file dev_adc.c
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
#include "dev_adc.h"
#include "assert_report.h"
#include "mlog.h"
#include "ifilter.h"


dev_adc_regular_t dev_adc_regular;
/* -------------------------------------- 规则组通道配置 -------------------------------------- */

// ADCX_EN_PR,
// ADCX_VBUS ,
// ADCX_IBUS,
// ADCX_TEMP_DRIVER,
// ADCX_TEMP_MOTOR,
static const dev_adc_regular_config_t adc_regular_config[ADCX_REGULAR_MAX] = {
    {.name = "ibus",        .id = DRV_ADC_1, .channel = DRV_ADC_CH4},
    {.name = "vbus",        .id = DRV_ADC_1, .channel = DRV_ADC_CH12 },
};


/* --------------------------------------------- 规则组 --------------------------------------------- */

static int dev_adc_regular_start(struct dev_adc_regular *pobj)
{
    assert_report(pobj != NULL);
    int status = 0;
    memset(pobj->adc_nbr, 0, sizeof(pobj->adc_nbr)); // 初始化规则组每个通道对应的adc数量

    // 计数规则组下的每个通道对应的adc数量（多个通道可能来自同一个adc，这里就是计算同一adc下的通道数），
    // 同时对规则组的ADC进行校准，并启动注入组的ADC
    for (int i = 0; i < ADCX_REGULAR_MAX; i++)
    {
        switch (adc_regular_config[i].id)
        {
            case DRV_ADC_1: pobj->adc_nbr[DRV_ADC_1]++; break;
            case DRV_ADC_2: pobj->adc_nbr[DRV_ADC_2]++; break;
            case DRV_ADC_3: pobj->adc_nbr[DRV_ADC_3]++; break;
            default: log_error("adc id error| %s", __func__); break;
        }
        drv_adc_calibration_start(adc_regular_config[i].id); // 校准规则组的adc
    }
    // TODO: 适配不同的ADC，均存在多通道的情况（dma启动时需要依据adc，不同的adc且都存在多通道时，需要单独开启）
    for (adcNumber_e id = DRV_ADC_1; id < DRV_ADC_MAX; id++)
    {
        if (pobj->adc_nbr[id] == 0)
        {
            log_error("This adc%d has no channels", id);
        }
        else
        {
            log_base("This adc%d has %d channels", id, pobj->adc_nbr[id]);
            status = drv_adc_start_dma(id, pobj->raw[id], (pobj->adc_nbr[id] * REGULAR_AVERAGE_LPF));
        }
    }
    // 规则组adc的总通道数
    pobj->channel_nbr = pobj->adc_nbr[DRV_ADC_1] + pobj->adc_nbr[DRV_ADC_2] + pobj->adc_nbr[DRV_ADC_3];
    log_base("regular adc total channels: %d", pobj->channel_nbr);

    return status;
}

/**
 * @brief 读取DMA模式下规则组的通道数据（这里是母线电压、温度、母线电流数据）
 * @param  pobj 
 */
static void dev_adc_regular_get_value(struct dev_adc_regular *pobj)
{
    assert_report(pobj != NULL);

    uint32_t adc_sum[DRV_ADC_MAX][REGULAR_Nbrs] = {0};
    uint16_t adc_separate[REGULAR_Nbrs][REGULAR_AVERAGE_LPF] = {0};

    // TODO: DMA模式下，可以设置不同的通道倍数比例来作平均滤波处理，比如这里规则组原本通道数是3，倍数为10，
    // dma在启动配置的时候便以30个数据来设置的，故实际的3个通道的值需要除以10，才能得到最终经过平均的值
    // 遍历每个adc下（stm32一般是有3个ADC）是否有通道存在，且根据通道数进行遍历读取adc值
    for (adcNumber_e id = DRV_ADC_1; id < DRV_ADC_MAX; id++)
    {
        if (pobj->adc_nbr[id] != 0)
        {
            for (int i = 0; i < pobj->adc_nbr[id]; i++)
            {
                for (int j = 0; j < REGULAR_AVERAGE_LPF; j++)
                {
                    adc_separate[i][j] = pobj->raw[id][i + pobj->adc_nbr[id] * j];
                    adc_sum[id][i] += adc_separate[i][j];
                }
                pobj->adc[id][i] = adc_sum[id][i] / REGULAR_AVERAGE_LPF;
            }
        }
    }
}

/**
 * @brief 获取ADC（规则组）每通道的电压数据
 * @param  pobj
 */
static void dev_adc_regular_get_voltage(struct dev_adc_regular *pobj)
{
    assert_report(pobj != NULL);
    // 遍历adc，并将adc下存在通道的adc值转换成电压值
    for (adcNumber_e id = DRV_ADC_1; id < DRV_ADC_MAX; id++)
    {
        if (pobj->adc_nbr[id] != 0)
        {
            for (int i = 0; i < pobj->adc_nbr[id]; i++)
            {
                pobj->voltage[id][i] = pobj->adc[id][i] * 3.3F / 4096.0F;
            }
        }
    }
}

static void dev_adc_regular_update(struct dev_adc_regular *pobj)
{
    assert_report(pobj != NULL);

    // step 1: 读取adc数据
    dev_adc_regular_get_value(pobj);

    // step 2: adc转为电压数据
    dev_adc_regular_get_voltage(pobj);

}

static float dev_adc_get_en_pr(struct dev_adc_regular *pobj)
{
//     assert_report(pobj != NULL);
//     pobj->enpr = pobj->voltage[DRV_ADC_1][ADCX_EN_PR] * 11.0F;
//     return pobj->enpr;
}

static float dev_adc_get_vbus(struct dev_adc_regular *pobj)
{
    assert_report(pobj != NULL);
    pobj->vbus = pobj->voltage[DRV_ADC_1][ADCX_VBUS] * 11.0F;
    return pobj->vbus;
}

static float dev_adc_get_ibus(struct dev_adc_regular *pobj)
{
    assert_report(pobj != NULL);
    pobj->ibus = pobj->voltage[DRV_ADC_1][ADCX_IBUS] * 11.0F;
    return pobj->ibus;
}

static float dev_adc_get_temp_driver(struct dev_adc_regular *pobj)
{
    // assert_report(pobj != NULL);
    // pobj->temp_driver = pobj->voltage[DRV_ADC_1][ADCX_TEMP_DRIVER] * 11.0F;
    // return pobj->temp_driver;
}

static float dev_adc_get_temp_motor(struct dev_adc_regular *pobj)
{
    // assert_report(pobj != NULL);
    // pobj->temp_motor = pobj->voltage[DRV_ADC_1][ADCX_TEMP_MOTOR] * 11.0F;
    // return pobj->temp_motor;
}

void dev_adc_regular_init(struct dev_adc_regular *pobj)
{
    assert_report(pobj != NULL);
    pobj->start = dev_adc_regular_start;
    pobj->update = dev_adc_regular_update;

    pobj->get_enpr = dev_adc_get_en_pr;

    pobj->get_vbus = dev_adc_get_vbus;

    pobj->get_ibus = dev_adc_get_ibus;

    pobj->get_temp_driver = dev_adc_get_temp_driver;

    pobj->get_temp_motor = dev_adc_get_temp_motor;
}
