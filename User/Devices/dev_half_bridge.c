
/**
 * @file dev_half_bridge.c
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

#include "dev_half_bridge.h"

#if defined(USE_DEV_HALF_BRIDGE)
#include "assert_report.h"
#include "mlog.h"

static const dev_half_bridge_config_t half_bridge_config[BRIDGE_ID_MAX] = 
{
    {
        .name = "device_1", 
        .tim = DRV_TIM1, 
        .channel = { TIM_CH1, TIM_CH2, TIM_CH3, TIM_CH4 },
    },
};

static void dev_half_bridge_start(struct dev_half_bridge *pobj)
{
    assert_report(pobj != NULL);
    half_bridge_id_e id = pobj->id;

    // 启动定时器PWM
    drv_pwm_start(half_bridge_config[id].tim, half_bridge_config[id].channel[0]);
    drv_pwm_start(half_bridge_config[id].tim, half_bridge_config[id].channel[1]);
    drv_pwm_start(half_bridge_config[id].tim, half_bridge_config[id].channel[2]);

    // 启动定时器PWM互补
    drv_pwmN_start(half_bridge_config[id].tim, half_bridge_config[id].channel[0]);
    drv_pwmN_start(half_bridge_config[id].tim, half_bridge_config[id].channel[1]);
    drv_pwmN_start(half_bridge_config[id].tim, half_bridge_config[id].channel[2]);

    drv_pwm_start(half_bridge_config[id].tim, half_bridge_config[id].channel[3]);
    drv_pwm_set_dutycycle(half_bridge_config[id].tim, half_bridge_config[id].channel[3], 8380);

    drv_tim_start_it(half_bridge_config[id].tim);    // 启动定时器（中断模式下）
    pobj->tim = half_bridge_config[id].tim;
}

static void dev_half_bridge_stop(struct dev_half_bridge *pobj)
{
    assert_report(pobj != NULL);
    half_bridge_id_e id = pobj->id;

    // 关闭定时器PWM
    drv_pwm_stop(half_bridge_config[id].tim, half_bridge_config[id].channel[0]);
    drv_pwm_stop(half_bridge_config[id].tim, half_bridge_config[id].channel[1]);
    drv_pwm_stop(half_bridge_config[id].tim, half_bridge_config[id].channel[2]);

    // 关闭定时器PWM互补
    drv_pwmN_stop(half_bridge_config[id].tim, half_bridge_config[id].channel[0]);
    drv_pwmN_stop(half_bridge_config[id].tim, half_bridge_config[id].channel[1]);
    drv_pwmN_stop(half_bridge_config[id].tim, half_bridge_config[id].channel[2]);

    // drv_pwm_start(half_bridge_config[id].tim, half_bridge_config[id].channel[3]);
    // drv_pwm_set_dutycycle(half_bridge_config[id].tim, half_bridge_config[id].channel[3], 8380);

    // drv_tim_start_it(half_bridge_config[id].tim);    // 启动定时器（中断模式下）
    // pobj->tim = half_bridge_config[id].tim;
}

static int dev_half_bridge_pwmA_dutycycle(struct dev_half_bridge *pobj, uint32_t duty)
{
    return (int)drv_pwm_set_dutycycle(  half_bridge_config[pobj->id].tim, \
                                        half_bridge_config[pobj->id].channel[0], duty);
}

static int dev_half_bridge_pwmB_dutycycle(struct dev_half_bridge *pobj, uint32_t duty)
{
    return (int)drv_pwm_set_dutycycle(  half_bridge_config[pobj->id].tim, \
                                        half_bridge_config[pobj->id].channel[1], duty);
}

static int dev_half_bridge_pwmC_dutycycle(struct dev_half_bridge *pobj, uint32_t duty)
{
    return (int)drv_pwm_set_dutycycle(  half_bridge_config[pobj->id].tim, \
                                        half_bridge_config[pobj->id].channel[2], duty);
}

uint8_t control_out_enable = 1;
static int dev_half_bridge_set_3pwm(struct dev_half_bridge *pobj, uint32_t ccr1, \
                                                                  uint32_t ccr2, \
                                                                  uint32_t ccr3)
{
    assert_report(pobj != NULL);
    half_bridge_id_e id = pobj->id; 
    uint16_t autoreload;
    int status = DEV_EOK;
    /* get timer autoreload */
    status |= drv_tim_get_autoreload(half_bridge_config[id].tim, &autoreload);

    /* limit pwm duty cycle */
    ccr1 = (ccr1 > autoreload) ? autoreload : ccr1;
    ccr2 = (ccr2 > autoreload) ? autoreload : ccr2;
    ccr3 = (ccr3 > autoreload) ? autoreload : ccr3;

    /* set pwm duty cycle */
    if(control_out_enable)
    {
        status |= dev_half_bridge_pwmA_dutycycle(pobj, ccr1);
        status |= dev_half_bridge_pwmB_dutycycle(pobj, ccr2);
        status |= dev_half_bridge_pwmC_dutycycle(pobj, ccr3);
    }
    else
    {
        status |= dev_half_bridge_pwmA_dutycycle(pobj, 0);
        status |= dev_half_bridge_pwmB_dutycycle(pobj, 0);
        status |= dev_half_bridge_pwmC_dutycycle(pobj, 0);
    }


    pobj->autoreload = autoreload;


    pobj->ccr[0] = ccr1;
    pobj->ccr[1] = ccr2;
    pobj->ccr[2] = ccr3;

    
    return status;
}

void dev_half_bridge_init(dev_half_bridge_t *pobj, half_bridge_id_e id)
{
    assert_report(pobj != NULL);
    assert_report(id < BRIDGE_ID_MAX);

    pobj->id = id;
    pobj->start = dev_half_bridge_start;
    pobj->stop = dev_half_bridge_stop;
    pobj->set_3pwm = dev_half_bridge_set_3pwm;
}


#endif /* USE_DEV_HALF_BRIDGE */

