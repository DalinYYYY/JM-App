/**
 * @file dev_motor.c
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

#include "dev_motor.h"
#include "assert_report.h"
#include "dev_motorinfo.h"

ffc_lpf_filter_t ffc_filter;
// notch_filter_t notch_filter;
lf_notch_filter_t lf_notch_filter;

dev_motor_enable_config_t motor_enable_list[DEV_MOTOR_MAX] = {
    {"MOTOR1_EN", {(gpioType_e)DRV_GPIOB, (gpioPin_e)DRV_PIN_2, (drvPinState_e)0}},
    // {"MOTOR1_EN", {(gpioType_e)DRV_GPIOB, (gpioPin_e)DRV_PIN_2, (drvPinState_e)0}},
};

static void dev_motor_enable(void)
{
    drv_gpio_write(motor_enable_list[DEV_MOTOR_1].gpio, (drvPinState_e)1);
}

static uint8_t dev_motor_search_poles(dev_motor_t *pobj)
{
}

//
float device_compensation(void)
{
    return motor_info.device.device_compensation;
}

// 待增加自动获取电机极对数函数
static uint8_t dev_motor_get_poles(motor_id_e id)
{
    uint8_t poles = 0;
    switch (id)
    {
        case DEV_MOTOR_1: poles = 7; break;
        // case DEV_MOTOR_2: poles = 3;    break;
        default: poles = 7; break;
    }
    return poles;
}

void dev_motor_init(dev_motor_t *pobj,
                    motor_id_e id,
                    focCurrent_t (*current_callback)(void),
                    float (*ele_radian_callback)(void))
{
    assert_report(pobj != NULL);
    assert_report(&pobj->mt6835 != NULL);
    assert_report(&pobj->foc != NULL);
    assert_report(current_callback != NULL);
    assert_report(ele_radian_callback != NULL);
    assert_report(id < DEV_MOTOR_MAX);
    memset(pobj, 0, sizeof(dev_motor_t));

    pobj->id = id;
    pobj->poles = dev_motor_get_poles((motor_id_e)id);

    pobj->fsm_tim = DRV_TIM2; // 定时器2

    // 初始化编码器
    dev_mt6701_init(&pobj->mt6701, (mt6701_id_e)id);
    pobj->mt6701.set_zero_angle(&pobj->mt6701, usr.motor[id].encoder_param.encoder_offset);
    if(usr.motor[id].encoder_param.change_dir)
        pobj->mt6701.set_dir(&pobj->mt6701, MT6701_DIR_CCW);
    else
        pobj->mt6701.set_dir(&pobj->mt6701, MT6701_DIR_CW);

//    dev_mt6835_init(&pobj->mt6835, (mt6835_id_e)id);
    // pobj->mt6835.set_offset(&pobj->mt6835, usr.motor[id].encoder_param.encoder_offset);
    // pobj->mt6835.set_dir(&pobj->mt6835, usr.motor[id].encoder_param.change_dir);

    // 初始化角度转化器
    motion_param_init(&pobj->motor_param, pobj->poles, 10, device_compensation);

    // 初始化控制信号采集器
    // dev_control_signal_acq_init(&pobj->acq);
    //   pobj->acq.update(&pobj->acq);

    // 初始化半桥驱动器
    dev_half_bridge_init(&pobj->half_bridge, (half_bridge_id_e)id);

    // 初始化三相adc电流采样
    dev_phase_current_init(&pobj->phase_current, 50.0F, 0.01F);                             // 采样增益倍数 10 和采样电阻 0.01欧姆
    pobj->phase_current.set_offset(&pobj->phase_current, (dev_current_i3axis_t){1660, 1660, 1660}); // 电流零位时adc采样值偏移

    // 初始化FOC
    pobj->current_callback = current_callback;
    pobj->ele_radian_callback = ele_radian_callback;
    foc_init(&pobj->foc, pobj->current_callback, pobj->ele_radian_callback);

    dev_motor_enable();

    //初始化滤波器
    ffc_lpfilter_init(&ffc_filter); //低通滤波器
    // notch_filter_init(&notch_filter);//Notch filter滤波器
    lf_notch_filter_init(&lf_notch_filter); //Notch filter滤波器
}
