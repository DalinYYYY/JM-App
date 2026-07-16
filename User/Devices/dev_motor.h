/**
 * @file        dev_motor.h
 * @brief 		电机实例化：编码器+多圈计数+FOC+PWM+相电流采样
 * 
 * @author      Dalin (dalinyy@163.com)
 * @version     1.0
 * @date        2026-06-17
 * 
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 * 
 * 
 * @par 修改日志:
 * | 日期       | 版本 | 作者   | 修改内容   |
 * |------------|------|--------|------------|
 * | 2026-06-17     | 1.0  | yangsl | 初始创建   |
 * 
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 */
#ifndef __DEV_MOTOR_H__
#define __DEV_MOTOR_H__

#include <stdint.h>
#include "dev_encoder.h" /* dev_encoder_t 抽象接口（替代内联定义）*/
#include "foc_core.h"
#include "dev_power_monitor.h"
#include "dev_half_bridge.h"
#include "motion_param.h"
#include "multiturn_counter.h"
#include "dev_motor_phase_current.h"

/*============================================================================
 * 编码器型号选择
 *   控制层只面向 dev_encoder_t 抽象接口，与型号无关；具体用哪颗芯片由本宏决定。
 *   切换编码器：只改本宏值，dev_motor_init 内的初始化/装配按宏条件编译，
 *==========================================================================*/
#define DEV_MOTOR_ENCODER_MT6701 1
#define DEV_MOTOR_ENCODER_MT6835 2
#define DEV_MOTOR_ENCODER_AS5047 3

#ifndef DEV_MOTOR_ENCODER_TYPE
#define DEV_MOTOR_ENCODER_TYPE DEV_MOTOR_ENCODER_AS5047
#endif

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

/* dev_encoder_t 已移至 dev_encoder.h，此处不再重复定义 */

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
	dev_encoder_t encoder;             // 抽象编码器（ctx 指向适配层静态实体）
	motion_param_t motor_param;        // 角度/速度转化
	multiturn_t multiturn;             // 绝对多圈计数
	foc_t foc;                         // foc
	dev_half_bridge_t half_bridge;     // dev_half_bridge
	dev_phase_current_t phase_current; // adc for current
} dev_motor_t;

void dev_motor_init(dev_motor_t *pobj, motor_id_e id,
                    focCurrent_t (*current_callback)(void),
                    float (*ele_radian_callback)(void));

/**
 * @brief 运行时翻转编码器方向(换电机/换安装后快速调试用)
 * @param pobj  电机设备对象
 * @param dir   方向: 1=CW(正向), -1=CCW(反向)
 * @note  通过抽象编码器层 set_dir 设置方向，同步更新 usr.motor_param.encoder_param.enc_direction,
 *        无需重新初始化即可生效; 已刷新的 mechanical_angle 会在下次 update 时按新方向计算。
 *        切换方向后建议同时重新校准 enc_offset(零位), 因方向反转后原零位不再有效。
 */
void dev_motor_set_encoder_dir(dev_motor_t *pobj, int8_t dir);

#endif /* __DEV_MOTOR_H__ */
