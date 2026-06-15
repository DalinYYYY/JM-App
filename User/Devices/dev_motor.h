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
#include "foc_core.h"
#include "dev_adc.h"
#include "dev_half_bridge.h"
#include "dev_mt6835.h"
#include "motion_param.h"
#include "multiturn_counter.h"
#include "dev_control_signal_acq.h"
#include "dev_motor_phase_current.h"

#include "feedforward_lpf.h"
//#include "notch_filter.h"
#include "lf_notch_filter.h"

/*============================================================================
 * 编码器型号选择
 *   控制层只面向 dev_encoder_t 抽象接口，与型号无关；具体用哪颗芯片由本宏决定。
 *   切换编码器：只改本宏值，dev_motor_init 内的初始化/装配按宏条件编译，
 *   控制层与三环代码零改动。
 *==========================================================================*/
#define DEV_MOTOR_ENCODER_MT6701 1
#define DEV_MOTOR_ENCODER_MT6835 2

#ifndef DEV_MOTOR_ENCODER_TYPE
#define DEV_MOTOR_ENCODER_TYPE DEV_MOTOR_ENCODER_MT6835
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

/**
 * @brief 抽象编码器接口（与具体芯片型号无关）
 * @details 控制层只面向本接口，不感知背后是 MT6701 / MT6835 / AS5047 等。
 *          dev_motor_init 把选定的具体编码器对象（&mt6701 / &mt6835 ...）绑定到
 *          ctx，并将其 update / get_mechanical_angle 适配到本接口。
 *          调用约定：先 update(self) 刷新，再读 self->mechanical_angle，
 *          或调 get_mechanical_angle(self)。
 *          注：本结构与 dev_motor_virtual.h 中的同名定义保持布局一致（二选一编译）。
 */
typedef struct dev_encoder
{
	void *ctx;				// 指向具体编码器对象（dev_mt6701_t* / dev_mt6835_t* ...）
	float mechanical_angle; // 最新机械角度(deg)，update 后刷新
	void (*update)(struct dev_encoder *pobj);
	float (*get_mechanical_angle)(struct dev_encoder *pobj);
} dev_encoder_t;

typedef struct dev_motor
{
	motor_id_e id; // 电机id
	uint8_t poles; // 极对数

	/* 外部输入回调函数 */
	focCurrent_t (*current_callback)(void);
	float (*ele_radian_callback)(void);

	motor_ctrl_target_t target; // 电机控制目标
	timNumber_e fsm_tim;		// 状态机定时器

	/* public */
	dev_encoder_t encoder; // 抽象编码器（型号无关，控制层入口）
	dev_mt6701_t mt6701;   // mt6701（具体芯片实体，由 encoder.ctx 绑定）
	dev_mt6835_t mt6835;   // mt6835（具体芯片实体，由 encoder.ctx 绑定）

	motion_param_t motor_param;		   // motor_param
	multiturn_t multiturn;			   // 绝对多圈计数
	dev_control_signal_acq_t acq;	   // control signal acquisition
	foc_t foc;						   // foc
	dev_half_bridge_t half_bridge;	   // dev_half_bridge
	dev_phase_current_t phase_current; // adc for current
} dev_motor_t;

void dev_motor_init(dev_motor_t *pobj, motor_id_e id,
					focCurrent_t (*current_callback)(void),
					float (*ele_radian_callback)(void));

extern ffc_lpf_filter_t ffc_filter; //低通滤波器
// extern notch_filter_t notch_filter;                 //带阻滤波器
extern lf_notch_filter_t lf_notch_filter; //带阻滤波器

#endif /* __DEV_MOTOR_H__ */
