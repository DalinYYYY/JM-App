/**
 * @file        dev_motor_stub.h
 * @brief       设备层 dev_motor 占位头（驱动未就绪时替代 dev_motor.h）
 * @details     真实 dev_motor.h 会拉入整条 dev/drv 底层依赖链（drv_gpio/adc/tim/
 *              spi、imath/ifilter、dev_control_signal_acq/feedforward_lpf/
 *              lf_notch_filter 等），这些底层当前尚未实现，只要被 #include 即在
 *              预处理阶段编译失败。
 *
 *              本占位头提供与 dev_motor.h 完全同名的 dev_motor_t 及其子结构、
 *              motor_id_e 枚举与 dev_motor_init 声明，但仅依赖已就绪且自包含的
 *              foc.h / motion_param.h，不触碰任何 drv_ 底层。使三环控制层
 *              （current_loop / motor_loop）在 MOTOR_LOOP_ENABLE_DEV_DRIVER==0
 *              时可独立编译并空跑算法链路。
 *
 *              字段名、类型名、函数指针签名均与真实 dev_motor.h 保持一致，
 *              驱动就绪后将 MOTOR_LOOP_ENABLE_DEV_DRIVER 置 1 切回真实头，
 *              三环控制层代码零改动。
 *
 * @note        本头仅在屏蔽态由 current_loop.h / motor_loop.h 条件包含，
 *              不应与真实 dev_motor.h 同时进入同一翻译单元。
 */

#ifndef __DEV_MOTOR_STUB_H__
#define __DEV_MOTOR_STUB_H__

#include <stdint.h>
#include "foc.h"		  // 真实且自包含：focCurrent_t / foc_t
#include "motion_param.h" // 真实且自包含：motion_param_t

/*
 * 真实 dev_motor.h 与本 stub 的互斥由 dev_motor_select.h 保证（按
 * MOTOR_LOOP_ENABLE_DEV_DRIVER 二选一包含），且设备层 dev_*.c 不在 Keil
 * 编译列表中，二者不会进入同一翻译单元。此处不再以 guard 宏强行拦截，
 * 避免与真实头 guard 命名耦合。
 */

typedef enum
{
	DEV_MOTOR_1 = 0,
	DEV_MOTOR_MAX,
} motor_id_e;

typedef struct
{
	float position_target; // 位置环目标
	float velocity_target; // 速度环目标
	float current_target;  // 电流环目标
} motor_ctrl_target_t;

/*--------------------------------------------------------------------------
 * 子设备占位结构：字段名/函数指针签名与真实 dev_*.h 对齐，
 * 仅保留三环控制层会访问到的成员，去除对 drv_ 底层的依赖。
 *------------------------------------------------------------------------*/

/** @brief 编码器占位（对应 dev_mt6701_t） */
typedef struct dev_mt6701
{
	float mechanical_angle; // 机械角度
	void (*update)(struct dev_mt6701 *pobj);
} dev_mt6701_t;

/** @brief 三相电流采样占位（对应 dev_phase_current_t） */
typedef struct
{
	float a;
	float b;
	float c;
} dev_current_f3axis_t;

typedef struct dev_adc_injected
{
	dev_current_f3axis_t current; // 三相电流(A)
	void (*update)(struct dev_adc_injected *pobj);
} dev_phase_current_t;

/** @brief 半桥 PWM 占位（对应 dev_half_bridge_t） */
typedef struct dev_half_bridge
{
	int (*set_3pwm)(struct dev_half_bridge *pobj, uint32_t ccr1, uint32_t ccr2, uint32_t ccr3);
} dev_half_bridge_t;

/** @brief 电机设备占位（对应 dev_motor_t），成员布局与真实头一致 */
typedef struct dev_motor
{
	motor_id_e id; // 电机id
	uint8_t poles; // 极对数

	focCurrent_t (*current_callback)(void);
	float (*ele_radian_callback)(void);

	motor_ctrl_target_t target; // 电机控制目标

	dev_mt6701_t mt6701;			   // 编码器
	motion_param_t motor_param;		   // 运动参数解算
	foc_t foc;						   // FOC
	dev_half_bridge_t half_bridge;	   // 半桥
	dev_phase_current_t phase_current; // 三相电流采样
} dev_motor_t;

/**
 * @brief dev_motor 初始化（占位声明，签名与真实头一致）
 * @note  屏蔽态下不会被调用（motor_loop.c 内以 MOTOR_LOOP_ENABLE_DEV_DRIVER
 *        条件编译），此处仅保证 API 签名一致与编译期可见。
 */
void dev_motor_init(dev_motor_t *pobj, motor_id_e id,
					focCurrent_t (*current_callback)(void),
					float (*ele_radian_callback)(void));

#endif /* __DEV_MOTOR_STUB_H__ */
