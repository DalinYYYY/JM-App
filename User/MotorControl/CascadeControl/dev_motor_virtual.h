/**
 * @file        dev_motor_virtual.h
 * @brief       虚拟电机设备（在环 dq 物理模型，伪装 dev_motor_t）
 * @details     在没有真实硬件时，用一个完整的 dq PMSM 物理模型伪装成 dev_motor_t，
 *              让 FOC 三环（current_loop / motor_loop）无需修改即可闭环运行、调参。
 *
 *              本头提供与真实 dev_motor.h 完全同名、同布局的 dev_motor_t 及其子结构、
 *              motor_id_e 枚举与 dev_motor_init 声明，但：
 *                - 不依赖任何未就绪的 drv_ 底层驱动
 *                - foc 复用真实 foc.c 的算法（dev_motor_init 内调 foc_init 装配），
 *                  虚拟与真实电机共享同一套 FOC，foc.c 优化后两者自动同步
 *                - motor_param 复用真实 motion_param.h 的 motion_param_t，仅覆盖其
 *                  update / get_position 等函数指针为虚拟实现
 *
 *              切换由 motor_loop_config.h 的 MOTOR_LOOP_ENABLE_DEV_DRIVER 决定：
 *                - 0：dev_motor_select.h 包含本头，dev_motor_init 装配虚拟模型
 *                - 1：包含真实 dev_motor.h，本头与 dev_motor_virtual.c 均不参与
 *
 *              物理模型（每个电流环 tick 推进一步）：
 *                电气：did/dt=(ud-Rs*id+we*Lq*iq)/Ld
 *                      diq/dt=(uq-Rs*iq-we*Ld*id-we*flux)/Lq,  we=pole*omega
 *                转矩：Te=1.5*pole*(flux*iq+(Ld-Lq)*id*iq)
 *                机械：domega/dt=(Te-Tload-Tfric)/inertia, dtheta/dt=omega
 *                反馈：由 id/iq + 电角度 反Park→反Clarke 生成三相电流，
 *                      使 Clarke/Park 链路真实参与；电角度=wrap(pole*theta)。
 *
 * @author      yangsl
 * @version     1.0
 * @date        2026-06-12
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @note        本头与真实 dev_motor.h 不应进入同一翻译单元（由 dev_motor_select.h
 *              二选一包含保证）。
 */

#ifndef __DEV_MOTOR_VIRTUAL_H__
#define __DEV_MOTOR_VIRTUAL_H__

#include <stdint.h>
#include "foc.h"		  // 真实 foc_t / focCurrent_t（虚拟实现复用其类型）
#include "motion_param.h" // 真实 motion_param_t（虚拟实现复用其类型）

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
 * 仅保留三环控制层会访问到的成员。
 *------------------------------------------------------------------------*/

/**
 * @brief 抽象编码器接口（与具体芯片型号无关）
 * @details 控制层只面向本接口，不感知背后是 MT6701 / MT6835 / AS5047 等。
 *          真实模式下由 dev_motor_init 把具体编码器对象绑定到 ctx，并将其
 *          update / get_mechanical_angle 适配到本接口；虚拟模式下由物理模型实现。
 *          调用约定：先 update(self) 刷新，再读 self->mechanical_angle，
 *          或调 get_mechanical_angle(self)。
 */
typedef struct dev_encoder
{
	void *ctx;				// 指向具体编码器对象（dev_mt6701_t* / dev_mt6835_t* ...），虚拟模式可为 NULL
	float mechanical_angle; // 最新机械角度(deg)，update 后刷新
	void (*update)(struct dev_encoder *pobj);
	float (*get_mechanical_angle)(struct dev_encoder *pobj);
} dev_encoder_t;

/** @brief 三相电流值（对应 dev_current_f3axis_t） */
typedef struct
{
	float a;
	float b;
	float c;
} dev_current_f3axis_t;

/** @brief 三相电流采样占位（对应 dev_phase_current_t） */
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

/**
 * @brief 虚拟电机物理模型状态（调试可观测）
 */
typedef struct
{
	/* 配置参数（init 时从 motor_param 读取，Rs 取本地默认） */
	float Rs;		  // 定子电阻(Ω) —— motor_param 无此项，用本地默认
	float Ld;		  // d 轴电感(H)
	float Lq;		  // q 轴电感(H)
	float flux;		  // 永磁磁链(Wb)
	float inertia;	  // 转动惯量(kg·m²)
	float fric_visc;  // 粘滞摩擦系数(Nm/(rad/s))
	float fric_coul;  // 库仑摩擦力矩(Nm)
	uint8_t poles;	  // 极对数
	float dt;		  // 积分步长(s) = 电流环周期
	uint8_t substeps; // 每 tick 子步细分数（数值稳定）

	/* 运行状态 */
	float id;	   // d 轴实际电流(A)
	float iq;	   // q 轴实际电流(A)
	float omega;   // 机械角速度(rad/s)
	float theta_m; // 机械角度(rad，连续累计)
	float theta_e; // 电角度(rad，[0,2π))
	float Te;	   // 电磁转矩(Nm)
	float Tload;   // 外部负载转矩(Nm，默认 0，可手动注入)
} virtual_motor_model_t;

/**
 * @brief 电机设备（虚拟实现，成员布局与真实 dev_motor_t 对齐）
 */
typedef struct dev_motor
{
	motor_id_e id; // 电机id
	uint8_t poles; // 极对数

	focCurrent_t (*current_callback)(void);
	float (*ele_radian_callback)(void);

	motor_ctrl_target_t target; // 电机控制目标

	dev_encoder_t encoder;			   // 抽象编码器（型号无关）
	motion_param_t motor_param;		   // 运动参数解算
	foc_t foc;						   // FOC（函数指针指向虚拟实现）
	dev_half_bridge_t half_bridge;	   // 半桥（set_3pwm 触发物理积分）
	dev_phase_current_t phase_current; // 三相电流采样（输出模型电流）

	virtual_motor_model_t model; // 虚拟物理模型状态
} dev_motor_t;

/**
 * @brief dev_motor 初始化（虚拟版，签名与真实头一致）
 * @param pobj 电机设备对象
 * @param id 电机 id
 * @param current_callback 三相电流回调（FOC clarke 取电流）
 * @param ele_radian_callback 电角度回调（FOC park 取角度）
 * @note 调 foc_init 装配真实 FOC 算法、装配运动量函数指针，并从 usr.motor_param 读取物理参数。
 */
void dev_motor_init(dev_motor_t *pobj, motor_id_e id,
					focCurrent_t (*current_callback)(void),
					float (*ele_radian_callback)(void));

/**
 * @brief 设置虚拟模型积分步长（电流环周期）
 * @param pobj 电机设备对象
 * @param dt 步长(s)
 * @note 由 motor_loop_init 用实际电流环周期调用，保证仿真 dt 与控制 dt 一致。
 */
void virtual_motor_set_period(dev_motor_t *pobj, float dt);

/**
 * @brief 注入外部负载转矩（调试用）
 * @param pobj 电机设备对象
 * @param load_nm 负载转矩(Nm)
 */
void virtual_motor_set_load(dev_motor_t *pobj, float load_nm);

#endif /* __DEV_MOTOR_VIRTUAL_H__ */
