/**
 * @file        foc_core.h
 * @brief       BLDC FOC 算法核心接口定义
 *
 * @author      yangsl (yangsl@robot.com)
 * @version     1.0
 * @date        2026-06-15
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @par 修改日志:
 * | 日期       | 版本 | 作者   | 修改内容   |
 * |------------|------|--------|------------|
 * | 2026-06-15 | 1.0  | yangsl | 初始创建   |
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 */

#ifndef _BLDC_FOC_H
#define _BLDC_FOC_H

#include <stdint.h>

// #define IQ_MATH_ENABLE   // 启用IQ_MATH使能

// 低通滤波器系数计算
#define _lpfilter(alpha, cur_val, prev_val) ((alpha) * (cur_val) + (1 - alpha) * (prev_val))
#define PWM_PERIOD 8500.0F

typedef struct
{
	float ia;
	float ib;
	float ic;
} focCurrent_t;

typedef struct
{
	float alpha; // α轴电流
	float beta;	 // β轴电流
} alphaBeta_t;

typedef struct
{
	float d; // d轴电流
	float q; // q轴电流
} focDQ_t;

typedef struct
{
	float u_alpha; // α轴电压
	float u_beta;  // β轴电压
	int sector;

	float u1;
	float u2;
	float u3;

	float ta;
	float tb;
	float tc;

	float Ts;
	float t0;
	float t1;
	float t2;
	float t3;
	float t4;
	float t5;
	float t6;
	float t7;
} focSvpwm_t;

// 面向对象封装的回调接口说明：
// 外部输入：三相电流 Ia/Ib/Ic（ADC 采样）、电弧度 Theta、u_dq（PID 输出）
// 内部算法：clarke_transfer、park_transfer、inverse_park_transfer、foc_svpwm
typedef struct foc
{
	float Theta;
	focCurrent_t current;
	alphaBeta_t i_alphaBeta;
	alphaBeta_t u_alphaBeta;
	focDQ_t i_dq;
	focDQ_t u_dq;
	focSvpwm_t svpwm;

	float foc_sin;
	float foc_cos;

	/* public */
	/* 外部输入接口（回调函数） */
	focCurrent_t (*current_callback)(void); // 三相电流
	float (*ele_radian_callback)(void);		// 电弧度

	/* public */
	void (*clarke)(struct foc *pobj);
	void (*park)(struct foc *pobj);
	void (*inverse_park)(struct foc *pobj);
	void (*pfsvpwm)(struct foc *pobj);
	void (*set_udq)(struct foc *pobj, float ud, float uq);

	/* 标定旁路 LPF 标志：=1 时 park_transfer 不做低通滤波，直接输出原始 id/iq
	 * 用于 L2 标定期间消除 LPF(α=0.8) 对阶跃响应的延迟污染。
	 * calib_hw_enter 时置 1，calib_hw_exit 时清 0。*/
	uint8_t calib_raw_mode;
	float calib_prev_id; /* 标定期间的 id 滤波状态（替代原 static prev_id）*/
	float calib_prev_iq; /* 标定期间的 iq 滤波状态（替代原 static prev_iq）*/
} foc_t;

void foc_init(foc_t *pobj, focCurrent_t (*current_cb)(void), float (*ele_radian_cb)(void));

#endif // _BLDC_FOC_H
