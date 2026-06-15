/**
 * @file        foc.h
 * @brief       BLDC FOC算法核心接口定义
 * 
 * @author      name (name@robot.com)
 * @version     1.0
 * @date        2026-06-15
 * 
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 * 
 * 
 * @par 修改日志:
 * | 日期       | 版本 | 作者   | 修改内容   |
 * |------------|------|--------|------------|
 * | 2026-06-15     | 1.0  | yangsl | 初始创建   |
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
	float alpha; // alpha-axis current
	float beta;	 // beta-axis current
} alphaBeta_t;

typedef struct
{
	float d; // d-axis current
	float q; // q-axis current
} focDQ_t;

typedef struct
{
	float u_alpha; // alpha-axis current
	float u_beta;  // beta-axis current
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

// oop
/* 
 * 外部输入接口（回调函数）
 * 1、ia ib ic三相电流（来源adc采样）
 * 2、Theta（电弧度）
 * 3、u_dq数据（来源i_dq经过pid运算后的结果）
 * 
 * 外部访问接口（函数）
 * 1、clarke_transfer
 * 2、park_transfer
 * 3、inverse_park_transfer
 * 4、foc_svpwm
 */
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
} foc_t;

void foc_init(foc_t *pobj, focCurrent_t (*current_cb)(void), float (*ele_radian_cb)(void));

#endif // _BLDC_FOC_H
