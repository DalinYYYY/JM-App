/**
 * @file        foc_core.h
 * @brief       BLDC FOC 算法核心接口定义
 *
 * @details     调用顺序（电流环内每拍）：
 *                clarke → park → PI(算 ud/uq) → set_udq → inverse_park → pfsvpwm
 *              park 与 inverse_park 共享按电角度缓存的 sin/cos，同拍只算一次；
 *              若单独调用 inverse_park（如标定强制角度），其内部会自行重算，
 *              不依赖 park 先执行。
 *
 * @author      yangsl (yangsl@robot.com)
 * @version     1.1
 * @date        2026-08-18
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @par 修改日志:
 * | 日期       | 版本 | 作者   | 修改内容                                 |
 * |------------|------|--------|------------------------------------------|
 * | 2026-06-15 | 1.0  | yangsl | 初始创建                                 |
 * | 2026-08-18 | 1.1  | yangsl | 热路径优化，标注 svpwm 中间量不再更新     |
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

/* SVPWM 输出。
 * 仅 ta/tb/tc 由 pfsvpwm 更新，是唯一对外有效的字段。
 * 其余字段为历史遗留的中间量：为缩短电流环执行时间，SVPWM 已改为全程用
 * 局部变量运算，不再逐个回写，运行期恒为 0。读取它们得不到有效值，
 * 需要观测中间量请在 foc_svpwm 内部临时插桩。*/
typedef struct
{
	float ta; // A 相占空比 (0~1)
	float tb; // B 相占空比 (0~1)
	float tc; // C 相占空比 (0~1)

	/* 以下字段已废弃，运行期恒为 0，可在确认无人引用后整段删除 */
	float u_alpha;
	float u_beta;
	int sector;
	float u1;
	float u2;
	float u3;
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
	focCurrent_t current;
	alphaBeta_t i_alphaBeta;
	alphaBeta_t u_alphaBeta;
	focDQ_t i_dq;
	focDQ_t u_dq;
	focSvpwm_t svpwm;

	/* sin/cos 缓存三元组，三者必须保持自洽（Theta 为 foc_sin/foc_cos 对应的角度）。
	 * park/inverse_park 靠比对 Theta 判断能否复用，从外部改写任一字段都会破坏
	 * 该不变式，导致变换用错角度。初值由 foc_init 建立。*/
	float Theta;
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
	float calib_prev_id; /* id 的 LPF 状态（两种模式都更新）*/
	float calib_prev_iq; /* iq 的 LPF 状态（两种模式都更新）*/
} foc_t;

void foc_init(foc_t *pobj, focCurrent_t (*current_cb)(void), float (*ele_radian_cb)(void));

#endif // _BLDC_FOC_H
