/**
 * @file        foc_core.c
 * @brief 		FOC算法实现
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
#include "foc_core.h"
#include <stdio.h>
#include "utils.h"

//#define IQ_MATH_ENABLE 0
#define DSP_MATH_ENABLE 1

#ifdef IQ_MATH_ENABLE
#include "IQmathLib.h"
#elif DSP_MATH_ENABLE
//#include "dsp/fast_math_functions.h"
#include "arm_math.h"
#else
#include <math.h>
#endif

/*
  * @brief  等幅值Clarke变换：
  * 将三相电流(Ia, Ib, Ic)转换为两相α-β坐标系中的电流(Ialpha, Ibeta)
  * Iα = Ia
  * Iβ = (Ia + 2Ib) / sqrt(3)
  */
static void clarke_transfer(struct foc *pobj)
{
	pobj->current = pobj->current_callback(); /* 获取当前三相电流 */

	/*三相平衡时使用：两项*/
	//    pobj->i_alphaBeta.alpha = pobj->current.ia ;
	//    pobj->i_alphaBeta.beta = (pobj->current.ia + 2*pobj->current.ib) * ONE_BY_SQRT3;

	// 等幅值Clarke变换公式
	pobj->i_alphaBeta.alpha = (2.0f / 3.0f) * (pobj->current.ia - 0.5f * pobj->current.ib - 0.5f * pobj->current.ic);
	pobj->i_alphaBeta.beta = (2.0f / 3.0f) * ((pobj->current.ib - pobj->current.ic) * SQRT3_BY_2);
}

/*
  * @brief  Park变换: 
  * 将电流向量从直角坐标系转换到极坐标系(输入电角度、Ialpha和Ibeta，经过Park变换得到Iq、Id)
  * Id =  Iα · cosθ + Iβ · sinθ
  * Iq = -Iα · sinθ + Iβ · cosθ
  */
static void park_transfer(struct foc *pobj)
{
	static float prev_id, prev_iq;
	pobj->Theta = pobj->ele_radian_callback(); // 获取当前电角度

	/* park变换 */
#ifdef IQ_MATH_ENABLE
	pobj->i_dq.d = pobj->i_alphaBeta.alpha * _IQ28toF(_IQ28cos(_IQ28(pobj->Theta))) + pobj->i_alphaBeta.beta * _IQ28toF(_IQ28sin(_IQ28(pobj->Theta)));
	pobj->i_dq.q = -pobj->i_alphaBeta.alpha * _IQ28toF(_IQ28sin(_IQ28(pobj->Theta))) + pobj->i_alphaBeta.beta * _IQ28toF(_IQ28cos(_IQ28(pobj->Theta)));
#elif DSP_MATH_ENABLE
	pobj->foc_sin = arm_sin_f32(pobj->Theta);
	pobj->foc_cos = arm_cos_f32(pobj->Theta);
	pobj->i_dq.d = pobj->i_alphaBeta.alpha * pobj->foc_cos + pobj->i_alphaBeta.beta * pobj->foc_sin;
	pobj->i_dq.q = -pobj->i_alphaBeta.alpha * pobj->foc_sin + pobj->i_alphaBeta.beta * pobj->foc_cos;
#else
	pobj->i_dq.d = pobj->i_alphaBeta.alpha * pobj->foc_cos + pobj->i_alphaBeta.beta * pobj->foc_sin;
	pobj->i_dq.q = -pobj->i_alphaBeta.alpha * pobj->foc_sin + pobj->i_alphaBeta.beta * pobj->foc_cos;
#endif // IQ_MATH_ENABLE

	/* 滤波 */
	pobj->i_dq.d = _lpfilter(0.8F, pobj->i_dq.d, prev_id);
	pobj->i_dq.q = _lpfilter(0.8F, pobj->i_dq.q, prev_iq);

	prev_id = pobj->i_dq.d;
	prev_iq = pobj->i_dq.q;
}

/*
  * @brief  Park逆变换: 
  * 将d轴和q轴的模值转换为α轴和β轴的模值(输入Uq、Ud得到Ualpha、Ubeta).
  * Uα = Ud · cosθ - Uq · sinθ
  * Uβ = Ud · sinθ + Uq · cosθ
  */
static void inverse_park_transfer(struct foc *pobj)
{
	pobj->Theta = pobj->ele_radian_callback(); // 获取当前电角度
											   // float sin_theta;
											   // float cos_theta;

#ifdef IQ_MATH_ENABLE
	pobj->u_alphaBeta.alpha = _IQ28toF(_IQ28mpy(_IQ28cos(_IQ28(pobj->Theta)), _IQ28(pobj->u_dq.d)) - _IQ28mpy(_IQ28sin(_IQ28(pobj->Theta)), _IQ28(pobj->u_dq.q)));
	pobj->u_alphaBeta.beta = _IQ28toF(_IQ28mpy(_IQ28sin(_IQ28(pobj->Theta)), _IQ28(pobj->u_dq.d)) + _IQ28mpy(_IQ28cos(_IQ28(pobj->Theta)), _IQ28(pobj->u_dq.q)));
#elif DSP_MATH_ENABLE
	pobj->foc_sin = arm_sin_f32(pobj->Theta);
	pobj->foc_cos = arm_cos_f32(pobj->Theta);
	pobj->u_alphaBeta.alpha = pobj->u_dq.d * pobj->foc_cos - pobj->u_dq.q * pobj->foc_sin;
	pobj->u_alphaBeta.beta = pobj->u_dq.d * pobj->foc_sin + pobj->u_dq.q * pobj->foc_cos;
#else
	pobj->u_alphaBeta.alpha = pobj->u_dq.d * pobj->foc_cos - pobj->u_dq.q * pobj->foc_sin;
	pobj->u_alphaBeta.beta = pobj->u_dq.d * pobj->foc_sin + pobj->u_dq.q * pobj->foc_cos;
#endif // IQ_MATH_ENABLE
}

static void set_udq(struct foc *pobj, float ud, float uq)
{
	pobj->u_dq.d = ud;
	pobj->u_dq.q = uq;
}

/*
 * @brief SVPWM算法
 * @param pobj foc结构体指针对象
 */
static void foc_svpwm(struct foc *pobj)
{
	float sum, k_svpwm;

	// SVPWM的采样周期，周期归一化处理为1，
	// 得到的ta、tb、tc时间（占空比）乘上PWM周期值 = pwm的ccr输出值
	pobj->svpwm.Ts = 1.0f;
	pobj->svpwm.u_alpha = pobj->u_alphaBeta.alpha;
	pobj->svpwm.u_beta = pobj->u_alphaBeta.beta;

	// step1 计算u1、u2和u3 , 计算SVPWM算法中的三个控制电压u1、u2和u3
	pobj->svpwm.u1 = pobj->svpwm.u_beta;

#ifdef IQ_MATH_ENABLE
	pobj->svpwm.u2 = _IQ28toF(_IQ28mpy(_IQ28(SQRT3_BY_2), _IQ28(pobj->svpwm.u_alpha)) - _IQ28mpy(_IQ28(0.5F), _IQ28(pobj->svpwm.u_beta)));
	pobj->svpwm.u3 = _IQ28toF(_IQ28mpy(_IQ28(-SQRT3_BY_2), _IQ28(pobj->svpwm.u_alpha)) - _IQ28mpy(_IQ28(0.5F), _IQ28(pobj->svpwm.u_beta)));
#else
	pobj->svpwm.u2 = (SQRT3_BY_2)*pobj->svpwm.u_alpha - 0.5F * pobj->svpwm.u_beta;
	pobj->svpwm.u3 = (-SQRT3_BY_2) * pobj->svpwm.u_alpha - 0.5F * pobj->svpwm.u_beta;
#endif // IQ_MATH_ENABLE

	// step2：扇区判断 , 根据u1、u2和u3的正负情况确定所处的扇区 N = 4*C + 2*B + A
	pobj->svpwm.sector = ((pobj->svpwm.u1 > 0.0F) ? 1 : 0) + ((pobj->svpwm.u2 > 0.0F) ? (1 << 1) : 0) + ((pobj->svpwm.u3 > 0.0F) ? (1 << 2) : 0);

	// step3:计算基本矢量电压作用时间（占空比）, 根据扇区的不同，计算对应的ta、tb和tc的值，表示生成的三相电压的时间
	switch (pobj->svpwm.sector)
	{
		case 3:
			pobj->svpwm.t4 = pobj->svpwm.u2;
			pobj->svpwm.t6 = pobj->svpwm.u1;
			sum = pobj->svpwm.t4 + pobj->svpwm.t6;
			if (sum > pobj->svpwm.Ts)
			{
				k_svpwm = pobj->svpwm.Ts / sum; // 计算缩放系数
				pobj->svpwm.t4 = k_svpwm * pobj->svpwm.t4;
				pobj->svpwm.t6 = k_svpwm * pobj->svpwm.t6;
			}
			pobj->svpwm.t0 = (pobj->svpwm.Ts - pobj->svpwm.t4 - pobj->svpwm.t6) / 2;
			pobj->svpwm.ta = pobj->svpwm.t4 + pobj->svpwm.t6 + pobj->svpwm.t0;
			pobj->svpwm.tb = pobj->svpwm.t6 + pobj->svpwm.t0;
			pobj->svpwm.tc = pobj->svpwm.t0;
			break;
		case 1:
			pobj->svpwm.t6 = -pobj->svpwm.u3;
			pobj->svpwm.t2 = -pobj->svpwm.u2;
			sum = pobj->svpwm.t2 + pobj->svpwm.t6;
			if (sum > pobj->svpwm.Ts)
			{
				k_svpwm = pobj->svpwm.Ts / sum;
				pobj->svpwm.t2 = k_svpwm * pobj->svpwm.t2;
				pobj->svpwm.t6 = k_svpwm * pobj->svpwm.t6;
			}
			pobj->svpwm.t0 = (pobj->svpwm.Ts - pobj->svpwm.t2 - pobj->svpwm.t6) / 2;
			pobj->svpwm.ta = pobj->svpwm.t6 + pobj->svpwm.t0;
			pobj->svpwm.tb = pobj->svpwm.t2 + pobj->svpwm.t6 + pobj->svpwm.t0;
			pobj->svpwm.tc = pobj->svpwm.t0;
			break;
		case 5:
			pobj->svpwm.t2 = pobj->svpwm.u1;
			pobj->svpwm.t3 = pobj->svpwm.u3;
			sum = pobj->svpwm.t2 + pobj->svpwm.t3;
			if (sum > pobj->svpwm.Ts)
			{
				k_svpwm = pobj->svpwm.Ts / sum;
				pobj->svpwm.t2 = k_svpwm * pobj->svpwm.t2;
				pobj->svpwm.t3 = k_svpwm * pobj->svpwm.t3;
			}
			pobj->svpwm.t0 = (pobj->svpwm.Ts - pobj->svpwm.t2 - pobj->svpwm.t3) / 2;
			pobj->svpwm.ta = pobj->svpwm.t0;
			pobj->svpwm.tb = pobj->svpwm.t2 + pobj->svpwm.t3 + pobj->svpwm.t0;
			pobj->svpwm.tc = pobj->svpwm.t3 + pobj->svpwm.t0;
			break;

		case 4:
			pobj->svpwm.t3 = -pobj->svpwm.u2;
			pobj->svpwm.t1 = -pobj->svpwm.u1;
			sum = pobj->svpwm.t1 + pobj->svpwm.t3;
			if (sum > pobj->svpwm.Ts)
			{
				k_svpwm = pobj->svpwm.Ts / sum;
				pobj->svpwm.t1 = k_svpwm * pobj->svpwm.t1;
				pobj->svpwm.t3 = k_svpwm * pobj->svpwm.t3;
			}
			pobj->svpwm.t0 = (pobj->svpwm.Ts - pobj->svpwm.t1 - pobj->svpwm.t3) / 2;
			pobj->svpwm.ta = pobj->svpwm.t0;
			pobj->svpwm.tb = pobj->svpwm.t3 + pobj->svpwm.t0;
			pobj->svpwm.tc = pobj->svpwm.t1 + pobj->svpwm.t3 + pobj->svpwm.t0;
			break;
		case 6:
			pobj->svpwm.t1 = pobj->svpwm.u3;
			pobj->svpwm.t5 = pobj->svpwm.u2;
			sum = pobj->svpwm.t1 + pobj->svpwm.t5;
			if (sum > pobj->svpwm.Ts)
			{
				k_svpwm = pobj->svpwm.Ts / sum;
				pobj->svpwm.t1 = k_svpwm * pobj->svpwm.t1;
				pobj->svpwm.t5 = k_svpwm * pobj->svpwm.t5;
			}
			pobj->svpwm.t0 = (pobj->svpwm.Ts - pobj->svpwm.t1 - pobj->svpwm.t5) / 2;
			pobj->svpwm.ta = pobj->svpwm.t5 + pobj->svpwm.t0;
			pobj->svpwm.tb = pobj->svpwm.t0;
			pobj->svpwm.tc = pobj->svpwm.t1 + pobj->svpwm.t5 + pobj->svpwm.t0;
			break;
		case 2:
			pobj->svpwm.t5 = -pobj->svpwm.u1;
			pobj->svpwm.t4 = -pobj->svpwm.u3;
			sum = pobj->svpwm.t4 + pobj->svpwm.t5;
			if (sum > pobj->svpwm.Ts)
			{
				k_svpwm = pobj->svpwm.Ts / sum;
				pobj->svpwm.t4 = k_svpwm * pobj->svpwm.t4;
				pobj->svpwm.t5 = k_svpwm * pobj->svpwm.t5;
			}
			pobj->svpwm.t0 = (pobj->svpwm.Ts - pobj->svpwm.t4 - pobj->svpwm.t5) / 2;
			pobj->svpwm.ta = pobj->svpwm.t4 + pobj->svpwm.t5 + pobj->svpwm.t0;
			pobj->svpwm.tb = pobj->svpwm.t0;
			pobj->svpwm.tc = pobj->svpwm.t5 + pobj->svpwm.t0;
			break;
		default:
			break;
	}
}

/**
 * @brief 初始化 foc_t 结构体
 * @param[in] pobj foc_t 结构体指针
 * @param[in] current_cb 电流回调函数
 * @param[in] ele_radian_cb 电角度回调函数
 * @param[in] ud_cb ud回调函数
 * @param[in] uq_cb uq回调函数
 */
void foc_init(foc_t *pobj, focCurrent_t (*current_cb)(void), float (*ele_radian_cb)(void))
{
	memset(pobj, 0, sizeof(foc_t));

	pobj->current_callback = current_cb;
	pobj->ele_radian_callback = ele_radian_cb;
	pobj->clarke = clarke_transfer;
	pobj->park = park_transfer;
	pobj->inverse_park = inverse_park_transfer;
	pobj->pfsvpwm = foc_svpwm;
	pobj->set_udq = set_udq;
}
