/**
 * @file        foc_core.c
 * @brief       FOC 算法实现（Clarke / Park / 逆 Park / SVPWM）
 *
 * @details     全部函数运行在电流环 ISR 内，按执行时间优先实现：
 *              - sin/cos 按电角度缓存，同拍 park 与 inverse_park 只算一次
 *                （见 foc_update_sincos）
 *              - SVPWM 中间量只用局部变量，仅回写外部消费的 ta/tb/tc
 *              - 三角系数编译期折叠，除法仅在过调制时执行
 *              等幅值 (2/3) Clarke 约定，前向 SVPWM 侧以 k_amp=1.5 补偿，
 *              保证 u_dq 命令电压与实际相电压 1:1（标定正确性依赖此增益）。
 *
 * @author      yangsl (yangsl@robot.com)
 * @version     1.1
 * @date        2026-08-18
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @par 修改日志:
 * | 日期       | 版本 | 作者   | 修改内容                                   |
 * |------------|------|--------|--------------------------------------------|
 * | 2026-06-15 | 1.0  | yangsl | 初始创建                                   |
 * | 2026-08-18 | 1.1  | yangsl | 热路径优化：sin/cos 缓存、SVPWM 去中间量落地 |
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 */
#include "foc_core.h"
#include <string.h>
#include "utils.h"

/* 置 1 用 CMSIS-DSP 查表插值 arm_sin_f32/arm_cos_f32（M4F 上远快于 libm），置 0 回退 libm */
#define DSP_MATH_ENABLE 1

#if DSP_MATH_ENABLE
#include "arm_math.h"
#else
#include <math.h>
#endif

/*
 * @brief 等幅值(2/3) Clarke 变换：Ia/Ib/Ic → Iα/Iβ
 * @param pobj foc结构体指针对象
 * @note  Iα = (2/3)·(Ia - (Ib+Ic)/2)，Iβ = (Ib-Ic)/√3
 *        不假设三相平衡，直接用三相值计算（平衡时 Iα 自然退化为 Ia）。
 */
static void clarke_transfer(struct foc *pobj)
{
	pobj->current = pobj->current_callback();

	pobj->i_alphaBeta.alpha = (2.0f / 3.0f) * (pobj->current.ia - 0.5f * (pobj->current.ib + pobj->current.ic));
	pobj->i_alphaBeta.beta = ONE_BY_SQRT3 * (pobj->current.ib - pobj->current.ic);
}

/*
 * @brief 按电角度刷新 sin/cos 缓存（同一角度则复用，省下一对三角函数）
 * @param pobj foc结构体指针对象
 * @param theta 当前电角度（弧度）
 * @note  电流环内 park 与 inverse_park 同拍同角度，后者命中缓存；
 *        标定路径（calib_hw_apply_voltage）不调 park 而强制改角度后单独调
 *        inverse_park，此时比较失配会正常重算，不会用到过期 sin/cos。
 *        theta 为 NaN 时比较恒假，走重算分支。
 * @warning 缓存不变式：foc_sin/foc_cos 必须始终对应 Theta。
 *          新增写 Theta 的代码须同步刷新这两个值（初值见 foc_init）。
 */
static inline void foc_update_sincos(struct foc *pobj, float theta)
{
	if (theta == pobj->Theta)
	{
		return;
	}

#if DSP_MATH_ENABLE
	pobj->foc_sin = arm_sin_f32(theta);
	pobj->foc_cos = arm_cos_f32(theta);
#else
	pobj->foc_sin = sinf(theta);
	pobj->foc_cos = cosf(theta);
#endif
	pobj->Theta = theta;
}

/*
 * @brief Park 变换：Iα/Iβ → Id/Iq
 * @param pobj foc结构体指针对象
 * @note  Id = Iα·cosθ + Iβ·sinθ，Iq = -Iα·sinθ + Iβ·cosθ
 */
static void park_transfer(struct foc *pobj)
{
	float theta = pobj->ele_radian_callback();
	float s, c;

	foc_update_sincos(pobj, theta);
	s = pobj->foc_sin;
	c = pobj->foc_cos;

	float ia = pobj->i_alphaBeta.alpha;
	float ib = pobj->i_alphaBeta.beta;
	float id = ia * c + ib * s;
	float iq = ib * c - ia * s;

	/* calib_raw_mode：标定期间旁路 LPF，消除滤波对阶跃响应的延迟污染。
	 * 两种模式的滤波状态更新一致，故合并写回。*/
	if (!pobj->calib_raw_mode)
	{
		id = _lpfilter(FOC_DQ_LPF_ALPHA, id, pobj->calib_prev_id);
		iq = _lpfilter(FOC_DQ_LPF_ALPHA, iq, pobj->calib_prev_iq);
	}

	pobj->i_dq.d = id;
	pobj->i_dq.q = iq;
	pobj->calib_prev_id = id;
	pobj->calib_prev_iq = iq;
}

/*
 * @brief Park 逆变换：Ud/Uq → Uα/Uβ
 * @param pobj foc结构体指针对象
 * @note  Uα = Ud·cosθ - Uq·sinθ，Uβ = Ud·sinθ + Uq·cosθ
 */
static void inverse_park_transfer(struct foc *pobj)
{
	float theta = pobj->ele_radian_callback();
	float s, c, ud, uq;

	foc_update_sincos(pobj, theta);
	s = pobj->foc_sin;
	c = pobj->foc_cos;
	ud = pobj->u_dq.d;
	uq = pobj->u_dq.q;

	pobj->u_alphaBeta.alpha = ud * c - uq * s;
	pobj->u_alphaBeta.beta = ud * s + uq * c;
}

static void set_udq(struct foc *pobj, float ud, float uq)
{
	pobj->u_dq.d = ud;
	pobj->u_dq.q = uq;
}

/* 采样周期归一化为 1：ta/tb/tc 即占空比，乘 PWM 周期值得到 CCR */
#define SVPWM_TS 1.0f

/*
 * @brief 过调制钳位：两邻矢量作用时间之和超出采样周期时等比缩放
 * @param t_a [in,out] 第一个矢量作用时间
 * @param t_b [in,out] 第二个矢量作用时间
 * @note  线性区内不含除法，仅过调制时执行一次浮点除法
 */
static inline void clamp_vector_time(float *t_a, float *t_b)
{
	float sum = *t_a + *t_b;
	if (sum > SVPWM_TS)
	{
		float k = SVPWM_TS / sum;
		*t_a *= k;
		*t_b *= k;
	}
}

/*
 * @brief SVPWM 算法：Uα/Uβ → 三相占空比 ta/tb/tc
 * @param pobj foc结构体指针对象
 * @note  中间量（t0~t6 / 扇区判断量 / 扇区号）全工程仅本文件使用，
 *        故只用局部变量、不回写结构体，避免无用 RAM 访问；
 *        外部只消费 ta/tb/tc（current_loop / motor_observer / calib_hw / motor_loop）。
 */
static void foc_svpwm(struct foc *pobj)
{
	float u_alpha = pobj->u_alphaBeta.alpha;
	float u_beta = pobj->u_alphaBeta.beta;
	float ta, tb, tc;

	/* 零电压指令：直接输出零矢量，跳过扇区判断 */
	if (u_alpha == 0.0f && u_beta == 0.0f)
	{
		ta = tb = tc = 0.5f * SVPWM_TS;
	}
	else
	{
		/* 幅值补偿 k_amp = 1.5：等幅值(2/3)Clarke 约定下，单一满占空比有效矢量
		 * 仅产生 (2/3)·Vbus 的相-中性点电压，故轴向作用时间须为 1.5·Uα，才能使
		 * 实际相电压 Va = 命令电压 ud。缺此因子时 Va 仅为 (2/3)·ud，开环标定
		 * R = ud/id 会偏大 3/2 倍。补偿后前向增益=1，与反馈侧等幅值 Clarke 一致。
		 * 线性区上限 |Uα|max=1/√3 时轴向作用时间=0.866<1，过调制钳位仍正常触发。
		 * 系数为编译期常量，ub_2sqrt3 复用 ub_sqrt3 相加，共 3 次乘法。*/
		float ua = 1.5f * u_alpha;
		float ub_sqrt3 = (1.5f * ONE_BY_SQRT3) * u_beta; /* 1.5·Uβ/√3 */
		float ub_2sqrt3 = ub_sqrt3 + ub_sqrt3;
		float t_x, t_y; /* 当前扇区两邻矢量作用时间 */
		float t0;

		/* 扇区判断量（仅取符号）：u1 = Uβ, u2 = (√3/2)Uα - Uβ/2, u3 = -(√3/2)Uα - Uβ/2 */
		float half_u_beta = 0.5f * u_beta;
		float sqrt3_2_ua = SQRT3_BY_2 * u_alpha;
		float u2 = sqrt3_2_ua - half_u_beta;
		float u3 = -sqrt3_2_ua - half_u_beta;

		/* 扇区号 N = 4·C + 2·B + A */
		int sector = ((u_beta > 0.0f) ? 1 : 0) | ((u2 > 0.0f) ? 2 : 0) | ((u3 > 0.0f) ? 4 : 0);

		/* 各扇区仅两邻矢量的取值与 ta/tb/tc 分配顺序不同，t0 合成方式一致 */
		switch (sector)
		{
			case 3: /* Sector I: V4(100) + V6(110) */
				t_x = ua - ub_sqrt3;
				t_y = ub_2sqrt3;
				clamp_vector_time(&t_x, &t_y);
				t0 = (SVPWM_TS - t_x - t_y) * 0.5f;
				ta = t_x + t_y + t0;
				tb = t_y + t0;
				tc = t0;
				break;

			case 1: /* Sector II: V6(110) + V2(010) */
				t_x = ua + ub_sqrt3;  /* t6 */
				t_y = ub_sqrt3 - ua;  /* t2 */
				clamp_vector_time(&t_x, &t_y);
				t0 = (SVPWM_TS - t_x - t_y) * 0.5f;
				ta = t_x + t0;
				tb = t_x + t_y + t0;
				tc = t0;
				break;

			case 5: /* Sector III: V2(010) + V3(011) */
				t_x = ub_2sqrt3;       /* t2 */
				t_y = -ua - ub_sqrt3;  /* t3 */
				clamp_vector_time(&t_x, &t_y);
				t0 = (SVPWM_TS - t_x - t_y) * 0.5f;
				ta = t0;
				tb = t_x + t_y + t0;
				tc = t_y + t0;
				break;

			case 4: /* Sector IV: V3(011) + V1(001) */
				t_x = ub_sqrt3 - ua;  /* t3 */
				t_y = -ub_2sqrt3;     /* t1 */
				clamp_vector_time(&t_x, &t_y);
				t0 = (SVPWM_TS - t_x - t_y) * 0.5f;
				ta = t0;
				tb = t_x + t0;
				tc = t_x + t_y + t0;
				break;

			case 6: /* Sector V: V1(001) + V5(101) */
				t_x = -ua - ub_sqrt3; /* t1 */
				t_y = ua - ub_sqrt3;  /* t5 */
				clamp_vector_time(&t_x, &t_y);
				t0 = (SVPWM_TS - t_x - t_y) * 0.5f;
				ta = t_y + t0;
				tb = t0;
				tc = t_x + t_y + t0;
				break;

			case 2: /* Sector VI: V5(101) + V4(100) */
				t_x = ua + ub_sqrt3; /* t4 */
				t_y = -ub_2sqrt3;    /* t5 */
				clamp_vector_time(&t_x, &t_y);
				t0 = (SVPWM_TS - t_x - t_y) * 0.5f;
				ta = t_x + t_y + t0;
				tb = t0;
				tc = t_y + t0;
				break;

			default:
				/* sector=0 仅在 Uα/Uβ 为 NaN 等异常输入时出现（零电压已提前处理）：
				 * 必须输出零矢量，否则沿用上一周期占空比会失控 */
				ta = tb = tc = 0.5f * SVPWM_TS;
				break;
		}
	}

	pobj->svpwm.ta = ta;
	pobj->svpwm.tb = tb;
	pobj->svpwm.tc = tc;
}

/**
 * @brief 初始化 foc_t 结构体
 * @param[in] pobj foc_t 结构体指针
 * @param[in] current_cb 电流回调函数（返回三相电流）
 * @param[in] ele_radian_cb 电角度回调函数（返回电弧度）
 */
void foc_init(foc_t *pobj, focCurrent_t (*current_cb)(void), float (*ele_radian_cb)(void))
{
	memset(pobj, 0, sizeof(foc_t));

	/* 建立 sin/cos 缓存不变式：memset 后 Theta=0，故须置 cos(0)=1。
	 * 否则首次电角度恰为 0 时会命中缓存并误用 cos=0，首拍 Park/逆 Park 输出全零。*/
	pobj->Theta = 0.0f;
	pobj->foc_sin = 0.0f;
	pobj->foc_cos = 1.0f;

	pobj->current_callback = current_cb;
	pobj->ele_radian_callback = ele_radian_cb;
	pobj->clarke = clarke_transfer;
	pobj->park = park_transfer;
	pobj->inverse_park = inverse_park_transfer;
	pobj->pfsvpwm = foc_svpwm;
	pobj->set_udq = set_udq;
}
