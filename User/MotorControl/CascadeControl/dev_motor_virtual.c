/**
 * @file        dev_motor_virtual.c
 * @brief       虚拟电机设备实现（在环 dq 物理模型，复用 foc.c 的 FOC 算法）
 *
 * @details     伪装 dev_motor_t，使 FOC 三环无需修改即可闭环仿真运行。
 *              FOC 数学（Clarke/Park/反Park/SVPWM）直接复用真实 foc.c（经 foc_init
 *              装配函数指针），保证虚拟电机与真实电机跑同一套 FOC 算法——后期优化
 *              foc.c 时两者自动同步，无需维护两份。虚拟电机只负责物理建模：接收
 *              FOC 算出的 dq 电压，积分出电流/角度/速度，反算三相电流回喂采样。
 *
 *              控制时序与一拍延迟：
 *                cur_loop_run 每拍顺序为
 *                  encoder.update → phase_current.update → clarke → park
 *                  → PI(算 ud/uq) → set_udq → inverse_park → svpwm → set_3pwm
 *                本拍 ud/uq 直到 set_3pwm 才齐全，故在 set_3pwm 末尾用本拍 u_dq
 *                推进物理模型一步，更新 id/iq/theta；下一拍 encoder/phase_current
 *                输出新状态。即“本拍电压 → 下拍反馈”，符合真实数字控制的一拍延迟。
 *
 * @author      yangsl
 * @version     1.0
 * @date        2026-06-12
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @note        本文件始终在 Keil 编译列表中，但整个实现体由
 *              MOTOR_LOOP_ENABLE_DEV_DRIVER 控制：宏=1（真实驱动）时编译为空，
 *              与真实 dev_motor.c 零符号冲突。复用 foc.c 需保证 foc.c 也在编译列表。
 */

#include "motor_loop_config.h"

#if !(MOTOR_LOOP_ENABLE_DEV_DRIVER)

#include "dev_motor_virtual.h"
#include "foc_core.h"
#include "motor_param.h"
#include "runtime_param.h"
#include <string.h>
#include <math.h>

#ifndef M_2PI
#define M_2PI (6.283185307179586f)
#endif

/* 物理参数下限保护，避免未初始化(全0)导致除零/发散 */
#define VIRT_MIN_L       1.0e-5f /* 电感下限 10µH */
#define VIRT_MIN_INERTIA 1.0e-7f /* 惯量下限 */
#define VIRT_MIN_FLUX    1.0e-4f /* 磁链下限 */

/* 单个 dev_motor 虚拟对象的自引用指针：供无参 FOC 回调访问模型。
 * 工程仅 DEV_MOTOR_1 一路电机，单实例足够。 */
static dev_motor_t *s_virtual_self = NULL;
static float s_virtual_mt_offset = 0.0f; /* 虚拟多圈零点偏移(rad) */

/* 反 Clarke 用等幅值系数：由 dq 电流生成模型三相电流（供真实 foc.c 的 clarke 复用） */
#define VIRT_SQRT3_BY_2 0.8660254037844386f

/*============================================================================
 * dq 物理模型推进
 *==========================================================================*/

static float virt_wrap_2pi(float a)
{
	while (a >= M_2PI)
		a -= M_2PI;
	while (a < 0.0f)
		a += M_2PI;
	return a;
}

/**
 * @brief 电机模型实现（电压进、角度出）
 * @param m 电机对象
 * @param ud d 轴电压
 * @param uq q 轴电压
 */
static void virt_model_step(dev_motor_t *m, float ud, float uq)
{
	virtual_motor_model_t *p = &m->model;
	uint8_t n = (p->substeps > 0u) ? p->substeps : 1u;
	float h = p->dt / (float)n;

	for (uint8_t i = 0; i < n; ++i)
	{
		float we = (float)p->poles * p->omega; /* 电角速度 */

		/* 电气：前向欧拉积分 dq 电流 */
		float did = (ud - p->Rs * p->id + we * p->Lq * p->iq) / p->Ld;
		float diq = (uq - p->Rs * p->iq - we * p->Ld * p->id - we * p->flux) / p->Lq;
		p->id += did * h;
		p->iq += diq * h;

		/* 电磁转矩（含磁阻转矩项） */
		p->Te = 1.5f * (float)p->poles * (p->flux * p->iq + (p->Ld - p->Lq) * p->id * p->iq);

		/* 摩擦：粘滞 + 库仑 */
		float tfric = p->fric_visc * p->omega;
		if (p->omega > 1.0e-6f)
			tfric += p->fric_coul;
		else if (p->omega < -1.0e-6f)
			tfric -= p->fric_coul;

		/* 机械：积分角速度与角度 */
		float domega = (p->Te - p->Tload - tfric) / p->inertia;
		p->omega += domega * h;
		p->theta_m += p->omega * h;
	}

	/* 机械角连续累计后映射电角度 [0,2π) */
	p->theta_e = virt_wrap_2pi((float)p->poles * p->theta_m);
}

/*============================================================================
 * 子设备回调实现
 *==========================================================================*/

/** @brief 编码器更新（抽象接口）：输出模型机械角度(deg)，[0,360) */
static void virt_encoder_update(struct dev_encoder *enc)
{
	dev_motor_t *m = s_virtual_self;
	if (m == NULL)
		return;
	float deg = m->model.theta_m * (180.0f / 3.14159265358979f);
	/* 折算到 [0,360) */
	deg = fmodf(deg, 360.0f);
	if (deg < 0.0f)
		deg += 360.0f;
	enc->mechanical_angle = deg;
}

/** @brief 编码器取角（抽象接口）：返回最新机械角度(deg) */
static float virt_encoder_get_mechanical_angle(struct dev_encoder *enc)
{
	return enc->mechanical_angle;
}

/**
 * @brief 三相电流采样：由模型 id/iq + 当前电角度 反Park→反Clarke 生成 ia/ib/ic
 * @note  使 Clarke/Park 链路真实参与，三环看到的是经过坐标变换的真实电流。
 */
static void virt_phase_current_update(struct dev_adc_injected *pc)
{
	dev_motor_t *m = s_virtual_self;
	if (m == NULL)
		return;

	float th = m->model.theta_e;
	float s = sinf(th), c = cosf(th);

	/* 反 Park：dq → αβ（电流） */
	float ialpha = m->model.id * c - m->model.iq * s;
	float ibeta = m->model.id * s + m->model.iq * c;

	/* 反 Clarke（等幅值）：αβ → 三相 */
	pc->current.a = ialpha;
	pc->current.b = -0.5f * ialpha + VIRT_SQRT3_BY_2 * ibeta;
	pc->current.c = -0.5f * ialpha - VIRT_SQRT3_BY_2 * ibeta;
}

/**
 * @brief 半桥 PWM：占空比已由 SVPWM 给出；在此（每拍末尾）推进物理模型一步。
 * @note  用本拍 foc.u_dq 作为施加到电机的 dq 电压，体现一拍延迟。
 */
static int virt_half_bridge_set_3pwm(struct dev_half_bridge *hb, uint32_t ccr1, uint32_t ccr2, uint32_t ccr3)
{
	dev_motor_t *m = s_virtual_self;
	(void)hb;
	(void)ccr1;
	(void)ccr2;
	(void)ccr3; /* 虚拟模式不驱动真实定时器 */
	if (m == NULL)
		return 0;
	virt_model_step(m, m->foc.u_dq.d, m->foc.u_dq.q);
	return 0;
}

/*============================================================================
 * FOC 三相电流回调 / 电弧度回调由 motor_loop.c 提供（读 phase_current/motor_param）
 * 这里通过 motion_param 的 ele_radian 字段把电角度交给 FOC park。
 *==========================================================================*/

/*============================================================================
 * motion_param 虚拟实现：直接由物理模型给出运动量，不做编码器解算
 *==========================================================================*/

static void virt_motion_update(struct motion_param *mp, motion_type_e type, float mechanical_angle)
{
	dev_motor_t *m = s_virtual_self;
	(void)type;
	(void)mechanical_angle;
	if (m == NULL)
		return;
	mp->mechanical_angle = m->model.theta_m * (180.0f / 3.14159265358979f);
	mp->ele_radian = m->model.theta_e;
	mp->rad_s = m->model.omega;
	mp->slide_rad_s = m->model.omega;
}

static float virt_motion_get_ele_radian(struct motion_param *mp)
{
	return mp->ele_radian;
}

/*============================================================================
 * multiturn 虚拟实现：连续累计机械角(rad)即绝对多圈位置，不做齿轮解算
 *==========================================================================*/

static float virt_multiturn_update(struct multiturn *mt, const float *gear_angles, uint8_t count)
{
	dev_motor_t *m = s_virtual_self;
	(void)gear_angles;
	(void)count;
	if (m == NULL)
		return 0.0f;
	/* 连续累计机械角(rad) 减去零点偏移即多圈位置（支持运行时零点复位） */
	mt->position = m->model.theta_m - s_virtual_mt_offset;
	mt->multiturn_position = mt->position;
	mt->turns = (int32_t)(mt->position / (2.0f * 3.14159265358979f));
	return mt->position;
}

static float virt_multiturn_update_single(struct multiturn *mt, float mechanical_angle)
{
	(void)mechanical_angle;
	return virt_multiturn_update(mt, NULL, 0u);
}

static float virt_multiturn_get_position(struct multiturn *mt)
{
	return mt->position;
}

static void virt_multiturn_reset_position(struct multiturn *mt)
{
	dev_motor_t *m = s_virtual_self;
	(void)mt;
	if (m == NULL)
		return;
	/* 虚拟多圈位置每拍由物理模型 theta_m 覆盖, 置零必须记偏移量 */
	s_virtual_mt_offset = m->model.theta_m;
}

/*============================================================================
 * 初始化与公开接口
 *==========================================================================*/

void virtual_motor_set_period(dev_motor_t *pobj, float dt)
{
	if (pobj == NULL || dt <= 0.0f)
		return;
	pobj->model.dt = dt;
	/* 子步：保证子步长不超过电气时间常数的 ~1/10，提升数值稳定 */
	float tau_e = pobj->model.Ld / pobj->model.Rs; /* 电气时间常数 */
	uint8_t n = 1u;
	if (tau_e > 0.0f)
	{
		float ratio = dt / (0.1f * tau_e);
		while ((float)n < ratio && n < 16u)
			n <<= 1;
	}
	pobj->model.substeps = n;
}

void virtual_motor_set_load(dev_motor_t *pobj, float load_nm)
{
	if (pobj == NULL)
		return;
	pobj->model.Tload = load_nm;
}

void dev_motor_init(dev_motor_t *pobj, motor_id_e id,
                    focCurrent_t (*current_callback)(void),
                    float (*ele_radian_callback)(void))
{
	if (pobj == NULL)
		return;
	memset(pobj, 0, sizeof(dev_motor_t));

	pobj->id = id;
	pobj->current_callback = current_callback;
	pobj->ele_radian_callback = ele_radian_callback;

	/* 从全局电机参数读取物理量，缺省/非法值用下限保护 */
	motor_param_t *mp = &usr.motor_param[M1];
	virtual_motor_model_t *vm = &pobj->model;

	vm->Ld = (mp)->motor_base.ld;
	vm->Lq = (mp)->motor_base.lq;
	vm->flux = (mp)->motor_base.flux;
	vm->inertia = (mp)->motor_base.inertia;
	vm->fric_visc = (mp)->position_loop.friction_viscous;
	vm->fric_coul = (mp)->position_loop.friction_coulomb;
	vm->poles = (mp)->motor_base.pole_pairs;
	vm->Rs = (mp)->motor_base.r;

	if (vm->Ld < VIRT_MIN_L)
		vm->Ld = VIRT_MIN_L;
	if (vm->Lq < VIRT_MIN_L)
		vm->Lq = VIRT_MIN_L;
	if (vm->flux < VIRT_MIN_FLUX)
		vm->flux = VIRT_MIN_FLUX;
	if (vm->inertia < VIRT_MIN_INERTIA)
		vm->inertia = VIRT_MIN_INERTIA;
	if (vm->poles == 0u)
		vm->poles = 7u;

	vm->id = 0.0f;
	vm->iq = 0.0f;
	vm->omega = 0.0f;
	vm->theta_m = 0.0f;
	vm->theta_e = 0.0f;
	vm->Tload = 0.0f;
	vm->dt = 1.0e-4f; /* 占位，正式值由 virtual_motor_set_period 设置 */
	vm->substeps = 1u;

	pobj->poles = vm->poles;

	/* 装配 FOC：直接复用真实 foc.c 的算法（clarke/park/inverse_park/svpwm/set_udq）。
	 * 虚拟与真实电机由此共享同一套 FOC，后期优化 foc.c 两者自动同步。 */
	foc_init(&pobj->foc, current_callback, ele_radian_callback);

	/* 装配子设备函数指针 */
	pobj->encoder.ctx = NULL; // 虚拟模式无具体芯片，角度由物理模型给出
	pobj->encoder.update = virt_encoder_update;
	pobj->encoder.get_mechanical_angle = virt_encoder_get_mechanical_angle;
	pobj->phase_current.update = virt_phase_current_update;
	pobj->half_bridge.set_3pwm = virt_half_bridge_set_3pwm;

	/* 装配 motion_param 函数指针（虚拟运动量） */
	pobj->motor_param.poles = vm->poles;
	pobj->motor_param.update = virt_motion_update;
	pobj->motor_param.get_ele_radian = virt_motion_get_ele_radian;

	/* 装配 multiturn 函数指针（虚拟绝对多圈位置） */
	pobj->multiturn.update = virt_multiturn_update;
	pobj->multiturn.update_single = virt_multiturn_update_single;
	pobj->multiturn.get_position = virt_multiturn_get_position;
	pobj->multiturn.reset_position = virt_multiturn_reset_position;

	s_virtual_self = pobj;
}

#endif /* !MOTOR_LOOP_ENABLE_DEV_DRIVER */
