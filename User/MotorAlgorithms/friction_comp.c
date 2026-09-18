/**
 * @file    friction_comp.c
 * @brief   摩擦前馈补偿算法实现
 * @note    参数来源 motor_param position_loop.friction_coulomb/friction_viscous
 *          （motor_info PID 26/27），由调用方（级联控制层）直传，
 *          本模块不依赖 DataHub/参数结构，保持算法层零上层依赖。
 */

#include "friction_comp.h"

#if FRICTION_COMP_EN

float friction_comp_current(float kt, float coulomb, float viscous, float vel)
{
	float abs_vel;
	float tau;

	/* 快速路径：未配置摩擦参数时 3 条指令返回，控制环零负担 */
	if (kt <= 0.0f || (coulomb <= 0.0f && viscous <= 0.0f))
		return 0.0f;

	abs_vel = (vel >= 0.0f) ? vel : -vel;
	tau = coulomb + viscous * abs_vel;

	return vel / (abs_vel + FRICTION_COMP_SMOOTH_V0) * tau / kt;
}

#endif /* FRICTION_COMP_EN */
