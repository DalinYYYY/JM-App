/**
 * @file    friction_comp.h
 * @brief   摩擦前馈补偿算法（纯函数、无状态、控制环安全）
 * @note    补偿静/动摩擦切换时的速度环 PI 积分拖尾，抑制零速 stick-slip
 *          松弛振荡（现象：空载定位缓慢漂移→突跳→周期性异响）。
 *          模块裁剪：FRICTION_COMP_EN=0 时本头文件将函数替换为恒 0 的
 *          static inline，调用方无需条件编译，编译器常量折叠后零
 *          ROM/RAM/时间开销。
 *          实时性：纯算术、无阻塞、无内存间接访问，可在 ISR/控制环调用。
 */

#ifndef __FRICTION_COMP_H__
#define __FRICTION_COMP_H__

/* 模块裁剪开关：1=启用 0=裁剪（输出恒 0） */
#ifndef FRICTION_COMP_EN
#define FRICTION_COMP_EN 1
#endif

/* smoothsign 线性区宽度(rad/s)：越小越接近硬 sign，零速抖振风险越大；
 * 越大前馈越弱。运行时常量，编译器直接内联。 */
#ifndef FRICTION_COMP_SMOOTH_V0
#define FRICTION_COMP_SMOOTH_V0 0.05f
#endif

#if FRICTION_COMP_EN

/**
 * @brief  计算摩擦前馈电流 iq_ff = smoothsign(vel)·(Tf + B·|vel|)/Kt
 * @param  kt       转矩常数 Kt (Nm/A)，<=0 时返回 0
 * @param  coulomb  库仑摩擦力矩 Tf (Nm)
 * @param  viscous  粘滞摩擦系数 B (Nm/(rad/s))
 * @param  vel      机械角速度 (rad/s)
 * @return 前馈电流 (A)；Tf=B=0 时返回 0
 */
float friction_comp_current(float kt, float coulomb, float viscous, float vel);

#else /* 裁剪：恒 0 内联，零开销 */

static inline float friction_comp_current(float kt, float coulomb, float viscous, float vel)
{
	(void)kt;
	(void)coulomb;
	(void)viscous;
	(void)vel;
	return 0.0f;
}

#endif /* FRICTION_COMP_EN */

#endif /* __FRICTION_COMP_H__ */
