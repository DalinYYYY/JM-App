/**
 * @file        motor_loop.c
 * @brief       电机三环控制集成/编排层实现
 *
 * @author      yangsl
 * @version     1.0
 * @date        2026-06-11
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 */

#include "motor_loop.h"
#include "pid_profile.h"
#include "runtime_param.h"
#include "motion_param.h"
#include "multiturn_counter.h"
#include "dev_power_monitor.h"

#define MOTOR_LOOP_DEG_TO_RAD (0.01745329252f) /* π/180 */

/* 全局电机三环控制上下文 */
motor_loop_t s_motor_loop;

motor_loop_t *motor_loop_get(void)
{
	return &s_motor_loop;
}

/**
 * @brief FOC 三相电流回调：从相电流采样对象读取 ia/ib/ic
 * @note  真实模式来自 ADC 采样；虚拟模式来自 dq 物理模型反算的三相电流。
 */
static focCurrent_t motor_loop_current_cb(void)
{
	dev_phase_current_t *pc = &s_motor_loop.motor.phase_current;
	focCurrent_t c;
	c.ia = pc->current.a;
	c.ib = pc->current.b;
	c.ic = pc->current.c;
	return c;
}

/**
 * @brief FOC 电弧度回调：返回当前电角度弧度
 */
static float motor_loop_ele_radian_cb(void)
{
	return s_motor_loop.motor.motor_param.ele_radian;
}

void motor_loop_init(float current_freq_hz)
{
	motor_loop_t *m = &s_motor_loop;
	motor_param_t *param = &usr.motor_param[M1];

	// 各环控制周期：电流环由中断频率决定，外环按分频系数派生
	float dt_current = 1.0f / current_freq_hz;
	float dt_velocity = dt_current * MOTOR_LOOP_VEL_DIV;
	float dt_position = dt_current * MOTOR_LOOP_POS_DIV;

	m->vel_cnt = 0;
	m->pos_cnt = 0;
	m->sync_pending = false;

	// PID 参数管理器（位置/速度/电流环共用同一套 profile 体系）
	pid_profile_init(param);

	// 底层电机设备（真实硬件 或 虚拟 dq 物理模型，由 select 头决定）
	dev_motor_init(&m->motor, DEV_MOTOR_1, motor_loop_current_cb, motor_loop_ele_radian_cb);

#if !(MOTOR_LOOP_ENABLE_DEV_DRIVER)
	// 虚拟电机：用实际电流环周期设置物理模型积分步长，保证仿真 dt 与控制 dt 一致
	virtual_motor_set_period(&m->motor, dt_current);
#endif

	// 上层状态机（模式管理 + 参考生成）
	system_state_init(&m->sys, param, dt_current);

	// 级联外环（位置/速度）与电流环
	cascade_control_init(&m->cascade, param, dt_position, dt_velocity);
	cur_loop_init(&m->current, &m->motor, dt_current);

	// 电流采样零位校准
	cur_loop_calibrate_offset(&m->current);

	m->motor.phase_current.start(&m->motor.phase_current);
	m->motor.half_bridge.start(&m->motor.half_bridge);

	drv_tim_start_it(m->motor.fsm_tim); // 启动控制定时器
}

/**
 * @brief 解算运动反馈（速度/位置），并同步到状态机与级联反馈
 * @param m 上下文指针
 * @param fb 输出给级联控制的反馈
 * @param update_vel 是否解算速度（速度环节拍）
 * @param update_pos 是否解算位置（位置环节拍）
 * @note 编码器机械角度与电弧度、三相电流由 cur_loop_run 每拍刷新，
 *       本函数复用其结果，仅追加速度/位置等外环所需运动量解算，避免重复采样。
 */
static void motor_loop_update_feedback(motor_loop_t *m, cascade_fb_t *fb, bool update_vel, bool update_pos)
{
	motion_param_t *mp = &m->motor.motor_param;
	multiturn_t *mt = &m->motor.multiturn;

	// 基于已刷新的机械角度解算速度/位置（不重复触发编码器采样）
	// 虚拟模式下 update 指向物理模型实现，直接给出运动量
	if (update_pos)
	{
		mp->update(mp, MOTION_TYPE_ALL, mp->mechanical_angle);
		mt->update_single(mt, mp->mechanical_angle); // 多圈位置解算（单编码器/软件累圈）
	}
	else if (update_vel)
		mp->update(mp, MOTION_TYPE_ELE_VEL_RADIAN, mp->mechanical_angle);

	// 组织级联反馈：位置来自多圈解算，速度来自运动解算，电流来自 FOC park 结果
	fb->pos = mt->get_position(mt);
	fb->vel = mp->slide_rad_s;
	fb->id = m->motor.foc.i_dq.d;
	fb->iq = m->motor.foc.i_dq.q;

	// 同步到状态机反馈（供 IDLE 保持位置、MIT 等 handler 使用）
	m->sys.motor.fb.pos = fb->pos;
	m->sys.motor.fb.vel = fb->vel;
	m->sys.motor.fb.id = fb->id;
	m->sys.motor.fb.iq = fb->iq;
}

/**
 * @brief 发布电气测量量 0xC2/0xC3 (FOC 工作集 → electrical 子块)
 */
static void publish_electrical(motor_electrical_t *e, const foc_t *foc)
{
	e->ia = foc->current.ia;
	e->ib = foc->current.ib;
	e->ic = foc->current.ic;
	e->i_alpha = foc->i_alphaBeta.alpha;
	e->i_beta = foc->i_alphaBeta.beta;
	e->id_meas = foc->i_dq.d;
	e->iq_meas = foc->i_dq.q;
	e->ud = foc->u_dq.d;
	e->uq = foc->u_dq.q;
	e->u_alpha = foc->u_alphaBeta.alpha;
	e->u_beta = foc->u_alphaBeta.beta;
	e->duty_a = foc->svpwm.ta;
	e->duty_b = foc->svpwm.tb;
	e->duty_c = foc->svpwm.tc;
}

/**
 * @brief 发布运动反馈量 0xC6/0xC7 (motion/multiturn → motion 子块)
 * @note  motion_param 角度单位为 deg, runtime 统一用 rad。
 */
static void publish_motion(motor_motion_t *mo, const motion_param_t *mp, multiturn_t *mt)
{
	mo->mech_angle_rad = mp->mechanical_angle * MOTOR_LOOP_DEG_TO_RAD;
	mo->elec_angle_rad = mp->ele_radian;
	mo->single_turn_rad = mt->last_single_rad;
	mo->multiturn = mt->get_turns(mt);
	mo->position_rad = mt->get_position(mt);
	mo->velocity_rad_s = mp->slide_rad_s;
	mo->velocity_filt = mp->slide_rad_s;
	mo->accel_rad_s2 = mp->acceleration;
}

/**
 * @brief 发布母线/功率/力矩 0xC4 与温度 0xC5
 * @note  母线/温度来自全局 power_monitor; 力矩估算 = iq*kt*gear。
 */
static void publish_power_thermal(motor_state_t *st, const foc_t *foc,
								  const motion_param_t *mp, const motor_param_t *param)
{
	st->power.v_bus = dev_power_monitor.vbus;
	st->power.i_bus = dev_power_monitor.ibus;
	st->power.power_elec_w = dev_power_monitor.vbus * dev_power_monitor.ibus;
	st->power.torque_est = foc->i_dq.q * param->motor_base.kt * param->gearbox_param.gear_ratio;
	st->power.power_mech_w = st->power.torque_est * mp->slide_rad_s;

	st->thermal.temp_fet = dev_power_monitor.temp_driver;
	st->thermal.temp_motor = dev_power_monitor.temp_motor;
}

/**
 * @brief 把控制上下文(s_motor_loop)的运行量单向同步到全局数据视图 usr
 * @note  数据流向: 控制层(私有工作集) → runtime_param(对外遥测快照)。
 *        供通信(jm_proto)/显示/日志统一读 usr, 不直接耦合控制层内部结构。
 *        在位置环节拍调用即可(频率足够仪表盘与通信), 不必每个电流环基频都同步。
 *        按子块拆分: 新增遥测字段时只改对应 publish_*(), 不必动本编排函数。
 */
static void motor_loop_sync_runtime(motor_loop_t *m)
{
	motor_state_t *st = &usr.motor_state[M1];
	const motor_param_t *param = &usr.motor_param[M1];

	publish_electrical(&st->electrical, &m->motor.foc);
	publish_motion(&st->motion, &m->motor.motor_param, &m->motor.multiturn);
	publish_power_thermal(st, &m->motor.foc, &m->motor.motor_param, param);

	st->run_mode = (run_state_e)m->sys.top_state;
}

void motor_loop_isr(void)
{
	motor_loop_t *m = &s_motor_loop;
	cascade_fb_t fb;
	// 分频判断：速度环/位置环各用独立计数器，各自到阈值清零，互不干扰
	// 仅用自增与比较，避免中断内取模/除法
	bool vel_tick = (++m->vel_cnt >= MOTOR_LOOP_VEL_DIV);
	bool pos_tick = (++m->pos_cnt >= MOTOR_LOOP_POS_DIV);
	if (vel_tick)
		m->vel_cnt = 0;
	if (pos_tick)
		m->pos_cnt = 0;

	// step1: 解算运动反馈（复用上一拍电流环刷新的角度/电流）
	motor_loop_update_feedback(m, &fb, vel_tick, pos_tick);

	// 遥测同步错开位置环：上一拍位置拍挂起的同步在本拍执行，避开位置环重负载拍
	if (m->sync_pending)
	{
		motor_loop_sync_runtime(m);
		m->sync_pending = false;
	}
	// 本拍命中位置拍：挂起同步，留到下一拍执行
	if (pos_tick)
		m->sync_pending = true;

	// step2: 状态机生成参考输出 motor.ref（含模式管理与平滑过渡）
	motor_control_loop(&m->sys);

	// 非运行态：电流环输出零电流，外环复位
	if (m->sys.top_state != TOP_FSM_RUN)
	{
		cascade_control_reset(&m->cascade);
		cur_loop_reset(&m->current);
		m->out.id_ref = 0.0f;
		m->out.iq_ref = 0.0f;
		cur_loop_run(&m->current, 0.0f, 0.0f);
		return;
	}

	// step3: 位置环（分频）——输出速度设定
	if (pos_tick)
		cascade_control_run_position(&m->cascade, &m->sys.motor.ref, &fb);

	// step4: 速度环 + 入环分发（分频）——输出 dq 电流参考
	if (vel_tick)
		cascade_control_run(&m->cascade, &m->sys.motor.ref, &fb, &m->out);

	// step5: 电流环（基频）——FOC + PI + SVPWM + PWM 输出
	cur_loop_run(&m->current, m->out.id_ref, m->out.iq_ref);
}

void motor_loop_set_cmd(ctrl_mode_e cmd)
{
	process_ctrl_cmd(&s_motor_loop.sys, cmd);
}
