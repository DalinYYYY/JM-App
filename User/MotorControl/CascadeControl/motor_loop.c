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
#include "motor_param.h" /* motor_param_init 加载默认电机参数 */

#define MOTOR_LOOP_DEG_TO_RAD (0.01745329252f) /* π/180 */

/* 全局电机三环控制上下文 */
motor_loop_t s_motor_loop;

static void motor_loop_sync_state(motor_loop_t *m);

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

	/* 加载默认电机参数（R/L/kt/pole_pairs/PID/限幅等）
	 * usr.motor_param[M1] 为 BSS 段全局变量，启动时全零，
	 * 若不加载默认值会导致除零、控制环无响应等问题。
	 * TODO: Flash 参数加载实现后，改为先尝试 Flash 加载，失败再 fallback 到默认。*/
	motor_param_init(param);

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

	// 速度解算频率必须等于速度环真实节拍(电流环/VEL_DIV)，否则 d(angle)/dt 标定错比例；
	// dev_motor_init 内按默认 1kHz 初始化，这里用实际派生频率覆盖，并选用 PLL 观测器
	// (相比后向差分对编码器量化噪声更平滑、低滞后)。
	{
		motion_param_t *mp = &m->motor.motor_param;
		uint32_t vel_freq_hz = (uint32_t)(1.0f / dt_velocity + 0.5f);
		mp->set_update_freq(mp, vel_freq_hz);
		mp->set_vel_method(mp, VEL_METHOD_PLL);
	}

#if !(MOTOR_LOOP_ENABLE_DEV_DRIVER)
	// 虚拟电机：用实际电流环周期设置物理模型积分步长，保证仿真 dt 与控制 dt 一致
	virtual_motor_set_period(&m->motor, dt_current);
#endif

	// 上层状态机（模式管理 + 参考生成）
	system_state_init(&m->sys, param, dt_current);
	motor_loop_sync_state(m);

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

static void motor_loop_sync_state(motor_loop_t *m)
{
	motor_state_t *st = &usr.motor_state[M1];
	const system_state_t *sys = &m->sys;
	bool enabled = (sys->top_state == TOP_FSM_READY) || (sys->top_state == TOP_FSM_RUN);

	st->top_state = sys->top_state;
	st->run_state = sys->motor.run_state;
	st->ctrl_mode = sys->ctrl_mode;
	st->enable_motor = enabled;
	st->enable_pwm = (sys->top_state == TOP_FSM_RUN);
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

	motor_loop_sync_state(m);
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

	// step0: 刷新编码器与电角度（所有模式统一执行，确保上位机随时可读角度）
	m->motor.encoder.update(&m->motor.encoder);
	m->motor.motor_param.update(&m->motor.motor_param,
								MOTION_TYPE_ELE_RADIAN,
								m->motor.encoder.mechanical_angle);

	// step1: 解算运动反馈（使用本拍刷新的角度）
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
	motor_loop_sync_state(m);

	// 非运行态：外环复位，电流环以 IDLE 直通模式输出零 PWM
	if (m->sys.top_state != TOP_FSM_RUN)
	{
		cascade_control_reset(&m->cascade);
		cur_loop_reset(&m->current);
		/* 清零 out，避免进入 RUN 态第一拍 vel_tick=false 时电流环用旧值 */
		m->out.id_ref = 0.0f;
		m->out.iq_ref = 0.0f;
		/* IDLE 直通：cur_loop_run 内部检测 ctrl_type==IDLE 后 PWM 置零 */
		cur_loop_run(&m->current, &m->sys.motor.ref, &m->out);
		return;
	}

	// step3: 位置环（分频）——仅 POSITION 模式需要
	if (pos_tick && m->sys.motor.ref.ctrl_type == REF_CTRL_POSITION)
		cascade_control_run_position(&m->cascade, &m->sys.motor.ref, &fb);

	// step4: 速度环 + 入环分发（分频）——跳过 VOLTAGE/DUTY/IDLE 直通模式
	if (vel_tick && m->sys.motor.ref.ctrl_type >= REF_CTRL_CURRENT)
		cascade_control_run(&m->cascade, &m->sys.motor.ref, &fb, &m->out);

	// step5: 电流环（基频）——传入完整 ref，内部按 ctrl_type 分流
	cur_loop_run(&m->current, &m->sys.motor.ref, &m->out);
}

void motor_loop_set_cmd(ctrl_mode_e cmd)
{
	process_ctrl_cmd(&s_motor_loop.sys, cmd);
	motor_loop_sync_state(&s_motor_loop);
}
