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
#include "motor_pid_profile.h"
#include "motor_pid_load.h"
#include "runtime_param.h"
#include "motion_param.h"
#include "multiturn_counter.h"
#include "motor_param.h"        /* motor_param_init 加载默认电机参数 */
#include "motor_profile.h"      /* motor_profile_apply_param / sync_to_param */
#include "motor_info_storage.h" /* motor_info_storage_get：Flash 加载的标定参数 */
#include "dev_dwt_counter.h"   /* ISR 分段耗时打点(调试期) */
#include "fault_manager.h"      /* 仲裁结果同步到 usr.motor_state.fault */
#if (MOTOR_LOOP_ENABLE_DEV_DRIVER) && defined(USE_DEV_POWER_MONITOR)
#include "dev_power_monitor.h"  /* 母线电流合成: 配置表检测 SYNTH 通道时 ISR 调用 */
#endif

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
 * @brief FOC 电弧度回调：返回当前电角度弧度（含管线延迟超前补偿）
 * @note  we = 机械角速度(PLL) × 极对数；静止/低速补偿量≈0，标定不受影响。
 */
static float motor_loop_ele_radian_cb(void)
{
	motion_param_t *mp = &s_motor_loop.motor.motor_param;
	float we = mp->slide_rad_s * (float)mp->poles;
	return mp->ele_radian + we * (FOC_ELE_ANGLE_LEAD_CYCLES * s_motor_loop.current.dt);
}

void motor_loop_init(float current_freq_hz)
{
	motor_loop_t *m = &s_motor_loop;
	motor_param_t *param = &usr.motor_param[M1];

	/* 加载默认电机参数（R/L/kt/pole_pairs/PID/限幅等）
	 * usr.motor_param[M1] 为 BSS 段全局变量，启动时全零，
	 * 若不加载默认值会导致除零、控制环无响应等问题。*/
	motor_param_init(param);
	motor_profile_apply_param(param); /* 用 motor_profile.h 的 MOTOR_* 覆盖电气身份字段 */

	/* Flash 标定参数同步到运行期 motor_param_t*/
#if defined(USE_DEV_FLASH)
	motor_profile_sync_to_param(param, motor_info_storage_get());

	/* 上电启动加载：Flash ControlParam 范围检查 → autotune 理论估计 → default 三级回退
	 * 三环各自独立判断，有效则用 Flash 值，无效则尝试 autotune，再无效用 motor_param.c 默认值 */
	motor_pid_load_boot(param, motor_info_storage_get());
	/* Flash 保存的 source 覆盖自动回退结果（用户曾显式选择的环）*/
	motor_pid_load_source_from_flash(motor_info_storage_get());
	motor_pid_load(param, motor_info_storage_get()); /* 按 source 重新加载 */
#else
	/* 未启用 Flash 存储: 用 motor_param 默认值 + motor_profile 覆盖, 不加载 Flash 标定参数 */
	motor_pid_load_boot(param, NULL);
	motor_pid_load(param, NULL);
#endif

	// 各环控制周期：电流环由中断频率决定，外环按分频系数派生
	float dt_current = 1.0f / current_freq_hz;
	float dt_velocity = dt_current * MOTOR_LOOP_VEL_DIV;
	float dt_position = dt_current * MOTOR_LOOP_POS_DIV;

	m->vel_cnt = 0;
	m->pos_cnt = 0;

	// PID 参数管理器（位置/速度/电流环共用同一套 profile 体系）
	motor_pid_profile_init(param);

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
	system_state_init(&m->sys, &m->motor, param, dt_current);

#if defined(USE_DEV_FLASH)
	// 缓启动渐变配置: motor_info(EEPROM优先加载) → 运行时 smooth_cfg
	// (softstart_valid=0 时跳过, 保持 ref_smooth_cfg_init_defaults 编译期默认)
	transition_mgr_apply_softstart(&m->sys.trans_mgr, motor_info_storage_get());

	// 故障管理配置重新加载: 补偿 fault_mgr_init 在前执行时 cfg 被默认值覆盖
	// (user_interface.c 已调整顺序, 此处为双重保险, 确保持久化配置生效)
	motor_profile_sync_fault_cfg_reload(motor_info_storage_get());
#endif

	motor_loop_sync_state(m);

	// 级联外环（位置/速度）与电流环
	cascade_control_init(&m->cascade, param, dt_position, dt_velocity);
	cur_loop_init(&m->current, &m->motor, param, dt_current);

	// 注入组先使能(ADC 注入组 + JEOC/JEOS 完成中断), 具体事件由 EOCSelection 选择；
	// 转换由 TIM1_CC4 硬件触发,
	// 必须等 half_bridge.start 启动 TIM1 后才会有转换, JDR 才有有效值。
	m->motor.phase_current.start(&m->motor.phase_current);

	// 启动半桥: TIM1 计数器开始运行, CC4 触发 ADC 注入组转换。
	// 此时三相 PWM 默认 CCR=0(0%占空比, 下桥全导通, 三相绕组接GND, 无电位差, 电流=0),
	// 正是标定 INA199B1 零位(输出=REF)所需工况; 控制环 ISR 已开始跑(IDLE态安全)。
	m->motor.half_bridge.start(&m->motor.half_bridge);

	// 等待若干 PWM 周期, 确保 ADC 完成首次注入转换, JDR 寄存器已更新为真实采样值
	HAL_Delay(2);

	// 电流采样零位校准(此时 INA199B1 输出=REF, 标定的是真正的零位)
	cur_loop_calibrate_offset(&m->current);

	drv_tim_start_it(m->motor.fsm_tim); // 启动控制定时器
}

/**
 * @brief 运行时翻转编码器方向(换电机/换安装后调试用)
 * @note  直接调用设备层接口, 同步更新 mt6701.dir 和 motor_param 持久化字段。
 *        应在 IDLE 状态调用, 切换后须重新做编码器零位标定(enc_offset)。
 *        虚拟电机模式下为空实现(无真实编码器)。
 */
void motor_loop_set_encoder_dir(int8_t dir)
{
#if (MOTOR_LOOP_ENABLE_DEV_DRIVER)
	motor_loop_t *m = &s_motor_loop;
	dev_motor_set_encoder_dir(&m->motor, dir);
#else
	(void)dir; /* 虚拟电机模式: 无真实编码器, 空实现 */
#endif
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
	const motor_state_t *st = &usr.motor_state[M1];

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
	m->sys.motor.fb.bus_voltage = st->power.v_bus;
	/* 编码器健康信息(故障检测数据源; 虚拟模式方法为 NULL 恒健康) */
	m->sys.motor.fb.mech_angle_deg = m->motor.encoder.mechanical_angle;
	m->sys.motor.fb.enc_err_cnt = (m->motor.encoder.get_err_cnt != NULL)
	                                 ? m->motor.encoder.get_err_cnt(&m->motor.encoder)
	                                 : 0u;
	m->sys.motor.fb.enc_health = (m->motor.encoder.get_health != NULL)
	                                 ? m->motor.encoder.get_health(&m->motor.encoder)
	                                 : 0u;
	/* 栅极驱动器硬件故障(nFAULT): 真实驱动每拍读引脚; 虚拟电机/未连接引脚的板型恒 0 */
#if (MOTOR_LOOP_ENABLE_DEV_DRIVER)
	m->sys.motor.fb.gate_driver_fault = dev_motor_gate_driver_fault();
#else
	m->sys.motor.fb.gate_driver_fault = 0u;
#endif
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
	st->fault.fault_mask = sys->fault_code;
	st->fault.fault_latched = sys->fault_latched;
	st->fault.error_count = sys->fault_count;
	st->fault.last_fault_code = sys->last_fault_code;
	/* fault_mgr 仲裁结果(0xAA/0xAB 查询与 LED 提示源) */
	st->fault.warn_mask = fault_mgr_get_warn_mask();
	st->fault.top_fault_code = fault_mgr_get_top_fault();
	st->fault.active_count = (uint8_t)fault_mgr_active_count();
	st->fault.derate_pct = (uint8_t)(fault_mgr_get_derate() * 100.0f + 0.5f);
	st->fault.level_active = fault_mgr_level_active();
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
	// 每拍必须无条件刷新: step1 的 ELE_VEL/ALL 解算传入的是 mp->mechanical_angle
	// (motion_param 内部缓存的上拍值), 依赖本拍此处写入最新编码器角度,
	// 跳过刷新会导致 vel/pos 拍电角度滞后一拍(高速时>25°电角度错位, FOC失控过流)
	dev_dwt_counter_start(SYS_TIMER_RECORD_TEST_1); /* 分段耗时: 编码器+电角度 */
	m->motor.encoder.update(&m->motor.encoder);
	m->motor.motor_param.update(&m->motor.motor_param, MOTION_TYPE_ELE_RADIAN, m->motor.encoder.mechanical_angle);
	dev_dwt_counter_stop(SYS_TIMER_RECORD_TEST_1);

	// step1: 解算运动反馈（使用本拍刷新的角度）
	dev_dwt_counter_start(SYS_TIMER_RECORD_TEST_2); /* 分段耗时: 反馈解算+状态机 */
	motor_loop_update_feedback(m, &fb, vel_tick, pos_tick);

	// step2: 状态机生成参考输出 motor.ref（含模式管理与平滑过渡）
	motor_control_loop(&m->sys);
	motor_loop_sync_state(m);
	dev_dwt_counter_stop(SYS_TIMER_RECORD_TEST_2);

	// CALIB 态：标定模块在 calib_mgr_poll() 中直接操作 FOC 链路施加电压，
	// 不走 cur_loop_run（避免被 IDLE 直通覆盖为零 PWM）
	// 但须刷新相电流采样, 供 R/Ld/Lq/flux 标定调用 foc.clarke/park 读取实时电流
	if (m->sys.top_state == TOP_FSM_CALIB)
	{
		m->motor.phase_current.update(&m->motor.phase_current);
		return;
	}

	// READY 态：采样三相电流 + FOC 变换(Clarke/Park)，但 PWM 输出零
	// 下管全导通(CCR=0)，三相绕组接GND，无电位差，电流为零，电机不转动
	if (m->sys.top_state == TOP_FSM_READY)
	{
		cascade_control_reset(&m->cascade);
		cur_loop_reset(&m->current);
		m->out.id_ref = 0.0f;
		m->out.iq_ref = 0.0f;

		m->motor.phase_current.update(&m->motor.phase_current);
		m->motor.foc.clarke(&m->motor.foc);
		m->motor.foc.park(&m->motor.foc);
		m->motor.half_bridge.set_3pwm(&m->motor.half_bridge, 0, 0, 0);
		return;
	}

	// 其他非运行态（IDLE/FAULT/SAFETY）：外环复位，PWM 置零
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
	dev_dwt_counter_start(SYS_TIMER_RECORD_TEST_3); /* 分段耗时: 外环+电流环(RUN 主路径) */
	if (pos_tick && m->sys.motor.ref.ctrl_type == REF_CTRL_POSITION)
		cascade_control_run_position(&m->cascade, &m->sys.motor.ref, &fb);

	// step4: 速度环 + 入环分发（分频）——跳过 VOLTAGE/DUTY/IDLE 直通模式
	if (vel_tick && m->sys.motor.ref.ctrl_type >= REF_CTRL_CURRENT)
		cascade_control_run(&m->cascade, &m->sys.motor.ref, &fb, &m->out);

	// step5: 电流环（基频）——传入完整 ref，内部按 ctrl_type 分流
	cur_loop_run(&m->current, &m->sys.motor.ref, &m->out);

#if (MOTOR_LOOP_ENABLE_DEV_DRIVER) && defined(USE_DEV_POWER_MONITOR)
	/* step6: 合成母线电流 (SYNTH 源, 10kHz 高频)
	 * 仅板级配置了 PM_CH_IBUS_SYNTH 通道时执行; SFOC 板配置 IBUS_HW 不执行
	 * 仅 RUN 态执行: CALIB/IDLE 态 PWM 未真正驱动电机, ibus 保持上次值
	 * 公式: Ibus = da*Ia + db*Ib + dc*Ic (功率守恒推导)
	 * has_channel 检测 const 配置表, 编译器可常量折叠, 运行期无开销 */
	if (dev_power_monitor_has_channel(PM_CH_IBUS_SYNTH))
	{
		const foc_t *foc = &m->motor.foc;
		const dev_phase_current_t *pc = &m->motor.phase_current;
		dev_power_monitor_synthesize_ibus(pc->current.a, pc->current.b, pc->current.c,
		                                  foc->svpwm.ta, foc->svpwm.tb, foc->svpwm.tc);
	}
#endif

	dev_dwt_counter_stop(SYS_TIMER_RECORD_TEST_3);
}

void motor_loop_set_cmd(ctrl_mode_e cmd)
{
	process_ctrl_cmd(&s_motor_loop.sys, cmd);
	motor_loop_sync_state(&s_motor_loop);
}
