/**
 * @file        system_state.c
 * @brief 系统状态机实现文件
 * @details 仅负责状态管理：顶层主状态机表驱动转移 + 进入/退出动作，
 *          控制逻辑(参考生成)经 motor_ctrl_dispatch 调用，本文件不含。
 *
 * @author      yangsl (yangsl@robot.com)
 * @version     1.1
 * @date        2026-06-11
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @par 修改日志:
 * | 日期       | 版本 | 作者   | 修改内容                              |
 * |------------|------|--------|---------------------------------------|
 * | 2026-06-11 | 1.0  | yangsl | 初始创建                              |
 * | 2026-06-12 | 1.1  | yangsl | 加 READY 态/转移许可表，剥离控制逻辑  |
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 */

#include "system_state.h"
#include "calib_mgr.h"
#include <string.h>

/**
 * @brief 运行模式平滑过渡的调用次数
 * @details 过渡时长以 motor_control_loop（建议置于电流环）的调用次数计，
 *          而非软件定时器。可在运行期配置：调用次数 = 期望过渡时长 / 电流环周期。
 *          例：电流环 50us，期望过渡 5ms，则置为 100。
 */
uint32_t g_run_state_trans_count = 1000;

#define SYSTEM_SPEED_GUARD_SECONDS (0.020f)

static int state_float_is_finite(float value)
{
	uint32_t bits;
	memcpy(&bits, &value, sizeof(bits));
	return (bits & 0x7F800000u) != 0x7F800000u;
}

static float state_absf(float value)
{
	return (value < 0.0f) ? -value : value;
}

static uint8_t state_first_fault(uint32_t mask)
{
	uint8_t bit;
	for (bit = 0u; bit < 32u; bit++)
	{
		if ((mask & (1u << bit)) != 0u)
			return bit;
	}
	return 0u;
}

static uint32_t state_active_faults(system_state_t *sys)
{
	const protection_param_t *p = &sys->motor.param->protection_param;
	const motor_fb_t *fb = &sys->motor.fb;
	uint32_t active = 0u;
	uint32_t enable = p->protect_enable_mask;
	float current_limit = p->protect_over_current;
	float bus = fb->bus_voltage;

	if (!state_float_is_finite(fb->id) || !state_float_is_finite(fb->iq) || !state_float_is_finite(fb->vel) || !state_float_is_finite(bus))
		active |= SYSTEM_FAULT_NUMERIC;

	if ((enable & SYSTEM_PROTECT_OVER_CURRENT) != 0u && state_float_is_finite(fb->id) && state_float_is_finite(fb->iq) && (fb->id * fb->id + fb->iq * fb->iq) > current_limit * current_limit)
		active |= SYSTEM_FAULT_OVER_CURRENT;

	if (state_float_is_finite(bus) && bus > 1.0f)
		sys->power_sample_valid = 1u;
	if (sys->power_sample_valid)
	{
		if ((enable & SYSTEM_PROTECT_OVER_VOLTAGE) != 0u && bus > p->protect_over_voltage)
			active |= SYSTEM_FAULT_OVER_VOLTAGE;
		if ((enable & SYSTEM_PROTECT_UNDER_VOLTAGE) != 0u && sys->top_state != TOP_FSM_IDLE && bus < p->protect_under_voltage)
			active |= SYSTEM_FAULT_UNDER_VOLTAGE;
	}

	/* 编码器冷启动阶段可能先返回无效角度，随后跳到真实角度。
	 * 仅在输出已使能且启动保护窗口结束后检测超速，避免 IDLE 态误锁存。 */
	if (sys->speed_guard_cycles == 0u && (sys->top_state == TOP_FSM_READY || sys->top_state == TOP_FSM_RUN) && (enable & SYSTEM_PROTECT_OVER_SPEED) != 0u && state_float_is_finite(fb->vel) && state_absf(fb->vel) > p->protect_over_speed)
		active |= SYSTEM_FAULT_OVER_SPEED;

	return active;
}

/**
 * @brief 控制指令到运行状态的映射表
 * @details 上层运动控制指令（ctrl_mode_e）到底层运行状态（run_state_e）的映射。
 */
static const run_state_e s_ctrl_mode_to_run_state[CONTROL_MODE_MAX] = {
	[CONTROL_MODE_IDLE] = RUN_STATE_IDLE,
	[CONTROL_MODE_HOLD] = RUN_STATE_HOLD,
	[CONTROL_MODE_BRAKE] = RUN_STATE_HOLD, /* 刹车=位置保持 */

	[CONTROL_MODE_OPEN_LOOP] = RUN_STATE_OPEN_LOOP,
	[CONTROL_MODE_CURRENT] = RUN_STATE_CURRENT,
	[CONTROL_MODE_TORQUE] = RUN_STATE_TORQUE,
	[CONTROL_MODE_MIT] = RUN_STATE_MIT,
	[CONTROL_MODE_VELOCITY] = RUN_STATE_VELOCITY,
	[CONTROL_MODE_POSITION] = RUN_STATE_POSITION,
	[CONTROL_MODE_POSITION_VELOCITY] = RUN_STATE_POSITION_VELOCITY,
	[CONTROL_MODE_POSITION_TORQUE] = RUN_STATE_POSITION_TORQUE,
	[CONTROL_MODE_VELOCITY_TORQUE] = RUN_STATE_VELOCITY_TORQUE,
	[CONTROL_MODE_DUTY_CYCLE] = RUN_STATE_DUTY_CYCLE,
	[CONTROL_MODE_VOLTAGE_VECTOR] = RUN_STATE_VOLTAGE_VECTOR,
	[CONTROL_MODE_FIELD_WEAKENING] = RUN_STATE_FIELD_WEAKENING,
	[CONTROL_MODE_SENSORLESS] = RUN_STATE_SENSORLESS,

	[CONTROL_MODE_IMPEDANCE] = RUN_STATE_IMPEDANCE,
	[CONTROL_MODE_ADMITTANCE] = RUN_STATE_ADMITTANCE,
	[CONTROL_MODE_FORCE_CONTROL] = RUN_STATE_FORCE_CONTROL,
	[CONTROL_MODE_FORCE_POSITION_HYBRID] = RUN_STATE_FORCE_POSITION_HYBRID,
	[CONTROL_MODE_GRAVITY_COMPENSATION] = RUN_STATE_GRAVITY_COMPENSATION,
	[CONTROL_MODE_COLLISION_DETECTION] = RUN_STATE_COLLISION_DETECTION,
	[CONTROL_MODE_ZERO_FORCE] = RUN_STATE_ZERO_FORCE,
	[CONTROL_MODE_CONSTANT_FORCE] = RUN_STATE_CONSTANT_FORCE,
	[CONTROL_MODE_VARIABLE_IMPEDANCE] = RUN_STATE_VARIABLE_IMPEDANCE,
	[CONTROL_MODE_ADAPTIVE_GRAVITY_COMP] = RUN_STATE_ADAPTIVE_GRAVITY_COMP,
	[CONTROL_MODE_LANDING_BUFFER] = RUN_STATE_LANDING_BUFFER,

	[CONTROL_MODE_PVT] = RUN_STATE_PVT,
	[CONTROL_MODE_CUBIC_SPLINE] = RUN_STATE_CUBIC_SPLINE,
	[CONTROL_MODE_TRAPEZOIDAL_TRAJ] = RUN_STATE_TRAPEZOIDAL_TRAJ,
	[CONTROL_MODE_S_CURVE_TRAJ] = RUN_STATE_S_CURVE_TRAJ,
	[CONTROL_MODE_HOMING] = RUN_STATE_HOMING,
	[CONTROL_MODE_CANOPEN_SYNC] = RUN_STATE_POSITION, /* SYNC 同步位置 */
	[CONTROL_MODE_ETHERCAT_CSP] = RUN_STATE_POSITION, /* CSP = Cyclic Sync Position */
	[CONTROL_MODE_ETHERCAT_CSV] = RUN_STATE_VELOCITY, /* CSV = Cyclic Sync Velocity */
	[CONTROL_MODE_ETHERCAT_CST] = RUN_STATE_TORQUE,   /* CST = Cyclic Sync Torque */
	[CONTROL_MODE_PP] = RUN_STATE_POSITION,           /* Profile Position（前期复用 POSITION）*/
	[CONTROL_MODE_PV] = RUN_STATE_PROFILE_VELOCITY,   /* Profile Velocity → 独立模式文件 */
	[CONTROL_MODE_PT] = RUN_STATE_PROFILE_TORQUE,     /* Profile Torque → 独立模式文件 */
	[CONTROL_MODE_ELECTRONIC_GEAR] = RUN_STATE_ELECTRONIC_GEAR,
	[CONTROL_MODE_ELECTRONIC_CAM] = RUN_STATE_ELECTRONIC_CAM,

	[CONTROL_MODE_STEP_DIR] = RUN_STATE_STEP_DIR,
	[CONTROL_MODE_ANALOG_INPUT] = RUN_STATE_ANALOG_INPUT,
	[CONTROL_MODE_PWM_INPUT] = RUN_STATE_PWM_INPUT,
	[CONTROL_MODE_JOG] = RUN_STATE_JOG,
	[CONTROL_MODE_SAFE_TEACH] = RUN_STATE_SAFE_TEACH,

	[CONTROL_MODE_TEST_AGING] = RUN_STATE_TEST_AGING,
	[CONTROL_MODE_TEST_SWEEP_FREQ] = RUN_STATE_TEST_SWEEP_FREQ,
	[CONTROL_MODE_TEST_COGGING] = RUN_STATE_TEST_COGGING,
	[CONTROL_MODE_TEST_FRICTION] = RUN_STATE_TEST_FRICTION,
	[CONTROL_MODE_TEST_INERTIA] = RUN_STATE_TEST_INERTIA,
	[CONTROL_MODE_DIAGNOSTIC] = RUN_STATE_DIAGNOSTIC,
	[CONTROL_MODE_HIGH_SPEED_DAQ] = RUN_STATE_HIGH_SPEED_DAQ,
	[CONTROL_MODE_SINGLE_STEP] = RUN_STATE_SINGLE_STEP,
};

/**
 * @brief 顶层状态转移许可表
 * @details 表驱动的状态机：s_top_fsm_allowed[from][to] 为 true 表示允许该转移。
 *          FAULT/SAFETY 作为最高优先级，可从任意状态进入（在判断函数中单独处理）。
 *          对照状态机图核对/修改本表即可调整全部转移规则。
 *  
 *          转移逻辑：
 *            INIT  → IDLE
 *            IDLE  → READY / CALIB / CONFIG / BOOTLOADER
 *            READY → RUN / IDLE
 *            RUN   → READY            （停止运行后回就绪，再回 IDLE 下使能）
 *            FAULT → IDLE             （清除故障）
 *            SAFETY→ IDLE             （解除急停）
 *            CALIB → IDLE
 *            CONFIG→ IDLE
 *            BOOTLOADER→ IDLE
 */
static const bool s_top_fsm_allowed[TOP_FSM_MAX][TOP_FSM_MAX] = {
	[TOP_FSM_INIT] = {[TOP_FSM_IDLE] = true},
	[TOP_FSM_IDLE] = {[TOP_FSM_READY] = true, [TOP_FSM_CALIB] = true, [TOP_FSM_CONFIG] = true, [TOP_FSM_BOOTLOADER] = true},
	[TOP_FSM_READY] = {[TOP_FSM_RUN] = true, [TOP_FSM_IDLE] = true, [TOP_FSM_CALIB] = true},
	[TOP_FSM_RUN] = {[TOP_FSM_READY] = true},
	[TOP_FSM_FAULT] = {[TOP_FSM_IDLE] = true},
	[TOP_FSM_SAFETY] = {[TOP_FSM_IDLE] = true},
	[TOP_FSM_CALIB] = {[TOP_FSM_IDLE] = true},
	[TOP_FSM_CONFIG] = {[TOP_FSM_IDLE] = true},
	[TOP_FSM_BOOTLOADER] = {[TOP_FSM_IDLE] = true},
};

/**
 * @brief 判断顶层状态转移是否合法
 * @param from 源状态
 * @param to 目标状态
 * @return true 允许 / false 禁止
 * @note FAULT 与 SAFETY 可从任意状态进入（最高优先级），不受许可表限制。
 */
static bool top_fsm_transition_allowed(top_fsm_e from, top_fsm_e to)
{
	if (to >= TOP_FSM_MAX || from >= TOP_FSM_MAX)
		return false;

	// 故障/安全：任意状态可进入
	if (to == TOP_FSM_FAULT || to == TOP_FSM_SAFETY)
		return true;

	return s_top_fsm_allowed[from][to];
}

/**
 * @brief 初始化系统状态机（具体实现）
 */
void system_state_init(system_state_t *sys, struct dev_motor *motor, motor_param_t *param, float dt)
{
	memset(sys, 0, sizeof(system_state_t));

	motor_ctrl_init(&sys->motor, param, dt);
	calib_mgr_init(motor, param, dt);
	transition_mgr_init(&sys->trans_mgr, dt);

	sys->top_state = TOP_FSM_INIT;
	sys->ctrl_mode = CONTROL_MODE_IDLE;
	sys->fault_code = 0;
	if (dt > 0.0f)
	{
		sys->speed_guard_cycles = (uint32_t)(SYSTEM_SPEED_GUARD_SECONDS / dt + 0.5f);
	}
	if (sys->speed_guard_cycles == 0u)
	{
		sys->speed_guard_cycles = 1u;
	}

	top_fsm_switch(sys, TOP_FSM_IDLE);
}

/**
 * @brief 切换顶层有限状态机状态（具体实现）
 * @details 表驱动 + 进入/退出动作。非法转移直接拒绝。
 */
void top_fsm_switch(system_state_t *sys, top_fsm_e new_state)
{
	// 没有切换状态就直接返回
	if (new_state == sys->top_state)
		return;

	// 转移许可表校验
	if (!top_fsm_transition_allowed(sys->top_state, new_state))
		return;

	switch (sys->top_state)
	{
		case TOP_FSM_RUN:
			// 退出运行：停止参考输出，强制结束过渡
			sys->motor.ref.ctrl_type = REF_CTRL_IDLE;
			transition_mgr_on_top_fsm_change(&sys->trans_mgr, new_state);
			break;

		case TOP_FSM_CALIB:
			calib_mgr_abort();
			sys->calib_state = CALIB_STATE_IDLE;
			break;
		case TOP_FSM_CONFIG:
			break;

		default:
			break;
	}

	/* ---- 进入动作（进入新状态的初始化）---- */
	switch (new_state)
	{
		case TOP_FSM_IDLE:
			// 待机：伺服失能，运行子状态归零
			sys->motor.run_state = RUN_STATE_IDLE;
			sys->motor.ref.ctrl_type = REF_CTRL_IDLE;
			break;

		case TOP_FSM_READY:
			// 就绪：已使能但不运动，运行子状态置空闲保持
			sys->motor.run_state = RUN_STATE_IDLE;
			sys->motor.ref.ctrl_type = REF_CTRL_IDLE;
			break;

		case TOP_FSM_FAULT:
		case TOP_FSM_SAFETY:
			// 故障/急停：立即失能输出
			sys->motor.ref.ctrl_type = REF_CTRL_IDLE;
			transition_mgr_on_top_fsm_change(&sys->trans_mgr, new_state);
			break;

		case TOP_FSM_CALIB:
			sys->calib_state = CALIB_STATE_IDLE;
			sys->motor.ref.ctrl_type = REF_CTRL_IDLE;
			break;

		default:
			break;
	}

	sys->top_state = new_state;
}

/**
 * @brief 切换电机运行状态（具体实现，仅在 RUN 态内有效）
 * @details 启动参考层平滑过渡
 */
void run_state_switch(system_state_t *sys, run_state_e new_state, uint32_t trans_count)
{
	if (new_state >= RUN_STATE_MAX || new_state == sys->motor.run_state)
		return;
	/* 扫频带有内部时序和 TRACE 点边界，不能在普通参考平滑过渡期间
	 * 被临时 dispatch，否则会消耗 settle/measure 计数而尚未真正输出
	 * 扫频参考。直接进入子状态，首个控制周期从相位 0 开始。 */
	if (new_state == RUN_STATE_TEST_SWEEP_FREQ)
	{
		transition_force_complete(&sys->trans_mgr.trans);
		sys->trans_mgr.target_run_state = new_state;
		sys->motor.run_state = new_state;
		return;
	}

	transition_mgr_on_mode_switch(&sys->trans_mgr, new_state, trans_count, &sys->motor.ref);
}

/**
 * @brief 电机控制主循环（具体实现）
 * @details 仅 RUN 态生成运动参考；其余状态参考保持 IDLE。
 *          控制逻辑（参考生成）经 motor_ctrl_dispatch 调用，本文件不含。
 */
void motor_control_loop(system_state_t *sys)
{
	/* 故障保护必须位于参考生成之前；扫频在本周期不得再产生新的激励。 */
	fault_check(sys);
	if (sys->top_state == TOP_FSM_FAULT || sys->top_state == TOP_FSM_SAFETY)
	{
		sys->motor.ref.ctrl_type = REF_CTRL_IDLE;
		return;
	}
	/* CALIB 态：周期推进标定，不生成运动参考 */
	if (sys->top_state == TOP_FSM_CALIB)
	{
		sys->calib_state = calib_mgr_poll();
		sys->motor.ref.ctrl_type = REF_CTRL_IDLE;
		return;
	}

	// 非运行态：失能输出，不生成运动参考
	if (sys->top_state != TOP_FSM_RUN)
	{
		sys->motor.ref.ctrl_type = REF_CTRL_IDLE;
		return;
	}

	// 运行态：所有过渡策略收敛到 transition_mgr_step
	transition_mgr_step(&sys->trans_mgr, sys);
	if (sys->motor.run_state == RUN_STATE_TEST_SWEEP_FREQ &&
		motor_sweep_is_complete(&sys->motor) &&
		sys->motor.sweep_finish_pending == 0u)
	{
		sys->motor.ref.ctrl_type = REF_CTRL_IDLE;
		top_fsm_switch(sys, TOP_FSM_READY);
	}
}

/**
 * @brief 系统故障检测（具体实现，预留）
 */
void fault_check(system_state_t *sys)
{
	uint32_t active;
	uint32_t new_faults;

	if (sys == NULL || sys->motor.param == NULL)
		return;
	if (sys->speed_guard_cycles > 0u)
		sys->speed_guard_cycles--;
	active = state_active_faults(sys);
	new_faults = active & ~sys->fault_latched;
	sys->fault_code = active;
	if (active == 0u)
		return;

	if (new_faults != 0u)
	{
		if (sys->fault_count != 0xFFFFu)
			sys->fault_count++;
		sys->last_fault_code = state_first_fault(new_faults);
	}
	motor_sweep_abort(&sys->motor);
	sys->fault_latched |= active;
	sys->motor.ref.ctrl_type = REF_CTRL_IDLE;
	top_fsm_switch(sys, TOP_FSM_FAULT);
}

/**
 * @brief 处理上层控制指令（具体实现）
 * @details 指令分发与状态切换，三段式使能流程：
 *            IDLE --ENABLE--> READY --运动指令--> RUN
 *            RUN  --STOP----> READY --DISABLE--> IDLE
 *          系统指令（IDLE/ENABLE/DISABLE/STOP/ESTOP/BOOTLOADER等）单独处理；
 *          运动指令仅在 READY/RUN 态被接受，经映射表转为运行子状态。
 */
void process_ctrl_cmd(system_state_t *sys, ctrl_mode_e cmd)
{
	if (cmd >= CONTROL_MODE_MAX)
		return;

	/* ---- 急停：任意状态最高优先级响应 ---- */
	if (cmd == CONTROL_MODE_ESTOP)
	{
		sys->ctrl_mode = cmd;
		top_fsm_switch(sys, TOP_FSM_SAFETY);
		return;
	}

	/* ---- STOP 延迟过渡期间：拒绝其他命令（ESTOP 已上面处理）---- */
	if (sys->trans_mgr.stop_pending)
		return;

	/* ---- 故障态：仅响应清除故障 ---- */
	if (sys->top_state == TOP_FSM_FAULT)
	{
		if (cmd == CONTROL_MODE_CLEAR_FAULT)
		{
			fault_check(sys);
			if (sys->fault_code == 0u)
			{
				sys->fault_latched = 0u;
				sys->ctrl_mode = cmd;
				top_fsm_switch(sys, TOP_FSM_IDLE);
			}
		}
		return;
	}

	/* ---- 安全态：仅响应清除故障（解除急停）---- */
	if (sys->top_state == TOP_FSM_SAFETY)
	{
		if (cmd == CONTROL_MODE_CLEAR_FAULT)
		{
			fault_check(sys);
			if (sys->fault_code == 0u)
			{
				sys->fault_latched = 0u;
				sys->ctrl_mode = cmd;
				top_fsm_switch(sys, TOP_FSM_IDLE);
			}
		}
		return;
	}

	/* ---- 系统级指令 ---- */
	switch (cmd)
	{
		case CONTROL_MODE_IDLE:
		case CONTROL_MODE_DISABLE:
			// 下使能：READY/RUN 经许可表逐级回到 IDLE
			if (sys->top_state == TOP_FSM_RUN)
				top_fsm_switch(sys, TOP_FSM_READY);

			top_fsm_switch(sys, TOP_FSM_IDLE);
			sys->ctrl_mode = cmd;
			return;

		case CONTROL_MODE_ENABLE:
			// 上使能：IDLE → READY
			top_fsm_switch(sys, TOP_FSM_READY);
			if (sys->top_state == TOP_FSM_READY)
				sys->ctrl_mode = cmd;
			return;

		case CONTROL_MODE_STOP:
			/* 停止运行：RUN → READY（保持使能）
			 * 支持停机过渡的模式：启动延迟过渡，保持当前模式减速到零，完成后自动切 READY
			 * 不支持的模式（MIT/HOLD/直控）：立即切 READY */
			if (sys->top_state == TOP_FSM_RUN)
			{
				sys->ctrl_mode = cmd;
				if (!transition_mgr_on_stop(&sys->trans_mgr, sys))
					top_fsm_switch(sys, TOP_FSM_READY);
			}
			else if (sys->top_state == TOP_FSM_READY)
			{
				sys->ctrl_mode = cmd;
			}
			return;

		case CONTROL_MODE_ENTER_BOOTLOADER:
			top_fsm_switch(sys, TOP_FSM_BOOTLOADER);
			if (sys->top_state == TOP_FSM_BOOTLOADER)
				sys->ctrl_mode = cmd;
			return;

			/* 校准指令：进入 CALIB 状态
			 * 0x90-0x96: 启动标定（子模式已由 app_set_mode 传给 calib_mgr）
			 * 0x97/0x98: 查询/中止，不切状态（app_set_mode 已处理并 return）*/
		case CONTROL_MODE_CALIB_LEVEL1:
		case CONTROL_MODE_CALIB_LEVEL2:
		case CONTROL_MODE_CALIB_LEVEL3:
		case CONTROL_MODE_CALIB_LEVEL4:
		case CONTROL_MODE_CALIB_LEVEL5:
		case CONTROL_MODE_CALIB_LEVEL6:
		case CONTROL_MODE_CALIB_LEVEL7:
			/* IDLE 或 READY 态可进入校准；RUN 态需先停止再标定 */
			if (sys->top_state == TOP_FSM_IDLE || sys->top_state == TOP_FSM_READY)
			{
				top_fsm_switch(sys, TOP_FSM_CALIB);
				if (sys->top_state == TOP_FSM_CALIB)
					sys->ctrl_mode = cmd;
				else
					calib_mgr_abort(); /* 状态切换失败，回滚标定避免卡死 */
			}
			else if (sys->top_state == TOP_FSM_CALIB)
			{
				/* 已在 CALIB 态：上一个标定已 DONE/FAILED，calib_mgr_start 已成功启动新标定，
				 * 直接接受即可，不得 abort（否则会终止刚启动的新标定）*/
				sys->ctrl_mode = cmd;
			}
			else
			{
				/* 非法状态（RUN/FAULT/SAFETY等），回滚 calib_mgr_start */
				calib_mgr_abort();
			}
			return;

		case CONTROL_MODE_CALIB_QUERY:
		case CONTROL_MODE_CALIB_ABORT:
			/* 查询/中止不切状态，app_set_mode 已处理 */
			return;

		case CONTROL_MODE_SAVE_CONFIG:
			sys->ctrl_mode = cmd;
			return;

		case CONTROL_MODE_FACTORY_RESET:
			sys->ctrl_mode = cmd;
			return;

		default:
			break;
	}

	/* ---- 运动控制指令：需已使能（READY 或 RUN）---- */
	if (sys->top_state == TOP_FSM_READY)
	{
		/* READY → RUN：首次进入也走平滑过渡，避免位置阶跃 */
		top_fsm_switch(sys, TOP_FSM_RUN);
		run_state_e target = s_ctrl_mode_to_run_state[cmd];

		/* 进入 RUN 前把 motor.ref 设为 IDLE（保持当前位置）作为过渡起点 */
		sys->motor.ref.ctrl_type = REF_CTRL_IDLE;
		sys->motor.ref.pos = sys->motor.fb.pos;
		sys->motor.ref.vel = 0.0f;
		sys->motor.ref.torque = 0.0f;

		run_state_switch(sys, target, g_run_state_trans_count);
		sys->ctrl_mode = cmd;
	}
	else if (sys->top_state == TOP_FSM_RUN)
	{
		// RUN 态内运动模式切换：走平滑过渡，时长由全局调用次数配置
		run_state_e target = s_ctrl_mode_to_run_state[cmd];
		run_state_switch(sys, target, g_run_state_trans_count);
		sys->ctrl_mode = cmd;
	}
}
