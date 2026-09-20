/**
 * @file motor_mode_load_sim.c
 * @brief 对拖台负载模拟模式 (0x60~0x67)
 * @details 本电机作为对拖台的加载电机，输出与被测电机方向相反的制动转矩。
 *          第一版仅实现 0x60 被动恒转矩；0x61~0x67 由本文件公共框架扩展，
 *          app_mode_is_supported 未放行前不可达。
 *          指令参数与运行时状态内聚于 motor_ctrl_t.load_sim，模式进入时经
 *          motor_load_sim_reset 复位，参数热更新不复位相位。
 */
#include "motor_mode.h"
#include <math.h>

/* 0x60 输出转矩斜坡速率(N·m/s): 目标转矩经速率限制后输出,
 * 方向判定抖动/换向时给定转矩缓慢过渡, 换向必经平滑过零,
 * 根除转矩在 ±t_set 间跳变引发的机械共振。 */
#define LOAD_SIM_TORQUE_RAMP_NM_S 5.0f

/* 低速方向判定参数。滤波只用于方向判定，不改变 FOC 电流环反馈。 */
#define LOAD_SIM_VEL_FILTER_BW_HZ       10.0f
#define LOAD_SIM_DIR_CONFIRM_MS         50.0f
#define LOAD_SIM_RELEASE_CONFIRM_MS     80.0f

/* 进入负载模拟模式时复位运行时状态，由 run_state_switch 在过渡启动前调用 */
void motor_load_sim_reset(motor_ctrl_t *ctrl)
{
	motor_load_sim_t *ls = &ctrl->load_sim;

	ls->phase_acc = 0u;
	ls->elapsed_ticks = 0u;
	ls->dir_state = 0.0f;
	ls->torque_out = 0.0f;
	ls->vel_filt = ctrl->fb.vel;
	ls->dir_candidate = 0.0f;
	ls->dir_confirm_ticks = 0u;
	ls->release_ticks = 0u;
	ls->vel_prev = ctrl->fb.vel;
	ls->a_est = 0.0f;
	ls->overload_active = 1u;
	ls->overload_ticks = 0u;
	ls->cooldown_ticks = 0u;
}

/* Backward-Euler first-order low-pass coefficient. */
static float load_filter_alpha(float dt)
{
	float x;

	if (dt <= 0.0f)
		return 1.0f;
	x = 6.28318530718f * LOAD_SIM_VEL_FILTER_BW_HZ * dt;
	return x / (1.0f + x);
}

static uint32_t load_time_to_ticks(float time_ms, float dt)
{
	float ticks;

	if (dt <= 0.0f)
		return 1u;
	ticks = (time_ms * 0.001f) / dt;
	if (ticks < 1.0f)
		return 1u;
	return (uint32_t)(ticks + 0.5f);
}

static int load_velocity_sign(float velocity, float threshold)
{
	if (velocity >= threshold)
		return 1;
	if (velocity <= -threshold)
		return -1;
	return 0;
}

/* Update the direction state machine with filtered velocity and time confirmation. */
static void load_update_direction(motor_ctrl_t *ctrl, float velocity, float dead)
{
	motor_load_sim_t *ls = &ctrl->load_sim;
	float enter_threshold = dead;
	float exit_threshold = dead * 0.5f;
	uint32_t confirm_ticks = load_time_to_ticks(LOAD_SIM_DIR_CONFIRM_MS, ctrl->dt);
	uint32_t release_ticks = load_time_to_ticks(LOAD_SIM_RELEASE_CONFIRM_MS, ctrl->dt);
	int current = (ls->dir_state > 0.0f) ? 1 :
		(ls->dir_state < 0.0f) ? -1 : 0;
	int candidate;

	if (current == 0)
	{
		candidate = load_velocity_sign(velocity, enter_threshold);
		ls->release_ticks = 0u;

		if (candidate != (int)ls->dir_candidate)
		{
			ls->dir_candidate = (float)candidate;
			ls->dir_confirm_ticks = 0u;
		}
		if (candidate == 0)
		{
			ls->dir_confirm_ticks = 0u;
		}
		else if (++ls->dir_confirm_ticks >= confirm_ticks)
		{
			ls->dir_state = (float)candidate;
			ls->dir_candidate = 0.0f;
			ls->dir_confirm_ticks = 0u;
		}
		return;
	}

	/* A reversal must remain in the opposite direction for the same
	 * confirmation interval. The torque calculation below forces zero while
	 * this is pending, so the transition always passes through zero torque. */
	candidate = load_velocity_sign(velocity, enter_threshold);
	if (candidate == -current)
	{
		if (ls->dir_candidate != (float)candidate)
		{
			ls->dir_candidate = (float)candidate;
			ls->dir_confirm_ticks = 0u;
		}
		if (++ls->dir_confirm_ticks >= confirm_ticks)
		{
			ls->dir_state = (float)candidate;
			ls->dir_candidate = 0.0f;
			ls->dir_confirm_ticks = 0u;
			ls->release_ticks = 0u;
		}
		return;
	}

	ls->dir_candidate = 0.0f;
	ls->dir_confirm_ticks = 0u;
	if ((current > 0 && velocity < exit_threshold) ||
		(current < 0 && velocity > -exit_threshold))
	{
		if (++ls->release_ticks >= release_ticks)
		{
			ls->dir_state = 0.0f;
			ls->release_ticks = 0u;
		}
	}
	else
	{
		ls->release_ticks = 0u;
	}
}

/* 单值速率限制: 向 target 逼近, 每拍变化量 <= ramp*dt */
static float load_ramp_step(float current, float target, float ramp, float dt)
{
	float delta = target - current;
	float step = ramp * dt;

	if (delta > step)
		return current + step;
	if (delta < -step)
		return current - step;
	return target;
}

/* 被动恒转矩 0x60: T_out = ramp(-dir(ω)·t_set)
 * 两级防抖: 滞回方向判定 + 输出转矩速率限制。
 * 方向判定用滞回(进入阈值 dead / 退出阈值 dead/2)抑制 0↔±1 斩波;
 * 但机械震动会让速度反复越过换向阈值, 方向仍可能在 +1↔-1 间翻转,
 * 故输出级必须再加速率限制——给定转矩每拍只变一小步, 换向平滑过零,
 * 跳变的能量谱被斜坡滤除, 转矩建立约 t_set/ramp (0.1Nm→20ms)。 */
static void load_passive_torque_run(motor_ctrl_t *ctrl)
{
	motor_param_t *p = ctrl->param;
	motor_ref_t *ref = &ctrl->ref;
	motor_load_sim_t *ls = &ctrl->load_sim;
	float peak_t = p->motor_base.peak_torque;
	float dead = p->load_sim_param.load_sim_dead_zone_rad_s;
	float vel;
	float vel_abs;
	float torque_gain;
	float target;

	if (!isfinite(dead) || dead <= 0.0f)
		dead = 0.5f;

	/* Filter the measured speed before any sign decision. */
	vel = ctrl->fb.vel;
	if (!isfinite(vel))
		vel = 0.0f;
	if (!isfinite(ls->vel_filt))
		ls->vel_filt = vel;
	ls->vel_filt += load_filter_alpha(ctrl->dt) * (vel - ls->vel_filt);
	load_update_direction(ctrl, ls->vel_filt, dead);

	ls->elapsed_ticks++;

	/* Continuous low-speed boundary layer: reduce load before zero speed.
	 * During a pending reversal, force zero instead of applying the old sign. */
	vel_abs = fabsf(ls->vel_filt);
	torque_gain = motor_mode_clamp(
		(vel_abs - dead * 0.5f) / (dead * 0.5f), 0.0f, 1.0f);
	if (ls->dir_state == 0.0f ||
		(ls->dir_state > 0.0f && ls->vel_filt < 0.0f) ||
		(ls->dir_state < 0.0f && ls->vel_filt > 0.0f))
	{
		torque_gain = 0.0f;
	}
	target = motor_mode_clamp(-ls->dir_state * ls->t_set * torque_gain,
		-peak_t, peak_t);

	/* Torque ramp guarantees a zero crossing on release and reversal. */
	ls->torque_out = load_ramp_step(ls->torque_out, target,
		LOAD_SIM_TORQUE_RAMP_NM_S, ctrl->dt);

	ref->pos_profile = MOTOR_PID_PROFILE_POSITION;
	ref->vel_profile = MOTOR_PID_PROFILE_VELOCITY;
	ref->ctrl_type = REF_CTRL_TORQUE;
	ref->torque = ls->torque_out;
	ref->torque_ff = 0.0f;
}

void motor_mode_load_sim_run(motor_ctrl_t *ctrl)
{
	switch (ctrl->run_state)
	{
		case RUN_STATE_PASSIVE_TORQUE:
			load_passive_torque_run(ctrl);
			break;

		default:
			/* 0x61~0x67，当前不可达 */
			ctrl->ref.ctrl_type = REF_CTRL_IDLE;
			break;
	}
}
