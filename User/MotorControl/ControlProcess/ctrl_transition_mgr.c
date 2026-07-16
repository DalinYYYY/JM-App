/* ctrl_transition_mgr.c */
#include "ctrl_transition_mgr.h"
#include "system_state.h" /* 完整定义 system_state_t（头文件仅前向声明） */
#include <string.h>

void transition_mgr_init(transition_mgr_t *mgr, float dt)
{
	memset(mgr, 0, sizeof(*mgr));
	transition_init(&mgr->trans);
	ref_smooth_cfg_init_defaults(&mgr->smooth_cfg);
	mgr->target_run_state = RUN_STATE_IDLE;
	mgr->dt = (dt > 0.0f) ? dt : 1.0e-4f; /* 默认 100µs，供 rate 模式算 duration */
}

void transition_mgr_on_top_fsm_change(transition_mgr_t *mgr, top_fsm_e new_state)
{
	/* 退出 RUN 或进入 FAULT/SAFETY：立即结束过渡，中止停机延迟 */
	(void)new_state;
	mgr->stop_pending = false;
	transition_force_complete(&mgr->trans);
}

void transition_mgr_on_mode_switch(transition_mgr_t *mgr,
								   run_state_e new_state,
								   uint32_t trans_count,
								   const motor_ref_t *cur_ref)
{
	if (new_state >= RUN_STATE_MAX)
		return;
	mgr->target_run_state = new_state;
	transition_start(&mgr->trans, trans_count, cur_ref);
}

/**
 * @brief 构造 STOP 零目标 cmd：保持当前 run_state，目标渐变到零
 * @details POSITION 类停在当前 ref.pos（不漂移），其余模式目标清零。
 *          每拍调用：防止 stop_pending 期间协议层写入新目标。 */
static void apply_stop_cmd(system_state_t *sys)
{
	switch (sys->motor.run_state)
	{
		case RUN_STATE_POSITION:
		case RUN_STATE_POSITION_VELOCITY:
		case RUN_STATE_POSITION_TORQUE:
			/* POSITION 类：停在当前参考位置，前馈清零 */
			sys->motor.cmd.pos = sys->motor.ref.pos;
			sys->motor.cmd.vel = 0.0f;
			sys->motor.cmd.vel_ff = 0.0f;
			sys->motor.cmd.torque = 0.0f;
			sys->motor.cmd.torque_ff = 0.0f;
			break;
		case RUN_STATE_VELOCITY:
			sys->motor.cmd.vel = 0.0f;
			break;
		case RUN_STATE_TORQUE:
			sys->motor.cmd.torque = 0.0f;
			break;
		case RUN_STATE_CURRENT:
			sys->motor.cmd.id = 0.0f;
			sys->motor.cmd.iq = 0.0f;
			break;
		default:
			break;
	}
}

bool transition_mgr_on_stop(transition_mgr_t *mgr, system_state_t *sys)
{
	run_state_e rs = sys->motor.run_state;

	/* 不支持停机过渡的模式（MIT/HOLD/直控）：直接切 READY */
	switch (rs)
	{
		case RUN_STATE_POSITION:
		case RUN_STATE_POSITION_VELOCITY:
		case RUN_STATE_POSITION_TORQUE:
		case RUN_STATE_VELOCITY:
		case RUN_STATE_TORQUE:
		case RUN_STATE_CURRENT:
			break;
		default:
			return false;
	}

	/* 终止任何进行中的过渡（模式切换或渐变），让停机过渡接管 */
	if (mgr->trans.state == TRANSITION_IN_PROGRESS)
		transition_force_complete(&mgr->trans);

	/* 构造零目标 cmd */
	apply_stop_cmd(sys);

	mgr->stop_pending = true;
	return true;
}

void transition_mgr_step(transition_mgr_t *mgr, system_state_t *sys)
{
	if (mgr->trans.state == TRANSITION_IN_PROGRESS)
	{
		/* 模式切换过渡中：dispatch 目标模式生成 new_ref */
		run_state_e old = sys->motor.run_state;
		sys->motor.run_state = mgr->target_run_state;
		motor_ctrl_dispatch(&sys->motor);
		motor_ref_t new_ref = sys->motor.ref;
		sys->motor.run_state = old;

		/* blend */
		motor_ref_t mixed_ref;
		bool done = transition_update(&mgr->trans, &new_ref, &mixed_ref);
		sys->motor.ref = mixed_ref;

		if (done)
			sys->motor.run_state = mgr->target_run_state;
	}
	else
	{
		/* STOP 延迟过渡期间：每拍强制零目标，防止协议层写入 */
		if (mgr->stop_pending)
			apply_stop_cmd(sys);

		/* 无模式切换过渡：dispatch + 同模式渐变检测 */
		motor_ref_t prev_ref = sys->motor.ref;
		motor_ctrl_dispatch(&sys->motor);

		/* 同模式目标值渐变：突变超阈值时启动参考层 blend。
		 * 判据用 sys->motor.run_state（白名单排除 MIT/HOLD/直控）。
		 * dt 取自 mgr->dt（运行期可配），供 rate 模式算 duration。 */
		transition_ref_smooth_check(&mgr->trans, sys->motor.run_state,
									&sys->motor.ref, &prev_ref,
									&mgr->smooth_cfg, mgr->dt);

		/* 渐变进行中：对参考做线性混合，消除目标阶跃 */
		if (mgr->trans.state == TRANSITION_IN_PROGRESS)
		{
			motor_ref_t mixed_ref;
			transition_update(&mgr->trans, &sys->motor.ref, &mixed_ref);
			sys->motor.ref = mixed_ref;
		}

		/* STOP 延迟过渡完成检测：渐变完成(COMPLETED)或无需渐变(IDLE) */
		if (mgr->stop_pending &&
			(mgr->trans.state == TRANSITION_COMPLETED ||
			 mgr->trans.state == TRANSITION_IDLE))
		{
			mgr->stop_pending = false;
			sys->motor.ref.ctrl_type = REF_CTRL_IDLE;
			top_fsm_switch(sys, TOP_FSM_READY);
		}
	}
}
