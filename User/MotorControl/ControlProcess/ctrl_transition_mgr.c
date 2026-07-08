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
	/* 退出 RUN 或进入 FAULT/SAFETY：立即结束过渡 */
	(void)new_state;
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
	}
}
