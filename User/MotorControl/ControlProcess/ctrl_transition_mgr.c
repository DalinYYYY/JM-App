/* ctrl_transition_mgr.c */
#include "ctrl_transition_mgr.h"
#include "system_state.h" /* 完整定义 system_state_t（头文件仅前向声明） */
#include "motor_info.h"	 /* motor_info_t: softstart 持久化镜像 */
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
	transition_start(&mgr->trans, trans_count, cur_ref, mgr->smooth_cfg.shape);
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
		case RUN_STATE_PASSIVE_TORQUE:
			/* 被动恒转矩: 制动转矩归零, 电机随被测电机自由旋转 */
			sys->motor.load_sim.t_set = 0.0f;
			break;
		default:
			break;
	}
}

/* STOP 请求：仅纯参考闭环模式支持停机过渡，构造零目标并置挂起标志 */
bool transition_mgr_on_stop(transition_mgr_t *mgr, system_state_t *sys)
{
	/* 模式切换过渡中停止：判据用目标模式（过渡期间 run_state 仍为旧值，
	 * 而输出参考已按目标模式量纲混合，缓起途中 STOP 即此场景） */
	run_state_e rs = sys->motor.run_state;
	bool mode_switching = (mgr->trans.state == TRANSITION_IN_PROGRESS &&
						   rs != mgr->target_run_state);
	if (mode_switching)
		rs = mgr->target_run_state;

	/* 不支持停机过渡的模式（MIT/HOLD/直控）：直接切 READY */
	switch (rs)
	{
		case RUN_STATE_POSITION:
		case RUN_STATE_POSITION_VELOCITY:
		case RUN_STATE_POSITION_TORQUE:
		case RUN_STATE_VELOCITY:
		case RUN_STATE_TORQUE:
		case RUN_STATE_CURRENT:
		case RUN_STATE_PASSIVE_TORQUE: /* REF_CTRL_TORQUE 输出, 支持转矩渐减 */
			break;
		default:
			return false;
	}

	/* 模式切换过渡中：立即提交目标模式，保证 run_state 与输出参考量纲
	 * 一致（apply_stop_cmd 与停机渐变检测均依赖 run_state） */
	if (mode_switching)
		sys->motor.run_state = rs;

	/* 过渡状态复位 IDLE（含进行中的模式切换/渐变与刚完成的 COMPLETED）：
	 * 停机渐变从当前混合参考接管。若保持 COMPLETED，下一拍 ref_smooth
	 * 检测会先做 COMPLETED→IDLE 复位并跳过本拍，零目标阶跃漏检，
	 * 参考瞬降为零且停机当拍即完成 */
	mgr->trans.state = TRANSITION_IDLE;

	/* 构造零目标 cmd */
	apply_stop_cmd(sys);

	mgr->stop_pending = true;
	return true;
}

void transition_mgr_cancel_stop(transition_mgr_t *mgr)
{
	mgr->stop_pending = false;
}

/* motor_info → 运行时 smooth_cfg 全量应用（启动加载 / 0xA3 镜像写 / 0xE7 hook 共用）。
 * softstart_valid=0（老配置迁移或未配置）时跳过，保持编译期默认。 */
void transition_mgr_apply_softstart(transition_mgr_t *mgr, const motor_info_t *info)
{
	const AdvancedAlgoParam_t *ss = &info->blocks.advanced;

	if (mgr == NULL || info == NULL || ss->softstart_valid == 0u)
		return;

	ref_smooth_cfg_t *cfg = &mgr->smooth_cfg;
	cfg->enable = (ss->softstart_enable != 0u);
	cfg->shape = (ss->softstart_shape != 0u) ? TRANSITION_SHAPE_SCURVE : TRANSITION_SHAPE_LINEAR;
	cfg->smooth_duration = ss->softstart_duration;
	cfg->pos_rate = ss->softstart_pos_rate;
	cfg->vel_rate = ss->softstart_vel_rate;
	cfg->torque_rate = ss->softstart_torque_rate;
	cfg->current_rate = ss->softstart_current_rate;
	/* v1.12: thresh/fallback 已从 motor_info 精简,
	 * 运行时保持 ref_smooth_cfg_init_defaults 的编译期默认值 */
}

static float mgr_absf(float v)
{
	return (v < 0.0f) ? -v : v;
}

/**
 * @brief 渐变中目标再变化检测：主字段差异超触发阈值
 * @details 混合公式以启动时 old_ref 为锚（out = old×(1-r) + target×r），
 *          目标中途变化时输出瞬跳 Δtarget×ratio。检测到再变化后由
 *          blend_restart 以当前输出为起点重启渐变，恢复连续。
 *          阈值复用 ref_smooth 的触发阈值：小于阈值的目标微调不重启，
 *          跳变量小于阈值可接受。
 */
static bool blend_target_changed(const transition_t *trans,
								 const motor_ref_t *raw,
								 const ref_smooth_cfg_t *cfg)
{
	switch (raw->ctrl_type)
	{
		case REF_CTRL_POSITION:
			return mgr_absf(raw->pos - trans->target_ref.pos) > cfg->pos_thresh;
		case REF_CTRL_VELOCITY:
			return mgr_absf(raw->vel - trans->target_ref.vel) > cfg->vel_thresh;
		case REF_CTRL_TORQUE:
			return mgr_absf(raw->torque - trans->target_ref.torque) > cfg->torque_thresh;
		case REF_CTRL_CURRENT:
			return mgr_absf(raw->id - trans->target_ref.id) > cfg->current_thresh ||
				   mgr_absf(raw->iq - trans->target_ref.iq) > cfg->current_thresh;
		default:
			return false;
	}
}

/**
 * @brief 以 cur_out 为起点、朝 raw 目标重启渐变
 * @details 时长优先按速率模式计算（|Δ|/rate），回退 smooth_duration；
 *          两者都无效则强制完成（下拍透传目标）。
 *          重启后 elapsed=0，transition_update 首拍重新登记 target_ref。
 */
static void blend_restart(transition_mgr_t *mgr, const motor_ref_t *raw,
						  const motor_ref_t *cur_out)
{
	uint32_t dur = transition_calc_duration_by_rate(raw, cur_out,
													&mgr->smooth_cfg, mgr->dt);
	if (dur == 0)
		dur = mgr->smooth_cfg.smooth_duration;
	if (dur == 0)
	{
		transition_force_complete(&mgr->trans);
		return;
	}
	transition_start(&mgr->trans, dur, cur_out, mgr->smooth_cfg.shape);
}

/* 过渡管理器每拍处理：模式切换过渡/同模式渐变/STOP 停机过渡统一调度 */
void transition_mgr_step(transition_mgr_t *mgr, system_state_t *sys)
{
	if (mgr->trans.state == TRANSITION_IN_PROGRESS)
	{
		/* 模式切换过渡中：dispatch 目标模式生成 new_ref。
		 * dispatch 会覆盖 sys->motor.ref，先保存当前混合输出（重启起点） */
		motor_ref_t cur_out = sys->motor.ref;
		run_state_e old = sys->motor.run_state;
		sys->motor.run_state = mgr->target_run_state;
		motor_ctrl_dispatch(&sys->motor);
		motor_ref_t new_ref = sys->motor.ref;
		sys->motor.run_state = old;

		/* 首拍处理：跨量纲锚定 + 按速率模式重算过渡时长 */
		if (mgr->trans.elapsed == 0)
		{
			/* 跨量纲切换时，新量纲主字段在旧参考中为陈旧值（如 VEL→POS
			 * 时 old_ref.pos 停留在使能时刻，电机实际已转出很远）。
			 * 用反馈重新锚定，保证切换瞬间参考≈当前实际值，
			 * 避免位置环巨大初始误差引发猛烈动作 */
			if (mgr->trans.old_ref.ctrl_type != new_ref.ctrl_type)
			{
				switch (new_ref.ctrl_type)
				{
					case REF_CTRL_POSITION:
						mgr->trans.old_ref.pos = sys->motor.fb.pos;
						break;
					case REF_CTRL_VELOCITY:
						mgr->trans.old_ref.vel = sys->motor.fb.vel;
						break;
					case REF_CTRL_TORQUE:
						mgr->trans.old_ref.torque = sys->motor.fb.torque;
						break;
					case REF_CTRL_CURRENT:
						mgr->trans.old_ref.id = sys->motor.fb.id;
						mgr->trans.old_ref.iq = sys->motor.fb.iq;
						break;
					default:
						break;
				}
			}

			/* 按速率模式重算过渡时长：使能/切模式的目标阶跃也遵循
			 * rate 斜坡（如 vel_rate=20 → 0→50rad/s 使能即 2.5s 缓起）。
			 * rate 不适用（全<=0 或无差异）时保持固定 trans_count */
			uint32_t dur = transition_calc_duration_by_rate(&new_ref,
															 &mgr->trans.old_ref,
															 &mgr->smooth_cfg,
															 mgr->dt);
			if (dur > 0)
				mgr->trans.duration = dur;
		}
		else if (blend_target_changed(&mgr->trans, &new_ref, &mgr->smooth_cfg))
		{
			/* 过渡中目标再变化：以当前混合输出为起点重启 */
			blend_restart(mgr, &new_ref, &cur_out);
		}

		/* blend */
		motor_ref_t mixed_ref;
		bool done = transition_update(&mgr->trans, &new_ref, &mixed_ref, mgr->dt);
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
			/* 渐变中目标再变化：混合公式以启动时 old_ref 为锚，
			 * 目标平移会使输出瞬跳 Δtarget×ratio。以当前输出为起点
			 * 重启渐变，恢复连续（降速途中改目标即此场景）。
			 * elapsed>0 守卫：本拍 check 刚启动的渐变 target_ref 尚未
			 * 登记（仍是上一次渐变的陈旧值），跳过检测 */
			if (mgr->trans.elapsed > 0 &&
				blend_target_changed(&mgr->trans, &sys->motor.ref, &mgr->smooth_cfg))
				blend_restart(mgr, &sys->motor.ref, &prev_ref);

			motor_ref_t mixed_ref;
			transition_update(&mgr->trans, &sys->motor.ref, &mixed_ref, mgr->dt);
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
