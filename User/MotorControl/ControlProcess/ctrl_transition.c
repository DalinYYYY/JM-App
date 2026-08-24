#include "ctrl_transition.h"
#include <string.h>

void transition_init(transition_t *trans)
{
	memset(trans, 0, sizeof(transition_t));
	trans->state = TRANSITION_IDLE;
}

void transition_start(transition_t *trans, uint32_t duration, const motor_ref_t *old_ref)
{
	trans->state = TRANSITION_IN_PROGRESS;
	trans->elapsed = 0;
	trans->duration = duration;
	trans->ratio = 0.0f;
	trans->old_ref = *old_ref;
}

/**
 * @brief 对同量纲的标量参考做线性混合
 */
static float blend(float old_v, float new_v, float ratio)
{
	return old_v * (1.0f - ratio) + new_v * ratio;
}

/* 状态机参考渐变：按累计调用次数推进线性混合，量纲变化时直接切换 */
bool transition_update(transition_t *trans, const motor_ref_t *new_ref, motor_ref_t *out_ref)
{
	if (trans->state != TRANSITION_IN_PROGRESS)
	{
		*out_ref = *new_ref;
		return true;
	}

	// 以调用次数计时：每次更新自增一次
	trans->elapsed++;

	if (trans->duration == 0 || trans->elapsed >= trans->duration)
	{
		trans->state = TRANSITION_COMPLETED;
		trans->ratio = 1.0f;
		*out_ref = *new_ref;
		return true;
	}

	// 量纲不同（入环层级变化）：不混合，直接采用新参考，
	// 无扰切换交由下游三环检测 ctrl_type 变化后预装载积分实现
	if (trans->old_ref.ctrl_type != new_ref->ctrl_type)
	{
		*out_ref = *new_ref;
		return false;
	}

	// 同量纲：对目标值做线性混合，保证参考连续
	trans->ratio = (float)trans->elapsed / trans->duration;

	*out_ref = *new_ref;
	out_ref->pos = blend(trans->old_ref.pos, new_ref->pos, trans->ratio);
	out_ref->vel = blend(trans->old_ref.vel, new_ref->vel, trans->ratio);
	out_ref->torque = blend(trans->old_ref.torque, new_ref->torque, trans->ratio);
	out_ref->id = blend(trans->old_ref.id, new_ref->id, trans->ratio);
	out_ref->iq = blend(trans->old_ref.iq, new_ref->iq, trans->ratio);
	out_ref->ud = blend(trans->old_ref.ud, new_ref->ud, trans->ratio);
	out_ref->voltage = blend(trans->old_ref.voltage, new_ref->voltage, trans->ratio);
	out_ref->duty = blend(trans->old_ref.duty, new_ref->duty, trans->ratio);

	/* PID profile 保留旧值直到过渡完成，避免增益突变+中间参考值导致力矩跳变 */
	out_ref->pos_profile = trans->old_ref.pos_profile;
	out_ref->vel_profile = trans->old_ref.vel_profile;

	return false;
}

void transition_force_complete(transition_t *trans)
{
	trans->state = TRANSITION_COMPLETED;
	trans->ratio = 1.0f;
}

/* ===== 同模式目标值渐变 ===== */

void ref_smooth_cfg_init_defaults(ref_smooth_cfg_t *cfg)
{
	cfg->enable = true;
	cfg->smooth_duration = 500;
	cfg->pos_thresh = 0.1f;
	cfg->vel_thresh = 1.0f;
	cfg->torque_thresh = 0.1f;
	cfg->current_thresh = 0.5f;
	cfg->voltage_thresh = 1.0f;
	cfg->duty_thresh = 0.1f;
	/* 速率模式默认启用（>0 即生效），覆盖 smooth_duration */
	cfg->pos_rate = 100.0f;     /* 50 rad/s：5 rad 突变 → 0.1s 过渡 */
	cfg->vel_rate = 500.0f;     /* 500 rad/s²：50 rad/s 突变 → 0.1s 过渡 */
	cfg->torque_rate = 20.0f;   /* 20 N·m/s */
	cfg->current_rate = 100.0f; /* 100 A/s */
}

static float ref_smooth_absf(float v)
{
	return (v < 0.0f) ? -v : v;
}

/**
 * @brief 按 rate 模式计算过渡时长(调用次数)
 * @details duration_i = ceil(|delta_i| / (rate_i × dt))，取各字段最大值。
 *          rate_i <= 0 表示该字段不参与速率计算（交由 smooth_duration 兜底）。
 *          所有 rate 都 <= 0 时返回 0，由调用方回退到 smooth_duration。
 */
static uint32_t ref_smooth_calc_duration_by_rate(const motor_ref_t *raw,
                                                 const motor_ref_t *prev,
                                                 const ref_smooth_cfg_t *cfg,
                                                 float dt)
{
	if (dt <= 0.0f)
		return 0;

	float max_needed = 0.0f;

	if (cfg->pos_rate > 0.0f)
	{
		float d = ref_smooth_absf(raw->pos - prev->pos) / (cfg->pos_rate * dt);
		if (d > max_needed)
			max_needed = d;
	}
	if (cfg->vel_rate > 0.0f)
	{
		float d = ref_smooth_absf(raw->vel - prev->vel) / (cfg->vel_rate * dt);
		if (d > max_needed)
			max_needed = d;
	}
	if (cfg->torque_rate > 0.0f)
	{
		float d = ref_smooth_absf(raw->torque - prev->torque) / (cfg->torque_rate * dt);
		if (d > max_needed)
			max_needed = d;
	}
	if (cfg->current_rate > 0.0f)
	{
		float d_id = ref_smooth_absf(raw->id - prev->id) / (cfg->current_rate * dt);
		float d_iq = ref_smooth_absf(raw->iq - prev->iq) / (cfg->current_rate * dt);
		float d = (d_id > d_iq) ? d_id : d_iq;
		if (d > max_needed)
			max_needed = d;
	}

	if (max_needed <= 0.0f)
		return 0;

	/* ceil，至少 1 */
	uint32_t dur = (uint32_t)(max_needed + 0.999f);
	return (dur < 1) ? 1 : dur;
}

/**
 * @brief run_state 白名单：仅纯参考型闭环模式启用同模式渐变
 * @details 排除 MIT（反馈型，ctrl_type 与 TORQUE 相同无法用 ctrl_type 区分）、
 *          HOLD（目标跟随 fb.pos）、OPEN_LOOP/DUTY/VOLTAGE（直控）。
 */
static bool ref_smooth_run_state_enabled(run_state_e s)
{
	switch (s)
	{
		case RUN_STATE_POSITION:
		case RUN_STATE_POSITION_VELOCITY:
		case RUN_STATE_POSITION_TORQUE:
		case RUN_STATE_VELOCITY:
		case RUN_STATE_TORQUE:
		case RUN_STATE_CURRENT:
		case RUN_STATE_PASSIVE_TORQUE: /* t_set 热更新经 torque blend 平滑 */
			return true;
		default:
			return false;
	}
}

/* 同模式目标值渐变检测：单拍突变超阈值则启动参考渐变过渡 */
bool transition_ref_smooth_check(transition_t *trans,
                                 run_state_e run_state,
                                 const motor_ref_t *raw_ref,
                                 const motor_ref_t *prev_ref,
                                 const ref_smooth_cfg_t *cfg,
                                 float dt)
{
	/* 总开关关闭：完全跳过 */
	if (!cfg->enable)
		return false;

	/* 过渡进行中：不重启，让现有渐变朝最新 raw_ref 继续混合 */
	if (trans->state == TRANSITION_IN_PROGRESS)
		return false;

	/* 过渡刚完成：复位为 IDLE，本拍不检测。
	 * 关键：模式切换过渡与同模式渐变共用同一 trans，若不在此复位，
	 * 模式切换完成那拍（state=COMPLETED）会继续往下走检测，可能误触发渐变。 */
	if (trans->state == TRANSITION_COMPLETED)
	{
		trans->state = TRANSITION_IDLE;
		return false;
	}

	/* run_state 白名单：排除 MIT/HOLD/直控模式 */
	if (!ref_smooth_run_state_enabled(run_state))
		return false;

	/* 量纲变化交给模式切换过渡处理 */
	if (raw_ref->ctrl_type != prev_ref->ctrl_type)
		return false;

	bool exceed = false;
	switch (raw_ref->ctrl_type)
	{
		case REF_CTRL_POSITION:
			if (ref_smooth_absf(raw_ref->pos - prev_ref->pos) > cfg->pos_thresh)
				exceed = true;
			break;
		case REF_CTRL_VELOCITY:
			if (ref_smooth_absf(raw_ref->vel - prev_ref->vel) > cfg->vel_thresh)
				exceed = true;
			break;
		case REF_CTRL_TORQUE:
			if (ref_smooth_absf(raw_ref->torque - prev_ref->torque) > cfg->torque_thresh)
				exceed = true;
			break;
		case REF_CTRL_CURRENT:
			if (ref_smooth_absf(raw_ref->id - prev_ref->id) > cfg->current_thresh || ref_smooth_absf(raw_ref->iq - prev_ref->iq) > cfg->current_thresh)
				exceed = true;
			break;
		default:
			break;
	}

	if (!exceed)
		return false;

	/* 计算过渡时长：优先速率模式，回退固定时长 */
	uint32_t duration = ref_smooth_calc_duration_by_rate(raw_ref, prev_ref, cfg, dt);
	if (duration == 0)
		duration = cfg->smooth_duration;
	if (duration == 0)
		return false; /* 无效配置，不启动 */

	/* 以上一拍实际输出为起点启动渐变 */
	transition_start(trans, duration, prev_ref);
	return true;
}
