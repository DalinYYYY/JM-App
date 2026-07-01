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
