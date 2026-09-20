/**
 * @file multiturn_counter.c
 * @brief 电机绝对多圈计数模块实现（自包含）
 *
 * @author Dalin
 * @version 1.0
 * @date 2026-06-15
 */

#include "multiturn_counter.h"
#include <math.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/* 本地常量与工具                                                       */
/* ------------------------------------------------------------------ */
#ifndef MT_PI
#define MT_PI 3.14159265358979323846f
#endif
#define MT_2PI (2.0f * MT_PI)
#define MT_DEG2RAD (MT_PI / 180.0f)

/* 上电后丢弃的解算拍数默认值（等待编码器稳定） */
#define MT_DEFAULT_SETTLE_TICKS 1000u

/* 把角度归一化到 [0, 360) */
static float normalize_angle(float angle)
{
	int n = (int)(angle / 360.0f);
	float remainder = angle - n * 360.0f;
	return remainder >= 0.0f ? remainder : (remainder + 360.0f);
}

/* 把弧度差归一化到 (-pi, pi]，用于处理过零跳变 */
static float wrap_rad_pi(float delta)
{
	while (delta > MT_PI)
		delta -= MT_2PI;
	while (delta <= -MT_PI)
		delta += MT_2PI;
	return delta;
}

/* 圆周距离：两个单圈位置 [0,1) 的最近角差，范围 [0, 0.5] */
static float circular_dist(float a, float b)
{
	float d = a - b;
	d -= floorf(d); /* mod 1 -> [0,1) */
	return (d <= 0.5f) ? d : (1.0f - d);
}

/* ------------------------------------------------------------------ */
/* 软件累圈（掉电丢失）                                                 */
/* ------------------------------------------------------------------ */
static float multiturn_soft(multiturn_t *pobj, float mechanical_angle)
{
	uint16_t settle = (pobj->cfg.settle_ticks != 0u)
						  ? pobj->cfg.settle_ticks
						  : MT_DEFAULT_SETTLE_TICKS;

	/* 编码器上电需要稳定一定时间 */
	if (pobj->pos_settle_ticks < settle)
	{
		pobj->pos_settle_ticks++;
		pobj->last_single_rad = mechanical_angle * MT_DEG2RAD;
		return 0.0f;
	}

	/* 当前单圈弧度（含设备补偿），归一化到 [0, 2pi) */
	float comp = (pobj->cfg.device_compensation_callback != NULL)
					 ? pobj->cfg.device_compensation_callback()
					 : 0.0f;
	float curr_rad = mechanical_angle * MT_DEG2RAD - comp;
	curr_rad = curr_rad >= MT_2PI ? (curr_rad - MT_2PI)
								  : (curr_rad < 0.0f ? (curr_rad + MT_2PI) : curr_rad);

	/* 过零跳变修正后累加 */
	float delta = wrap_rad_pi(curr_rad - pobj->last_single_rad);
	pobj->position += delta;
	pobj->turns = (int32_t)(pobj->position / MT_2PI);
	pobj->last_single_rad = curr_rad;

	pobj->multiturn_position = pobj->position;
	return pobj->position;
}

/* ------------------------------------------------------------------ */
/* 齿轮游标(Nonius)绝对多圈解算（predict-match 法）                     */
/* ------------------------------------------------------------------ */
/**
 * @brief 齿轮游标绝对多圈解算
 *
 * 物理模型：主齿轮（齿数 Nm）与副齿轮（齿数 Ns）啮合，主齿轮转 1 圈，副齿轮转
 * Nm/Ns 圈。已知主齿轮绝对圈数 P 时，副齿轮单圈位置可预测为 frac(P * Nm/Ns)。
 *
 * 重建：主齿轮单圈位置 θm 已测得（[0,1)）。遍历候选整圈数 T，预测各副齿轮单圈
 * 位置 frac((T+θm)*Nm/Ns_i)，与实测副齿轮位置做圆周距离求和，取误差最小的 T。
 * 单游标(双齿轮)量程 = Ns1 圈；双游标(三齿轮)量程 = Ns1*Ns2 圈（齿数两两互质
 * 时），且第二个游标提供交叉校验抗噪。
 *
 * 该法相比解析相位差更鲁棒：无需齿数模逆，误差以圆周距离显式度量，可用于异常判定。
 *
 * @param pobj 多圈对象
 * @param gear_angles 各齿轮机械角度 (deg)，[0]=主齿轮
 * @param count 齿轮数量
 */
static float multiturn_nonius(multiturn_t *pobj, const float *gear_angles, uint8_t count)
{
	const multiturn_config_t *cfg = &pobj->cfg;
	uint16_t Nm = cfg->gear_teeth[0];

	/* 主齿轮单圈位置 θm [0,1) */
	int8_t dir_m = (cfg->gear_dir[0] != 0) ? cfg->gear_dir[0] : 1;
	float theta_m = normalize_angle((float)dir_m * gear_angles[0]) / 360.0f;

	/* 各副齿轮单圈位置与齿数（最多 2 个） */
	uint8_t nsec = (count >= MULTITURN_GEAR_MAX) ? (MULTITURN_GEAR_MAX - 1u)
												 : (uint8_t)(count - 1u);
	if (cfg->mode == MULTITURN_MODE_NONIUS_2GEAR && nsec > 1u)
		nsec = 1u;

	float theta_s[MULTITURN_GEAR_MAX - 1u];
	uint16_t teeth_s[MULTITURN_GEAR_MAX - 1u];
	int32_t period = 1;
	for (uint8_t i = 0; i < nsec; i++)
	{
		int8_t dir = (cfg->gear_dir[i + 1u] != 0) ? cfg->gear_dir[i + 1u] : 1;
		theta_s[i] = normalize_angle((float)dir * gear_angles[i + 1u]) / 360.0f;
		teeth_s[i] = cfg->gear_teeth[i + 1u];
		period *= (int32_t)teeth_s[i]; /* 量程 = 各副齿轮齿数之积 */
	}

	/* predict-match：扫描候选整圈数，取副齿轮预测误差最小者 */
	int32_t best_turns = 0;
	float best_err = 1.0e30f;
	for (int32_t T = 0; T < period; T++)
	{
		float p_abs = (float)T + theta_m; /* 主齿轮绝对圈数 */
		float err = 0.0f;
		for (uint8_t i = 0; i < nsec; i++)
		{
			float pred = p_abs * (float)Nm / (float)teeth_s[i];
			pred -= floorf(pred); /* 预测副齿轮单圈位置 */
			err += circular_dist(pred, theta_s[i]);
		}
		if (err < best_err)
		{
			best_err = err;
			best_turns = T;
		}
	}

	/* 绝对多圈位置 = 整圈数 + 主齿轮单圈位置，转成弧度 */
	pobj->turns = best_turns;
	pobj->multiturn_position = ((float)best_turns + theta_m) * MT_2PI;
	pobj->position = pobj->multiturn_position;
	return pobj->multiturn_position;
}

/* ------------------------------------------------------------------ */
/* 更新分发                                                            */
/* ------------------------------------------------------------------ */
static float multiturn_update(multiturn_t *pobj, const float *gear_angles, uint8_t count)
{
	if (pobj == NULL || gear_angles == NULL || count < 1u)
		return 0.0f;

	switch (pobj->cfg.mode)
	{
		case MULTITURN_MODE_NONIUS_2GEAR:
			if (count >= 2u)
				return multiturn_nonius(pobj, gear_angles, count);
			return multiturn_soft(pobj, gear_angles[0]); /* 副齿轮缺失则回退软件累圈 */

		case MULTITURN_MODE_NONIUS_3GEAR:
			/* 三齿轮模式：齐全(>=3)用双游标，缺一路(==2)自动退化为单游标 */
			if (count >= 2u)
				return multiturn_nonius(pobj, gear_angles, count);
			return multiturn_soft(pobj, gear_angles[0]);

		case MULTITURN_MODE_SOFT:
		case MULTITURN_MODE_NONE:
		default:
			return multiturn_soft(pobj, gear_angles[0]);
	}
}

static float multiturn_update_single(multiturn_t *pobj, float mechanical_angle)
{
	return multiturn_update(pobj, &mechanical_angle, 1u);
}

static float multiturn_get_position(multiturn_t *pobj)
{
	return pobj->position;
}

static int32_t multiturn_get_turns(multiturn_t *pobj)
{
	return pobj->turns;
}

static void multiturn_reset_position(multiturn_t *pobj)
{
	/* 保留 last_single_rad 与 settle 状态: 下一拍增量连续, 不引入过零跳变 */
	pobj->position = 0.0f;
	pobj->turns = 0;
	pobj->multiturn_position = 0.0f;
}

/* ------------------------------------------------------------------ */
/* 初始化                                                              */
/* ------------------------------------------------------------------ */
void multiturn_init(multiturn_t *pobj, const multiturn_config_t *cfg)
{
	if (pobj == NULL || cfg == NULL)
		return;

	memset(pobj, 0, sizeof(multiturn_t));
	pobj->cfg = *cfg;

	pobj->update = multiturn_update;
	pobj->update_single = multiturn_update_single;
	pobj->get_position = multiturn_get_position;
	pobj->get_turns = multiturn_get_turns;
	pobj->reset_position = multiturn_reset_position;
}
