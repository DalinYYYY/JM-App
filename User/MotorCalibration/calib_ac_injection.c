/**
 * @file calib_ac_injection.c
 * @brief 交流注入法 R/Ld 辨识实现（低阻电机专用）
 */
#include <math.h>
#include "arm_math.h" /* arm_cos_f32 / arm_sin_f32 */
#include "calib_ac_injection.h"
#include "calib_config.h"
#include "calib_config_runtime.h" /* 运行期派生参数 */
#include "calib_hw.h"
#include "dev_motor.h" /* struct dev_motor 完整定义，m->foc.clarke 等 */

#ifndef PI
#define PI 3.14159265358979f
#endif

void calib_ac_injection_init(calib_ac_injection_t *ctx, calib_hw_session_t *session)
{
	ctx->session = session;
	ctx->ud_dc = calib_cfg_ac_inject_dc_v();
	ctx->ud_ac = calib_cfg_ac_inject_amp_v();
	ctx->freq_hz = calib_cfg_ac_inject_freq_hz();
	ctx->tick = 0;
	ctx->warmup_ticks = calib_cfg_ac_inject_warmup_ticks();
	ctx->total_ticks = ctx->warmup_ticks + calib_cfg_ac_inject_total_ticks();
	ctx->sum_id_cos = 0.0f;
	ctx->sum_id_sin = 0.0f;
	ctx->id_dc_est = 0.0f;
	ctx->sample_cnt = 0;
}

void calib_ac_injection_poll(calib_ac_injection_t *ctx)
{
	struct dev_motor *m = ctx->session->motor;
	float t = (float)ctx->tick / CALIB_TICKS_PER_SEC;
	float omega = 2.0f * PI * ctx->freq_hz;
	float phase = omega * t;
	float cos_wt = arm_cos_f32(phase);
	float sin_wt = arm_sin_f32(phase);

	/* 施加交流电压（d 轴 DC + AC）*/
	calib_hw_apply_ac_injection(ctx->session, ctx->ud_dc, ctx->ud_ac,
	                             ctx->freq_hz, ctx->tick, 0.0f);

	/* 预热期：仅施加电压，不采样。预热最后一拍记录 DC 电流估计值 */
	if (ctx->tick < ctx->warmup_ticks)
	{
		ctx->tick++;
		return;
	}

	/* 预热刚结束时记录 DC 电流估计（用于相敏检测去除 DC 偏置）*/
	if (ctx->tick == ctx->warmup_ticks)
	{
		m->foc.clarke(&m->foc);
		m->foc.park(&m->foc);
		ctx->id_dc_est = m->foc.i_dq.d;
	}

	/* 采样 id（刷新 clarke/park）*/
	m->foc.clarke(&m->foc);
	m->foc.park(&m->foc);
	float id = m->foc.i_dq.d;

	/* 相敏检测累加（去除 DC 偏置后的 AC 分量）*/
	float id_ac = id - ctx->id_dc_est;
	ctx->sum_id_cos += id_ac * cos_wt;
	ctx->sum_id_sin += id_ac * sin_wt;
	ctx->sample_cnt++;
	ctx->tick++;
}

void calib_ac_injection_result(const calib_ac_injection_t *ctx, float *r, float *ld)
{
	if (ctx->sample_cnt == 0)
	{
		*r = 0.0f;
		*ld = 0.0f;
		return;
	}
	float n = (float)ctx->sample_cnt;
	/* I_ac 的 cos/sin 分量（去 DC 后的交流幅值的正交分解）*/
	float i_cos = 2.0f * ctx->sum_id_cos / n;
	float i_sin = 2.0f * ctx->sum_id_sin / n;
	/* 电流交流幅值 */
	float i_ac = sqrtf(i_cos * i_cos + i_sin * i_sin);
	if (i_ac < 1e-6f)
	{
		*r = 0.0f;
		*ld = 0.0f;
		return;
	}
	/* φ = atan2(i_sin, i_cos)，电流滞后电压的角度
	 * 纯阻性: φ=0, i_sin=0; 纯感性: φ=90°, i_cos=0 */
	float phi = atan2f(i_sin, i_cos);
	float omega = 2.0f * PI * ctx->freq_hz;
	/* R = U_ac·cos(φ) / I_ac */
	*r = (ctx->ud_ac * cosf(phi)) / i_ac;
	/* Ld = U_ac·sin(φ) / (ω·I_ac) */
	*ld = (ctx->ud_ac * sinf(phi)) / (omega * i_ac);
}
