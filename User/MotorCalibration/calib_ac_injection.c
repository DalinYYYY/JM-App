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

/* 采样链总延迟(秒): 三数据点联合反解实际延迟 ≈1.5~1.9 拍(152Hz 下 R/Ld
 * 联立欠补角反推), 取 1.4 拍。构成: 1 拍 PWM 更新(CCR 写入→下周期输出)
 * + 0.11 拍 abc 相电流 LPF(α=0.9) 群延迟 + ~0.3 拍 ADC 注入组触发
 * (TIM1_CC4) 到 JEOC 中断读 JDR 的间隔。
 * 未补偿时电流相位滞后 Δ=ω·t_d, R 偏低 tanφ·Δ 且 Ld 偏高——实测
 * 0.071(无补偿)→0.080(真值)即此效应; Ld 偏差还叠加工作点漂移(饱和
 * 电机 L(I,T) 本身是变量)与 Flash 污染链(见 calib_config_runtime.c
 * 注入频率派生), 修正 Flash 后以多次中值评估残余。*/
#define CALIB_AC_DELAY_COMPENSATION_S (1.4f / CALIB_TICKS_PER_SEC)

void calib_ac_injection_poll(calib_ac_injection_t *ctx)
{
	struct dev_motor *m = ctx->session->motor;
	float t = (float)ctx->tick / CALIB_TICKS_PER_SEC;
	float omega = 2.0f * PI * ctx->freq_hz;
	/* 参考相位回退采样链延迟: 用 sin(ω(t-t_d)) 对齐实际采样到的电流,
	 * 消除延迟引起的 R 偏低 / Ld 偏高 */
	float phase = omega * (t - CALIB_AC_DELAY_COMPENSATION_S);
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
	/* 相敏分解：注入电压基波为 U_ac·sin(ωt)，电流滞后 φ=atan(ωL/R)。
	 * 对 sin/cos 的相关直接得到电流的两个正交分量：
	 *   i_sin = 2·<i·sin(ωt)> = I·cosφ  → 与电压同相的阻性分量
	 *   i_cos = 2·<i·cos(ωt)> = -I·sinφ → 滞后 90° 的感性分量(取反为正)
	 * 阻抗恢复：R = U·i_res/I²，Ld = U·i_rea/(ω·I²)。
	 * 注: 旧实现按 atan2(i_sin, i_cos) 取相角再 cos/sin 分解，相位参考
	 *     含 90° 旋转，输出实为 R↔ωLd 互换（低阻电机两者差数倍，
	 *     且随 Flash 值被污染后级联越界）。*/
	float i_cos = 2.0f * ctx->sum_id_cos / n;
	float i_sin = 2.0f * ctx->sum_id_sin / n;
	float i2 = i_cos * i_cos + i_sin * i_sin;
	if (i2 < 1e-12f)
	{
		*r = 0.0f;
		*ld = 0.0f;
		return;
	}
	float i_res = i_sin;  /* 阻性分量 */
	float i_rea = -i_cos; /* 感性分量(电流滞后为正) */
	float omega = 2.0f * PI * ctx->freq_hz;
	*r = (ctx->ud_ac * i_res) / i2;
	*ld = (ctx->ud_ac * i_rea) / (omega * i2);
}
