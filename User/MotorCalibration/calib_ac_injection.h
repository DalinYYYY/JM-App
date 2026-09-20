/**
 * @file calib_ac_injection.h
 * @brief 交流注入法 R/Ld 辨识（低阻电机专用）
 * @note 在 d 轴施加 ud = U_dc + U_ac·sin(2π·f·t)，通过相敏检测分离 R 和 Ld。
 *       适用于 R < 0.5Ω 的低阻电机，DC 两点差分法失效的场景。
 *
 * @par 算法原理
 *   ud = R·id + Ld·(did/dt)
 *   稳态时 id = I_dc + I_ac·sin(ωt - φ)，φ = atan(ω·Ld / R)
 *   相敏相关（对 sin/cos 直接分解，注入电压参考为 sin）：
 *     i_sin = 2·<id·sin(ωt)> = I_ac·cosφ   → 阻性分量
 *     i_cos = 2·<id·cos(ωt)> = -I_ac·sinφ  → 感性分量(取反为正)
 *   R  = U_ac·i_sin / I_ac²
 *   Ld = U_ac·(-i_cos) / (ω·I_ac²)
 *   注入频率取 ωL = R（φ=45°）使感性分量信噪比最优（见 calib_config_runtime.c）。
 *   相关参考相位补偿采样链延迟（控制 1 拍 + LPF 群延迟，见
 *   CALIB_AC_DELAY_COMPENSATION_S），消除 R 偏低 / Ld 偏高的系统偏差。
 */
#ifndef __CALIB_AC_INJECTION_H__
#define __CALIB_AC_INJECTION_H__

#include <stdint.h>
#include "calib_types.h" /* 引入 motor_param_t 等基础类型 */
#include "calib_hw.h"    /* calib_hw_session_t */

typedef struct
{
	calib_hw_session_t *session; /* 硬件会话指针 */
	float ud_dc;                 /* 直流偏置电压(V) */
	float ud_ac;                 /* 交流幅值(V) */
	float freq_hz;               /* 交流频率(Hz) */
	uint32_t tick;               /* 当前 tick 计数 */
	uint32_t warmup_ticks;       /* 预热 tick 数 */
	uint32_t total_ticks;        /* 总 tick 数（含预热）*/
	/* 相敏检测累加器 */
	float sum_id_cos;            /* Σ id·cos(ωt) */
	float sum_id_sin;            /* Σ id·sin(ωt) */
	float id_dc_est;             /* DC 电流估计（预热期末采样）*/
	uint32_t sample_cnt;         /* 采样计数 */
} calib_ac_injection_t;

/* 初始化交流注入会话
 * @param ctx 上下文
 * @param session 硬件会话（须已 calib_hw_enter）*/
void calib_ac_injection_init(calib_ac_injection_t *ctx, calib_hw_session_t *session);

/* 每 tick 调用：施加交流电压 + 累加相敏检测
 * @note 预热期内仅施加电压不采样 */
void calib_ac_injection_poll(calib_ac_injection_t *ctx);

/* 计算结果：从相敏检测累加器提取 R 和 Ld
 * @param ctx 上下文
 * @param r 输出：相电阻(Ω)
 * @param ld 输出：d轴电感(H) */
void calib_ac_injection_result(const calib_ac_injection_t *ctx, float *r, float *ld);

#endif /* __CALIB_AC_INJECTION_H__ */
