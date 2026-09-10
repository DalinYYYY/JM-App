/**
 * @file        fault_detect_fast.c
 * @brief       快速故障检测(10kHz ISR): 电气类/编码器类
 *
 * @author      Dalin (dalinyy@163.com)
 * @version     1.0
 * @date        2026-09-03
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @par 修改日志:
 * | 日期       | 版本 | 作者  | 修改内容   |
 * |------------|------|-------|------------|
 * | 2026-09-03 | 1.0  | Dalin | 初始创建   |
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 *
 * @details 数据源: sys->motor.fb(id/iq/vel/bus_voltage/gate_driver_fault/
 *          enc_health/enc_err_cnt/mech_angle_deg, 由 motor_loop 每拍刷新)。
 *          检测项启用由 fault_manager.h 的 FAULT_DET_EN_* 编译期开关控制。
 *          原 system_state.c state_active_faults() 的 6 项检测迁移至此并
 *          保持判据/消抖/启动窗口语义不变(烧板事故教训: 判据时序勿动)。
 *          单次执行预算 < 3µs(全部标量比较, 无函数调用开销大的运算)。
 */

#include "fault_manager.h"
#include "system_state.h"
#include <math.h>
#include <string.h>

/* 启动保护窗口: 编码器冷启动可能先返回无效角度, 窗口内不武装超速/nFAULT */
#define FDET_SPEED_GUARD_SECONDS (0.020f)
/* nFAULT 确认时间: DRV8350 VDS_OCP 触发后 8ms 自动重试, 须覆盖 retry 窗口 */
#define FDET_GATE_CONFIRM_SECONDS (0.010f)
/* VBUS 采样无效判定: 电压 < 0.5V 视为采样通道失效(非真实欠压) */
#define FDET_VBUS_DEAD_V (0.5f)
/* VBUS ä¸çµæ­¦è£çªå£: fb->bus_voltage æ°æ®é¾è·¯å°±ç»ªåæä¸ºåå¼ 0, å»¶åæ­¦è£é²è¯¯éå­ */
#define FDET_VBUS_GUARD_SECONDS (1.000f)

/* cfg 阈值引用判定: 全部引用 cfg 的检测项禁用时, 避免 cfg 声明未引用警告 */
#if (FAULT_DET_EN_VBUS_SAMPLE || FAULT_DET_EN_VBUS_MID || FAULT_DET_EN_ENC_HEALTH || \
     FAULT_DET_EN_ENC_JUMP || FAULT_DET_EN_FOLLOW_ERR || FAULT_DET_EN_SOFT_LIMIT)
#define FDET_FAST_CFG_USED 1
#else
#define FDET_FAST_CFG_USED 0
#endif

static float fdet_absf(float v)
{
	return (v < 0.0f) ? -v : v;
}

static int fdet_is_finite(float v)
{
	uint32_t bits;
	memcpy(&bits, &v, sizeof(bits));
	return (bits & 0x7F800000u) != 0x7F800000u;
}

/* 角度差取 [0,180): 跳变检测用最短弧 */
#if FAULT_DET_EN_ENC_JUMP
static float fdet_deg_delta(float a, float b)
{
	float d = a - b;
	while (d > 180.0f)
		d -= 360.0f;
	while (d < -180.0f)
		d += 360.0f;
	return fdet_absf(d);
}
#endif /* FAULT_DET_EN_ENC_JUMP */

void fault_detect_fast(fault_mgr_t *fm)
{
	system_state_t *sys = (system_state_t *)fm->sys;
	const motor_fb_t *fb;
	const protection_param_t *pp;
	uint32_t enable;

	if (sys == NULL || sys->motor.param == NULL)
		return;
	fb = &sys->motor.fb;
	pp = &sys->motor.param->protection_param;
	enable = pp->protect_enable; /* 保护总使能(0=全部禁用, 非0=启用) */
#if FDET_FAST_CFG_USED
	const fault_cfg_t *cfg = &fm->cfg;
#endif

	/* 启动保护窗口倒计时(每拍) */
	if (fm->speed_guard_cycles > 0u)
		fm->speed_guard_cycles--;
	if (fm->vbus_guard_cycles > 0u)
		fm->vbus_guard_cycles--;

	/* ---- 0x810A 算法发散(NaN/Inf): 全量纲无条件检测 ---- */
#if FAULT_DET_EN_ALGO_DIVERGE
	if (!fdet_is_finite(fb->id) || !fdet_is_finite(fb->iq) || !fdet_is_finite(fb->vel) || !fdet_is_finite(fb->bus_voltage))
		fault_mgr_internal_set(FAULT_ALGO_DIVERGE, 0.0f);
	else
		fault_mgr_internal_clear(FAULT_ALGO_DIVERGE);
#endif /* FAULT_DET_EN_ALGO_DIVERGE */

	/* ---- 0x3102 峰值过流: 矢量幅值平方比较(免开方), 5ms消抖 ---- */
	// 同时检查总使能和新级别掩码
	if (enable != 0u &&
	    fault_mgr_is_enabled(FAULT_I_OVER_PEAK) &&
	    fdet_is_finite(fb->id) && fdet_is_finite(fb->iq) &&
	    (fb->id * fb->id + fb->iq * fb->iq) > pp->protect_over_current * pp->protect_over_current)
	{
		if (fm->over_current_cycles < fm->over_current_confirm_cyc)
			fm->over_current_cycles++;
	}
	else
	{
		fm->over_current_cycles = 0u;
	}
	if (fm->over_current_cycles >= fm->over_current_confirm_cyc)
		fault_mgr_internal_set(FAULT_I_OVER_PEAK, fdet_absf(fb->iq));
	else
		fault_mgr_internal_clear(FAULT_I_OVER_PEAK);

	/* ---- 0x2105 VBUS 采样异常 / 0x2102 过压 / 0x2103 欠压 / 0x2201/02 中度 ---- */
	if (fdet_is_finite(fb->bus_voltage))
	{
		if (fb->bus_voltage < FDET_VBUS_DEAD_V)
		{
#if FAULT_DET_EN_VBUS_SAMPLE
			/* 采样疑似失效: ms 级消抖(过压/欠压阈值远高于 0.5V,
			 * 消抖期间不会误触发欠压——欠压阈值 15V 量级)。
			 * 上电武装窗口内不累计: fb->bus_voltage 就绪前恒为 0,
			 * 直接判定必误锁存(故障级不自动清除) */
			if (fm->vbus_guard_cycles == 0u && fm->vbus_invalid_cycles < 0xFFFFFFFFu)
				fm->vbus_invalid_cycles++;
			/* 阈值计算: cfg->vbus_invalid_ms 转换为 ISR 周期数
			 * 控制频率 10kHz (dt=0.0001s), 200ms = 2000个周期 */
			uint32_t vbus_invalid_thresh = (uint32_t)((float)cfg->vbus_invalid_ms * 0.001f / sys->motor.dt + 0.5f);
			if (vbus_invalid_thresh == 0u)
				vbus_invalid_thresh = 1u;
			if (fm->vbus_guard_cycles == 0u && fm->vbus_invalid_cycles >= vbus_invalid_thresh)
				fault_mgr_internal_set(FAULT_VBUS_SAMPLE, fb->bus_voltage);
#endif /* FAULT_DET_EN_VBUS_SAMPLE */
			/* 采样疑似失效(<0.5V): 过压/欠压判据跳过, 防失效读数误锁存 */
		}
		else
		{
			const uint32_t en_ov = enable; /* 总使能 */
			const uint32_t en_uv = enable;
			const uint8_t not_idle = (sys->top_state != TOP_FSM_IDLE);

			fm->vbus_invalid_cycles = 0u;
			fault_mgr_internal_clear(FAULT_VBUS_SAMPLE);

			/* 过压: 故障档置位时清中度, 故障档未触发且在中度区间则置中度 */
			if (en_ov != 0u && fault_mgr_is_enabled(FAULT_VBUS_OVER) &&
			    fb->bus_voltage > pp->protect_over_voltage)
			{
				fault_mgr_internal_set(FAULT_VBUS_OVER, fb->bus_voltage);
				fault_mgr_internal_clear(FAULT_VBUS_OVER_MID);
			}
			else
			{
				fault_mgr_internal_clear(FAULT_VBUS_OVER);
#if FAULT_DET_EN_VBUS_MID
				if (fb->bus_voltage > cfg->ov_mid_v)
					fault_mgr_internal_set(FAULT_VBUS_OVER_MID, fb->bus_voltage);
				else
					fault_mgr_internal_clear(FAULT_VBUS_OVER_MID);
#else
				fault_mgr_internal_clear(FAULT_VBUS_OVER_MID);
#endif /* FAULT_DET_EN_VBUS_MID */
			}

			/* 欠压: 仅使能后检测(上电前母线未建立) */
			if (en_uv != 0u && fault_mgr_is_enabled(FAULT_VBUS_UNDER) &&
			    not_idle && fb->bus_voltage < pp->protect_under_voltage)
			{
				fault_mgr_internal_set(FAULT_VBUS_UNDER, fb->bus_voltage);
				fault_mgr_internal_clear(FAULT_VBUS_UNDER_MID);
			}
			else
			{
				fault_mgr_internal_clear(FAULT_VBUS_UNDER);
#if FAULT_DET_EN_VBUS_MID
				if (not_idle && fb->bus_voltage < cfg->uv_mid_v)
					fault_mgr_internal_set(FAULT_VBUS_UNDER_MID, fb->bus_voltage);
				else
					fault_mgr_internal_clear(FAULT_VBUS_UNDER_MID);
#else
				fault_mgr_internal_clear(FAULT_VBUS_UNDER_MID);
#endif /* FAULT_DET_EN_VBUS_MID */
			}
		}
	}

	/* ---- 0x4106 超速: 启动窗口结束 + READY/RUN 态, 10ms消抖 ---- */
	if (fm->speed_guard_cycles == 0u &&
	    (sys->top_state == TOP_FSM_READY || sys->top_state == TOP_FSM_RUN) &&
	    enable != 0u &&
	    fault_mgr_is_enabled(FAULT_MOTOR_OVER_SPEED) &&
	    fdet_is_finite(fb->vel) && fdet_absf(fb->vel) > pp->protect_over_speed)
	{
		if (fm->over_speed_cycles < fm->over_speed_confirm_cyc)
			fm->over_speed_cycles++;
	}
	else
	{
		fm->over_speed_cycles = 0u;
	}
	if (fm->speed_guard_cycles == 0u && fm->over_speed_cycles >= fm->over_speed_confirm_cyc)
		fault_mgr_internal_set(FAULT_MOTOR_OVER_SPEED, fb->vel);
	else
		fault_mgr_internal_clear(FAULT_MOTOR_OVER_SPEED);

	/* ---- 0x3101 nFAULT: 10ms 消抖(覆盖 DRV8350 8ms 自动重试窗口) ---- */
	if (fb->gate_driver_fault != 0u)
	{
		if (fm->gate_fault_cycles < fm->gate_fault_confirm_cyc)
			fm->gate_fault_cycles++;
	}
	else
	{
		fm->gate_fault_cycles = 0u;
	}
	if (fm->speed_guard_cycles == 0u &&
	    enable != 0u &&
	    fault_mgr_is_enabled(FAULT_GATE_NFAULT) &&
	    fm->gate_fault_cycles >= fm->gate_fault_confirm_cyc)
		fault_mgr_internal_set(FAULT_GATE_NFAULT, 1.0f);
	else
		fault_mgr_internal_clear(FAULT_GATE_NFAULT);

	/* ---- 0x5101 绝对位置丢失 / 0x5102 磁编码器消磁 / 0x5105 通讯中断 ---- */
#if FAULT_DET_EN_ENC_HEALTH
	/* enc_health 位图: bit0=位置无效(mg INVALID) bit1=磁场过弱(mg TOO_WEAK) */
	if ((fb->enc_health & 0x01u) != 0u)
		fault_mgr_internal_set(FAULT_ENC_POS_LOST, fb->mech_angle_deg);
	else
		fault_mgr_internal_clear(FAULT_ENC_POS_LOST);

	if ((fb->enc_health & 0x02u) != 0u)
		fault_mgr_internal_set(FAULT_ENC_DEMAG, fb->mech_angle_deg);
	else
		fault_mgr_internal_clear(FAULT_ENC_DEMAG);

	if (fb->enc_err_cnt >= cfg->enc_err_frames)
		fault_mgr_internal_set(FAULT_ENC_COMM_LOST, (float)fb->enc_err_cnt);
	else
		fault_mgr_internal_clear(FAULT_ENC_COMM_LOST);
#endif /* FAULT_DET_EN_ENC_HEALTH */

	/* ---- 0x5104 位置跳变: 单拍机械角度跳变(编码器误码残余/CRC 漏检)
	 *      仅 READY/RUN 武装: 标定电流注入/手拧转子时跳变判据不可靠 ---- */
#if FAULT_DET_EN_ENC_JUMP
	if ((sys->top_state == TOP_FSM_READY || sys->top_state == TOP_FSM_RUN) && fm->enc_prev_valid != 0u)
	{
		float jump = fdet_deg_delta(fb->mech_angle_deg, fm->enc_prev_deg);
		if (jump > cfg->pos_jump_deg)
			fault_mgr_internal_set(FAULT_ENC_JUMP, jump);
		else
			fault_mgr_internal_clear(FAULT_ENC_JUMP);
	}
	else
	{
		fault_mgr_internal_clear(FAULT_ENC_JUMP);
	}
	fm->enc_prev_deg = fb->mech_angle_deg;
	fm->enc_prev_valid = 1u;
#endif /* FAULT_DET_EN_ENC_JUMP */

	/* ---- 0x8108 跟随误差(仅 RUN 态位置类模式) ---- */
#if FAULT_DET_EN_FOLLOW_ERR
	if (sys->top_state == TOP_FSM_RUN && sys->motor.ref.ctrl_type == REF_CTRL_POSITION)
	{
		float err = fdet_absf(sys->motor.ref.pos - fb->pos);
		if (err > cfg->follow_err_rad)
			fault_mgr_internal_set(FAULT_FOLLOW_ERR, err);
		else
			fault_mgr_internal_clear(FAULT_FOLLOW_ERR);
	}
	else
	{
		fault_mgr_internal_clear(FAULT_FOLLOW_ERR);
	}
#endif /* FAULT_DET_EN_FOLLOW_ERR */

	/* ---- 0x1201 软限位触发 / 0x1301 接近警告(仅 RUN 态) ---- */
#if FAULT_DET_EN_SOFT_LIMIT
	if (sys->top_state == TOP_FSM_RUN)
	{
		float warn_rad = cfg->soft_limit_warn_deg * 0.01745329f;
		if (fb->pos > cfg->soft_limit_max_rad || fb->pos < cfg->soft_limit_min_rad)
			fault_mgr_internal_set(FAULT_SOFT_LIMIT, fb->pos);
		else
			fault_mgr_internal_clear(FAULT_SOFT_LIMIT);

		if (fb->pos > (cfg->soft_limit_max_rad - warn_rad) || fb->pos < (cfg->soft_limit_min_rad + warn_rad))
			fault_mgr_internal_set(FAULT_NEAR_SOFT_LIMIT, fb->pos);
		else
			fault_mgr_internal_clear(FAULT_NEAR_SOFT_LIMIT);
	}
	else
	{
		fault_mgr_internal_clear(FAULT_SOFT_LIMIT);
		fault_mgr_internal_clear(FAULT_NEAR_SOFT_LIMIT);
	}
#endif /* FAULT_DET_EN_SOFT_LIMIT */
}

/* 消抖阈值初始化(由 fault_mgr_init 按控制周期换算, 此处导出供其调用) */
void fault_detect_fast_init_cycles(fault_mgr_t *fm, float dt)
{
	if (dt > 0.0f)
	{
		fm->speed_guard_cycles = (uint32_t)(FDET_SPEED_GUARD_SECONDS / dt + 0.5f);
		fm->gate_fault_confirm_cyc = (uint32_t)(FDET_GATE_CONFIRM_SECONDS / dt + 0.5f);
		fm->vbus_guard_cycles = (uint32_t)(FDET_VBUS_GUARD_SECONDS / dt + 0.5f);
		/* 峰值过流消抖: 5ms */
		fm->over_current_confirm_cyc = (uint32_t)(0.005f / dt + 0.5f);
		/* 超速消抖: 10ms */
		fm->over_speed_confirm_cyc = (uint32_t)(0.010f / dt + 0.5f);
	}
	/* 确保阈值至少为1，避免除零或永不触发 */
	if (fm->speed_guard_cycles == 0u)
		fm->speed_guard_cycles = 1u;
	if (fm->gate_fault_confirm_cyc == 0u)
		fm->gate_fault_confirm_cyc = 1u;
	if (fm->vbus_guard_cycles == 0u)
		fm->vbus_guard_cycles = 1u;
	if (fm->over_current_confirm_cyc == 0u)
		fm->over_current_confirm_cyc = 1u;
	if (fm->over_speed_confirm_cyc == 0u)
		fm->over_speed_confirm_cyc = 1u;
}
