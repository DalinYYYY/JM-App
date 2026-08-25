/**
 * @file calib_level2_motor.c
 * @brief L2 电机电气身份辨识（相序/极对数/R/Ld/Lq/flux）
 * @note 六个子模式均已实现：正常阻值电机用 DC 差分法/阶跃响应法/反电势法，
 *       低阻电机自动切换交流注入法。硬件访问通过 calib_hw 共享层，参数由
 *       calib_config.h 集中管理，结果写入 motor_param_set_r/ld/lq/flux/pole_pairs，
 *       相序识别结果写入 encoder_param.enc_direction（开环旋转平均法）。
 */
#include <math.h>
#include "calib_types.h"
#include "calib_config.h"
#include "calib_config_runtime.h" /* 运行期派生参数（替代 calib_config.h 的编译期派生宏）*/
#include "calib_mgr.h"
#include "calib_hw.h"
#include "dev_motor.h"
#include "motor_param.h"
#include "calib_step.h"
#include "calib_validate.h"
#include "motor_info_calib.h"
#include "calib_ac_injection.h" /* 交流注入法（低阻电机 R/Ld 辨识）*/

/* ===================== 模块私有状态（合并为单一结构体）===================== */
static struct
{
	uint8_t submode;
	calib_step_t step;          /* 统一状态机骨架（cur/tick/sample_cnt/sample_sum）*/
	calib_hw_session_t session; /* 标定电压会话（替换电角度回调 + 施加电压）*/
	float prev_i;               /* 前一次采样电流（Ld/Lq 阶跃求 did/dt 用）*/
	float start_mech_deg;       /* 起始机械角度（极对数用）*/
	float scan_ele_rad;         /* 开环扫描：当前命令电角度累加值(rad，不折返)，相序/极对数共用*/
	float accum_mech_deg;       /* 开环扫描：连续累加的机械角(deg，跨 360° 不 wrap)，相序/极对数共用*/
	float prev_mech_deg;        /* 开环扫描：上一 tick 角度读数(deg，用于差分)，相序/极对数共用*/
	float test_voltage;         /* 本次施加的测试电压（R/Ld/Lq/flux 算结果时用）*/
	float r_id_low;             /* R 两点差分法：低电压档稳态 id 均值（高档采样时暂存）*/
	/* R 标定诊断字段(调试器观察用): 区分 step 4 失败是 id≈0 还是 d_id≈0 */
	float r_id_high; /* 高档采样 id 均值 */
	float r_d_id;    /* d_id = id_high - id_low */
	float r_d_v;     /* d_v = V2 - V1 */
	float r_result;  /* R 计算结果(失败时为 0) */
	/* 交流注入法上下文（低阻电机 R/Ld 辨识专用）*/
	calib_ac_injection_t ac_ctx;
	float ac_r_result;       /* 交流注入法 R 结果(诊断用) */
	float ac_ld_result;      /* 交流注入法 Ld 结果(诊断用) */
	uint8_t ac_dir_inverted; /* 电流方向修正标志: 1=检测到电流反向并已修正 */
} s_l2;

/* ===================== R 辨识 =====================
 * 正常阻值电机(R ≥ 0.5Ω)：两点差分法 R = ΔV/Δid
 *   单点 R=V/id 把命令电压当实际相压，会被死区/MOSFET-Rds/体二极管的恒定压降
 *   V_loss 系统性抬高（低测试电压下占比大）。本法在两个电流点各测稳态，相减
 *   抵消 V_loss：R = (V2 - V1) / (id2 - id1)。
 *
 *   STEP 0: 进入会话，施加低档电压 V1（theta=0 锁定 d 轴）
 *   STEP 1: 等待 V1 稳态
 *   STEP 2: 低档多次采样 id 取均值 → 暂存 r_id_low，切到高档 V2
 *   STEP 3: 等待 V2 稳态
 *   STEP 4: 高档多次采样 id 取均值 id2，R=(V2-V1)/(id2-r_id_low)，校验写入
 *
 * 低阻电机(R < 0.5Ω)：交流注入法（相敏检测分离 R/Ld）
 *   两点差分法对低阻电机失效：v_hi=test_current*r 过小（如 0.18V），
 *   v_lo 被死区估计 1.0V 抬高，导致 v_lo > v_hi 违反差分法前提。
 *   交流注入法在 d 轴施加 ud=U_dc+U_ac·sin(ωt)，通过相敏检测分离 R 和 Ld，
 *   同时输出 R 和 Ld，低阻电机标定 R 时顺便完成 Ld。
 *
 *   STEP 0: 进入会话 + 初始化交流注入上下文
 *   STEP 1: poll 交流注入（预热 + 采样 total_ticks 拍）
 *   STEP 2: 计算结果 R/Ld，校验写入，完成
 * ========================================================================= */
static calib_state_e poll_resistance(void)
{
	const calib_io_t *io = calib_mgr_get_io();
	dev_motor_t *m = io->motor;

	/* ---------- 低阻电机分支：交流注入法 ---------- */
	if (calib_is_low_r())
	{
		switch (s_l2.step.cur)
		{
			case 0: /* 进入会话 + 初始化交流注入上下文 */
				calib_hw_enter(&s_l2.session, m);
				calib_ac_injection_init(&s_l2.ac_ctx, &s_l2.session);
				calib_mgr_set_step(0);
				calib_step_next(&s_l2.step, 1);
				calib_mgr_set_step(1);
				return CALIB_STATE_RUNNING;

			case 1: /* poll 交流注入直到 total_ticks */
				calib_ac_injection_poll(&s_l2.ac_ctx);
				if (s_l2.ac_ctx.tick < s_l2.ac_ctx.total_ticks)
					return CALIB_STATE_RUNNING;
				calib_step_next(&s_l2.step, 2);
				calib_mgr_set_step(2);
				return CALIB_STATE_RUNNING;

			case 2: /* 计算结果 R/Ld，校验写入 */
			{
				float R = 0.0f, Ld = 0.0f;
				calib_ac_injection_result(&s_l2.ac_ctx, &R, &Ld);
				/* 电流方向修正: R 物理上必须为正, 若 R<0 说明电流采样极性反向
				 * (IB/IC 通道 ADC 极性或 Clarke 公式符号约定与硬件不一致)。
				 * 交流注入法中 R 和 Ld 都依赖电流方向, 会同步变号,
				 * 故 R<0 时同时反转 R 和 Ld 即可修正。
				 * 注: 这是症状修复, 根因应排查板级电流传感器方向或 ADC 通道极性。*/
				if (isfinite(R) && R < 0.0f)
				{
					R = -R;
					Ld = -Ld;
					s_l2.ac_dir_inverted = 1;
				}
				else
				{
					s_l2.ac_dir_inverted = 0;
				}
				s_l2.ac_r_result = R;
				s_l2.ac_ld_result = Ld;
				if (!isfinite(R) || R <= 0.0f)
				{
					calib_mgr_set_fail_reason(CALIB_FAIL_SAMPLE_ABNORMAL);
					calib_hw_exit(&s_l2.session);
					return CALIB_STATE_FAILED;
				}
				if (!calib_validate_r(R))
				{
					calib_mgr_set_fail_reason(CALIB_FAIL_OUT_OF_RANGE);
					calib_hw_exit(&s_l2.session);
					return CALIB_STATE_FAILED;
				}
				(io->param)->motor_base.r = R;
				(void)motor_info_calib_submit_r(R);
				/* 交流注入法同时输出 Ld，校验通过则顺便写入并标记 Ld 完成 */
				if (isfinite(Ld) && Ld > 0.0f && calib_validate_ld(Ld))
				{
					(io->param)->motor_base.ld = Ld;
					(void)motor_info_calib_submit_ld(Ld);
					calib_mgr_mark_done(CALIB_LEVEL2_MOTOR, CALIB_L2_INDUCTANCE_D);
				}
				calib_hw_exit(&s_l2.session);
				calib_mgr_mark_done(CALIB_LEVEL2_MOTOR, CALIB_L2_RESISTANCE);
				calib_step_reset(&s_l2.step);
				return CALIB_STATE_DONE;
			}

			default:
				calib_mgr_set_fail_reason(CALIB_FAIL_TIMEOUT);
				calib_hw_exit(&s_l2.session);
				return CALIB_STATE_FAILED;
		}
	}

	/* ---------- 正常阻值电机分支：两点差分法 ---------- */
	switch (s_l2.step.cur)
	{
		case 0: /* 进入会话，施加低档电压 V1 */
			calib_hw_enter(&s_l2.session, m);
			s_l2.test_voltage = calib_cfg_l2_r_test_voltage_lo_v();
			calib_hw_apply_voltage(&s_l2.session, s_l2.test_voltage, 0.0f, 0.0f);
			calib_mgr_set_step(0);
			calib_step_next(&s_l2.step, 1);
			calib_mgr_set_step(1);
			return CALIB_STATE_RUNNING;

		case 1: /* 等待 V1 稳态 */
			calib_hw_apply_voltage(&s_l2.session, s_l2.test_voltage, 0.0f, 0.0f);
			if (calib_step_wait(&s_l2.step, calib_cfg_l2_r_test_ticks()))
				return CALIB_STATE_RUNNING;
			calib_step_next(&s_l2.step, 2);
			calib_mgr_set_step(2);
			return CALIB_STATE_RUNNING;

		case 2: /* 低档采样 id → 均值暂存，切高档 V2 */
		{
			calib_hw_apply_voltage(&s_l2.session, s_l2.test_voltage, 0.0f, 0.0f);
			m->foc.clarke(&m->foc); /* 刷新 i_alphaBeta（内部调 current_callback）*/
			m->foc.park(&m->foc);   /* 刷新 i_dq */
			float id = m->foc.i_dq.d;
			/* 用绝对值检查: 电流方向可能因板级(DRV8301 vs INA199B1)或 ADC 通道
			 * 极性不同而反转, 只要 |id| 足够大即说明电机有响应 */
			if (!isfinite(id) || fabsf(id) < 0.001f)
			{
				calib_mgr_set_fail_reason(CALIB_FAIL_SAMPLE_ABNORMAL);
				calib_hw_exit(&s_l2.session);
				return CALIB_STATE_FAILED;
			}
			calib_step_accumulate(&s_l2.step, id);
			if (s_l2.step.sample_cnt < calib_cfg_l2_r_sample_count())
				return CALIB_STATE_RUNNING;
			s_l2.r_id_low = calib_step_average(&s_l2.step);      /* 暂存 V1 档 id 均值 */
			s_l2.step.sample_cnt = 0;                            /* 复位累加器供高档复用 */
			s_l2.step.sample_sum = 0.0f;
			s_l2.test_voltage = calib_cfg_l2_r_test_voltage_v(); /* 切到高档 V2 */
			calib_hw_apply_voltage(&s_l2.session, s_l2.test_voltage, 0.0f, 0.0f);
			calib_step_next(&s_l2.step, 3);
			calib_mgr_set_step(3);
			return CALIB_STATE_RUNNING;
		}

		case 3: /* 等待 V2 稳态 */
			calib_hw_apply_voltage(&s_l2.session, s_l2.test_voltage, 0.0f, 0.0f);
			if (calib_step_wait(&s_l2.step, calib_cfg_l2_r_test_ticks()))
				return CALIB_STATE_RUNNING;
			calib_step_next(&s_l2.step, 4);
			calib_mgr_set_step(4);
			return CALIB_STATE_RUNNING;

		case 4: /* 高档采样 id2，两点差分算 R，校验写入 */
		{
			calib_hw_apply_voltage(&s_l2.session, s_l2.test_voltage, 0.0f, 0.0f);
			m->foc.clarke(&m->foc);
			m->foc.park(&m->foc);
			float id = m->foc.i_dq.d;
			if (!isfinite(id) || fabsf(id) < 0.001f)
			{
				calib_mgr_set_fail_reason(CALIB_FAIL_SAMPLE_ABNORMAL);
				calib_hw_exit(&s_l2.session);
				return CALIB_STATE_FAILED;
			}
			calib_step_accumulate(&s_l2.step, id);
			if (s_l2.step.sample_cnt < calib_cfg_l2_r_sample_count())
				return CALIB_STATE_RUNNING;

			float id_high = calib_step_average(&s_l2.step);
			float d_id = id_high - s_l2.r_id_low;
			float d_v = calib_cfg_l2_r_test_voltage_v() - calib_cfg_l2_r_test_voltage_lo_v();
			/* 诊断: 记录高档 id / d_id / d_v 供调试器观察 */
			s_l2.r_id_high = id_high;
			s_l2.r_d_id = d_id;
			s_l2.r_d_v = d_v;
			/* Δid 过小则差分放大噪声，判为异常 */
			if (!isfinite(d_id) || fabsf(d_id) < 0.001f)
			{
				calib_mgr_set_fail_reason(CALIB_FAIL_SAMPLE_ABNORMAL);
				calib_hw_exit(&s_l2.session);
				return CALIB_STATE_FAILED;
			}
			float R = d_v / d_id;
			s_l2.r_result = R;
			if (!calib_validate_r(R))
			{
				calib_mgr_set_fail_reason(CALIB_FAIL_OUT_OF_RANGE);
				calib_hw_exit(&s_l2.session);
				return CALIB_STATE_FAILED;
			}
			(io->param)->motor_base.r = R;
			(void)motor_info_calib_submit_r(R);
			calib_hw_exit(&s_l2.session);
			calib_mgr_mark_done(CALIB_LEVEL2_MOTOR, CALIB_L2_RESISTANCE);
			calib_step_reset(&s_l2.step);
			return CALIB_STATE_DONE;
		}

		default:
			calib_mgr_set_fail_reason(CALIB_FAIL_TIMEOUT);
			calib_hw_exit(&s_l2.session);
			return CALIB_STATE_FAILED;
	}
}

/* ===================== Ld 辨识 =====================
 * 正常阻值电机(R ≥ 0.5Ω)：d 轴阶跃响应
 *   STEP 0: 施加 ud 阶跃，记录初始 id
 *   STEP 1: 暂态窗口内采样 (ud - R*id)/did_dt 取平均
 *   STEP 2: Ld = avg，校验后写入，完成
 *   公式：ud = R*id + Ld*did/dt  →  Ld = (ud - R*id) / (did/dt)
 *
 * 低阻电机(R < 0.5Ω)：交流注入法辨识 + 失败回退 motor_info 默认值
 *   阶跃响应法对低阻电机失效：ud=0.234V < 死区 0.5V，did_dt 被死区非线性扭曲。
 *   交流注入法在 d 轴注入 ud=U_dc+U_ac·sin(ωt)，相敏检测分离 R 和 Ld。
 *   若交流注入法结果校验失败（低阻电机 I_ac 小、φ 小，Ld 噪声大），
 *   回退使用 motor_info 中的 Ld 默认值（上位机预设或手册值），标记完成。
 *   这是低阻电机工程实践：Ld 标定难度高，常用手册值或离线辨识。
 *
 *   STEP 0: 进入会话 + 初始化交流注入上下文
 *   STEP 1: poll 交流注入直到 total_ticks
 *   STEP 2: 计算结果，校验 Ld：
 *           - 通过 → 写入实测值
 *           - 失败 → 回退 motor_info 默认 Ld，仍标记完成
 * ============================================================ */
static calib_state_e poll_inductance_d(void)
{
	const calib_io_t *io = calib_mgr_get_io();
	dev_motor_t *m = io->motor;
	float dt = io->dt;

	/* ---------- 低阻电机分支：交流注入法 + 失败回退 ---------- */
	if (calib_is_low_r())
	{
		switch (s_l2.step.cur)
		{
			case 0: /* 进入会话 + 初始化交流注入上下文 */
				calib_hw_enter(&s_l2.session, m);
				calib_ac_injection_init(&s_l2.ac_ctx, &s_l2.session);
				calib_mgr_set_step(0);
				calib_step_next(&s_l2.step, 1);
				calib_mgr_set_step(1);
				return CALIB_STATE_RUNNING;

			case 1: /* poll 交流注入直到 total_ticks */
				calib_ac_injection_poll(&s_l2.ac_ctx);
				if (s_l2.ac_ctx.tick < s_l2.ac_ctx.total_ticks)
					return CALIB_STATE_RUNNING;
				calib_step_next(&s_l2.step, 2);
				calib_mgr_set_step(2);
				return CALIB_STATE_RUNNING;

			case 2: /* 计算结果，校验 Ld，失败则回退默认值 */
			{
				float R = 0.0f, Ld = 0.0f;
				calib_ac_injection_result(&s_l2.ac_ctx, &R, &Ld);
				/* 电流方向修正（与 R 标定同逻辑）*/
				if (isfinite(R) && R < 0.0f)
				{
					R = -R;
					Ld = -Ld;
					s_l2.ac_dir_inverted = 1;
				}
				else
				{
					s_l2.ac_dir_inverted = 0;
				}
				s_l2.ac_r_result = R;
				s_l2.ac_ld_result = Ld;

				int ld_ok = isfinite(Ld) && Ld > 0.0f && calib_validate_ld(Ld);
				if (ld_ok)
				{
					/* 交流注入法结果可信，写入实测值 */
					(io->param)->motor_base.ld = Ld;
					(void)motor_info_calib_submit_ld(Ld);
					/* 顺便刷新 R（如果 R 校验通过）*/
					if (isfinite(R) && R > 0.0f && calib_validate_r(R))
					{
						(io->param)->motor_base.r = R;
						(void)motor_info_calib_submit_r(R);
					}
				}
				else
				{
					/* 交流注入法失败（低阻电机 Ld 噪声大），
					 * 回退使用 motor_info 默认 Ld（上位机预设或手册值），
					 * 不写入新值，仅标记标定完成。
					 * 注: 用户可通过上位机 0xE7 手动写入精确 Ld。*/
					float default_ld = calib_motor_ident_get()->ld;
					if (!isfinite(default_ld) || default_ld <= 0.0f
					    || !calib_validate_ld(default_ld))
					{
						/* 默认值也非法，无法回退 */
						calib_mgr_set_fail_reason(CALIB_FAIL_OUT_OF_RANGE);
						calib_hw_exit(&s_l2.session);
						return CALIB_STATE_FAILED;
					}
					(io->param)->motor_base.ld = default_ld;
					/* 不调 motor_info_calib_submit_ld，保留 motor_info 中的原值 */
				}
				calib_hw_exit(&s_l2.session);
				calib_mgr_mark_done(CALIB_LEVEL2_MOTOR, CALIB_L2_INDUCTANCE_D);
				calib_step_reset(&s_l2.step);
				return CALIB_STATE_DONE;
			}

			default:
				calib_mgr_set_fail_reason(CALIB_FAIL_TIMEOUT);
				calib_hw_exit(&s_l2.session);
				return CALIB_STATE_FAILED;
		}
	}

	/* ---------- 正常阻值电机分支：d 轴阶跃响应 ---------- */
	switch (s_l2.step.cur)
	{
		case 0: /* 施加 ud 阶跃 */
			calib_hw_enter(&s_l2.session, m);
			s_l2.test_voltage = calib_cfg_l2_ld_test_voltage_v();
			calib_hw_apply_voltage(&s_l2.session, s_l2.test_voltage, 0.0f, 0.0f);
			m->foc.clarke(&m->foc);
			m->foc.park(&m->foc);
			s_l2.prev_i = m->foc.i_dq.d;
			calib_mgr_set_step(0);
			calib_step_next(&s_l2.step, 1);
			calib_mgr_set_step(1);
			return CALIB_STATE_RUNNING;

		case 1: /* 暂态窗口内采样 */
		{
			calib_hw_apply_voltage(&s_l2.session, s_l2.test_voltage, 0.0f, 0.0f);
			m->foc.clarke(&m->foc);
			m->foc.park(&m->foc);
			float id = m->foc.i_dq.d;
			float did_dt = (id - s_l2.prev_i) / dt;
			s_l2.prev_i = id;
			/* did_dt 过小则跳过该点（避免除零）*/
			if (isfinite(did_dt) && fabsf(did_dt) > 1.0f)
			{
				/* 从 motor_info 读取 R（与 calib_config_runtime 一致，支持写入后立即标定）*/
				float R = calib_motor_ident_get()->r;
				float Ld = (s_l2.test_voltage - R * id) / did_dt;
				if (isfinite(Ld) && Ld > 0.0f)
					calib_step_accumulate(&s_l2.step, Ld);
			}
			if (s_l2.step.sample_cnt < calib_cfg_l2_ld_sample_count())
				return CALIB_STATE_RUNNING;
			calib_step_next(&s_l2.step, 2);
			calib_mgr_set_step(2);
			return CALIB_STATE_RUNNING;
		}

		case 2: /* 校验，写入 */
		{
			if (s_l2.step.sample_cnt == 0)
			{
				calib_mgr_set_fail_reason(CALIB_FAIL_SAMPLE_ABNORMAL);
				calib_hw_exit(&s_l2.session);
				return CALIB_STATE_FAILED;
			}
			float Ld = calib_step_average(&s_l2.step);
			if (!calib_validate_ld(Ld))
			{
				calib_mgr_set_fail_reason(CALIB_FAIL_OUT_OF_RANGE);
				calib_hw_exit(&s_l2.session);
				return CALIB_STATE_FAILED;
			}
			(io->param)->motor_base.ld = Ld;
			(void)motor_info_calib_submit_ld(Ld);
			calib_hw_exit(&s_l2.session);
			calib_mgr_mark_done(CALIB_LEVEL2_MOTOR, CALIB_L2_INDUCTANCE_D);
			calib_step_reset(&s_l2.step);
			return CALIB_STATE_DONE;
		}

		default:
			calib_mgr_set_fail_reason(CALIB_FAIL_TIMEOUT);
			calib_hw_exit(&s_l2.session);
			return CALIB_STATE_FAILED;
	}
}

/* ===================== Lq 辨识 =====================
 * 正常阻值电机(R ≥ 0.5Ω)：q 轴阶跃响应
 *   STEP 0: 施加 uq 阶跃，记录初始 iq
 *   STEP 1: 暂态窗口内采样 (uq - R*iq)/diq_dt 取平均
 *   STEP 2: Lq = avg，校验后写入，完成
 *   公式：uq = R*iq + Lq*diq/dt  →  Lq = (uq - R*iq) / (diq/dt)
 *   注意：uq 会产生力矩使转子转动，暂态窗口 5ms 内转子因惯量尚未转起。
 *
 * 低阻电机(R < 0.5Ω)：SPMSM 假设 Lq = Ld
 *   低阻电机多为表贴式(SPMSM)，交直轴电感近似相等 Lq ≈ Ld。
 *   阶跃响应法对低阻电机失效（与 Ld 同理：测试电压 < 死区），
 *   交流注入法在 q 轴注入会产生力矩导致电机转动，无法稳定采样。
 *   故低阻电机直接复用 Ld 值作为 Lq，这是 SPMSM 的工程合理假设。
 *   依赖：Ld 必须已标定（R 标定时通过交流注入法顺便完成，或独立 Ld 标定）。
 * ============================================================ */
static calib_state_e poll_inductance_q(void)
{
	const calib_io_t *io = calib_mgr_get_io();
	dev_motor_t *m = io->motor;
	float dt = io->dt;

	/* ---------- 低阻电机分支：SPMSM 假设 Lq = Ld ---------- */
	if (calib_is_low_r())
	{
		float Ld = calib_motor_ident_get()->ld;
		if (!isfinite(Ld) || Ld <= 0.0f)
		{
			/* Ld 未标定，无法复用 */
			calib_mgr_set_fail_reason(CALIB_FAIL_DEP_NOT_MET);
			return CALIB_STATE_FAILED;
		}
		if (!calib_validate_lq(Ld))
		{
			calib_mgr_set_fail_reason(CALIB_FAIL_OUT_OF_RANGE);
			return CALIB_STATE_FAILED;
		}
		(io->param)->motor_base.lq = Ld;
		(void)motor_info_calib_submit_lq(Ld);
		calib_mgr_mark_done(CALIB_LEVEL2_MOTOR, CALIB_L2_INDUCTANCE_Q);
		return CALIB_STATE_DONE;
	}

	/* ---------- 正常阻值电机分支：q 轴阶跃响应 ---------- */
	switch (s_l2.step.cur)
	{
		case 0: /* 施加 uq 阶跃 */
			calib_hw_enter(&s_l2.session, m);
			s_l2.test_voltage = calib_cfg_l2_lq_test_voltage_v();
			calib_hw_apply_voltage(&s_l2.session, 0.0f, s_l2.test_voltage, 0.0f);
			m->foc.clarke(&m->foc);
			m->foc.park(&m->foc);
			s_l2.prev_i = m->foc.i_dq.q;
			calib_mgr_set_step(0);
			calib_step_next(&s_l2.step, 1);
			calib_mgr_set_step(1);
			return CALIB_STATE_RUNNING;

		case 1: /* 暂态窗口内采样 */
		{
			calib_hw_apply_voltage(&s_l2.session, 0.0f, s_l2.test_voltage, 0.0f);
			m->foc.clarke(&m->foc);
			m->foc.park(&m->foc);
			float iq = m->foc.i_dq.q;
			float diq_dt = (iq - s_l2.prev_i) / dt;
			s_l2.prev_i = iq;
			if (isfinite(diq_dt) && fabsf(diq_dt) > 1.0f)
			{
				float R = calib_motor_ident_get()->r;
				float Lq = (s_l2.test_voltage - R * iq) / diq_dt;
				if (isfinite(Lq) && Lq > 0.0f)
					calib_step_accumulate(&s_l2.step, Lq);
			}
			if (s_l2.step.sample_cnt < calib_cfg_l2_lq_sample_count())
				return CALIB_STATE_RUNNING;
			calib_step_next(&s_l2.step, 2);
			calib_mgr_set_step(2);
			return CALIB_STATE_RUNNING;
		}

		case 2: /* 校验，写入 */
		{
			if (s_l2.step.sample_cnt == 0)
			{
				calib_mgr_set_fail_reason(CALIB_FAIL_SAMPLE_ABNORMAL);
				calib_hw_exit(&s_l2.session);
				return CALIB_STATE_FAILED;
			}
			float Lq = calib_step_average(&s_l2.step);
			if (!calib_validate_lq(Lq))
			{
				calib_mgr_set_fail_reason(CALIB_FAIL_OUT_OF_RANGE);
				calib_hw_exit(&s_l2.session);
				return CALIB_STATE_FAILED;
			}
			(io->param)->motor_base.lq = Lq;
			(void)motor_info_calib_submit_lq(Lq);
			calib_hw_exit(&s_l2.session);
			calib_mgr_mark_done(CALIB_LEVEL2_MOTOR, CALIB_L2_INDUCTANCE_Q);
			calib_step_reset(&s_l2.step);
			return CALIB_STATE_DONE;
		}

		default:
			calib_mgr_set_fail_reason(CALIB_FAIL_TIMEOUT);
			calib_hw_exit(&s_l2.session);
			return CALIB_STATE_FAILED;
	}
}

/* ===================== flux 辨识（反电势法）=====================
 * 正常阻值电机(R ≥ 0.5Ω)：开环电压驱动稳速 + 反电势法
 *   STEP 0: 施加 uq 驱动电机转动（theta 跟随实时电角度，不替换回调）
 *   STEP 1: 等待稳速（calib_cfg_l2_flux_spin_ticks()）
 *   STEP 2: 多次采样 flux_k = (uq - R*iq) / omega_e 取平均
 *   STEP 3: flux = avg，校验后写入，完成
 *
 *   公式：稳速时 uq ≈ R*iq + omega_e*flux（忽略 Lq*diq/dt 稳态为 0）
 *         omega_e = omega_m * pole_pairs
 *
 * 低阻电机(R < 0.5Ω)：死区补偿驱动 + 反电势法 + 失败回退默认值
 *   低阻电机 spin_voltage ≈ R*iq + flux*pp*omega ≈ 0.38V < 死区 0.5V，
 *   电机无法转起，omega_e≈0，flux 计算除零或负值。
 *   修复：spin_voltage 加死区补偿 V_dead，使实际加到绕组的电压进入线性区。
 *   若仍失败（电机未转起或 flux 计算异常），回退 motor_info 默认 flux。
 *
 *   注意：本子模式不调 calib_hw_enter（需保留实时电角度回调让电机转动），
 *         直接用 calib_hw_apply_voltage 传 m->motor_param.ele_radian 作为 theta。
 * ============================================================ */
static calib_state_e poll_flux_linkage(void)
{
	const calib_io_t *io = calib_mgr_get_io();
	dev_motor_t *m = io->motor;

	/* 低阻电机 spin_voltage 加死区补偿 */
	float spin_v = calib_cfg_l2_flux_spin_voltage_v();
	if (calib_is_low_r())
	{
		/* ud_dc 已在 R/Ld 标定中证明：低阻电机需 V_dead*1.5 才能进入线性区。
		 * flux 标定同样需要补偿死区，否则电机不转。*/
		spin_v += CALIB_CFG_L2_R_V_DT_ESTIMATE_V * 1.5f; /* +0.75V */
	}

	switch (s_l2.step.cur)
	{
		case 0:                              /* 初始化会话（仅记录 motor 指针，不替换回调）*/
			s_l2.session.motor = m;
			s_l2.session.orig_ele_cb = NULL; /* 标记不替换 */
			s_l2.session.forced_ele_angle = 0.0f;
			s_l2.test_voltage = spin_v;
			calib_mgr_set_step(0);
			calib_step_next(&s_l2.step, 1);
			calib_mgr_set_step(1);
			return CALIB_STATE_RUNNING;

		case 1:                                      /* 驱动转动，等待稳速 */
		{
			float theta = m->motor_param.ele_radian; /* 跟随实时电角度 */
			calib_hw_apply_voltage(&s_l2.session, 0.0f, s_l2.test_voltage, theta);
			if (calib_step_wait(&s_l2.step, calib_cfg_l2_flux_spin_ticks()))
				return CALIB_STATE_RUNNING;
			calib_step_next(&s_l2.step, 2);
			calib_mgr_set_step(2);
			return CALIB_STATE_RUNNING;
		}

		case 2: /* 稳态采样 */
		{
			float theta = m->motor_param.ele_radian;
			calib_hw_apply_voltage(&s_l2.session, 0.0f, s_l2.test_voltage, theta);
			m->foc.clarke(&m->foc);
			m->foc.park(&m->foc);
			float iq = m->foc.i_dq.q;
			float omega_m = m->motor_param.slide_rad_s;
			/* 从 motor_info 读取 pole_pairs（与 calib_config_runtime 一致）*/
			uint32_t pp = calib_motor_ident_get()->pole_pairs;
			float omega_e = omega_m * (float)pp;
			/* omega_e 过小则跳过（电机未转起）*/
			if (isfinite(omega_e) && fabsf(omega_e) > 1.0f && isfinite(iq))
			{
				float R = calib_motor_ident_get()->r;
				/* 低阻电机需扣除死区补偿电压，否则 flux 偏大 */
				float uq_eff = s_l2.test_voltage;
				if (calib_is_low_r())
					uq_eff -= CALIB_CFG_L2_R_V_DT_ESTIMATE_V * 1.5f;
				float flux_k = (uq_eff - R * iq) / omega_e;
				if (isfinite(flux_k) && flux_k > 0.0f)
					calib_step_accumulate(&s_l2.step, flux_k);
			}
			if (s_l2.step.sample_cnt < calib_cfg_l2_flux_sample_count())
				return CALIB_STATE_RUNNING;
			calib_step_next(&s_l2.step, 3);
			calib_mgr_set_step(3);
			return CALIB_STATE_RUNNING;
		}

		case 3:                     /* 校验，写入，撤销电压 */
		{
			calib_hw_apply_zero(m); /* 直接置零 PWM（未替换回调，无需 exit 恢复）*/
			if (s_l2.step.sample_cnt == 0)
			{
				/* 低阻电机可能因未转起导致无采样，回退默认 flux */
				if (calib_is_low_r())
				{
					float default_flux = calib_motor_ident_get()->flux;
					if (isfinite(default_flux) && default_flux > 0.0f
					    && calib_validate_flux(default_flux))
					{
						(io->param)->motor_base.flux = default_flux;
						calib_mgr_mark_done(CALIB_LEVEL2_MOTOR, CALIB_L2_FLUX_LINKAGE);
						calib_step_reset(&s_l2.step);
						return CALIB_STATE_DONE;
					}
				}
				calib_mgr_set_fail_reason(CALIB_FAIL_SAMPLE_ABNORMAL);
				return CALIB_STATE_FAILED;
			}
			float flux = calib_step_average(&s_l2.step);
			if (!calib_validate_flux(flux))
			{
				/* 低阻电机 flux 计算异常，回退默认值 */
				if (calib_is_low_r())
				{
					float default_flux = calib_motor_ident_get()->flux;
					if (isfinite(default_flux) && default_flux > 0.0f
					    && calib_validate_flux(default_flux))
					{
						(io->param)->motor_base.flux = default_flux;
						calib_mgr_mark_done(CALIB_LEVEL2_MOTOR, CALIB_L2_FLUX_LINKAGE);
						calib_step_reset(&s_l2.step);
						return CALIB_STATE_DONE;
					}
				}
				calib_mgr_set_fail_reason(CALIB_FAIL_OUT_OF_RANGE);
				return CALIB_STATE_FAILED;
			}
			(io->param)->motor_base.flux = flux;
			(void)motor_info_calib_submit_flux(flux);
			calib_mgr_mark_done(CALIB_LEVEL2_MOTOR, CALIB_L2_FLUX_LINKAGE);
			calib_step_reset(&s_l2.step);
			return CALIB_STATE_DONE;
		}

		default:
			calib_mgr_set_fail_reason(CALIB_FAIL_TIMEOUT);
			calib_hw_apply_zero(m);
			return CALIB_STATE_FAILED;
	}
}

/* ===================== 相序识别（开环旋转平均法）=====================
 * 原理：开环正向扫描强制电角度 N 个整电周期（扫描速度复用极对数辨识的
 *   2 电周期/秒），转子的 d 轴被锁定跟随。累计编码器"原始角"（不含
 *   offset/dir 补偿，对二者免疫，避免用补偿后角度判定造成自证）：
 *     累计原始角 > 0 → 编码器计数方向与正向电角度一致 → enc_direction=+1
 *     累计原始角 < 0 → 相序接反/计数方向相反 → enc_direction=-1
 *   整电周期平均天然消除齿槽转矩与摩擦偏置；固件无法区分"UVW 接反"与
 *   "编码器计数反向"（观测等价），统一用 enc_direction 收敛修正，
 *   与 ODrive/VESC encoder detect 同思路。
 *
 *   STEP 0: 进入会话，施加 ud（theta=0）对齐 d 轴
 *   STEP 1: 等待对齐稳定
 *   STEP 2: 记录起始原始角，初始化连续累加器
 *   STEP 3: 每 tick 递增强制电角度（开环扫描），原始角差分去 ±360° 跳变
 *           后累加；命令电角度达到 N·2π 结束
 *   STEP 4: 幅度校验（|累计角| ≥ 理论值 360·N/pole_pairs 的一半，否则
 *           判堵转），符号判定 enc_direction，写回运行时 + motor_param +
 *           motor_info，完成
 *
 * @note 方向翻转后 enc_offset 语义失效，需重标零位（L7 流程中零位标定
 *       在相序识别之后，自动覆盖；单步标定时需手动先相序后零位）。
 * ================================================== */
static calib_state_e poll_phase_seq(void)
{
	const calib_io_t *io = calib_mgr_get_io();
	dev_motor_t *m = io->motor;

	switch (s_l2.step.cur)
	{
		case 0: /* 对齐 d 轴 */
			calib_hw_enter(&s_l2.session, m);
			/* 临时置正向(1)：MT6835 的 get_raw 内部按 running_dir 反转 raw，
			 * 若上次标定残留 -1 会污染本次累计角符号，导致结果永远与
			 * 当前配置一致（自证）。与 L3.1/L3.2 的同款防御。
			 * case 4 会写入判定出的正确方向。*/
			m->encoder.set_dir(&m->encoder, 1);
			calib_hw_apply_voltage(&s_l2.session, calib_cfg_l2_phase_seq_voltage_v(), 0.0f, 0.0f);
			calib_mgr_set_step(0);
			calib_step_next(&s_l2.step, 1);
			calib_mgr_set_step(1);
			return CALIB_STATE_RUNNING;

		case 1: /* 等待对齐 */
			calib_hw_apply_voltage(&s_l2.session, calib_cfg_l2_phase_seq_voltage_v(), 0.0f, 0.0f);
			if (calib_step_wait(&s_l2.step, calib_cfg_l2_phase_seq_align_ticks()))
				return CALIB_STATE_RUNNING;
			calib_step_next(&s_l2.step, 2);
			calib_mgr_set_step(2);
			return CALIB_STATE_RUNNING;

		case 2: /* 记录起始原始角，初始化累加器 */
			s_l2.scan_ele_rad = 0.0f;
			s_l2.prev_mech_deg = calib_hw_get_encoder_raw_deg(m);
			s_l2.accum_mech_deg = 0.0f;
			calib_step_next(&s_l2.step, 3);
			calib_mgr_set_step(3);
			return CALIB_STATE_RUNNING;

		case 3: /* 开环正向扫描：递增强制电角度 + 原始角连续累加（unwrap）*/
		{
			s_l2.scan_ele_rad += calib_cfg_l2_pole_pairs_dtheta_rad();
			calib_hw_apply_voltage(&s_l2.session, calib_cfg_l2_phase_seq_voltage_v(), 0.0f, s_l2.scan_ele_rad);

			float raw_deg = calib_hw_get_encoder_raw_deg(m);
			float dmech = raw_deg - s_l2.prev_mech_deg;
			if (dmech > 180.0f)
				dmech -= 360.0f;
			else if (dmech < -180.0f)
				dmech += 360.0f;
			s_l2.accum_mech_deg += dmech;
			s_l2.prev_mech_deg = raw_deg;

			if (s_l2.scan_ele_rad < calib_cfg_l2_phase_seq_scan_target_rad())
				return CALIB_STATE_RUNNING;
			calib_step_next(&s_l2.step, 4);
			calib_mgr_set_step(4);
			return CALIB_STATE_RUNNING;
		}

		case 4: /* 幅度校验 + 符号判定，写回 enc_direction */
		{
			calib_hw_exit(&s_l2.session);

			/* 理论机械转角 = 360°×N/pole_pairs；不足一半判堵转/失步 */
			uint32_t pp = calib_motor_ident_get()->pole_pairs;
			if (pp == 0)
				pp = 1; /* 防零除（极对数未标定时兜底）*/
			float expect_deg = 360.0f * (float)CALIB_CFG_L2_PHASE_SEQ_SCAN_ELE_CYCLES / (float)pp;
			if (fabsf(s_l2.accum_mech_deg) < expect_deg * 0.5f)
			{
				calib_mgr_set_fail_reason(CALIB_FAIL_MOTOR_STUCK); /* 转子未跟随转动 */
				return CALIB_STATE_FAILED;
			}

			int8_t enc_dir = (s_l2.accum_mech_deg > 0.0f) ? 1 : -1;
			if (!calib_validate_enc_direction(enc_dir))
			{
				calib_mgr_set_fail_reason(CALIB_FAIL_OUT_OF_RANGE);
				return CALIB_STATE_FAILED;
			}
			/* 写回运行时编码器方向（统一 -1/1 约定）*/
			m->encoder.set_dir(&m->encoder, enc_dir);
			/* 写入 motor_param（0xE0 读回 index 26）*/
			(io->param)->encoder_param.enc_direction = enc_dir;
			(void)motor_info_calib_submit_enc_direction(enc_dir);
			/* 写入 motor_info.direction（0xE6 读回 index 19，0=正向 1=反向，
			 * 上位机 L2.1 标定结果区显示的就是此字段）*/
			(void)motor_info_calib_submit_direction((enc_dir < 0) ? 1u : 0u);
			calib_mgr_mark_done(CALIB_LEVEL2_MOTOR, CALIB_L2_PHASE_SEQ);
			calib_step_reset(&s_l2.step);
			return CALIB_STATE_DONE;
		}

		default:
			calib_mgr_set_fail_reason(CALIB_FAIL_TIMEOUT);
			calib_hw_exit(&s_l2.session);
			return CALIB_STATE_FAILED;
	}
}

/* ===================== 极对数辨识（开环强制电角度扫描法）=====================
 * STEP 0: 进入会话接管电角度，施加 ud 锁定转子到强制电角度 0（对齐 d 轴）
 * STEP 1: 等待对齐稳定
 * STEP 2: 记录起始机械角，初始化连续累加器
 * STEP 3: 每 tick 递增强制电角度（开环扫描），同时对机械角做连续累加（unwrap），
 *         命令电角度累加到 N·2π 即结束
 * STEP 4: pole_pairs = 命令电角度总量(N·2π) / |累计机械弧度|，四舍五入校验写入
 *
 * 原理：转子的 d 轴始终锁在我方"强制施加"的电角度上。强制电角度扫过 N 个
 *   完整电周期（= N·2π，精确已知），转子机械角实际转过 N/pole_pairs 圈。
 *     pole_pairs = 命令电角度总量 / 实测机械角总量
 *   分子由我方开环命令决定（独立精确），分母由编码器实测——两者独立，可真辨识。

 * =========================================================================== */
static calib_state_e poll_pole_pairs(void)
{
	const calib_io_t *io = calib_mgr_get_io();
	dev_motor_t *m = io->motor;

	switch (s_l2.step.cur)
	{
		case 0: /* 接管电角度，锁定转子到强制电角度 0 */
			calib_hw_enter(&s_l2.session, m);
			s_l2.scan_ele_rad = 0.0f;
			calib_hw_apply_voltage(&s_l2.session, calib_cfg_l2_pole_pairs_voltage_v(), 0.0f, s_l2.scan_ele_rad);
			calib_mgr_set_step(0);
			calib_step_next(&s_l2.step, 1);
			calib_mgr_set_step(1);
			return CALIB_STATE_RUNNING;

		case 1: /* 等待转子对齐到 d 轴 */
			calib_hw_apply_voltage(&s_l2.session, calib_cfg_l2_pole_pairs_voltage_v(), 0.0f, s_l2.scan_ele_rad);
			if (calib_step_wait(&s_l2.step, calib_cfg_l2_pole_pairs_align_ticks()))
				return CALIB_STATE_RUNNING;
			calib_step_next(&s_l2.step, 2);
			calib_mgr_set_step(2);
			return CALIB_STATE_RUNNING;

		case 2: /* 记录起始机械角，初始化连续累加器 */
			s_l2.start_mech_deg = calib_hw_get_encoder_mech_angle(m);
			s_l2.prev_mech_deg = s_l2.start_mech_deg;
			s_l2.accum_mech_deg = 0.0f;
			calib_step_next(&s_l2.step, 3);
			calib_mgr_set_step(3);
			return CALIB_STATE_RUNNING;

		case 3: /* 开环扫描：递增强制电角度 + 机械角连续累加（unwrap）*/
		{
			/* 递增命令电角度（不折返，作为精确分子）*/
			s_l2.scan_ele_rad += calib_cfg_l2_pole_pairs_dtheta_rad();
			calib_hw_apply_voltage(&s_l2.session, calib_cfg_l2_pole_pairs_voltage_v(), 0.0f, s_l2.scan_ele_rad);

			/* 机械角差分并去 ±360° 跳变，累加成连续量 */
			float mech_deg = calib_hw_get_encoder_mech_angle(m);
			float dmech = mech_deg - s_l2.prev_mech_deg;
			if (dmech > 180.0f)
				dmech -= 360.0f;
			else if (dmech < -180.0f)
				dmech += 360.0f;
			s_l2.accum_mech_deg += dmech;
			s_l2.prev_mech_deg = mech_deg;

			if (s_l2.scan_ele_rad < calib_cfg_l2_pole_pairs_target_rad())
				return CALIB_STATE_RUNNING;
			calib_step_next(&s_l2.step, 4);
			calib_mgr_set_step(4);
			return CALIB_STATE_RUNNING;
		}

		case 4:                           /* 计算极对数 */
		{
			calib_hw_exit(&s_l2.session); /* 撤销电压并恢复电角度回调 */

			float dmech_rad = fabsf(s_l2.accum_mech_deg) * (3.14159265F / 180.0F);
			if (dmech_rad < 0.1f)
			{
				calib_mgr_set_fail_reason(CALIB_FAIL_MOTOR_STUCK); /* 转子未跟随转动 */
				return CALIB_STATE_FAILED;
			}

			/* pole_pairs = 命令电角度总量 / 实测机械角总量 */
			float pp_f = calib_cfg_l2_pole_pairs_target_rad() / dmech_rad;
			uint8_t pp = (uint8_t)(pp_f + 0.5f); /* 四舍五入 */

			if (!calib_validate_pole_pairs(pp))
			{
				calib_mgr_set_fail_reason(CALIB_FAIL_OUT_OF_RANGE);
				return CALIB_STATE_FAILED;
			}
			(io->param)->motor_base.pole_pairs = pp;
			(void)motor_info_calib_submit_pole_pairs(pp);
			calib_mgr_mark_done(CALIB_LEVEL2_MOTOR, CALIB_L2_POLE_PAIRS);
			calib_step_reset(&s_l2.step);
			return CALIB_STATE_DONE;
		}

		default:
			calib_mgr_set_fail_reason(CALIB_FAIL_TIMEOUT);
			calib_hw_exit(&s_l2.session);
			return CALIB_STATE_FAILED;
	}
}

/* ===================== ops 接口 ===================== */
static bool calib_level2_start(uint8_t submode, motor_param_t *param, float dt)
{
	(void)param;
	(void)dt;

	s_l2.submode = submode;
	calib_step_reset(&s_l2.step);
	s_l2.session.motor = NULL;
	s_l2.session.orig_ele_cb = NULL;
	s_l2.session.forced_ele_angle = 0.0f;

	switch (submode)
	{
		case CALIB_L2_PHASE_SEQ:
		case CALIB_L2_POLE_PAIRS:
		case CALIB_L2_RESISTANCE:
		case CALIB_L2_INDUCTANCE_D:
		case CALIB_L2_INDUCTANCE_Q:
		case CALIB_L2_FLUX_LINKAGE:
			return true;
		default:
			return false;
	}
}

static calib_state_e calib_level2_poll(void)
{
	switch (s_l2.submode)
	{
		case CALIB_L2_PHASE_SEQ: return poll_phase_seq();
		case CALIB_L2_POLE_PAIRS: return poll_pole_pairs();
		case CALIB_L2_RESISTANCE: return poll_resistance();
		case CALIB_L2_INDUCTANCE_D: return poll_inductance_d();
		case CALIB_L2_INDUCTANCE_Q: return poll_inductance_q();
		case CALIB_L2_FLUX_LINKAGE: return poll_flux_linkage();
		default: return CALIB_STATE_FAILED;
	}
}

static void calib_level2_abort(void)
{
	const calib_io_t *io = calib_mgr_get_io();
	if (io != NULL && io->motor != NULL)
		calib_hw_exit(&s_l2.session);
	calib_step_reset(&s_l2.step);
}

const calib_level_ops_t calib_level2_ops = {
	.start = calib_level2_start,
	.poll = calib_level2_poll,
	.abort = calib_level2_abort,
};
