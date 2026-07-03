/**
 * @file calib_level2_motor.c
 * @brief L2 电机电气身份辨识（相序/极对数/R/Ld/Lq/flux）
 * @note L2 子模式 1-2（相序/极对数）与 3-6（R/Ld/Lq/flux）当前为桩实现。
 *       真实算法填充 poll_*() 函数即可，硬件访问通过 calib_hw 共享层，
 *       参数通过 calib_config.h 集中管理，结果写入 motor_param_set_r/ld/lq/flux/pole_pairs。
 *
 * @par 各子模式算法（未来实现参考）
 *   - 相序识别：施加 ud，观测三相电流相序方向
 *   - 极对数：施加 ud 旋转一周，数电周期数 = 极对数
 *   - R：施加 DC 电压 ud，稳态后 R = ud / id
 *   - Ld：施加 ud 阶跃，观测 di/dt，Ld = (ud - R*id) / (did/dt)
 *   - Lq：施加 uq 阶跃，观测 diq/dt，Lq = uq / (diq/dt)（近似）
 *   - flux：开环驱动电机稳速转动，flux = (uq - R*iq) / ω
 */
#include <math.h>
#include "calib_types.h"
#include "calib_config.h"
#include "calib_mgr.h"
#include "calib_hw.h"
#include "dev_motor.h"
#include "motor_param.h"
#include "calib_step.h"
#include "calib_validate.h"
#include "motor_info_calib.h"

/* ===================== 模块私有状态（合并为单一结构体）===================== */
static struct
{
	uint8_t submode;
	calib_step_t step;          /* 统一状态机骨架（cur/tick/sample_cnt/sample_sum）*/
	calib_hw_session_t session; /* 标定电压会话（替换电角度回调 + 施加电压）*/
	float prev_i;               /* 前一次采样电流（Ld/Lq 阶跃求 did/dt 用）*/
	float start_mech_deg;       /* 起始机械角度（相序用）*/
	float scan_ele_rad;         /* 极对数扫描：当前命令电角度累加值(rad，不折返)*/
	float accum_mech_deg;       /* 极对数扫描：连续累加的机械角(deg，跨 360° 不 wrap)*/
	float prev_mech_deg;        /* 极对数扫描：上一 tick 机械角原始读数(deg，用于差分)*/
	float test_voltage;         /* 本次施加的测试电压（R/Ld/Lq/flux 算结果时用）*/
} s_l2;

/* ===================== R 辨识（DC 法）=====================
 * STEP 0: 施加 ud DC 电压（theta=0 锁定转子 d 轴）
 * STEP 1: 等待稳态（CALIB_CFG_L2_R_TEST_TICKS）
 * STEP 2: 多次采样 id 取平均
 * STEP 3: R = ud / id_avg，校验后写入 motor_param，完成
 * ========================================================= */
static calib_state_e poll_resistance(void)
{
	const calib_io_t *io = calib_mgr_get_io();
	dev_motor_t *m = io->motor;

	switch (s_l2.step.cur)
	{
		case 0: /* 进入标定会话，施加 DC 电压 */
			calib_hw_enter(&s_l2.session, m);
			s_l2.test_voltage = CALIB_CFG_L2_R_TEST_VOLTAGE_V;
			calib_hw_apply_voltage(&s_l2.session, s_l2.test_voltage, 0.0f, 0.0f);
			calib_step_next(&s_l2.step, 1);
			return CALIB_STATE_RUNNING;

		case 1: /* 等待稳态 */
			calib_hw_apply_voltage(&s_l2.session, s_l2.test_voltage, 0.0f, 0.0f);
			if (calib_step_wait(&s_l2.step, CALIB_CFG_L2_R_TEST_TICKS))
				return CALIB_STATE_RUNNING;
			calib_step_next(&s_l2.step, 2);
			return CALIB_STATE_RUNNING;

		case 2: /* 多次采样 id */
		{
			calib_hw_apply_voltage(&s_l2.session, s_l2.test_voltage, 0.0f, 0.0f);
			m->foc.clarke(&m->foc); /* 刷新 i_alphaBeta（内部调 current_callback）*/
			m->foc.park(&m->foc);   /* 刷新 i_dq */
			float id = m->foc.i_dq.d;
			/* 过滤异常值（NaN/过大）*/
			if (!isfinite(id) || id < 0.001f)
			{
				calib_hw_exit(&s_l2.session);
				return CALIB_STATE_FAILED;
			}
			calib_step_accumulate(&s_l2.step, id);
			if (s_l2.step.sample_cnt < CALIB_CFG_L2_R_SAMPLE_COUNT)
				return CALIB_STATE_RUNNING;
			calib_step_next(&s_l2.step, 3);
			return CALIB_STATE_RUNNING;
		}

		case 3: /* 计算 R，校验，写入 */
		{
			float id_avg = calib_step_average(&s_l2.step);
			float R = s_l2.test_voltage / id_avg;
			if (!calib_validate_r(R))
			{
				calib_hw_exit(&s_l2.session);
				return CALIB_STATE_FAILED;
			}
			motor_param_set_r(io->param, R);
			(void)motor_info_calib_submit_r(R);
			calib_hw_exit(&s_l2.session);
			calib_mgr_mark_done(CALIB_LEVEL2_MOTOR, CALIB_L2_RESISTANCE);
			calib_step_reset(&s_l2.step);
			return CALIB_STATE_DONE;
		}

		default:
			calib_hw_exit(&s_l2.session);
			return CALIB_STATE_FAILED;
	}
}

/* ===================== Ld 辨识（d 轴阶跃响应）=====================
 * STEP 0: 施加 ud 阶跃，记录初始 id
 * STEP 1: 暂态窗口内采样 (ud - R*id)/did_dt 取平均
 * STEP 2: Ld = avg，校验后写入，完成
 *
 * 公式：ud = R*id + Ld*did/dt  →  Ld = (ud - R*id) / (did/dt)
 * R 从 motor_param 读取（前置依赖 L2.3 R 已标定）
 * ============================================================ */
static calib_state_e poll_inductance_d(void)
{
	const calib_io_t *io = calib_mgr_get_io();
	dev_motor_t *m = io->motor;
	float dt = io->dt;

	switch (s_l2.step.cur)
	{
		case 0: /* 施加 ud 阶跃 */
			calib_hw_enter(&s_l2.session, m);
			s_l2.test_voltage = CALIB_CFG_L2_LD_TEST_VOLTAGE_V;
			calib_hw_apply_voltage(&s_l2.session, s_l2.test_voltage, 0.0f, 0.0f);
			m->foc.clarke(&m->foc);
			m->foc.park(&m->foc);
			s_l2.prev_i = m->foc.i_dq.d;
			calib_step_next(&s_l2.step, 1);
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
				float R = motor_param_get_r(io->param);
				float Ld = (s_l2.test_voltage - R * id) / did_dt;
				if (isfinite(Ld) && Ld > 0.0f)
					calib_step_accumulate(&s_l2.step, Ld);
			}
			if (s_l2.step.sample_cnt < CALIB_CFG_L2_LD_SAMPLE_COUNT)
				return CALIB_STATE_RUNNING;
			calib_step_next(&s_l2.step, 2);
			return CALIB_STATE_RUNNING;
		}

		case 2: /* 校验，写入 */
		{
			if (s_l2.step.sample_cnt == 0)
			{
				calib_hw_exit(&s_l2.session);
				return CALIB_STATE_FAILED;
			}
			float Ld = calib_step_average(&s_l2.step);
			if (!calib_validate_ld(Ld))
			{
				calib_hw_exit(&s_l2.session);
				return CALIB_STATE_FAILED;
			}
			motor_param_set_ld(io->param, Ld);
			(void)motor_info_calib_submit_ld(Ld);
			calib_hw_exit(&s_l2.session);
			calib_mgr_mark_done(CALIB_LEVEL2_MOTOR, CALIB_L2_INDUCTANCE_D);
			calib_step_reset(&s_l2.step);
			return CALIB_STATE_DONE;
		}

		default:
			calib_hw_exit(&s_l2.session);
			return CALIB_STATE_FAILED;
	}
}

/* ===================== Lq 辨识（q 轴阶跃响应）=====================
 * STEP 0: 施加 uq 阶跃，记录初始 iq
 * STEP 1: 暂态窗口内采样 (uq - R*iq)/diq_dt 取平均
 * STEP 2: Lq = avg，校验后写入，完成
 *
 * 公式：uq = R*iq + Lq*diq/dt  →  Lq = (uq - R*iq) / (diq/dt)
 * 注意：uq 会产生力矩使转子转动，暂态窗口 5ms 内转子因惯量尚未转起。
 * ============================================================ */
static calib_state_e poll_inductance_q(void)
{
	const calib_io_t *io = calib_mgr_get_io();
	dev_motor_t *m = io->motor;
	float dt = io->dt;

	switch (s_l2.step.cur)
	{
		case 0: /* 施加 uq 阶跃 */
			calib_hw_enter(&s_l2.session, m);
			s_l2.test_voltage = CALIB_CFG_L2_LQ_TEST_VOLTAGE_V;
			calib_hw_apply_voltage(&s_l2.session, 0.0f, s_l2.test_voltage, 0.0f);
			m->foc.clarke(&m->foc);
			m->foc.park(&m->foc);
			s_l2.prev_i = m->foc.i_dq.q;
			calib_step_next(&s_l2.step, 1);
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
				float R = motor_param_get_r(io->param);
				float Lq = (s_l2.test_voltage - R * iq) / diq_dt;
				if (isfinite(Lq) && Lq > 0.0f)
					calib_step_accumulate(&s_l2.step, Lq);
			}
			if (s_l2.step.sample_cnt < CALIB_CFG_L2_LQ_SAMPLE_COUNT)
				return CALIB_STATE_RUNNING;
			calib_step_next(&s_l2.step, 2);
			return CALIB_STATE_RUNNING;
		}

		case 2: /* 校验，写入 */
		{
			if (s_l2.step.sample_cnt == 0)
			{
				calib_hw_exit(&s_l2.session);
				return CALIB_STATE_FAILED;
			}
			float Lq = calib_step_average(&s_l2.step);
			if (!calib_validate_lq(Lq))
			{
				calib_hw_exit(&s_l2.session);
				return CALIB_STATE_FAILED;
			}
			motor_param_set_lq(io->param, Lq);
			(void)motor_info_calib_submit_lq(Lq);
			calib_hw_exit(&s_l2.session);
			calib_mgr_mark_done(CALIB_LEVEL2_MOTOR, CALIB_L2_INDUCTANCE_Q);
			calib_step_reset(&s_l2.step);
			return CALIB_STATE_DONE;
		}

		default:
			calib_hw_exit(&s_l2.session);
			return CALIB_STATE_FAILED;
	}
}

/* ===================== flux 辨识（反电势法）=====================
 * STEP 0: 施加 uq 驱动电机转动（theta 跟随实时电角度，不替换回调）
 * STEP 1: 等待稳速（CALIB_CFG_L2_FLUX_SPIN_TICKS）
 * STEP 2: 多次采样 flux_k = (uq - R*iq) / omega_e 取平均
 * STEP 3: flux = avg，校验后写入，完成
 *
 * 公式：稳速时 uq ≈ R*iq + omega_e*flux（忽略 Lq*diq/dt 稳态为 0）
 *   omega_e = omega_m * pole_pairs
 *   omega_m = m->motor_param.slide_rad_s
 *
 * 注意：本子模式不调 calib_hw_enter（需保留实时电角度回调让电机转动），
 *       直接用 calib_hw_apply_voltage 传 m->motor_param.ele_radian 作为 theta。
 * ============================================================ */
static calib_state_e poll_flux_linkage(void)
{
	const calib_io_t *io = calib_mgr_get_io();
	dev_motor_t *m = io->motor;

	switch (s_l2.step.cur)
	{
		case 0:                              /* 初始化会话（仅记录 motor 指针，不替换回调）*/
			s_l2.session.motor = m;
			s_l2.session.orig_ele_cb = NULL; /* 标记不替换 */
			s_l2.session.forced_ele_angle = 0.0f;
			s_l2.test_voltage = CALIB_CFG_L2_FLUX_SPIN_VOLTAGE_V;
			calib_step_next(&s_l2.step, 1);
			return CALIB_STATE_RUNNING;

		case 1:                                      /* 驱动转动，等待稳速 */
		{
			float theta = m->motor_param.ele_radian; /* 跟随实时电角度 */
			calib_hw_apply_voltage(&s_l2.session, 0.0f, s_l2.test_voltage, theta);
			if (calib_step_wait(&s_l2.step, CALIB_CFG_L2_FLUX_SPIN_TICKS))
				return CALIB_STATE_RUNNING;
			calib_step_next(&s_l2.step, 2);
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
			uint8_t pp = motor_param_get_pole_pairs(io->param);
			float omega_e = omega_m * (float)pp;
			/* omega_e 过小则跳过（电机未转起）*/
			if (isfinite(omega_e) && fabsf(omega_e) > 1.0f && isfinite(iq))
			{
				float R = motor_param_get_r(io->param);
				float flux_k = (s_l2.test_voltage - R * iq) / omega_e;
				if (isfinite(flux_k) && flux_k > 0.0f)
					calib_step_accumulate(&s_l2.step, flux_k);
			}
			if (s_l2.step.sample_cnt < CALIB_CFG_L2_FLUX_SAMPLE_COUNT)
				return CALIB_STATE_RUNNING;
			calib_step_next(&s_l2.step, 3);
			return CALIB_STATE_RUNNING;
		}

		case 3:                     /* 校验，写入，撤销电压 */
		{
			calib_hw_apply_zero(m); /* 直接置零 PWM（未替换回调，无需 exit 恢复）*/
			if (s_l2.step.sample_cnt == 0)
				return CALIB_STATE_FAILED;
			float flux = calib_step_average(&s_l2.step);
			if (!calib_validate_flux(flux))
				return CALIB_STATE_FAILED;
			motor_param_set_flux(io->param, flux);
			(void)motor_info_calib_submit_flux(flux);
			calib_mgr_mark_done(CALIB_LEVEL2_MOTOR, CALIB_L2_FLUX_LINKAGE);
			calib_step_reset(&s_l2.step);
			return CALIB_STATE_DONE;
		}

		default:
			calib_hw_apply_zero(m);
			return CALIB_STATE_FAILED;
	}
}

/* ===================== 相序识别 =====================
 * STEP 0: 施加 ud（theta=0）让转子对齐 d 轴
 * STEP 1: 等待对齐稳定
 * STEP 2: 记录起始机械角度，施加 ud（theta=120°）让转子转 1/3 电周期
 * STEP 3: 等待转动完成
 * STEP 4: 读结束角度，判定 delta 是否合理（应朝一个方向变化），完成
 *
 * 简化版：本版只验证"电机能响应 ud 阶跃并转动"，不持久化结果
 * （motor_param 无相序字段）。若 delta < 阈值视为电机未响应，FAILED。
 * ================================================== */
static calib_state_e poll_phase_seq(void)
{
	const calib_io_t *io = calib_mgr_get_io();
	dev_motor_t *m = io->motor;
	(void)io;

	switch (s_l2.step.cur)
	{
		case 0: /* 对齐 d 轴 */
			calib_hw_enter(&s_l2.session, m);
			calib_hw_apply_voltage(&s_l2.session, CALIB_CFG_L2_PHASE_SEQ_VOLTAGE_V, 0.0f, 0.0f);
			calib_step_next(&s_l2.step, 1);
			return CALIB_STATE_RUNNING;

		case 1: /* 等待对齐 */
			calib_hw_apply_voltage(&s_l2.session, CALIB_CFG_L2_PHASE_SEQ_VOLTAGE_V, 0.0f, 0.0f);
			if (calib_step_wait(&s_l2.step, CALIB_CFG_L2_PHASE_SEQ_ALIGN_TICKS))
				return CALIB_STATE_RUNNING;
			calib_step_next(&s_l2.step, 2);
			return CALIB_STATE_RUNNING;

		case 2: /* 记录起始角度，施加 120° 电角度步进 */
			s_l2.start_mech_deg = calib_hw_get_encoder_mech_angle(m);
			/* 120° 电角度 = 2*pi/3 rad */
			calib_hw_apply_voltage(&s_l2.session, CALIB_CFG_L2_PHASE_SEQ_VOLTAGE_V, 0.0f, 2.0F * 3.14159265F / 3.0F);
			calib_step_next(&s_l2.step, 3);
			return CALIB_STATE_RUNNING;

		case 3: /* 等待转动 */
			calib_hw_apply_voltage(&s_l2.session, CALIB_CFG_L2_PHASE_SEQ_VOLTAGE_V, 0.0f, 2.0F * 3.14159265F / 3.0F);
			if (calib_step_wait(&s_l2.step, CALIB_CFG_L2_PHASE_SEQ_STEP_TICKS))
				return CALIB_STATE_RUNNING;
			calib_step_next(&s_l2.step, 4);
			return CALIB_STATE_RUNNING;

		case 4: /* 判定 */
		{
			float end_deg = calib_hw_get_encoder_mech_angle(m);
			float delta = end_deg - s_l2.start_mech_deg;
			if (delta > 180.0f)
				delta -= 360.0f;
			else if (delta < -180.0f)
				delta += 360.0f;
			calib_hw_exit(&s_l2.session);
			/* 120° 电角度应对应 120/pole_pairs 机械角度，默认 7 极对 ≈ 17°
			 * 阈值取 5°（电机应明显转动）*/
			if (fabsf(delta) < 5.0f)
				return CALIB_STATE_FAILED; /* 电机未响应 */
			calib_mgr_mark_done(CALIB_LEVEL2_MOTOR, CALIB_L2_PHASE_SEQ);
			calib_step_reset(&s_l2.step);
			return CALIB_STATE_DONE;
		}

		default:
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
 *
 * 【与旧实现的本质区别】旧版读 motor_param.ele_radian 反推，而该量本身 =
 *   机械角 × 已配置极对数，是循环自证的派生量（且被 normalize_angle 折返破坏），
 *   最好情况只把配置值 7 还回来，实测因双重折返坍缩到 2。故彻底改为开环扫描。
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
			calib_hw_apply_voltage(&s_l2.session, CALIB_CFG_L2_POLE_PAIRS_VOLTAGE_V, 0.0f, s_l2.scan_ele_rad);
			calib_step_next(&s_l2.step, 1);
			return CALIB_STATE_RUNNING;

		case 1: /* 等待转子对齐到 d 轴 */
			calib_hw_apply_voltage(&s_l2.session, CALIB_CFG_L2_POLE_PAIRS_VOLTAGE_V, 0.0f, s_l2.scan_ele_rad);
			if (calib_step_wait(&s_l2.step, CALIB_CFG_L2_POLE_PAIRS_ALIGN_TICKS))
				return CALIB_STATE_RUNNING;
			calib_step_next(&s_l2.step, 2);
			return CALIB_STATE_RUNNING;

		case 2: /* 记录起始机械角，初始化连续累加器 */
			s_l2.start_mech_deg = calib_hw_get_encoder_mech_angle(m);
			s_l2.prev_mech_deg = s_l2.start_mech_deg;
			s_l2.accum_mech_deg = 0.0f;
			calib_step_next(&s_l2.step, 3);
			return CALIB_STATE_RUNNING;

		case 3: /* 开环扫描：递增强制电角度 + 机械角连续累加（unwrap）*/
		{
			/* 递增命令电角度（不折返，作为精确分子）*/
			s_l2.scan_ele_rad += CALIB_CFG_L2_POLE_PAIRS_DTHETA_RAD;
			calib_hw_apply_voltage(&s_l2.session, CALIB_CFG_L2_POLE_PAIRS_VOLTAGE_V, 0.0f, s_l2.scan_ele_rad);

			/* 机械角差分并去 ±360° 跳变，累加成连续量 */
			float mech_deg = calib_hw_get_encoder_mech_angle(m);
			float dmech = mech_deg - s_l2.prev_mech_deg;
			if (dmech > 180.0f)
				dmech -= 360.0f;
			else if (dmech < -180.0f)
				dmech += 360.0f;
			s_l2.accum_mech_deg += dmech;
			s_l2.prev_mech_deg = mech_deg;

			if (s_l2.scan_ele_rad < CALIB_CFG_L2_POLE_PAIRS_TARGET_RAD)
				return CALIB_STATE_RUNNING;
			calib_step_next(&s_l2.step, 4);
			return CALIB_STATE_RUNNING;
		}

		case 4: /* 计算极对数 */
		{
			calib_hw_exit(&s_l2.session); /* 撤销电压并恢复电角度回调 */

			float dmech_rad = fabsf(s_l2.accum_mech_deg) * (3.14159265F / 180.0F);
			if (dmech_rad < 0.1f)
				return CALIB_STATE_FAILED; /* 转子未跟随转动 */

			/* pole_pairs = 命令电角度总量 / 实测机械角总量 */
			float pp_f = CALIB_CFG_L2_POLE_PAIRS_TARGET_RAD / dmech_rad;
			uint8_t pp = (uint8_t)(pp_f + 0.5f); /* 四舍五入 */

			if (!calib_validate_pole_pairs(pp))
				return CALIB_STATE_FAILED;
			motor_param_set_pole_pairs(io->param, pp);
			(void)motor_info_calib_submit_pole_pairs(pp);
			calib_mgr_mark_done(CALIB_LEVEL2_MOTOR, CALIB_L2_POLE_PAIRS);
			calib_step_reset(&s_l2.step);
			return CALIB_STATE_DONE;
		}

		default:
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
