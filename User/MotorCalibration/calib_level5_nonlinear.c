/**
 * @file calib_level5_nonlinear.c
 * @brief L5 非线性补偿（齿槽/摩擦/死区补偿/磁饱和）
 * @note L5.3 死区补偿标定已实现（三点最小二乘拟合 R 和 V_dt）。
 *       L5.1 齿槽转矩标定已实现（正反双程开环同步拖动扫描, 表存独立 Flash 区）。
 *       L5.2/4 仍为桩实现。
 */
#include <math.h>
#include <stddef.h>
#include <string.h>
#include "calib_types.h"
#include "calib_config.h"
#include "calib_config_runtime.h" /* 运行期派生参数 */
#include "calib_mgr.h"
#include "calib_hw.h"
#include "dev_motor.h"
#include "motor_param.h"
#include "calib_step.h"
#include "calib_validate.h"
#include "motor_info_calib.h"
#include "cogging_comp.h"
#if defined(USE_DEV_FLASH)
#include "motor_info.h"
#include "motor_info_storage.h"
#endif

/* ===================== 模块私有状态 ===================== */
static struct
{
	uint8_t submode;
	calib_step_t step;
	calib_hw_session_t session;
	/* 三点拟合：3 个 (id, ud) 数据点 */
	float id_samples[3];
	float ud_samples[3];
	uint8_t point_idx;
	float test_voltage;
	float sum_id;
	uint32_t sample_cnt; /* 采样计数须 uint32：目标 500 超 uint8 上限，回绕将致采样死循环 */

#if defined(USE_DEV_FLASH)
	/* ---- L5.1 齿槽扫描（COGGING_TABLE_N 点/方向独立累加, ~14KB ZI）---- */
	float cog_acc_f[COGGING_TABLE_N];    /* 正转 bin 电流累加 */
	float cog_acc_r[COGGING_TABLE_N];    /* 反转 bin 电流累加 */
	uint16_t cog_cnt_f[COGGING_TABLE_N]; /* 正转 bin 样本数 */
	uint16_t cog_cnt_r[COGGING_TABLE_N]; /* 反转 bin 样本数 */
	int16_t cog_table[COGGING_TABLE_N];  /* 合成表(mA) */
	float cog_theta;                     /* 强制电角度(rad, 连续递增/递减) */
	float cog_uq;                        /* 拖动电压(V) */
	float cog_mech_prev;                 /* 上拍机械角(deg, [0,360)) */
	float cog_travel;                    /* 窗口累计行程(deg, 带符号) */
	uint16_t cog_slip_ticks;             /* 失步连续计数 */
	uint8_t cog_active;                  /* 会话活跃标志(abort 恢复用) */
	uint8_t cog_dt_saved;                /* 死区补偿使能保存 */
	uint8_t cog_en_saved;                /* 齿槽补偿使能保存 */
	float cog_gain_saved;                /* 齿槽补偿增益保存 */
#endif
} s_l5;

/* ===================== L5.3 死区补偿标定（三点拟合）======================
 * 原理：ud = R×id + V_dt（V_dt 为死区等效压降，含体二极管/MOSFET-Rds/开关延迟）
 * 施加 3 个不同 ud 档位，稳态后测 id，最小二乘拟合斜率=R，截距=V_dt。
 *
 * STEP 0: 进入会话，施加低档电压 V1
 * STEP 1: 等待稳态
 * STEP 2: 采样 id 均值，切中档 V2
 * STEP 3: 等待稳态
 * STEP 4: 采样 id 均值，切高档 V3
 * STEP 5: 等待稳态
 * STEP 6: 采样 id 均值，三点拟合 R 和 V_dt，校验写入
 * ===================================================================== */
static calib_state_e poll_deadtime_comp(void)
{
	const calib_io_t *io = calib_mgr_get_io();
	dev_motor_t *m = io->motor;

	/* 三档测试电压：低/中/高，均产生正向 id */
	const float v_table[3] = {
		calib_cfg_l2_r_test_voltage_lo_v(),
		calib_cfg_l2_r_test_voltage_v() * 0.7f,
		calib_cfg_l2_r_test_voltage_v(),
	};

	switch (s_l5.step.cur)
	{
		case 0: /* 施加 V1 */
			calib_hw_enter(&s_l5.session, m);
			s_l5.point_idx = 0;
			s_l5.test_voltage = v_table[0];
			calib_hw_apply_voltage(&s_l5.session, s_l5.test_voltage, 0.0f, 0.0f);
			s_l5.sum_id = 0.0f;
			s_l5.sample_cnt = 0;
			calib_mgr_set_step(0);
			calib_step_next(&s_l5.step, 1);
			calib_mgr_set_step(1);
			return CALIB_STATE_RUNNING;

		case 1: /* 等待稳态 */
		case 3: /* 等待稳态 */
		case 5: /* 等待稳态 */
			calib_hw_apply_voltage(&s_l5.session, s_l5.test_voltage, 0.0f, 0.0f);
			if (calib_step_wait(&s_l5.step, calib_cfg_l2_r_test_ticks()))
				return CALIB_STATE_RUNNING;
			/* 进入采样步 */
			s_l5.sum_id = 0.0f;
			s_l5.sample_cnt = 0;
			calib_step_next(&s_l5.step, s_l5.step.cur + 1);
			calib_mgr_set_step(s_l5.step.cur);
			return CALIB_STATE_RUNNING;

		case 2: /* 采样点 0 */
		case 4: /* 采样点 1 */
		case 6: /* 采样点 2 */
		{
			calib_hw_apply_voltage(&s_l5.session, s_l5.test_voltage, 0.0f, 0.0f);
			m->foc.clarke(&m->foc);
			m->foc.park(&m->foc);
			float id = m->foc.i_dq.d;
			if (!isfinite(id) || fabsf(id) < 0.001f)
			{
				calib_mgr_set_fail_reason(CALIB_FAIL_SAMPLE_ABNORMAL);
				calib_hw_exit(&s_l5.session);
				return CALIB_STATE_FAILED;
			}
			s_l5.sum_id += id;
			s_l5.sample_cnt++;
			if (s_l5.sample_cnt < calib_cfg_l2_r_sample_count())
				return CALIB_STATE_RUNNING;

			/* 记录 (id, ud) 数据点 */
			uint8_t idx = (uint8_t)((s_l5.step.cur - 2) / 2); /* case 2→0, 4→1, 6→2 */
			s_l5.id_samples[idx] = s_l5.sum_id / (float)s_l5.sample_cnt;
			s_l5.ud_samples[idx] = s_l5.test_voltage;

			/* 切下一档或进入拟合 */
			if (idx < 2)
			{
				s_l5.point_idx = (uint8_t)(idx + 1);
				s_l5.test_voltage = v_table[s_l5.point_idx];
				calib_hw_apply_voltage(&s_l5.session, s_l5.test_voltage, 0.0f, 0.0f);
				calib_step_next(&s_l5.step, s_l5.step.cur + 1);
				calib_mgr_set_step(s_l5.step.cur);
				return CALIB_STATE_RUNNING;
			}
			/* 三点采完，进入拟合 */
			calib_step_next(&s_l5.step, 7);
			calib_mgr_set_step(7);
			return CALIB_STATE_RUNNING;
		}

		case 7: /* 三点最小二乘拟合 */
		{
			calib_hw_exit(&s_l5.session);
			/* 最小二乘: ud = R*id + V_dt
			 * R = (N×Σ(id×ud) - Σid×Σud) / (N×Σ(id²) - (Σid)²)
			 * V_dt = (Σud - R×Σid) / N */
			float s_id = 0, s_ud = 0, s_id2 = 0, s_id_ud = 0;
			for (int i = 0; i < 3; i++)
			{
				s_id += s_l5.id_samples[i];
				s_ud += s_l5.ud_samples[i];
				s_id2 += s_l5.id_samples[i] * s_l5.id_samples[i];
				s_id_ud += s_l5.id_samples[i] * s_l5.ud_samples[i];
			}
			float denom = 3.0f * s_id2 - s_id * s_id;
			if (fabsf(denom) < 1e-9f)
			{
				calib_mgr_set_fail_reason(CALIB_FAIL_SAMPLE_ABNORMAL);
				return CALIB_STATE_FAILED;
			}
			float R_fit = (3.0f * s_id_ud - s_id * s_ud) / denom;
			float V_dt = (s_ud - R_fit * s_id) / 3.0f;

			if (!calib_validate_r(R_fit) || !isfinite(V_dt) || V_dt < 0.0f || V_dt > 2.0f)
			{
				calib_mgr_set_fail_reason(CALIB_FAIL_OUT_OF_RANGE);
				return CALIB_STATE_FAILED;
			}
			/* V_dt 写入 motor_param，供 foc_decoupling 运行时使用 */
			(io->param)->current_loop.deadtime_comp_v = V_dt;
			(io->param)->current_loop.deadtime_comp_enable = 1;
			/* R 也顺便更新（三点法精度高于两点法）*/
			(io->param)->motor_base.r = R_fit;
			(void)motor_info_calib_submit_r(R_fit);
			/* V_dt/enable 回写 motor_info(ID 70/88)，随 0xEA 固化持久化 */
			(void)motor_info_calib_submit_deadtime_comp(V_dt, 1u);
			calib_mgr_mark_done(CALIB_LEVEL5_NONLINEAR, CALIB_L5_DEADTIME_COMP);
			calib_step_reset(&s_l5.step);
			return CALIB_STATE_DONE;
		}

		default:
			calib_mgr_set_fail_reason(CALIB_FAIL_TIMEOUT);
			calib_hw_exit(&s_l5.session);
			return CALIB_STATE_FAILED;
	}
}

/* ===================== L5.1 齿槽转矩标定（开环同步拖动扫描）======================
 * 原理：calib_hw_enter 强制电角度会话下, theta 以恒定机械角速度匀速递增/递减
 * （开环同步拖动, 同 flux 标定机制）, 恒 uq 电压下实测 iq(θm) 即
 * [T_cog(θm) + Tf·sign(ω)]/Kt; 正反双程按机械角 bin 累加取平均自动抵消
 * 库仑摩擦, 去均值后即纯齿槽补偿电流表(mA)。
 * 失步保护: 实测机械角速度持续低于命令值 → FAILED。
 * 标定期间关闭齿槽补偿与死区补偿（避免补偿量污染测量）, 结束时恢复。
 * ===================================================================== */
#if defined(USE_DEV_FLASH)

/* 机械角环形差分(deg): 处理 [0,360) 界面 ±180 跳变 */
static float cog_mech_delta(float prev, float now)
{
	float d = now - prev;
	if (d > 180.0f)
		d -= 360.0f;
	else if (d < -180.0f)
		d += 360.0f;
	return d;
}

/* 失败清理: 撤销会话 + 恢复标定前补偿状态 */
static calib_state_e cog_fail(calib_fail_reason_e reason, const calib_io_t *io, dev_motor_t *m)
{
	calib_hw_exit(&s_l5.session);
	io->param->current_loop.deadtime_comp_enable = s_l5.cog_dt_saved;
	io->param->position_loop.cogging_comp_enable = s_l5.cog_en_saved;
	io->param->position_loop.cogging_comp_gain = s_l5.cog_gain_saved;
	s_l5.cog_active = 0u;
	(void)m;
	calib_mgr_set_fail_reason(reason);
	return CALIB_STATE_FAILED;
}

static calib_state_e poll_cogging(void)
{
	const calib_io_t *io = calib_mgr_get_io();
	dev_motor_t *m = io->motor;
	float dt = io->dt;
	float mech;
	float d;
	float w_cmd;

	switch (s_l5.step.cur)
	{
		case 0: /* 初始化: 关补偿/算拖动电压/清 bin/进强制角度会话 */
		{
			float r = io->param->motor_base.r;
			float flux = io->param->motor_base.flux;
			float pp = (float)io->param->motor_base.pole_pairs;
			float w_e;

			if (r <= 0.0f || flux <= 0.0f || pp < 1.0f)
				return cog_fail(CALIB_FAIL_SAMPLE_ABNORMAL, io, m);

			w_e = CALIB_CFG_L5_COG_SPEED_RAD_S * pp;
			s_l5.cog_uq = r * CALIB_CFG_L5_COG_HOLD_CURRENT * CALIB_CFG_L5_COG_VOLTAGE_MUL +
						  w_e * flux;
			if (s_l5.cog_uq > CALIB_CFG_MAX_VOLTAGE_MAG_V)
				s_l5.cog_uq = CALIB_CFG_MAX_VOLTAGE_MAG_V;

			/* 保存并关闭运行期补偿（防止补偿量污染 iq 测量）*/
			s_l5.cog_dt_saved = io->param->current_loop.deadtime_comp_enable;
			s_l5.cog_en_saved = io->param->position_loop.cogging_comp_enable;
			s_l5.cog_gain_saved = io->param->position_loop.cogging_comp_gain;
			io->param->current_loop.deadtime_comp_enable = 0;
			io->param->position_loop.cogging_comp_enable = 0;
			io->param->position_loop.cogging_comp_gain = 0.0f;

			memset(s_l5.cog_acc_f, 0, sizeof(s_l5.cog_acc_f));
			memset(s_l5.cog_acc_r, 0, sizeof(s_l5.cog_acc_r));
			memset(s_l5.cog_cnt_f, 0, sizeof(s_l5.cog_cnt_f));
			memset(s_l5.cog_cnt_r, 0, sizeof(s_l5.cog_cnt_r));
			s_l5.cog_theta = 0.0f;
			s_l5.cog_mech_prev = calib_hw_get_encoder_mech_angle(m);
			s_l5.cog_travel = 0.0f;
			s_l5.cog_slip_ticks = 0u;

			calib_hw_enter(&s_l5.session, m);
			s_l5.cog_active = 1u;
			calib_mgr_set_step(0);
			calib_step_next(&s_l5.step, 1);
			calib_mgr_set_step(1);
			return CALIB_STATE_RUNNING;
		}

		case 1: /* 正转过渡（跳过起步圈）*/
		case 3: /* 反转过渡（换向后重新同步）*/
		{
			int dir = (s_l5.step.cur == 1) ? 1 : -1;
			w_cmd = dir * CALIB_CFG_L5_COG_SPEED_RAD_S;
			s_l5.cog_theta += w_cmd * (float)io->param->motor_base.pole_pairs * dt;
			calib_hw_apply_voltage(&s_l5.session, 0.0f, s_l5.cog_uq, s_l5.cog_theta);
			m->foc.clarke(&m->foc);
			m->foc.park(&m->foc);

			mech = calib_hw_get_encoder_mech_angle(m);
			s_l5.cog_travel += cog_mech_delta(s_l5.cog_mech_prev, mech) * (float)dir;
			s_l5.cog_mech_prev = mech;

			/* 失步检测: 实际速度方向/大小持续偏离命令 */
			if (m->motor_param.slide_rad_s * w_cmd < CALIB_CFG_L5_COG_SLIP_RATIO * CALIB_CFG_L5_COG_SPEED_RAD_S)
			{
				if (++s_l5.cog_slip_ticks >= CALIB_CFG_L5_COG_SLIP_TICKS)
					return cog_fail(CALIB_FAIL_SAMPLE_ABNORMAL, io, m);
			}
			else
			{
				s_l5.cog_slip_ticks = 0u;
			}

			if (s_l5.cog_travel >= CALIB_CFG_L5_COG_ROUND_SKIP * 360.0f)
			{
				s_l5.cog_travel = 0.0f;
				s_l5.cog_slip_ticks = 0u;
				calib_step_next(&s_l5.step, s_l5.step.cur + 1);
				calib_mgr_set_step(s_l5.step.cur);
			}
			return CALIB_STATE_RUNNING;
		}

		case 2: /* 正转采集 */
		case 4: /* 反转采集 */
		{
			int dir = (s_l5.step.cur == 2) ? 1 : -1;
			uint16_t idx;
			w_cmd = dir * CALIB_CFG_L5_COG_SPEED_RAD_S;
			s_l5.cog_theta += w_cmd * (float)io->param->motor_base.pole_pairs * dt;
			calib_hw_apply_voltage(&s_l5.session, 0.0f, s_l5.cog_uq, s_l5.cog_theta);
			m->foc.clarke(&m->foc);
			m->foc.park(&m->foc);

			mech = calib_hw_get_encoder_mech_angle(m);
			d = cog_mech_delta(s_l5.cog_mech_prev, mech);
			s_l5.cog_travel += d * (float)dir;
			s_l5.cog_mech_prev = mech;

			/* 按机械角 bin 累加 iq（环形索引, float 定点无除法）*/
			{
				float u = mech * ((float)COGGING_TABLE_N / 360.0f);
				int i = (int)u;
				if (u < 0.0f)
					i -= 1;
				i %= (int)COGGING_TABLE_N;
				if (i < 0)
					i += (int)COGGING_TABLE_N;
				idx = (uint16_t)i;
			}

			if (dir > 0)
			{
				s_l5.cog_acc_f[idx] += m->foc.i_dq.q;
				s_l5.cog_cnt_f[idx]++;
			}
			else
			{
				s_l5.cog_acc_r[idx] += m->foc.i_dq.q;
				s_l5.cog_cnt_r[idx]++;
			}

			/* 失步检测 */
			if (m->motor_param.slide_rad_s * w_cmd < CALIB_CFG_L5_COG_SLIP_RATIO * CALIB_CFG_L5_COG_SPEED_RAD_S)
			{
				if (++s_l5.cog_slip_ticks >= CALIB_CFG_L5_COG_SLIP_TICKS)
					return cog_fail(CALIB_FAIL_SAMPLE_ABNORMAL, io, m);
			}
			else
			{
				s_l5.cog_slip_ticks = 0u;
			}

			if (s_l5.cog_travel >= CALIB_CFG_L5_COG_ROUND_SCAN * 360.0f)
			{
				s_l5.cog_travel = 0.0f;
				s_l5.cog_slip_ticks = 0u;
				calib_step_next(&s_l5.step, s_l5.step.cur + 1);
				calib_mgr_set_step(s_l5.step.cur);
			}
			return CALIB_STATE_RUNNING;
		}

		case 5: /* 合成: 正反平均抵消摩擦 → 去均值 → 平滑 → 量程校验 → 写表 */
		{
			float mean = 0.0f;
			float ma_max = 0.0f;
			uint16_t i;

			calib_hw_exit(&s_l5.session);
			io->param->current_loop.deadtime_comp_enable = s_l5.cog_dt_saved;
			s_l5.cog_active = 0u;

			for (i = 0u; i < COGGING_TABLE_N; i++)
			{
				float vf = (s_l5.cog_cnt_f[i] > 0u) ? s_l5.cog_acc_f[i] / (float)s_l5.cog_cnt_f[i] : 0.0f;
				float vr = (s_l5.cog_cnt_r[i] > 0u) ? s_l5.cog_acc_r[i] / (float)s_l5.cog_cnt_r[i] : 0.0f;
				float v = 0.5f * (vf + vr); /* 正反平均: Tf·sign(ω) 项抵消 */
				s_l5.cog_acc_f[i] = v;
				mean += v;
			}
			mean /= (float)COGGING_TABLE_N;

			/* 去直流(摩擦常量/零偏归 friction_comp 管) + 3 点环形平滑 */
			for (i = 0u; i < COGGING_TABLE_N; i++)
			{
				uint16_t ip = (uint16_t)((i + COGGING_TABLE_N - 1u) % COGGING_TABLE_N);
				uint16_t in = (uint16_t)((i + 1u) % COGGING_TABLE_N);
				float v = 0.25f * (s_l5.cog_acc_f[ip] + 2.0f * s_l5.cog_acc_f[i] + s_l5.cog_acc_f[in]);
				float ma = (v - mean) * 1000.0f; /* A → mA */
				float am = (ma >= 0.0f) ? ma : -ma;
				if (am > ma_max)
					ma_max = am;
				s_l5.cog_table[i] = (int16_t)((ma > 0.0f) ? (ma + 0.5f) : (ma - 0.5f));
			}

			if (ma_max > CALIB_CFG_L5_COG_TABLE_MA_MAX)
			{
				/* 幅值异常: 拖动失败/齿槽远超预期, 表不可信 */
				io->param->position_loop.cogging_comp_enable = s_l5.cog_en_saved;
				io->param->position_loop.cogging_comp_gain = s_l5.cog_gain_saved;
				calib_mgr_set_fail_reason(CALIB_FAIL_OUT_OF_RANGE);
				return CALIB_STATE_FAILED;
			}

			/* 表落盘走"ISR 登记+线程擦写": Flash 擦写期间该 Bank 不能取指,
			 * 且本 poll 运行于 10kHz 中断上下文, 直接擦写会与 Bank2 代码执行
			 * 冲突(RM0440 禁止)。enable/gain 先行生效(RAM), 表由 idle 线程
			 * 异步落盘, reload 后补偿自动生效。 */
			cogging_comp_request_write(s_l5.cog_table);
			io->param->position_loop.cogging_comp_enable = 1u;
			io->param->position_loop.cogging_comp_gain = 1.0f;
			{
				motor_info_t *info = motor_info_storage_get();
				if (info != NULL)
				{
					(void)motor_info_write_u32(info, 184u, 1u);   /* cogging_comp_enable */
					(void)motor_info_write_f32(info, 185u, 1.0f); /* cogging_comp_gain */
				}
			}

			calib_mgr_mark_done(CALIB_LEVEL5_NONLINEAR, CALIB_L5_COGGING);
			calib_step_reset(&s_l5.step);
			return CALIB_STATE_DONE;
		}

		default:
			return cog_fail(CALIB_FAIL_TIMEOUT, io, m);
	}
}

#else /* 无 Flash 存储: 保留桩 */

static calib_state_e poll_cogging(void)
{
	return CALIB_STATE_DONE;
}

#endif /* USE_DEV_FLASH */

static calib_state_e poll_friction(void)
{
	return CALIB_STATE_DONE;
}
static calib_state_e poll_saturation(void)
{
	return CALIB_STATE_DONE;
}

static bool calib_level5_start(uint8_t submode, motor_param_t *param, float dt)
{
	(void)param;
	(void)dt;
	s_l5.submode = submode;
	calib_step_reset(&s_l5.step);
	s_l5.session.motor = NULL;
	s_l5.session.orig_ele_cb = NULL;
	s_l5.session.forced_ele_angle = 0.0f;

	switch (submode)
	{
		case CALIB_L5_COGGING:
		case CALIB_L5_FRICTION:
		case CALIB_L5_DEADTIME_COMP:
		case CALIB_L5_SATURATION: return true;
		default: return false;
	}
}

static calib_state_e calib_level5_poll(void)
{
	switch (s_l5.submode)
	{
		case CALIB_L5_COGGING: return poll_cogging();
		case CALIB_L5_FRICTION: return poll_friction();
		case CALIB_L5_DEADTIME_COMP: return poll_deadtime_comp();
		case CALIB_L5_SATURATION: return poll_saturation();
		default: return CALIB_STATE_FAILED;
	}
}

static void calib_level5_abort(void)
{
	const calib_io_t *io = calib_mgr_get_io();
	if (io != NULL && io->motor != NULL)
		calib_hw_exit(&s_l5.session);
#if defined(USE_DEV_FLASH)
	/* 齿槽会话中途中止: 恢复标定前补偿状态（case 0 起保存, cog_active 门控）*/
	if (s_l5.cog_active != 0u && io != NULL && io->param != NULL)
	{
		io->param->current_loop.deadtime_comp_enable = s_l5.cog_dt_saved;
		io->param->position_loop.cogging_comp_enable = s_l5.cog_en_saved;
		io->param->position_loop.cogging_comp_gain = s_l5.cog_gain_saved;
		s_l5.cog_active = 0u;
	}
#endif
	calib_step_reset(&s_l5.step);
}

const calib_level_ops_t calib_level5_ops = {
	.start = calib_level5_start,
	.poll = calib_level5_poll,
	.abort = calib_level5_abort,
};
