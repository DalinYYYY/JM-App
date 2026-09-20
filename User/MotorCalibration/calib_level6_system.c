/**
 * @file calib_level6_system.c
 * @brief L6 负载系统级校准（惯量辨识 + PID 自整定；阻尼/回程间隙预留）
 * @note L6.1 双向恒流加速法，L6.4 复用 motor_pid_autotune 理论整定并加载生效。
 *       标定结果写入 motor_param_t 运行期 + motor_info_calib_submit_* 持久化，
 *       上位机 0xEA 落盘。CALIB 态电流环被旁路，L6.1 内置本地 q 轴电流 PI。
 */
#include <math.h>
#include "calib_types.h"
#include "calib_config.h"
#include "calib_config_runtime.h"
#include "calib_mgr.h"
#include "calib_hw.h"
#include "calib_step.h"
#include "calib_validate.h"
#include "motor_info_calib.h"
#include "dev_motor.h"
#include "motor_param.h"
#include "motor_pid_autotune.h"
#include "motor_pid_load.h"
#if defined(USE_DEV_FLASH)
#include "motor_info_storage.h"
#endif

#ifndef PI
#define PI 3.14159265358979f
#endif

/* ===================== 模块私有状态 ===================== */
/* 速度线性回归累加器：最小二乘斜率抗编码器量化噪声，
 * t 为相内整定期后的相对时间(s)，避免大数相减的精度损失 */
typedef struct
{
	uint32_t n;  /* 样本数 */
	float st;    /* Σt */
	float sv;    /* Σv (rad/s) */
	float stv;   /* Σt·v */
	float stt;   /* Σt² */
	float siq;   /* Σiq (A) */
} inertia_fit_t;

static struct
{
	uint8_t submode;
	calib_step_t step;          /* 统一状态机骨架 */
	calib_hw_session_t session; /* 仅持 motor 指针，不替换电角度回调(仿 flux) */
	float cur_kp, cur_ki;       /* 本地电流 PI 增益: ωc·Lq / ωc·R */
	float integ;                /* 电流 PI 积分项(V) */
	float iq_ref;               /* 电流给定(+I / -I / 0 / 制动) */
	float test_current;         /* 当前尝试的测试电流 I(A) */
	float i_init;               /* 初始测试电流(自适应上限)(A) */
	float kt;                   /* 转矩常数(N·m/A) */
	float dir1;                 /* 加速段运动方向(+1/-1)，减速段反向判据基准 */
	bool win_open;              /* 采样窗开启标志(运动触发) */
	uint32_t win_open_tick;     /* 开窗时刻 tick(t 相对基准) */
	uint32_t flight[2];         /* 各窗飞行时长 tick(自适应用) */
	uint8_t attempt;            /* 已用尝试次数 */
	calib_fail_reason_e adapt_reason; /* 尝试耗尽时的失败原因 */
	inertia_fit_t fit[2];       /* [0]=加速段 [1]=减速段 */
	float j_result;             /* 诊断: J 计算结果(失败时保留中间值) */
} s_l6;

/* ===================== L6.1 惯量辨识（双向恒流加速法）=====================
 * 原理：J·dv/dt = Kt·iq − T_load。恒流 ±I 往返加速，负载转矩（摩擦/
 *   磁滞制动等近似恒幅、方向随转速）在两段中反号，取和抵消：
 *     |a₊| = (Kt·I − T_f)/J ,  |a₋| = (Kt·I + T_f)/J
 *     J = Kt·(iq̄₊ − iq̄₋) / (|a₊| + |a₋|)
 *   用实测电流均值 iq̄ 而非指令值：电流 PI 跟踪误差/限幅不引入系统偏差。
 *   取绝对值配对使公式对编码器方向/FOC 方向翻转也成立。
 *
 *   采样窗由运动触发（|v| 进入 [v_a, v_b]），而非固定时间——小惯量轴系
 *   斜坡可能 <10ms 就走完，固定整定期会导致 0 样本（v1 实测教训）。
 *   斜坡过快时按飞行时长自适应缩小电流重试（300ms 目标飞行），
 *   起动困难时放大电流重试，最多 4 次尝试。
 *
 *   STEP 0:  绑定会话电机，进入加速段
 *   STEP 10: iq=+I 加速段，|v|∈[v_a,v_b] 开窗采样，达 v_b 关窗转减速段
 *   STEP 20: iq=−I 减速反向段，反向 |v|∈[v_a,v_b] 开窗采样，达 v_b 关窗
 *   STEP 30: LSQ 斜率 → J，校验写入；不达标按原因自适应重试
 *   STEP 40: 尝试间软制动停机，等待静止后进入下一尝试
 *
 * 硬件访问仿 flux 标定：不替换电角度回调（theta 跟随实时值，
 * park/inverse_park 同拍同角度），本模块直接出电压。
 * ================================================================= */
static void inertia_fit_reset(inertia_fit_t *f)
{
	f->n = 0;
	f->st = 0.0f;
	f->sv = 0.0f;
	f->stv = 0.0f;
	f->stt = 0.0f;
	f->siq = 0.0f;
}

/* 采样一个点：t 相对时间(s)，v 速度(rad/s)，iq 实测电流(A) */
static void inertia_fit_add(inertia_fit_t *f, float t, float v, float iq)
{
	f->n++;
	f->st += t;
	f->sv += v;
	f->stv += t * v;
	f->stt += t * t;
	f->siq += iq;
}

/* 最小二乘斜率 dv/dt (rad/s²)，样本不足返回 0 */
static float inertia_fit_slope(const inertia_fit_t *f)
{
	if (f->n < 2u)
		return 0.0f;
	float denom = (float)f->n * f->stt - f->st * f->st;
	if (denom <= 0.0f)
		return 0.0f;
	return ((float)f->n * f->stv - f->st * f->sv) / denom;
}

/* 相内平均电流(A) */
static float inertia_fit_avg_iq(const inertia_fit_t *f)
{
	return (f->n > 0u) ? (f->siq / (float)f->n) : 0.0f;
}

/* 每拍推进：本地 q 轴电流 PI → uq 电压（实时电角度换相）*/
static void inertia_drive(float dt, dev_motor_t *m)
{
	m->foc.clarke(&m->foc);
	m->foc.park(&m->foc);

	float err = s_l6.iq_ref - m->foc.i_dq.q;
	s_l6.integ += s_l6.cur_ki * err * dt;
	if (s_l6.integ > CALIB_CFG_L6_INERTIA_PI_INTEG_MAX_V)
		s_l6.integ = CALIB_CFG_L6_INERTIA_PI_INTEG_MAX_V;
	else if (s_l6.integ < -CALIB_CFG_L6_INERTIA_PI_INTEG_MAX_V)
		s_l6.integ = -CALIB_CFG_L6_INERTIA_PI_INTEG_MAX_V;

	float uq = s_l6.cur_kp * err + s_l6.integ;
	if (uq > CALIB_CFG_MAX_VOLTAGE_MAG_V)
		uq = CALIB_CFG_MAX_VOLTAGE_MAG_V;
	else if (uq < -CALIB_CFG_MAX_VOLTAGE_MAG_V)
		uq = -CALIB_CFG_MAX_VOLTAGE_MAG_V;

	calib_hw_apply_voltage(&s_l6.session, 0.0f, uq, m->motor_param.ele_radian);
}

/* 开窗：清累加器，记录开窗时刻与飞行起点 */
static void inertia_win_open(uint32_t tick, int idx)
{
	inertia_fit_reset(&s_l6.fit[idx]);
	s_l6.win_open = true;
	s_l6.win_open_tick = tick;
	s_l6.flight[idx] = 0;
}

/* 关窗：记录飞行时长 */
static void inertia_win_close(int idx)
{
	s_l6.win_open = false;
	s_l6.flight[idx] = s_l6.step.tick - s_l6.win_open_tick;
}

/* 窗内采样：t 相对开窗时刻(s)，v 速度(rad/s)，iq 实测电流(A) */
static void inertia_win_sample(float dt, dev_motor_t *m, int idx)
{
	float t = (float)(s_l6.step.tick - s_l6.win_open_tick) * dt;
	inertia_fit_add(&s_l6.fit[idx], t, m->motor_param.slide_rad_s, m->foc.i_dq.q);
}

/* 自适应缩小电流（斜坡过快）。t_meas=本次飞行/到达耗时 tick。
 * 已到下限仍过快 → 采样窗无法保证，直接失败。*/
static calib_state_e inertia_adapt_down(uint32_t t_meas)
{
	float i = s_l6.test_current;
	float ratio = (float)t_meas / (float)CALIB_CFG_L6_INERTIA_FLIGHT_TARGET_TICKS;
	if (ratio > 0.6f)
		ratio = 0.6f;
	if (ratio < 0.05f)
		ratio = 0.05f;
	i *= ratio;
	if (i < CALIB_CFG_L6_INERTIA_CURRENT_FLOOR_A)
		i = CALIB_CFG_L6_INERTIA_CURRENT_FLOOR_A;
	if (i >= s_l6.test_current * 0.95f)
	{
		const calib_io_t *io = calib_mgr_get_io();
		if (io != NULL && io->motor != NULL)
			calib_hw_apply_zero(io->motor);
		calib_step_reset(&s_l6.step);
		calib_mgr_set_fail_reason(CALIB_FAIL_SAMPLE_ABNORMAL);
		return CALIB_STATE_FAILED;
	}
	s_l6.test_current = i;
	s_l6.adapt_reason = CALIB_FAIL_SAMPLE_ABNORMAL;
	s_l6.iq_ref = 0.0f;
	calib_step_next(&s_l6.step, 40);
	calib_mgr_set_step(40);
	return CALIB_STATE_RUNNING;
}

/* 自适应放大电流（起动困难/斜率过弱）。已到初始上限 → 直接失败。*/
static calib_state_e inertia_adapt_up(void)
{
	if (s_l6.test_current >= s_l6.i_init * 0.95f)
	{
		const calib_io_t *io = calib_mgr_get_io();
		if (io != NULL && io->motor != NULL)
			calib_hw_apply_zero(io->motor);
		calib_step_reset(&s_l6.step);
		calib_mgr_set_fail_reason(CALIB_FAIL_MOTOR_STUCK);
		return CALIB_STATE_FAILED;
	}
	s_l6.test_current *= 1.5f;
	if (s_l6.test_current > s_l6.i_init)
		s_l6.test_current = s_l6.i_init;
	s_l6.adapt_reason = CALIB_FAIL_MOTOR_STUCK;
	s_l6.iq_ref = 0.0f;
	calib_step_next(&s_l6.step, 40);
	calib_mgr_set_step(40);
	return CALIB_STATE_RUNNING;
}

static calib_state_e poll_inertia(void)
{
	const calib_io_t *io = calib_mgr_get_io();
	dev_motor_t *m = io->motor;
	float v = m->motor_param.slide_rad_s;
	bool v_ok = isfinite(v);
	float v_abs = v_ok ? fabsf(v) : 0.0f;
	float v_lim = calib_cfg_l6_inertia_speed_limit_rad_s();
	float v_hard = io->param->motor_base.max_speed * CALIB_CFG_L6_INERTIA_VHARD_FRAC;

	switch (s_l6.step.cur)
	{
		case 0: /* 绑定会话电机，进入加速段 */
			s_l6.session.motor = m;
			s_l6.attempt = 0;
			s_l6.integ = 0.0f;
			s_l6.win_open = false;
			calib_mgr_set_step(0);
			calib_step_next(&s_l6.step, 10);
			calib_mgr_set_step(10);
			return CALIB_STATE_RUNNING;

		case 10: /* 加速段: iq=+I，运动触发开窗 [v_a,v_b] */
		{
			s_l6.step.tick++;
			s_l6.iq_ref = s_l6.test_current;
			inertia_drive(io->dt, m);

			if (v_ok && v_abs > v_hard)
				goto fail_over_speed;

			if (!s_l6.win_open)
			{
				if (v_ok && v_abs >= v_lim)
				{
					/* 未及开窗已达 v_b：斜坡过快（含 LEAD 前达速）*/
					return inertia_adapt_down(s_l6.step.tick);
				}
				if (s_l6.step.tick >= CALIB_CFG_L6_INERTIA_PRE_TIMEOUT_TICKS)
				{
					if (v_ok && v_abs >= CALIB_CFG_L6_INERTIA_VSTART_RAD_S)
					{
						/* 起动超时但已动：直接开窗（慢爬由窗口超时兜底）*/
						s_l6.dir1 = (v >= 0.0f) ? 1.0f : -1.0f;
						inertia_win_open(s_l6.step.tick, 0);
					}
					else
					{
						/* 起动困难：电流不足克服静摩擦/负载 */
						return inertia_adapt_up();
					}
				}
				else if (v_ok && v_abs >= CALIB_CFG_L6_INERTIA_VSTART_RAD_S
				         && s_l6.step.tick >= CALIB_CFG_L6_INERTIA_LEAD_TICKS)
				{
					s_l6.dir1 = (v >= 0.0f) ? 1.0f : -1.0f;
					inertia_win_open(s_l6.step.tick, 0);
				}
			}
			else
			{
				inertia_win_sample(io->dt, m, 0);
				if (v_ok && v_abs >= v_lim)
				{
					inertia_win_close(0);
					if (s_l6.fit[0].n < CALIB_CFG_L6_INERTIA_MIN_SAMPLES)
					{
						/* 窗内飞行过短（同 tick 开+关），无需做减速段 */
						return inertia_adapt_down(s_l6.flight[0] + 1u);
					}
					/* 转减速段 */
					s_l6.iq_ref = -s_l6.test_current;
					s_l6.step.tick = 0;
					calib_step_next(&s_l6.step, 20);
					calib_mgr_set_step(20);
				}
				else if (s_l6.step.tick - s_l6.win_open_tick
				         >= CALIB_CFG_L6_INERTIA_WIN_TIMEOUT_TICKS)
				{
					/* 窗内爬行未达 v_b：力矩偏弱 */
					return inertia_adapt_up();
				}
			}
			return CALIB_STATE_RUNNING;
		}

		case 20: /* 减速反向段: iq=-I，反向运动触发开窗 */
		{
			s_l6.step.tick++;
			inertia_drive(io->dt, m);

			if (v_ok && v_abs > v_hard)
				goto fail_over_speed;

			if (!s_l6.win_open)
			{
				if (v_ok && (v * s_l6.dir1 < 0.0f)
				    && v_abs >= CALIB_CFG_L6_INERTIA_VSTART_RAD_S)
					inertia_win_open(s_l6.step.tick, 1);
				else if (s_l6.step.tick >= CALIB_CFG_L6_INERTIA_PRE_TIMEOUT_TICKS)
				{
					/* 未能在超时内反向通过 v_a（负载驱动/卡死）*/
					return inertia_adapt_up();
				}
			}
			else
			{
				inertia_win_sample(io->dt, m, 1);
				if (v_ok && v_abs >= v_lim)
				{
					inertia_win_close(1);
					calib_step_next(&s_l6.step, 30);
					calib_mgr_set_step(30);
				}
				else if (s_l6.step.tick - s_l6.win_open_tick
				         >= CALIB_CFG_L6_INERTIA_WIN_TIMEOUT_TICKS)
				{
					return inertia_adapt_up();
				}
			}
			return CALIB_STATE_RUNNING;
		}

		case 30: /* 计算 J / 自适应重试 / 提交 */
		{
			calib_hw_apply_zero(m);
			calib_step_reset(&s_l6.step);

			float iq1 = inertia_fit_avg_iq(&s_l6.fit[0]);
			float iq2 = inertia_fit_avg_iq(&s_l6.fit[1]);

			/* 样本量：窗口过短 → 缩小电流重试 */
			if (s_l6.fit[0].n < CALIB_CFG_L6_INERTIA_MIN_SAMPLES
			    || s_l6.fit[1].n < CALIB_CFG_L6_INERTIA_MIN_SAMPLES)
			{
				uint32_t t_min = (s_l6.flight[0] < s_l6.flight[1]) ? s_l6.flight[0] : s_l6.flight[1];
				return inertia_adapt_down(t_min + 1u);
			}
			/* 电流跟踪：符号正确且幅值达给定的一半（否则电压饱和/采样异常）*/
			if (iq1 < s_l6.test_current * CALIB_CFG_L6_INERTIA_IQ_TRACK_RATIO
			    || iq2 > -s_l6.test_current * CALIB_CFG_L6_INERTIA_IQ_TRACK_RATIO)
			{
				calib_mgr_set_fail_reason(CALIB_FAIL_SAMPLE_ABNORMAL);
				return CALIB_STATE_FAILED;
			}

			float a1 = fabsf(inertia_fit_slope(&s_l6.fit[0]));
			float a2 = fabsf(inertia_fit_slope(&s_l6.fit[1]));
			/* 斜率过小：力矩几乎全被摩擦消耗 → 放大电流重试 */
			if (a1 < CALIB_CFG_L6_INERTIA_MIN_ACCEL_RAD_S2
			    || a2 < CALIB_CFG_L6_INERTIA_MIN_ACCEL_RAD_S2)
				return inertia_adapt_up();

			float j = s_l6.kt * (iq1 - iq2) / (a1 + a2);
			s_l6.j_result = j;
			if (!calib_validate_inertia(j))
			{
				calib_mgr_set_fail_reason(CALIB_FAIL_OUT_OF_RANGE);
				return CALIB_STATE_FAILED;
			}

			io->param->motor_base.inertia = j;
			(void)motor_info_calib_submit_inertia(j);
			calib_mgr_mark_done(CALIB_LEVEL6_SYSTEM, CALIB_L6_INERTIA);
			return CALIB_STATE_DONE;
		}

		case 40: /* 尝试间停机：软制动至近零 */
		{
			s_l6.step.tick++;
			/* 反向恒流制动，近零后撤销 */
			s_l6.iq_ref = (v_ok && v_abs > CALIB_CFG_L6_INERTIA_VSTOP_RAD_S)
			                  ? -((v >= 0.0f) ? 1.0f : -1.0f) * s_l6.test_current
			                  : 0.0f;
			inertia_drive(io->dt, m);

			if (v_ok && v_abs > v_hard)
				goto fail_over_speed;

			if ((v_ok && v_abs <= CALIB_CFG_L6_INERTIA_VSTOP_RAD_S)
			    || s_l6.step.tick >= CALIB_CFG_L6_INERTIA_SETTLE_TICKS)
			{
				if (s_l6.attempt + 1u >= CALIB_CFG_L6_INERTIA_MAX_ATTEMPTS)
				{
					calib_hw_apply_zero(m);
					calib_step_reset(&s_l6.step);
					calib_mgr_set_fail_reason(s_l6.adapt_reason);
					return CALIB_STATE_FAILED;
				}
				s_l6.attempt++;
				s_l6.integ = 0.0f;
				s_l6.win_open = false;
				s_l6.step.tick = 0;
				calib_step_next(&s_l6.step, 10);
				calib_mgr_set_step(10);
			}
			return CALIB_STATE_RUNNING;
		}

		default:
			calib_hw_apply_zero(m);
			calib_mgr_set_fail_reason(CALIB_FAIL_TIMEOUT);
			calib_step_reset(&s_l6.step);
			return CALIB_STATE_FAILED;
	}

fail_over_speed:
	calib_hw_apply_zero(m);
	calib_mgr_set_fail_reason(CALIB_FAIL_OVER_SPEED);
	calib_step_reset(&s_l6.step);
	return CALIB_STATE_FAILED;
}

/* L6.1 启动前置校验 + 参数初始化（calib_level6_start 调用）*/
static bool inertia_start(motor_param_t *param)
{
	/* 前置：Kt 有效（J 公式分子）*/
	float kt = param->motor_base.kt;
	if (!isfinite(kt) || kt < 1.0e-5f)
	{
		calib_mgr_set_fail_reason(CALIB_FAIL_DEP_NOT_MET);
		return false;
	}
	/* 前置：编码器零位已标定（FOC 转矩控制依赖电角度，
	 * 零值=未标定，与 motor_profile_apply_info 的零值语义一致）*/
	if (param->encoder_param.elec_angle_bias == 0.0f &&
		param->encoder_param.enc_offset == 0.0f)
	{
		calib_mgr_set_fail_reason(CALIB_FAIL_DEP_NOT_MET);
		return false;
	}
	/* 本地电流 PI 增益：零极点对消 Kp=ωc·Lq, Ki=ωc·R（Lq 无效回落 Ld）*/
	const calib_motor_ident_t *id = calib_motor_ident_get();
	float lq = id->lq;
	if (!isfinite(lq) || lq <= 0.0f)
		lq = id->ld;
	if (!isfinite(id->r) || id->r <= 0.0f || !isfinite(lq) || lq <= 0.0f)
	{
		calib_mgr_set_fail_reason(CALIB_FAIL_DEP_NOT_MET);
		return false;
	}
	float wc = 2.0f * PI * CALIB_CFG_L6_INERTIA_CURLOOP_BW_HZ;
	s_l6.cur_kp = wc * lq;
	s_l6.cur_ki = wc * id->r;
	s_l6.integ = 0.0f;
	s_l6.iq_ref = 0.0f;
	s_l6.kt = kt;
	s_l6.test_current = calib_cfg_l6_inertia_test_current_a();
	s_l6.i_init = s_l6.test_current;
	s_l6.dir1 = 1.0f;
	s_l6.win_open = false;
	s_l6.attempt = 0;
	s_l6.adapt_reason = CALIB_FAIL_NONE;
	s_l6.flight[0] = 0;
	s_l6.flight[1] = 0;
	s_l6.j_result = 0.0f;
	inertia_fit_reset(&s_l6.fit[0]);
	inertia_fit_reset(&s_l6.fit[1]);
	return true;
}

/* ===================== L6.2 阻尼（预留）===================== */
static calib_state_e poll_damping(void)
{
	calib_mgr_set_fail_reason(CALIB_FAIL_TIMEOUT);
	return CALIB_STATE_FAILED;
}

/* ===================== L6.3 回程间隙（预留）===================== */
static calib_state_e poll_backlash(void)
{
	calib_mgr_set_fail_reason(CALIB_FAIL_TIMEOUT);
	return CALIB_STATE_FAILED;
}

/* ===================== L6.4 PID 自整定 ======================
 * 复用 motor_pid_autotune 理论整定（速度环 Kp=J·ωc/Kt, Ki=J·ωc²/4Kt；
 * 位置环纯比例），整定值写入 motor_info->blocks.control 后经
 * motor_pid_reload 加载到运行期 motor_param_t。
 * 带宽传 0：autotune 内部回落 motor_info 配置值/模块默认值。*/
static calib_state_e poll_pid_autotune(void)
{
#if defined(USE_DEV_FLASH)
	motor_info_t *info = motor_info_storage_get();
	if (info == NULL)
	{
		calib_mgr_set_fail_reason(CALIB_FAIL_DEP_NOT_MET);
		return CALIB_STATE_FAILED;
	}

	/* ring_mask=0x06: 速度+位置环（电流环增益由 L2 电气参数派生，不在此重算）*/
	int rc = motor_pid_autotune_apply(info, 0x06, 0.0f, 0.0f, 0.0f);
	if (rc == -2)
	{
		/* J/Kt/R/Ld 未就绪：先完成 L6.1 惯量辨识（或预设电机参数）*/
		calib_mgr_set_fail_reason(CALIB_FAIL_DEP_NOT_MET);
		return CALIB_STATE_FAILED;
	}
	if (rc != 0)
	{
		calib_mgr_set_fail_reason(CALIB_FAIL_OUT_OF_RANGE);
		return CALIB_STATE_FAILED;
	}

	/* 惯量加速度前馈增益 aff = J/Kt（A/(rad/s²)）：与 PID 同源写入 Flash 控制段，
	 * 重载后经 motor_pid_load 生效。越界(>1，超大惯量小 Kt)时跳过，前馈保持原值。*/
	{
		float j = info->blocks.motor_calib.rotor_inertia;
		float kt = info->blocks.motor_calib.torque_constant;
		float gain = (kt > 0.0f) ? j / kt : 0.0f;
		if (isfinite(gain) && gain > 0.0f && gain <= 1.0f)
			info->blocks.control.aff = gain;
	}

	/* 整定值已在 info->blocks.control（与 0xE7 Flash 写入同字段），
	 * 切换来源为 FLASH 并加载。CALIB 态控制环未运行，无并发冲突。*/
	motor_pid_set_source(PID_RING_VELOCITY, PID_SOURCE_FLASH);
	motor_pid_set_source(PID_RING_POSITION, PID_SOURCE_FLASH);
	motor_pid_reload();

	calib_mgr_mark_done(CALIB_LEVEL6_SYSTEM, CALIB_L6_PID_AUTOTUNE);
	return CALIB_STATE_DONE;
#else
	/* 无 Flash 板无 motor_info 实例，autotune 无输入来源 */
	calib_mgr_set_fail_reason(CALIB_FAIL_DEP_NOT_MET);
	return CALIB_STATE_FAILED;
#endif
}

/* ===================== ops 接口 ===================== */
static bool calib_level6_start(uint8_t submode, motor_param_t *param, float dt)
{
	(void)dt;

	s_l6.submode = submode;
	calib_step_reset(&s_l6.step);
	s_l6.session.motor = NULL;
	s_l6.session.orig_ele_cb = NULL; /* 不替换电角度回调 */
	s_l6.session.forced_ele_angle = 0.0f;

	switch (submode)
	{
		case CALIB_L6_INERTIA:
			return inertia_start(param);
		case CALIB_L6_DAMPING:
		case CALIB_L6_BACKLASH:
			return true; /* 预留：poll 直接失败 */
		case CALIB_L6_PID_AUTOTUNE:
#if defined(USE_DEV_FLASH)
		{
			/* 启动即校验输入就绪（J/Kt/R/Ld），同步 NACK 优于异步失败 */
			motor_info_t *info = motor_info_storage_get();
			if (info != NULL)
			{
				float j = info->blocks.motor_calib.rotor_inertia;
				float kt = info->blocks.motor_calib.torque_constant;
				float r = info->blocks.motor_calib.phase_resistance;
				float ld = info->blocks.motor_calib.phase_inductance_d;
				if (isfinite(j) && j >= 1.0e-7f && isfinite(kt) && kt >= 1.0e-5f &&
					isfinite(r) && r >= 0.001f && isfinite(ld) && ld >= 1.0e-6f)
					return true;
			}
			calib_mgr_set_fail_reason(CALIB_FAIL_DEP_NOT_MET);
			return false;
		}
#else
		calib_mgr_set_fail_reason(CALIB_FAIL_DEP_NOT_MET);
		return false;
#endif
		default:
			return false;
	}
}

static calib_state_e calib_level6_poll(void)
{
	switch (s_l6.submode)
	{
		case CALIB_L6_INERTIA: return poll_inertia();
		case CALIB_L6_DAMPING: return poll_damping();
		case CALIB_L6_BACKLASH: return poll_backlash();
		case CALIB_L6_PID_AUTOTUNE: return poll_pid_autotune();
		default: return CALIB_STATE_FAILED;
	}
}

static void calib_level6_abort(void)
{
	const calib_io_t *io = calib_mgr_get_io();
	if (io != NULL && io->motor != NULL)
	{
		/* 会话未替换回调(orig=NULL)，exit 仅撤销 PWM 输出 */
		s_l6.session.motor = io->motor;
		calib_hw_exit(&s_l6.session);
	}
	calib_step_reset(&s_l6.step);
}

const calib_level_ops_t calib_level6_ops = {
	.start = calib_level6_start,
	.poll = calib_level6_poll,
	.abort = calib_level6_abort,
};
