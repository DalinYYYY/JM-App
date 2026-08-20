/**
 * @file motor_mode_test_sweep.c
 * @brief 逐点正弦扫频：线性/对数/1Hz步进频点、32位NCO和测量段标记
 */
#include "motor_mode.h"

#include <math.h>
#include <string.h>

#define MOTOR_SWEEP_MAX_POINTS       1024u
#define MOTOR_SWEEP_SETTLE_CYCLES     3u
#define MOTOR_SWEEP_MEASURE_CYCLES    8u
#define MOTOR_SWEEP_MEASURE_MAX_S     2.0f
#define MOTOR_SWEEP_TOTAL_MAX_S     600.0f
#define MOTOR_SWEEP_TWO_PI            6.2831853071795864769f
#define MOTOR_SWEEP_PHASE_SCALE       (MOTOR_SWEEP_TWO_PI / 4294967296.0f)
#define MOTOR_SWEEP_TRACE_SWEEP       0x08u
#define MOTOR_SWEEP_TRACE_POINT_START 0x10u
#define MOTOR_SWEEP_TRACE_POINT_END   0x20u

static uint32_t sweep_period_ticks(float control_hz, uint32_t freq_mhz)
{
	float freq = (float)freq_mhz * 0.001f;
	uint32_t ticks;
	if (freq <= 0.0f)
		return 1u;
	ticks = (uint32_t)(control_hz / freq + 0.5f);
	return (ticks == 0u) ? 1u : ticks;
}

static uint32_t sweep_measure_cycles(uint32_t freq_mhz)
{
	/* 自适应测量周期: 低频单周期(缩短单点耗时), 频率升高逐渐过渡到8周期。
	 * cycles = clamp(2*freq, 1, 8): 0.5Hz→1周期, 1Hz→2, 2Hz→4, 4Hz+→8,
	 * 各频点测量时长恒约 2s, 同步解调至少覆盖 1 个整周期。 */
	float freq = (float)freq_mhz * 0.001f;
	uint32_t cycles = (uint32_t)(MOTOR_SWEEP_MEASURE_MAX_S * freq + 0.5f);
	if (cycles < 1u)
		cycles = 1u;
	if (cycles > MOTOR_SWEEP_MEASURE_CYCLES)
		cycles = MOTOR_SWEEP_MEASURE_CYCLES;
	return cycles;
}

static void sweep_load_point(motor_ctrl_t *ctrl, uint16_t point)
{
	float control_hz = 1.0f / ctrl->dt;
	uint32_t period = sweep_period_ticks(control_hz,
		ctrl->sweep_freq_mhz_table[point]);
	ctrl->sweep_point = point;
	ctrl->sweep_phase_acc = 0u;
	ctrl->sweep_phase_inc = ctrl->sweep_phase_inc_table[point];
	ctrl->sweep_settle_ticks = MOTOR_SWEEP_SETTLE_CYCLES * period;
	ctrl->sweep_measure_ticks = sweep_measure_cycles(
		ctrl->sweep_freq_mhz_table[point]) * period;
	ctrl->sweep_state_ticks = 0u;
	ctrl->sweep_state = MOTOR_SWEEP_STATE_SETTLE;
}

static float sweep_amp_from_raw(uint8_t test_mode, uint16_t amp_raw)
{
	return (test_mode >= 3u) ? ((float)amp_raw * 0.01f) :
		((float)amp_raw * 0.001f);
}

static float sweep_bias_from_raw(uint8_t test_mode, int16_t bias_raw)
{
	return (test_mode >= 3u) ? ((float)bias_raw * 0.01f) :
		((float)bias_raw * 0.001f);
}

static int sweep_amp_valid(const motor_ctrl_t *ctrl, uint8_t test_mode,
	float amp)
{
	if (ctrl->param == NULL || amp <= 0.0f)
		return 0;
	if (test_mode == 0u)
		return amp <= ctrl->param->motor_base.peak_torque;
	if (test_mode == 1u || test_mode == 2u)
		return amp <= ctrl->param->motor_base.peak_current;
	/* mode 3~5(速度/位置参考)统一按最大转速数值限幅:
	 * 位置正弦幅值(rad)换算速度峰值 2*pi*f*A 才受真实约束,
	 * 数值上限只是粗防呆, 行程安全由用户保证 */
	if (test_mode >= 3u)
		return amp <= ctrl->param->motor_base.max_speed;
	return 0;
}

int motor_sweep_configure(motor_ctrl_t *ctrl, const motor_sweep_config_t *cfg,
	uint16_t *point_count, uint32_t *duration_x100)
{
	float control_hz;
	float f_start;
	float f_end;
	float amp;
	float bias;
	float limit;
	uint16_t count;
	uint16_t i;
	uint64_t total_ticks = 0u;

	if (ctrl == NULL || cfg == NULL || point_count == NULL ||
		duration_x100 == NULL || ctrl->param == NULL || ctrl->dt <= 0.0f)
		return -1;
	if (cfg->test_mode > 5u || cfg->sweep_mode > 2u || cfg->flags != 0u ||
		cfg->point_cfg == 0u || cfg->point_cfg > MOTOR_SWEEP_MAX_POINTS ||
		cfg->f_start_x10 == 0u || cfg->f_start_x10 > cfg->f_end_x10 ||
		cfg->amp_raw == 0u)
		return -1;

	control_hz = 1.0f / ctrl->dt;
	f_start = (float)cfg->f_start_x10 * 0.1f;
	f_end = (float)cfg->f_end_x10 * 0.1f;
	amp = sweep_amp_from_raw(cfg->test_mode, cfg->amp_raw);
	bias = sweep_bias_from_raw(cfg->test_mode, cfg->bias_raw);
	limit = (cfg->test_mode == 0u) ? ctrl->param->motor_base.peak_torque :
		((cfg->test_mode <= 2u) ? ctrl->param->motor_base.peak_current :
		ctrl->param->motor_base.max_speed);
	if (f_end > control_hz * 0.1f ||
		!sweep_amp_valid(ctrl, cfg->test_mode, amp) ||
		fabsf(bias) + amp > limit)
		return -1;

	if (f_start == f_end)
		count = 1u;
	else if (cfg->sweep_mode == 0u)
	{
		if (cfg->point_cfg < 2u)
			return -1;
		count = cfg->point_cfg;
	}
	else if (cfg->sweep_mode == 1u)
	{
		float decades = log10f(f_end / f_start);
		count = (uint16_t)ceilf(decades * (float)cfg->point_cfg) + 1u;
	}
	else
	{
		/* 1Hz固定步进: 频点 f_start, f_start+1, ..., f_end,
		 * 频点数由起止频率决定(point_cfg 不参与), 超上限拒绝。 */
		count = (uint16_t)ceilf(f_end - f_start) + 1u;
	}
	if (count == 0u || count > MOTOR_SWEEP_MAX_POINTS)
		return -1;

	memset(ctrl->sweep_phase_inc_table, 0, sizeof(ctrl->sweep_phase_inc_table));
	memset(ctrl->sweep_freq_mhz_table, 0, sizeof(ctrl->sweep_freq_mhz_table));
	for (i = 0u; i < count; i++)
	{
		float f;
		uint32_t phase_inc;
		uint32_t actual_mhz;
		uint32_t period;
		uint32_t measure_cycles;
		if (i == count - 1u)
			f = f_end;
		else if (cfg->sweep_mode == 0u)
			f = f_start + (f_end - f_start) * (float)i / (float)(count - 1u);
		else if (cfg->sweep_mode == 1u)
		{
			f = f_start * powf(10.0f, (float)i / (float)cfg->point_cfg);
			if (f > f_end)
				f = f_end;
		}
		else
			f = f_start + (float)i; /* 1Hz步进: 末点由上面的 f_end 钳制 */
		phase_inc = (uint32_t)((double)f * 4294967296.0 /
			(double)control_hz + 0.5);
		if (phase_inc == 0u)
			return -1;
		actual_mhz = (uint32_t)((double)phase_inc * (double)control_hz *
			1000.0 / 4294967296.0 + 0.5);
		if (actual_mhz == 0u)
			return -1;
		ctrl->sweep_phase_inc_table[i] = phase_inc;
		ctrl->sweep_freq_mhz_table[i] = actual_mhz;
		period = sweep_period_ticks(control_hz, actual_mhz);
		measure_cycles = sweep_measure_cycles(actual_mhz);
		total_ticks += (uint64_t)(MOTOR_SWEEP_SETTLE_CYCLES + measure_cycles) *
			(uint64_t)period;
	}
	if ((double)total_ticks / (double)control_hz > MOTOR_SWEEP_TOTAL_MAX_S)
		return -1;

	ctrl->sweep_test_mode = cfg->test_mode;
	ctrl->sweep_mode = cfg->sweep_mode;
	ctrl->sweep_session_id = cfg->session_id;
	ctrl->sweep_point_count = count;
	ctrl->sweep_amp = amp;
	ctrl->sweep_bias = bias;
	ctrl->sweep_total_ticks = (total_ticks > 0xFFFFFFFFu) ?
		0xFFFFFFFFu : (uint32_t)total_ticks;
	ctrl->sweep_trace_sample_enable = 0u;
	ctrl->sweep_trace_flags = 0u;
	ctrl->sweep_finish_pending = 0u;
	ctrl->sweep_active = 1u;
	sweep_load_point(ctrl, 0u);
	*point_count = count;
	*duration_x100 = (uint32_t)((double)total_ticks * 100.0 /
		(double)control_hz + 0.5);
	return 0;
}

void motor_sweep_abort(motor_ctrl_t *ctrl)
{
	if (ctrl == NULL || ctrl->sweep_active == 0u)
		return;
	ctrl->sweep_active = 0u;
	ctrl->sweep_trace_sample_enable = 0u;
	ctrl->sweep_trace_flags = 0u;
	ctrl->sweep_finish_pending = 1u;
	ctrl->sweep_state = MOTOR_SWEEP_STATE_ABORTED;
	ctrl->ref.ctrl_type = REF_CTRL_IDLE;
	ctrl->ref.torque = 0.0f;
	ctrl->ref.iq = 0.0f;
	ctrl->ref.vel = 0.0f;
	ctrl->ref.pos = 0.0f;
}

int motor_sweep_is_active(const motor_ctrl_t *ctrl)
{
	return (ctrl != NULL && ctrl->sweep_active != 0u) ? 1 : 0;
}

int motor_sweep_is_complete(const motor_ctrl_t *ctrl)
{
	return (ctrl != NULL && (ctrl->sweep_state == MOTOR_SWEEP_STATE_DONE ||
		ctrl->sweep_state == MOTOR_SWEEP_STATE_ABORTED)) ? 1 : 0;
}

void motor_mode_test_sweep_run(motor_ctrl_t *ctrl)
{
	motor_ref_t *ref = &ctrl->ref;
	float sine;
	uint8_t is_measure;

	ref->pos_profile = MOTOR_PID_PROFILE_POSITION;
	ref->vel_profile = MOTOR_PID_PROFILE_VELOCITY;
	ctrl->sweep_trace_sample_enable = 0u;
	ctrl->sweep_trace_flags = 0u;
	if (!ctrl->sweep_active || motor_sweep_is_complete(ctrl))
	{
		ref->ctrl_type = REF_CTRL_IDLE;
		ref->torque = 0.0f;
		ref->iq = 0.0f;
		ref->vel = 0.0f;
		ref->pos = 0.0f;
		return;
	}

	is_measure = (ctrl->sweep_state == MOTOR_SWEEP_STATE_MEASURE) ? 1u : 0u;
	sine = sinf((float)ctrl->sweep_phase_acc * MOTOR_SWEEP_PHASE_SCALE);
	ctrl->sweep_phase_acc += ctrl->sweep_phase_inc;
	if (ctrl->sweep_test_mode == 0u)
	{
		ref->ctrl_type = REF_CTRL_TORQUE;
		ref->torque = ctrl->sweep_bias + sine * ctrl->sweep_amp;
		ref->torque_ff = 0.0f;
	}
	else if (ctrl->sweep_test_mode == 1u || ctrl->sweep_test_mode == 2u)
	{
		ref->ctrl_type = REF_CTRL_CURRENT;
		ref->id = 0.0f;
		ref->iq = ctrl->sweep_bias + sine * ctrl->sweep_amp;
	}
	else if (ctrl->sweep_test_mode == 5u)
	{
		/* 方向二·位置环闭环验证: 位置给定端正弦激励, 全环闭合 */
		ref->ctrl_type = REF_CTRL_POSITION;
		ref->pos = ctrl->sweep_bias + sine * ctrl->sweep_amp;
		ref->torque_ff = 0.0f;
	}
	else
	{
		ref->ctrl_type = REF_CTRL_VELOCITY;
		ref->vel = ctrl->sweep_bias + sine * ctrl->sweep_amp;
		ref->torque_ff = 0.0f;
	}

	if (is_measure)
	{
		ctrl->sweep_trace_sample_enable = 1u;
		ctrl->sweep_trace_point = ctrl->sweep_point;
		ctrl->sweep_trace_phase_inc = ctrl->sweep_phase_inc;
		ctrl->sweep_trace_freq_mhz = ctrl->sweep_freq_mhz_table[ctrl->sweep_point];
		ctrl->sweep_trace_flags = MOTOR_SWEEP_TRACE_SWEEP;
		if (ctrl->sweep_state_ticks == 0u)
			ctrl->sweep_trace_flags |= MOTOR_SWEEP_TRACE_POINT_START;
		if (ctrl->sweep_state_ticks + 1u >= ctrl->sweep_measure_ticks)
			ctrl->sweep_trace_flags |= MOTOR_SWEEP_TRACE_POINT_END;
	}

	ctrl->sweep_state_ticks++;
	if (ctrl->sweep_state == MOTOR_SWEEP_STATE_SETTLE &&
		ctrl->sweep_state_ticks >= ctrl->sweep_settle_ticks)
	{
		ctrl->sweep_state = MOTOR_SWEEP_STATE_MEASURE;
		ctrl->sweep_state_ticks = 0u;
	}
	else if (ctrl->sweep_state == MOTOR_SWEEP_STATE_MEASURE &&
		ctrl->sweep_state_ticks >= ctrl->sweep_measure_ticks)
	{
		if (ctrl->sweep_point + 1u < ctrl->sweep_point_count)
			sweep_load_point(ctrl, (uint16_t)(ctrl->sweep_point + 1u));
		else
		{
			ctrl->sweep_active = 0u;
			ctrl->sweep_state = MOTOR_SWEEP_STATE_DONE;
			ctrl->sweep_finish_pending = 1u;
		}
	}
}
