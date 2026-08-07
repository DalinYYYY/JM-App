/**
 * @file        motor_observer.c
 * @brief       Demand-driven motor live snapshot and continuous trace recorder
 */

#include "motor_observer.h"

#include <string.h>

#include "motor_loop.h"
#include "motor_loop_config.h"
#include "runtime_param.h"

#define MOTOR_OBSERVER_DEG_TO_RAD (0.01745329252f)
#define MOTOR_OBSERVER_CHANNELS_MAX 8u
#define TRACE_BUFFER_WORDS \
	(MOTOR_OBSERVER_TRACE_BUFFER_SAMPLES * MOTOR_OBSERVER_CHANNELS_MAX)

static motor_observer_snapshot_t s_live[2];
static volatile uint8_t s_live_index;
static volatile uint32_t s_live_sequence;
static uint16_t s_live_div_count;
static uint32_t s_control_tick;

static float s_trace[TRACE_BUFFER_WORDS];
static volatile uint16_t s_trace_read;
static volatile uint16_t s_trace_write;
static volatile uint16_t s_trace_count;
static volatile uint16_t s_trace_decim;
static volatile uint16_t s_trace_decim_count;
static volatile uint32_t s_trace_mask;
static volatile uint32_t s_trace_rate;
static volatile uint16_t s_trace_session;
static volatile uint32_t s_trace_sequence;
static volatile uint32_t s_trace_first_index;
static volatile uint32_t s_trace_overflow;
static volatile uint8_t s_trace_channels;
static volatile uint8_t s_trace_packet_samples;
static volatile uint8_t s_trace_flags;
static volatile uint8_t s_trace_state;
static uint8_t s_live_publish_pending;

static uint8_t observer_popcount(uint32_t mask)
{
	uint8_t count = 0u;
	while (mask != 0u)
	{
		count = (uint8_t)(count + (uint8_t)(mask & 1u));
		mask >>= 1;
	}
	return count;
}

static float observer_velocity_ref(const motor_loop_t *m)
{
	return (m->sys.motor.ref.ctrl_type == REF_CTRL_POSITION) ?
		m->cascade.vel_setpoint : m->sys.motor.ref.vel;
}

static void observer_wr_u16(uint8_t *p, uint16_t value)
{
	p[0] = (uint8_t)value;
	p[1] = (uint8_t)(value >> 8);
}

static void observer_wr_u32(uint8_t *p, uint32_t value)
{
	p[0] = (uint8_t)value;
	p[1] = (uint8_t)(value >> 8);
	p[2] = (uint8_t)(value >> 16);
	p[3] = (uint8_t)(value >> 24);
}

static void observer_publish_runtime(const motor_loop_t *m)
{
	motor_state_t *st = &usr.motor_state[M1];
	const foc_t *foc = &m->motor.foc;
	const motion_param_t *motion = &m->motor.motor_param;
	const multiturn_t *multiturn = &m->motor.multiturn;
	const motor_param_t *param = &usr.motor_param[M1];
	const motor_ref_t *ref = &m->sys.motor.ref;
	motor_observer_snapshot_t *snap;
	uint8_t next_index = (uint8_t)(s_live_index ^ 1u);
	uint32_t next_sequence = s_live_sequence + 1u;
	if (next_sequence == 0u)
		next_sequence = 1u;

	st->electrical.ia = foc->current.ia;
	st->electrical.ib = foc->current.ib;
	st->electrical.ic = foc->current.ic;
	st->electrical.i_alpha = foc->i_alphaBeta.alpha;
	st->electrical.i_beta = foc->i_alphaBeta.beta;
	st->electrical.id_meas = foc->i_dq.d;
	st->electrical.iq_meas = foc->i_dq.q;
	st->electrical.ud = foc->u_dq.d;
	st->electrical.uq = foc->u_dq.q;
	st->electrical.u_alpha = foc->u_alphaBeta.alpha;
	st->electrical.u_beta = foc->u_alphaBeta.beta;
	st->electrical.duty_a = foc->svpwm.ta;
	st->electrical.duty_b = foc->svpwm.tb;
	st->electrical.duty_c = foc->svpwm.tc;
	st->motion.mech_angle_rad = motion->mechanical_angle * MOTOR_OBSERVER_DEG_TO_RAD;
	st->motion.elec_angle_rad = motion->ele_radian;
	st->motion.single_turn_rad = multiturn->last_single_rad;
	st->motion.multiturn = multiturn->turns;
	st->motion.position_rad = multiturn->position;
	st->motion.velocity_rad_s = motion->rad_s;
	st->motion.velocity_filt = motion->slide_rad_s;
	st->motion.accel_rad_s2 = motion->acceleration;
	st->power.torque_est = foc->i_dq.q * param->motor_base.kt *
		param->gearbox_param.gear_ratio;
	st->power.power_mech_w = st->power.torque_est * motion->slide_rad_s;
	st->setpoint.pos_rad = ref->pos;
	st->setpoint.velocity_rad_s = observer_velocity_ref(m);
	st->setpoint.torque_nm = ref->torque;
	st->setpoint.current_id = m->out.id_ref;
	st->setpoint.current_iq = m->out.iq_ref;
	st->setpoint.voltage_ud = ref->ud;
	st->setpoint.voltage_uq = ref->voltage;
	st->setpoint.duty = ref->duty;
	st->setpoint.mit_kp = ref->kp;
	st->setpoint.mit_kd = ref->kd;
	st->setpoint.mit_tff = ref->torque_ff;

	snap = &s_live[next_index];
	snap->sequence = next_sequence;
	snap->control_tick = s_control_tick;
	snap->valid_mask = MOTOR_OBS_ALL;
	snap->top_state = (uint8_t)m->sys.top_state;
	snap->ctrl_type = (uint8_t)ref->ctrl_type;
	snap->reserved[0] = 0u;
	snap->reserved[1] = 0u;
	snap->id_ref = m->out.id_ref;
	snap->id = foc->i_dq.d;
	snap->iq_ref = m->out.iq_ref;
	snap->iq = foc->i_dq.q;
	snap->vel_ref = observer_velocity_ref(m);
	snap->vel = motion->rad_s;
	snap->pos_ref = ref->pos;
	snap->pos = m->sys.motor.fb.pos;
	__DMB();
	s_live_index = next_index;
	s_live_sequence = next_sequence;
}

static void observer_trace_write(const motor_loop_t *m)
{
	uint16_t write_index;
	float *dst;
	uint32_t mask;

	if (s_trace_state != 1u || m->sys.top_state != TOP_FSM_RUN)
		return;
	if (++s_trace_decim_count < s_trace_decim)
		return;
	s_trace_decim_count = 0u;
	write_index = s_trace_write;
	mask = s_trace_mask;
	if (s_trace_count >= MOTOR_OBSERVER_TRACE_BUFFER_SAMPLES)
	{
		s_trace_read = (uint16_t)((s_trace_read + 1u) % MOTOR_OBSERVER_TRACE_BUFFER_SAMPLES);
		s_trace_count = MOTOR_OBSERVER_TRACE_BUFFER_SAMPLES - 1u;
		s_trace_overflow++;
		s_trace_first_index++;
		s_trace_flags |= MOTOR_OBSERVER_TRACE_FLAG_OVERFLOW |
			MOTOR_OBSERVER_TRACE_FLAG_DISCONTINUITY;
	}
	dst = &s_trace[(uint32_t)write_index * s_trace_channels];
	if ((mask & MOTOR_OBS_ID_REF) != 0u)  *dst++ = m->out.id_ref;
	if ((mask & MOTOR_OBS_ID) != 0u)      *dst++ = m->motor.foc.i_dq.d;
	if ((mask & MOTOR_OBS_IQ_REF) != 0u)  *dst++ = m->out.iq_ref;
	if ((mask & MOTOR_OBS_IQ) != 0u)      *dst++ = m->motor.foc.i_dq.q;
	if ((mask & MOTOR_OBS_VEL_REF) != 0u) *dst++ = observer_velocity_ref(m);
	if ((mask & MOTOR_OBS_VEL) != 0u)     *dst++ = m->sys.motor.fb.vel;
	if ((mask & MOTOR_OBS_POS_REF) != 0u) *dst++ = m->sys.motor.ref.pos;
	if ((mask & MOTOR_OBS_POS) != 0u)     *dst++ = m->sys.motor.fb.pos;
	__DMB();
	s_trace_write = (uint16_t)((write_index + 1u) % MOTOR_OBSERVER_TRACE_BUFFER_SAMPLES);
	s_trace_count++;
}

void motor_observer_on_control_isr(const struct motor_loop_s *loop)
{
	const motor_loop_t *m = (const motor_loop_t *)loop;
	if (m == NULL)
		return;
	s_control_tick++;
	observer_trace_write(m);
	if (s_live_publish_pending)
	{
		s_live_publish_pending = 0u;
		observer_publish_runtime(m);
	}
	if (++s_live_div_count >= MOTOR_LOOP_POS_DIV)
	{
		s_live_div_count = 0u;
		s_live_publish_pending = 1u;
	}
}

int motor_observer_snapshot_read(motor_observer_snapshot_t *out)
{
	uint8_t retry;
	if (out == NULL || s_live_sequence == 0u)
		return -1;
	for (retry = 0u; retry < 3u; retry++)
	{
		uint32_t sequence_before = s_live_sequence;
		uint8_t index = s_live_index;
		__DMB();
		memcpy(out, &s_live[index], sizeof(*out));
		__DMB();
		if (sequence_before == s_live_sequence && out->sequence == sequence_before)
			return 0;
	}
	return -1;
}

int motor_observer_trace_start(uint32_t channel_mask, uint32_t rate_hz,
	uint8_t packet_samples, float control_hz, uint16_t session_id,
	motor_observer_trace_status_t *status)
{
	uint8_t channels;
	uint16_t div;
	uint32_t primask;
	if (rate_hz == 0u || packet_samples == 0u ||
		packet_samples > MOTOR_OBSERVER_TRACE_MAX_PACKET_SAMPLES ||
		control_hz <= 0.0f || (channel_mask & ~MOTOR_OBS_ALL) != 0u)
		return -1;
	channels = observer_popcount(channel_mask);
	if (channels == 0u || (float)rate_hz > control_hz || channels > MOTOR_OBSERVER_CHANNELS_MAX)
		return -1;
	div = (uint16_t)(control_hz / (float)rate_hz + 0.5f);
	if (div == 0u)
		div = 1u;
	primask = __get_PRIMASK();
	__disable_irq();
	s_trace_state = 0u;
	s_trace_read = 0u;
	s_trace_write = 0u;
	s_trace_count = 0u;
	s_trace_decim = div;
	s_trace_decim_count = 0u;
	s_trace_mask = channel_mask;
	s_trace_rate = (uint32_t)(control_hz / (float)div + 0.5f);
	s_trace_session = session_id;
	s_trace_sequence = 0u;
	s_trace_first_index = 0u;
	s_trace_overflow = 0u;
	s_trace_channels = channels;
	s_trace_packet_samples = packet_samples;
	s_trace_flags = 0u;
	__DMB();
	s_trace_state = 1u;
	__set_PRIMASK(primask);
	if (status != NULL)
	{
		status->session_id = session_id;
		status->channel_mask = channel_mask;
		status->actual_rate_hz = s_trace_rate;
		status->packet_samples = packet_samples;
		status->channels = channels;
		status->buffer_capacity = MOTOR_OBSERVER_TRACE_BUFFER_SAMPLES;
		status->overflow_count = 0u;
		status->running = 1u;
	}
	return 0;
}

int motor_observer_trace_stop(void)
{
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	s_trace_state = 0u;
	s_trace_count = 0u;
	s_trace_read = 0u;
	s_trace_write = 0u;
	__DMB();
	__set_PRIMASK(primask);
	return 0;
}

int motor_observer_trace_pop(uint8_t *out, uint16_t *out_len)
{
	uint8_t count;
	uint8_t i;
	uint8_t flags;
	uint8_t channels;
	uint16_t n = 0u;
	uint16_t read_index;
	uint16_t bytes;
	uint32_t seq;
	uint32_t first;
	uint32_t session;
	uint32_t mask;
	uint32_t rate;
	uint32_t overflow;
	uint32_t primask;
	if (out == NULL || out_len == NULL)
		return -1;

	/* 复制期间保持生产者暂停，避免缓冲区在通信任务读取时被覆盖。
	 * 批量上限为 8 点×8 通道，临界区固定且不包含协议发送。 */
	primask = __get_PRIMASK();
	__disable_irq();
	if (s_trace_state != 1u || s_trace_count < s_trace_packet_samples)
	{
		__set_PRIMASK(primask);
		return -1;
	}
	count = s_trace_packet_samples;
	channels = s_trace_channels;
	bytes = (uint16_t)count * (uint16_t)channels * 4u;
	if ((uint16_t)(25u + bytes) > MOTOR_OBSERVER_TRACE_PAYLOAD_MAX)
	{
		__set_PRIMASK(primask);
		return -1;
	}
	read_index = s_trace_read;
	session = s_trace_session;
	seq = s_trace_sequence++;
	first = s_trace_first_index;
	rate = s_trace_rate;
	mask = s_trace_mask;
	flags = s_trace_flags;
	overflow = s_trace_overflow;
	s_trace_flags = 0u;
	s_trace_read = (uint16_t)((read_index + count) % MOTOR_OBSERVER_TRACE_BUFFER_SAMPLES);
	s_trace_count = (uint16_t)(s_trace_count - count);
	s_trace_first_index += count;
	__DMB();

	observer_wr_u16(&out[n], (uint16_t)session); n += 2u;
	observer_wr_u32(&out[n], seq); n += 4u;
	observer_wr_u32(&out[n], first); n += 4u;
	observer_wr_u32(&out[n], rate); n += 4u;
	observer_wr_u32(&out[n], mask); n += 4u;
	out[n++] = count;
	out[n++] = channels;
	out[n++] = flags;
	observer_wr_u32(&out[n], overflow); n += 4u;
	for (i = 0u; i < count; i++)
	{
		memcpy(&out[n], &s_trace[(uint32_t)read_index * channels], bytes / count);
		n = (uint16_t)(n + bytes / count);
		read_index = (uint16_t)((read_index + 1u) % MOTOR_OBSERVER_TRACE_BUFFER_SAMPLES);
	}
	__set_PRIMASK(primask);
	*out_len = n;
	return 0;
}
