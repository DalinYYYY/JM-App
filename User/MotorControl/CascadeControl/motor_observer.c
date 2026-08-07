/**
 * @file        motor_observer.c
 * @brief       Demand-driven motor live snapshot and high-speed trace recorder
 */

#include "motor_observer.h"

#include <string.h>

#include "motor_loop.h"
#include "motor_loop_config.h"
#include "runtime_param.h"

#define MOTOR_OBSERVER_DEG_TO_RAD (0.01745329252f)
#define MOTOR_OBSERVER_CAPTURE_WORDS \
	(MOTOR_OBSERVER_CAPTURE_MAX_SAMPLES * MOTOR_OBSERVER_CAPTURE_CHANNELS)

/* Live data uses double buffering plus a sequence retry in task context. */
static motor_observer_snapshot_t s_live[2];
static volatile uint8_t s_live_index;
static volatile uint32_t s_live_sequence;
static uint16_t s_live_div_count;
static uint32_t s_control_tick;

/* Trace rows are packed by enabled channel count instead of a fixed 8-float stride. */
static float s_capture[MOTOR_OBSERVER_CAPTURE_WORDS];
static volatile uint16_t s_capture_write;
static volatile uint16_t s_capture_total;
static volatile uint16_t s_capture_decim;
static volatile uint16_t s_capture_decim_count;
static volatile uint32_t s_capture_mask;
static volatile uint32_t s_capture_rate;
static volatile uint8_t s_capture_channels;
static volatile uint8_t s_capture_state; /* 0=idle, 1=capturing, 2=ready */
static volatile uint32_t s_capture_generation;
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

static float observer_velocity_ref(const motor_loop_t *m)
{
	return (m->sys.motor.ref.ctrl_type == REF_CTRL_POSITION) ?
		m->cascade.vel_setpoint : m->sys.motor.ref.vel;
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

	/* Existing DataHub remains the low-rate public view for non-control telemetry. */
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

	/* Setpoint means the value actually applied to each controller, not raw host input. */
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

	/* The control pairs below are published atomically from the consumer's view. */
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

static void observer_capture_isr(const motor_loop_t *m)
{
	uint16_t write_index;
	uint32_t mask;
	float *dst;

	if (s_capture_state != 1u || m->sys.top_state != TOP_FSM_RUN)
		return;
	if (++s_capture_decim_count < s_capture_decim)
		return;
	s_capture_decim_count = 0u;
	write_index = s_capture_write;
	if (write_index >= s_capture_total)
	{
		s_capture_state = 2u;
		return;
	}

	mask = s_capture_mask;
	dst = &s_capture[(uint32_t)write_index * s_capture_channels];
	if ((mask & MOTOR_OBS_ID_REF) != 0u)  *dst++ = m->out.id_ref;
	if ((mask & MOTOR_OBS_ID) != 0u)      *dst++ = m->motor.foc.i_dq.d;
	if ((mask & MOTOR_OBS_IQ_REF) != 0u)  *dst++ = m->out.iq_ref;
	if ((mask & MOTOR_OBS_IQ) != 0u)      *dst++ = m->motor.foc.i_dq.q;
	if ((mask & MOTOR_OBS_VEL_REF) != 0u) *dst++ = observer_velocity_ref(m);
	if ((mask & MOTOR_OBS_VEL) != 0u)     *dst++ = m->sys.motor.fb.vel;
	if ((mask & MOTOR_OBS_POS_REF) != 0u) *dst++ = m->sys.motor.ref.pos;
	if ((mask & MOTOR_OBS_POS) != 0u)     *dst++ = m->sys.motor.fb.pos;
	__DMB();
	s_capture_write = (uint16_t)(write_index + 1u);
	if (s_capture_write >= s_capture_total)
		s_capture_state = 2u;
}

void motor_observer_on_control_isr(const struct motor_loop_s *loop)
{
	const motor_loop_t *m = (const motor_loop_t *)loop;
	if (m == NULL)
		return;
	s_control_tick++;
	observer_capture_isr(m);
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

int motor_observer_capture_start(uint32_t channel_mask, uint32_t rate_hz,
	uint16_t sample_count, float control_hz)
{
	uint16_t div;
	uint8_t channels;
	uint32_t primask;
	if (rate_hz == 0u || sample_count == 0u ||
		sample_count > MOTOR_OBSERVER_CAPTURE_MAX_SAMPLES || control_hz <= 0.0f)
		return -1;
	if ((channel_mask & ~MOTOR_OBS_ALL) != 0u)
		return -1;
	channels = observer_popcount(channel_mask);
	if (channels == 0u || (float)rate_hz > control_hz)
		return -1;
	div = (uint16_t)(control_hz / (float)rate_hz + 0.5f);
	if (div == 0u)
		div = 1u;

	primask = __get_PRIMASK();
	__disable_irq();
	if (s_capture_state == 1u)
	{
		__set_PRIMASK(primask);
		return -1;
	}
	s_capture_generation++;
	s_capture_mask = channel_mask;
	s_capture_rate = (uint32_t)(control_hz / (float)div + 0.5f);
	s_capture_total = sample_count;
	s_capture_write = 0u;
	s_capture_decim = div;
	s_capture_decim_count = 0u;
	s_capture_channels = channels;
	__DMB();
	s_capture_state = 1u;
	s_capture_generation++;
	__set_PRIMASK(primask);
	return 0;
}

int motor_observer_capture_stop(void)
{
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	if (s_capture_state == 1u)
	{
		s_capture_generation++;
		s_capture_state = 2u;
		s_capture_generation++;
	}
	__set_PRIMASK(primask);
	return 0;
}

int motor_observer_capture_read(uint16_t offset, uint8_t count,
	uint8_t *out, uint16_t *out_len)
{
	uint8_t max_count;
	uint8_t channels;
	uint8_t i;
	uint16_t available;
	uint16_t n = 0u;
	uint32_t mask;
	uint32_t rate;
	uint8_t state;
	uint32_t generation_before;
	uint32_t generation_after;

	if (out == NULL || out_len == NULL)
		return -1;
	generation_before = s_capture_generation;
	if ((generation_before & 1u) != 0u)
		return -1;
	__DMB();
	channels = s_capture_channels;
	available = s_capture_write;
	mask = s_capture_mask;
	rate = s_capture_rate;
	state = s_capture_state;
	__DMB();
	if ((generation_before != s_capture_generation) ||
		channels == 0u)
		return -1;
	if (offset > available)
		return -1;
	max_count = (uint8_t)((256u - 15u) / ((uint16_t)channels * 4u));
	if (count > max_count)
		count = max_count;
	if ((uint16_t)(offset + count) > available)
		count = (uint8_t)(available - offset);

	out[n++] = state;
	observer_wr_u32(&out[n], mask); n += 4u;
	observer_wr_u32(&out[n], rate); n += 4u;
	observer_wr_u16(&out[n], available); n += 2u;
	observer_wr_u16(&out[n], offset); n += 2u;
	out[n++] = count;
	out[n++] = channels;
	for (i = 0u; i < count; i++)
	{
		uint16_t bytes = (uint16_t)channels * 4u;
		const float *src = &s_capture[(uint32_t)(offset + i) * channels];
		memcpy(&out[n], src, bytes);
		n = (uint16_t)(n + bytes);
	}
	__DMB();
	generation_after = s_capture_generation;
	if (generation_before != generation_after ||
		(generation_after & 1u) != 0u)
		return -1;
	*out_len = n;
	return 0;
}
