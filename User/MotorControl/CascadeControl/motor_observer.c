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
#define MOTOR_OBSERVER_SWEEP_POINTS_MAX 1024u

/* 大缓冲 RAM 放置: 见 motor_observer.h 的 MOTOR_OBSERVER_USE_CCM_RAM。
 * AC5 自定义段 ZI 必须显式 zero_init, 否则生成 RW 初始化数据;
 * AC6 以 .bss. 前缀段名保证 ZI。 */
#if MOTOR_OBSERVER_USE_CCM_RAM
#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050u)
#define MOTOR_OBS_CCM_BSS __attribute__((section(".bss.ccm_observer")))
#else
#define MOTOR_OBS_CCM_BSS __attribute__((section(".ccm_observer"), zero_init))
#endif
#else
#define MOTOR_OBS_CCM_BSS
#endif

#define TRACE_STATE_STOPPED  0u
#define TRACE_STATE_RUNNING  1u
#define TRACE_STATE_DRAINING 2u
/* TRACE 包头尺寸: session(2)+seq(4)+first(4)+rate(4)+mask(4)+count(1)
 * +channels(1)+flags(1)+overflow(4)=25B; sweep 包追加
 * point(2)+phase_inc(4)+freq_mhz(4)=10B。 */
#define TRACE_PKT_HDR_BASE        25u
#define TRACE_PKT_HDR_SWEEP_EXTRA 10u
#define TRACE_PKT_HDR_MAX         (TRACE_PKT_HDR_BASE + TRACE_PKT_HDR_SWEEP_EXTRA)
#define TRACE_BUFFER_WORDS \
	(MOTOR_OBSERVER_TRACE_BUFFER_SAMPLES * MOTOR_OBSERVER_CHANNELS_MAX)

typedef struct
{
	uint16_t point;
	uint8_t flags;
	uint8_t reserved;
} observer_trace_meta_t;

/* 单包发送信息: 临界区内决策填充, 临界区外用于组包 */
typedef struct
{
	uint16_t session;
	uint32_t seq;
	uint32_t first;
	uint32_t rate;
	uint32_t mask;
	uint32_t overflow;
	uint8_t count;
	uint8_t channels;
	uint8_t flags;
	uint8_t sweep_packet;
	uint16_t sweep_point;
	uint32_t phase_inc;
	uint32_t freq_mhz;
} observer_trace_pkt_t;

/* 以下大缓冲(约28KB)按 MOTOR_OBSERVER_USE_CCM_RAM 放入 CCM-SRAM 或主 SRAM */
static motor_observer_snapshot_t s_live[2] MOTOR_OBS_CCM_BSS;
static volatile uint8_t s_live_index;
static volatile uint32_t s_live_sequence;
static uint16_t s_live_div_count;
static uint32_t s_control_tick;

static float s_trace[TRACE_BUFFER_WORDS] MOTOR_OBS_CCM_BSS;
static observer_trace_meta_t s_trace_meta[MOTOR_OBSERVER_TRACE_BUFFER_SAMPLES] MOTOR_OBS_CCM_BSS;
static uint32_t s_sweep_phase_inc[MOTOR_OBSERVER_SWEEP_POINTS_MAX] MOTOR_OBS_CCM_BSS;
static uint32_t s_sweep_freq_mhz[MOTOR_OBSERVER_SWEEP_POINTS_MAX] MOTOR_OBS_CCM_BSS;
/* 各频点实际 TRACE 采样率(采样率随频点切换时, 包头 rate 必须与
 * 该频点缓冲样本的写入速率一致, 不能读全局当前值)。 */
static uint16_t s_sweep_rate_hz[MOTOR_OBSERVER_SWEEP_POINTS_MAX] MOTOR_OBS_CCM_BSS;
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

/* 位置模式取位置环输出的速度设定，其余模式取系统参考速度 */
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

/* 将控制环实时量打包进双缓冲快照，供上位机无锁读取 */
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

/* Bode 会话: 标记频点起止边界, 并缓存该频点的协议参数与实际采样率 */
static void trace_meta_mark_sweep(const motor_ctrl_t *ctrl, uint16_t write_index,
	observer_trace_meta_t *meta)
{
	uint16_t point = ctrl->sweep_trace_point;
	meta->point = point;
	meta->flags = MOTOR_OBSERVER_TRACE_FLAG_SWEEP;
	if (point < MOTOR_OBSERVER_SWEEP_POINTS_MAX)
	{
		s_sweep_phase_inc[point] = ctrl->sweep_trace_phase_inc;
		s_sweep_freq_mhz[point] = ctrl->sweep_trace_freq_mhz;
		s_sweep_rate_hz[point] = (uint16_t)s_trace_rate;
	}
	if (s_trace_count == 0u)
	{
		meta->flags |= MOTOR_OBSERVER_TRACE_FLAG_POINT_START;
		return;
	}
	{
		uint16_t prev = (uint16_t)((write_index +
			MOTOR_OBSERVER_TRACE_BUFFER_SAMPLES - 1u) %
			MOTOR_OBSERVER_TRACE_BUFFER_SAMPLES);
		if ((s_trace_meta[prev].flags & MOTOR_OBSERVER_TRACE_FLAG_SWEEP) == 0u ||
			s_trace_meta[prev].point != point)
		{
			s_trace_meta[prev].flags |= MOTOR_OBSERVER_TRACE_FLAG_POINT_END;
			meta->flags |= MOTOR_OBSERVER_TRACE_FLAG_POINT_START;
		}
	}
}

/* 按通道掩码写入一拍示波 trace 数据(环形缓冲，溢出时丢弃最旧) */
static void observer_trace_write(const motor_loop_t *m)
{
	uint16_t write_index;
	float *dst;
	uint32_t mask;
	const motor_ctrl_t *ctrl = &m->sys.motor;
	observer_trace_meta_t *meta;

	if (s_trace_state != TRACE_STATE_RUNNING || m->sys.top_state != TOP_FSM_RUN)
		return;
	/* Bode 会话只采集 MEASURE 段；普通 TRACE 不受影响。 */
	if (ctrl->run_state == RUN_STATE_TEST_SWEEP_FREQ &&
		ctrl->sweep_trace_sample_enable == 0u)
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
	meta = &s_trace_meta[write_index];
	meta->point = 0u;
	meta->flags = 0u;
	meta->reserved = 0u;
	if (ctrl->run_state == RUN_STATE_TEST_SWEEP_FREQ &&
		ctrl->sweep_trace_sample_enable != 0u)
		trace_meta_mark_sweep(ctrl, write_index, meta);
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

/* 控制环 ISR 入口：每拍写 trace，按分频发布实时快照 */
void motor_observer_on_control_isr(const struct motor_loop_s *loop)
{
	const motor_loop_t *m = (const motor_loop_t *)loop;
	if (m == NULL)
		return;
	s_control_tick++;
	observer_trace_write(m);
	if (m->sys.motor.sweep_finish_pending != 0u)
	{
		motor_observer_trace_finish();
		((motor_loop_t *)m)->sys.motor.sweep_finish_pending = 0u;
	}
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

/* 读取最新快照：用序列号校验避免读写竞争读到不一致的半帧数据 */
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

/* 启动示波采集：校验参数后在关中断临界区内配置环形缓冲并置采集态 */
int motor_observer_trace_start(uint32_t channel_mask, uint32_t rate_hz,
	uint8_t packet_samples, float control_hz, uint16_t session_id,
	motor_observer_trace_status_t *status)
{
	uint8_t channels;
	uint16_t div;
	uint16_t bytes_per_sample;
	uint16_t max_packet_samples;
	uint32_t primask;
	if (rate_hz == 0u || packet_samples == 0u ||
		packet_samples > MOTOR_OBSERVER_TRACE_MAX_PACKET_SAMPLES ||
		control_hz <= 0.0f || (channel_mask & ~MOTOR_OBS_ALL) != 0u)
		return -1;
	channels = observer_popcount(channel_mask);
	if (channels == 0u || (float)rate_hz > control_hz || channels > MOTOR_OBSERVER_CHANNELS_MAX)
		return -1;
	/* 载荷上限防御: 高通道数 × 大包长组合会使组包时帧体超过
	 * PAYLOAD_MAX, pop 永久拒绝而 RUNNING 又要求凑满包 → 缓冲涨满
	 * 溢出、会话僵死。按通道数把包长钳到上限内(按最坏 sweep 包头
	 * 预留), 钳制值回传 status。 */
	bytes_per_sample = (uint16_t)channels * 4u;
	max_packet_samples = (uint16_t)((MOTOR_OBSERVER_TRACE_PAYLOAD_MAX -
		TRACE_PKT_HDR_MAX) / bytes_per_sample);
	if (max_packet_samples == 0u)
		return -1;
	if (packet_samples > max_packet_samples)
		packet_samples = (uint8_t)max_packet_samples;
	if (s_trace_state != TRACE_STATE_STOPPED)
		return -2;
	div = (uint16_t)(control_hz / (float)rate_hz + 0.5f);
	if (div == 0u)
		div = 1u;
	primask = __get_PRIMASK();
	__disable_irq();
	s_trace_state = TRACE_STATE_STOPPED;
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
	memset(s_trace_meta, 0, sizeof(s_trace_meta));
	memset(s_sweep_phase_inc, 0, sizeof(s_sweep_phase_inc));
	memset(s_sweep_freq_mhz, 0, sizeof(s_sweep_freq_mhz));
	memset(s_sweep_rate_hz, 0, sizeof(s_sweep_rate_hz));
	__DMB();
	s_trace_state = TRACE_STATE_RUNNING;
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

int motor_observer_trace_set_rate(uint32_t rate_hz, float control_hz)
{
	uint16_t div;
	uint32_t primask;
	if (rate_hz == 0u || control_hz <= 0.0f)
		return -1;
	primask = __get_PRIMASK();
	__disable_irq();
	if (s_trace_state != TRACE_STATE_RUNNING)
	{
		__set_PRIMASK(primask);
		return -1;
	}
	if ((float)rate_hz > control_hz)
		rate_hz = (uint32_t)control_hz;
	div = (uint16_t)(control_hz / (float)rate_hz + 0.5f);
	if (div == 0u)
		div = 1u;
	s_trace_decim = div;
	s_trace_decim_count = 0u;
	s_trace_rate = (uint32_t)(control_hz / (float)div + 0.5f);
	__DMB();
	__set_PRIMASK(primask);
	return 0;
}

int motor_observer_trace_stop(void)
{
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	s_trace_state = TRACE_STATE_STOPPED;
	s_trace_count = 0u;
	s_trace_read = 0u;
	s_trace_write = 0u;
	__DMB();
	__set_PRIMASK(primask);
	return 0;
}

int motor_observer_trace_finish(void)
{
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	if (s_trace_state == TRACE_STATE_RUNNING)
	{
		if (s_trace_count > 0u)
		{
			uint16_t last = (uint16_t)((s_trace_write +
				MOTOR_OBSERVER_TRACE_BUFFER_SAMPLES - 1u) %
				MOTOR_OBSERVER_TRACE_BUFFER_SAMPLES);
			if ((s_trace_meta[last].flags & MOTOR_OBSERVER_TRACE_FLAG_SWEEP) != 0u)
				s_trace_meta[last].flags |= MOTOR_OBSERVER_TRACE_FLAG_POINT_END;
			s_trace_state = TRACE_STATE_DRAINING;
		}
		else
			s_trace_state = TRACE_STATE_STOPPED;
	}
	__DMB();
	__set_PRIMASK(primask);
	return 0;
}

int motor_observer_trace_is_busy(void)
{
	return (s_trace_state != TRACE_STATE_STOPPED) ? 1 : 0;
}

/* 决策一个可发包: 确定样本数并填充 sweep 元数据。
 * 规则: Bode 包不跨频点(边界不足固定包长发部分包); RUNNING 态非 sweep
 * 须凑满整包、sweep 须见到频点结束标记; 帧长不得超出载荷上限。
 * 须在关中断临界区内调用。返回 0 成功, -1 本次无包可发。 */
static int trace_pick_packet(uint16_t read_index, uint16_t bytes_per_sample,
	observer_trace_pkt_t *pkt)
{
	uint16_t max_count = s_trace_packet_samples;
	uint16_t count;
	uint16_t i;

	if (max_count == 0u || (uint16_t)(TRACE_PKT_HDR_MAX +
			max_count * bytes_per_sample) > MOTOR_OBSERVER_TRACE_PAYLOAD_MAX)
		max_count = MOTOR_OBSERVER_TRACE_MAX_PACKET_SAMPLES;
	count = (s_trace_count < max_count) ? (uint16_t)s_trace_count : max_count;

	pkt->sweep_packet = 0u;
	pkt->sweep_point = 0u;
	pkt->phase_inc = 0u;
	pkt->freq_mhz = 0u;
	if ((s_trace_meta[read_index].flags & MOTOR_OBSERVER_TRACE_FLAG_SWEEP) != 0u)
	{
		pkt->sweep_packet = 1u;
		pkt->sweep_point = s_trace_meta[read_index].point;
		if (pkt->sweep_point < MOTOR_OBSERVER_SWEEP_POINTS_MAX)
		{
			pkt->phase_inc = s_sweep_phase_inc[pkt->sweep_point];
			pkt->freq_mhz = s_sweep_freq_mhz[pkt->sweep_point];
		}
		/* Bode 包不得跨越频点，边界不足固定包长时发送部分包。 */
		for (i = 1u; i < count; i++)
		{
			uint16_t idx = (uint16_t)((read_index + i) %
				MOTOR_OBSERVER_TRACE_BUFFER_SAMPLES);
			if ((s_trace_meta[idx].flags & MOTOR_OBSERVER_TRACE_FLAG_SWEEP) == 0u ||
				s_trace_meta[idx].point != pkt->sweep_point)
			{
				count = i;
				break;
			}
		}
	}
	if (s_trace_state == TRACE_STATE_RUNNING && !pkt->sweep_packet &&
		count < s_trace_packet_samples)
		return -1;
	if (s_trace_state == TRACE_STATE_RUNNING && pkt->sweep_packet &&
		count < max_count)
	{
		uint16_t last = (uint16_t)((read_index + count - 1u) %
			MOTOR_OBSERVER_TRACE_BUFFER_SAMPLES);
		if ((s_trace_meta[last].flags & MOTOR_OBSERVER_TRACE_FLAG_POINT_END) == 0u)
			return -1;
	}
	if ((uint16_t)(TRACE_PKT_HDR_BASE +
			(pkt->sweep_packet ? TRACE_PKT_HDR_SWEEP_EXTRA : 0u) +
			count * bytes_per_sample) > MOTOR_OBSERVER_TRACE_PAYLOAD_MAX)
		return -1;

	pkt->count = (uint8_t)count;
	return 0;
}

/* 打包协议包头(尺寸见 TRACE_PKT_HDR_*), 返回包头长度 */
static uint16_t trace_pack_header(uint8_t *out, const observer_trace_pkt_t *pkt)
{
	uint16_t n = 0u;
	observer_wr_u16(&out[n], pkt->session); n += 2u;
	observer_wr_u32(&out[n], pkt->seq); n += 4u;
	observer_wr_u32(&out[n], pkt->first); n += 4u;
	observer_wr_u32(&out[n], pkt->rate); n += 4u;
	observer_wr_u32(&out[n], pkt->mask); n += 4u;
	out[n++] = pkt->count;
	out[n++] = pkt->channels;
	out[n++] = pkt->flags;
	observer_wr_u32(&out[n], pkt->overflow); n += 4u;
	if (pkt->sweep_packet)
	{
		observer_wr_u16(&out[n], pkt->sweep_point); n += 2u;
		observer_wr_u32(&out[n], pkt->phase_inc); n += 4u;
		observer_wr_u32(&out[n], pkt->freq_mhz); n += 4u;
	}
	return n;
}

/* 从环形缓冲取出一包采集数据并打包成协议帧(含包头元数据)。
 * 临界区内完成边界决策与读指针推进; 组包拷贝在临界区外执行,
 * 读区间已出队, ISR 溢出覆盖前有充足时间差。 */
int motor_observer_trace_pop(uint8_t *out, uint16_t *out_len)
{
	observer_trace_pkt_t pkt;
	uint16_t read_index;
	uint16_t bytes_per_sample;
	uint16_t n;
	uint8_t i;
	uint32_t primask;
	if (out == NULL || out_len == NULL)
		return -1;

	primask = __get_PRIMASK();
	__disable_irq();
	if ((s_trace_state != TRACE_STATE_RUNNING && s_trace_state != TRACE_STATE_DRAINING) ||
		s_trace_count == 0u)
	{
		if (s_trace_state == TRACE_STATE_DRAINING && s_trace_count == 0u)
			s_trace_state = TRACE_STATE_STOPPED;
		__set_PRIMASK(primask);
		return -1;
	}

	read_index = s_trace_read;
	pkt.channels = s_trace_channels;
	bytes_per_sample = (uint16_t)pkt.channels * 4u;
	if (trace_pick_packet(read_index, bytes_per_sample, &pkt) != 0)
	{
		__set_PRIMASK(primask);
		return -1;
	}

	/* 快照包头元数据(含 sweep 边界与 DRAINING 末包标记), 再推进读指针 */
	pkt.session = s_trace_session;
	pkt.seq = s_trace_sequence++;
	pkt.first = s_trace_first_index;
	pkt.rate = s_trace_rate;
	/* sweep 包采样率取该频点样本写入时的记录值, 与样本严格对应 */
	if (pkt.sweep_packet && pkt.sweep_point < MOTOR_OBSERVER_SWEEP_POINTS_MAX &&
		s_sweep_rate_hz[pkt.sweep_point] != 0u)
		pkt.rate = s_sweep_rate_hz[pkt.sweep_point];
	pkt.mask = s_trace_mask;
	pkt.flags = s_trace_flags;
	pkt.overflow = s_trace_overflow;
	if (pkt.sweep_packet)
	{
		uint16_t last = (uint16_t)((read_index + pkt.count - 1u) %
			MOTOR_OBSERVER_TRACE_BUFFER_SAMPLES);
		pkt.flags |= (uint8_t)(s_trace_meta[read_index].flags &
			(MOTOR_OBSERVER_TRACE_FLAG_SWEEP | MOTOR_OBSERVER_TRACE_FLAG_POINT_START));
		pkt.flags |= (uint8_t)(s_trace_meta[last].flags &
			MOTOR_OBSERVER_TRACE_FLAG_POINT_END);
	}
	if (s_trace_state == TRACE_STATE_DRAINING && pkt.count == s_trace_count)
		pkt.flags |= MOTOR_OBSERVER_TRACE_FLAG_LAST;
	s_trace_flags = 0u;
	s_trace_read = (uint16_t)((read_index + pkt.count) %
		MOTOR_OBSERVER_TRACE_BUFFER_SAMPLES);
	s_trace_count = (uint16_t)(s_trace_count - pkt.count);
	s_trace_first_index += pkt.count;
	if (s_trace_state == TRACE_STATE_DRAINING && s_trace_count == 0u)
		s_trace_state = TRACE_STATE_STOPPED;
	__DMB();
	__set_PRIMASK(primask);

	n = trace_pack_header(out, &pkt);
	for (i = 0u; i < pkt.count; i++)
	{
		memcpy(&out[n], &s_trace[(uint32_t)read_index * pkt.channels],
			bytes_per_sample);
		n = (uint16_t)(n + bytes_per_sample);
		read_index = (uint16_t)((read_index + 1u) % MOTOR_OBSERVER_TRACE_BUFFER_SAMPLES);
	}
	*out_len = n;
	return 0;
}
