/**
 * @file        motor_observer.h
 * @brief       Motor-control observation, live snapshot and high-speed trace
 * @details     This module is the only adapter allowed to map motor_loop internals
 *              to telemetry/plot channels. Control code produces state; protocol
 *              and UI consumers read snapshots or frozen trace chunks.
 */

#ifndef __MOTOR_OBSERVER_H__
#define __MOTOR_OBSERVER_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct motor_loop_s;

typedef enum
{
	MOTOR_OBS_ID_REF  = (1u << 0),
	MOTOR_OBS_ID      = (1u << 1),
	MOTOR_OBS_IQ_REF  = (1u << 2),
	MOTOR_OBS_IQ      = (1u << 3),
	MOTOR_OBS_VEL_REF = (1u << 4),
	MOTOR_OBS_VEL     = (1u << 5),
	MOTOR_OBS_POS_REF = (1u << 6),
	MOTOR_OBS_POS     = (1u << 7),
	MOTOR_OBS_ALL     = 0xFFu
} motor_observer_channel_e;

typedef struct
{
	uint32_t sequence;
	uint32_t control_tick;
	uint32_t valid_mask;
	uint8_t top_state;
	uint8_t ctrl_type;
	uint8_t reserved[2];
	float id_ref;
	float id;
	float iq_ref;
	float iq;
	float vel_ref;
	float vel;
	float pos_ref;
	float pos;
} motor_observer_snapshot_t;

/* Called once after motor_loop_isr(). Inactive trace cost is one state check. */
void motor_observer_on_control_isr(const struct motor_loop_s *loop);

/* Coherent, decimated live snapshot for telemetry, plots and PID observation. */
int motor_observer_snapshot_read(motor_observer_snapshot_t *out);

/* B7/B6/C9 trace backend. Channel bits are motor_observer_channel_e. */
#define MOTOR_OBSERVER_CAPTURE_MAX_SAMPLES 256u
#define MOTOR_OBSERVER_CAPTURE_CHANNELS 8u
int motor_observer_capture_start(uint32_t channel_mask, uint32_t rate_hz,
		uint16_t sample_count, float control_hz);
int motor_observer_capture_stop(void);
int motor_observer_capture_read(uint16_t offset, uint8_t count,
		uint8_t *out, uint16_t *out_len);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_OBSERVER_H__ */
