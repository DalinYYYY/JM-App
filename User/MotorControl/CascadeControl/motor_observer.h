/**
 * @file        motor_observer.h
 * @brief       电机控制的观测、实时快照与高速波形采集
 * @details     本模块是唯一允许将 motor_loop 内部映射到遥测/绘图通道的适配层。
 *              控制代码产生状态，协议与 UI 消费方读取快照或冻结的波形分片。
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

/* 在 motor_loop_isr() 之后调用一次；未激活波形时仅一次状态判断开销。 */
void motor_observer_on_control_isr(const struct motor_loop_s *loop);

/* 供遥测、绘图与 PID 观测读取的同步、降采样实时快照。 */
int motor_observer_snapshot_read(motor_observer_snapshot_t *out);

/* 连续波形(TRACE)后端；行数据按通道位顺序紧凑排列。 */
#ifndef MOTOR_OBSERVER_TRACE_BUFFER_SAMPLES
#define MOTOR_OBSERVER_TRACE_BUFFER_SAMPLES 512u
#endif

/* ---- 大缓冲 RAM 放置开关 ----
 * 1(默认): s_trace、s_trace_meta、s_sweep_*、s_live 约28KB 放入 CCM-SRAM
 *   (0x10000000; G474 32KB / F405 64KB, 仅 CPU 可访问, 本模块缓冲全部
 *   CPU 读写无 DMA), 主 SRAM 释放约28KB。要求链接 scatter 含 RW_CCMRAM
 *   region(匹配 .ccm_observer 段, 见各板 MDK-ARM 下 .sct)。
 * 0: 常规主 SRAM。无 CCM 或 CCM 另作他用的板子用此值。链接时会提示
 *   一条 L6314W(无 .ccm_observer 段匹配), 属预期, 表示缓冲已回主 SRAM。
 * 覆盖方式: Keil target Define 加 MOTOR_OBSERVER_USE_CCM_RAM=0, 或在
 * 包含本头文件之前定义。 */
#ifndef MOTOR_OBSERVER_USE_CCM_RAM
#define MOTOR_OBSERVER_USE_CCM_RAM 1
#endif

/* 扫频高速上传允许每包最多24样本(2通道227B载荷<=256B上限);
 * 普通 TRACE(0xB9) 上位机仍按 1~8 样本/包配置, 不受影响。 */
#define MOTOR_OBSERVER_TRACE_MAX_PACKET_SAMPLES 24u
#define MOTOR_OBSERVER_TRACE_PAYLOAD_MAX 256u
#define MOTOR_OBSERVER_TRACE_FLAG_OVERFLOW       0x01u
#define MOTOR_OBSERVER_TRACE_FLAG_DISCONTINUITY  0x02u
#define MOTOR_OBSERVER_TRACE_FLAG_LAST           0x04u
#define MOTOR_OBSERVER_TRACE_FLAG_SWEEP          0x08u
#define MOTOR_OBSERVER_TRACE_FLAG_POINT_START    0x10u
#define MOTOR_OBSERVER_TRACE_FLAG_POINT_END      0x20u

typedef struct
{
	uint16_t session_id;
	uint32_t channel_mask;
	uint32_t actual_rate_hz;
	uint8_t packet_samples;
	uint8_t channels;
	uint16_t buffer_capacity;
	uint32_t overflow_count;
	uint8_t running;
} motor_observer_trace_status_t;

int motor_observer_trace_start(uint32_t channel_mask, uint32_t rate_hz,
		uint8_t packet_samples, float control_hz, uint16_t session_id,
		motor_observer_trace_status_t *status);
/* 运行中动态调整采样率(扫频逐频点钳制写入速率, 防缓冲溢出)。 */
int motor_observer_trace_set_rate(uint32_t rate_hz, float control_hz);
int motor_observer_trace_stop(void);
/* 停止写入但保留缓存，最后一包置 LAST 后自动转 STOPPED。 */
int motor_observer_trace_finish(void);
int motor_observer_trace_is_busy(void);
int motor_observer_trace_pop(uint8_t *out, uint16_t *out_len);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_OBSERVER_H__ */
