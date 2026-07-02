#ifndef __CALIB_STEP_H__
#define __CALIB_STEP_H__

#include <stdint.h>

/* ===================== 标定状态机骨架 =====================
 * 各 level 模块的 poll_*() 函数使用此统一框架管理步骤推进，
 * 保证风格一致、便于 code review 与后续维护。
 *
 * 典型用法：
 *   static calib_step_t s_step;
 *   switch (s_step.cur) {
 *       case 0:
 *           calib_hw_enter(...);
 *           calib_step_next(&s_step, 1);   // 进入下一步，清 tick
 *           return CALIB_STATE_RUNNING;
 *       case 1:
 *           if (calib_step_wait(&s_step, 20000))  // 等 20000 tick
 *               return CALIB_STATE_RUNNING;
 *           calib_step_next(&s_step, 2);
 *           return CALIB_STATE_RUNNING;
 *       case 2:
 *           ...
 *           calib_step_reset(&s_step);
 *           return CALIB_STATE_DONE;
 *   }
 */

typedef struct
{
	uint8_t cur;         /* 当前步骤号 */
	uint32_t tick;       /* 当前步骤已运行 tick 数 */
	uint32_t sample_cnt; /* 采样计数器 */
	float sample_sum;    /* 采样累加和（取平均用）*/
} calib_step_t;

/* 重置状态机到 step 0，清零所有计数器 */
static inline void calib_step_reset(calib_step_t *st)
{
	st->cur = 0;
	st->tick = 0;
	st->sample_cnt = 0;
	st->sample_sum = 0.0f;
}

/* 推进到下一步，清零 tick（sample_cnt/sample_sum 保留，跨步累加场景用）*/
static inline void calib_step_next(calib_step_t *st, uint8_t next)
{
	st->cur = next;
	st->tick = 0;
}

/* 等待指定 tick 数。返回 1=还在等, 0=已等够（调用方应推进下一步）*/
static inline int calib_step_wait(calib_step_t *st, uint32_t ticks)
{
	if (++st->tick < ticks)
		return 1;
	return 0;
}

/* 累加一次采样值，计数器自增 */
static inline void calib_step_accumulate(calib_step_t *st, float value)
{
	st->sample_sum += value;
	st->sample_cnt++;
}

/* 取采样平均值（需先确认 sample_cnt > 0）*/
static inline float calib_step_average(const calib_step_t *st)
{
	return st->sample_sum / (float)st->sample_cnt;
}

#endif /* __CALIB_STEP_H__ */
