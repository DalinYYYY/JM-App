/**
 * @file calib_level7_auto.c
 * @brief L7 自动化集成校准（一键全自动）
 * @note 真实实现：按顺序调用 L1→L6 的所有必要子模式。
 *       L7 维护自己的子标定序列状态机，不依赖 calib_mgr 的 active_ops
 *       （否则会与 calib_mgr 的单例 active_ops 冲突）。
 *       step 字段标识当前执行到第几步，通过 calib_status_t.step 上报。
 */
#include <stddef.h>
#include "calib_types.h"
#include "calib_mgr.h"
#include "motor_info_calib.h"

/* ===================== L7 错误处理策略 ===================== */
typedef enum
{
	CALIB_L7_STRATEGY_STOP = 0,   /* 失败即停止（默认，最安全）*/
	CALIB_L7_STRATEGY_SKIP,       /* 跳过失败步，继续下一步 */
	CALIB_L7_STRATEGY_RETRY_ONCE, /* 重试一次再跳过 */
} calib_l7_strategy_e;

/* 当前策略（可改为运行时配置）*/
#define CALIB_L7_STRATEGY CALIB_L7_STRATEGY_STOP

/* ===================== L7 一键标定内容配置 =====================
 * 通过下面 0/1 开关宏控制 L7 一键标定执行哪些子项。
 * 改 0 即跳过该子项, 无需动算法代码。
 *
 * 注意:
 *   1. 子项之间的依赖仍需保证顺序 (如 L2.4 Ld 依赖 L2.3 R, 需 R 在前)
 *   2. L7 全流程成功后会调用 motor_info_calib_mark_calibrated() 置位
 *      is_calibrated 标记, 下次上电即使用标定值
 *   3. 默认配置: 电机基础身份(L2) + 编码器标定(L3);
 *      L1 硬件底层与 L4/L5/L6 高级标定按需开启
 */
#define CALIB_L7_ENABLE_L1_ADC_OFFSET     0 /* L1.1 ADC 偏置标定 */
#define CALIB_L7_ENABLE_L1_ADC_GAIN       0 /* L1.2 ADC 增益标定 */
#define CALIB_L7_ENABLE_L1_CURRENT_SENSOR 0 /* L1.3 电流传感器标定 */
#define CALIB_L7_ENABLE_L2_PHASE_SEQ      1 /* L2.1 相序识别 */
#define CALIB_L7_ENABLE_L2_POLE_PAIRS     1 /* L2.2 极对数辨识 */
#define CALIB_L7_ENABLE_L2_RESISTANCE     1 /* L2.3 R 相电阻 (Ld/Lq/flux 前置) */
#define CALIB_L7_ENABLE_L2_INDUCTANCE_D   1 /* L2.4 Ld */
#define CALIB_L7_ENABLE_L2_INDUCTANCE_Q   1 /* L2.5 Lq */
#define CALIB_L7_ENABLE_L2_FLUX_LINKAGE   1 /* L2.6 flux (需电机转动) */
#define CALIB_L7_ENABLE_L3_ZERO_OFFSET    1 /* L3.1 编码器零位 */
#define CALIB_L7_ENABLE_L3_DIRECTION      1 /* L3.2 编码器方向 */
#define CALIB_L7_ENABLE_L4_KT             0 /* L4.1 力矩常数 */
#define CALIB_L7_ENABLE_L5_COGGING        0 /* L5.1 齿槽转矩 */
#define CALIB_L7_ENABLE_L5_FRICTION       0 /* L5.2 摩擦辨识 */
#define CALIB_L7_ENABLE_L6_INERTIA        0 /* L6.1 惯量辨识 */
#define CALIB_L7_ENABLE_L6_PID_AUTOTUNE   0 /* L6.2 PID 自整定 */

static motor_param_t *s_param;
static float s_dt;
static uint8_t s_step;        /* 当前执行步骤索引（0..L7_SEQ_LEN）*/
static uint8_t s_failed_step; /* 失败的步号（0xFF=无失败）*/
static uint8_t s_retry_count; /* 当前步重试计数 */

/* L7 内部子标定序列定义（由上面开关宏条件编译构造） */
static const struct
{
	uint8_t level;
	uint8_t submode;
} s_sequence[] = {
#if CALIB_L7_ENABLE_L1_ADC_OFFSET
	{CALIB_LEVEL1_DRIVER,    CALIB_L1_ADC_OFFSET    },
#endif
#if CALIB_L7_ENABLE_L1_ADC_GAIN
	{CALIB_LEVEL1_DRIVER,    CALIB_L1_ADC_GAIN      },
#endif
#if CALIB_L7_ENABLE_L1_CURRENT_SENSOR
	{CALIB_LEVEL1_DRIVER,    CALIB_L1_CURRENT_SENSOR},
#endif
#if CALIB_L7_ENABLE_L2_PHASE_SEQ
	{CALIB_LEVEL2_MOTOR,     CALIB_L2_PHASE_SEQ     },
#endif
#if CALIB_L7_ENABLE_L2_POLE_PAIRS
	{CALIB_LEVEL2_MOTOR,     CALIB_L2_POLE_PAIRS    },
#endif
#if CALIB_L7_ENABLE_L2_RESISTANCE
	{CALIB_LEVEL2_MOTOR,     CALIB_L2_RESISTANCE    }, /* R 先做，Ld/Lq 计算需用 R */
#endif
#if CALIB_L7_ENABLE_L2_INDUCTANCE_D
	{CALIB_LEVEL2_MOTOR,     CALIB_L2_INDUCTANCE_D  },
#endif
#if CALIB_L7_ENABLE_L2_INDUCTANCE_Q
	{CALIB_LEVEL2_MOTOR,     CALIB_L2_INDUCTANCE_Q  },
#endif
#if CALIB_L7_ENABLE_L2_FLUX_LINKAGE
	{CALIB_LEVEL2_MOTOR,     CALIB_L2_FLUX_LINKAGE  }, /* flux 需电机转动，放最后 */
#endif
#if CALIB_L7_ENABLE_L3_ZERO_OFFSET
	{CALIB_LEVEL3_ENCODER,   CALIB_L3_ZERO_OFFSET   },
#endif
#if CALIB_L7_ENABLE_L3_DIRECTION
	{CALIB_LEVEL3_ENCODER,   CALIB_L3_DIRECTION     },
#endif
#if CALIB_L7_ENABLE_L4_KT
	{CALIB_LEVEL4_TORQUE,    CALIB_L4_KT            },
#endif
#if CALIB_L7_ENABLE_L5_COGGING
	{CALIB_LEVEL5_NONLINEAR, CALIB_L5_COGGING       },
#endif
#if CALIB_L7_ENABLE_L5_FRICTION
	{CALIB_LEVEL5_NONLINEAR, CALIB_L5_FRICTION      },
#endif
#if CALIB_L7_ENABLE_L6_INERTIA
	{CALIB_LEVEL6_SYSTEM,    CALIB_L6_INERTIA       },
#endif
#if CALIB_L7_ENABLE_L6_PID_AUTOTUNE
	{CALIB_LEVEL6_SYSTEM,    CALIB_L6_PID_AUTOTUNE  },
#endif
};
#define L7_SEQ_LEN (sizeof(s_sequence) / sizeof(s_sequence[0]))

/* ---- 各子级 ops（直接调用，避免与 calib_mgr 单例 active_ops 冲突）---- */
extern const calib_level_ops_t calib_level1_ops;
extern const calib_level_ops_t calib_level2_ops;
extern const calib_level_ops_t calib_level3_ops;
extern const calib_level_ops_t calib_level4_ops;
extern const calib_level_ops_t calib_level5_ops;
extern const calib_level_ops_t calib_level6_ops;

static const calib_level_ops_t *s_l7_level_table[CALIB_LEVEL_MAX] = {
	[0] = NULL,
	[CALIB_LEVEL1_DRIVER] = &calib_level1_ops,
	[CALIB_LEVEL2_MOTOR] = &calib_level2_ops,
	[CALIB_LEVEL3_ENCODER] = &calib_level3_ops,
	[CALIB_LEVEL4_TORQUE] = &calib_level4_ops,
	[CALIB_LEVEL5_NONLINEAR] = &calib_level5_ops,
	[CALIB_LEVEL6_SYSTEM] = &calib_level6_ops,
};

/* 启动当前步对应的子标定 */
static bool l7_start_current_step(void)
{
	const calib_level_ops_t *ops = s_l7_level_table[s_sequence[s_step].level];
	if (ops == NULL || ops->start == NULL)
		return false;
	return ops->start(s_sequence[s_step].submode, s_param, s_dt);
}

/* 轮询当前步对应的子标定 */
static calib_state_e l7_poll_current_step(void)
{
	const calib_level_ops_t *ops = s_l7_level_table[s_sequence[s_step].level];
	if (ops == NULL || ops->poll == NULL)
		return CALIB_STATE_FAILED;
	return ops->poll();
}

/* 中止当前步对应的子标定（清理硬件）*/
static void l7_abort_current_step(void)
{
	if (s_step < L7_SEQ_LEN)
	{
		const calib_level_ops_t *ops = s_l7_level_table[s_sequence[s_step].level];
		if (ops != NULL && ops->abort != NULL)
			ops->abort();
	}
}

static bool calib_level7_start(uint8_t submode, motor_param_t *param, float dt)
{
	s_param = param;
	s_dt = dt;
	s_step = 0;
	s_failed_step = 0xFF;
	s_retry_count = 0;

	/* 上报 L7 序列总步数, 使 0x97 查询能看到 step_total */
	calib_mgr_set_l7_progress(0, (uint8_t)L7_SEQ_LEN);

	switch (submode)
	{
		case CALIB_L7_FULL_AUTO:
			/* 启动序列中第一个子标定 */
			return l7_start_current_step();
		default:
			return false;
	}
}

static calib_state_e calib_level7_poll(void)
{
	/* 全部步骤已完成 */
	if (s_step >= L7_SEQ_LEN)
		return CALIB_STATE_DONE;

	/* 上报当前步骤索引, 使 0x97 查询能看到 L7 进度 */
	calib_mgr_set_l7_progress(s_step, (uint8_t)L7_SEQ_LEN);

	calib_state_e st = l7_poll_current_step();

	if (st == CALIB_STATE_DONE)
	{
		/* 当前步完成，标记 done 并前进到下一步 */
		calib_mgr_mark_done(s_sequence[s_step].level, s_sequence[s_step].submode);
		s_step++;
		calib_mgr_set_l7_progress(s_step, (uint8_t)L7_SEQ_LEN);
		s_retry_count = 0;
		if (s_step >= L7_SEQ_LEN)
		{
			/* L7 全流程完成，置位 is_calibrated，下次上电使用标定值 */
			(void)motor_info_calib_mark_calibrated();
			return CALIB_STATE_DONE;
		}
		/* 启动下一步 */
		if (!l7_start_current_step())
			return CALIB_STATE_FAILED;
		return CALIB_STATE_RUNNING;
	}
	else if (st == CALIB_STATE_FAILED)
	{
		s_failed_step = s_step;
		/* fail_reason 已由子级 poll 设置(如 CALIB_FAIL_OUT_OF_RANGE),
		 * L7 不覆盖, 保留子级具体原因供上位机诊断 */
		switch (CALIB_L7_STRATEGY)
		{
			case CALIB_L7_STRATEGY_STOP:
				/* 失败即停止，L7 整体 FAILED */
				return CALIB_STATE_FAILED;

			case CALIB_L7_STRATEGY_SKIP:
				/* 跳过失败步，继续下一步 */
				s_step++;
				calib_mgr_set_l7_progress(s_step, (uint8_t)L7_SEQ_LEN);
				s_retry_count = 0;
				if (s_step >= L7_SEQ_LEN)
					return CALIB_STATE_DONE;
				if (!l7_start_current_step())
					return CALIB_STATE_FAILED;
				return CALIB_STATE_RUNNING;

			case CALIB_L7_STRATEGY_RETRY_ONCE:
				if (s_retry_count < 1)
				{
					/* 重试当前步：重新 start */
					s_retry_count++;
					if (!l7_start_current_step())
						return CALIB_STATE_FAILED;
					return CALIB_STATE_RUNNING;
				}
				/* 重试过一次仍失败，跳过 */
				s_step++;
				calib_mgr_set_l7_progress(s_step, (uint8_t)L7_SEQ_LEN);
				s_retry_count = 0;
				if (s_step >= L7_SEQ_LEN)
					return CALIB_STATE_DONE;
				if (!l7_start_current_step())
					return CALIB_STATE_FAILED;
				return CALIB_STATE_RUNNING;
		}
	}
	return CALIB_STATE_RUNNING;
}

static void calib_level7_abort(void)
{
	/* 中止当前子标定（清理硬件），再重置 L7 状态 */
	l7_abort_current_step();
	s_step = 0;
	s_failed_step = 0xFF;
	s_retry_count = 0;
	/* 清空 L7 进度, 避免 0x97 查询到残留的 step_total */
	calib_mgr_set_l7_progress(0, 0);
}

const calib_level_ops_t calib_level7_ops = {
	.start = calib_level7_start,
	.poll = calib_level7_poll,
	.abort = calib_level7_abort,
};
