/**
 * @file calib_mgr.c
 * @brief 标定管理器：按 level 路由到级别模块，管理标定状态机
 */
#include "calib_mgr.h"
#include "calib_config.h"
#include <string.h>

/* ---- 各级别模块导出的 ops（前向声明） ---- */
extern const calib_level_ops_t calib_level1_ops;
extern const calib_level_ops_t calib_level2_ops;
extern const calib_level_ops_t calib_level3_ops;
extern const calib_level_ops_t calib_level4_ops;
extern const calib_level_ops_t calib_level5_ops;
extern const calib_level_ops_t calib_level6_ops;
extern const calib_level_ops_t calib_level7_ops;

/* ---- level → ops 调度表（索引 1-7，0 保留） ---- */
static const calib_level_ops_t *s_level_table[CALIB_LEVEL_MAX] = {
	[0] = NULL,
	[CALIB_LEVEL1_DRIVER] = &calib_level1_ops,
	[CALIB_LEVEL2_MOTOR] = &calib_level2_ops,
	[CALIB_LEVEL3_ENCODER] = &calib_level3_ops,
	[CALIB_LEVEL4_TORQUE] = &calib_level4_ops,
	[CALIB_LEVEL5_NONLINEAR] = &calib_level5_ops,
	[CALIB_LEVEL6_SYSTEM] = &calib_level6_ops,
	[CALIB_LEVEL7_AUTO] = &calib_level7_ops,
};

/* ---- 管理器私有状态 ---- */
static struct
{
	calib_io_t io; /* 硬件访问接口（motor + param + dt） */
	calib_status_t status;
	const calib_level_ops_t *active_ops;
} s_mgr;

static uint64_t s_done_mask = 0;   /* 已完成子模式位图（CALIB_DONE_*）*/
static uint32_t s_global_tick = 0; /* 单次标定全局 tick 计数（超时保护）*/

void calib_mgr_init(struct dev_motor *motor, motor_param_t *param, float dt)
{
	memset(&s_mgr, 0, sizeof(s_mgr));
	s_mgr.io.motor = motor;
	s_mgr.io.param = param;
	s_mgr.io.dt = dt;
	s_mgr.status.state = CALIB_STATE_IDLE;
}

const calib_io_t *calib_mgr_get_io(void)
{
	return &s_mgr.io;
}

bool calib_mgr_start(uint8_t level, uint8_t submode)
{
	if (level == 0 || level >= CALIB_LEVEL_MAX)
		return false;
	if (s_mgr.status.state == CALIB_STATE_RUNNING)
		return false;

	/* 前置依赖检查（submode 越界则跳过，交由 level 的 start 校验 submode 合法性）*/
	if (submode >= 1 && submode < CALIB_LEVEL_MAX)
	{
		uint64_t dep = s_calib_dep_table[level][submode];
		if (dep != 0 && (s_done_mask & dep) != dep)
		{
			s_mgr.status.state = CALIB_STATE_FAILED;
			s_mgr.status.fail_reason = CALIB_FAIL_DEP_NOT_MET;
			return false;
		}
	}

	const calib_level_ops_t *ops = s_level_table[level];
	if (ops == NULL || ops->start == NULL)
		return false;

	/* start 返回 false 表示 submode 不支持 */
	if (!ops->start(submode, s_mgr.io.param, s_mgr.io.dt))
		return false;

	s_mgr.active_ops = ops;
	s_mgr.status.state = CALIB_STATE_RUNNING;
	s_mgr.status.progress = 0;
	s_mgr.status.level = level;
	s_mgr.status.submode = submode;
	s_mgr.status.step = 0;
	s_mgr.status.fail_reason = CALIB_FAIL_NONE; /* 启动成功，清失败原因 */
	s_global_tick = 0;                          /* 重置全局超时计数 */
	return true;
}

calib_state_e calib_mgr_poll(void)
{
	if (s_mgr.status.state != CALIB_STATE_RUNNING || s_mgr.active_ops == NULL)
		return s_mgr.status.state;

	/* 全局超时保护 */
	if (++s_global_tick >= CALIB_CFG_GLOBAL_TIMEOUT_TICKS)
	{
		s_mgr.status.state = CALIB_STATE_FAILED;
		s_mgr.status.fail_reason = CALIB_FAIL_TIMEOUT;
		/* 调当前 level 的 abort 清理硬件 */
		if (s_mgr.active_ops->abort)
			s_mgr.active_ops->abort();
		return s_mgr.status.state;
	}

	calib_state_e st = s_mgr.active_ops->poll();
	s_mgr.status.state = st;

	if (st == CALIB_STATE_DONE)
	{
		s_mgr.status.progress = 100;
		s_mgr.status.fail_reason = CALIB_FAIL_NONE;
		calib_mgr_mark_done(s_mgr.status.level, s_mgr.status.submode);
	}
	else if (st == CALIB_STATE_FAILED)
	{
		s_mgr.status.progress = 0;
		/* level 模块未设置具体失败原因时，默认记为超时 */
		if (s_mgr.status.fail_reason == CALIB_FAIL_NONE)
			s_mgr.status.fail_reason = CALIB_FAIL_TIMEOUT;
	}

	return st;
}

calib_status_t calib_mgr_get_status(void)
{
	return s_mgr.status;
}

void calib_mgr_set_step(uint8_t step)
{
	s_mgr.status.step = step;
}

void calib_mgr_set_fail_reason(calib_fail_reason_e reason)
{
	s_mgr.status.fail_reason = reason;
}

void calib_mgr_set_l7_progress(uint8_t step, uint8_t step_total)
{
	s_mgr.status.step = step;
	s_mgr.status.step_total = step_total;
	/* L7 当前子项的 level/submode 由 s_sequence[step] 决定, 此处不更新;
	 * level/submode 在 calib_mgr_start 时已设为 L7/FULL_AUTO, 保持不变即可。
	 * L7 poll 内部推进子标定时若需细化, 可由 L7 直接调 calib_mgr_set_step。*/
}

void calib_mgr_abort(void)
{
	s_mgr.status.fail_reason = CALIB_FAIL_ABORTED; /* 标记为被中止 */
	if (s_mgr.active_ops && s_mgr.active_ops->abort)
		s_mgr.active_ops->abort();
	s_mgr.status.state = CALIB_STATE_IDLE;
	s_mgr.status.progress = 0;
	s_mgr.active_ops = NULL;
}

/* ---- 标定完成标志管理 ---- */
void calib_mgr_mark_done(uint8_t level, uint8_t submode)
{
	if (level >= 1 && level <= 7 && submode >= 1 && submode <= 8)
		s_done_mask |= CALIB_DONE_BIT(level, submode);
}

bool calib_mgr_is_done(uint8_t level, uint8_t submode)
{
	if (level >= 1 && level <= 7 && submode >= 1 && submode <= 8)
		return (s_done_mask & CALIB_DONE_BIT(level, submode)) != 0;
	return false;
}

void calib_mgr_clear_done(void)
{
	s_done_mask = 0;
}
