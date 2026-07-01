/**
 * @file calib_mgr.c
 * @brief 标定管理器：按 level 路由到级别模块，管理标定状态机
 */
#include "calib_mgr.h"
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
	[CALIB_LEVEL1_DRIVER]    = &calib_level1_ops,
	[CALIB_LEVEL2_MOTOR]     = &calib_level2_ops,
	[CALIB_LEVEL3_ENCODER]   = &calib_level3_ops,
	[CALIB_LEVEL4_TORQUE]    = &calib_level4_ops,
	[CALIB_LEVEL5_NONLINEAR] = &calib_level5_ops,
	[CALIB_LEVEL6_SYSTEM]    = &calib_level6_ops,
	[CALIB_LEVEL7_AUTO]      = &calib_level7_ops,
};

/* ---- 管理器私有状态 ---- */
static struct
{
	motor_param_t *param;
	float dt;
	calib_status_t status;
	const calib_level_ops_t *active_ops;
} s_mgr;

void calib_mgr_init(motor_param_t *param, float dt)
{
	memset(&s_mgr, 0, sizeof(s_mgr));
	s_mgr.param = param;
	s_mgr.dt = dt;
	s_mgr.status.state = CALIB_STATE_IDLE;
}

bool calib_mgr_start(uint8_t level, uint8_t submode)
{
	if (level == 0 || level >= CALIB_LEVEL_MAX)
		return false;
	if (s_mgr.status.state == CALIB_STATE_RUNNING)
		return false;

	const calib_level_ops_t *ops = s_level_table[level];
	if (ops == NULL || ops->start == NULL)
		return false;

	/* start 返回 false 表示 submode 不支持 */
	if (!ops->start(submode, s_mgr.param, s_mgr.dt))
		return false;

	s_mgr.active_ops = ops;
	s_mgr.status.state = CALIB_STATE_RUNNING;
	s_mgr.status.progress = 0;
	s_mgr.status.level = level;
	s_mgr.status.submode = submode;
	s_mgr.status.step = 0;
	return true;
}

calib_state_e calib_mgr_poll(void)
{
	if (s_mgr.status.state != CALIB_STATE_RUNNING || s_mgr.active_ops == NULL)
		return s_mgr.status.state;

	calib_state_e st = s_mgr.active_ops->poll();
	s_mgr.status.state = st;

	if (st == CALIB_STATE_DONE)
		s_mgr.status.progress = 100;
	else if (st == CALIB_STATE_FAILED)
		s_mgr.status.progress = 0;

	return st;
}

calib_status_t calib_mgr_get_status(void)
{
	return s_mgr.status;
}

void calib_mgr_abort(void)
{
	if (s_mgr.active_ops && s_mgr.active_ops->abort)
		s_mgr.active_ops->abort();
	s_mgr.status.state = CALIB_STATE_IDLE;
	s_mgr.status.progress = 0;
	s_mgr.active_ops = NULL;
}
