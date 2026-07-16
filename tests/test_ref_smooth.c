/*
 * test_ref_smooth.c - 同模式目标值渐变 Host 单元测试
 *
 * 编译：
 *   clang-cl tests/test_ref_smooth.c User/MotorControl/ControlProcess/ctrl_transition.c \
 *       -IUser/MotorControl/ControlProcess -IUser/MotorControl/PidManager \
 *       -IUser/DataHub -IUser/AppServices/StateMachine \
 *       -IUser/MotorControl/CascadeControl -IUser/Common \
 *       /Fe:tests/test_ref_smooth.exe /link /subsystem:console
 *
 * 注意：STOP 延迟过渡（transition_mgr_on_stop）依赖 system_state_t 完整定义和
 *       top_fsm_switch/motor_ctrl_dispatch，host 单测无法直接覆盖，
 *       由虚拟电机集成测试验证（Task 8 场景 F1-F3）。
 */
#include "ctrl_transition.h"
#include <stdio.h>
#include <math.h>

#define TEST_DT 0.0001f

static int tests_run = 0;
static int tests_failed = 0;

static void check(bool cond, const char *msg)
{
	tests_run++;
	if (!cond)
	{
		printf("FAIL: %s\n", msg);
		tests_failed++;
	}
	else
	{
		printf("ok  : %s\n", msg);
	}
}

static void cfg_disable_rate(ref_smooth_cfg_t *cfg)
{
	cfg->pos_rate = 0.0f;
	cfg->vel_rate = 0.0f;
	cfg->torque_rate = 0.0f;
	cfg->current_rate = 0.0f;
}

/* 用例1: 目标无变化不启动渐变 */
static void test_no_change_no_start(void)
{
	transition_t t;
	transition_init(&t);
	ref_smooth_cfg_t cfg;
	ref_smooth_cfg_init_defaults(&cfg);
	cfg_disable_rate(&cfg);

	motor_ref_t raw = {0}, prev = {0};
	raw.ctrl_type = REF_CTRL_POSITION;
	prev.ctrl_type = REF_CTRL_POSITION;
	raw.pos = 1.0f;
	prev.pos = 1.0f;

	bool started = transition_ref_smooth_check(&t, RUN_STATE_POSITION, &raw, &prev, &cfg, TEST_DT);
	check(!started, "no change: should not start");
	check(t.state == TRANSITION_IDLE, "no change: state idle");
}

/* 用例2: 位置目标突变超阈值启动渐变 */
static void test_pos_exceed_starts(void)
{
	transition_t t;
	transition_init(&t);
	ref_smooth_cfg_t cfg;
	ref_smooth_cfg_init_defaults(&cfg);
	cfg_disable_rate(&cfg);

	motor_ref_t raw = {0}, prev = {0};
	raw.ctrl_type = REF_CTRL_POSITION;
	prev.ctrl_type = REF_CTRL_POSITION;
	prev.pos = 0.0f;
	raw.pos = 5.0f;

	bool started = transition_ref_smooth_check(&t, RUN_STATE_POSITION, &raw, &prev, &cfg, TEST_DT);
	check(started, "pos exceed: should start");
	check(t.state == TRANSITION_IN_PROGRESS, "pos exceed: in progress");
	check(t.old_ref.pos == 0.0f, "pos exceed: old_ref=prev (0.0)");
	check(t.duration == cfg.smooth_duration, "pos exceed: duration=smooth_duration (rate disabled)");
}

/* 用例3: 渐变进行中不重启 */
static void test_in_progress_no_restart(void)
{
	transition_t t;
	transition_init(&t);
	ref_smooth_cfg_t cfg;
	ref_smooth_cfg_init_defaults(&cfg);
	cfg_disable_rate(&cfg);

	motor_ref_t raw = {0}, prev = {0};
	raw.ctrl_type = REF_CTRL_POSITION;
	prev.ctrl_type = REF_CTRL_POSITION;
	prev.pos = 0.0f;
	raw.pos = 5.0f;
	transition_ref_smooth_check(&t, RUN_STATE_POSITION, &raw, &prev, &cfg, TEST_DT);

	motor_ref_t prev2 = prev;
	raw.pos = 10.0f;
	bool started = transition_ref_smooth_check(&t, RUN_STATE_POSITION, &raw, &prev2, &cfg, TEST_DT);
	check(!started, "in progress: no restart");
	check(t.old_ref.pos == 0.0f, "in progress: old_ref unchanged");
}

/* 用例4: 变化小于阈值不启动 */
static void test_below_threshold_no_start(void)
{
	transition_t t;
	transition_init(&t);
	ref_smooth_cfg_t cfg;
	ref_smooth_cfg_init_defaults(&cfg);
	cfg_disable_rate(&cfg);

	motor_ref_t raw = {0}, prev = {0};
	raw.ctrl_type = REF_CTRL_VELOCITY;
	prev.ctrl_type = REF_CTRL_VELOCITY;
	prev.vel = 0.0f;
	raw.vel = 0.5f;

	bool started = transition_ref_smooth_check(&t, RUN_STATE_VELOCITY, &raw, &prev, &cfg, TEST_DT);
	check(!started, "below thresh: no start");
}

/* 用例5: DUTY 模式不启用渐变 */
static void test_duty_disabled(void)
{
	transition_t t;
	transition_init(&t);
	ref_smooth_cfg_t cfg;
	ref_smooth_cfg_init_defaults(&cfg);

	motor_ref_t raw = {0}, prev = {0};
	raw.ctrl_type = REF_CTRL_DUTY;
	prev.ctrl_type = REF_CTRL_DUTY;
	prev.duty = 0.0f;
	raw.duty = 0.9f;

	bool started = transition_ref_smooth_check(&t, RUN_STATE_DUTY_CYCLE, &raw, &prev, &cfg, TEST_DT);
	check(!started, "duty mode: disabled");
}

/* 用例14: MIT 模式不渐变 */
static void test_mit_disabled(void)
{
	transition_t t;
	transition_init(&t);
	ref_smooth_cfg_t cfg;
	ref_smooth_cfg_init_defaults(&cfg);

	motor_ref_t raw = {0}, prev = {0};
	raw.ctrl_type = REF_CTRL_TORQUE;
	prev.ctrl_type = REF_CTRL_TORQUE;
	prev.torque = 0.0f;
	raw.torque = 5.0f;

	bool started = transition_ref_smooth_check(&t, RUN_STATE_MIT, &raw, &prev, &cfg, TEST_DT);
	check(!started, "MIT mode: disabled even with TORQUE ctrl_type");
	check(t.state == TRANSITION_IDLE, "MIT mode: state idle");

	transition_init(&t);
	bool started2 = transition_ref_smooth_check(&t, RUN_STATE_TORQUE, &raw, &prev, &cfg, TEST_DT);
	check(started2, "TORQUE mode: starts (contrast to MIT)");
}

/* 用例15: COMPLETED 态复位为 IDLE */
static void test_completed_resets_to_idle(void)
{
	transition_t t;
	transition_init(&t);
	t.state = TRANSITION_COMPLETED;

	ref_smooth_cfg_t cfg;
	ref_smooth_cfg_init_defaults(&cfg);
	cfg_disable_rate(&cfg);

	motor_ref_t raw = {0}, prev = {0};
	raw.ctrl_type = REF_CTRL_POSITION;
	prev.ctrl_type = REF_CTRL_POSITION;
	prev.pos = 0.0f;
	raw.pos = 5.0f;

	bool started = transition_ref_smooth_check(&t, RUN_STATE_POSITION, &raw, &prev, &cfg, TEST_DT);
	check(!started, "completed: no start on reset tick");
	check(t.state == TRANSITION_IDLE, "completed: reset to idle");

	bool started2 = transition_ref_smooth_check(&t, RUN_STATE_POSITION, &raw, &prev, &cfg, TEST_DT);
	check(started2, "completed: starts on next tick after reset");
}

/* 用例6: ctrl_type 变化不介入 */
static void test_ctrl_type_change_skipped(void)
{
	transition_t t;
	transition_init(&t);
	ref_smooth_cfg_t cfg;
	ref_smooth_cfg_init_defaults(&cfg);

	motor_ref_t raw = {0}, prev = {0};
	prev.ctrl_type = REF_CTRL_POSITION;
	prev.pos = 0.0f;
	raw.ctrl_type = REF_CTRL_VELOCITY;
	raw.vel = 10.0f;

	bool started = transition_ref_smooth_check(&t, RUN_STATE_POSITION, &raw, &prev, &cfg, TEST_DT);
	check(!started, "ctrl_type change: skipped");
}

/* 用例7: 电流环 id/iq 任一超阈值启动 */
static void test_current_id_iq(void)
{
	transition_t t;
	transition_init(&t);
	ref_smooth_cfg_t cfg;
	ref_smooth_cfg_init_defaults(&cfg);

	motor_ref_t raw = {0}, prev = {0};
	raw.ctrl_type = REF_CTRL_CURRENT;
	prev.ctrl_type = REF_CTRL_CURRENT;
	prev.id = 0.0f;
	prev.iq = 0.0f;
	raw.id = 0.1f;
	raw.iq = 2.0f;

	bool started = transition_ref_smooth_check(&t, RUN_STATE_CURRENT, &raw, &prev, &cfg, TEST_DT);
	check(started, "current iq exceed: start");
}

/* 用例8: blend 进展正确 */
static void test_blend_progression(void)
{
	transition_t t;
	transition_init(&t);
	ref_smooth_cfg_t cfg;
	ref_smooth_cfg_init_defaults(&cfg);
	cfg_disable_rate(&cfg);
	cfg.smooth_duration = 10;

	motor_ref_t raw = {0}, prev = {0};
	raw.ctrl_type = REF_CTRL_POSITION;
	prev.ctrl_type = REF_CTRL_POSITION;
	prev.pos = 0.0f;
	raw.pos = 10.0f;
	transition_ref_smooth_check(&t, RUN_STATE_POSITION, &raw, &prev, &cfg, TEST_DT);

	motor_ref_t out;
	transition_update(&t, &raw, &out);
	check(fabsf(out.pos - 1.0f) < 1e-6f, "blend tick1: pos=1.0");

	for (int i = 0; i < 9; i++)
		transition_update(&t, &raw, &out);
	check(t.state == TRANSITION_COMPLETED, "blend: completed");
	check(fabsf(out.pos - 10.0f) < 1e-6f, "blend final: pos=10.0");
}

/* 用例9: 渐变期间 raw 变化，blend 朝最新 raw 收敛 */
static void test_blend_tracks_new_raw(void)
{
	transition_t t;
	transition_init(&t);
	ref_smooth_cfg_t cfg;
	ref_smooth_cfg_init_defaults(&cfg);
	cfg_disable_rate(&cfg);
	cfg.smooth_duration = 10;

	motor_ref_t raw = {0}, prev = {0};
	raw.ctrl_type = REF_CTRL_POSITION;
	prev.ctrl_type = REF_CTRL_POSITION;
	prev.pos = 0.0f;
	raw.pos = 10.0f;
	transition_ref_smooth_check(&t, RUN_STATE_POSITION, &raw, &prev, &cfg, TEST_DT);

	motor_ref_t out;
	transition_update(&t, &raw, &out);

	raw.pos = 20.0f;
	for (int i = 0; i < 9; i++)
		transition_update(&t, &raw, &out);
	check(t.state == TRANSITION_COMPLETED, "track: completed");
	check(fabsf(out.pos - 20.0f) < 1e-6f, "track: converge to new raw 20.0");
}

/* 用例10: enable=false 完全跳过 */
static void test_enable_false_skip(void)
{
	transition_t t;
	transition_init(&t);
	ref_smooth_cfg_t cfg;
	ref_smooth_cfg_init_defaults(&cfg);
	cfg.enable = false;

	motor_ref_t raw = {0}, prev = {0};
	raw.ctrl_type = REF_CTRL_POSITION;
	prev.ctrl_type = REF_CTRL_POSITION;
	prev.pos = 0.0f;
	raw.pos = 100.0f;

	bool started = transition_ref_smooth_check(&t, RUN_STATE_POSITION, &raw, &prev, &cfg, TEST_DT);
	check(!started, "enable=false: skip even huge jump");
	check(t.state == TRANSITION_IDLE, "enable=false: state idle");
}

/* 用例11: rate 模式算 duration */
static void test_rate_mode_duration(void)
{
	transition_t t;
	transition_init(&t);
	ref_smooth_cfg_t cfg;
	ref_smooth_cfg_init_defaults(&cfg);

	motor_ref_t raw = {0}, prev = {0};
	raw.ctrl_type = REF_CTRL_POSITION;
	prev.ctrl_type = REF_CTRL_POSITION;
	prev.pos = 0.0f;
	raw.pos = 5.0f;

	bool started = transition_ref_smooth_check(&t, RUN_STATE_POSITION, &raw, &prev, &cfg, TEST_DT);
	check(started, "rate mode: start");
	check(t.duration == 500, "rate mode: duration=500 (5/(100*0.0001))");

	motor_ref_t out;
	transition_update(&t, &raw, &out);
	check(fabsf(out.pos - 0.01f) < 1e-6f, "rate mode: tick1 pos=0.01");
}

/* 用例12: rate=0 回退固定时长模式 */
static void test_rate_zero_fallback_duration(void)
{
	transition_t t;
	transition_init(&t);
	ref_smooth_cfg_t cfg;
	ref_smooth_cfg_init_defaults(&cfg);
	cfg_disable_rate(&cfg);
	cfg.smooth_duration = 200;

	motor_ref_t raw = {0}, prev = {0};
	raw.ctrl_type = REF_CTRL_POSITION;
	prev.ctrl_type = REF_CTRL_POSITION;
	prev.pos = 0.0f;
	raw.pos = 5.0f;

	bool started = transition_ref_smooth_check(&t, RUN_STATE_POSITION, &raw, &prev, &cfg, TEST_DT);
	check(started, "rate zero: start");
	check(t.duration == 200, "rate zero: fallback to smooth_duration=200");
}

/* 用例13: rate 模式下小突变 duration 短，大突变 duration 长 */
static void test_rate_mode_scales_with_delta(void)
{
	{
		transition_t t;
		transition_init(&t);
		ref_smooth_cfg_t cfg;
		ref_smooth_cfg_init_defaults(&cfg);
		motor_ref_t raw = {0}, prev = {0};
		raw.ctrl_type = REF_CTRL_POSITION;
		prev.ctrl_type = REF_CTRL_POSITION;
		raw.pos = 1.0f;
		bool started = transition_ref_smooth_check(&t, RUN_STATE_POSITION, &raw, &prev, &cfg, TEST_DT);
		check(started, "rate small: start");
		check(t.duration == 100, "rate small: duration=100 (1/(100*0.0001))");
	}
	{
		transition_t t;
		transition_init(&t);
		ref_smooth_cfg_t cfg;
		ref_smooth_cfg_init_defaults(&cfg);
		motor_ref_t raw = {0}, prev = {0};
		raw.ctrl_type = REF_CTRL_POSITION;
		prev.ctrl_type = REF_CTRL_POSITION;
		raw.pos = 10.0f;
		bool started = transition_ref_smooth_check(&t, RUN_STATE_POSITION, &raw, &prev, &cfg, TEST_DT);
		check(started, "rate large: start");
		check(t.duration == 1000, "rate large: duration=1000 (10/(100*0.0001))");
	}
}

/* 用例16: VELOCITY 模式 STOP 语义——速度目标从 50 渐变到 0 */
static void test_stop_velocity_smooth(void)
{
	transition_t t;
	transition_init(&t);
	ref_smooth_cfg_t cfg;
	ref_smooth_cfg_init_defaults(&cfg);

	motor_ref_t raw = {0}, prev = {0};
	raw.ctrl_type = REF_CTRL_VELOCITY;
	prev.ctrl_type = REF_CTRL_VELOCITY;
	prev.vel = 50.0f;
	raw.vel = 0.0f;

	bool started = transition_ref_smooth_check(&t, RUN_STATE_VELOCITY, &raw, &prev, &cfg, TEST_DT);
	check(started, "stop velocity: smooth start");
	check(t.duration == 1000, "stop velocity: duration=1000 (50/(500*0.0001))");

	motor_ref_t out;
	transition_update(&t, &raw, &out);
	check(fabsf(out.vel - 49.95f) < 1e-4f, "stop velocity: tick1 vel=49.95");

	for (int i = 0; i < 999; i++)
		transition_update(&t, &raw, &out);
	check(t.state == TRANSITION_COMPLETED, "stop velocity: completed");
	check(fabsf(out.vel - 0.0f) < 1e-6f, "stop velocity: final vel=0.0");
}

/* 用例17: POSITION 模式 STOP 语义——目标=当前位置，无渐变，立即完成 */
static void test_stop_position_no_smooth(void)
{
	transition_t t;
	transition_init(&t);
	ref_smooth_cfg_t cfg;
	ref_smooth_cfg_init_defaults(&cfg);

	motor_ref_t raw = {0}, prev = {0};
	raw.ctrl_type = REF_CTRL_POSITION;
	prev.ctrl_type = REF_CTRL_POSITION;
	prev.pos = 3.14f;
	raw.pos = 3.14f;

	bool started = transition_ref_smooth_check(&t, RUN_STATE_POSITION, &raw, &prev, &cfg, TEST_DT);
	check(!started, "stop position: no smooth (target=current)");
	check(t.state == TRANSITION_IDLE, "stop position: idle (immediate stop)");
}

int main(void)
{
	test_no_change_no_start();
	test_pos_exceed_starts();
	test_in_progress_no_restart();
	test_below_threshold_no_start();
	test_duty_disabled();
	test_ctrl_type_change_skipped();
	test_current_id_iq();
	test_blend_progression();
	test_blend_tracks_new_raw();
	test_enable_false_skip();
	test_rate_mode_duration();
	test_rate_zero_fallback_duration();
	test_rate_mode_scales_with_delta();
	test_mit_disabled();
	test_completed_resets_to_idle();
	test_stop_velocity_smooth();
	test_stop_position_no_smooth();

	printf("\n==== %d/%d passed ====\n", tests_run - tests_failed, tests_run);
	return tests_failed ? 1 : 0;
}
