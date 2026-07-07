#include "motor_control.h"
#include "motor_mode.h"
#include <string.h>

/**
 * @brief 运行状态 → 模式处理函数 映射表（ops 函数表）
 * @details 前期已拆分的模式指向独立 motor_mode_xxx_run；
 *          未拆分的模式统一指向 motor_mode_legacy_run，后续逐个迁移。
 *          新增模式时在此表注册即可，无需改动 dispatch 逻辑。
 */
static const motor_mode_fn s_mode_table[RUN_STATE_MAX] = {
	/* ===== 基础模式（已拆分）===== */
	[RUN_STATE_IDLE] = motor_mode_idle_run,
	[RUN_STATE_HOLD] = motor_mode_hold_run,
	[RUN_STATE_OPEN_LOOP] = motor_mode_open_loop_run,
	[RUN_STATE_CURRENT] = motor_mode_current_run,
	[RUN_STATE_TORQUE] = motor_mode_torque_run,
	[RUN_STATE_MIT] = motor_mode_mit_run,
	[RUN_STATE_VELOCITY] = motor_mode_velocity_run,
	[RUN_STATE_POSITION] = motor_mode_position_run,
	[RUN_STATE_POSITION_VELOCITY] = motor_mode_position_run,
	[RUN_STATE_POSITION_TORQUE] = motor_mode_position_run,
	[RUN_STATE_VELOCITY_TORQUE] = motor_mode_velocity_run,
	[RUN_STATE_PROFILE_VELOCITY] = motor_mode_profile_velocity_run, /* PV */
	[RUN_STATE_PROFILE_TORQUE] = motor_mode_profile_torque_run,     /* PT */
	[RUN_STATE_DUTY_CYCLE] = motor_mode_duty_run,
	[RUN_STATE_VOLTAGE_VECTOR] = motor_mode_open_loop_run,
	[RUN_STATE_FIELD_WEAKENING] = motor_mode_current_run,
	[RUN_STATE_SENSORLESS] = motor_mode_current_run,

	/* ===== 轨迹模式（未拆分，走 legacy）===== */
	[RUN_STATE_PVT] = motor_mode_legacy_run, /* 后续拆为 motor_mode_pvt_run */
	[RUN_STATE_CUBIC_SPLINE] = motor_mode_legacy_run,
	[RUN_STATE_TRAPEZOIDAL_TRAJ] = motor_mode_legacy_run,
	[RUN_STATE_S_CURVE_TRAJ] = motor_mode_legacy_run,
	[RUN_STATE_HOMING] = motor_mode_legacy_run,
	[RUN_STATE_ELECTRONIC_GEAR] = motor_mode_legacy_run,
	[RUN_STATE_ELECTRONIC_CAM] = motor_mode_legacy_run,

	/* ===== 力控模式（未拆分，走 legacy）===== */
	[RUN_STATE_IMPEDANCE] = motor_mode_legacy_run,
	[RUN_STATE_ADMITTANCE] = motor_mode_legacy_run,
	[RUN_STATE_FORCE_CONTROL] = motor_mode_legacy_run,
	[RUN_STATE_FORCE_POSITION_HYBRID] = motor_mode_legacy_run,
	[RUN_STATE_GRAVITY_COMPENSATION] = motor_mode_legacy_run,
	[RUN_STATE_COLLISION_DETECTION] = motor_mode_legacy_run,
	[RUN_STATE_ZERO_FORCE] = motor_mode_legacy_run,
	[RUN_STATE_CONSTANT_FORCE] = motor_mode_legacy_run,
	[RUN_STATE_VARIABLE_IMPEDANCE] = motor_mode_legacy_run,
	[RUN_STATE_ADAPTIVE_GRAVITY_COMP] = motor_mode_legacy_run,
	[RUN_STATE_LANDING_BUFFER] = motor_mode_legacy_run,

	/* ===== 特殊模式（未拆分，走 legacy）===== */
	[RUN_STATE_STEP_DIR] = motor_mode_legacy_run,
	[RUN_STATE_ANALOG_INPUT] = motor_mode_legacy_run,
	[RUN_STATE_PWM_INPUT] = motor_mode_legacy_run,
	[RUN_STATE_JOG] = motor_mode_legacy_run,
	[RUN_STATE_SAFE_TEACH] = motor_mode_legacy_run,

	/* ===== 测试模式（扫频已拆分，其余走 legacy）===== */
	[RUN_STATE_TEST_AGING] = motor_mode_legacy_run,
	[RUN_STATE_TEST_SWEEP_FREQ] = motor_mode_test_sweep_run,
	[RUN_STATE_TEST_COGGING] = motor_mode_legacy_run,
	[RUN_STATE_TEST_FRICTION] = motor_mode_legacy_run,
	[RUN_STATE_TEST_INERTIA] = motor_mode_legacy_run,
	[RUN_STATE_DIAGNOSTIC] = motor_mode_legacy_run,
	[RUN_STATE_HIGH_SPEED_DAQ] = motor_mode_legacy_run,
	[RUN_STATE_SINGLE_STEP] = motor_mode_legacy_run,
};

void motor_ctrl_dispatch(motor_ctrl_t *ctrl)
{
	if (ctrl->run_state >= RUN_STATE_MAX)
		return;

	motor_mode_fn handler = s_mode_table[ctrl->run_state];
	if (handler != NULL)
		handler(ctrl);
}

void motor_ctrl_init(motor_ctrl_t *ctrl, motor_param_t *param, float dt)
{
	memset(ctrl, 0, sizeof(motor_ctrl_t));

	ctrl->param = param;
	ctrl->dt = dt;

	ctrl->run_state = RUN_STATE_IDLE;
	ctrl->ref.ctrl_type = REF_CTRL_IDLE;
	ctrl->ref.pos_profile = PID_PROFILE_POSITION;
	ctrl->ref.vel_profile = PID_PROFILE_VELOCITY;

	ctrl->test_phase = 0.0f;
	ctrl->test_freq = 1.0f;
}

/**
 * @brief Legacy fallback：处理未拆分的模式
 * @details 保留原 run_force_control / run_trajectory_control / run_special_control /
 *          run_test_control 的 switch 逻辑。后续逐个模式迁移到独立文件后，
 *          此函数逐渐缩小直至删除。
 */
void motor_mode_legacy_run(motor_ctrl_t *ctrl)
{
	motor_ref_t *ref = &ctrl->ref;
	ref->pos_profile = PID_PROFILE_POSITION;
	ref->vel_profile = PID_PROFILE_VELOCITY;

	switch (ctrl->run_state)
	{
		/* ---- 轨迹模式（位置为基底）---- */
		case RUN_STATE_PVT:
		case RUN_STATE_CUBIC_SPLINE:
		case RUN_STATE_TRAPEZOIDAL_TRAJ:
		case RUN_STATE_S_CURVE_TRAJ:
		case RUN_STATE_ELECTRONIC_GEAR:
		case RUN_STATE_ELECTRONIC_CAM:
			ref->ctrl_type = REF_CTRL_POSITION;
			ref->pos = ctrl->cmd.pos;
			ref->vel_ff = ctrl->cmd.vel_ff;
			break;

		case RUN_STATE_HOMING:
			ref->ctrl_type = REF_CTRL_POSITION;
			ref->pos_profile = PID_PROFILE_HOMING;
			ref->pos = ctrl->cmd.pos;
			ref->vel_ff = ctrl->cmd.vel_ff;
			break;

		/* ---- 力控模式（位置为基底 + 阻抗 profile）---- */
		case RUN_STATE_IMPEDANCE:
		case RUN_STATE_ADMITTANCE:
		case RUN_STATE_FORCE_CONTROL:
		case RUN_STATE_FORCE_POSITION_HYBRID:
		case RUN_STATE_GRAVITY_COMPENSATION:
		case RUN_STATE_COLLISION_DETECTION:
		case RUN_STATE_ZERO_FORCE:
		case RUN_STATE_CONSTANT_FORCE:
		case RUN_STATE_VARIABLE_IMPEDANCE:
		case RUN_STATE_ADAPTIVE_GRAVITY_COMP:
		case RUN_STATE_LANDING_BUFFER:
			ref->ctrl_type = REF_CTRL_POSITION;
			ref->pos_profile = PID_PROFILE_IMPEDANCE;
			ref->pos = ctrl->cmd.pos;
			ref->vel_ff = 0.0f;
			break;

		/* ---- 特殊模式 ---- */
		case RUN_STATE_JOG:
			ref->ctrl_type = REF_CTRL_VELOCITY;
			ref->vel_profile = PID_PROFILE_JOG;
			ref->vel = motor_mode_clamp(ctrl->cmd.vel,
			                            -motor_param_get_max_speed(ctrl->param),
			                            motor_param_get_max_speed(ctrl->param));
			break;

		case RUN_STATE_STEP_DIR:
		case RUN_STATE_ANALOG_INPUT:
		case RUN_STATE_PWM_INPUT:
		case RUN_STATE_SAFE_TEACH:
		default:
			ref->ctrl_type = REF_CTRL_POSITION;
			ref->pos = ctrl->cmd.pos;
			ref->vel_ff = 0.0f;
			break;

		/* ---- 其他测试模式 ---- */
		case RUN_STATE_TEST_AGING:
		case RUN_STATE_TEST_COGGING:
		case RUN_STATE_TEST_FRICTION:
		case RUN_STATE_TEST_INERTIA:
		case RUN_STATE_DIAGNOSTIC:
		case RUN_STATE_HIGH_SPEED_DAQ:
		case RUN_STATE_SINGLE_STEP:
			ref->ctrl_type = REF_CTRL_IDLE;
			ref->pos = ctrl->fb.pos;
			break;
	}
}
