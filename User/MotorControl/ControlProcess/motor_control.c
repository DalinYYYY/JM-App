#include "motor_control.h"
#include <string.h>
#include <math.h>

/**
 * @brief 数值限幅辅助函数
 */
static float clamp(float value, float min, float max)
{
	if (value < min)
		return min;
	if (value > max)
		return max;
	return value;
}

/**
 * @brief 运行状态处理函数指针表
 * @details 不同运行状态映射到对应的控制处理函数，实现状态与逻辑的解耦。
 *          新增运行状态时需同步更新此表，保证函数指针非NULL。
 */
static void (*const s_run_state_handler_fn_table[RUN_STATE_MAX])(motor_ctrl_t *) = {
	[RUN_STATE_IDLE] = run_generic_control,
	[RUN_STATE_OPEN_LOOP] = run_generic_control,
	[RUN_STATE_CURRENT] = run_generic_control,
	[RUN_STATE_TORQUE] = run_generic_control,
	[RUN_STATE_MIT] = run_generic_control,
	[RUN_STATE_VELOCITY] = run_generic_control,
	[RUN_STATE_POSITION] = run_generic_control,
	[RUN_STATE_POSITION_VELOCITY] = run_generic_control,
	[RUN_STATE_POSITION_TORQUE] = run_generic_control,
	[RUN_STATE_VELOCITY_TORQUE] = run_generic_control,
	[RUN_STATE_DUTY_CYCLE] = run_generic_control,
	[RUN_STATE_VOLTAGE_VECTOR] = run_generic_control,
	[RUN_STATE_FIELD_WEAKENING] = run_generic_control,
	[RUN_STATE_SENSORLESS] = run_generic_control,

	[RUN_STATE_IMPEDANCE] = run_force_control,
	[RUN_STATE_ADMITTANCE] = run_force_control,
	[RUN_STATE_FORCE_CONTROL] = run_force_control,
	[RUN_STATE_FORCE_POSITION_HYBRID] = run_force_control,
	[RUN_STATE_GRAVITY_COMPENSATION] = run_force_control,
	[RUN_STATE_COLLISION_DETECTION] = run_force_control,
	[RUN_STATE_ZERO_FORCE] = run_force_control,
	[RUN_STATE_CONSTANT_FORCE] = run_force_control,
	[RUN_STATE_VARIABLE_IMPEDANCE] = run_force_control,
	[RUN_STATE_ADAPTIVE_GRAVITY_COMP] = run_force_control,
	[RUN_STATE_LANDING_BUFFER] = run_force_control,

	[RUN_STATE_PVT] = run_trajectory_control,
	[RUN_STATE_CUBIC_SPLINE] = run_trajectory_control,
	[RUN_STATE_TRAPEZOIDAL_TRAJ] = run_trajectory_control,
	[RUN_STATE_S_CURVE_TRAJ] = run_trajectory_control,
	[RUN_STATE_HOMING] = run_trajectory_control,
	[RUN_STATE_ELECTRONIC_GEAR] = run_trajectory_control,
	[RUN_STATE_ELECTRONIC_CAM] = run_trajectory_control,

	[RUN_STATE_STEP_DIR] = run_special_control,
	[RUN_STATE_ANALOG_INPUT] = run_special_control,
	[RUN_STATE_PWM_INPUT] = run_special_control,
	[RUN_STATE_JOG] = run_special_control,
	[RUN_STATE_SAFE_TEACH] = run_special_control,

	[RUN_STATE_TEST_AGING] = run_test_control,
	[RUN_STATE_TEST_SWEEP_FREQ] = run_test_control,
	[RUN_STATE_TEST_COGGING] = run_test_control,
	[RUN_STATE_TEST_FRICTION] = run_test_control,
	[RUN_STATE_TEST_INERTIA] = run_test_control,
	[RUN_STATE_DIAGNOSTIC] = run_test_control,
	[RUN_STATE_HIGH_SPEED_DAQ] = run_test_control,
	[RUN_STATE_SINGLE_STEP] = run_test_control,
};

void motor_ctrl_dispatch(motor_ctrl_t *ctrl)
{
	if (ctrl->run_state >= RUN_STATE_MAX)
		return;

	void (*handler)(motor_ctrl_t *) = s_run_state_handler_fn_table[ctrl->run_state];
	if (handler != NULL)
		handler(ctrl);
}

/**
 * @brief 初始化电机控制核心
 */
void motor_ctrl_init(motor_ctrl_t *ctrl, motor_param_t *param, float dt)
{
	memset(ctrl, 0, sizeof(motor_ctrl_t));

	ctrl->param = param;
	ctrl->dt = dt;

	ctrl->run_state = RUN_STATE_IDLE;
	ctrl->ref.ctrl_type = REF_CTRL_IDLE;
	ctrl->ref.pos_profile = PID_PROFILE_POSITION;
	ctrl->ref.vel_profile = PID_PROFILE_VELOCITY;
}

/**
 * @brief 模式预处理（基础模式：仅设置参考目标，不执行任何环计算）
 * @details 根据当前运行状态填充 ctrl->ref，下游三环模块据 ref.ctrl_type 入环。
 */
void run_generic_control(motor_ctrl_t *ctrl)
{
	motor_param_t *p = ctrl->param;
	motor_ref_t *ref = &ctrl->ref;

	// 默认使用标准位置/速度环参数（特殊模式在各自handler覆盖）
	ref->pos_profile = PID_PROFILE_POSITION;
	ref->vel_profile = PID_PROFILE_VELOCITY;

	switch (ctrl->run_state)
	{
		// 空闲：保持当前位置，下游输出置零/保持
		case RUN_STATE_IDLE:
			ref->ctrl_type = REF_CTRL_IDLE;
			ref->pos = ctrl->fb.pos;
			ref->vel_ff = 0.0f;
			break;

		// 开环电压 / 电压矢量
		case RUN_STATE_OPEN_LOOP:
		case RUN_STATE_VOLTAGE_VECTOR:
			ref->ctrl_type = REF_CTRL_VOLTAGE;
			ref->voltage = clamp(ctrl->cmd.torque, -motor_param_get_rated_voltage(p), motor_param_get_rated_voltage(p));
			break;

		// 占空比直控
		case RUN_STATE_DUTY_CYCLE:
			ref->ctrl_type = REF_CTRL_DUTY;
			ref->duty = clamp(ctrl->cmd.torque, -1.0f, 1.0f);
			break;

		// 电流环：直接使用指令电流
		case RUN_STATE_CURRENT:
		case RUN_STATE_FIELD_WEAKENING:
		case RUN_STATE_SENSORLESS:
			ref->ctrl_type = REF_CTRL_CURRENT;
			ref->id = clamp(ctrl->cmd.id,
							-motor_param_get_peak_current(p),
							motor_param_get_peak_current(p));
			ref->iq = clamp(ctrl->cmd.iq,
							-motor_param_get_peak_current(p),
							motor_param_get_peak_current(p));
			break;

		// 力矩环：力矩为目标，下游换算为 iq
		case RUN_STATE_TORQUE:
			ref->ctrl_type = REF_CTRL_TORQUE;
			ref->torque = clamp(ctrl->cmd.torque,
								-motor_param_get_peak_torque(p),
								motor_param_get_peak_torque(p));
			ref->torque_ff = 0.0f;
			break;

		// MIT模式：本模块算合成力矩，下游走电流环
		case RUN_STATE_MIT:
		{
			float pos_err = ctrl->cmd.pos - ctrl->fb.pos;
			float vel_err = ctrl->cmd.vel - ctrl->fb.vel;
			float torque = ctrl->cmd.kp * pos_err + ctrl->cmd.kd * vel_err + ctrl->cmd.torque_ff;

			ref->ctrl_type = REF_CTRL_TORQUE;
			ref->torque = clamp(torque,
								-motor_param_get_peak_torque(p),
								motor_param_get_peak_torque(p));
			ref->torque_ff = 0.0f;
			break;
		}

		// 速度环
		case RUN_STATE_VELOCITY:
			ref->ctrl_type = REF_CTRL_VELOCITY;
			ref->vel = clamp(ctrl->cmd.vel, -motor_param_get_max_speed(p), motor_param_get_max_speed(p));
			ref->torque_ff = ctrl->cmd.torque_ff;
			break;

		// 速度+力矩限幅
		case RUN_STATE_VELOCITY_TORQUE:
			ref->ctrl_type = REF_CTRL_VELOCITY;
			ref->vel = clamp(ctrl->cmd.vel, -motor_param_get_max_speed(p), motor_param_get_max_speed(p));
			ref->torque = clamp(ctrl->cmd.torque, -motor_param_get_peak_torque(p), motor_param_get_peak_torque(p));
			ref->torque_ff = ctrl->cmd.torque_ff;
			break;

		// 位置环
		case RUN_STATE_POSITION:
			ref->ctrl_type = REF_CTRL_POSITION;
			ref->pos = ctrl->cmd.pos;
			ref->vel_ff = 0.0f;
			break;

		// 位置+速度前馈
		case RUN_STATE_POSITION_VELOCITY:
			ref->ctrl_type = REF_CTRL_POSITION;
			ref->pos = ctrl->cmd.pos;
			ref->vel_ff = ctrl->cmd.vel;
			break;

		// 位置+力矩限幅
		case RUN_STATE_POSITION_TORQUE:
			ref->ctrl_type = REF_CTRL_POSITION;
			ref->pos = ctrl->cmd.pos;
			ref->vel_ff = 0.0f;
			ref->torque = clamp(ctrl->cmd.torque,
								-motor_param_get_peak_torque(p),
								motor_param_get_peak_torque(p));
			break;

		default:
			break;
	}
}

/**
 * @brief 执行力控控制
 * @details 力控模式以位置环为基底，叠加重力/摩擦/阻抗补偿后修正参考目标。
 *          补偿后的目标仍交由下游三环执行。
 */
void run_force_control(motor_ctrl_t *ctrl)
{
	// 力控以位置为基底，先生成位置参考
	ctrl->ref.ctrl_type = REF_CTRL_POSITION;
	ctrl->ref.pos = ctrl->cmd.pos;
	ctrl->ref.vel_ff = 0.0f;
	// 力控类模式使用阻抗专用位置环参数（低刚度/带输出滤波）
	ctrl->ref.pos_profile = PID_PROFILE_IMPEDANCE;
	ctrl->ref.vel_profile = PID_PROFILE_VELOCITY;

	switch (ctrl->run_state)
	{
		case RUN_STATE_ZERO_FORCE:
			// 零力模式：抵消重力和摩擦，下游直接走力矩
			//			extern float calculate_gravity_torque(float pos);
			//			extern float calculate_friction_torque(float vel, float torque);
			//			float gravity = calculate_gravity_torque(ctrl->fb.pos);
			//			float friction = calculate_friction_torque(ctrl->fb.vel, ctrl->fb.torque);
			//			ctrl->ref.ctrl_type = REF_CTRL_TORQUE;
			//			ctrl->ref.torque = -gravity - friction;
			break;

		case RUN_STATE_IMPEDANCE:
			// 阻抗控制：根据外力调整位置参考
			//			float actual_torque = ctrl->fb.iq * motor_param_get_kt(ctrl->param);
			//			float pos_comp = actual_torque / motor_param_get_impedance_kp(ctrl->param);
			//			ctrl->ref.pos += pos_comp;
			break;

		default:
			break;
	}
}

/**
 * @brief 执行轨迹控制
 * @details 先做轨迹规划更新目标位置/速度，再生成位置参考。
 */
void run_trajectory_control(motor_ctrl_t *ctrl)
{
	// 轨迹模式默认标准位置/速度环参数（回零模式下覆盖）
	ctrl->ref.pos_profile = PID_PROFILE_POSITION;
	ctrl->ref.vel_profile = PID_PROFILE_VELOCITY;

	switch (ctrl->run_state)
	{
		case RUN_STATE_TRAPEZOIDAL_TRAJ:
			// 梯形轨迹规划更新
			//			extern void trapezoidal_traj_update(motor_ctrl_t * ctrl);
			//			trapezoidal_traj_update(ctrl);
			break;

		case RUN_STATE_PVT:
			// PVT插补更新
			//			extern void pvt_update(motor_ctrl_t * ctrl);
			//			pvt_update(ctrl);
			break;

		case RUN_STATE_HOMING:
			// 回零流程更新，使用回零专用位置环参数（低速软碰）
			ctrl->ref.pos_profile = PID_PROFILE_HOMING;
			//			extern void homing_update(motor_ctrl_t * ctrl);
			//			homing_update(ctrl);
			break;

		default:
			break;
	}

	// 轨迹模式统一走位置参考
	ctrl->ref.ctrl_type = REF_CTRL_POSITION;
	ctrl->ref.pos = ctrl->cmd.pos;
	ctrl->ref.vel_ff = ctrl->cmd.vel_ff;
}

/**
 * @brief 执行特殊应用控制
 */
void run_special_control(motor_ctrl_t *ctrl)
{
	// 特殊模式默认标准参数（JOG模式下覆盖速度环参数）
	ctrl->ref.pos_profile = PID_PROFILE_POSITION;
	ctrl->ref.vel_profile = PID_PROFILE_VELOCITY;

	switch (ctrl->run_state)
	{
		case RUN_STATE_STEP_DIR:
			// 脉冲方向控制：计数脉冲并更新目标位置
			break;

		case RUN_STATE_ANALOG_INPUT:
			// 模拟量控制：将0-10V映射到-π~+π位置
			break;

		case RUN_STATE_JOG:
			// 点动：速度参考，使用点动专用速度环参数
			ctrl->ref.ctrl_type = REF_CTRL_VELOCITY;
			ctrl->ref.vel_profile = PID_PROFILE_JOG;
			ctrl->ref.vel = clamp(ctrl->cmd.vel,
								  -motor_param_get_max_speed(ctrl->param),
								  motor_param_get_max_speed(ctrl->param));
			return;

		default:
			break;
	}

	// 默认走位置参考
	ctrl->ref.ctrl_type = REF_CTRL_POSITION;
	ctrl->ref.pos = ctrl->cmd.pos;
	ctrl->ref.vel_ff = 0.0f;
}

/**
 * @brief 执行测试控制
 */
void run_test_control(motor_ctrl_t *ctrl)
{
	switch (ctrl->run_state)
	{
		case RUN_STATE_TEST_SWEEP_FREQ:
		{
			// 扫频测试：生成正弦力矩指令
			static float phase = 0.0f;
			static float freq = 1.0f; // 1Hz初始频率

			ctrl->ref.ctrl_type = REF_CTRL_TORQUE;
			ctrl->ref.torque = sinf(phase) * 0.5f; // 0.5N·m振幅
			phase += 2.0f * (float)M_PI * freq * ctrl->dt;
			return;
		}

		default:
			break;
	}

	// 默认保持空闲
	ctrl->ref.ctrl_type = REF_CTRL_IDLE;
	ctrl->ref.pos = ctrl->fb.pos;
}
