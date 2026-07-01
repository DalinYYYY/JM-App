#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

#include <stdint.h>
#include "motor_param.h"
#include "pid_profile.h"
#include "state_define.h"
#include "utils.h"

/**
 * @brief 电机控制指令（上层下发）
 */
typedef struct
{
	float pos;
	float vel;
	float torque;
	float id;
	float iq;
	float kp;
	float kd;
	float torque_ff;
	float vel_ff;
} motor_cmd_t;

/**
 * @brief 电机反馈数据
 */
typedef struct
{
	float pos;
	float vel;
	float ia;
	float ib;
	float ic;
	float id;
	float iq;
	float bus_voltage;
	float temperature;
	float torque;
} motor_fb_t;

/**
 * @brief 参考控制类型
 * @details 告知下游三环模块应从哪一层入环。下游据此选择级联入口，
 *          并在 ctrl_type 发生变化时对目标环积分做预装载，实现无扰切换。
 */
typedef enum
{
	REF_CTRL_IDLE = 0, // 空闲：下游输出保持/置零
	REF_CTRL_VOLTAGE,  // 开环电压：下游直接用 dq 电压
	REF_CTRL_DUTY,	   // 占空比直控
	REF_CTRL_CURRENT,  // 从电流环入（id/iq 为目标）
	REF_CTRL_TORQUE,   // 从电流环入（torque 换算为 iq）
	REF_CTRL_VELOCITY, // 从速度环入（vel 为目标）
	REF_CTRL_POSITION, // 从位置环入（pos 为目标）
} ref_ctrl_type_e;

/**
 * @brief 电机控制参考输出
 * @details 本模块的唯一对外输出。下游三环模块每个控制周期读取此结构，
 *          根据 ctrl_type 决定从哪一级环路开始级联计算。
 *          所有目标值在本模块内已完成物理限幅。
 */
typedef struct
{
	ref_ctrl_type_e ctrl_type; // 入环层级
	float pos;				   // 目标位置(rad)
	float vel;				   // 目标速度(rad/s)
	float torque;			   // 目标力矩(N·m)
	float id;				   // 目标d轴电流(A)
	float iq;				   // 目标q轴电流(A)
	float kp;				   // MIT刚度
	float kd;				   // MIT阻尼
	float torque_ff;		   // 力矩前馈(N·m)
	float vel_ff;			   // 速度前馈(rad/s)
	float voltage;			   // 开环q轴电压(V)
	float duty;				   // 占空比(-1.0~1.0)

	// PID参数配置文件选择（下游级联控制据此为不同模式加载不同PID参数）
	pid_profile_id_e pos_profile; // 位置环参数配置文件
	pid_profile_id_e vel_profile; // 速度环参数配置文件
} motor_ref_t;

/**
 * @brief 电机控制核心
 * @details 仅负责运行模式管理与参考目标生成，不包含任何环路计算。
 *          位置/速度/电流三环由下游模块依据 ref.ctrl_type 实现。
 */
typedef struct
{
	run_state_e run_state;
	motor_cmd_t cmd;
	motor_fb_t fb;
	motor_param_t *param;
	motor_ref_t ref; // 对外参考输出（唯一）
	float dt;		 // 控制周期(s)

	/* 测试模式运行时状态（避免 static 变量导致的重入性问题）*/
	float test_phase; /* 扫频测试相位累计 */
	float test_freq;  /* 扫频测试当前频率 */
} motor_ctrl_t;

/**
 * @brief 初始化电机控制核心
 * @param ctrl 电机控制指针
 * @param param 电机参数指针
 * @param dt 控制周期(s)
 */
void motor_ctrl_init(motor_ctrl_t *ctrl, motor_param_t *param, float dt);

/**
 * @brief 按当前运行状态分发到对应控制处理函数，生成 ctrl->ref
 * @param ctrl 电机控制指针
 * @details 内部维护“运行状态 → 控制处理函数”映射表，封装全部控制逻辑。
 *          状态机模块只需调用本函数，无需感知具体控制实现。
 */
void motor_ctrl_dispatch(motor_ctrl_t *ctrl);

#endif /* __MOTOR_CONTROL_H__ */
