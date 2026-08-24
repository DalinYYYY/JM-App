#ifndef _STATE_DEFINE_H_
#define _STATE_DEFINE_H_

#include <stdint.h>
#include <stdbool.h>

/*****************************************************************************
 * @brief                   系统顶层主状态（优先级从高到低）
 *****************************************************************************/
typedef enum
{
	TOP_FSM_INIT = 0,   // 系统初始化
	TOP_FSM_SAFETY,     // 安全状态（最高优先级）
	TOP_FSM_FAULT,      // 故障状态
	TOP_FSM_IDLE,       // 待机状态
	TOP_FSM_READY,      // 就绪状态（已使能，等待运行指令，电机不动）
	TOP_FSM_RUN,        // 运行状态
	TOP_FSM_CALIB,      // 校准状态
	TOP_FSM_CONFIG,     // 配置状态
	TOP_FSM_BOOTLOADER, // 固件升级状态

	TOP_FSM_MAX
} top_fsm_e;

/*****************************************************************************
 * @brief                   上位机控制指令
 *****************************************************************************/
typedef enum
{
	// 基础控制指令 (0x00-0x0F)
	CONTROL_MODE_IDLE = 0x00,    // 进入待机
	CONTROL_MODE_HOLD = 0x01,    // 位置保持
	CONTROL_MODE_BRAKE = 0x02,   // 机械刹车
	CONTROL_MODE_ESTOP = 0x03,   // 紧急停止
	CONTROL_MODE_ENABLE = 0x04,  // 上使能（IDLE→READY，伺服使能但不运动）
	CONTROL_MODE_DISABLE = 0x05, // 下使能（READY/RUN→IDLE，伺服失能）
	CONTROL_MODE_STOP = 0x06,    // 停止运行（RUN→READY，停止运动但保持使能）

	// 核心运动控制指令 (0x10-0x2F)
	CONTROL_MODE_OPEN_LOOP = 0x10,         // 开环电压
	CONTROL_MODE_CURRENT = 0x11,           // 电流环
	CONTROL_MODE_TORQUE = 0x12,            // 力矩环
	CONTROL_MODE_MIT = 0x13,               // MIT控制
	CONTROL_MODE_VELOCITY = 0x14,          // 速度环
	CONTROL_MODE_POSITION = 0x15,          // 位置环
	CONTROL_MODE_POSITION_VELOCITY = 0x16, // 位置+速度前馈
	CONTROL_MODE_POSITION_TORQUE = 0x17,   // 位置+力矩限幅
	CONTROL_MODE_VELOCITY_TORQUE = 0x18,   // 速度+力矩限幅
	CONTROL_MODE_DUTY_CYCLE = 0x19,        // 占空比直接控制
	CONTROL_MODE_VOLTAGE_VECTOR = 0x1A,    // 电压矢量
	CONTROL_MODE_FIELD_WEAKENING = 0x1B,   // 弱磁控制
	CONTROL_MODE_SENSORLESS = 0x1C,        // 无感FOC

	// 高级力控指令 (0x30-0x4F)
	CONTROL_MODE_IMPEDANCE = 0x30,             // 阻抗控制
	CONTROL_MODE_ADMITTANCE = 0x31,            // 导纳控制
	CONTROL_MODE_FORCE_CONTROL = 0x32,         // 纯力控制
	CONTROL_MODE_FORCE_POSITION_HYBRID = 0x33, // 力位混合
	CONTROL_MODE_GRAVITY_COMPENSATION = 0x34,  // 重力补偿
	CONTROL_MODE_COLLISION_DETECTION = 0x35,   // 碰撞检测
	CONTROL_MODE_ZERO_FORCE = 0x36,            // 零力模式
	CONTROL_MODE_CONSTANT_FORCE = 0x37,        // 恒力控制
	CONTROL_MODE_VARIABLE_IMPEDANCE = 0x38,    // 变阻抗
	CONTROL_MODE_ADAPTIVE_GRAVITY_COMP = 0x39, // 自适应重力补偿
	CONTROL_MODE_LANDING_BUFFER = 0x3A,        // 落地缓冲

	// 轨迹与同步指令 (0x50-0x5F)
	CONTROL_MODE_PVT = 0x50,              // PVT插补
	CONTROL_MODE_CUBIC_SPLINE = 0x51,     // 三次样条
	CONTROL_MODE_TRAPEZOIDAL_TRAJ = 0x52, // 梯形轨迹
	CONTROL_MODE_S_CURVE_TRAJ = 0x53,     // S型轨迹
	CONTROL_MODE_HOMING = 0x54,           // 回零
	CONTROL_MODE_CANOPEN_SYNC = 0x55,     // CANopen同步
	CONTROL_MODE_ETHERCAT_CSP = 0x56,     // EtherCAT CSP
	CONTROL_MODE_ETHERCAT_CSV = 0x57,     // EtherCAT CSV
	CONTROL_MODE_ETHERCAT_CST = 0x58,     // EtherCAT CST
	CONTROL_MODE_PP = 0x59,               // 轮廓位置
	CONTROL_MODE_PV = 0x5A,               // 轮廓速度
	CONTROL_MODE_PT = 0x5B,               // 轮廓力矩
	CONTROL_MODE_ELECTRONIC_GEAR = 0x5C,  // 电子齿轮
	CONTROL_MODE_ELECTRONIC_CAM = 0x5D,   // 电子凸轮

	// 负载模拟指令 (0x60-0x6F, 0x68-0x6F预留)
	CONTROL_MODE_PASSIVE_TORQUE = 0x60, // 被动恒转矩加载
	CONTROL_MODE_DYNAMIC_TORQUE = 0x61, // 动态转矩加载（阶跃/正弦/方波/脉冲）
	CONTROL_MODE_QUADRATIC_LOAD = 0x62, // 平方转矩负载（风机/水泵）
	CONTROL_MODE_CONSTANT_POWER = 0x63, // 恒功率负载（主轴/卷绕）
	CONTROL_MODE_FRICTION_LOAD = 0x64,  // 摩擦负载（库仑+粘性）
	CONTROL_MODE_INERTIA_SIM = 0x65,    // 惯量模拟
	CONTROL_MODE_DUTY_PROFILE = 0x66,   // 工况谱复现（下载/回放）
	CONTROL_MODE_IMPACT_LOAD = 0x67,    // 冲击/过载负载（分层限幅）

	// 特殊应用指令 (0x70-0x8F)
	CONTROL_MODE_STEP_DIR = 0x70,           // 脉冲方向
	CONTROL_MODE_ANALOG_INPUT = 0x71,       // 模拟量输入
	CONTROL_MODE_PWM_INPUT = 0x72,          // PWM输入
	CONTROL_MODE_JOG = 0x73,                // 点动
	CONTROL_MODE_SAFE_TEACH = 0x74,         // 安全示教
	CONTROL_MODE_TEST_AGING = 0x75,         // 老化测试
	CONTROL_MODE_TEST_SWEEP_FREQ = 0x76,    // 扫频测试
	CONTROL_MODE_TEST_COGGING = 0x77,       // 齿槽测试
	CONTROL_MODE_TEST_FRICTION = 0x78,      // 摩擦测试
	CONTROL_MODE_TEST_INERTIA = 0x79,       // 惯量测试
	CONTROL_MODE_TEST_CURRENT_LOOP = 0x7A,  // 电流环测试
	CONTROL_MODE_TEST_VELOCITY_LOOP = 0x7B, // 速度环测试

	// 校准指令 (0x90-0xAF) — 类别命令+子命令模式
	CONTROL_MODE_CALIB_LEVEL1 = 0x90, // L1 驱动硬件底层（payload[0]=子模式）
	CONTROL_MODE_CALIB_LEVEL2 = 0x91, // L2 电机电气身份
	CONTROL_MODE_CALIB_LEVEL3 = 0x92, // L3 编码器校准
	CONTROL_MODE_CALIB_LEVEL4 = 0x93, // L4 转矩基础
	CONTROL_MODE_CALIB_LEVEL5 = 0x94, // L5 非线性补偿
	CONTROL_MODE_CALIB_LEVEL6 = 0x95, // L6 负载系统级
	CONTROL_MODE_CALIB_LEVEL7 = 0x96, // L7 自动化集成
	CONTROL_MODE_CALIB_QUERY = 0x97,  // 标定进度查询
	CONTROL_MODE_CALIB_ABORT = 0x98,  // 中止标定

	// 系统诊断指令 (0xB0-0xCF)
	CONTROL_MODE_CLEAR_FAULT = 0xB0,      // 清除故障
	CONTROL_MODE_DIAGNOSTIC = 0xB1,       // 诊断模式
	CONTROL_MODE_ENTER_BOOTLOADER = 0xB2, // 进入Bootloader
	CONTROL_MODE_SAVE_CONFIG = 0xB3,      // 保存配置
	CONTROL_MODE_FACTORY_RESET = 0xB4,    // 恢复出厂
	CONTROL_MODE_START_LOG = 0xB5,        // 开始日志
	CONTROL_MODE_STOP_LOG = 0xB6,         // 停止日志
	CONTROL_MODE_HIGH_SPEED_DAQ = 0xB7,   // 高速采集
	CONTROL_MODE_SINGLE_STEP = 0xB8,      // 单步调试

	CONTROL_MODE_MAX = 0xFF
} ctrl_mode_e;

/*****************************************************************************
 * @brief                   电机运行子状态
 *****************************************************************************/
typedef enum
{
	RUN_STATE_IDLE = 0,              // 空闲保持
	RUN_STATE_HOLD,                  // 位置保持（主动锁定当前位置）
	RUN_STATE_OPEN_LOOP,             // 开环电压控制
	RUN_STATE_CURRENT,               // 电流环控制
	RUN_STATE_TORQUE,                // 力矩环控制
	RUN_STATE_MIT,                   // MIT Cheetah控制
	RUN_STATE_VELOCITY,              // 速度环控制
	RUN_STATE_POSITION,              // 位置环控制
	RUN_STATE_POSITION_VELOCITY,     // 位置+速度前馈
	RUN_STATE_POSITION_TORQUE,       // 位置+力矩限幅
	RUN_STATE_VELOCITY_TORQUE,       // 速度+力矩限幅
	RUN_STATE_PROFILE_VELOCITY,      // 轮廓速度模式（PV）
	RUN_STATE_PROFILE_TORQUE,        // 轮廓力矩模式（PT）
	RUN_STATE_DUTY_CYCLE,            // 占空比直接控制
	RUN_STATE_VOLTAGE_VECTOR,        // 电压矢量控制
	RUN_STATE_FIELD_WEAKENING,       // 弱磁控制
	RUN_STATE_SENSORLESS,            // 无感FOC控制

	RUN_STATE_IMPEDANCE,             // 阻抗控制
	RUN_STATE_ADMITTANCE,            // 导纳控制
	RUN_STATE_FORCE_CONTROL,         // 纯力控制
	RUN_STATE_FORCE_POSITION_HYBRID, // 力位混合控制
	RUN_STATE_GRAVITY_COMPENSATION,  // 重力补偿
	RUN_STATE_COLLISION_DETECTION,   // 碰撞检测
	RUN_STATE_ZERO_FORCE,            // 零力模式
	RUN_STATE_CONSTANT_FORCE,        // 恒力控制
	RUN_STATE_VARIABLE_IMPEDANCE,    // 变阻抗控制
	RUN_STATE_ADAPTIVE_GRAVITY_COMP, // 自适应重力补偿
	RUN_STATE_LANDING_BUFFER,        // 落地缓冲

	RUN_STATE_PVT,                   // PVT插补
	RUN_STATE_CUBIC_SPLINE,          // 三次样条插补
	RUN_STATE_TRAPEZOIDAL_TRAJ,      // 梯形轨迹
	RUN_STATE_S_CURVE_TRAJ,          // S型轨迹
	RUN_STATE_HOMING,                // 回零
	RUN_STATE_ELECTRONIC_GEAR,       // 电子齿轮
	RUN_STATE_ELECTRONIC_CAM,        // 电子凸轮

	RUN_STATE_PASSIVE_TORQUE,        // 被动恒转矩加载
	RUN_STATE_DYNAMIC_TORQUE,        // 动态转矩加载
	RUN_STATE_QUADRATIC_LOAD,        // 平方转矩负载
	RUN_STATE_CONSTANT_POWER,        // 恒功率负载
	RUN_STATE_FRICTION_LOAD,         // 摩擦负载
	RUN_STATE_INERTIA_SIM,           // 惯量模拟
	RUN_STATE_DUTY_PROFILE,          // 工况谱复现
	RUN_STATE_IMPACT_LOAD,           // 冲击/过载负载

	RUN_STATE_STEP_DIR,              // 脉冲方向控制
	RUN_STATE_ANALOG_INPUT,          // 模拟量控制
	RUN_STATE_PWM_INPUT,             // PWM输入控制
	RUN_STATE_JOG,                   // 点动
	RUN_STATE_SAFE_TEACH,            // 安全示教

	RUN_STATE_TEST_AGING,            // 老化测试
	RUN_STATE_TEST_SWEEP_FREQ,       // 扫频测试
	RUN_STATE_TEST_COGGING,          // 齿槽转矩测试
	RUN_STATE_TEST_FRICTION,         // 摩擦力测试
	RUN_STATE_TEST_INERTIA,          // 转动惯量测试
	RUN_STATE_DIAGNOSTIC,            // 诊断模式
	RUN_STATE_HIGH_SPEED_DAQ,        // 高速数据采集
	RUN_STATE_SINGLE_STEP,           // 单步调试

	RUN_STATE_MAX
} run_state_e;

#endif // STATE_DEFINE_H
