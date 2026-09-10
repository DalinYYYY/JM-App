#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

#include <stdint.h>
#include "motor_param.h"
#include "motor_pid_profile.h"
#include "state_define.h"
#include "utils.h"

/**
 * @brief 电机控制指令（上层下发）
 * @details 仅承载各模式通用的基础运动目标；模式专用指令参数
 *          (如负载模拟)独立成结构挂载于 motor_ctrl_t，避免污染本结构。
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
 * @brief 负载模拟 (0x60~0x67)
 * @details 指令参数与运行时状态内聚一体。指令参数由通信线程在
 *          app_set_mode 临界区内写入、控制ISR读取(单生产者单消费者,
 *          沿用工程无锁约定); 运行时状态在模式进入/切换时经
 *          motor_load_sim_reset 复位, 参数热更新不复位相位。
 */
typedef struct
{
	/* ---- 指令参数 (协议载荷解析写入) ---- */
	float t_set;      /* 0x60 制动转矩设定(N·m) */
	float amp;        /* 0x61 波形幅值(N·m) */
	float freq;       /* 0x61 波形频率(Hz) */
	float bias;       /* 0x61 波形偏置(N·m) */
	float duration;   /* 0x61/0x67 总时长(s, 0=连续) */
	float k;          /* 0x62 平方系数(N·m·s²/rad²) */
	float power;      /* 0x63 恒功率值(W) */
	float t_c;        /* 0x64 库仑摩擦(N·m) */
	float b_visc;     /* 0x64 粘性系数(N·m·s/rad) */
	float j_sim;      /* 0x65 模拟惯量(kg·m²) */
	float t_bias;     /* 0x65 恒转矩基载(N·m) */
	float ratio;      /* 0x67 过载倍数(相对peak_torque, 1.0~2.0) */
	float period;     /* 0x67 周期(s) */
	float width;      /* 0x67 脉宽(s) */
	uint8_t waveform; /* 0x61: 0=阶跃1=正弦2=方波3=单脉冲 0x67: 0=周期脉冲1=方波循环2=单次冲击 */
	uint8_t flags;    /* bit0=被动方向(方向跟随转速) */

	/* ---- 运行时状态 (模式进入/切换时复位) ---- */
	uint32_t phase_acc;      /* 波形NCO相位累加器(0x61/0x67) */
	uint32_t elapsed_ticks;  /* 进入模式后控制周期计数 */
	float dir_state;         /* 0x60 当前制动方向(0=无载荷/±1), 滞回状态机 */
	float torque_out;        /* 0x60 斜坡后的实际输出转矩(N·m), 速率限制平滑 */
	float vel_filt;          /* 0x60 低速方向判定用滤波速度(rad/s) */
	float dir_candidate;    /* 0x60 待确认方向(0=无) */
	uint32_t dir_confirm_ticks; /* 0x60 方向确认计时 */
	uint32_t release_ticks;  /* 0x60 零速释放确认计时 */
	float vel_prev;          /* 惯量模拟: 上一拍速度(rad/s) */
	float a_est;             /* 惯量模拟: 加速度低通估计(rad/s²) */
	uint8_t overload_active; /* 冲击/过载: 过载窗口是否允许(冷却期禁止) */
	uint32_t overload_ticks; /* 冲击/过载: 过载累计计时 */
	uint32_t cooldown_ticks; /* 冲击/过载: 冷却期剩余计时 */
} motor_load_sim_t;

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
	uint8_t gate_driver_fault; /*!< 栅极驱动器硬件故障(nFAULT): 1=故障, 0=正常; 无此硬件的板型恒 0 */

	/* 编码器健康信息(motor_loop 每拍刷新, 供故障检测) */
	float mech_angle_deg;      /*!< 单圈机械角度 ° [0,360) */
	uint16_t enc_err_cnt;      /*!< 连续坏帧计数(CRC 失败累计, 有效帧清零) */
	uint8_t enc_health;        /*!< 健康位图: bit0=位置无效 bit1=磁场过弱 bit2=磁场过强 */
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
	REF_CTRL_DUTY,     // 占空比直控
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
	float pos;                 // 目标位置(rad)
	float vel;                 // 目标速度(rad/s)
	float accel;               // 参考加速度(rad/s²) 解析生成(PV斜坡/过渡导数/扫频)，速度环惯量前馈用
	float torque;              // 目标力矩(N·m)
	float id;                  // 目标d轴电流(A)
	float iq;                  // 目标q轴电流(A)
	float kp;                  // MIT刚度
	float kd;                  // MIT阻尼
	float torque_ff;           // 力矩前馈(N·m)
	float vel_ff;              // 速度前馈(rad/s)
	float ud;                  // 开环d轴电压(V)
	float voltage;             // 开环q轴电压(V)
	float duty;                // 占空比(-1.0~1.0)

	// PID参数配置文件选择（下游级联控制据此为不同模式加载不同PID参数）
	motor_pid_profile_id_e pos_profile; // 位置环参数配置文件
	motor_pid_profile_id_e vel_profile; // 速度环参数配置文件
} motor_ref_t;

typedef enum
{
	MOTOR_SWEEP_STATE_IDLE = 0,
	MOTOR_SWEEP_STATE_SETTLE,
	MOTOR_SWEEP_STATE_MEASURE,
	MOTOR_SWEEP_STATE_DONE,
	MOTOR_SWEEP_STATE_ABORTED
} motor_sweep_state_e;

typedef struct
{
	uint8_t test_mode;
	uint8_t sweep_mode;
	uint8_t point_cfg;
	uint8_t flags;
	uint16_t f_start_x10;
	uint16_t f_end_x10;
	uint16_t amp_raw;
	int16_t bias_raw;
	uint16_t session_id;
} motor_sweep_config_t;

/**
 * @brief PV 速度轮廓运行时状态（斜坡发生器）
 * @details 斜坡输出与解析加速度同拍生成（accel=±rate 或 0），
 *          供速度环惯量前馈使用。模式进入时经 motor_profile_vel_reset
 *          复位，下次调用从实测速度无扰起步。
 */
typedef struct
{
	float vel;      /* 当前斜坡输出(rad/s) */
	uint8_t active; /* 模式激活标志：0=下次调用从 fb.vel 起步 */
} motor_profile_vel_t;

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
	float dt;        // 控制周期(s)

	/* 扫频测试运行时状态（实例化，禁止文件级 static 运行态） */
	motor_sweep_state_e sweep_state;
	uint8_t sweep_active;
	uint8_t sweep_test_mode;
	uint8_t sweep_mode;
	uint8_t sweep_trace_sample_enable;
	uint8_t sweep_trace_flags;
	uint8_t sweep_finish_pending;
	uint16_t sweep_session_id;
	uint16_t sweep_point;
	uint16_t sweep_point_count;
	uint16_t sweep_trace_point;
	uint32_t sweep_phase_acc;
	uint32_t sweep_phase_inc;
	uint32_t sweep_trace_phase_inc;
	uint32_t sweep_trace_freq_mhz;
	uint32_t sweep_state_ticks;
	uint32_t sweep_settle_ticks;
	uint32_t sweep_measure_ticks;
	uint32_t sweep_total_ticks;
	/* TRACE 采样率逐频点钳制范围: 低频点按 20*f 降到 min, 高频点封顶 start */
	uint32_t sweep_trace_rate_min;
	uint32_t sweep_trace_rate_start;
	float sweep_amp;
	float sweep_bias;
	uint32_t sweep_phase_inc_table[1024];
	uint32_t sweep_freq_mhz_table[1024];

	/* 负载模拟 (0x60~0x67): 指令参数 + 运行时状态内聚, 见 motor_load_sim_t */
	motor_load_sim_t load_sim;

	/* PV 速度轮廓斜坡状态（实例化，禁止文件级 static 运行态） */
	motor_profile_vel_t profile_vel;
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

/* 扫频配置在通信线程调用，逐拍参考生成由 motor_ctrl_dispatch 调用。
 * trace_rate_min/start 为 TRACE 采样率逐频点钳制范围(协议层按测试对象确定)。 */
int motor_sweep_configure(motor_ctrl_t *ctrl, const motor_sweep_config_t *cfg,
                          uint32_t trace_rate_min, uint32_t trace_rate_start,
                          uint16_t *point_count, uint32_t *duration_x100);
void motor_sweep_abort(motor_ctrl_t *ctrl);
int motor_sweep_is_active(const motor_ctrl_t *ctrl);
int motor_sweep_is_complete(const motor_ctrl_t *ctrl);

#endif /* __MOTOR_CONTROL_H__ */
