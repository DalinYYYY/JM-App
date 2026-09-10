#ifndef __RUNTIME_PARAM_H__
#define __RUNTIME_PARAM_H__

#include <stdbool.h>
#include <stdint.h>
#include "motor_param.h"
#include "state_define.h"

/* ===== 调试映射指针的前向声明 =====
 * 仅声明 struct 标签，不引入对应头文件，避免 DataHub 层反向依赖
 * Devices/MotorControl/ParamService 层。完整类型在 runtime_param.c 中
 * include 头文件后可见；调试器靠 DWARF 信息展开字段，不受前向声明影响。
 * 前 3 个 struct 标签由 Task 1 具名化产生；后 2 个原本就是具名 struct。
 */
struct dwtTimer_s;         /* dev_dwt_counter.h:    dwtTimer_t              */
struct motor_loop_s;       /* motor_loop.h:         motor_loop_t            */
struct motor_info_storage; /* motor_info_storage.h: motor_info_storage_t    */
struct dev_power_monitor;  /* dev_power_monitor.h:  dev_power_monitor_t（原具名）*/
struct dev_commun_uart;    /* dev_commun_uart.h:    dev_commun_uart_t（原具名）*/

#define OFFSET_LUT_NUM 128
#define DT             (1.0f / 10000.0f)

/************************************* 枚举变量 *************************************/
typedef enum
{
	M1 = 0,
	MOTOR_MAX,
} motor_num_e;

typedef enum
{
	MOTOR_INFO_CTRL_IDLE = 0,      // 空闲
	MOTOR_INFO_CTRL_UPDATE = 0x01, // 保存参数
	MOTOR_INFO_CTRL_AMEND = 0x02,  // 修改参数
	MOTOR_INFO_CTRL_CLEAR = 0x03,  // 清空参数
	MOTOR_INFO_CTRL_MAX,
} motor_info_ctrl_e;

typedef enum
{
	ENCODER_DIR_NOCHANGE = 0, // 方向未变化
	ENCODER_MODE_CHANGE = 1,  // 方向已变化
} encoder_dir_e;

/* DWT 耗时打点槽位索引(start/stop 区间存入 dwt_timer.duration_us[i])
 * 注: 槽位 2~5 枚举名为历史遗留, 实际测的是电流环内部分段耗时(见注释) */
typedef enum
{
	SYS_TIMER_RECORD_CURRENT_LOOP_CYCLE = 0,  // 电流环中断周期(相邻两次进 ISR 间隔)
	SYS_TIMER_RECORD_CURRENT_LOOP_TIME = 1,   // 电流环 ISR 总耗时(三环入口+观察者)
	SYS_TIMER_RECORD_POSITION_LOOP_CYCLE = 2, // 实测: 相电流采样+Clarke+Park 段耗时(电流环 step1~3)
	SYS_TIMER_RECORD_POSITION_LOOP_TIME = 3,  // 实测: 电流环闭环段耗时(PI+解耦+反Park+SVPWM+PWM)
	SYS_TIMER_RECORD_TIM_1MS_CYCLE = 4,       // 实测: 电流环 PI+解耦 段耗时
	SYS_TIMER_RECORD_TIM_1MS_TIME = 5,        // 实测: 电流环 反Park+SVPWM+PWM 段耗时
	SYS_TIMER_RECORD_TEST_1 = 6,              // 实测: 编码器刷新+电角度更新耗时(motor_loop step0)
	SYS_TIMER_RECORD_TEST_2 = 7,              // 实测: 反馈解算+状态机耗时(motor_loop step1~2)
	SYS_TIMER_RECORD_TEST_3 = 8,              // 实测: 外环(位置/速度)+电流环 RUN 主路径耗时(step3~6)
	SYS_TIMER_RECORD_TEST_4 = 9,              // 实测: 观察者 motor_observer 耗时(control_irq)
} sys_timer_record_index_e;

/************************************* 三级变量 *************************************/
typedef union base_version_
{
	struct
	{
		char version_build;
		char version_patch;
		char version_minor;
		char version_major;
	} byte;
	uint32_t var;
} base_version_t;

typedef struct sys_version_
{
	base_version_t boot;
	base_version_t app;
	base_version_t hardware;
} sys_version_t;

typedef struct
{
	float zero_ud;
	float zero_ud_offset;
	float uq;
	float iq;
	float id;
	float velocity;
	float pos_rad;
	float target_pos_rad;
	float target_velocity;
	float target_current;
} sys_control_data_t;

/*****************************************************************************
 * @brief   运行时反馈/遥测量(按功能拆分, 由控制层/采集层周期刷新)
 * @note    每个子块对应一个数据源模块与一类 jm_cmd_def.h 反馈查询命令(0xC2~0xC8),
 *          由对应模块的 *_publish() 接口单向填充(控制层 → runtime 快照),
 *          供通信(jm_proto)/显示/日志统一读 usr, 不直接耦合控制层内部结构。
 *          字段命名参考 ODrive / VESC / SimpleFOC / MIT Cheetah 常见遥测量。
 *****************************************************************************/

/* 电气测量量: 对应 READ_PHASE_CURRENT(0xC2) / READ_DQ_CURRENT(0xC3) */
typedef struct
{
	float ia, ib, ic;             /* 三相电流 A (来自 phase_current 设备) */
	float i_alpha, i_beta;        /* Clarke 变换 αβ 静止坐标电流 A */
	float id_meas, iq_meas;       /* Park 变换 dq 旋转坐标电流测量值 A */
	float i_bus;                  /* 母线电流 A */
	float i_phase_peak;           /* 相电流峰值 A (限流/诊断用) */
	float ud, uq;                 /* dq 轴输出电压 V */
	float u_alpha, u_beta;        /* αβ 静止坐标输出电压 V */
	float duty_a, duty_b, duty_c; /* 三相 PWM 占空比 0~1 */
	float modulation;             /* 调制度 (Vref/Vbus) 0~1 */
} motor_electrical_t;

/* 运动反馈量: 对应 READ_POS_VEL(0xC6) / READ_MULTITURN(0xC7) */
typedef struct
{
	float mech_angle_rad;  /* 机械角度 rad (单圈, 0~2π) */
	float elec_angle_rad;  /* 电角度 rad (0~2π) */
	float single_turn_rad; /* 单圈位置 rad (带方向) */
	int32_t multiturn;     /* 多圈计数 */
	float position_rad;    /* 多圈累计位置 rad (输出端) */
	float velocity_rad_s;  /* 机械角速度 rad/s */
	float velocity_filt;   /* 滤波后角速度 rad/s */
	float accel_rad_s2;    /* 角加速度 rad/s² */
	uint32_t encoder_raw;  /* 编码器原始计数 */
} motor_motion_t;

/* 母线/功率/力矩: 对应 READ_BUS(0xC4) */
typedef struct
{
	float v_bus;        /* 母线电压 V */
	float i_bus;        /* 母线电流 A */
	float power_elec_w; /* 电功率 W (= vbus*ibus) */
	float power_mech_w; /* 机械功率 W (= torque*velocity) */
	float torque_est;   /* 估算输出力矩 Nm (= iq*kt*gear) */
	float efficiency;   /* 效率 0~1 */
} motor_power_t;

/* 温度: 对应 READ_TEMPERATURE(0xC5) */
typedef struct
{
	float temp_fet;   /* 功率管温度 ℃ */
	float temp_motor; /* 电机绕组温度 ℃ */
	float temp_mcu;   /* MCU 内核温度 ℃ */
} motor_thermal_t;

/* 故障与诊断: READ_FAULT 查询与 JM_TLM_FAULT 遥测(fault_mgr 派生) */
typedef struct
{
	uint32_t fault_mask;     /* 当前故障位掩码(停机类活动故障, 低13位兼容旧语义) */
	uint32_t warn_mask;      /* 当前警告掩码(异常级降功率/警告级活动) */
	uint32_t fault_latched;  /* 锁存故障(需清障清除) */
	uint32_t warn_latched;   /* 锁存警告 */
	uint16_t error_count;    /* 累计错误次数 */
	uint8_t last_fault_code; /* 最近一次故障码 */
	/* fault_mgr 扩展(0xAA/0xAB 查询源) */
	uint16_t top_fault_code; /* 最高优先级活动故障码 0xSLNN(0=无) */
	uint8_t active_count;    /* 活动故障总数 */
	uint8_t derate_pct;      /* 当前功率档百分比(100=正常) */
	uint32_t level_active;   /* bit0/1/2=故障级/异常级/警告级有活动 */
} motor_fault_t;

/* 控制目标设定: 覆盖各 CONTROL_MODE_* 指令的目标量 */
typedef struct
{
	float pos_rad;                /* 目标位置 rad (位置环/PVT) */
	float velocity_rad_s;         /* 目标速度 rad/s (速度环/前馈) */
	float torque_nm;              /* 目标力矩 Nm (力矩环/限幅) */
	float current_iq;             /* 目标 q 轴电流 A (电流环) */
	float current_id;             /* 目标 d 轴电流 A (弱磁/标定) */
	float voltage_ud, voltage_uq; /* 开环电压目标 V */
	float duty;                   /* 目标占空比 0~1 */
	/* MIT Cheetah 阻抗控制参数 */
	float mit_kp;  /* 位置刚度 Nm/rad */
	float mit_kd;  /* 速度阻尼 Nm/(rad/s) */
	float mit_tff; /* 前馈力矩 Nm */
	/* 力控/阻抗 */
	float force_nm;     /* 目标力 Nm */
	float impedance_kp; /* 阻抗刚度 Nm/rad */
	float impedance_kd; /* 阻抗阻尼 Nm/(rad/s) */
} motor_setpoint_t;

typedef struct task_count_
{
	uint32_t control_cnt;
	uint32_t commun_cnt;
	uint32_t period_cnt;
	uint32_t display_cnt;
	uint32_t idle_cnt;
	uint32_t foc_open_static_cnt;
	uint32_t foc_open_dynamic_cnt;
	uint32_t foc_open_velocity_cnt;
	uint32_t foc_current_cnt;
	uint32_t foc_velocity_cnt;
	uint32_t foc_position_cnt;
	uint32_t foc_position_variable_freq_cnt;
	uint32_t vofa_update_cnt;
	uint32_t vofa_default_cnt;
} system_task_cnt_t;

/* 运行时系统量: 对应 READ_DEV_INFO(0xD0) / HEARTBEAT(0xD2) */
typedef struct
{
	uint32_t uptime_ms;     /* 上电运行时长 ms */
	uint32_t reset_reason;  /* 复位原因(RCC 复位标志位) */
	uint32_t cpu_usage;     /* CPU 占用率 ‰ (0~1000) */
	uint32_t heartbeat_cnt; /* 心跳计数(主机存活监测) */
	uint8_t device_uid[12]; /* MCU 唯一 ID */
} sys_runtime_info_t;

/************************************* 二级变量 *************************************/
typedef struct
{
	bool enable_motor;
	bool enable_pwm;
	top_fsm_e top_state;
	run_state_e run_state;
	ctrl_mode_e ctrl_mode;
	motor_electrical_t electrical; /* 电气测量量 0xC2/0xC3 */
	motor_motion_t motion;         /* 运动反馈量 0xC6/0xC7 */
	motor_power_t power;           /* 母线/功率/力矩 0xC4 */
	motor_thermal_t thermal;       /* 温度 0xC5 */
	motor_fault_t fault;           /* 故障诊断 0xC8 */
	motor_setpoint_t setpoint;     /* 控制目标设定 */
} motor_state_t;

typedef struct
{
	sys_version_t version;
	system_task_cnt_t task_cnt;
	sys_runtime_info_t info; /* 运行时系统量 0xD0/0xD2 */
} system_t;

/************************************* 一级变量 *************************************/
typedef struct sys_data_
{
	system_t sys;
	motor_state_t motor_state[MOTOR_MAX];
	motor_param_t motor_param[MOTOR_MAX];

	/* ===== 调试映射指针 =====
	 * 指向已存在的全局变量（dwt_timer / s_motor_loop / g_motor_info_storage /
	 * dev_power_monitor / dev_commun_uart），不持有数据、不复制数据，
	 * 仅方便调试时通过 usr 一个变量统一观察。
	 * 绑定在 user_init() 的硬件初始化阶段完成；非 const 以便调试时强制设值。
	 * 访问示例: usr.p_dwt_timer->duration_us[0], usr.p_motor_loop->vel_cnt,
	 *           usr.p_dev_power_monitor->vbus, usr.p_dev_commun_uart->tx_count
	 */
	struct dwtTimer_s *p_dwt_timer;                  /* -> dwt_timer            */
	struct motor_loop_s *p_motor_loop;               /* -> s_motor_loop         */
	struct motor_info_storage *p_motor_info_storage; /* -> g_motor_info_storage */
	struct dev_power_monitor *p_dev_power_monitor;   /* -> dev_power_monitor    */
	struct dev_commun_uart *p_dev_commun_uart;       /* -> dev_commun_uart      */
} sys_data_t;

void user_data_init(void);

/**************************************** 通用调试通道 ****************************************/
/**
 * @brief   通用调试观测通道 (对应 READ_DEBUG 0xC9)
 * @note    免改协议加观测点: 任意处写 jm_dbg[i] = 变量; 即可在上位机看到。
 *          上报帧把整块 float 一次性发出, 解析器固定映射到连续槽位, 故:
 *          - 加一个观测量 = 固件写一行 + 上位机加一个数据集(纯 GUI), 解析器不动。
 *          - 通道含义由使用者自行约定(建议在调用处注释), 不固化字段名。
 *          0~7 为历史通道; 8~15 为 FOC 补偿诊断:
 *          ud_pi, uq_pi, ud_cross, uq_bemf, ud, uq, omega_mech, config。
 *          上位机按实际帧长解析，无需改协议字段。
 */
#define JM_DBG_CH 16 /* 调试通道数; 0~7兼容旧通道, 8~15为FOC补偿诊断 */
extern float jm_dbg[JM_DBG_CH];

/**************************************** 数据接口 ****************************************/
extern sys_data_t usr; // 全局变量加g_前缀

#endif                 /* __RUNTIME_PARAM_H__ */

