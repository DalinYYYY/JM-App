/**
 * @file    motor_info.h
 * @brief   MotorInfo 配置参数 API 接口（1024B 整块空间）
 * @date    2026-07-09
 *
 * @warning 【自动生成文件，请勿手动修改】
 *          本文件由脚本 motor_info_generate.py 根据 motor_info.csv 自动生成，
 *          任何手动改动都会在下次运行脚本时被覆盖。
 *          如需修改参数定义，请编辑源 CSV 配置表后重新生成。
 */

#ifndef __MOTOR_INFO_H__
#define __MOTOR_INFO_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

/* 字节对齐宏：保证 RAM 布局与 Flash 字节布局完全一致，可整块 memcpy */
#ifndef __ALIGNED_4
#define __ALIGNED_4 __attribute__((aligned(4)))
#endif

/* ===== 自动生成元信息 ===== */
#define MOTOR_INFO_GEN_DATE      "2026-07-09"
#define MOTOR_INFO_PARAM_COUNT   85
#define MOTOR_INFO_VERSION_MAJOR 1
#define MOTOR_INFO_VERSION_MINOR 0

/* ===== Flash 区域布局常量 ===== */
#define PARAM_MAGIC     0x53455256u /* "SERVO_V2" */
#define PARAM_AREA_SIZE 1024
#define MAX_BLOCK_COUNT 6

/* 各子块偏移/大小常量（供 Flash 读写寻址） */
#define MOTOR_INFO_BLOCK_SYSTEMPARAM_OFFSET       0x0040u
#define MOTOR_INFO_BLOCK_SYSTEMPARAM_SIZE         64u
#define MOTOR_INFO_BLOCK_MOTORCALIBPARAM_OFFSET   0x0080u
#define MOTOR_INFO_BLOCK_MOTORCALIBPARAM_SIZE     128u
#define MOTOR_INFO_BLOCK_DEVICEPARAM_OFFSET       0x0100u
#define MOTOR_INFO_BLOCK_DEVICEPARAM_SIZE         64u
#define MOTOR_INFO_BLOCK_CONTROLPARAM_OFFSET      0x0140u
#define MOTOR_INFO_BLOCK_CONTROLPARAM_SIZE        320u
#define MOTOR_INFO_BLOCK_PROTECTCOMMPARAM_OFFSET  0x0280u
#define MOTOR_INFO_BLOCK_PROTECTCOMMPARAM_SIZE    128u
#define MOTOR_INFO_BLOCK_ADVANCEDALGOPARAM_OFFSET 0x0300u
#define MOTOR_INFO_BLOCK_ADVANCEDALGOPARAM_SIZE   64u

	/* ===== 全局头部与子块索引表 ===== */
	typedef struct __ALIGNED_4
	{
		uint32_t offset; /* 子块偏移(字节) */
		uint32_t size;   /* 子块大小(字节) */
	} BlockIndex_t;

	typedef struct __ALIGNED_4
	{
		uint32_t magic;                       /* 魔数 PARAM_MAGIC */
		uint16_t version_major;               /* 主版本 */
		uint16_t version_minor;               /* 次版本 */
		uint32_t crc32;                       /* 参数区 CRC32(校验范围跳过本字段) */
		BlockIndex_t blocks[MAX_BLOCK_COUNT]; /* 6*8 = 48B */
		uint32_t reserved;                    /* 4B 填充凑足 64B */
	} ParamHeader_t;                          /* 64B */

	/* ===== 各子块数据结构（字段顺序与 CSV 一致，4 字节对齐）===== */
	/**
 * @brief   系统级参数
 * @details 块大小 64B，已用 20B，预留 44B
 */
	typedef struct __ALIGNED_4
	{
		uint32_t config_version;    /* 配置版本号  [主版本.次版本(高16位.低16位) [SystemParam段0-15 本表占用0-4]] */
		uint32_t enable_uart;       /* 接口使能  [bit0:UART bit1:CAN bit2:CANFD bit3:USB] */
		uint32_t enable_bus_sensor; /* 总线传感器使能  [0:禁用 1:启用] */
		uint32_t safety_limit;      /* 安全限制使能  [0:禁用 1:启用 调试期建议关闭] */
		uint32_t total_runtime_s;   /* 累计运行时间 (s)  [掉电保存 运维数据 定期写入避免频繁擦写] */
		uint32_t reserved[11];      /* 预留 44B */
	} SystemParam_t;

	/**
 * @brief   电机标定参数（含减速器/编码器/功率级/电流采样）
 * @details 块大小 128B，已用 108B，预留 20B
 */
	typedef struct __ALIGNED_4
	{
		uint32_t is_calibrated;             /* 电机是否校准  [0:未校准 1:已校准 [MotorCalibParam段16-47 本表占用16-42]] */
		uint32_t pole_pairs;                /* 电机极对数 (pairs)  [电机极对数 影响电角度=机械角×极对数] */
		uint32_t motor_type;                /* 电机类型  [0:SPMSM 1:IPMSM 2:BLDC] */
		uint32_t direction;                 /* 电机方向  [0:正向 1:反向] */
		float phase_resistance;             /* 相电阻 (ohm)  [标定值 电流环Ki=ωc·R依赖此值] */
		float phase_inductance_d;           /* d轴相电感 (H)  [标定值 电流环Kp=ωc·Ld依赖此值] */
		float phase_inductance_q;           /* q轴相电感 (H)  [标定值 IPMSM的Lq一般略大于Ld] */
		float flux_linkage;                 /* 永磁体磁链 (Wb)  [标定值 反电势法辨识] */
		float torque_constant;              /* 转矩常数 (Nm/A)  [标定值 速度环Kp=J·ωc/Kt依赖此值] */
		float rotor_inertia;                /* 转子惯量 (kg·m²)  [标定值 速度环Kp/Ki依赖此值] */
		float friction_coulomb;             /* 库仑摩擦力矩 (Nm)  [标定值 L5摩擦辨识] */
		float friction_viscous;             /* 粘滞摩擦系数 (Nm/(rad/s))  [标定值 L5摩擦辨识] */
		float gear_ratio;                   /* 减速比  [电机转速/输出转速] */
		float gear_efficiency;              /* 减速器效率  [传动效率 0~1] */
		float calibration_current;          /* 校准电流 (A)  [R/L标定施加电流 注意发热] */
		float resistance_calib_max_voltage; /* 电阻校准最大电压 (V)  [R标定电压限幅 防过流] */
		float current_lim;                  /* 峰值电流限制 (A)  [硬件保护 瞬时最大电流] */
		float current_control_bandwidth;    /* 电流环带宽 (Hz)  [autotune用此值算电流环PID] */
		uint32_t enc_type;                  /* 编码器类型  [1:MT6701 2:MT6835 0:ABZ增量 3:霍尔 同板可换] */
		uint32_t enc_lines;                 /* 编码器分辨率 (CPR)  [SPI绝对值为分辨率 增量式为CPR] */
		int32_t enc_direction;              /* 编码器计数方向  [1:正向 -1:反向] */
		float enc_offset;                   /* 编码器初始位置偏移 (deg)  [校准后保存 机械零点偏移] */
		float elec_angle_bias;              /* 电角度偏移 (rad)  [FOC换相必需 校准后保存 最关键] */
		uint32_t pwm_freq_hz;               /* PWM载波频率 (Hz)  [不同功率器件可能不同] */
		float dead_time_ns;                 /* PWM死区时间 (ns)  [不同功率器件可能不同] */
		float shunt_resistance;             /* 电流采样电阻 (ohm)  [硬件相关 不同板子不同] */
		float current_amp_gain;             /* 电流放大增益  [硬件相关 运放增益] */
		uint32_t reserved[5];               /* 预留 20B */
	} MotorCalibParam_t;

	/**
 * @brief   设备参数（CAN/UART）
 * @details 块大小 64B，已用 32B，预留 32B
 */
	typedef struct __ALIGNED_4
	{
		float device_zero;        /* 设备零度 (rad)  [机械零点位置 [DeviceParam段48-63 本表占用48-55]] */
		uint32_t device_time;     /* 设备生产日期  [YYYYMMDD格式] */
		uint32_t can_id;          /* CAN节点ID  [11位标准ID] */
		uint32_t can_baudrate;    /* CAN波特率 (bps) */
		float can_timeout_s;      /* CAN通信超时 (s)  [0=禁用超时] */
		uint32_t can_fd_enable;   /* CAN FD使能  [0:传统CAN 1:CAN FD] */
		uint32_t can_fd_baudrate; /* CAN FD数据波特率 (bps) */
		uint32_t uart_baudrate;   /* UART波特率 (bps) */
		uint32_t reserved[8];     /* 预留 32B */
	} DeviceParam_t;

	/**
 * @brief   控制参数（三环PID+前馈+滤波）
 * @details 块大小 320B，已用 92B，预留 228B
 */
	typedef struct __ALIGNED_4
	{
		float kp_ld;                     /* d轴比例增益 (V/A)  [电流环d轴P增益 autotune默认Kp=ωc·Ld [ControlParam段64-127 本表占用64-85]] */
		float ki_ld;                     /* d轴积分增益 (V/(A·s))  [电流环d轴I增益 autotune默认Ki=ωc·R] */
		float kp_lq;                     /* q轴比例增益 (V/A)  [电流环q轴P增益 autotune默认Kp=ωc·Lq] */
		float ki_lq;                     /* q轴积分增益 (V/(A·s))  [电流环q轴I增益 autotune默认Ki=ωc·R] */
		float integral_limit;            /* 积分限幅 (V)  [电流环积分输出限幅 建议Kp_q×额定电流×1.5 与motor_param.c默认值对齐] */
		float decoupling_gain;           /* dq轴解耦增益  [0~1 调试期可适当增大观察效果] */
		float comp_du_V;                 /* 死区补偿电压 (V)  [死区非线性补偿电压] */
		float pwm_duty_max;              /* PWM最大占空比  [0~1 防过调制] */
		float kp_s;                      /* 速度环比例增益 (A/(rad/s))  [速度环P增益 autotune默认Kp=J·ωc/Kt] */
		float ki_s;                      /* 速度环积分增益 (A/rad)  [速度环I增益 autotune默认Ki=J·ωc²/(4·Kt)] */
		float speed_integral_limit;      /* 速度环积分限幅 (A)  [建议额定电流×0.5] */
		float vff;                       /* 速度前馈系数  [0~1 调试期可适当增大] */
		float aff;                       /* 加速度前馈系数  [0~1 加速度前馈] */
		float jerk_ff;                   /* 加加速度前馈系数  [0~1 加加速度前馈] */
		float speed_filter_alpha;        /* 速度滤波系数  [一阶低通滤波系数 越大滤波越弱] */
		uint32_t speed_filter_enable;    /* 速度滤波使能  [0:禁用 1:启用] */
		float kp_p;                      /* 位置环比例增益 (Hz)  [位置环P增益 autotune默认Kp=2π·f] */
		float ki_p;                      /* 位置环积分增益 (1/s)  [位置环I增益 一般为0] */
		float position_integral_limit;   /* 位置环积分限幅 (rad)  [位置环积分输出限幅] */
		float position_filter_alpha;     /* 位置滤波系数  [一阶低通滤波系数] */
		uint32_t position_filter_enable; /* 位置滤波使能  [0:禁用 1:启用] */
		float following_error_limit;     /* 跟随误差限制 (P)  [位置跟随误差保护阈值 调试期建议放大] */
		uint32_t pid_source_mask;        /* PID来源位掩码  [bit[3:0]=电流环 bit[7:4]=速度环 bit[11:8]=位置环 0=默认 1=Flash 2=理论估计] */
		uint32_t reserved[57];           /* 预留 228B */
	} ControlParam_t;

	/**
 * @brief   保护与通信参数
 * @details 块大小 128B，已用 44B，预留 84B
 */
	typedef struct __ALIGNED_4
	{
		float over_current_A;               /* 过流保护阈值 (A)  [ [ProtectCommParam段128-159 本表占用128-138]] */
		float over_voltage_V;               /* 过压保护阈值 (V) */
		float under_voltage_V;              /* 欠压保护阈值 (V) */
		float over_temp_drive;              /* 驱动器过温阈值 (℃) */
		float over_temp_motor;              /* 电机过温阈值 (℃) */
		float under_temp_d;                 /* 欠温保护阈值 (℃) */
		float over_speed_rad_s;             /* 过速保护阈值 (rad/s) */
		int32_t position_following_error_p; /* 位置跟随误差保护 (P) */
		int32_t pos_limit_min;              /* 位置下限 (P)  [硬件位置下限] */
		int32_t pos_limit_max;              /* 位置上限 (P)  [硬件位置上限] */
		uint32_t error_enable_mask;         /* 保护使能掩码  [bit0:过流 bit1:过压 bit2:欠压 bit3:过温 bit4:过速 bit5:跟随误差] */
		uint32_t reserved[21];              /* 预留 84B */
	} ProtectCommParam_t;

	/**
 * @brief   高级算法参数（MIT/力控/回零）
 * @details 块大小 64B，已用 44B，预留 20B
 */
	typedef struct __ALIGNED_4
	{
		float mit_kp;                  /* MIT位置刚度 (Nm/rad)  [ [AdvancedAlgoParam段160-191 本表占用160-170]] */
		float mit_kd;                  /* MIT速度阻尼 (Nm/(rad/s)) */
		float mit_max_current;         /* MIT最大电流 (A) */
		float mit_feedforward_torque;  /* MIT前馈力矩 (Nm) */
		float force_kp;                /* 力控比例增益 (A/Nm) */
		float force_ki;                /* 力控积分增益 (A/(Nm·s)) */
		float force_limit;             /* 力控力矩限制 (Nm) */
		uint32_t force_control_enable; /* 力控使能  [0:禁用 1:启用] */
		uint32_t homing_method;        /* 回零方法  [0:当前位置回零 1:限位回零] */
		float homing_speed;            /* 回零速度 (rad/s) */
		float homing_offset;           /* 回零偏移 (rad) */
		uint32_t reserved[5];          /* 预留 20B */
	} AdvancedAlgoParam_t;

	/* ===== 主参数区联合体：1024B 整块空间 ===== */
	typedef union __ALIGNED_4
	{
		uint8_t raw[PARAM_AREA_SIZE]; /* 原始字节数组，可直接 memcpy 到 Flash */
		struct __ALIGNED_4
		{
			ParamHeader_t header;            /* 0x0000  64B */
			SystemParam_t system;            /* 0x0040  64B  系统级参数 */
			MotorCalibParam_t motor_calib;   /* 0x0080  128B  电机标定参数（含减速器/编码器/功率级/电流采样） */
			DeviceParam_t device;            /* 0x0100  64B  设备参数（CAN/UART） */
			ControlParam_t control;          /* 0x0140  320B  控制参数（三环PID+前馈+滤波） */
			ProtectCommParam_t protect_comm; /* 0x0280  128B  保护与通信参数 */
			AdvancedAlgoParam_t advanced;    /* 0x0300  64B  高级算法参数（MIT/力控/回零） */
			uint8_t reserved[192];           /* 0x0340  192B  末尾预留 */
		} blocks;
	} motor_info_t;                          /* 1024B */

	/******************************************************************************
 * @brief   基础 API
 ******************************************************************************/
	/**
 * @brief   初始化为默认值（含头部魔数/版本/块索引表 + 各参数默认值）
 * @param   cfg 参数区指针
 * @return  0=成功, -EINVAL=空指针
 */
	int motor_info_init(motor_info_t *cfg);

	/**
 * @brief   校验所有参数范围
 * @param   cfg 参数区指针
 * @return  0=全部通过, >0=首个越界参数的 Index(见CSV), -EINVAL=空指针
 */
	int motor_info_validate(const motor_info_t *cfg);

	/**
 * @brief   打印所有参数
 * @param   cfg 参数区指针
 */
	void motor_info_print(const motor_info_t *cfg);

	/******************************************************************************
 * @brief   系统级参数
 ******************************************************************************/
	/**
 * @brief   读取 配置版本号
 * @param   cfg 参数区指针
 * @return  配置版本号
 */
	uint32_t motor_info_get_config_version(const motor_info_t *cfg);
	/**
 * @brief   设置 配置版本号
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_config_version(motor_info_t *cfg, uint32_t value);

	/**
 * @brief   读取 接口使能
 * @param   cfg 参数区指针
 * @return  接口使能
 */
	uint32_t motor_info_get_enable_uart(const motor_info_t *cfg);
	/**
 * @brief   设置 接口使能
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_enable_uart(motor_info_t *cfg, uint32_t value);

	/**
 * @brief   读取 总线传感器使能
 * @param   cfg 参数区指针
 * @return  总线传感器使能
 */
	uint32_t motor_info_get_enable_bus_sensor(const motor_info_t *cfg);
	/**
 * @brief   设置 总线传感器使能
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_enable_bus_sensor(motor_info_t *cfg, uint32_t value);

	/**
 * @brief   读取 安全限制使能
 * @param   cfg 参数区指针
 * @return  安全限制使能
 */
	uint32_t motor_info_get_safety_limit(const motor_info_t *cfg);
	/**
 * @brief   设置 安全限制使能
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_safety_limit(motor_info_t *cfg, uint32_t value);

	/**
 * @brief   读取 累计运行时间 (s)
 * @param   cfg 参数区指针
 * @return  累计运行时间
 */
	uint32_t motor_info_get_total_runtime_s(const motor_info_t *cfg);
	/**
 * @brief   设置 累计运行时间 (s)
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_total_runtime_s(motor_info_t *cfg, uint32_t value);

	/******************************************************************************
 * @brief   电机标定参数（含减速器/编码器/功率级/电流采样）
 ******************************************************************************/
	/**
 * @brief   读取 电机是否校准
 * @param   cfg 参数区指针
 * @return  电机是否校准
 */
	uint32_t motor_info_get_is_calibrated(const motor_info_t *cfg);
	/**
 * @brief   设置 电机是否校准
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_is_calibrated(motor_info_t *cfg, uint32_t value);

	/**
 * @brief   读取 电机极对数 (pairs)
 * @param   cfg 参数区指针
 * @return  电机极对数
 */
	uint32_t motor_info_get_pole_pairs(const motor_info_t *cfg);
	/**
 * @brief   设置 电机极对数 (pairs)
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_pole_pairs(motor_info_t *cfg, uint32_t value);

	/**
 * @brief   读取 电机类型
 * @param   cfg 参数区指针
 * @return  电机类型
 */
	uint32_t motor_info_get_motor_type(const motor_info_t *cfg);
	/**
 * @brief   设置 电机类型
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_motor_type(motor_info_t *cfg, uint32_t value);

	/**
 * @brief   读取 电机方向
 * @param   cfg 参数区指针
 * @return  电机方向
 */
	uint32_t motor_info_get_direction(const motor_info_t *cfg);
	/**
 * @brief   设置 电机方向
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_direction(motor_info_t *cfg, uint32_t value);

	/**
 * @brief   读取 相电阻 (ohm)
 * @param   cfg 参数区指针
 * @return  相电阻
 */
	float motor_info_get_phase_resistance(const motor_info_t *cfg);
	/**
 * @brief   设置 相电阻 (ohm)
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_phase_resistance(motor_info_t *cfg, float value);

	/**
 * @brief   读取 d轴相电感 (H)
 * @param   cfg 参数区指针
 * @return  d轴相电感
 */
	float motor_info_get_phase_inductance_d(const motor_info_t *cfg);
	/**
 * @brief   设置 d轴相电感 (H)
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_phase_inductance_d(motor_info_t *cfg, float value);

	/**
 * @brief   读取 q轴相电感 (H)
 * @param   cfg 参数区指针
 * @return  q轴相电感
 */
	float motor_info_get_phase_inductance_q(const motor_info_t *cfg);
	/**
 * @brief   设置 q轴相电感 (H)
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_phase_inductance_q(motor_info_t *cfg, float value);

	/**
 * @brief   读取 永磁体磁链 (Wb)
 * @param   cfg 参数区指针
 * @return  永磁体磁链
 */
	float motor_info_get_flux_linkage(const motor_info_t *cfg);
	/**
 * @brief   设置 永磁体磁链 (Wb)
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_flux_linkage(motor_info_t *cfg, float value);

	/**
 * @brief   读取 转矩常数 (Nm/A)
 * @param   cfg 参数区指针
 * @return  转矩常数
 */
	float motor_info_get_torque_constant(const motor_info_t *cfg);
	/**
 * @brief   设置 转矩常数 (Nm/A)
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_torque_constant(motor_info_t *cfg, float value);

	/**
 * @brief   读取 转子惯量 (kg·m²)
 * @param   cfg 参数区指针
 * @return  转子惯量
 */
	float motor_info_get_rotor_inertia(const motor_info_t *cfg);
	/**
 * @brief   设置 转子惯量 (kg·m²)
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_rotor_inertia(motor_info_t *cfg, float value);

	/**
 * @brief   读取 库仑摩擦力矩 (Nm)
 * @param   cfg 参数区指针
 * @return  库仑摩擦力矩
 */
	float motor_info_get_friction_coulomb(const motor_info_t *cfg);
	/**
 * @brief   设置 库仑摩擦力矩 (Nm)
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_friction_coulomb(motor_info_t *cfg, float value);

	/**
 * @brief   读取 粘滞摩擦系数 (Nm/(rad/s))
 * @param   cfg 参数区指针
 * @return  粘滞摩擦系数
 */
	float motor_info_get_friction_viscous(const motor_info_t *cfg);
	/**
 * @brief   设置 粘滞摩擦系数 (Nm/(rad/s))
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_friction_viscous(motor_info_t *cfg, float value);

	/**
 * @brief   读取 减速比
 * @param   cfg 参数区指针
 * @return  减速比
 */
	float motor_info_get_gear_ratio(const motor_info_t *cfg);
	/**
 * @brief   设置 减速比
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_gear_ratio(motor_info_t *cfg, float value);

	/**
 * @brief   读取 减速器效率
 * @param   cfg 参数区指针
 * @return  减速器效率
 */
	float motor_info_get_gear_efficiency(const motor_info_t *cfg);
	/**
 * @brief   设置 减速器效率
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_gear_efficiency(motor_info_t *cfg, float value);

	/**
 * @brief   读取 校准电流 (A)
 * @param   cfg 参数区指针
 * @return  校准电流
 */
	float motor_info_get_calibration_current(const motor_info_t *cfg);
	/**
 * @brief   设置 校准电流 (A)
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_calibration_current(motor_info_t *cfg, float value);

	/**
 * @brief   读取 电阻校准最大电压 (V)
 * @param   cfg 参数区指针
 * @return  电阻校准最大电压
 */
	float motor_info_get_resistance_calib_max_voltage(const motor_info_t *cfg);
	/**
 * @brief   设置 电阻校准最大电压 (V)
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_resistance_calib_max_voltage(motor_info_t *cfg, float value);

	/**
 * @brief   读取 峰值电流限制 (A)
 * @param   cfg 参数区指针
 * @return  峰值电流限制
 */
	float motor_info_get_current_lim(const motor_info_t *cfg);
	/**
 * @brief   设置 峰值电流限制 (A)
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_current_lim(motor_info_t *cfg, float value);

	/**
 * @brief   读取 电流环带宽 (Hz)
 * @param   cfg 参数区指针
 * @return  电流环带宽
 */
	float motor_info_get_current_control_bandwidth(const motor_info_t *cfg);
	/**
 * @brief   设置 电流环带宽 (Hz)
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_current_control_bandwidth(motor_info_t *cfg, float value);

	/**
 * @brief   读取 编码器类型
 * @param   cfg 参数区指针
 * @return  编码器类型
 */
	uint32_t motor_info_get_enc_type(const motor_info_t *cfg);
	/**
 * @brief   设置 编码器类型
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_enc_type(motor_info_t *cfg, uint32_t value);

	/**
 * @brief   读取 编码器分辨率 (CPR)
 * @param   cfg 参数区指针
 * @return  编码器分辨率
 */
	uint32_t motor_info_get_enc_lines(const motor_info_t *cfg);
	/**
 * @brief   设置 编码器分辨率 (CPR)
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_enc_lines(motor_info_t *cfg, uint32_t value);

	/**
 * @brief   读取 编码器计数方向
 * @param   cfg 参数区指针
 * @return  编码器计数方向
 */
	int32_t motor_info_get_enc_direction(const motor_info_t *cfg);
	/**
 * @brief   设置 编码器计数方向
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_enc_direction(motor_info_t *cfg, int32_t value);

	/**
 * @brief   读取 编码器初始位置偏移 (deg)
 * @param   cfg 参数区指针
 * @return  编码器初始位置偏移
 */
	float motor_info_get_enc_offset(const motor_info_t *cfg);
	/**
 * @brief   设置 编码器初始位置偏移 (deg)
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_enc_offset(motor_info_t *cfg, float value);

	/**
 * @brief   读取 电角度偏移 (rad)
 * @param   cfg 参数区指针
 * @return  电角度偏移
 */
	float motor_info_get_elec_angle_bias(const motor_info_t *cfg);
	/**
 * @brief   设置 电角度偏移 (rad)
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_elec_angle_bias(motor_info_t *cfg, float value);

	/**
 * @brief   读取 PWM载波频率 (Hz)
 * @param   cfg 参数区指针
 * @return  PWM载波频率
 */
	uint32_t motor_info_get_pwm_freq_hz(const motor_info_t *cfg);
	/**
 * @brief   设置 PWM载波频率 (Hz)
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_pwm_freq_hz(motor_info_t *cfg, uint32_t value);

	/**
 * @brief   读取 PWM死区时间 (ns)
 * @param   cfg 参数区指针
 * @return  PWM死区时间
 */
	float motor_info_get_dead_time_ns(const motor_info_t *cfg);
	/**
 * @brief   设置 PWM死区时间 (ns)
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_dead_time_ns(motor_info_t *cfg, float value);

	/**
 * @brief   读取 电流采样电阻 (ohm)
 * @param   cfg 参数区指针
 * @return  电流采样电阻
 */
	float motor_info_get_shunt_resistance(const motor_info_t *cfg);
	/**
 * @brief   设置 电流采样电阻 (ohm)
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_shunt_resistance(motor_info_t *cfg, float value);

	/**
 * @brief   读取 电流放大增益
 * @param   cfg 参数区指针
 * @return  电流放大增益
 */
	float motor_info_get_current_amp_gain(const motor_info_t *cfg);
	/**
 * @brief   设置 电流放大增益
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_current_amp_gain(motor_info_t *cfg, float value);

	/******************************************************************************
 * @brief   设备参数（CAN/UART）
 ******************************************************************************/
	/**
 * @brief   读取 设备零度 (rad)
 * @param   cfg 参数区指针
 * @return  设备零度
 */
	float motor_info_get_device_zero(const motor_info_t *cfg);
	/**
 * @brief   设置 设备零度 (rad)
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_device_zero(motor_info_t *cfg, float value);

	/**
 * @brief   读取 设备生产日期
 * @param   cfg 参数区指针
 * @return  设备生产日期
 */
	uint32_t motor_info_get_device_time(const motor_info_t *cfg);
	/**
 * @brief   设置 设备生产日期
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_device_time(motor_info_t *cfg, uint32_t value);

	/**
 * @brief   读取 CAN节点ID
 * @param   cfg 参数区指针
 * @return  CAN节点ID
 */
	uint32_t motor_info_get_can_id(const motor_info_t *cfg);
	/**
 * @brief   设置 CAN节点ID
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_can_id(motor_info_t *cfg, uint32_t value);

	/**
 * @brief   读取 CAN波特率 (bps)
 * @param   cfg 参数区指针
 * @return  CAN波特率
 */
	uint32_t motor_info_get_can_baudrate(const motor_info_t *cfg);
	/**
 * @brief   设置 CAN波特率 (bps)
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_can_baudrate(motor_info_t *cfg, uint32_t value);

	/**
 * @brief   读取 CAN通信超时 (s)
 * @param   cfg 参数区指针
 * @return  CAN通信超时
 */
	float motor_info_get_can_timeout_s(const motor_info_t *cfg);
	/**
 * @brief   设置 CAN通信超时 (s)
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_can_timeout_s(motor_info_t *cfg, float value);

	/**
 * @brief   读取 CAN FD使能
 * @param   cfg 参数区指针
 * @return  CAN FD使能
 */
	uint32_t motor_info_get_can_fd_enable(const motor_info_t *cfg);
	/**
 * @brief   设置 CAN FD使能
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_can_fd_enable(motor_info_t *cfg, uint32_t value);

	/**
 * @brief   读取 CAN FD数据波特率 (bps)
 * @param   cfg 参数区指针
 * @return  CAN FD数据波特率
 */
	uint32_t motor_info_get_can_fd_baudrate(const motor_info_t *cfg);
	/**
 * @brief   设置 CAN FD数据波特率 (bps)
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_can_fd_baudrate(motor_info_t *cfg, uint32_t value);

	/**
 * @brief   读取 UART波特率 (bps)
 * @param   cfg 参数区指针
 * @return  UART波特率
 */
	uint32_t motor_info_get_uart_baudrate(const motor_info_t *cfg);
	/**
 * @brief   设置 UART波特率 (bps)
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_uart_baudrate(motor_info_t *cfg, uint32_t value);

	/******************************************************************************
 * @brief   控制参数（三环PID+前馈+滤波）
 ******************************************************************************/
	/**
 * @brief   读取 d轴比例增益 (V/A)
 * @param   cfg 参数区指针
 * @return  d轴比例增益
 */
	float motor_info_get_kp_ld(const motor_info_t *cfg);
	/**
 * @brief   设置 d轴比例增益 (V/A)
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_kp_ld(motor_info_t *cfg, float value);

	/**
 * @brief   读取 d轴积分增益 (V/(A·s))
 * @param   cfg 参数区指针
 * @return  d轴积分增益
 */
	float motor_info_get_ki_ld(const motor_info_t *cfg);
	/**
 * @brief   设置 d轴积分增益 (V/(A·s))
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_ki_ld(motor_info_t *cfg, float value);

	/**
 * @brief   读取 q轴比例增益 (V/A)
 * @param   cfg 参数区指针
 * @return  q轴比例增益
 */
	float motor_info_get_kp_lq(const motor_info_t *cfg);
	/**
 * @brief   设置 q轴比例增益 (V/A)
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_kp_lq(motor_info_t *cfg, float value);

	/**
 * @brief   读取 q轴积分增益 (V/(A·s))
 * @param   cfg 参数区指针
 * @return  q轴积分增益
 */
	float motor_info_get_ki_lq(const motor_info_t *cfg);
	/**
 * @brief   设置 q轴积分增益 (V/(A·s))
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_ki_lq(motor_info_t *cfg, float value);

	/**
 * @brief   读取 积分限幅 (V)
 * @param   cfg 参数区指针
 * @return  积分限幅
 */
	float motor_info_get_integral_limit(const motor_info_t *cfg);
	/**
 * @brief   设置 积分限幅 (V)
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_integral_limit(motor_info_t *cfg, float value);

	/**
 * @brief   读取 dq轴解耦增益
 * @param   cfg 参数区指针
 * @return  dq轴解耦增益
 */
	float motor_info_get_decoupling_gain(const motor_info_t *cfg);
	/**
 * @brief   设置 dq轴解耦增益
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_decoupling_gain(motor_info_t *cfg, float value);

	/**
 * @brief   读取 死区补偿电压 (V)
 * @param   cfg 参数区指针
 * @return  死区补偿电压
 */
	float motor_info_get_comp_du_V(const motor_info_t *cfg);
	/**
 * @brief   设置 死区补偿电压 (V)
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_comp_du_V(motor_info_t *cfg, float value);

	/**
 * @brief   读取 PWM最大占空比
 * @param   cfg 参数区指针
 * @return  PWM最大占空比
 */
	float motor_info_get_pwm_duty_max(const motor_info_t *cfg);
	/**
 * @brief   设置 PWM最大占空比
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_pwm_duty_max(motor_info_t *cfg, float value);

	/**
 * @brief   读取 速度环比例增益 (A/(rad/s))
 * @param   cfg 参数区指针
 * @return  速度环比例增益
 */
	float motor_info_get_kp_s(const motor_info_t *cfg);
	/**
 * @brief   设置 速度环比例增益 (A/(rad/s))
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_kp_s(motor_info_t *cfg, float value);

	/**
 * @brief   读取 速度环积分增益 (A/rad)
 * @param   cfg 参数区指针
 * @return  速度环积分增益
 */
	float motor_info_get_ki_s(const motor_info_t *cfg);
	/**
 * @brief   设置 速度环积分增益 (A/rad)
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_ki_s(motor_info_t *cfg, float value);

	/**
 * @brief   读取 速度环积分限幅 (A)
 * @param   cfg 参数区指针
 * @return  速度环积分限幅
 */
	float motor_info_get_speed_integral_limit(const motor_info_t *cfg);
	/**
 * @brief   设置 速度环积分限幅 (A)
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_speed_integral_limit(motor_info_t *cfg, float value);

	/**
 * @brief   读取 速度前馈系数
 * @param   cfg 参数区指针
 * @return  速度前馈系数
 */
	float motor_info_get_vff(const motor_info_t *cfg);
	/**
 * @brief   设置 速度前馈系数
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_vff(motor_info_t *cfg, float value);

	/**
 * @brief   读取 加速度前馈系数
 * @param   cfg 参数区指针
 * @return  加速度前馈系数
 */
	float motor_info_get_aff(const motor_info_t *cfg);
	/**
 * @brief   设置 加速度前馈系数
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_aff(motor_info_t *cfg, float value);

	/**
 * @brief   读取 加加速度前馈系数
 * @param   cfg 参数区指针
 * @return  加加速度前馈系数
 */
	float motor_info_get_jerk_ff(const motor_info_t *cfg);
	/**
 * @brief   设置 加加速度前馈系数
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_jerk_ff(motor_info_t *cfg, float value);

	/**
 * @brief   读取 速度滤波系数
 * @param   cfg 参数区指针
 * @return  速度滤波系数
 */
	float motor_info_get_speed_filter_alpha(const motor_info_t *cfg);
	/**
 * @brief   设置 速度滤波系数
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_speed_filter_alpha(motor_info_t *cfg, float value);

	/**
 * @brief   读取 速度滤波使能
 * @param   cfg 参数区指针
 * @return  速度滤波使能
 */
	uint32_t motor_info_get_speed_filter_enable(const motor_info_t *cfg);
	/**
 * @brief   设置 速度滤波使能
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_speed_filter_enable(motor_info_t *cfg, uint32_t value);

	/**
 * @brief   读取 位置环比例增益 (Hz)
 * @param   cfg 参数区指针
 * @return  位置环比例增益
 */
	float motor_info_get_kp_p(const motor_info_t *cfg);
	/**
 * @brief   设置 位置环比例增益 (Hz)
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_kp_p(motor_info_t *cfg, float value);

	/**
 * @brief   读取 位置环积分增益 (1/s)
 * @param   cfg 参数区指针
 * @return  位置环积分增益
 */
	float motor_info_get_ki_p(const motor_info_t *cfg);
	/**
 * @brief   设置 位置环积分增益 (1/s)
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_ki_p(motor_info_t *cfg, float value);

	/**
 * @brief   读取 位置环积分限幅 (rad)
 * @param   cfg 参数区指针
 * @return  位置环积分限幅
 */
	float motor_info_get_position_integral_limit(const motor_info_t *cfg);
	/**
 * @brief   设置 位置环积分限幅 (rad)
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_position_integral_limit(motor_info_t *cfg, float value);

	/**
 * @brief   读取 位置滤波系数
 * @param   cfg 参数区指针
 * @return  位置滤波系数
 */
	float motor_info_get_position_filter_alpha(const motor_info_t *cfg);
	/**
 * @brief   设置 位置滤波系数
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_position_filter_alpha(motor_info_t *cfg, float value);

	/**
 * @brief   读取 位置滤波使能
 * @param   cfg 参数区指针
 * @return  位置滤波使能
 */
	uint32_t motor_info_get_position_filter_enable(const motor_info_t *cfg);
	/**
 * @brief   设置 位置滤波使能
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_position_filter_enable(motor_info_t *cfg, uint32_t value);

	/**
 * @brief   读取 跟随误差限制 (P)
 * @param   cfg 参数区指针
 * @return  跟随误差限制
 */
	float motor_info_get_following_error_limit(const motor_info_t *cfg);
	/**
 * @brief   设置 跟随误差限制 (P)
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_following_error_limit(motor_info_t *cfg, float value);

	/**
 * @brief   读取 PID来源位掩码
 * @param   cfg 参数区指针
 * @return  PID来源位掩码
 */
	uint32_t motor_info_get_pid_source_mask(const motor_info_t *cfg);
	/**
 * @brief   设置 PID来源位掩码
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_pid_source_mask(motor_info_t *cfg, uint32_t value);

	/******************************************************************************
 * @brief   保护与通信参数
 ******************************************************************************/
	/**
 * @brief   读取 过流保护阈值 (A)
 * @param   cfg 参数区指针
 * @return  过流保护阈值
 */
	float motor_info_get_over_current_A(const motor_info_t *cfg);
	/**
 * @brief   设置 过流保护阈值 (A)
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_over_current_A(motor_info_t *cfg, float value);

	/**
 * @brief   读取 过压保护阈值 (V)
 * @param   cfg 参数区指针
 * @return  过压保护阈值
 */
	float motor_info_get_over_voltage_V(const motor_info_t *cfg);
	/**
 * @brief   设置 过压保护阈值 (V)
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_over_voltage_V(motor_info_t *cfg, float value);

	/**
 * @brief   读取 欠压保护阈值 (V)
 * @param   cfg 参数区指针
 * @return  欠压保护阈值
 */
	float motor_info_get_under_voltage_V(const motor_info_t *cfg);
	/**
 * @brief   设置 欠压保护阈值 (V)
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_under_voltage_V(motor_info_t *cfg, float value);

	/**
 * @brief   读取 驱动器过温阈值 (℃)
 * @param   cfg 参数区指针
 * @return  驱动器过温阈值
 */
	float motor_info_get_over_temp_drive(const motor_info_t *cfg);
	/**
 * @brief   设置 驱动器过温阈值 (℃)
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_over_temp_drive(motor_info_t *cfg, float value);

	/**
 * @brief   读取 电机过温阈值 (℃)
 * @param   cfg 参数区指针
 * @return  电机过温阈值
 */
	float motor_info_get_over_temp_motor(const motor_info_t *cfg);
	/**
 * @brief   设置 电机过温阈值 (℃)
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_over_temp_motor(motor_info_t *cfg, float value);

	/**
 * @brief   读取 欠温保护阈值 (℃)
 * @param   cfg 参数区指针
 * @return  欠温保护阈值
 */
	float motor_info_get_under_temp_d(const motor_info_t *cfg);
	/**
 * @brief   设置 欠温保护阈值 (℃)
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_under_temp_d(motor_info_t *cfg, float value);

	/**
 * @brief   读取 过速保护阈值 (rad/s)
 * @param   cfg 参数区指针
 * @return  过速保护阈值
 */
	float motor_info_get_over_speed_rad_s(const motor_info_t *cfg);
	/**
 * @brief   设置 过速保护阈值 (rad/s)
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_over_speed_rad_s(motor_info_t *cfg, float value);

	/**
 * @brief   读取 位置跟随误差保护 (P)
 * @param   cfg 参数区指针
 * @return  位置跟随误差保护
 */
	int32_t motor_info_get_position_following_error_p(const motor_info_t *cfg);
	/**
 * @brief   设置 位置跟随误差保护 (P)
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_position_following_error_p(motor_info_t *cfg, int32_t value);

	/**
 * @brief   读取 位置下限 (P)
 * @param   cfg 参数区指针
 * @return  位置下限
 */
	int32_t motor_info_get_pos_limit_min(const motor_info_t *cfg);
	/**
 * @brief   设置 位置下限 (P)
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_pos_limit_min(motor_info_t *cfg, int32_t value);

	/**
 * @brief   读取 位置上限 (P)
 * @param   cfg 参数区指针
 * @return  位置上限
 */
	int32_t motor_info_get_pos_limit_max(const motor_info_t *cfg);
	/**
 * @brief   设置 位置上限 (P)
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_pos_limit_max(motor_info_t *cfg, int32_t value);

	/**
 * @brief   读取 保护使能掩码
 * @param   cfg 参数区指针
 * @return  保护使能掩码
 */
	uint32_t motor_info_get_error_enable_mask(const motor_info_t *cfg);
	/**
 * @brief   设置 保护使能掩码
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_error_enable_mask(motor_info_t *cfg, uint32_t value);

	/******************************************************************************
 * @brief   高级算法参数（MIT/力控/回零）
 ******************************************************************************/
	/**
 * @brief   读取 MIT位置刚度 (Nm/rad)
 * @param   cfg 参数区指针
 * @return  MIT位置刚度
 */
	float motor_info_get_mit_kp(const motor_info_t *cfg);
	/**
 * @brief   设置 MIT位置刚度 (Nm/rad)
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_mit_kp(motor_info_t *cfg, float value);

	/**
 * @brief   读取 MIT速度阻尼 (Nm/(rad/s))
 * @param   cfg 参数区指针
 * @return  MIT速度阻尼
 */
	float motor_info_get_mit_kd(const motor_info_t *cfg);
	/**
 * @brief   设置 MIT速度阻尼 (Nm/(rad/s))
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_mit_kd(motor_info_t *cfg, float value);

	/**
 * @brief   读取 MIT最大电流 (A)
 * @param   cfg 参数区指针
 * @return  MIT最大电流
 */
	float motor_info_get_mit_max_current(const motor_info_t *cfg);
	/**
 * @brief   设置 MIT最大电流 (A)
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_mit_max_current(motor_info_t *cfg, float value);

	/**
 * @brief   读取 MIT前馈力矩 (Nm)
 * @param   cfg 参数区指针
 * @return  MIT前馈力矩
 */
	float motor_info_get_mit_feedforward_torque(const motor_info_t *cfg);
	/**
 * @brief   设置 MIT前馈力矩 (Nm)
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_mit_feedforward_torque(motor_info_t *cfg, float value);

	/**
 * @brief   读取 力控比例增益 (A/Nm)
 * @param   cfg 参数区指针
 * @return  力控比例增益
 */
	float motor_info_get_force_kp(const motor_info_t *cfg);
	/**
 * @brief   设置 力控比例增益 (A/Nm)
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_force_kp(motor_info_t *cfg, float value);

	/**
 * @brief   读取 力控积分增益 (A/(Nm·s))
 * @param   cfg 参数区指针
 * @return  力控积分增益
 */
	float motor_info_get_force_ki(const motor_info_t *cfg);
	/**
 * @brief   设置 力控积分增益 (A/(Nm·s))
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_force_ki(motor_info_t *cfg, float value);

	/**
 * @brief   读取 力控力矩限制 (Nm)
 * @param   cfg 参数区指针
 * @return  力控力矩限制
 */
	float motor_info_get_force_limit(const motor_info_t *cfg);
	/**
 * @brief   设置 力控力矩限制 (Nm)
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_force_limit(motor_info_t *cfg, float value);

	/**
 * @brief   读取 力控使能
 * @param   cfg 参数区指针
 * @return  力控使能
 */
	uint32_t motor_info_get_force_control_enable(const motor_info_t *cfg);
	/**
 * @brief   设置 力控使能
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_force_control_enable(motor_info_t *cfg, uint32_t value);

	/**
 * @brief   读取 回零方法
 * @param   cfg 参数区指针
 * @return  回零方法
 */
	uint32_t motor_info_get_homing_method(const motor_info_t *cfg);
	/**
 * @brief   设置 回零方法
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_homing_method(motor_info_t *cfg, uint32_t value);

	/**
 * @brief   读取 回零速度 (rad/s)
 * @param   cfg 参数区指针
 * @return  回零速度
 */
	float motor_info_get_homing_speed(const motor_info_t *cfg);
	/**
 * @brief   设置 回零速度 (rad/s)
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_homing_speed(motor_info_t *cfg, float value);

	/**
 * @brief   读取 回零偏移 (rad)
 * @param   cfg 参数区指针
 * @return  回零偏移
 */
	float motor_info_get_homing_offset(const motor_info_t *cfg);
	/**
 * @brief   设置 回零偏移 (rad)
 * @param   cfg 参数区指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=空指针或越界
 */
	int motor_info_set_homing_offset(motor_info_t *cfg, float value);

/******************************************************************************
 * @brief   协议分发表 (param_id -> get/set), 供 0xE6-0xEB 单参读写
 ******************************************************************************/
/* dispatch 返回码 */
#define MOTOR_INFO_DISPATCH_OK       0
#define MOTOR_INFO_DISPATCH_E_BAD_ID -1
#define MOTOR_INFO_DISPATCH_E_BOUNDS -2
#define MOTOR_INFO_DISPATCH_E_RO     -3

	/**
 * @brief 按 param_id 读单个参数, 值写入 out4(固定4B, 零填充)
 * @param  pid      参数ID(CSV Index)
 * @param  cfg      motor_info_t 指针
 * @param  out4     输出缓冲(4字节)
 * @param  out_type 输出协议类型码(0=u8..6=f32)
 * @param  out_len  输出实际值字节数(1/2/4)
 * @return 0=成功, -1=未知pid/空指针
 */
	int motor_info_dispatch_read(uint16_t pid, const motor_info_t *cfg, uint8_t out4[4], uint8_t *out_type, uint8_t *out_len);

	/**
 * @brief 按 param_id 写单个参数, 值取自 in4(固定4B)
 * @param  pid  参数ID(CSV Index)
 * @param  cfg  motor_info_t 指针
 * @param  in4  输入缓冲(4字节)
 * @param  len  实际有效字节数(仅校验, 取低4B)
 * @return 0=成功, -1=未知pid/空指针, -2=越界, -3=只读
 */
	int motor_info_dispatch_write(uint16_t pid, motor_info_t *cfg, const uint8_t in4[4], uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_INFO_H__ */
