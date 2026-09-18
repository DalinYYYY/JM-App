/**
 * @file    motor_info.h
 * @brief   MotorInfo 配置参数 API 接口（1024B 整块空间）
 * @date    2026-09-11
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

/* ===== 功能裁剪开关（默认 0=裁剪以节省 Flash，置 1 恢复完整功能，可外部覆盖）===== */
#ifndef MOTOR_INFO_EN_INIT_DEFAULTS
#define MOTOR_INFO_EN_INIT_DEFAULTS 0 /* init: 1=填充块索引表+各参数默认值, 0=仅全零+版本号 */
#endif
#ifndef MOTOR_INFO_EN_VALIDATE
#define MOTOR_INFO_EN_VALIDATE 0 /* validate: 1=完整范围校验, 0=直接返回通过 */
#endif
#ifndef MOTOR_INFO_EN_PRINT
#define MOTOR_INFO_EN_PRINT 0 /* print: 1=打印全部参数, 0=空实现 */
#endif

/* 字节对齐宏：保证 RAM 布局与 Flash 字节布局完全一致，可整块 memcpy */
#ifndef __ALIGNED_4
#define __ALIGNED_4 __attribute__((aligned(4)))
#endif
#ifndef __ALIGNED_8
#define __ALIGNED_8 __attribute__((aligned(8)))
#endif

/* ===== 自动生成元信息 ===== */
#define MOTOR_INFO_GEN_DATE      "2026-09-11"
#define MOTOR_INFO_PARAM_COUNT   113
#define MOTOR_INFO_VERSION_MAJOR 1
#define MOTOR_INFO_VERSION_MINOR 0

/* ===== Flash 区域布局常量 ===== */
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
#define MOTOR_INFO_BLOCK_ADVANCEDALGOPARAM_SIZE   128u

	/* ===== 全局头部与子块索引表 ===== */
	typedef struct __ALIGNED_4
	{
		uint32_t offset; /* 子块偏移(字节) */
		uint32_t size;   /* 子块大小(字节) */
	} BlockIndex_t;

	typedef struct __ALIGNED_4
	{
		uint16_t version_major;               /* 主版本 */
		uint16_t version_minor;               /* 次版本 */
		uint32_t crc32;                       /* 参数区 CRC32(校验范围跳过本字段) */
		BlockIndex_t blocks[MAX_BLOCK_COUNT]; /* 6*8 = 48B */
		uint32_t reserved[2];                 /* 8B 填充凑足 64B */
	} ParamHeader_t;                          /* 64B */

	/* ===== 各子块数据结构（字段顺序与 CSV 一致，8 字节对齐以支持 u64 成员）===== */
	/**
 * @brief   系统级参数
 * @details 块大小 64B，已用 24B，预留 40B
 */
	typedef struct __ALIGNED_8
	{
		uint32_t config_version;    /* 配置版本号  [参数表版本号 主.次=(高16位.低16位) 升级时比对校验] */
		uint32_t enable_uart;       /* 接口使能  [通信接口使能掩码 bit0:UART bit1:CAN bit2:CANFD bit3:USB] */
		uint32_t enable_bus_sensor; /* 总线传感器使能  [使能母线传感器(母线电压/电流监测反馈)] */
		uint32_t safety_limit;      /* 安全限制使能  [安全限幅使能 0禁1启 调试期建议关闭] */
		uint32_t total_runtime_s;   /* 累计运行时间 (s)  [累计运行时间 掉电保存 运维统计 定期写入防频繁擦写] */
		uint32_t save_count;        /* 固化次数  [固化(保存)次数 每次0xEA保存成功自增 达寿命阈值拒绝写入] */
		uint32_t reserved[10];      /* 预留 40B */
	} SystemParam_t;

	/**
 * @brief   电机标定参数（含减速器/编码器/功率级/电流采样）
 * @details 块大小 128B，已用 116B，预留 12B
 */
	typedef struct __ALIGNED_8
	{
		uint32_t is_calibrated;             /* 电机是否校准  [电机标定标记 0未校准 1已校准] */
		uint32_t pole_pairs;                /* 电机极对数 p (pairs)  [电机极对数 电角度=机械角×极对数] */
		uint32_t motor_type;                /* 电机类型  [电机类型 0:SPMSM 1:IPMSM 2:BLDC] */
		uint32_t direction;                 /* 电机方向  [电机输出方向 0正向 1反向] */
		float phase_resistance;             /* 相电阻 Rs (ohm)  [相电阻(标定) 电流环Ki=ωc·R依赖] */
		float phase_inductance_d;           /* d轴相电感 Ld (H)  [d轴电感(标定) 电流环Kp=ωc·Ld依赖] */
		float phase_inductance_q;           /* q轴相电感 Lq (H)  [q轴电感(标定) 电流环Kp=ωc·Lq依赖 一般略大于Ld] */
		float flux_linkage;                 /* 永磁体磁链 ψf (Wb)  [永磁体磁链 反电势/BEMF前馈计算依赖] */
		float torque_constant;              /* 转矩常数 Kt (Nm/A)  [转矩常数Kt 速度环Kp=J·ωc/Kt依赖] */
		float rotor_inertia;                /* 转子惯量 J (kg·m²)  [转子惯量 速度环Kp/Ki依赖] */
		float friction_coulomb;             /* 库仑摩擦力矩 Tf (Nm)  [库仑摩擦力矩 摩擦前馈/辨识] */
		float friction_viscous;             /* 粘滞摩擦系数 B (Nm/(rad/s))  [粘滞摩擦系数 摩擦前馈/辨识] */
		float gear_ratio;                   /* 减速比 i  [减速比 输入转速/输出转速 惯量与力矩折算依赖] */
		float gear_efficiency;              /* 减速器效率 η  [减速器效率 输出力矩折算] */
		float calibration_current;          /* 校准电流 (A)  [R/L标定施加电流 注意发热] */
		float resistance_calib_max_voltage; /* 电阻校准最大电压 (V)  [R标定电压限幅 防过流] */
		float current_lim;                  /* 峰值电流限制 Imax (A)  [硬件峰值电流限幅 瞬时保护] */
		float current_control_bandwidth;    /* 电流环带宽 ωc (Hz)  [电流环带宽 autotune据此计算电流环PID] */
		uint32_t enc_type;                  /* 编码器类型  [编码器类型 0:ABZ 1:MT6701 2:MT6835 3:霍尔] */
		uint32_t enc_lines;                 /* 编码器分辨率 (CPR)  [编码器分辨率 SPI绝对值为分辨率 增量式CPR] */
		int32_t enc_direction;              /* 编码器计数方向  [编码器计数方向 1正向 -1反向] */
		float enc_offset;                   /* 编码器位置偏移 θ0 (deg)  [编码器零点偏移 校准后保存] */
		float elec_angle_bias;              /* 电角度偏置 θe0 (rad)  [电角度偏置 FOC换相对齐 校准后保存] */
		uint32_t pwm_freq_hz;               /* PWM载波频率 fpwm (Hz)  [PWM载波频率 决定开关与电流采样频率] */
		float dead_time_ns;                 /* PWM死区时间 Td (ns)  [PWM死区时间 防桥臂直通] */
		float shunt_resistance;             /* 电流采样电阻 Rsh (ohm)  [电流采样电阻阻值 电流计算依赖] */
		float current_amp_gain;             /* 电流放大增益 G  [电流采样运放增益 电流计算依赖] */
		float peak_current;                 /* 峰值电流 Ipk (A)  [电机峰值电流限幅 切换电机型号需同步] */
		float max_speed;                    /* 最大转速 ωmax (rad/s)  [电机最大转速限幅 切换电机型号需同步] */
		uint32_t reserved[3];               /* 预留 12B */
	} MotorCalibParam_t;

	/**
 * @brief   设备参数（CAN/UART）
 * @details 块大小 64B，已用 32B，预留 32B
 */
	typedef struct __ALIGNED_8
	{
		float device_zero;        /* 设备零度 (rad)  [设备机械零点位置] */
		uint32_t device_time;     /* 设备生产日期  [设备生产日期 YYYYMMDD格式] */
		uint32_t can_id;          /* CAN节点ID  [CAN节点ID 0保留为广播] */
		uint32_t can_baudrate;    /* CAN波特率 (bps)  [CAN经典帧波特率] */
		float can_timeout_s;      /* CAN通信超时 (s)  [CAN通信超时 超过判失效 0禁用超时] */
		uint32_t can_fd_enable;   /* CAN FD使能  [CAN FD使能 0传统CAN 1:CAN FD] */
		uint32_t can_fd_baudrate; /* CAN FD数据波特率 (bps)  [CAN FD数据段波特率] */
		uint32_t uart_baudrate;   /* UART波特率 (bps)  [UART串口波特率] */
		uint32_t reserved[8];     /* 预留 32B */
	} DeviceParam_t;

	/**
 * @brief   控制参数（三环PID+前馈+滤波）
 * @details 块大小 320B，已用 104B，预留 216B
 */
	typedef struct __ALIGNED_8
	{
		float kp_ld;                     /* d轴比例增益 Kp_d (V/A)  [电流环d轴P增益 autotune默认Kp=ωc·Ld] */
		float ki_ld;                     /* d轴积分增益 Ki_d (V/(A·s))  [电流环d轴I增益 autotune默认Ki=ωc·R] */
		float kp_lq;                     /* q轴比例增益 Kp_q (V/A)  [电流环q轴P增益 autotune默认Kp=ωc·Lq] */
		float ki_lq;                     /* q轴积分增益 Ki_q (V/(A·s))  [电流环q轴I增益 autotune默认Ki=ωc·R] */
		float integral_limit;            /* 积分限幅 (V)  [电流环积分输出限幅 建议Kp_q×额定电流×1.5] */
		float decoupling_gain;           /* dq轴解耦增益  [dq轴解耦增益 0~1 调试期可适当增大] */
		float comp_du_V;                 /* 死区补偿电压 Δu (V)  [死区补偿电压 补偿死区非线性] */
		float pwm_duty_max;              /* PWM最大占空比 Dmax  [PWM最大占空比 防过调制] */
		float kp_s;                      /* 速度环比例增益 Kp_s (A/(rad/s))  [速度环P增益 autotune默认Kp=J·ωc/Kt] */
		float ki_s;                      /* 速度环积分增益 Ki_s (A/rad)  [速度环I增益 autotune默认Ki=J·ωc²/(4Kt)] */
		float speed_integral_limit;      /* 速度环积分限幅 (A)  [速度环积分限幅 建议额定电流×0.5] */
		float vff;                       /* 速度前馈系数  [速度前馈系数 0~1 减小跟随误差] */
		float aff;                       /* 加速度前馈系数  [加速度前馈系数 0~1 改善动态性能] */
		float jerk_ff;                   /* 加加速度前馈系数  [加加速度前馈系数 0~1] */
		float speed_filter_alpha;        /* 速度滤波系数 α  [速度一阶低通滤波系数 越大滤波越弱] */
		uint32_t speed_filter_enable;    /* 速度滤波使能  [速度滤波使能 0禁1启] */
		float kp_p;                      /* 位置环比例增益 Kp_p (Hz)  [位置环P增益(带宽Hz) autotune默认Kp=2π·f] */
		float ki_p;                      /* 位置环积分增益 Ki_p (1/s)  [位置环I增益 一般为0] */
		float position_integral_limit;   /* 位置环积分限幅 (rad)  [位置环积分输出限幅] */
		float position_filter_alpha;     /* 位置滤波系数 α  [位置一阶低通滤波系数] */
		uint32_t position_filter_enable; /* 位置滤波使能  [位置滤波使能 0禁1启] */
		float following_error_limit;     /* 跟随误差限制 (P)  [位置跟随误差保护阈值 调试期可放大] */
		uint32_t decouple_algo;          /* 交叉解耦算法  [交叉解耦算法 0:NONE 1:FEEDFORWARD 2:FEEDBACK] */
		uint32_t bemf_ff_enable;         /* 反电势前馈使能  [反电势前馈使能 0禁1启] */
		uint32_t deadtime_comp_enable;   /* 死区补偿使能  [死区补偿使能 0禁1启] */
		uint32_t pid_source_mask;        /* PID来源位掩码  [PID来源掩码 bit[3:0]电流环 bit[7:4]速度环 bit[11:8]位置环 0默认 1Flash 2理论] */
		uint32_t reserved[54];           /* 预留 216B */
	} ControlParam_t;

	/**
 * @brief   保护与通信参数（含故障分级阈值/三级使能掩码）
 * @details 块大小 128B，已用 92B，预留 36B
 */
	typedef struct __ALIGNED_8
	{
		uint32_t protect_enable;            /* 保护总使能  [保护功能总使能(块首) 0=全部禁用 1=启用] */
		float over_current_A;               /* 过流保护阈值 (A)  [过流保护阈值 母线/相电流超此值触发] */
		float over_voltage_V;               /* 过压保护阈值 (V)  [过压保护阈值 母线电压超此值触发] */
		float under_voltage_V;              /* 欠压保护阈值 (V)  [欠压保护阈值 母线电压低于此值触发] */
		float over_speed_rad_s;             /* 过速保护阈值 (rad/s)  [过速保护阈值] */
		int32_t position_following_error_p; /* 位置跟随误差保护 (P)  [位置跟随误差保护阈值(编码器计数)] */
		int32_t pos_limit_min;              /* 位置下限 (P)  [硬件位置下限] */
		int32_t pos_limit_max;              /* 位置上限 (P)  [硬件位置上限] */
		float ov_mid_v;                     /* 中度过压阈值 (V)  [超过此值且低于故障阈值触发中度过压(异常级降功率)] */
		float uv_mid_v;                     /* 中度欠压阈值 (V)  [低于此值且高于故障阈值触发中度欠压] */
		float ibus_over_A;                  /* 母线过流阈值 (A)  [母线电流瞬时过流(故障级停机)] */
		float ibus_cont_A;                  /* 持续过流阈值 (A)  [母线电流持续过流(异常级降功率)] */
		float ibus_cont_time_s;             /* 持续过流时长 (s)  [持续过流判定时间] */
		float temp_fet_over_d;              /* 驱动器过温阈值 (℃)  [驱动器过温降功率(异常级)] */
		float temp_motor_over_d;            /* 电机过温熔断阈值 (℃)  [电机过温熔断(故障级停机)] */
		float temp_motor_hot_d;             /* 绕组过热阈值 (℃)  [绕组过热降功率(异常级)] */
		float temp_under_d;                 /* 低温阈值 (℃)  [低于此值禁止使能(预热解除)] */
		uint32_t mask_critical1;            /* 故障级使能掩码低32位  [故障级检测使能掩码 bit0-31 与mask_critical2合成64bit 非0=启用] */
		uint32_t mask_critical2;            /* 故障级使能掩码高32位  [故障级检测使能掩码 bit32-63 与mask_critical1合成64bit 非0=启用] */
		uint32_t mask_exception1;           /* 异常级使能掩码低32位  [异常级检测使能掩码 bit0-31 与mask_exception2合成64bit 非0=启用] */
		uint32_t mask_exception2;           /* 异常级使能掩码高32位  [异常级检测使能掩码 bit32-63 与mask_exception1合成64bit 非0=启用] */
		uint32_t mask_warning1;             /* 警告级使能掩码低32位  [警告级检测使能掩码 bit0-31 与mask_warning2合成64bit 非0=启用] */
		uint32_t mask_warning2;             /* 警告级使能掩码高32位  [警告级检测使能掩码 bit32-63 与mask_warning1合成64bit 非0=启用] */
		uint32_t reserved[9];               /* 预留 36B */
	} ProtectCommParam_t;

	/**
 * @brief   高级算法参数（MIT/力控/回零/缓启动）
 * @details 块大小 128B，已用 84B，预留 44B
 */
	typedef struct __ALIGNED_8
	{
		float mit_kp;                  /* MIT位置刚度 Kp (Nm/rad)  [MIT模式位置刚度 Kp 决定弹性力矩Kp·Δθ] */
		float mit_kd;                  /* MIT速度阻尼 Kd (Nm/(rad/s))  [MIT模式速度阻尼 Kd 决定阻尼力矩Kd·ω] */
		float mit_max_current;         /* MIT最大电流 Imax (A)  [MIT模式输出电流上限] */
		float mit_feedforward_torque;  /* MIT前馈力矩 τff (Nm)  [MIT模式前馈力矩偏置] */
		float force_kp;                /* 力控比例增益 (A/Nm)  [力控比例增益 力矩误差→电流] */
		float force_ki;                /* 力控积分增益 (A/(Nm·s))  [力控积分增益 消除稳态力矩误差] */
		float force_limit;             /* 力控力矩限制 (Nm)  [力控输出力矩上限] */
		uint32_t force_control_enable; /* 力控使能  [力控使能 0禁1启] */
		uint32_t homing_method;        /* 回零方法  [回零方式 0当前位置回零 1限位回零] */
		float homing_speed;            /* 回零速度 (rad/s)  [回零运动速度] */
		float homing_offset;           /* 回零偏移 (rad)  [回零后位置偏移补偿] */
		uint32_t softstart_valid;      /* 缓启动配置有效标志  [缓启动配置迁移标志 0未配置(用编译期默认) 首次0xA3写入置1] */
		uint32_t softstart_enable;     /* 缓启动总开关  [缓启动渐变总开关 0禁用(目标直接透传) 1启用] */
		uint32_t softstart_shape;      /* 缓启动渐变形状  [渐变形状 0线性恒斜率 1S曲线(起停柔和时长×1.5)] */
		uint32_t softstart_duration;   /* 缓启动固定时长 (次)  [兜底渐变时长(控制环调用次数) 仅rate全禁用时生效] */
		float softstart_pos_rate;      /* 缓启动位置速率 (rad/s)  [位置目标变化速率上限 过渡时长=|Δ|/rate] */
		float softstart_vel_rate;      /* 缓启动速度速率 (rad/s²)  [速度目标变化速率上限(加速度限值)] */
		float softstart_torque_rate;   /* 缓启动力矩速率 (Nm/s)  [力矩目标变化速率上限] */
		float softstart_current_rate;  /* 缓启动电流速率 (A/s)  [电流目标变化速率上限] */
		uint32_t cogging_comp_enable;  /* 齿槽补偿使能  [齿槽转矩补偿使能 表CRC有效时生效 L5.1标定成功自动置1] */
		float cogging_comp_gain;       /* 齿槽补偿增益  [齿槽补偿增益 负值=反相(标定方向校验用) 幅值现场微调 欠补加大过补减小] */
		uint32_t reserved[11];         /* 预留 44B */
	} AdvancedAlgoParam_t;

	/* ===== 主参数区联合体：1024B 整块空间 ===== */
	/* 8 字节对齐：被 (u64*) 强转传给 Flash 读写时，保证 u64 访问不产生未对齐故障 */
	typedef union __ALIGNED_8
	{
		uint8_t raw[PARAM_AREA_SIZE]; /* 原始字节数组，可直接 memcpy 到 Flash */
		struct __ALIGNED_8
		{
			ParamHeader_t header;            /* 0x0000  64B */
			SystemParam_t system;            /* 0x0040  64B  系统级参数 */
			MotorCalibParam_t motor_calib;   /* 0x0080  128B  电机标定参数（含减速器/编码器/功率级/电流采样） */
			DeviceParam_t device;            /* 0x0100  64B  设备参数（CAN/UART） */
			ControlParam_t control;          /* 0x0140  320B  控制参数（三环PID+前馈+滤波） */
			ProtectCommParam_t protect_comm; /* 0x0280  128B  保护与通信参数（含故障分级阈值/三级使能掩码） */
			AdvancedAlgoParam_t advanced;    /* 0x0300  128B  高级算法参数（MIT/力控/回零/缓启动） */
			uint8_t reserved[256];           /* 0x0340  256B  末尾预留 */
		} blocks;
	} motor_info_t;                          /* 1024B */

/* ===== Parameter IDs (CSV Index) ===== */
#define MOTOR_INFO_PID_CONFIG_VERSION               0u
#define MOTOR_INFO_PID_ENABLE_UART                  1u
#define MOTOR_INFO_PID_ENABLE_BUS_SENSOR            2u
#define MOTOR_INFO_PID_SAFETY_LIMIT                 3u
#define MOTOR_INFO_PID_TOTAL_RUNTIME_S              4u
#define MOTOR_INFO_PID_SAVE_COUNT                   5u
#define MOTOR_INFO_PID_IS_CALIBRATED                16u
#define MOTOR_INFO_PID_POLE_PAIRS                   17u
#define MOTOR_INFO_PID_MOTOR_TYPE                   18u
#define MOTOR_INFO_PID_DIRECTION                    19u
#define MOTOR_INFO_PID_PHASE_RESISTANCE             20u
#define MOTOR_INFO_PID_PHASE_INDUCTANCE_D           21u
#define MOTOR_INFO_PID_PHASE_INDUCTANCE_Q           22u
#define MOTOR_INFO_PID_FLUX_LINKAGE                 23u
#define MOTOR_INFO_PID_TORQUE_CONSTANT              24u
#define MOTOR_INFO_PID_ROTOR_INERTIA                25u
#define MOTOR_INFO_PID_FRICTION_COULOMB             26u
#define MOTOR_INFO_PID_FRICTION_VISCOUS             27u
#define MOTOR_INFO_PID_GEAR_RATIO                   28u
#define MOTOR_INFO_PID_GEAR_EFFICIENCY              29u
#define MOTOR_INFO_PID_CALIBRATION_CURRENT          30u
#define MOTOR_INFO_PID_RESISTANCE_CALIB_MAX_VOLTAGE 31u
#define MOTOR_INFO_PID_CURRENT_LIM                  32u
#define MOTOR_INFO_PID_CURRENT_CONTROL_BANDWIDTH    33u
#define MOTOR_INFO_PID_ENC_TYPE                     34u
#define MOTOR_INFO_PID_ENC_LINES                    35u
#define MOTOR_INFO_PID_ENC_DIRECTION                36u
#define MOTOR_INFO_PID_ENC_OFFSET                   37u
#define MOTOR_INFO_PID_ELEC_ANGLE_BIAS              38u
#define MOTOR_INFO_PID_PWM_FREQ_HZ                  39u
#define MOTOR_INFO_PID_DEAD_TIME_NS                 40u
#define MOTOR_INFO_PID_SHUNT_RESISTANCE             41u
#define MOTOR_INFO_PID_CURRENT_AMP_GAIN             42u
#define MOTOR_INFO_PID_PEAK_CURRENT                 43u
#define MOTOR_INFO_PID_MAX_SPEED                    44u
#define MOTOR_INFO_PID_DEVICE_ZERO                  48u
#define MOTOR_INFO_PID_DEVICE_TIME                  49u
#define MOTOR_INFO_PID_CAN_ID                       50u
#define MOTOR_INFO_PID_CAN_BAUDRATE                 51u
#define MOTOR_INFO_PID_CAN_TIMEOUT_S                52u
#define MOTOR_INFO_PID_CAN_FD_ENABLE                53u
#define MOTOR_INFO_PID_CAN_FD_BAUDRATE              54u
#define MOTOR_INFO_PID_UART_BAUDRATE                55u
#define MOTOR_INFO_PID_KP_LD                        64u
#define MOTOR_INFO_PID_KI_LD                        65u
#define MOTOR_INFO_PID_KP_LQ                        66u
#define MOTOR_INFO_PID_KI_LQ                        67u
#define MOTOR_INFO_PID_INTEGRAL_LIMIT               68u
#define MOTOR_INFO_PID_DECOUPLING_GAIN              69u
#define MOTOR_INFO_PID_COMP_DU_V                    70u
#define MOTOR_INFO_PID_PWM_DUTY_MAX                 71u
#define MOTOR_INFO_PID_KP_S                         72u
#define MOTOR_INFO_PID_KI_S                         73u
#define MOTOR_INFO_PID_SPEED_INTEGRAL_LIMIT         74u
#define MOTOR_INFO_PID_VFF                          75u
#define MOTOR_INFO_PID_AFF                          76u
#define MOTOR_INFO_PID_JERK_FF                      77u
#define MOTOR_INFO_PID_SPEED_FILTER_ALPHA           78u
#define MOTOR_INFO_PID_SPEED_FILTER_ENABLE          79u
#define MOTOR_INFO_PID_KP_P                         80u
#define MOTOR_INFO_PID_KI_P                         81u
#define MOTOR_INFO_PID_POSITION_INTEGRAL_LIMIT      82u
#define MOTOR_INFO_PID_POSITION_FILTER_ALPHA        83u
#define MOTOR_INFO_PID_POSITION_FILTER_ENABLE       84u
#define MOTOR_INFO_PID_FOLLOWING_ERROR_LIMIT        85u
#define MOTOR_INFO_PID_DECOUPLE_ALGO                86u
#define MOTOR_INFO_PID_BEMF_FF_ENABLE               87u
#define MOTOR_INFO_PID_DEADTIME_COMP_ENABLE         88u
#define MOTOR_INFO_PID_PID_SOURCE_MASK              127u
#define MOTOR_INFO_PID_PROTECT_ENABLE               128u
#define MOTOR_INFO_PID_OVER_CURRENT_A               129u
#define MOTOR_INFO_PID_OVER_VOLTAGE_V               130u
#define MOTOR_INFO_PID_UNDER_VOLTAGE_V              131u
#define MOTOR_INFO_PID_OVER_SPEED_RAD_S             132u
#define MOTOR_INFO_PID_POSITION_FOLLOWING_ERROR_P   133u
#define MOTOR_INFO_PID_POS_LIMIT_MIN                134u
#define MOTOR_INFO_PID_POS_LIMIT_MAX                135u
#define MOTOR_INFO_PID_OV_MID_V                     136u
#define MOTOR_INFO_PID_UV_MID_V                     137u
#define MOTOR_INFO_PID_IBUS_OVER_A                  138u
#define MOTOR_INFO_PID_IBUS_CONT_A                  139u
#define MOTOR_INFO_PID_IBUS_CONT_TIME_S             140u
#define MOTOR_INFO_PID_TEMP_FET_OVER_D              141u
#define MOTOR_INFO_PID_TEMP_MOTOR_OVER_D            142u
#define MOTOR_INFO_PID_TEMP_MOTOR_HOT_D             143u
#define MOTOR_INFO_PID_TEMP_UNDER_D                 144u
#define MOTOR_INFO_PID_MASK_CRITICAL1               154u
#define MOTOR_INFO_PID_MASK_CRITICAL2               155u
#define MOTOR_INFO_PID_MASK_EXCEPTION1              156u
#define MOTOR_INFO_PID_MASK_EXCEPTION2              157u
#define MOTOR_INFO_PID_MASK_WARNING1                158u
#define MOTOR_INFO_PID_MASK_WARNING2                159u
#define MOTOR_INFO_PID_MIT_KP                       160u
#define MOTOR_INFO_PID_MIT_KD                       161u
#define MOTOR_INFO_PID_MIT_MAX_CURRENT              162u
#define MOTOR_INFO_PID_MIT_FEEDFORWARD_TORQUE       163u
#define MOTOR_INFO_PID_FORCE_KP                     164u
#define MOTOR_INFO_PID_FORCE_KI                     165u
#define MOTOR_INFO_PID_FORCE_LIMIT                  166u
#define MOTOR_INFO_PID_FORCE_CONTROL_ENABLE         167u
#define MOTOR_INFO_PID_HOMING_METHOD                168u
#define MOTOR_INFO_PID_HOMING_SPEED                 169u
#define MOTOR_INFO_PID_HOMING_OFFSET                170u
#define MOTOR_INFO_PID_SOFTSTART_VALID              176u
#define MOTOR_INFO_PID_SOFTSTART_ENABLE             177u
#define MOTOR_INFO_PID_SOFTSTART_SHAPE              178u
#define MOTOR_INFO_PID_SOFTSTART_DURATION           179u
#define MOTOR_INFO_PID_SOFTSTART_POS_RATE           180u
#define MOTOR_INFO_PID_SOFTSTART_VEL_RATE           181u
#define MOTOR_INFO_PID_SOFTSTART_TORQUE_RATE        182u
#define MOTOR_INFO_PID_SOFTSTART_CURRENT_RATE       183u
#define MOTOR_INFO_PID_COGGING_COMP_ENABLE          184u
#define MOTOR_INFO_PID_COGGING_COMP_GAIN            185u
#define MOTOR_INFO_MAX_PID                          185u

	/******************************************************************************
 * @brief   基础 API
 ******************************************************************************/
	/**
 * @brief   初始化参数区（memset 全零 + 版本号）
 * @note    默认不填充默认值（MOTOR_INFO_EN_INIT_DEFAULTS=0，仅全零+版本号，由 profile/上位机配置参数）；
 *          置 1 后恢复填充块索引表 + 各参数默认值
 * @param   cfg 参数区指针
 * @return  0=成功, -EINVAL=空指针
 */
	int motor_info_init(motor_info_t *cfg);

	/**
 * @brief   校验所有参数范围
 * @note    默认裁剪（MOTOR_INFO_EN_VALIDATE=0，恒返回 0=通过）；置 1 后执行完整范围校验
 * @param   cfg 参数区指针
 * @return  0=全部通过, >0=首个越界参数的 Index(见CSV), -EINVAL=空指针
 */
	int motor_info_validate(const motor_info_t *cfg);

	/**
 * @brief   打印所有参数
 * @note    默认裁剪（MOTOR_INFO_EN_PRINT=0，空实现）；置 1 后输出全部参数
 * @param   cfg 参数区指针
 */
	void motor_info_print(const motor_info_t *cfg);

/******************************************************************************
 * @brief   协议分发表 (param_id -> get/set), 供 0xE6-0xEB 单参读写
 ******************************************************************************/
/* dispatch 返回码 */
#define MOTOR_INFO_DISPATCH_OK       0
#define MOTOR_INFO_DISPATCH_E_BAD_ID -1
#define MOTOR_INFO_DISPATCH_E_BOUNDS -2
#define MOTOR_INFO_DISPATCH_E_RO     -3

/* Protocol value type codes: u8=0 i8=1 u16=2 i16=3 u32=4 i32=5 f32=6 str=7 u64=8 (v1.12) */
#define MOTOR_INFO_TYPE_U8  0u
#define MOTOR_INFO_TYPE_I8  1u
#define MOTOR_INFO_TYPE_U16 2u
#define MOTOR_INFO_TYPE_I16 3u
#define MOTOR_INFO_TYPE_U32 4u
#define MOTOR_INFO_TYPE_I32 5u
#define MOTOR_INFO_TYPE_F32 6u
#define MOTOR_INFO_TYPE_STR 7u
#define MOTOR_INFO_TYPE_U64 8u

	/**
 * @brief 按 param_id 读单个参数, 值写入 out8(固定8B, 高字节补零)
 * @param  pid      参数ID(CSV Index)
 * @param  cfg      motor_info_t 指针
 * @param  out8     输出缓冲(8字节)
 * @param  out_type 输出协议类型码(0=u8..8=u64)
 * @param  out_len  输出实际值字节数(1/2/4/8)
 * @return 0=成功, -1=未知pid/空指针
 */
	int motor_info_dispatch_read(uint16_t pid, const motor_info_t *cfg, uint8_t out8[8], uint8_t *out_type, uint8_t *out_len);

	/**
 * @brief 按 param_id 写单个参数, 值取自 in8(固定8B)
 * @param  pid  参数ID(CSV Index)
 * @param  cfg  motor_info_t 指针
 * @param  in8  输入缓冲(8字节, 参数按 desc->size 取低字节)
 * @param  len  实际有效字节数(<参数 size 视为越界)
 * @return 0=成功, -1=未知pid/空指针, -2=越界, -3=只读
 */
	int motor_info_dispatch_write(uint16_t pid, motor_info_t *cfg, const uint8_t in8[8], uint8_t len);

	/* Generic typed accessors for internal code paths. No field-level get/set API is generated. */
	int motor_info_param_size(uint16_t pid, uint8_t *size);

	int motor_info_read_u32(const motor_info_t *cfg, uint16_t pid, uint32_t *value);
	int motor_info_write_u32(motor_info_t *cfg, uint16_t pid, uint32_t value);
	int motor_info_read_u64(const motor_info_t *cfg, uint16_t pid, uint64_t *value);
	int motor_info_write_u64(motor_info_t *cfg, uint16_t pid, uint64_t value);
	int motor_info_read_i32(const motor_info_t *cfg, uint16_t pid, int32_t *value);
	int motor_info_write_i32(motor_info_t *cfg, uint16_t pid, int32_t value);
	int motor_info_read_f32(const motor_info_t *cfg, uint16_t pid, float *value);
	int motor_info_write_f32(motor_info_t *cfg, uint16_t pid, float value);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_INFO_H__ */
