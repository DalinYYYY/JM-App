/**
 * @file    motor_info.h
 * @brief   MotorInfo 配置参数 API 接口（1024B 整块空间）
 * @date    2026-08-14
 *
 * @warning 【自动生成文件，请勿手动修改】
 *          本文件由脚本 motor_info_generate.py 根据 motor_info.csv 自动生成，
 *          任何手动改动都会在下次运行脚本时被覆盖。
 *          如需修改参数定义，请编辑源 CSV 配置表后重新生成。
 */

#ifndef __MOTOR_INFO_H__
#define __MOTOR_INFO_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* 字节对齐宏：保证 RAM 布局与 Flash 字节布局完全一致，可整块 memcpy */
#ifndef __ALIGNED_4
#define __ALIGNED_4 __attribute__((aligned(4)))
#endif
#ifndef __ALIGNED_8
#define __ALIGNED_8 __attribute__((aligned(8)))
#endif

/* ===== 自动生成元信息 ===== */
#define MOTOR_INFO_GEN_DATE     "2026-08-14"
#define MOTOR_INFO_PARAM_COUNT   91
#define MOTOR_INFO_VERSION_MAJOR 1
#define MOTOR_INFO_VERSION_MINOR 0

/* ===== Flash 区域布局常量 ===== */
#define PARAM_AREA_SIZE    1024
#define MAX_BLOCK_COUNT    6

/* 各子块偏移/大小常量（供 Flash 读写寻址） */
#define MOTOR_INFO_BLOCK_SYSTEMPARAM_OFFSET 0x0040u
#define MOTOR_INFO_BLOCK_SYSTEMPARAM_SIZE   64u
#define MOTOR_INFO_BLOCK_MOTORCALIBPARAM_OFFSET 0x0080u
#define MOTOR_INFO_BLOCK_MOTORCALIBPARAM_SIZE   128u
#define MOTOR_INFO_BLOCK_DEVICEPARAM_OFFSET 0x0100u
#define MOTOR_INFO_BLOCK_DEVICEPARAM_SIZE   64u
#define MOTOR_INFO_BLOCK_CONTROLPARAM_OFFSET 0x0140u
#define MOTOR_INFO_BLOCK_CONTROLPARAM_SIZE   320u
#define MOTOR_INFO_BLOCK_PROTECTCOMMPARAM_OFFSET 0x0280u
#define MOTOR_INFO_BLOCK_PROTECTCOMMPARAM_SIZE   128u
#define MOTOR_INFO_BLOCK_ADVANCEDALGOPARAM_OFFSET 0x0300u
#define MOTOR_INFO_BLOCK_ADVANCEDALGOPARAM_SIZE   64u

/* ===== 全局头部与子块索引表 ===== */
typedef struct __ALIGNED_4
{
    uint32_t offset;  /* 子块偏移(字节) */
    uint32_t size;    /* 子块大小(字节) */
} BlockIndex_t;

typedef struct __ALIGNED_4
{
    uint16_t      version_major;             /* 主版本 */
    uint16_t      version_minor;             /* 次版本 */
    uint32_t      crc32;                     /* 参数区 CRC32(校验范围跳过本字段) */
    BlockIndex_t  blocks[MAX_BLOCK_COUNT];   /* 6*8 = 48B */
    uint32_t      reserved[2];               /* 8B 填充凑足 64B */
} ParamHeader_t;  /* 64B */

/* ===== 各子块数据结构（字段顺序与 CSV 一致，4 字节对齐）===== */
/**
 * @brief   系统级参数
 * @details 块大小 64B，已用 24B，预留 40B
 */
typedef struct __ALIGNED_4
{
    uint32_t config_version                  ; /* 配置版本号  [主版本.次版本(高16位.低16位) [SystemParam段0-15 本表占用0-4]] */
    uint32_t enable_uart                     ; /* 接口使能  [bit0:UART bit1:CAN bit2:CANFD bit3:USB] */
    uint32_t enable_bus_sensor               ; /* 总线传感器使能  [0:禁用 1:启用] */
    uint32_t safety_limit                    ; /* 安全限制使能  [0:禁用 1:启用 调试期建议关闭] */
    uint32_t total_runtime_s                 ; /* 累计运行时间 (s)  [掉电保存 运维数据 定期写入避免频繁擦写] */
    uint32_t save_count                      ; /* 固化次数  [固化累计次数 每次0xEA保存成功自增 超过寿命阈值拒绝写入并报错] */
    uint32_t reserved[10];                    /* 预留 40B */
} SystemParam_t;

/**
 * @brief   电机标定参数（含减速器/编码器/功率级/电流采样）
 * @details 块大小 128B，已用 116B，预留 12B
 */
typedef struct __ALIGNED_4
{
    uint32_t is_calibrated                   ; /* 电机是否校准  [0:未校准 1:已校准 [MotorCalibParam段16-47 本表占用16-42]] */
    uint32_t pole_pairs                      ; /* 电机极对数 (pairs)  [电机极对数 影响电角度=机械角×极对数] */
    uint32_t motor_type                      ; /* 电机类型  [0:SPMSM 1:IPMSM 2:BLDC] */
    uint32_t direction                       ; /* 电机方向  [0:正向 1:反向] */
    float    phase_resistance                ; /* 相电阻 (ohm)  [标定值 电流环Ki=ωc·R依赖此值] */
    float    phase_inductance_d              ; /* d轴相电感 (H)  [标定值 电流环Kp=ωc·Ld依赖此值] */
    float    phase_inductance_q              ; /* q轴相电感 (H)  [标定值 IPMSM的Lq一般略大于Ld] */
    float    flux_linkage                    ; /* 永磁体磁链 (Wb)  [标定值 反电势法辨识] */
    float    torque_constant                 ; /* 转矩常数 (Nm/A)  [标定值 速度环Kp=J·ωc/Kt依赖此值] */
    float    rotor_inertia                   ; /* 转子惯量 (kg·m²)  [标定值 速度环Kp/Ki依赖此值] */
    float    friction_coulomb                ; /* 库仑摩擦力矩 (Nm)  [标定值 L5摩擦辨识] */
    float    friction_viscous                ; /* 粘滞摩擦系数 (Nm/(rad/s))  [标定值 L5摩擦辨识] */
    float    gear_ratio                      ; /* 减速比  [电机转速/输出转速] */
    float    gear_efficiency                 ; /* 减速器效率  [传动效率 0~1] */
    float    calibration_current             ; /* 校准电流 (A)  [R/L标定施加电流 注意发热] */
    float    resistance_calib_max_voltage    ; /* 电阻校准最大电压 (V)  [R标定电压限幅 防过流] */
    float    current_lim                     ; /* 峰值电流限制 (A)  [硬件保护 瞬时最大电流] */
    float    current_control_bandwidth       ; /* 电流环带宽 (Hz)  [autotune用此值算电流环PID] */
    uint32_t enc_type                        ; /* 编码器类型  [1:MT6701 2:MT6835 0:ABZ增量 3:霍尔 同板可换] */
    uint32_t enc_lines                       ; /* 编码器分辨率 (CPR)  [SPI绝对值为分辨率 增量式为CPR] */
    int32_t  enc_direction                   ; /* 编码器计数方向  [1:正向 -1:反向] */
    float    enc_offset                      ; /* 编码器初始位置偏移 (deg)  [校准后保存 机械零点偏移] */
    float    elec_angle_bias                 ; /* 电角度偏移 (rad)  [FOC换相必需 校准后保存 最关键] */
    uint32_t pwm_freq_hz                     ; /* PWM载波频率 (Hz)  [不同功率器件可能不同] */
    float    dead_time_ns                    ; /* PWM死区时间 (ns)  [不同功率器件可能不同] */
    float    shunt_resistance                ; /* 电流采样电阻 (ohm)  [硬件相关 不同板子不同] */
    float    current_amp_gain                ; /* 电流放大增益  [硬件相关 运放增益] */
    float    peak_current                    ; /* 峰值电流 (A)  [电机峰值电流(运行期限幅) 切换电机型号时需同步修改] */
    float    max_speed                       ; /* 最大转速 (rad/s)  [电机最大转速(运行期限幅) 切换电机型号时需同步修改] */
    uint32_t reserved[3];                    /* 预留 12B */
} MotorCalibParam_t;

/**
 * @brief   设备参数（CAN/UART）
 * @details 块大小 64B，已用 32B，预留 32B
 */
typedef struct __ALIGNED_4
{
    float    device_zero                     ; /* 设备零度 (rad)  [机械零点位置 [DeviceParam段48-63 本表占用48-55]] */
    uint32_t device_time                     ; /* 设备生产日期  [YYYYMMDD格式] */
    uint32_t can_id                          ; /* CAN节点ID  [设备节点地址 0保留为广播] */
    uint32_t can_baudrate                    ; /* CAN波特率 (bps) */
    float    can_timeout_s                   ; /* CAN通信超时 (s)  [0=禁用超时] */
    uint32_t can_fd_enable                   ; /* CAN FD使能  [0:传统CAN 1:CAN FD] */
    uint32_t can_fd_baudrate                 ; /* CAN FD数据波特率 (bps) */
    uint32_t uart_baudrate                   ; /* UART波特率 (bps) */
    uint32_t reserved[8];                    /* 预留 32B */
} DeviceParam_t;

/**
 * @brief   控制参数（三环PID+前馈+滤波）
 * @details 块大小 320B，已用 104B，预留 216B
 */
typedef struct __ALIGNED_4
{
    float    kp_ld                           ; /* d轴比例增益 (V/A)  [电流环d轴P增益 autotune默认Kp=ωc·Ld [ControlParam段64-127 本表占用64-85]] */
    float    ki_ld                           ; /* d轴积分增益 (V/(A·s))  [电流环d轴I增益 autotune默认Ki=ωc·R] */
    float    kp_lq                           ; /* q轴比例增益 (V/A)  [电流环q轴P增益 autotune默认Kp=ωc·Lq] */
    float    ki_lq                           ; /* q轴积分增益 (V/(A·s))  [电流环q轴I增益 autotune默认Ki=ωc·R] */
    float    integral_limit                  ; /* 积分限幅 (V)  [电流环积分输出限幅 建议Kp_q×额定电流×1.5 与motor_param.c默认值对齐] */
    float    decoupling_gain                 ; /* dq轴解耦增益  [0~1 调试期可适当增大观察效果] */
    float    comp_du_V                       ; /* 死区补偿电压 (V)  [死区非线性补偿电压] */
    float    pwm_duty_max                    ; /* PWM最大占空比  [0~1 防过调制] */
    float    kp_s                            ; /* 速度环比例增益 (A/(rad/s))  [速度环P增益 autotune默认Kp=J·ωc/Kt] */
    float    ki_s                            ; /* 速度环积分增益 (A/rad)  [速度环I增益 autotune默认Ki=J·ωc²/(4·Kt)] */
    float    speed_integral_limit            ; /* 速度环积分限幅 (A)  [建议额定电流×0.5] */
    float    vff                             ; /* 速度前馈系数  [0~1 调试期可适当增大] */
    float    aff                             ; /* 加速度前馈系数  [0~1 加速度前馈] */
    float    jerk_ff                         ; /* 加加速度前馈系数  [0~1 加加速度前馈] */
    float    speed_filter_alpha              ; /* 速度滤波系数  [一阶低通滤波系数 越大滤波越弱] */
    uint32_t speed_filter_enable             ; /* 速度滤波使能  [0:禁用 1:启用] */
    float    kp_p                            ; /* 位置环比例增益 (Hz)  [位置环P增益 autotune默认Kp=2π·f] */
    float    ki_p                            ; /* 位置环积分增益 (1/s)  [位置环I增益 一般为0] */
    float    position_integral_limit         ; /* 位置环积分限幅 (rad)  [位置环积分输出限幅] */
    float    position_filter_alpha           ; /* 位置滤波系数  [一阶低通滤波系数] */
    uint32_t position_filter_enable          ; /* 位置滤波使能  [0:禁用 1:启用] */
    float    following_error_limit           ; /* 跟随误差限制 (P)  [位置跟随误差保护阈值 调试期建议放大] */
    uint32_t decouple_algo                   ; /* 交叉解耦算法  [0:NONE 1:FEEDFORWARD 2:FEEDBACK] */
    uint32_t bemf_ff_enable                  ; /* 反电势前馈使能  [0:禁用 1:启用] */
    uint32_t deadtime_comp_enable            ; /* 死区补偿使能  [0:禁用 1:启用] */
    uint32_t pid_source_mask                 ; /* PID来源位掩码  [bit[3:0]=电流环 bit[7:4]=速度环 bit[11:8]=位置环 0=默认 1=Flash 2=理论估计] */
    uint32_t reserved[54];                    /* 预留 216B */
} ControlParam_t;

/**
 * @brief   保护与通信参数
 * @details 块大小 128B，已用 44B，预留 84B
 */
typedef struct __ALIGNED_4
{
    float    over_current_A                  ; /* 过流保护阈值 (A)  [ [ProtectCommParam段128-159 本表占用128-138]] */
    float    over_voltage_V                  ; /* 过压保护阈值 (V) */
    float    under_voltage_V                 ; /* 欠压保护阈值 (V) */
    float    over_temp_drive                 ; /* 驱动器过温阈值 (℃) */
    float    over_temp_motor                 ; /* 电机过温阈值 (℃) */
    float    under_temp_d                    ; /* 欠温保护阈值 (℃) */
    float    over_speed_rad_s                ; /* 过速保护阈值 (rad/s) */
    int32_t  position_following_error_p      ; /* 位置跟随误差保护 (P) */
    int32_t  pos_limit_min                   ; /* 位置下限 (P)  [硬件位置下限] */
    int32_t  pos_limit_max                   ; /* 位置上限 (P)  [硬件位置上限] */
    uint32_t error_enable_mask               ; /* 保护使能掩码  [bit0:过流 bit1:过压 bit2:欠压 bit3:过温 bit4:过速 bit5:栅极驱动故障(nFAULT)] */
    uint32_t reserved[21];                    /* 预留 84B */
} ProtectCommParam_t;

/**
 * @brief   高级算法参数（MIT/力控/回零）
 * @details 块大小 64B，已用 44B，预留 20B
 */
typedef struct __ALIGNED_4
{
    float    mit_kp                          ; /* MIT位置刚度 (Nm/rad)  [ [AdvancedAlgoParam段160-191 本表占用160-170]] */
    float    mit_kd                          ; /* MIT速度阻尼 (Nm/(rad/s)) */
    float    mit_max_current                 ; /* MIT最大电流 (A) */
    float    mit_feedforward_torque          ; /* MIT前馈力矩 (Nm) */
    float    force_kp                        ; /* 力控比例增益 (A/Nm) */
    float    force_ki                        ; /* 力控积分增益 (A/(Nm·s)) */
    float    force_limit                     ; /* 力控力矩限制 (Nm) */
    uint32_t force_control_enable            ; /* 力控使能  [0:禁用 1:启用] */
    uint32_t homing_method                   ; /* 回零方法  [0:当前位置回零 1:限位回零] */
    float    homing_speed                    ; /* 回零速度 (rad/s) */
    float    homing_offset                   ; /* 回零偏移 (rad) */
    uint32_t reserved[5];                    /* 预留 20B */
} AdvancedAlgoParam_t;

/* ===== 主参数区联合体：1024B 整块空间 ===== */
/* 8 字节对齐：被 (u64*) 强转传给 Flash 读写时，保证 u64 访问不产生未对齐故障 */
typedef union __ALIGNED_8
{
    uint8_t raw[PARAM_AREA_SIZE];  /* 原始字节数组，可直接 memcpy 到 Flash */
    struct __ALIGNED_4
    {
        ParamHeader_t header;            /* 0x0000  64B */
        SystemParam_t        system        ; /* 0x0040  64B  系统级参数 */
        MotorCalibParam_t    motor_calib   ; /* 0x0080  128B  电机标定参数（含减速器/编码器/功率级/电流采样） */
        DeviceParam_t        device        ; /* 0x0100  64B  设备参数（CAN/UART） */
        ControlParam_t       control       ; /* 0x0140  320B  控制参数（三环PID+前馈+滤波） */
        ProtectCommParam_t   protect_comm  ; /* 0x0280  128B  保护与通信参数 */
        AdvancedAlgoParam_t  advanced      ; /* 0x0300  64B  高级算法参数（MIT/力控/回零） */
        uint8_t reserved[192];            /* 0x0340  192B  末尾预留 */
    } blocks;
} motor_info_t;  /* 1024B */

/* ===== Parameter IDs (CSV Index) ===== */
#define MOTOR_INFO_PID_CONFIG_VERSION                    0u
#define MOTOR_INFO_PID_ENABLE_UART                       1u
#define MOTOR_INFO_PID_ENABLE_BUS_SENSOR                 2u
#define MOTOR_INFO_PID_SAFETY_LIMIT                      3u
#define MOTOR_INFO_PID_TOTAL_RUNTIME_S                   4u
#define MOTOR_INFO_PID_SAVE_COUNT                        5u
#define MOTOR_INFO_PID_IS_CALIBRATED                     16u
#define MOTOR_INFO_PID_POLE_PAIRS                        17u
#define MOTOR_INFO_PID_MOTOR_TYPE                        18u
#define MOTOR_INFO_PID_DIRECTION                         19u
#define MOTOR_INFO_PID_PHASE_RESISTANCE                  20u
#define MOTOR_INFO_PID_PHASE_INDUCTANCE_D                21u
#define MOTOR_INFO_PID_PHASE_INDUCTANCE_Q                22u
#define MOTOR_INFO_PID_FLUX_LINKAGE                      23u
#define MOTOR_INFO_PID_TORQUE_CONSTANT                   24u
#define MOTOR_INFO_PID_ROTOR_INERTIA                     25u
#define MOTOR_INFO_PID_FRICTION_COULOMB                  26u
#define MOTOR_INFO_PID_FRICTION_VISCOUS                  27u
#define MOTOR_INFO_PID_GEAR_RATIO                        28u
#define MOTOR_INFO_PID_GEAR_EFFICIENCY                   29u
#define MOTOR_INFO_PID_CALIBRATION_CURRENT               30u
#define MOTOR_INFO_PID_RESISTANCE_CALIB_MAX_VOLTAGE      31u
#define MOTOR_INFO_PID_CURRENT_LIM                       32u
#define MOTOR_INFO_PID_CURRENT_CONTROL_BANDWIDTH         33u
#define MOTOR_INFO_PID_ENC_TYPE                          34u
#define MOTOR_INFO_PID_ENC_LINES                         35u
#define MOTOR_INFO_PID_ENC_DIRECTION                     36u
#define MOTOR_INFO_PID_ENC_OFFSET                        37u
#define MOTOR_INFO_PID_ELEC_ANGLE_BIAS                   38u
#define MOTOR_INFO_PID_PWM_FREQ_HZ                       39u
#define MOTOR_INFO_PID_DEAD_TIME_NS                      40u
#define MOTOR_INFO_PID_SHUNT_RESISTANCE                  41u
#define MOTOR_INFO_PID_CURRENT_AMP_GAIN                  42u
#define MOTOR_INFO_PID_PEAK_CURRENT                      43u
#define MOTOR_INFO_PID_MAX_SPEED                         44u
#define MOTOR_INFO_PID_DEVICE_ZERO                       48u
#define MOTOR_INFO_PID_DEVICE_TIME                       49u
#define MOTOR_INFO_PID_CAN_ID                            50u
#define MOTOR_INFO_PID_CAN_BAUDRATE                      51u
#define MOTOR_INFO_PID_CAN_TIMEOUT_S                     52u
#define MOTOR_INFO_PID_CAN_FD_ENABLE                     53u
#define MOTOR_INFO_PID_CAN_FD_BAUDRATE                   54u
#define MOTOR_INFO_PID_UART_BAUDRATE                     55u
#define MOTOR_INFO_PID_KP_LD                             64u
#define MOTOR_INFO_PID_KI_LD                             65u
#define MOTOR_INFO_PID_KP_LQ                             66u
#define MOTOR_INFO_PID_KI_LQ                             67u
#define MOTOR_INFO_PID_INTEGRAL_LIMIT                    68u
#define MOTOR_INFO_PID_DECOUPLING_GAIN                   69u
#define MOTOR_INFO_PID_COMP_DU_V                         70u
#define MOTOR_INFO_PID_PWM_DUTY_MAX                      71u
#define MOTOR_INFO_PID_KP_S                              72u
#define MOTOR_INFO_PID_KI_S                              73u
#define MOTOR_INFO_PID_SPEED_INTEGRAL_LIMIT              74u
#define MOTOR_INFO_PID_VFF                               75u
#define MOTOR_INFO_PID_AFF                               76u
#define MOTOR_INFO_PID_JERK_FF                           77u
#define MOTOR_INFO_PID_SPEED_FILTER_ALPHA                78u
#define MOTOR_INFO_PID_SPEED_FILTER_ENABLE               79u
#define MOTOR_INFO_PID_KP_P                              80u
#define MOTOR_INFO_PID_KI_P                              81u
#define MOTOR_INFO_PID_POSITION_INTEGRAL_LIMIT           82u
#define MOTOR_INFO_PID_POSITION_FILTER_ALPHA             83u
#define MOTOR_INFO_PID_POSITION_FILTER_ENABLE            84u
#define MOTOR_INFO_PID_FOLLOWING_ERROR_LIMIT             85u
#define MOTOR_INFO_PID_DECOUPLE_ALGO                     86u
#define MOTOR_INFO_PID_BEMF_FF_ENABLE                    87u
#define MOTOR_INFO_PID_DEADTIME_COMP_ENABLE              88u
#define MOTOR_INFO_PID_PID_SOURCE_MASK                   127u
#define MOTOR_INFO_PID_OVER_CURRENT_A                    128u
#define MOTOR_INFO_PID_OVER_VOLTAGE_V                    129u
#define MOTOR_INFO_PID_UNDER_VOLTAGE_V                   130u
#define MOTOR_INFO_PID_OVER_TEMP_DRIVE                   131u
#define MOTOR_INFO_PID_OVER_TEMP_MOTOR                   132u
#define MOTOR_INFO_PID_UNDER_TEMP_D                      133u
#define MOTOR_INFO_PID_OVER_SPEED_RAD_S                  134u
#define MOTOR_INFO_PID_POSITION_FOLLOWING_ERROR_P        135u
#define MOTOR_INFO_PID_POS_LIMIT_MIN                     136u
#define MOTOR_INFO_PID_POS_LIMIT_MAX                     137u
#define MOTOR_INFO_PID_ERROR_ENABLE_MASK                 138u
#define MOTOR_INFO_PID_MIT_KP                            160u
#define MOTOR_INFO_PID_MIT_KD                            161u
#define MOTOR_INFO_PID_MIT_MAX_CURRENT                   162u
#define MOTOR_INFO_PID_MIT_FEEDFORWARD_TORQUE            163u
#define MOTOR_INFO_PID_FORCE_KP                          164u
#define MOTOR_INFO_PID_FORCE_KI                          165u
#define MOTOR_INFO_PID_FORCE_LIMIT                       166u
#define MOTOR_INFO_PID_FORCE_CONTROL_ENABLE              167u
#define MOTOR_INFO_PID_HOMING_METHOD                     168u
#define MOTOR_INFO_PID_HOMING_SPEED                      169u
#define MOTOR_INFO_PID_HOMING_OFFSET                     170u
#define MOTOR_INFO_MAX_PID                                   170u

/******************************************************************************
 * @brief   基础 API
 ******************************************************************************/
/**
 * @brief   初始化为默认值（含版本/块索引表 + 各参数默认值）
 * @param   cfg 参数区指针
 * @return  0=成功, -EINVAL=空指针
 */
int  motor_info_init(motor_info_t *cfg);

/**
 * @brief   校验所有参数范围
 * @param   cfg 参数区指针
 * @return  0=全部通过, >0=首个越界参数的 Index(见CSV), -EINVAL=空指针
 */
int  motor_info_validate(const motor_info_t *cfg);

/**
 * @brief   打印所有参数
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

/* Protocol value type codes: u8=0 i8=1 u16=2 i16=3 u32=4 i32=5 f32=6 */
#define MOTOR_INFO_TYPE_U8   0u
#define MOTOR_INFO_TYPE_I8   1u
#define MOTOR_INFO_TYPE_U16  2u
#define MOTOR_INFO_TYPE_I16  3u
#define MOTOR_INFO_TYPE_U32  4u
#define MOTOR_INFO_TYPE_I32  5u
#define MOTOR_INFO_TYPE_F32  6u

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

/* Generic typed accessors for internal code paths. No field-level get/set API is generated. */
int motor_info_read_u32(const motor_info_t *cfg, uint16_t pid, uint32_t *value);
int motor_info_write_u32(motor_info_t *cfg, uint16_t pid, uint32_t value);
int motor_info_read_i32(const motor_info_t *cfg, uint16_t pid, int32_t *value);
int motor_info_write_i32(motor_info_t *cfg, uint16_t pid, int32_t value);
int motor_info_read_f32(const motor_info_t *cfg, uint16_t pid, float *value);
int motor_info_write_f32(motor_info_t *cfg, uint16_t pid, float value);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_INFO_H__ */
