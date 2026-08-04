/**
 * @file    motor_param.h
 * @brief   关节电机配置参数API接口
 * @date    2026-08-04
 *
 * @warning 【自动生成文件，请勿手动修改】
 *          本文件由脚本 motor_param_generate_v9.py 根据配置表自动生成，
 *          任何手动改动都会在下次运行脚本时被覆盖。
 *          如需修改参数定义，请编辑源 CSV 配置表后重新生成。
 */

#ifndef __MOTOR_PARAM_H__
#define __MOTOR_PARAM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ===== 自动生成元信息 ===== */
#define MOTOR_PARAM_GEN_DATE   "2026-08-04"
#define MOTOR_PARAM_PARAM_COUNT 86

/* 配置结构体前向声明 */
typedef struct motor_param motor_param_t;

/**
 * @brief   电机实例标识
 */
typedef struct
{
    uint8_t motor_id; /* 电机实例ID */
    char motor_name[16]; /* 电机名称字符串 */
} motor_instance_t;

/**
 * @brief   电机本体参数
 */
typedef struct
{
    float r; /* 定子相电阻 (ohm) */
    float ld; /* d轴定子电感 (H) */
    float lq; /* q轴定子电感 (H) */
    float flux; /* 永磁体磁链 (Wb) */
    float kt; /* 转矩常数 (Nm/A) */
    uint8_t pole_pairs; /* 极对数 */
    float rated_current; /* 电机额定电流 (A) */
    float peak_current; /* 电机峰值电流 (A) */
    float max_speed; /* 电机最大转速 (rad/s) */
    float dead_time_ns; /* PWM死区时间 (ns) */
    float rated_voltage; /* 电机额定电压 (V) */
    float rated_speed_rpm; /* 电机额定转速 (rpm) */
    float rated_torque; /* 电机额定转矩 (Nm) */
    float peak_torque; /* 电机峰值转矩 (Nm) */
    float inertia; /* 转子转动惯量 (kg*m2) */
    float ke; /* 反电动势常数 (V/(rad/s)) */
    uint32_t pwm_freq_hz; /* PWM载波频率 (Hz) */
    uint32_t foc_freq_hz; /* FOC控制频率 (Hz) */
} motor_base_t;

/**
 * @brief   减速器参数
 */
typedef struct
{
    float gear_ratio; /* 减速器减速比 */
    float gear_efficiency; /* 减速器效率 */
    float output_torque_const; /* 输出转矩常数 (Nm/A) */
    float gear_backlash; /* 减速器回程间隙 (rad) */
} gearbox_param_t;

/**
 * @brief   编码器参数
 */
typedef struct
{
    uint32_t enc_lines; /* 编码器分辨率 (CPR) */
    int8_t enc_direction; /* 编码器计数方向 */
    float enc_offset; /* 编码器初始位置偏移 (deg) */
    float elec_angle_bias; /* 电角度偏移 (rad) */
    float pos_filter_alpha; /* 位置滤波系数 */
    uint8_t enc_type; /* 编码器类型 */
    uint8_t enc_auto_calib; /* 编码器自动校准 */
    float speed_obs_gain; /* 速度观测器增益 */
} encoder_param_t;

/**
 * @brief   位置限位配置
 */
typedef struct
{
    uint8_t multiturn_enable; /* 多圈位置使能 */
    float pos_min_limit; /* 软件位置负限位 (rad) */
    float pos_max_limit; /* 软件位置正限位 (rad) */
    uint8_t limit_sw_enable; /* 硬件限位开关使能 */
} position_limit_t;

/**
 * @brief   回零配置参数
 */
typedef struct
{
    uint8_t homing_method; /* 回零方式选择 */
    float homing_speed_fast; /* 回零快速速度 (rad/s) */
    float homing_speed_slow; /* 回零慢速速度 (rad/s) */
    float homing_offset; /* 零点位置偏移 (rad) */
    float homing_current; /* 回零电流限制 (A) */
} homing_param_t;

/**
 * @brief   电流环控制参数
 */
typedef struct
{
    float current_kp_d; /* d轴电流比例增益 (V/A) */
    float current_ki_d; /* d轴电流积分增益 (V/(A*s)) */
    float current_kp_q; /* q轴电流比例增益 (V/A) */
    float current_ki_q; /* q轴电流积分增益 (V/(A*s)) */
    float current_integral_limit; /* 电流积分限幅 (V) */
    float decoupling_gain; /* DQ交叉解耦增益 */
    float deadtime_comp_v; /* 死区补偿电压 (V) */
    float pwm_max_duty; /* PWM最大占空比 */
    float current_bandwidth_hz; /* 电流环带宽 (Hz) */
    float current_filter_alpha; /* 电流采样滤波系数 */
    float d_feedforward_gain; /* d轴前馈增益 */
    float q_feedforward_gain; /* q轴前馈增益 */
    uint8_t decouple_algo; /* 交叉解耦算法 */
    uint8_t bemf_ff_enable; /* 反电势前馈使能 */
    uint8_t deadtime_comp_enable; /* 死区补偿使能 */
} current_loop_t;

/**
 * @brief   位置速度环控制参数
 */
typedef struct
{
    float speed_kp; /* 速度环比例增益 (A/(rad/s)) */
    float speed_ki; /* 速度环积分增益 (A/rad) */
    float speed_integral_limit; /* 速度环积分限幅 (A) */
    float velocity_ff_gain; /* 速度前馈增益 */
    float accel_ff_gain; /* 加速度前馈增益 */
    float position_kp; /* 位置环比例增益 (Hz) */
    float position_integral_limit; /* 位置环积分限幅 (rad) */
    float friction_coulomb; /* 库仑摩擦力矩 (Nm) */
    float friction_viscous; /* 粘滞摩擦系数 (Nm/(rad/s)) */
    float notch_freq_hz; /* 陷波滤波器频率 (Hz) */
    float notch_width_hz; /* 陷波滤波器带宽 (Hz) */
    float notch_depth_db; /* 陷波滤波器深度 (dB) */
    uint8_t notch_enable; /* 陷波滤波器使能 */
    float speed_bandwidth_hz; /* 速度环带宽 (Hz) */
    float speed_filter_alpha; /* 速度滤波系数 */
    float position_bandwidth_hz; /* 位置环带宽 (Hz) */
} position_loop_t;

/**
 * @brief   阻抗控制参数
 */
typedef struct
{
    float impedance_kp; /* 阻抗控制位置刚度 (Nm/rad) */
    float impedance_kd; /* 阻抗控制速度阻尼 (Nm/(rad/s)) */
    float iq_max; /* 最大输出电流 (A) */
} impedance_ctrl_t;

/**
 * @brief   热模型参数
 */
typedef struct
{
    float thermal_resistance; /* 电机热阻 (K/W) */
    float thermal_time_const; /* 热时间常数 (s) */
    float derating_temp_start; /* 降额起始温度 (C) */
} thermal_model_t;

/**
 * @brief   保护参数配置
 */
typedef struct
{
    float protect_over_current; /* 过流保护阈值 (A) */
    float protect_over_voltage; /* 过压保护阈值 (V) */
    float protect_under_voltage; /* 欠压保护阈值 (V) */
    float protect_over_speed; /* 过速保护阈值 (rad/s) */
    float protect_over_temp; /* 过温保护阈值 (C) */
    float protect_under_temp; /* 欠温保护阈值 (C) */
    int32_t protect_pos_error; /* 位置跟随误差限制 (counts) */
    uint32_t protect_enable_mask; /* 保护使能掩码 */
} protection_param_t;

/**
 * @brief   关节电机完整配置结构体
 */
struct motor_param
{
    motor_instance_t motor_instance; /* 电机实例标识 */
    motor_base_t motor_base; /* 电机本体参数 */
    gearbox_param_t gearbox_param; /* 减速器参数 */
    encoder_param_t encoder_param; /* 编码器参数 */
    position_limit_t position_limit; /* 位置限位配置 */
    homing_param_t homing_param; /* 回零配置参数 */
    current_loop_t current_loop; /* 电流环控制参数 */
    position_loop_t position_loop; /* 位置速度环控制参数 */
    impedance_ctrl_t impedance_ctrl; /* 阻抗控制参数 */
    thermal_model_t thermal_model; /* 热模型参数 */
    protection_param_t protection_param; /* 保护参数配置 */
};

/******************************************************************************
 * @brief   基础API接口
 ******************************************************************************/

/**
 * @brief   初始化电机配置为默认值
 * @param   cfg 电机配置指针
 * @return  0=成功, -EINVAL=参数错误
 */
int motor_param_init(motor_param_t *cfg);

/**
 * @brief   校验电机配置参数范围
 * @param   cfg 电机配置指针
 * @return  0=全部通过, >0=首个越界参数的 id(见CSV), -EINVAL=空指针
 */
int motor_param_validate(const motor_param_t *cfg);

/**
 * @brief   打印电机配置所有参数
 * @param   cfg 电机配置指针
 */
void motor_param_print(const motor_param_t *cfg);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_PARAM_H__ */
