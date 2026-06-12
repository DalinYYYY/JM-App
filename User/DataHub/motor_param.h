/**
 * @file    motor_param.h
 * @brief   关节电机配置参数API接口
 * @date    2026-06-11
 */

#ifndef __MOTOR_PARAM_H__
#define __MOTOR_PARAM_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

	/* 配置结构体前向声明 */
	typedef struct motor_param motor_param_t;

	/**
 * @brief   电机实例标识
 */
	typedef struct
	{
		uint8_t motor_id;	 /* 电机实例ID */
		char motor_name[16]; /* 电机名称字符串 */
	} motor_instance_t;

	/**
 * @brief   电机本体参数
 */
	typedef struct
	{
		float r;			   /* 定子相电阻 (ohm) */
		float ld;			   /* d轴定子电感 (H) */
		float lq;			   /* q轴定子电感 (H) */
		float flux;			   /* 永磁体磁链 (Wb) */
		float kt;			   /* 转矩常数 (Nm/A) */
		uint8_t pole_pairs;	   /* 极对数 */
		float rated_current;   /* 电机额定电流 (A) */
		float peak_current;	   /* 电机峰值电流 (A) */
		float max_speed;	   /* 电机最大转速 (rad/s) */
		float dead_time_ns;	   /* PWM死区时间 (ns) */
		float rated_voltage;   /* 电机额定电压 (V) */
		float rated_speed_rpm; /* 电机额定转速 (rpm) */
		float rated_torque;	   /* 电机额定转矩 (Nm) */
		float peak_torque;	   /* 电机峰值转矩 (Nm) */
		float inertia;		   /* 转子转动惯量 (kg*m2) */
		float ke;			   /* 反电动势常数 (V/(rad/s)) */
		uint32_t pwm_freq_hz;  /* PWM载波频率 (Hz) */
		uint32_t foc_freq_hz;  /* FOC控制频率 (Hz) */
	} motor_base_t;

	/**
 * @brief   减速器参数
 */
	typedef struct
	{
		float gear_ratio;		   /* 减速器减速比 */
		float gear_efficiency;	   /* 减速器效率 */
		float output_torque_const; /* 输出转矩常数 (Nm/A) */
		float gear_backlash;	   /* 减速器回程间隙 (rad) */
	} gearbox_param_t;

	/**
 * @brief   编码器参数
 */
	typedef struct
	{
		uint32_t enc_lines;		/* 编码器分辨率 (CPR) */
		int8_t enc_direction;	/* 编码器计数方向 */
		int32_t enc_offset;		/* 编码器初始位置偏移 (counts) */
		float elec_angle_bias;	/* 电角度偏移 (rad) */
		float pos_filter_alpha; /* 位置滤波系数 */
		uint8_t enc_type;		/* 编码器类型 */
		uint8_t enc_auto_calib; /* 编码器自动校准 */
		float speed_obs_gain;	/* 速度观测器增益 */
	} encoder_param_t;

	/**
 * @brief   位置限位配置
 */
	typedef struct
	{
		uint8_t multiturn_enable; /* 多圈位置使能 */
		float pos_min_limit;	  /* 软件位置负限位 (rad) */
		float pos_max_limit;	  /* 软件位置正限位 (rad) */
		uint8_t limit_sw_enable;  /* 硬件限位开关使能 */
	} position_limit_t;

	/**
 * @brief   回零配置参数
 */
	typedef struct
	{
		uint8_t homing_method;	 /* 回零方式选择 */
		float homing_speed_fast; /* 回零快速速度 (rad/s) */
		float homing_speed_slow; /* 回零慢速速度 (rad/s) */
		float homing_offset;	 /* 零点位置偏移 (rad) */
		float homing_current;	 /* 回零电流限制 (A) */
	} homing_param_t;

	/**
 * @brief   电流环控制参数
 */
	typedef struct
	{
		float current_kp_d;			  /* d轴电流比例增益 (V/A) */
		float current_ki_d;			  /* d轴电流积分增益 (V/(A*s)) */
		float current_kp_q;			  /* q轴电流比例增益 (V/A) */
		float current_ki_q;			  /* q轴电流积分增益 (V/(A*s)) */
		float current_integral_limit; /* 电流积分限幅 (V) */
		float decoupling_gain;		  /* DQ交叉解耦增益 */
		float deadtime_comp_v;		  /* 死区补偿电压 (V) */
		float pwm_max_duty;			  /* PWM最大占空比 */
		float current_bandwidth_hz;	  /* 电流环带宽 (Hz) */
		float current_filter_alpha;	  /* 电流采样滤波系数 */
		float d_feedforward_gain;	  /* d轴前馈增益 */
		float q_feedforward_gain;	  /* q轴前馈增益 */
	} current_loop_t;

	/**
 * @brief   位置速度环控制参数
 */
	typedef struct
	{
		float speed_kp;				   /* 速度环比例增益 (A/(rad/s)) */
		float speed_ki;				   /* 速度环积分增益 (A/rad) */
		float speed_integral_limit;	   /* 速度环积分限幅 (A) */
		float velocity_ff_gain;		   /* 速度前馈增益 */
		float accel_ff_gain;		   /* 加速度前馈增益 */
		float position_kp;			   /* 位置环比例增益 (Hz) */
		float position_integral_limit; /* 位置环积分限幅 (rad) */
		float friction_coulomb;		   /* 库仑摩擦力矩 (Nm) */
		float friction_viscous;		   /* 粘滞摩擦系数 (Nm/(rad/s)) */
		float notch_freq_hz;		   /* 陷波滤波器频率 (Hz) */
		float notch_width_hz;		   /* 陷波滤波器带宽 (Hz) */
		float notch_depth_db;		   /* 陷波滤波器深度 (dB) */
		uint8_t notch_enable;		   /* 陷波滤波器使能 */
		float speed_bandwidth_hz;	   /* 速度环带宽 (Hz) */
		float speed_filter_alpha;	   /* 速度滤波系数 */
		float position_bandwidth_hz;   /* 位置环带宽 (Hz) */
	} position_loop_t;

	/**
 * @brief   阻抗控制参数
 */
	typedef struct
	{
		float impedance_kp; /* 阻抗控制位置刚度 (Nm/rad) */
		float impedance_kd; /* 阻抗控制速度阻尼 (Nm/(rad/s)) */
		float iq_max;		/* 最大输出电流 (A) */
	} impedance_ctrl_t;

	/**
 * @brief   热模型参数
 */
	typedef struct
	{
		float thermal_resistance;  /* 电机热阻 (K/W) */
		float thermal_time_const;  /* 热时间常数 (s) */
		float derating_temp_start; /* 降额起始温度 (C) */
	} thermal_model_t;

	/**
 * @brief   保护参数配置
 */
	typedef struct
	{
		float protect_over_current;	  /* 过流保护阈值 (A) */
		float protect_over_voltage;	  /* 过压保护阈值 (V) */
		float protect_under_voltage;  /* 欠压保护阈值 (V) */
		float protect_over_speed;	  /* 过速保护阈值 (rad/s) */
		float protect_over_temp;	  /* 过温保护阈值 (C) */
		float protect_under_temp;	  /* 欠温保护阈值 (C) */
		int32_t protect_pos_error;	  /* 位置跟随误差限制 (counts) */
		uint32_t protect_enable_mask; /* 保护使能掩码 */
	} protection_param_t;

	/**
 * @brief   关节电机完整配置结构体
 */
	struct motor_param
	{
		motor_instance_t motor_instance;	 /* 电机实例标识 */
		motor_base_t motor_base;			 /* 电机本体参数 */
		gearbox_param_t gearbox_param;		 /* 减速器参数 */
		encoder_param_t encoder_param;		 /* 编码器参数 */
		position_limit_t position_limit;	 /* 位置限位配置 */
		homing_param_t homing_param;		 /* 回零配置参数 */
		current_loop_t current_loop;		 /* 电流环控制参数 */
		position_loop_t position_loop;		 /* 位置速度环控制参数 */
		impedance_ctrl_t impedance_ctrl;	 /* 阻抗控制参数 */
		thermal_model_t thermal_model;		 /* 热模型参数 */
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
 * @return  0=成功, 其他=错误码
 */
	int motor_param_validate(const motor_param_t *cfg);

	/**
 * @brief   打印电机配置所有参数
 * @param   cfg 电机配置指针
 */
	void motor_param_print(const motor_param_t *cfg);

	/******************************************************************************
 * @brief   电机实例标识
 ******************************************************************************/

	/**
 * @brief   获取电机实例ID
 * @param   cfg 电机配置指针
 * @return  电机实例ID
 */
	uint8_t motor_param_get_motor_id(const motor_param_t *cfg);

	/**
 * @brief   设置电机实例ID
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_motor_id(motor_param_t *cfg, uint8_t value);

	/******************************************************************************
 * @brief   电机本体参数
 ******************************************************************************/

	/**
 * @brief   获取定子相电阻
 * @param   cfg 电机配置指针
 * @return  定子相电阻
 */
	float motor_param_get_r(const motor_param_t *cfg);

	/**
 * @brief   设置定子相电阻
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_r(motor_param_t *cfg, float value);

	/**
 * @brief   获取d轴定子电感
 * @param   cfg 电机配置指针
 * @return  d轴定子电感
 */
	float motor_param_get_ld(const motor_param_t *cfg);

	/**
 * @brief   设置d轴定子电感
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_ld(motor_param_t *cfg, float value);

	/**
 * @brief   获取q轴定子电感
 * @param   cfg 电机配置指针
 * @return  q轴定子电感
 */
	float motor_param_get_lq(const motor_param_t *cfg);

	/**
 * @brief   设置q轴定子电感
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_lq(motor_param_t *cfg, float value);

	/**
 * @brief   获取永磁体磁链
 * @param   cfg 电机配置指针
 * @return  永磁体磁链
 */
	float motor_param_get_flux(const motor_param_t *cfg);

	/**
 * @brief   设置永磁体磁链
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_flux(motor_param_t *cfg, float value);

	/**
 * @brief   获取转矩常数
 * @param   cfg 电机配置指针
 * @return  转矩常数
 */
	float motor_param_get_kt(const motor_param_t *cfg);

	/**
 * @brief   设置转矩常数
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_kt(motor_param_t *cfg, float value);

	/**
 * @brief   获取极对数
 * @param   cfg 电机配置指针
 * @return  极对数
 */
	uint8_t motor_param_get_pole_pairs(const motor_param_t *cfg);

	/**
 * @brief   设置极对数
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_pole_pairs(motor_param_t *cfg, uint8_t value);

	/**
 * @brief   获取电机额定电流
 * @param   cfg 电机配置指针
 * @return  电机额定电流
 */
	float motor_param_get_rated_current(const motor_param_t *cfg);

	/**
 * @brief   设置电机额定电流
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_rated_current(motor_param_t *cfg, float value);

	/**
 * @brief   获取电机峰值电流
 * @param   cfg 电机配置指针
 * @return  电机峰值电流
 */
	float motor_param_get_peak_current(const motor_param_t *cfg);

	/**
 * @brief   设置电机峰值电流
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_peak_current(motor_param_t *cfg, float value);

	/**
 * @brief   获取电机最大转速
 * @param   cfg 电机配置指针
 * @return  电机最大转速
 */
	float motor_param_get_max_speed(const motor_param_t *cfg);

	/**
 * @brief   设置电机最大转速
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_max_speed(motor_param_t *cfg, float value);

	/**
 * @brief   获取PWM死区时间
 * @param   cfg 电机配置指针
 * @return  PWM死区时间
 */
	float motor_param_get_dead_time_ns(const motor_param_t *cfg);

	/**
 * @brief   设置PWM死区时间
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_dead_time_ns(motor_param_t *cfg, float value);

	/**
 * @brief   获取电机额定电压
 * @param   cfg 电机配置指针
 * @return  电机额定电压
 */
	float motor_param_get_rated_voltage(const motor_param_t *cfg);

	/**
 * @brief   设置电机额定电压
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_rated_voltage(motor_param_t *cfg, float value);

	/**
 * @brief   获取电机额定转速
 * @param   cfg 电机配置指针
 * @return  电机额定转速
 */
	float motor_param_get_rated_speed_rpm(const motor_param_t *cfg);

	/**
 * @brief   设置电机额定转速
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_rated_speed_rpm(motor_param_t *cfg, float value);

	/**
 * @brief   获取电机额定转矩
 * @param   cfg 电机配置指针
 * @return  电机额定转矩
 */
	float motor_param_get_rated_torque(const motor_param_t *cfg);

	/**
 * @brief   设置电机额定转矩
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_rated_torque(motor_param_t *cfg, float value);

	/**
 * @brief   获取电机峰值转矩
 * @param   cfg 电机配置指针
 * @return  电机峰值转矩
 */
	float motor_param_get_peak_torque(const motor_param_t *cfg);

	/**
 * @brief   设置电机峰值转矩
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_peak_torque(motor_param_t *cfg, float value);

	/**
 * @brief   获取转子转动惯量
 * @param   cfg 电机配置指针
 * @return  转子转动惯量
 */
	float motor_param_get_inertia(const motor_param_t *cfg);

	/**
 * @brief   设置转子转动惯量
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_inertia(motor_param_t *cfg, float value);

	/**
 * @brief   获取反电动势常数
 * @param   cfg 电机配置指针
 * @return  反电动势常数
 */
	float motor_param_get_ke(const motor_param_t *cfg);

	/**
 * @brief   设置反电动势常数
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_ke(motor_param_t *cfg, float value);

	/**
 * @brief   获取PWM载波频率
 * @param   cfg 电机配置指针
 * @return  PWM载波频率
 */
	uint32_t motor_param_get_pwm_freq_hz(const motor_param_t *cfg);

	/**
 * @brief   设置PWM载波频率
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_pwm_freq_hz(motor_param_t *cfg, uint32_t value);

	/**
 * @brief   获取FOC控制频率
 * @param   cfg 电机配置指针
 * @return  FOC控制频率
 */
	uint32_t motor_param_get_foc_freq_hz(const motor_param_t *cfg);

	/**
 * @brief   设置FOC控制频率
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_foc_freq_hz(motor_param_t *cfg, uint32_t value);

	/******************************************************************************
 * @brief   减速器参数
 ******************************************************************************/

	/**
 * @brief   获取减速器减速比
 * @param   cfg 电机配置指针
 * @return  减速器减速比
 */
	float motor_param_get_gear_ratio(const motor_param_t *cfg);

	/**
 * @brief   设置减速器减速比
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_gear_ratio(motor_param_t *cfg, float value);

	/**
 * @brief   获取减速器效率
 * @param   cfg 电机配置指针
 * @return  减速器效率
 */
	float motor_param_get_gear_efficiency(const motor_param_t *cfg);

	/**
 * @brief   设置减速器效率
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_gear_efficiency(motor_param_t *cfg, float value);

	/**
 * @brief   获取输出转矩常数
 * @param   cfg 电机配置指针
 * @return  输出转矩常数
 */
	float motor_param_get_output_torque_const(const motor_param_t *cfg);

	/**
 * @brief   设置输出转矩常数
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_output_torque_const(motor_param_t *cfg, float value);

	/**
 * @brief   获取减速器回程间隙
 * @param   cfg 电机配置指针
 * @return  减速器回程间隙
 */
	float motor_param_get_gear_backlash(const motor_param_t *cfg);

	/**
 * @brief   设置减速器回程间隙
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_gear_backlash(motor_param_t *cfg, float value);

	/******************************************************************************
 * @brief   编码器参数
 ******************************************************************************/

	/**
 * @brief   获取编码器分辨率
 * @param   cfg 电机配置指针
 * @return  编码器分辨率
 */
	uint32_t motor_param_get_enc_lines(const motor_param_t *cfg);

	/**
 * @brief   设置编码器分辨率
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_enc_lines(motor_param_t *cfg, uint32_t value);

	/**
 * @brief   获取编码器计数方向
 * @param   cfg 电机配置指针
 * @return  编码器计数方向
 */
	int8_t motor_param_get_enc_direction(const motor_param_t *cfg);

	/**
 * @brief   设置编码器计数方向
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_enc_direction(motor_param_t *cfg, int8_t value);

	/**
 * @brief   获取编码器初始位置偏移
 * @param   cfg 电机配置指针
 * @return  编码器初始位置偏移
 */
	int32_t motor_param_get_enc_offset(const motor_param_t *cfg);

	/**
 * @brief   设置编码器初始位置偏移
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_enc_offset(motor_param_t *cfg, int32_t value);

	/**
 * @brief   获取电角度偏移
 * @param   cfg 电机配置指针
 * @return  电角度偏移
 */
	float motor_param_get_elec_angle_bias(const motor_param_t *cfg);

	/**
 * @brief   设置电角度偏移
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_elec_angle_bias(motor_param_t *cfg, float value);

	/**
 * @brief   获取位置滤波系数
 * @param   cfg 电机配置指针
 * @return  位置滤波系数
 */
	float motor_param_get_pos_filter_alpha(const motor_param_t *cfg);

	/**
 * @brief   设置位置滤波系数
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_pos_filter_alpha(motor_param_t *cfg, float value);

	/**
 * @brief   获取编码器类型
 * @param   cfg 电机配置指针
 * @return  编码器类型
 */
	uint8_t motor_param_get_enc_type(const motor_param_t *cfg);

	/**
 * @brief   设置编码器类型
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_enc_type(motor_param_t *cfg, uint8_t value);

	/**
 * @brief   获取编码器自动校准
 * @param   cfg 电机配置指针
 * @return  编码器自动校准
 */
	uint8_t motor_param_get_enc_auto_calib(const motor_param_t *cfg);

	/**
 * @brief   设置编码器自动校准
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_enc_auto_calib(motor_param_t *cfg, uint8_t value);

	/**
 * @brief   获取速度观测器增益
 * @param   cfg 电机配置指针
 * @return  速度观测器增益
 */
	float motor_param_get_speed_obs_gain(const motor_param_t *cfg);

	/**
 * @brief   设置速度观测器增益
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_speed_obs_gain(motor_param_t *cfg, float value);

	/******************************************************************************
 * @brief   位置限位配置
 ******************************************************************************/

	/**
 * @brief   获取多圈位置使能
 * @param   cfg 电机配置指针
 * @return  多圈位置使能
 */
	uint8_t motor_param_get_multiturn_enable(const motor_param_t *cfg);

	/**
 * @brief   设置多圈位置使能
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_multiturn_enable(motor_param_t *cfg, uint8_t value);

	/**
 * @brief   获取软件位置负限位
 * @param   cfg 电机配置指针
 * @return  软件位置负限位
 */
	float motor_param_get_pos_min_limit(const motor_param_t *cfg);

	/**
 * @brief   设置软件位置负限位
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_pos_min_limit(motor_param_t *cfg, float value);

	/**
 * @brief   获取软件位置正限位
 * @param   cfg 电机配置指针
 * @return  软件位置正限位
 */
	float motor_param_get_pos_max_limit(const motor_param_t *cfg);

	/**
 * @brief   设置软件位置正限位
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_pos_max_limit(motor_param_t *cfg, float value);

	/**
 * @brief   获取硬件限位开关使能
 * @param   cfg 电机配置指针
 * @return  硬件限位开关使能
 */
	uint8_t motor_param_get_limit_sw_enable(const motor_param_t *cfg);

	/**
 * @brief   设置硬件限位开关使能
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_limit_sw_enable(motor_param_t *cfg, uint8_t value);

	/******************************************************************************
 * @brief   回零配置参数
 ******************************************************************************/

	/**
 * @brief   获取回零方式选择
 * @param   cfg 电机配置指针
 * @return  回零方式选择
 */
	uint8_t motor_param_get_homing_method(const motor_param_t *cfg);

	/**
 * @brief   设置回零方式选择
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_homing_method(motor_param_t *cfg, uint8_t value);

	/**
 * @brief   获取回零快速速度
 * @param   cfg 电机配置指针
 * @return  回零快速速度
 */
	float motor_param_get_homing_speed_fast(const motor_param_t *cfg);

	/**
 * @brief   设置回零快速速度
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_homing_speed_fast(motor_param_t *cfg, float value);

	/**
 * @brief   获取回零慢速速度
 * @param   cfg 电机配置指针
 * @return  回零慢速速度
 */
	float motor_param_get_homing_speed_slow(const motor_param_t *cfg);

	/**
 * @brief   设置回零慢速速度
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_homing_speed_slow(motor_param_t *cfg, float value);

	/**
 * @brief   获取零点位置偏移
 * @param   cfg 电机配置指针
 * @return  零点位置偏移
 */
	float motor_param_get_homing_offset(const motor_param_t *cfg);

	/**
 * @brief   设置零点位置偏移
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_homing_offset(motor_param_t *cfg, float value);

	/**
 * @brief   获取回零电流限制
 * @param   cfg 电机配置指针
 * @return  回零电流限制
 */
	float motor_param_get_homing_current(const motor_param_t *cfg);

	/**
 * @brief   设置回零电流限制
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_homing_current(motor_param_t *cfg, float value);

	/******************************************************************************
 * @brief   电流环控制参数
 ******************************************************************************/

	/**
 * @brief   获取d轴电流比例增益
 * @param   cfg 电机配置指针
 * @return  d轴电流比例增益
 */
	float motor_param_get_current_kp_d(const motor_param_t *cfg);

	/**
 * @brief   设置d轴电流比例增益
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_current_kp_d(motor_param_t *cfg, float value);

	/**
 * @brief   获取d轴电流积分增益
 * @param   cfg 电机配置指针
 * @return  d轴电流积分增益
 */
	float motor_param_get_current_ki_d(const motor_param_t *cfg);

	/**
 * @brief   设置d轴电流积分增益
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_current_ki_d(motor_param_t *cfg, float value);

	/**
 * @brief   获取q轴电流比例增益
 * @param   cfg 电机配置指针
 * @return  q轴电流比例增益
 */
	float motor_param_get_current_kp_q(const motor_param_t *cfg);

	/**
 * @brief   设置q轴电流比例增益
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_current_kp_q(motor_param_t *cfg, float value);

	/**
 * @brief   获取q轴电流积分增益
 * @param   cfg 电机配置指针
 * @return  q轴电流积分增益
 */
	float motor_param_get_current_ki_q(const motor_param_t *cfg);

	/**
 * @brief   设置q轴电流积分增益
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_current_ki_q(motor_param_t *cfg, float value);

	/**
 * @brief   获取电流积分限幅
 * @param   cfg 电机配置指针
 * @return  电流积分限幅
 */
	float motor_param_get_current_integral_limit(const motor_param_t *cfg);

	/**
 * @brief   设置电流积分限幅
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_current_integral_limit(motor_param_t *cfg, float value);

	/**
 * @brief   获取DQ交叉解耦增益
 * @param   cfg 电机配置指针
 * @return  DQ交叉解耦增益
 */
	float motor_param_get_decoupling_gain(const motor_param_t *cfg);

	/**
 * @brief   设置DQ交叉解耦增益
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_decoupling_gain(motor_param_t *cfg, float value);

	/**
 * @brief   获取死区补偿电压
 * @param   cfg 电机配置指针
 * @return  死区补偿电压
 */
	float motor_param_get_deadtime_comp_v(const motor_param_t *cfg);

	/**
 * @brief   设置死区补偿电压
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_deadtime_comp_v(motor_param_t *cfg, float value);

	/**
 * @brief   获取PWM最大占空比
 * @param   cfg 电机配置指针
 * @return  PWM最大占空比
 */
	float motor_param_get_pwm_max_duty(const motor_param_t *cfg);

	/**
 * @brief   设置PWM最大占空比
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_pwm_max_duty(motor_param_t *cfg, float value);

	/**
 * @brief   获取电流环带宽
 * @param   cfg 电机配置指针
 * @return  电流环带宽
 */
	float motor_param_get_current_bandwidth_hz(const motor_param_t *cfg);

	/**
 * @brief   设置电流环带宽
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_current_bandwidth_hz(motor_param_t *cfg, float value);

	/**
 * @brief   获取电流采样滤波系数
 * @param   cfg 电机配置指针
 * @return  电流采样滤波系数
 */
	float motor_param_get_current_filter_alpha(const motor_param_t *cfg);

	/**
 * @brief   设置电流采样滤波系数
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_current_filter_alpha(motor_param_t *cfg, float value);

	/**
 * @brief   获取d轴前馈增益
 * @param   cfg 电机配置指针
 * @return  d轴前馈增益
 */
	float motor_param_get_d_feedforward_gain(const motor_param_t *cfg);

	/**
 * @brief   设置d轴前馈增益
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_d_feedforward_gain(motor_param_t *cfg, float value);

	/**
 * @brief   获取q轴前馈增益
 * @param   cfg 电机配置指针
 * @return  q轴前馈增益
 */
	float motor_param_get_q_feedforward_gain(const motor_param_t *cfg);

	/**
 * @brief   设置q轴前馈增益
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_q_feedforward_gain(motor_param_t *cfg, float value);

	/******************************************************************************
 * @brief   位置速度环控制参数
 ******************************************************************************/

	/**
 * @brief   获取速度环比例增益
 * @param   cfg 电机配置指针
 * @return  速度环比例增益
 */
	float motor_param_get_speed_kp(const motor_param_t *cfg);

	/**
 * @brief   设置速度环比例增益
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_speed_kp(motor_param_t *cfg, float value);

	/**
 * @brief   获取速度环积分增益
 * @param   cfg 电机配置指针
 * @return  速度环积分增益
 */
	float motor_param_get_speed_ki(const motor_param_t *cfg);

	/**
 * @brief   设置速度环积分增益
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_speed_ki(motor_param_t *cfg, float value);

	/**
 * @brief   获取速度环积分限幅
 * @param   cfg 电机配置指针
 * @return  速度环积分限幅
 */
	float motor_param_get_speed_integral_limit(const motor_param_t *cfg);

	/**
 * @brief   设置速度环积分限幅
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_speed_integral_limit(motor_param_t *cfg, float value);

	/**
 * @brief   获取速度前馈增益
 * @param   cfg 电机配置指针
 * @return  速度前馈增益
 */
	float motor_param_get_velocity_ff_gain(const motor_param_t *cfg);

	/**
 * @brief   设置速度前馈增益
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_velocity_ff_gain(motor_param_t *cfg, float value);

	/**
 * @brief   获取加速度前馈增益
 * @param   cfg 电机配置指针
 * @return  加速度前馈增益
 */
	float motor_param_get_accel_ff_gain(const motor_param_t *cfg);

	/**
 * @brief   设置加速度前馈增益
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_accel_ff_gain(motor_param_t *cfg, float value);

	/**
 * @brief   获取位置环比例增益
 * @param   cfg 电机配置指针
 * @return  位置环比例增益
 */
	float motor_param_get_position_kp(const motor_param_t *cfg);

	/**
 * @brief   设置位置环比例增益
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_position_kp(motor_param_t *cfg, float value);

	/**
 * @brief   获取位置环积分限幅
 * @param   cfg 电机配置指针
 * @return  位置环积分限幅
 */
	float motor_param_get_position_integral_limit(const motor_param_t *cfg);

	/**
 * @brief   设置位置环积分限幅
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_position_integral_limit(motor_param_t *cfg, float value);

	/**
 * @brief   获取库仑摩擦力矩
 * @param   cfg 电机配置指针
 * @return  库仑摩擦力矩
 */
	float motor_param_get_friction_coulomb(const motor_param_t *cfg);

	/**
 * @brief   设置库仑摩擦力矩
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_friction_coulomb(motor_param_t *cfg, float value);

	/**
 * @brief   获取粘滞摩擦系数
 * @param   cfg 电机配置指针
 * @return  粘滞摩擦系数
 */
	float motor_param_get_friction_viscous(const motor_param_t *cfg);

	/**
 * @brief   设置粘滞摩擦系数
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_friction_viscous(motor_param_t *cfg, float value);

	/**
 * @brief   获取陷波滤波器频率
 * @param   cfg 电机配置指针
 * @return  陷波滤波器频率
 */
	float motor_param_get_notch_freq_hz(const motor_param_t *cfg);

	/**
 * @brief   设置陷波滤波器频率
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_notch_freq_hz(motor_param_t *cfg, float value);

	/**
 * @brief   获取陷波滤波器带宽
 * @param   cfg 电机配置指针
 * @return  陷波滤波器带宽
 */
	float motor_param_get_notch_width_hz(const motor_param_t *cfg);

	/**
 * @brief   设置陷波滤波器带宽
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_notch_width_hz(motor_param_t *cfg, float value);

	/**
 * @brief   获取陷波滤波器深度
 * @param   cfg 电机配置指针
 * @return  陷波滤波器深度
 */
	float motor_param_get_notch_depth_db(const motor_param_t *cfg);

	/**
 * @brief   设置陷波滤波器深度
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_notch_depth_db(motor_param_t *cfg, float value);

	/**
 * @brief   获取陷波滤波器使能
 * @param   cfg 电机配置指针
 * @return  陷波滤波器使能
 */
	uint8_t motor_param_get_notch_enable(const motor_param_t *cfg);

	/**
 * @brief   设置陷波滤波器使能
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_notch_enable(motor_param_t *cfg, uint8_t value);

	/**
 * @brief   获取速度环带宽
 * @param   cfg 电机配置指针
 * @return  速度环带宽
 */
	float motor_param_get_speed_bandwidth_hz(const motor_param_t *cfg);

	/**
 * @brief   设置速度环带宽
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_speed_bandwidth_hz(motor_param_t *cfg, float value);

	/**
 * @brief   获取速度滤波系数
 * @param   cfg 电机配置指针
 * @return  速度滤波系数
 */
	float motor_param_get_speed_filter_alpha(const motor_param_t *cfg);

	/**
 * @brief   设置速度滤波系数
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_speed_filter_alpha(motor_param_t *cfg, float value);

	/**
 * @brief   获取位置环带宽
 * @param   cfg 电机配置指针
 * @return  位置环带宽
 */
	float motor_param_get_position_bandwidth_hz(const motor_param_t *cfg);

	/**
 * @brief   设置位置环带宽
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_position_bandwidth_hz(motor_param_t *cfg, float value);

	/******************************************************************************
 * @brief   阻抗控制参数
 ******************************************************************************/

	/**
 * @brief   获取阻抗控制位置刚度
 * @param   cfg 电机配置指针
 * @return  阻抗控制位置刚度
 */
	float motor_param_get_impedance_kp(const motor_param_t *cfg);

	/**
 * @brief   设置阻抗控制位置刚度
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_impedance_kp(motor_param_t *cfg, float value);

	/**
 * @brief   获取阻抗控制速度阻尼
 * @param   cfg 电机配置指针
 * @return  阻抗控制速度阻尼
 */
	float motor_param_get_impedance_kd(const motor_param_t *cfg);

	/**
 * @brief   设置阻抗控制速度阻尼
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_impedance_kd(motor_param_t *cfg, float value);

	/**
 * @brief   获取最大输出电流
 * @param   cfg 电机配置指针
 * @return  最大输出电流
 */
	float motor_param_get_iq_max(const motor_param_t *cfg);

	/**
 * @brief   设置最大输出电流
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_iq_max(motor_param_t *cfg, float value);

	/******************************************************************************
 * @brief   热模型参数
 ******************************************************************************/

	/**
 * @brief   获取电机热阻
 * @param   cfg 电机配置指针
 * @return  电机热阻
 */
	float motor_param_get_thermal_resistance(const motor_param_t *cfg);

	/**
 * @brief   设置电机热阻
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_thermal_resistance(motor_param_t *cfg, float value);

	/**
 * @brief   获取热时间常数
 * @param   cfg 电机配置指针
 * @return  热时间常数
 */
	float motor_param_get_thermal_time_const(const motor_param_t *cfg);

	/**
 * @brief   设置热时间常数
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_thermal_time_const(motor_param_t *cfg, float value);

	/**
 * @brief   获取降额起始温度
 * @param   cfg 电机配置指针
 * @return  降额起始温度
 */
	float motor_param_get_derating_temp_start(const motor_param_t *cfg);

	/**
 * @brief   设置降额起始温度
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_derating_temp_start(motor_param_t *cfg, float value);

	/******************************************************************************
 * @brief   保护参数配置
 ******************************************************************************/

	/**
 * @brief   获取过流保护阈值
 * @param   cfg 电机配置指针
 * @return  过流保护阈值
 */
	float motor_param_get_protect_over_current(const motor_param_t *cfg);

	/**
 * @brief   设置过流保护阈值
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_protect_over_current(motor_param_t *cfg, float value);

	/**
 * @brief   获取过压保护阈值
 * @param   cfg 电机配置指针
 * @return  过压保护阈值
 */
	float motor_param_get_protect_over_voltage(const motor_param_t *cfg);

	/**
 * @brief   设置过压保护阈值
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_protect_over_voltage(motor_param_t *cfg, float value);

	/**
 * @brief   获取欠压保护阈值
 * @param   cfg 电机配置指针
 * @return  欠压保护阈值
 */
	float motor_param_get_protect_under_voltage(const motor_param_t *cfg);

	/**
 * @brief   设置欠压保护阈值
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_protect_under_voltage(motor_param_t *cfg, float value);

	/**
 * @brief   获取过速保护阈值
 * @param   cfg 电机配置指针
 * @return  过速保护阈值
 */
	float motor_param_get_protect_over_speed(const motor_param_t *cfg);

	/**
 * @brief   设置过速保护阈值
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_protect_over_speed(motor_param_t *cfg, float value);

	/**
 * @brief   获取过温保护阈值
 * @param   cfg 电机配置指针
 * @return  过温保护阈值
 */
	float motor_param_get_protect_over_temp(const motor_param_t *cfg);

	/**
 * @brief   设置过温保护阈值
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_protect_over_temp(motor_param_t *cfg, float value);

	/**
 * @brief   获取欠温保护阈值
 * @param   cfg 电机配置指针
 * @return  欠温保护阈值
 */
	float motor_param_get_protect_under_temp(const motor_param_t *cfg);

	/**
 * @brief   设置欠温保护阈值
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_protect_under_temp(motor_param_t *cfg, float value);

	/**
 * @brief   获取位置跟随误差限制
 * @param   cfg 电机配置指针
 * @return  位置跟随误差限制
 */
	int32_t motor_param_get_protect_pos_error(const motor_param_t *cfg);

	/**
 * @brief   设置位置跟随误差限制
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_protect_pos_error(motor_param_t *cfg, int32_t value);

	/**
 * @brief   获取保护使能掩码
 * @param   cfg 电机配置指针
 * @return  保护使能掩码
 */
	uint32_t motor_param_get_protect_enable_mask(const motor_param_t *cfg);

	/**
 * @brief   设置保护使能掩码
 * @param   cfg 电机配置指针
 * @param   value 要设置的值
 * @return  0=成功, -EINVAL=参数错误
 */
	int motor_param_set_protect_enable_mask(motor_param_t *cfg, uint32_t value);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_PARAM_H__ */
