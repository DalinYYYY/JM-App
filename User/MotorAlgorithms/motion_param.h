/**
 * @file        motion_param.h
 * @brief       电机运动参数解算模块（仅角度/速度，不含多圈计数）
 * 
 * @author      Dalin (dalin@robot.com)
 * @version     1.0
 * @date        2026-06-12
 * 
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 * 
 * 输入机械角度，按照配置的极对数 / 更新频率解算出：
 *   - 电角度 / 电弧度
 *   - 机械角度（透传）
 *   - 角速度（deg/s、rad/s、滑动滤波、rpm）与角加速度
 *   - 前馈补偿（速度 / 加速度）
 *
 * 多圈/绝对位置计数已拆分到独立的 multiturn 模块（multiturn.h），二者由调用方
 * 组合使用：本模块吃机械角度出运动量，multiturn 吃齿轮角度出绝对圈数/位置。
 * 
 * @par 修改日志:
 * | 日期       | 版本 | 作者   | 修改内容   |
 * |------------|------|--------|------------|
 * | 2026-06-12     | 1.0  | yangsl | 初始创建   |
 * | 2026-06-15     | 1.1  | yangsl | 拆分：多圈计数移至 multiturn 模块，本模块仅保留角度/速度   |
 * 
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 */

#ifndef __MOTOR_MOTION_PARAM_H_
#define __MOTOR_MOTION_PARAM_H_

#include <stdint.h>
#include <stdbool.h>

/* 滑动滤波窗口最大长度（编译期分配，避免动态内存） */
#define MOTION_SLIDE_WINDOW_MAX 32u

/**
 * @brief 电机ID
 */
typedef enum
{
	MOTOR_ID_1 = 0,
	MOTOR_ID_MAX
} motor_param_id_e;

/**
 * @brief update 时需要解算的数据类型
 */
typedef enum
{
	MOTION_TYPE_NONE = 0,			// 数据类型：无
	MOTION_TYPE_ELE = 1,			// 数据类型：电角度
	MOTION_TYPE_ELE_RADIAN = 2,		// 数据类型：电角度弧度
	MOTION_TYPE_ELE_VEL = 3,		// 数据类型：电角度 + 速度
	MOTION_TYPE_ELE_VEL_RADIAN = 4, // 数据类型：电角度弧度 + 速度
	MOTION_TYPE_ALL = 0x0F,			// 数据类型：电角度 + 速度 + 加速度
} motion_type_e;

/**
 * @brief 运动参数模块初始化配置
 */
typedef struct
{
	uint8_t poles;				// 极对数
	uint16_t slide_window_size; // 速度滑动滤波窗口（<= MOTION_SLIDE_WINDOW_MAX）
	uint32_t update_freq_hz;	// 速度解算频率 (Hz)，用于 d(angle)/dt
} motion_param_config_t;

/**
 * @brief 轻量滑动平均滤波器（模块自包含，编译期分配）
 */
typedef struct
{
	float buf[MOTION_SLIDE_WINDOW_MAX];
	uint16_t size;	// 实际窗口长度
	uint16_t head;	// 写入位置
	uint16_t count; // 已填充样本数
	float sum;		// 窗口内样本和
} motion_slide_filter_t;

/**
 * @brief 运动参数对象（角度 / 速度 / 加速度 / 前馈）
 */
typedef struct motion_param
{
	motor_param_id_e id; // 电机ID

	/* ---- 配置 ---- */
	uint8_t poles;			 // 极对数
	uint32_t update_freq_hz; // 解算频率 (Hz)

	/* ---- 角度 / 电角度 ---- */
	volatile float mechanical_angle; // 当前机械角度 [0 ~ 360°]
	volatile float ele_angle;		 // 电角度 (deg)
	volatile float ele_radian;		 // 电弧度 (rad)
	bool ele_angle_update_status;	 // 电角度更新状态

	/* ---- 速度 / 加速度 ---- */
	motion_slide_filter_t slide_filter;		// 速度滑动滤波器
	motion_slide_filter_t slide_acc_filter; // 加速度滑动滤波器
	float prev_mech_angle;					// 上一次机械角度（速度解算用）
	int32_t deg_s;							// 度每秒
	volatile float rad_s;					// 弧度每秒
	volatile float slide_rad_s;				// 弧度每秒（滑动滤波）
	int32_t rpm;							// 转速
	float omegaHistory[3];					// 角速度历史（加速度解算用）
	float acceleration;						// 当前角加速度 (rad/s^2)
	bool rpm_update_status;					// 转速更新状态

	/* ---- 前馈补偿 ---- */
	float ff_expect_angle;				 // 期望前馈角度
	float ff_prev_angle;				 // 上一次前馈角度
	float ff_delta_angle;				 // 前馈补偿角度
	float ff_rad_s;						 // 前馈补偿速度
	float ff_slide_rad_s;				 // 前馈补偿速度（滑动滤波）
	float ff_prev_rad_s;				 // 上一次前馈速度
	float ff_delta_rad_s;				 // 前馈补偿速度差
	float ff_accel;						 // 前馈补偿加速度
	float ff_slide_acc;					 // 前馈补偿加速度（滑动滤波）
	motion_slide_filter_t ff_vel_filter; // 前馈速度滤波器
	motion_slide_filter_t ff_acc_filter; // 前馈加速度滤波器

	/* ---- 状态接口 ---- */
	bool (*get_eleangle_status)(struct motion_param *pobj);
	void (*set_eleangle_status)(struct motion_param *pobj, bool status);
	bool (*get_speed_update_state)(struct motion_param *pobj);
	void (*set_speed_update_state)(struct motion_param *pobj, bool status);

	/* ---- 获取接口 ---- */
	float (*get_ele_radian)(struct motion_param *pobj);		  // 电弧度 (rad)
	float (*get_rpm)(struct motion_param *pobj);			  // 转速 (rpm)
	float (*get_mechanical_angle)(struct motion_param *pobj); // 机械角度 (deg)

	/* ---- 配置接口 ---- */
	void (*set_update_freq)(struct motion_param *pobj, uint32_t freq_hz);

	/* ---- 更新接口 ---- */
	void (*update)(struct motion_param *pobj, motion_type_e type, float mechanical_angle);

	/* ---- 前馈补偿接口 ---- */
	void (*feedforword_compute)(struct motion_param *pobj, float expect_angle);
	float (*feedforword_get_vel)(struct motion_param *pobj);
	float (*feedforword_get_acc)(struct motion_param *pobj);
} motion_param_t;

/**
 * @brief 初始化运动参数模块（推荐：配置结构方式）
 * @param pobj 运动参数对象
 * @param cfg  初始化配置
 */
void motion_param_init_cfg(motion_param_t *pobj, const motion_param_config_t *cfg);

/**
 * @brief 初始化运动参数模块（兼容旧签名）
 * @param pobj 运动参数对象
 * @param poles 极对数
 * @param slide_window_size 速度滑动滤波窗口
 * @note 第三参数（旧的设备补偿回调）已移至 multiturn 模块，这里保留占位以兼容旧调用。
 */
void motion_param_init(motion_param_t *pobj, uint8_t poles, uint16_t slide_window_size,
					   float (*unused_compensation_callback)(void));

#endif /* __MOTOR_MOTION_PARAM_H_ */
