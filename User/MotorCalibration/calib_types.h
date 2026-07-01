#ifndef __CALIB_TYPES_H__
#define __CALIB_TYPES_H__

#include <stdint.h>
#include <stdbool.h>
#include "motor_param.h"

/* 前向声明，避免 calib_types.h 直接依赖 dev_motor.h */
struct dev_motor;

/* ===================== 标定子状态 ===================== */
typedef enum
{
	CALIB_STATE_IDLE = 0, /* 空闲（未开始或已完成） */
	CALIB_STATE_RUNNING,  /* 标定进行中 */
	CALIB_STATE_DONE,     /* 标定完成（成功） */
	CALIB_STATE_FAILED,   /* 标定失败 */
} calib_state_e;

/* ===================== 标定级别常量 ===================== */
#define CALIB_LEVEL1_DRIVER 1    /* 驱动硬件底层 */
#define CALIB_LEVEL2_MOTOR 2     /* 电机电气身份 */
#define CALIB_LEVEL3_ENCODER 3   /* 编码器校准 */
#define CALIB_LEVEL4_TORQUE 4    /* 转矩基础 */
#define CALIB_LEVEL5_NONLINEAR 5 /* 非线性补偿 */
#define CALIB_LEVEL6_SYSTEM 6    /* 负载系统级 */
#define CALIB_LEVEL7_AUTO 7      /* 自动化集成 */
#define CALIB_LEVEL_MAX 8

/* ===================== L1 子模式: 驱动硬件底层 ===================== */
#define CALIB_L1_ADC_OFFSET 1     /* ADC偏置 */
#define CALIB_L1_ADC_GAIN 2       /* ADC增益 */
#define CALIB_L1_CURRENT_SENSOR 3 /* 电流传感器 */
#define CALIB_L1_TEMP_SENSOR 4    /* 温度传感器 */
#define CALIB_L1_VBUS 5           /* 母线电压采样 */
#define CALIB_L1_DEADTIME 6       /* 驱动死区特性 */

/* ===================== L2 子模式: 电机电气身份 =====================
 * 原 CALIB_L2_RL_FLUX(3) 拆分为 4 个独立子模式：
 * R/Ld/Lq/flux 物理上需不同测试方法（DC法/阶跃响应/反电势法），
 * 拆分后可独立触发与重试。submode 编号 3-6 连续，协议层 payload[0] 透传无需改动。*/
#define CALIB_L2_PHASE_SEQ    1 /* 相序识别 */
#define CALIB_L2_POLE_PAIRS   2 /* 极对数 */
#define CALIB_L2_RESISTANCE   3 /* R 相电阻辨识（DC法）*/
#define CALIB_L2_INDUCTANCE_D 4 /* Ld d轴电感辨识（阶跃响应）*/
#define CALIB_L2_INDUCTANCE_Q 5 /* Lq q轴电感辨识（阶跃响应）*/
#define CALIB_L2_FLUX_LINKAGE 6 /* flux 磁链辨识（反电势法）*/

/* ===================== L3 子模式: 编码器校准 ===================== */
#define CALIB_L3_ZERO_OFFSET 1    /* 零位 */
#define CALIB_L3_DIRECTION 2      /* 方向校验 */
#define CALIB_L3_LINEARITY 3      /* 线性度 */
#define CALIB_L3_SINCOS 4         /* 正余弦/旋变幅值相位 */
#define CALIB_L3_MULTITURN_ZERO 5 /* 多圈绝对值零点 */

/* ===================== L4 子模式: 转矩基础 ===================== */
#define CALIB_L4_KT 1 /* 力矩常数 */

/* ===================== L5 子模式: 非线性补偿 ===================== */
#define CALIB_L5_COGGING 1       /* 齿槽补偿 */
#define CALIB_L5_FRICTION 2      /* 摩擦补偿 */
#define CALIB_L5_DEADTIME_COMP 3 /* 逆变器死区补偿 */
#define CALIB_L5_SATURATION 4    /* 电感磁饱和补偿 */

/* ===================== L6 子模式: 负载系统级 ===================== */
#define CALIB_L6_INERTIA 1      /* 负载惯量 */
#define CALIB_L6_DAMPING 2      /* 负载阻尼 */
#define CALIB_L6_BACKLASH 3     /* 传动回程间隙 */
#define CALIB_L6_PID_AUTOTUNE 4 /* 控制环参数自整定 */

/* ===================== L7 子模式: 自动化集成 ===================== */
#define CALIB_L7_FULL_AUTO 1 /* 一键全自动 */

/* ===================== 标定进度与结果 ===================== */
typedef struct
{
	calib_state_e state; /* 标定子状态 */
	uint8_t progress;    /* 进度 0-100 */
	uint8_t level;       /* 当前标定级别（1-7） */
	uint8_t submode;     /* 当前子模式 */
	uint8_t step;        /* 当前步骤（L7 全自动用，单步标定为 0） */
} calib_status_t;

/* ===================== 级别模块 ops 函数表 =====================
 * 每个级别模块导出一个 const calib_level_ops_t 实例，
 * calib_mgr 按 level 索引查表调用。
 * start 返回 false 表示 submode 不支持，calib_mgr 据此拒绝启动。*/
typedef struct
{
	bool (*start)(uint8_t submode, motor_param_t *param, float dt);
	calib_state_e (*poll)(void);
	void (*abort)(void);
} calib_level_ops_t;

/* ===================== 标定硬件访问接口 =====================
 * 标定模块需要施加电压、读取编码器、操作 FOC 链路等硬件能力。
 * calib_mgr_init 注入 dev_motor_t 指针，calib_mgr_get_motor() 对所有级别模块可见。
 * 这样避免每个 level 模块各自 extern 引用 motor_loop_get()，解耦标定与控制层。*/
typedef struct
{
	struct dev_motor *motor; /* 底层电机设备（FOC/编码器/半桥） */
	motor_param_t *param;    /* 电机参数（标定结果写入目标） */
	float dt;                /* 控制周期(s) */
} calib_io_t;

#endif /* __CALIB_TYPES_H__ */
