/**
 * @file motion_param.h
 * @brief 电机运动参数解算模块
 *
 * 输入机械角度，按照配置的极对数 / 更新频率解算出：
 *   - 电角度 / 电弧度
 *   - 角速度（deg/s、rad/s、滑动滤波、rpm）与角加速度
 *   - 位置（单编码器软件累圈 或 多齿轮游标(Nonius)绝对多圈）
 *   - 前馈补偿（速度 / 加速度）
 *
 * 多圈计数支持三种模式：
 *   - 软件累圈：单编码器靠过零跳变累加圈数（掉电丢失）
 *   - 双齿轮游标：主齿轮 + 1 个副齿轮，靠齿数互质用游标原理重建绝对圈数
 *   - 三齿轮游标：主齿轮 + 2 个副齿轮，扩展量程并交叉校验抗跳变
 *
 * @author Eamon (eamon.zhang@hyfoss-tec.com)
 * @version 3.0
 * @date 2025-04-07
 *
 * @par 修改日志:
 * <table>
 * <tr><th>Date       <th>Version <th>Author  <th>Description
 * <tr><td>2025-04-07 <td>1.0     <td>Eamon   <td>初始化
 * <tr><td>2025-04-17 <td>2.0     <td>Dalin   <td>重构代码，增加速度位置获取接口
 * <tr><td>2026-06-12 <td>3.0     <td>Dalin   <td>完全重构，模块自包含，新增齿轮游标绝对多圈
 * </table>
 */
/*Better Comments插件的使用方法   特别说明*/
//!：用于突出显示重要的注释或需要特别关注的部分；
//?：用于表示疑问或需要进一步解释的注释；
//TODO：用于标记需要完成的任务或待办事项；
//*：用于强调或标记注释中的关键信息；
//：用于普通的注释。

#ifndef __MOTOR_MOTION_PARAM_H_
#define __MOTOR_MOTION_PARAM_H_

#include <stdint.h>
#include <stdbool.h>

/* 副齿轮最大数量（三齿轮游标：主齿轮 + 2 副齿轮） */
#define MOTION_NONIUS_GEAR_MAX 3u
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
	MOTION_TYPE_ELE_VEL = 3,		// 数据类型：电角度速度
	MOTION_TYPE_ELE_VEL_RADIAN = 4, // 数据类型：电角度速度弧度
	MOTION_TYPE_ELE_POS = 5,		// 数据类型：电角度位置
	MOTION_TYPE_ELE_POS_RADIAN = 6, // 数据类型：电角度位置弧度
	MOTION_TYPE_ALL = 0x0F,			// 数据类型：电角度，速度，位置
} motion_type_e;

/**
 * @brief 多圈计数模式
 */
typedef enum
{
	MULTITURN_MODE_NONE = 0,	 // 不计多圈，position 即单圈累计
	MULTITURN_MODE_SOFT,		 // 单编码器软件累圈（掉电丢失，原有逻辑）
	MULTITURN_MODE_NONIUS_2GEAR, // 双齿轮游标绝对多圈
	MULTITURN_MODE_NONIUS_3GEAR, // 三齿轮游标绝对多圈（扩展量程 + 容错）
} multiturn_mode_e;

/**
 * @brief 齿轮游标(Nonius)配置
 *
 * 齿轮链以主齿轮（与电机/输出轴同轴的测量齿轮）为基准。配套的副齿轮齿数与主齿轮
 * 齿数互质（或近似互质），两个齿轮的相位差在一个游标周期内单调变化，由此可反推
 * 主齿轮转过的整圈数。
 *
 * gear_teeth[0] / gear_dir[0] 为主齿轮，其余为副齿轮。
 */
typedef struct
{
	multiturn_mode_e mode;						 // 多圈模式
	uint16_t gear_teeth[MOTION_NONIUS_GEAR_MAX]; // 各齿轮齿数（[0]=主齿轮）
	int8_t gear_dir[MOTION_NONIUS_GEAR_MAX];	 // 各齿轮计数方向 (+1/-1)
} multiturn_config_t;

/**
 * @brief 运动参数模块初始化配置
 */
typedef struct
{
	uint8_t poles;								 // 极对数
	uint16_t slide_window_size;					 // 速度滑动滤波窗口（<= MOTION_SLIDE_WINDOW_MAX）
	uint32_t update_freq_hz;					 // 速度/位置解算频率 (Hz)，用于 d(angle)/dt
	multiturn_config_t multiturn;				 // 多圈配置
	float (*device_compensation_callback)(void); // 设备角度补偿回调（rad），可为 NULL
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
 * @brief 运动参数对象
 */
typedef struct motion_param
{
	motor_param_id_e id; // 电机ID

	/* ---- 配置 ---- */
	uint8_t poles;					  // 极对数
	uint32_t update_freq_hz;		  // 解算频率 (Hz)
	multiturn_config_t multiturn_cfg; // 多圈配置

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

	/* ---- 位置 / 多圈 ---- */
	volatile float position;		   // 累计位置 (rad，单圈累积或软件多圈)
	int32_t rotation_count;			   // 整圈计数
	volatile float multiturn_position; // 绝对多圈位置 (rad)
	int32_t multiturn_turns;		   // 绝对多圈整圈数
	float last_single_rad;			   // 上一次单圈弧度（软件累圈用）
	uint16_t pos_settle_ticks;		   // 上电稳定计数

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

	/* ---- 外部输入回调 ---- */
	float (*device_compensation_callback)(void); // 设备角度补偿 (rad)

	/* ---- 状态接口 ---- */
	bool (*get_eleangle_status)(struct motion_param *pobj);
	void (*set_eleangle_status)(struct motion_param *pobj, bool status);
	bool (*get_speed_update_state)(struct motion_param *pobj);
	void (*set_speed_update_state)(struct motion_param *pobj, bool status);

	/* ---- 获取接口 ---- */
	float (*get_ele_radian)(struct motion_param *pobj);			// 电弧度 (rad)
	float (*get_rpm)(struct motion_param *pobj);				// 转速 (rpm)
	float (*get_mechanical_angle)(struct motion_param *pobj);	// 机械角度 (deg)
	float (*get_position)(struct motion_param *pobj);			// 位置 (rad，按多圈模式)
	float (*get_multiturn_position)(struct motion_param *pobj); // 绝对多圈位置 (rad)
	int32_t (*get_turns)(struct motion_param *pobj);			// 绝对整圈数

	/* ---- 配置接口 ---- */
	void (*set_update_freq)(struct motion_param *pobj, uint32_t freq_hz);

	/* ---- 更新接口 ---- */
	// 单角度入口（兼容旧调用）：仅主轴角度，多圈走 SOFT 模式
	void (*update)(struct motion_param *pobj, motion_type_e type, float mechanical_angle);
	// 多角度入口：gear_angles[0]=主轴角，其余为副齿轮角；count 为齿轮数量
	void (*update_ex)(struct motion_param *pobj, motion_type_e type,
					  const float *gear_angles, uint8_t count);

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
 * @brief 初始化运动参数模块（兼容旧签名：单编码器软件累圈）
 * @param pobj 运动参数对象
 * @param poles 极对数
 * @param slide_window_size 速度滑动滤波窗口
 * @param device_compensation_callback 角度补偿回调 (rad)，可为 NULL
 */
void motion_param_init(motion_param_t *pobj, uint8_t poles, uint16_t slide_window_size,
					   float (*device_compensation_callback)(void));

#endif /* __MOTOR_MOTION_PARAM_H_ */
