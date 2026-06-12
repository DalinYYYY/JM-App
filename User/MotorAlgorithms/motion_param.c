/**
 * @file motion_param.c
 * @brief 
 * 
 * @author Eamon (eamon.zhang@hyfoss-tec.com)
 * @version 1.0
 * @date 2025-04-07
 * 
 * doxdocgen.file.copyrightTag
 * 
 * @par 修改日志:
 * <table>
 * <tr><th>Date       <th>Version <th>Author  <th>Description
 * <tr><td>2025-04-07 <td>1.0     <td>Eamon     <td>初始化
 * <tr><td>2025-04-17 <td>2.0     <td>Dalin     <td>重构代码，增加了电机速度位置获取接口
 * </table>
 */

#include "assert_report.h"
#include "user_config.h"
#include "motion_param.h"
#include "dev_motor.h"
#include <math.h>
#include "imath.h"

#include "dev_motorinfo.h"
#include "motor_control.h"

butterworthFilter_t filter_;
/**
 * @brief 读取 电角度更新计算 状态
 * 
 * @param  pobj             : xxx
 * 
 * @return true 
 * @return false 
 */
static bool motor_get_eleangle_update_status(struct motion_param *pobj)
{
	return pobj->ele_angle_update_status;
}

/**
 * @brief 设置 电角度更新计算 状态
 * 
 * @param  pobj             : xxx
 * @param  status           : xxx
 * 
 * @return true 
 * @return false 
 */
static void motor_set_eleangle_update_status(struct motion_param *pobj, bool status)
{
	pobj->ele_angle_update_status = status;
}

static void motor_set_speed_update_state(struct motion_param *pobj, bool status)
{
	pobj->rpm_update_status = status;
}

static bool motor_get_speed_update_state(struct motion_param *pobj)
{
	return pobj->rpm_update_status;
}

// normalizing angle to [0,360]
static float normalize_angle(float angle)
{
	//Elapsedtime:20us
	//    float a = fmod(angle, 360.0F);
	//    return a >= 0 ? a : (a + 360.0F);

	//Elapsedtime:15us
	//    float remainder = angle - floorf(angle / 360.0F) * 360.0F;
	//    return remainder;

	//Elapsedtime:14us
	// 计算除法次数（向下取整）
	int n = (int)(angle / 360.0F);
	// 计算余数
	float remainder = angle - n * 360.0F;
	// 调整余数到 [0, 360) 范围
	return remainder >= 0 ? remainder : (remainder + 360.0F);
}

static float update_ele_radian(struct motion_param *pobj)
{
	// 计算电角度
	pobj->ele_angle = normalize_angle(pobj->mechanical_angle * (pobj->poles));

	// 计算电弧度
	pobj->ele_radian = pobj->ele_angle * _angle2rad;

	// 设置电角度更新状态为真
	motor_set_eleangle_update_status(pobj, true);
	return pobj->ele_radian;
}

/**
 * @brief 获取电机  角速度（单位：°/s）
 * 
 * @param  pobj             : xxx
 * 
 */
static void update_deg_s(struct motion_param *pobj)
{
	pobj->deg_s = pobj->rad_s * _rad2angle; //度每秒
}

/**
 * @brief 获取转速（单位：rad/s）
 * 
 * @param  pobj             : xxx
 * @param  mechanical_angle : xxx
 * @param  dt               : xxx
 * 
 */
static void update_rad_s(struct motion_param *pobj, float mechanical_angle, uint32_t dt)
{
	static float prev_angle;
	//	static float prev_delta = 0.0f;
	// 计算角度变化量
	float delta = mechanical_angle - prev_angle;
	if (fabs(delta) > 180.0F)
	{
		delta = (delta > 0) ? (delta - 360.0F) : (delta + 360.0F);
	}
	//	const float factor = 0.2f;
	//	delta = delta * factor + prev_delta * (1 - factor);
	//	prev_delta = delta;

	// 计算角速度（rad/s）
	pobj->rad_s = delta * (_angle2rad * dt);									   // _angle2rad = π/180
	pobj->slide_rad_s = slide_average_calculate(&pobj->slide_filter, pobj->rad_s); // 滑动滤波器计算
	// pobj->slide_rad_s = butterworth_process(&filter_, pobj->slide_rad_s); // 低通滤波器计算

	pobj->omegaHistory[0] = pobj->omegaHistory[1];
	pobj->omegaHistory[1] = pobj->omegaHistory[2];
	pobj->omegaHistory[2] = pobj->slide_rad_s;
	float acc = (pobj->omegaHistory[2] - pobj->omegaHistory[1]) * dt;
	// 滑动滤波器计算
	pobj->acceleration = slide_average_calculate(&pobj->slide_acc_filter, acc);

	prev_angle = mechanical_angle;

	motor_set_speed_update_state(pobj, true); // 设置转速更新状态真 1
}

/**
 * @brief 获取电机的RPM（转速）
 * 
 * @param  pobj             : xxx
 * @param  rad_s            : xxx
 * 
 * @return int32_t 
 */
static float update_rpm(struct motion_param *pobj)
{
	pobj->rpm = (float)(pobj->rad_s * _rad2angle * 60 / 360);
	return pobj->rpm;
}

/**
 * @brief 获取电机的位置信息 
 * 
 * @param  pobj : 电机运动参数对象
 * 
 */
static float update_position(struct motion_param *pobj)
{
	static float last_rad = 0;
	static uint16_t ticks = 0;

	// 编码器上电需要稳定一定时间
	if (ticks < 1000)
	{
		ticks++;
		return 0.0F;
	}

	if (pobj->device_compensation_callback == NULL)
	{
		return 0.0F;
	}

	// 将角度转换为弧度
	float curr_rad = pobj->mechanical_angle * _angle2rad - pobj->device_compensation_callback();
	curr_rad = curr_rad > _2pi ? (curr_rad - _2pi) : (curr_rad < 0 ? (curr_rad + _2pi) : curr_rad);

	// 计算弧度差值并无分支修正周期跳变
	float delta = curr_rad - last_rad;

	delta -= _2pi * ((delta > _pi) - (delta < -_pi)); // 处理超过π的跳变

	// 更新累计位置（弧度）
	pobj->position += delta;

	// 计算电机旋转圈数
	pobj->rotation_count = pobj->position / _2pi;

	// 更新上一次的机械角度（存储原始角度值）
	last_rad = curr_rad;

	return pobj->position;
}

/**
 * @brief 获取电机的机械角度
 * 
 * @param  pobj             : xxx
 * 
 */
static float get_mechanical_angle(struct motion_param *pobj)
{
	return pobj->mechanical_angle;
}

static float get_ele_radian(struct motion_param *pobj)
{
	return pobj->ele_radian;
}

static float get_position(struct motion_param *pobj)
{
	return pobj->position;
}

static void set_speed_dt(struct motion_param *pobj, uint32_t dt)
{
	pobj->dt = dt;
}

/* ------------------------------------------ 前馈补偿 ------------------------------------------ */
static void feedforword_compute(struct motion_param *pobj, float expect_angle)
{
	pobj->ff_expect_angle = expect_angle;
	pobj->ff_delta_angle = pobj->ff_expect_angle - pobj->ff_prev_angle;

	// 计算角速度（rad/s）
	pobj->ff_rad_s = pobj->ff_delta_angle * pobj->dt;
	pobj->ff_slide_rad_s = slide_average_calculate(&pobj->ff_vel_filter, pobj->ff_rad_s); // 滑动滤波器计算

	// 计算角加速度（rad/s²）
	pobj->ff_delta_rad_s = pobj->ff_slide_rad_s - pobj->ff_prev_rad_s;
	pobj->ff_accel = pobj->ff_delta_rad_s;
	pobj->ff_slide_acc = slide_average_calculate(&pobj->ff_acc_filter, pobj->ff_accel); // 滑动滤波器计算

	pobj->ff_prev_angle = pobj->ff_expect_angle;
	pobj->ff_prev_rad_s = pobj->ff_slide_rad_s;
}

static float feedforword_get_vel(struct motion_param *pobj)
{
	return pobj->ff_slide_rad_s;
}

static float feedforword_get_acc(struct motion_param *pobj)
{
	return pobj->ff_slide_acc;
}

/**
 * @brief 电机参数数据处理函数
 * 
 * @param  pobj : 电机运动参数对象
 * 
 */
static void motor_param_handle(struct motion_param *pobj, motion_type_e type, float mechanical_angle)
{
	assert_report(pobj != NULL);

	pobj->mechanical_angle = mechanical_angle;

	switch (type)
	{
		case MOTION_TYPE_ELE:
		case MOTION_TYPE_ELE_RADIAN:
		{
			update_ele_radian(pobj);
		}
		break;
		case MOTION_TYPE_ELE_VEL:
		case MOTION_TYPE_ELE_VEL_RADIAN:
		{
			update_ele_radian(pobj);
			update_rad_s(pobj, mechanical_angle, pobj->dt);
			update_deg_s(pobj);
			update_rpm(pobj);
		}
		break;
		case MOTION_TYPE_ELE_POS:
		case MOTION_TYPE_ELE_POS_RADIAN:
		{
			update_ele_radian(pobj);
			update_position(pobj);
		}
		break;

		case MOTION_TYPE_ALL:
		{
			update_ele_radian(pobj);
			update_rad_s(pobj, mechanical_angle, pobj->dt);
			update_deg_s(pobj);
			update_rpm(pobj);
			update_position(pobj);
		}
		break;

		default:
			break;
	}
	// update_rad_s(pobj,mechanical_angle, dt);
	// update_deg_s(pobj);
	// update_rpm(pobj);
	// update_position(pobj);
}

/**
  * @brief 初始化电机参数模块
  * 
  * @param  pobj             : 电机运动参数）对象
  * @param  poles            : 极对数
  * 
  */
void motion_param_init(motion_param_t *pobj, uint8_t poles, uint16_t slide_window_size, float (*device_compensation_callback)(void))
{
	assert_report(pobj != NULL);
	memset(pobj, 0, sizeof(motion_param_t));

	const double sample_rate = 200; // 采样率1kHz
	const double cutoff_freq = 50;	// 截止频率100Hz
	const double Q = 0.707;			// 巴特沃斯特征Q值
	butterworth_init(&filter_, cutoff_freq, sample_rate, Q);

	pobj->poles = poles;						 // 极对数
	pobj->slide_window_size = slide_window_size; // 滑动滤波器窗口大小

	slide_filter_init(&pobj->slide_filter, pobj->slide_window_size); // 初始化滑动滤波器
	slide_filter_init(&pobj->slide_acc_filter, 5);					 // 初始化滑动滤波器

	slide_filter_init(&pobj->ff_vel_filter, 5);
	slide_filter_init(&pobj->ff_acc_filter, 5);

	// 状态函数
	pobj->set_eleangle_status = motor_set_eleangle_update_status;
	pobj->get_eleangle_status = motor_get_eleangle_update_status;
	pobj->set_speed_update_state = motor_set_speed_update_state;
	pobj->get_speed_update_state = motor_get_speed_update_state;

	// 获取接口函数
	pobj->get_ele_radian = get_ele_radian;
	pobj->get_rpm = update_rpm;
	pobj->get_mechanical_angle = get_mechanical_angle;
	pobj->get_position = get_position;

	pobj->set_speed_dt = set_speed_dt;

	pobj->dt = 2000; // 2000hz

	pobj->device_compensation_callback = device_compensation_callback;

	// 更新函数
	pobj->update = motor_param_handle;

	// 前馈补偿
	pobj->feedforword_compute = feedforword_compute;
	pobj->feedforword_get_vel = feedforword_get_vel;
	pobj->feedforword_get_acc = feedforword_get_acc;
}
