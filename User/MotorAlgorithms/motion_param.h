/**
 * @file motion_param.h
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
 * <tr><td>2025-04-07 <td>1.0     <td>Eamon     <td>Init
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


//#include "dev_motor.h"
#include "user_config.h"
#include "ifilter.h"

typedef enum
{
	MOTOR_ID_1 = 0,
	MOTOR_ID_MAX
} motor_param_id_e;

typedef enum
{
    MOTION_TYPE_NONE = 0,           // 数据类型：无
	MOTION_TYPE_ELE  = 1,           // 数据类型：电角度
    MOTION_TYPE_ELE_RADIAN = 2,     // 数据类型：电角度弧度
    MOTION_TYPE_ELE_VEL = 3,        // 数据类型：电角度速度
    MOTION_TYPE_ELE_VEL_RADIAN = 4, // 数据类型：电角度速度弧度
    MOTION_TYPE_ELE_POS = 5,        // 数据类型：电角度位置
    MOTION_TYPE_ELE_POS_RADIAN = 6, // 数据类型：电角度位置弧度
	MOTION_TYPE_ALL = 0x0F,			// 数据类型：电角度，速度，位置
	
} motion_type_e;

typedef struct motion_param
{
    motor_param_id_e    id;                 // 电机ID

    volatile float mechanical_angle;        // 当前机械角度 [0 ~ 360°]
    uint8_t poles;                          // 极对数
    volatile float ele_angle;               // 电角度
    volatile float ele_radian;              // 电弧度

    uint32_t dt;                            // 速度计算周期
    uint16_t slide_window_size;             // 滑动窗口大小
    slide_filter_t slide_filter;            // 滑动滤波器
    int32_t deg_s;                          // 度每秒
    int32_t slide_deg_s;                    // 度每秒（滑动滤波）
    volatile float rad_s;                   // 弧度每秒
    volatile float slide_rad_s;             // 弧度每秒（滑动滤波）
    int32_t rpm;                            // 转速
    bool rpm_update_status;                 // 转速更新状态
    bool ele_angle_update_status;           // 电角度更新状态

    float ref_rad_s;                         // 参考电弧度每秒
    float ref_acc;
    float angleHistory[3];                  // 角度历史数据
    float omegaHistory[3];                  // 角速度历史数据 
    float velocity;                         // 当前角速度(rad/s)
    float acceleration;                      // 当前角加速度(rad/s²)
    slide_filter_t slide_acc_filter;         // 滑动加速度滤波器

    // 位置参数
    volatile float position;                // 累计位置（±∞双精度）
    int rotation_count;                     // 整圈计数器（带符号64位）
    volatile float last_mechanical_rad ;    // 上一次机械角度 [0 ~ 360°]
    
    // 前馈补偿参数
    float ff_expect_angle;                  // 期望前馈角度
    float ff_prev_angle;                    // 上一次前馈角度
    float ff_delta_angle;                   // 前馈补偿角度
    float ff_rad_s;                      // 前馈补偿速度
    float ff_slide_rad_s;                  // 前馈补偿速度（滑动滤波）
    float ff_prev_rad_s;                   // 上一次前馈速度
    float ff_delta_rad_s;                   // 前馈补偿速度
    float ff_accel;                         // 前馈补偿加速度
    float ff_slide_acc;                     // 前馈补偿加速度（滑动滤波）
    slide_filter_t ff_vel_filter;            // 滑动滤波器
    slide_filter_t ff_acc_filter;            // 滑动滤波器

    /* 外部输入回调函数 */
    float (*device_compensation_callback)(void); 

    /*public*/
    bool (*get_eleangle_status)(struct motion_param *pobj);                     //用来读取电角度更新的状态
    void (*set_eleangle_status)(struct motion_param *pobj, bool status);        //用来设置电角度更新的状态
    bool (*get_speed_update_state)(struct motion_param *pobj);                  //用来读取电角度更新的状态
    void (*set_speed_update_state)(struct motion_param *pobj, bool status);     //用来设置电角度更新的状态
    
    float (*get_ele_radian)(struct motion_param *pobj);                         //获取电弧度
    float (*get_rpm)(struct motion_param *pobj);                                //获取转速
    float (*get_mechanical_angle)(struct motion_param *pobj);                   //获取机械角度
    float (*get_position)(struct motion_param *pobj);                           //获取位置

    void (*set_speed_dt)(struct motion_param *pobj, uint32_t dt);        //用来设置电角度更新的状态

    void (*update)(struct motion_param *pobj, motion_type_e type, float mechanical_angle); //更新运动参数

    // 前馈补偿
    void (*feedforword_compute)(struct motion_param *pobj, float expect_angle); //前馈补偿
    float (*feedforword_get_vel) (struct motion_param *pobj); //获取前馈补偿速度
    float (*feedforword_get_acc) (struct motion_param *pobj); //获取前馈补偿加速度
} motion_param_t;
void motion_param_init(motion_param_t *pobj, uint8_t poles, uint16_t slide_window_size, float (*device_compensation_callback)(void));
#endif
