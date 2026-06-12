#ifndef __RUNTIME_PARAM_H__
#define __RUNTIME_PARAM_H__

#include <stdbool.h>
#include <stdint.h>
#include "motor_param.h"
#include "state_define.h"

#define OFFSET_LUT_NUM 128
#define DT (1.0f / 20000.0f)

/************************************* 枚举变量 *************************************/
typedef enum
{
	M1 = 0,
	MOTOR_MAX,
} motor_num_e;

typedef enum
{
	MOTOR_INFO_CTRL_IDLE = 0,	   // 空闲
	MOTOR_INFO_CTRL_UPDATE = 0x01, // 保存参数
	MOTOR_INFO_CTRL_AMEND = 0x02,  // 修改参数
	MOTOR_INFO_CTRL_CLEAR = 0x03,  // 清空参数
	MOTOR_INFO_CTRL_MAX,
} motor_info_ctrl_e;

typedef enum
{
	ENCODER_DIR_NOCHANGE = 0, // 方向未变化
	ENCODER_MODE_CHANGE = 1,  // 方向已变化
} encoder_dir_e;

typedef enum
{
	SYS_TIMER_RECORD_CURRENT_LOOP_CYCLE = 0,  // 电流环周期
	SYS_TIMER_RECORD_CURRENT_LOOP_TIME = 1,	  // 电流环耗时
	SYS_TIMER_RECORD_POSITION_LOOP_CYCLE = 2, // 位置环周期
	SYS_TIMER_RECORD_POSITION_LOOP_TIME = 3,  // 位置环耗时
	SYS_TIMER_RECORD_TIM_1MS_CYCLE = 4,		  // 1ms定时器周期
	SYS_TIMER_RECORD_TIM_1MS_TIME = 5,		  // 1ms定时器耗时
	SYS_TIMER_RECORD_TEST_1 = 6,			  // 测试1
	SYS_TIMER_RECORD_TEST_2 = 7,			  // 测试2
	SYS_TIMER_RECORD_TEST_3 = 8,			  // 测试3
	SYS_TIMER_RECORD_TEST_4 = 9,			  // 测试4
} sys_timer_record_index_e;

/************************************* 三级变量 *************************************/
typedef union base_version_
{
	struct
	{
		char version_build;
		char version_patch;
		char version_minor;
		char version_major;
	} byte;
	uint32_t var;
} base_version_t;

typedef struct sys_version_
{
	base_version_t boot;
	base_version_t app;
	base_version_t hardware;
} sys_version_t;

typedef struct
{
	float zero_ud;
	float zero_ud_offset;
	float uq;
	float iq;
	float id;
	float velocity;
	float pos_rad;
	float target_pos_rad;
	float target_velocity;
	float target_current;
} sys_control_data_t;

typedef struct task_count_
{
	uint32_t control_cnt;
	uint32_t commun_cnt;
	uint32_t period_cnt;
	uint32_t display_cnt;
	uint32_t idle_cnt;
	uint32_t foc_open_static_cnt;
	uint32_t foc_open_dynamic_cnt;
	uint32_t foc_open_velocity_cnt;
	uint32_t foc_current_cnt;
	uint32_t foc_velocity_cnt;
	uint32_t foc_position_cnt;
	uint32_t foc_position_variable_freq_cnt;
	uint32_t vofa_update_cnt;
	uint32_t vofa_default_cnt;
} system_task_cnt_t;

/************************************* 二级变量 *************************************/
typedef struct
{
	bool enable_motor;
	bool enable_pwm;
	run_state_e run_mode;
	ctrl_mode_e ctrl_mode;
	sys_control_data_t ctrl_data;
} motor_state_t;

typedef struct
{
	sys_version_t version;
	system_task_cnt_t task_cnt;
} system_t;

typedef struct
{
	top_fsm_e top_state;			   // 使用统一的top_fsm_e
	top_fsm_e motor_fsm[MOTOR_MAX];	   // 统一为top_fsm_e，删除重复的motorFsm_e
	ctrl_mode_e motor_mode[MOTOR_MAX]; // 使用统一的ctrl_mode_e
} fsm_t;

/************************************* 一级变量 *************************************/
typedef struct sys_data_
{
	fsm_t fsm;
	system_t sys;
	motor_state_t motor_state[MOTOR_MAX];
	motor_param_t motor_param[MOTOR_MAX];
} sys_data_t;

void user_data_init(void);

/**************************************** 数据接口 ****************************************/
extern sys_data_t usr; // 全局变量加g_前缀

#endif /* __RUNTIME_PARAM_H__ */
