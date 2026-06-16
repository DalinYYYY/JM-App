/**
 * @file        user_interface.c
 * @brief       用户接口层（硬件初始化 + 线程创建 + 主循环调度）
 * 
 * @author      name (name@robot.com)
 * @version     1.0
 * @date        2026-06-16
 * 
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 * 
 * 
 * @par 修改日志:
 * | 日期       | 版本 | 作者   | 修改内容   |
 * |------------|------|--------|------------|
 * | 2026-06-16     | 1.0  | yangsl | 初始创建   |
 * 
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 */

#include "user_interface.h"
#include "thread_management.h"
#include "dev_dwt_counter.h"
#include "motor_loop.h"
#include "motor_loop_config.h"
#include "runtime_param.h"
#include "tim.h" // TODO: 避免直接依赖具体外设头，改为抽象接口（如 timer.h），或通过 control_irq.c 传入时钟频率等参数实现解耦

static void hardware_init(void)
{
	/* 初始化DWT定时器 */
	dev_dwt_counter_init();

	/* 初始化电机三环控制（dev_motor + 状态机 + 级联控制）
     * 电流环频率由 ADC 注入转换中断决定，此处传入实际中断频率 */
	motor_loop_init(10000.0f);
}

void user_init(void)
{
	hardware_init();

	/* 创建线程 */
	thread_init();

	HAL_TIM_Base_Start_IT(&htim2); /* 启动定时器更新中断，进入 user_control 调周期执行 */
	HAL_TIM_Base_Start_IT(&htim5); /* 启动定时器更新中断，进入 motor_virtual_loop 调周期执行 */
}

void user_control(void)
{

	//    motor_ctrl_loop();
}

void motor_virtual_loop(void)
{
	// 在使用虚拟电机时，三环控制在中断里执行，主循环无需调用
#if (MOTOR_LOOP_ENABLE_DEV_DRIVER == 0u)
	dev_dwt_counter_stop(SYS_TIMER_RECORD_CURRENT_LOOP_CYCLE); // 测量电流环周期
	dev_dwt_counter_start(SYS_TIMER_RECORD_CURRENT_LOOP_CYCLE);

	dev_dwt_counter_start(SYS_TIMER_RECORD_CURRENT_LOOP_TIME); // 测量电流环运行时间

	// 三环控制入口（电流20kHz / 速度4kHz / 位置2kHz 分频）
	motor_loop_isr();

	dev_dwt_counter_stop(SYS_TIMER_RECORD_CURRENT_LOOP_TIME);
#endif
}
