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
}

void user_control(void)
{
	// 在使用虚拟电机时，三环控制在中断里执行，主循环无需调用
#if (MOTOR_LOOP_ENABLE_DEV_DRIVER == 0u)
	motor_loop_isr();
#endif
	//    motor_ctrl_loop();
}
