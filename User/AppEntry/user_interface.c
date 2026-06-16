/**
 * @file user_interface.c
 * @brief 
 * 
 * @author dalin (dalin@robot.com)
 * @version 1.0
 * @date 2026-06-10
 * 
 * @copyright Copyright (c) 2026 Robot Tech.co, Ltd. All rights reserved.
 * 
 * @par 修改日志:
 * <table>
 * <tr><th>日期</th><th>版本</th><th>作者</th><th>修改内容</th></tr>
 * <tr><td>2026-06-10</td><td>1.0</td><td>yangsl</td><td>初始创建</td></tr>
 * </table>
 * 
 * @note 本文件遵循《嵌入式C代码规范V1.0》开发
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
