/**
 * @file thread_period.c
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

#include "drv_rtos.h"

#include "runtime_param.h"
#include "thread_period.h"
#include "thread_config.h"
#include "dev_power_monitor.h"

void power_monitor_update(void)
{
	/* 电源监控刷新(100ms 周期, 母线电压变化缓慢足够)
	 * vbus/ibus 字段供 SVPWM 归一化(calib_hw)使用 */
	dev_power_monitor.update(&dev_power_monitor);
	(void)dev_power_monitor.get_vbus(&dev_power_monitor);
	(void)dev_power_monitor.get_ibus(&dev_power_monitor);

	/* 同步电源监控到电机实时参数(usr.motor_state.power)
	 * 任务层直接写 usr, 与中断 publish_power_thermal 解耦, 100ms 足够遥测 */
	{
		motor_power_t *p = &usr.motor_state[M1].power;
		p->v_bus = dev_power_monitor.vbus;
		p->i_bus = dev_power_monitor.ibus;
		p->power_elec_w = dev_power_monitor.vbus * dev_power_monitor.ibus;
	}
	usr.motor_state[M1].thermal.temp_fet = dev_power_monitor.temp_driver;
	usr.motor_state[M1].thermal.temp_motor = dev_power_monitor.temp_motor;
}

void period_thread(void const *argument)
{

	/* Infinite loop */
	drv_rtos_delay_ms(INTO_THREAD_DELAY / 5);

	for (;;)
	{
		drv_rtos_delay_ms(THREAD_DELAY_PERIOD);

		/* 电源监控刷新 */
		power_monitor_update();

		/* 任务计数 */
		usr.sys.task_cnt.period_cnt++;
	}
}
