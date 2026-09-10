/**
 * @file        user_interface.c
 * @brief       用户接口层（硬件初始化 + 线程创建 + 主循环调度）
 * 
 * @author      yangsl (yangsl@robot.com)
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
#include "dev_power_monitor.h"
#include "dev_commun_uart.h"
#if defined(USE_DEV_EEPROM)
#include "dev_eeprom.h"
/* 注：dev_eeprom 上电自检(dev_eeprom_test)会覆盖 motor_info 的 EEPROM 存储区，此处不启用 */
#endif
#if defined(USE_DEV_DRV8301)
#include "dev_drv8301.h"
#endif
#if defined(USE_DEV_LED)
#include "led_manager.h"
#endif
#include "motor_info_storage.h"
#include "fault_manager.h"
#include "motor_loop.h"
#include "motor_observer.h"
#include "motor_loop_config.h"
#include "runtime_param.h"
#if (MOTOR_LOOP_ENABLE_DEV_DRIVER == 0u)
/* 虚拟电机模式: 依赖 htim2/htim5 触发周期中断
 * (真实电机模式 MOTOR_LOOP_ENABLE_DEV_DRIVER==1 由 ADC 注入中断驱动, 不需要 tim.h) */
#include "tim.h"
#endif

static void hardware_init(void)
{
	/* 初始化DWT定时器 */
	dev_dwt_counter_init();

#if defined(USE_DEV_EEPROM)
	/* 片外 EEPROM(AT24C16) 先于存储服务初始化：
	 * 存储服务在 motor_info_storage_init 加载时优先读 EEPROM，
	 * 须在此前就绪并绑定到 g_motor_info_storage.eeprom_dev。 */
	dev_eeprom_init(&dev_eeprom, EEPROM_ID_1);
#endif

	/* 初始化 motor_info Flash+EEPROM 存储服务（须在 motor_loop_init 之前） */
#if defined(USE_DEV_FLASH)
#if defined(USE_DEV_EEPROM)
	g_motor_info_storage.eeprom_dev = &dev_eeprom; /* 挂接 EEPROM 子设备，上电优先加载 */
#endif
	motor_info_storage_init();
#endif

	/* 电源监控(规则组ADC+DMA): 初始化并启动, 供 vbus/ibus 遥测与 SVPWM 归一化使用
	 * 须在 motor_loop_init 之前启动规则组 DMA(独立于注入组, 不依赖 TIM1 触发) */
	dev_power_monitor_init(&dev_power_monitor);
	(void)dev_power_monitor.start(&dev_power_monitor);

#if defined(USE_DEV_DRV8301)
	/* DRV8301 SPI 寄存器配置(须在 motor_loop_init/dev_motor_enable 之前)
	 * CTRL1=0x003C: GAIN=40V/V(D2:D1=10b), DC_CAL=0(正常模式), OCTW=111(默认保护)
	 *   注: DRV8301 上电默认 DC_CAL=1(校准模式), SO1/SO2 输出固定电压, 电流采样恒为0,
	 *       必须通过 SPI 写入 CTRL1 清除 DC_CAL 位才能正常采样电流。
	 * CTRL2=0x0006: GATE_CURRENT=3.0A(D2:D1=11b), 6PWM mode(D7=0), OCP=current limit(D5:D4=00) */
	dev_drv8301_init(&g_dev_drv8301, DRV8301_ID_1);
	g_dev_drv8301.init(&g_dev_drv8301);
#endif

	/* 初始化故障管理器，必须早于 motor_loop_init 及控制中断启动。 */
	fault_mgr_init();

	/* 初始化电机三环控制（dev_motor + 状态机 + 级联控制）
     * 电流环频率由 ADC 注入转换中断决定，此处传入实际中断频率 */
	motor_loop_init(10000.0f);

#if defined(USE_DEV_LED)
	/* LED 状态指示初始化（LED1红=故障, LED2绿=运行状态）
	 * 须在 motor_loop_init 之后调用, 依赖状态机实例就绪 */
	led_manager_init();
#endif

	/* 调试映射指针绑定：指向已存在的全局变量地址（不复制数据）
	 * 须在上述各 init 完成后绑定，此时对象地址与内容均已就绪，
	 * 调试时通过 usr.p_xxx 实时反映对象最新值。
	 * s_motor_loop 经 getter 绑定，避免直接访问伪私有变量。 */
	usr.p_dwt_timer = &dwt_timer;
	usr.p_motor_loop = motor_loop_get();
#if defined(USE_DEV_FLASH)
	usr.p_motor_info_storage = &g_motor_info_storage;
#else
	usr.p_motor_info_storage = NULL;
#endif
	usr.p_dev_power_monitor = &dev_power_monitor;
	usr.p_dev_commun_uart = &dev_commun_uart;
}

void user_init(void)
{
	/* 先初始化运行数据和 MCU UID，后续硬件/协议服务均依赖 usr。 */
	user_data_init();
	hardware_init();

	/* 创建线程 */
	thread_init();

#if (MOTOR_LOOP_ENABLE_DEV_DRIVER == 0u)
	HAL_TIM_Base_Start_IT(&htim2); /* 启动 htim2 更新中断，周期进入 user_control 执行 */
	HAL_TIM_Base_Start_IT(&htim5); /* 启动 htim5 更新中断，周期进入 motor_virtual_loop 执行 */
#endif
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

	// 三环控制入口（电流10kHz / 速度2kHz / 位置1kHz 分频）
	motor_loop_isr();
	motor_observer_on_control_isr(motor_loop_get());

	dev_dwt_counter_stop(SYS_TIMER_RECORD_CURRENT_LOOP_TIME);
#endif
}
