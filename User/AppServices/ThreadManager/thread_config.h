/**
 * @file thread_config.h
 * @brief 线程延时与使能配置 
 * 
 * @author dalin (dalinyy@163.com)
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

#ifndef _THREAD_CONFIG_H_
#define _THREAD_CONFIG_H_

/* 任务周期控制 */

#define THREAD_DELAY_CONTROL 1
#define THREAD_DELAY_COMMUN  1
#define THREAD_DELAY_PERIOD  20
#define THREAD_DELAY_DISPLAY 10
#define THREAD_DELAY_IDLE    1000

/* 任务启动延时 */
#define INTO_THREAD_DELAY 500

// -------------------定义可视化配置---------------------
//***<<< Use Configuration Wizard in Context Menu >>>***

//  <h> VERSION INFO
// <s>Thread Config Version
//  <i>Driver configuration file version
#define THREAD_CONFIG_VERSION "0.0.1"
//  </h>

// <h>MCU ENABLE THREAD

// <c1>
// ENABLE THREAD ---> IDLE
#define USE_IDLE_THREAD
// </c>

// <c1>
// ENABLE THREAD ---> CONTROL
#define USE_CONTROL_THREAD
// </c>

// <c1>
// ENABLE THREAD ---> COMMUN
#define USE_COMMUN_THREAD
// </c>

// <c1>
// ENABLE THREAD ---> DISPLAY
#define USE_DISPLAY_THREAD
// </c>

// <c1>
// ENABLE THREAD ---> PERIOD
#define USE_PERIOD_THREAD
// </c>

// </h>

#endif /* _THREAD_CONFIG_H_ */
