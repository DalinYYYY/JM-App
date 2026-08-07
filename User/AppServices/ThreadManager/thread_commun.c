/**
 * @file thread_commun.c
 * @brief 通信线程实现：周期驱动上位机 UART/CAN 通信设备 
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

#include "drv_rtos.h"

#include "runtime_param.h"
#include "thread_config.h"
#include "thread_commun.h"
#include "main.h"
#include "jm_host_commun.h"

void commun_thread(void const *argument)
{
	drv_rtos_delay_ms(INTO_THREAD_DELAY / 5);

#if defined(USE_DEV_COMMUN_UART)
	/* 关节电机上位机串口通信(joint_proto): 初始化设备→注入业务回调→启动接收 */
	jm_host_commun_init();
#endif

#if defined(USE_DEV_COMMUN_CAN)
	/* 关节电机上位机 CAN/CAN-FD 通信(joint_proto): 与 UART 并存, 业务回调共用 */
	jm_host_commun_can_init();
#endif

	for (;;)
	{
#if defined(USE_DEV_COMMUN_UART)
		/* 上位机通信周期处理: 命令分发应答 + 遥控使能时按订阅周期推送遥测帧(无应答) */
		jm_host_commun_process();
#endif

#if defined(USE_DEV_COMMUN_CAN)
		/* CAN 通信周期处理: 诊断刷新 + 通信中断降级检查 */
		jm_host_commun_can_process();
#endif

		usr.sys.task_cnt.commun_cnt++;
#if defined(USE_DEV_COMMUN_UART)
		jm_host_commun_wait(THREAD_DELAY_COMMUN);
#else
		drv_rtos_delay_ms(THREAD_DELAY_COMMUN * 1);
#endif
	}
}
