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
#if defined(USE_DEV_COMMUN_CAN)
#include "drv_can.h"
#endif

#if defined(USE_DEV_COMMUN_CAN)
/* CAN 测试接口: 周期发送固定 ID 的原始测试帧, 供外接 CAN 盒子检查物理链路。
 * 仅用于调试, 确认链路正常后应删除或置 0 关闭。 */
#define CAN_TEST_ID        0x123u /* 测试帧标准 ID */
#define CAN_TEST_PERIOD_MS 100u   /* 发送周期(ms) */
#define CAN_TEST_ENABLED   0u     /* 1=使能周期发送, 0=关闭 */

static void can_test_frame_send(void)
{
	static uint32_t s_counter = 0u;
	drvCanMsg_t msg;

	memset(&msg, 0, sizeof(msg));
	msg.id = CAN_TEST_ID;
	msg.ide = 0u; /* 标准帧 */
	msg.rtr = 0u;
	msg.len = 8u;
	msg.is_fd = 0u; /* 经典 CAN 帧 */
	msg.data[0] = 'T';
	msg.data[1] = 'E';
	msg.data[2] = 'S';
	msg.data[3] = 'T';
	msg.data[4] = (uint8_t)(s_counter >> 0u);
	msg.data[5] = (uint8_t)(s_counter >> 8u);
	msg.data[6] = (uint8_t)(s_counter >> 16u);
	msg.data[7] = (uint8_t)(s_counter >> 24u);
	s_counter++;
	drv_can_send(DRV_CAN1, &msg);
}
#endif

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

#if CAN_TEST_ENABLED
		/* 周期发送原始测试帧, 供外接 CAN 盒子检查物理链路 */
		{
			static uint32_t can_test_tick = 0u;
			if (++can_test_tick >= (CAN_TEST_PERIOD_MS / THREAD_DELAY_COMMUN))
			{
				can_test_tick = 0u;
				can_test_frame_send();
			}
		}
#endif
#endif

		usr.sys.task_cnt.commun_cnt++;
#if defined(USE_DEV_COMMUN_UART)
		jm_host_commun_wait(THREAD_DELAY_COMMUN);
#else
		drv_rtos_delay_ms(THREAD_DELAY_COMMUN * 1);
#endif
	}
}
