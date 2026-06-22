/**
 * @file thread_communication.c
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
#include "thread_config.h"
#include "thread_commun.h"
#include "vofa.h"
#include "main.h"
#include "dev_commun_vesc.h"
#include "serialstudio_commun.h"

void vofa_update(void)
{
#define VOFA_MAX 10
	float vofa_buf[VOFA_MAX] = {0.0F};

	vofa_upload((uint8_t *)vofa_buf, VOFA_MAX * 4);
}

#if defined(USE_DEV_COMMUN_VESC)
#define RAD_TO_DEG (57.2957795f) /* 弧度转角度 (180/π) */

/* VESC Tool 仪表盘取值回调: 从运行参数填充实时量 */
static void commun_vesc_fill_values(vesc_values_t *v)
{
	const motor_state_t *m = &usr.motor_state[M1];

	v->id = m->electrical.id_meas;					  /* 直轴电流 A */
	v->iq = m->electrical.iq_meas;					  /* 交轴电流 A */
	v->current_motor = m->electrical.iq_meas;		  /* 电机电流 A (近似取 iq) */
	v->rpm = m->motion.velocity_rad_s;				  /* 转速 (rad/s, 如需 ERPM 另换算) */
	v->pid_pos = m->motion.position_rad * RAD_TO_DEG; /* PID 位置 ° */
	v->fault_code = 0;
	v->controller_id = 0;
}
#endif /* USE_DEV_COMMUN_VESC */

void commun_thread(void const *argument)
{
	drv_rtos_delay_ms(INTO_THREAD_DELAY / 5);

#if defined(USE_DEV_COMMUN_VESC)
	/* VESC Tool 串口通信(USART+DMA空闲中断): 初始化协议栈→注入数据源→启动接收 */
	dev_commun_vesc_init(&dev_commun_vesc, VESC_COMM_ID_1);
	dev_commun_vesc.set_values_cb(&dev_commun_vesc, commun_vesc_fill_values);
	dev_commun_vesc.start(&dev_commun_vesc);
#endif

#if defined(USE_DEV_COMMUN_UART)
	/* SerialStudio 串口通信(joint_proto): 初始化设备→注入业务回调→启动接收 */
	ss_commun_init();
#endif

	for (;;)
	{
		// vofa_update();

#if defined(USE_DEV_COMMUN_VESC)
		/* 取空闲突发数据喂协议栈, 自动完成识别握手与实时值回复 */
		dev_commun_vesc.poll(&dev_commun_vesc);
#endif

#if defined(USE_DEV_COMMUN_UART)
		/* SerialStudio 通信周期处理: 命令分发应答 + 按订阅周期推送遥测帧 */
		// ss_commun_process();
#endif

		usr.sys.task_cnt.commun_cnt++;
		drv_rtos_delay_ms(THREAD_DELAY_COMMUN * 1);
	}
}
