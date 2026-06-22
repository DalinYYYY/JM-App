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
#include "dev_commun_uart.h"

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
	const sys_control_data_t *d = &usr.motor_state[M1].ctrl_data;

	//	v->id = d->id;						  /* 直轴电流 A */
	//	v->iq = d->iq;						  /* 交轴电流 A */
	//	v->current_motor = d->iq;			  /* 电机电流 A (近似取 iq) */
	//	v->rpm = d->velocity;				  /* 转速 (单位按实际换算: 若为 rad/s 需转 ERPM) */
	//	v->pid_pos = d->pos_rad * RAD_TO_DEG; /* PID 位置 ° */
	//	v->fault_code = 0;
	//	v->controller_id = 0;
	v->id = 0.23;			 /* 直轴电流 A */
	v->iq = 0.45;			 /* 交轴电流 A */
	v->current_motor = 0.45; /* 电机电流 A (近似取 iq) */
	v->rpm = 1200.0f;		 /* 转速 (单位按实际换算: 若为 rad/s 需转 ERPM) */
	v->pid_pos = 45.0f;		 /* PID 位置 ° */
	v->fault_code = 0;
	v->controller_id = 0;
}
#endif /* USE_DEV_COMMUN_VESC */

#if defined(USE_DEV_COMMUN_UART)
/* ---------------- joint_proto 业务回调: 上位机命令的真正动作落点 ---------------- */

/* 设置控制模式并下发目标(CMD 0x00~0xB8) */
static jm_err_e commun_uart_set_mode(uint8_t cmd, const uint8_t *payload, uint16_t len)
{
	(void)cmd;
	(void)payload;
	(void)len;
	/* TODO: 按 cmd 解析载荷并下发到电机控制层 */
	return JM_ERR_OK;
}

/* 读实时反馈: 从运行参数填充 */
static jm_err_e commun_uart_get_feedback(jm_feedback_t *fb)
{
	const sys_control_data_t *d = &usr.motor_state[M1].ctrl_data;

	fb->pos = d->pos_rad;  /* 输出端位置 rad */
	fb->vel = d->velocity; /* 输出端速度 rad/s */
	fb->torque = 0.0f;	   /* 输出端力矩 Nm (暂无) */
	fb->id = d->id;		   /* d轴电流 A */
	fb->iq = d->iq;		   /* q轴电流 A */
	fb->ia = fb->ib = fb->ic = 0.0f;
	fb->vbus = 0.0f;
	fb->ibus = 0.0f;
	fb->temp_fet = 0.0f;
	fb->temp_motor = 0.0f;
	fb->multiturn = 0;
	fb->single = d->pos_rad;
	fb->fault_mask = 0;
	fb->warn_mask = 0;
	fb->top_fsm = (uint8_t)usr.fsm.motor_fsm[M1];
	fb->run_state = (uint8_t)usr.motor_state[M1].run_mode;
	fb->ctrl_mode = (uint8_t)usr.fsm.motor_mode[M1];
	fb->enable = usr.motor_state[M1].enable_motor ? 1 : 0;
	return JM_ERR_OK;
}

/* 读单个参数 */
static jm_err_e commun_uart_param_read(uint16_t param_id, uint8_t *value,
									   uint8_t *out_type, uint8_t *out_len)
{
	(void)param_id;
	(void)value;
	(void)out_type;
	(void)out_len;
	/* TODO: 按 param_id 查参数表并填充 value/out_type/out_len */
	return JM_ERR_BAD_PARAM_ID;
}

/* 写单个参数 */
static jm_err_e commun_uart_param_write(uint16_t param_id, const uint8_t *value, uint8_t len)
{
	(void)param_id;
	(void)value;
	(void)len;
	/* TODO: 按 param_id 写入参数表 */
	return JM_ERR_BAD_PARAM_ID;
}

static const jm_proto_ops_t commun_uart_ops = {
	.set_mode = commun_uart_set_mode,
	.get_feedback = commun_uart_get_feedback,
	.param_read = commun_uart_param_read,
	.param_write = commun_uart_param_write,
	.param_save = NULL,
	.param_reset = NULL,
	.get_dev_info = NULL,
	.get_dev_name = NULL,
};
#endif /* USE_DEV_COMMUN_UART */

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
	/* 关节电机串口通信(USART+DMA空闲中断): 初始化→注入业务回调→启动接收 */
	dev_commun_uart_init(&dev_commun_uart, JM_UART_COMM_ID_1);
	dev_commun_uart.set_ops(&dev_commun_uart, &commun_uart_ops);
	dev_commun_uart.start(&dev_commun_uart);
#endif

	for (;;)
	{
		// vofa_update();

#if defined(USE_DEV_COMMUN_VESC)
		/* 取空闲突发数据喂协议栈, 自动完成识别握手与实时值回复 */
		dev_commun_vesc.poll(&dev_commun_vesc);
#endif

#if defined(USE_DEV_COMMUN_UART)
		/* 取空闲突发数据喂协议栈, 自动完成命令分发与应答 */
		dev_commun_uart.poll(&dev_commun_uart);
#endif

		usr.sys.task_cnt.commun_cnt++;
		drv_rtos_delay_ms(THREAD_DELAY_COMMUN * 1);
	}
}
