
#include "drv_rtos.h"

#include "thread_idle.h"
#include "thread_config.h"
#include "runtime_param.h"
#include "cogging_comp.h" /* 标定表异步落盘(线程上下文) + 落盘状态 */
#if defined(USE_DEV_FLASH)
#include "motor_info_storage.h" /* 落盘结果回写 motor_info 使能位 */
#endif

void idle_thread(void const *argument)
{
	drv_rtos_delay_ms(INTO_THREAD_DELAY);

	for (;;)
	{

		drv_rtos_delay_ms(THREAD_DELAY_IDLE);
		usr.sys.task_cnt.idle_cnt++;

		/* 齿槽标定表落盘: L5.1 在 10kHz 标定上下文仅登记待写表(RAM),
		 * Flash 擦写必须在线程上下文执行(擦写期间该 Bank 不能取指, ISR
		 * 直接擦写会与 Bank2 代码执行冲突)。无待写时为一次标志读, 零开销。
		 * 状态门控: 仅在功率输出安全的状态落盘——
		 *   RUN: 电流环闭环带功率, 擦写全程关中断 10~40ms 会冻结非零 PWM;
		 *   CALIB: 标定会话直接驱动 PWM 施加电压;
		 *   READY: cur_loop 每拍把 PWM 驱到零, 擦写安全。
		 * 待写期间进 RUN/带电标定已被 process_ctrl_cmd 互锁阻断,
		 * 故落盘窗口内状态不可能切换到带功率态(usr.motor_state 在
		 * 每条指令后同步刷新, 读到的即最新状态)。 */
		{
			top_fsm_e top = usr.motor_state[M1].top_state;
			if (top != TOP_FSM_RUN && top != TOP_FSM_CALIB)
				(void)cogging_comp_flush();
		}

		/* 落盘结果回写: 失败自动关补偿(表不可信), 成功保持使能。
		 * 上位机读 PID184 即可判断落盘链路: 重标后读到 0=Flash 擦写失败。 */
		{
			uint8_t st = cogging_comp_get_write_state();
			if (st == 2u || st == 3u)
			{
				uint8_t en = (st == 2u) ? 1u : 0u;
				usr.motor_param[M1].position_loop.cogging_comp_enable = en;
#if defined(USE_DEV_FLASH)
				{
					motor_info_t *info = motor_info_storage_get();
					if (info != NULL)
						info->blocks.advanced.cogging_comp_enable = en;
				}
#endif
				cogging_comp_clear_write_state();
			}
		}
	}
}
