
#include "control_irq.h"
#include "dev_dwt_counter.h"
#include "adc.h"
#include "motor_loop.h"
#include "motor_observer.h"

// 电流环中断任务 典型频率  10KHZ
void CURRENT_LOOP_IRQ_TASK(ADC_HandleTypeDef *hadc)
{
	if (hadc->Instance == UVW_CURRENT_U_HANDLE.Instance)
	{
		dev_dwt_counter_stop(SYS_TIMER_RECORD_CURRENT_LOOP_CYCLE); // 测量电流环周期
		dev_dwt_counter_start(SYS_TIMER_RECORD_CURRENT_LOOP_CYCLE);

		dev_dwt_counter_start(SYS_TIMER_RECORD_CURRENT_LOOP_TIME); // 测量电流环运行时间

		// 三环控制入口（电流10kHz / 速度2kHz / 位置1kHz 分频）
		motor_loop_isr();
		/* 控制完成后的唯一观察点：按需高速采样，并按位置环分频发布实时快照。 */
		motor_observer_on_control_isr(motor_loop_get());

		dev_dwt_counter_stop(SYS_TIMER_RECORD_CURRENT_LOOP_TIME);
	}
}
