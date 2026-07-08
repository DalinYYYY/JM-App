
#include "control_irq.h"
#include "dev_dwt_counter.h"
#include "adc.h"
#include "motor_loop.h"

// 电流环中断任务 典型频率  20KHZ
void CURRENT_LOOP_IRQ_TASK(ADC_HandleTypeDef *hadc)
{
	if (hadc->Instance == UVW_CURRENT_U_HANDLE.Instance)
	{
		dev_dwt_counter_stop(SYS_TIMER_RECORD_CURRENT_LOOP_CYCLE); // 测量电流环周期
		dev_dwt_counter_start(SYS_TIMER_RECORD_CURRENT_LOOP_CYCLE);

		dev_dwt_counter_start(SYS_TIMER_RECORD_CURRENT_LOOP_TIME); // 测量电流环运行时间

		// 三环控制入口（电流10kHz / 速度2kHz / 位置1kHz 分频）
		motor_loop_isr();

		dev_dwt_counter_stop(SYS_TIMER_RECORD_CURRENT_LOOP_TIME);
	}
}
