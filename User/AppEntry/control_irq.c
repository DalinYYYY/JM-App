
#include "control_irq.h"
#include "dev_dwt_counter.h"
#include "adc.h"
#include "motor_loop.h"

// 电流环中断任务 典型频率  10KHZ
void CURRENT_LOOP_IRQ_TASK(ADC_HandleTypeDef *hadc)
{
	if (hadc->Instance == UVW_CURRENT_U_HANDLE.Instance)
	{
		/* [诊断] 标记注入转换完成时刻: 用示波器 CH2 探 PC13, 与 CH1(A相运放输出)叠看,
		 * 判断 ADC 采样点是否落在电流平台上。PC13 上升沿≈转换完成, 采样时刻在其前约1µs。
		 * 若上升沿落在 CH1 的 REF 凹槽/开关瞬态上 → 采样时刻错开电流平台(触发相位问题)。
		 * 诊断完成后删除本段。 */
		HAL_GPIO_WritePin(TEST_IO1_GPIO_Port, TEST_IO1_Pin, GPIO_PIN_SET);

		dev_dwt_counter_stop(SYS_TIMER_RECORD_CURRENT_LOOP_CYCLE); // 测量电流环周期
		dev_dwt_counter_start(SYS_TIMER_RECORD_CURRENT_LOOP_CYCLE);

		dev_dwt_counter_start(SYS_TIMER_RECORD_CURRENT_LOOP_TIME); // 测量电流环运行时间

		// 三环控制入口（电流10kHz / 速度2kHz / 位置1kHz 分频）
		motor_loop_isr();

		dev_dwt_counter_stop(SYS_TIMER_RECORD_CURRENT_LOOP_TIME);

		HAL_GPIO_WritePin(TEST_IO1_GPIO_Port, TEST_IO1_Pin, GPIO_PIN_RESET); /* [诊断] */
	}
}
