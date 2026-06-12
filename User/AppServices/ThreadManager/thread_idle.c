
#include "drv_rtos.h"

#include "thread_idle.h"
#include "thread_config.h"
#include "runtime_param.h"

void idle_thread(void const *argument)
{
	drv_rtos_delay_ms(INTO_THREAD_DELAY);

	for (;;)
	{

		drv_rtos_delay_ms(THREAD_DELAY_IDLE);
		/* 任务计数 */
		usr.sys.task_cnt.idle_cnt++;
	}
}
