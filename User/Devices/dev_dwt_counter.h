#ifndef _DEV_DWT_COUNTER_H_
#define _DEV_DWT_COUNTER_H_

#include "drv_dwt_timer.h"
#ifdef __cplusplus
extern "C"
{
#endif

#define SYS_TIMER_RECORD_MAX_INDEX 10
	typedef struct
	{
		uint32_t now_records[SYS_TIMER_RECORD_MAX_INDEX];	   /* 记录各索引对应的起始时刻(DWT_CYCCNT计数值) */
		uint32_t duration_records[SYS_TIMER_RECORD_MAX_INDEX]; /* 记录各索引对应的持续时长(时钟周期数) */
		uint32_t sys_freq_hz;								   /* 系统时钟频率，单位Hz */
		float ticks_to_us;									   /* 时钟周期数→微秒的换算系数(1e6/freq)，用于快速换算 */
	} dwtTimer_t;

	/**
 * @brief  初始化DWT计数器（含底层周期计数器使能与频率换算系数计算）
 */
	void dev_dwt_counter_init(void);

	/**
 * @brief  打点：记录指定索引的起始时刻
 * @param  index 计数器索引(0 ~ SYS_TIMER_RECORD_MAX_INDEX-1)，越界则忽略
 */
	void dev_dwt_counter_start(uint8_t index);

	/**
 * @brief  打点：记录从start到此刻的持续周期数
 * @param  index 计数器索引(0 ~ SYS_TIMER_RECORD_MAX_INDEX-1)，越界则忽略
 */
	void dev_dwt_counter_stop(uint8_t index);

	/**
 * @brief  获取指定索引的持续时间
 * @param  index 计数器索引(0 ~ SYS_TIMER_RECORD_MAX_INDEX-1)
 * @return 持续时间(单位: 微秒)，索引越界返回0
 */
	float dev_dwt_counter_get_duration_us(uint8_t index);

	/**
 * @brief  微秒级阻塞延时
 * @param  us 延时时长(单位: 微秒)
 */
	void dev_dwt_counter_blocking_delay_us(uint32_t us);

	/**
 * @brief  毫秒级阻塞延时
 * @param  ms 延时时长(单位: 毫秒)
 */
	void dev_dwt_counter_blocking_delay_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* _DEV_DWT_COUNTER_H_ */
