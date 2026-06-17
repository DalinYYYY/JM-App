/**
 * @file        drv_delay.h
 * @brief       延时驱动接口，基于DWT周期计数器实现us/ms级阻塞延时
 *
 * @author      Dalin (dalin@robot.com)
 * @version     1.0
 * @date        2026-06-16
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @par 修改日志:
 * | 日期       | 版本 | 作者   | 修改内容   |
 * |------------|------|--------|------------|
 * | 2026-06-16 | 1.0  | Dalin  | 初始创建   |
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 * @note        基于DWT计数器，使用前需调用drv_dwt_timer_init初始化DWT
 */
#ifndef _DRV_DELAY_H_
#define _DRV_DELAY_H_

#include <stdint.h>

/**
 * @brief       微秒级阻塞延时
 * @param        us                : 延时时长 (单位: us)
 */
void drv_delay_us(uint32_t us);

/**
 * @brief       毫秒级阻塞延时
 * @param        ms                : 延时时长 (单位: ms)
 */
void drv_delay_ms(uint32_t ms);

#endif /* _DRV_DELAY_H_ */
