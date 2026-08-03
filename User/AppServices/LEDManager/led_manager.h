/**
 * @file        led_manager.h
 * @brief       LED 状态映射管理器（顶层状态机 → LED1/LED2 行为）
 *
 * @author      Dalin (dalinyy@163.com)
 * @version     1.0
 * @date        2026-07-21
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @par 修改日志:
 * | 日期       | 版本 | 作者   | 修改内容   |
 * |------------|------|--------|------------|
 * | 2026-07-21 | 1.0  | Dalin  | 初始创建   |
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 * @note        分层: led_manager(业务层) → dev_led(设备对象) → drv_gpio
 *              读取 motor_loop_get()->sys.top_state 决定 LED 行为:
 *              LED1(红): FAULT 常亮 / SAFETY+BOOTLOADER 快闪 / 其他常灭
 *              LED2(绿): RUN 常亮 / INIT+READY+CONFIG 慢闪 / CALIB+BOOTLOADER 快闪 / 其他常灭
 */
#ifndef __LED_MANAGER_H__
#define __LED_MANAGER_H__

#include "dev_config.h"
#if defined(USE_DEV_LED)

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief 初始化 LED 管理器（初始化对象池 + 默认常灭）
 * @note  须在 motor_loop_init() 之后调用（依赖状态机就绪）
 */
void led_manager_init(void);

/**
 * @brief LED 状态周期更新（由 thread_display 10ms 周期调用）
 * @note  读取顶层状态机, 仅在状态变化时重写 GPIO, 每拍调用 dev_led.update 处理闪烁
 */
void led_manager_update(void);

/**
 * @brief CAN-DI物理识别临时覆盖，使用状态灯快闪
 * @param duration_ms 持续时间，0表示立即停止覆盖
 */
void led_manager_identify(uint32_t duration_ms);

#ifdef __cplusplus
}
#endif
#endif /* USE_DEV_LED */
#endif /* __LED_MANAGER_H__ */
