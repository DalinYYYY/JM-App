/**
 * @file        assert_report.h
 * @brief       断言上报接口：assert_report 宏与断言失败回调函数声明
 * 
 * @author      yangsl (yangsl@robot.com)
 * @version     1.0
 * @date        2026-06-17
 * 
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 * 
 * 
 * @par 修改日志:
 * | 日期       | 版本 | 作者   | 修改内容   |
 * |------------|------|--------|------------|
 * | 2026-06-17     | 1.0  | yangsl | 初始创建   |
 * 
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 */

#ifndef __ASSERT_REPORT_H__
#define __ASSERT_REPORT_H__

/* Includes -----------------------------------------------------------------------*/
#include <stdint.h>

/* Exported macro -----------------------------------------------------------------*/

#define assert_report(expr) \
	((expr) ? (uint8_t)0U : user_assert((uint8_t *)__FILE__, __LINE__))

uint8_t user_assert(uint8_t *file, uint32_t location);

#endif // __ASSERT_REPORT_H__
