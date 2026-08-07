/**
  ******************************************************************************
  * @file    assert_report.c
  * @author  Dalin
  * @version V1.0
  * @date    2026-06-17
  * @brief   assert 失败上报源文件。
  ******************************************************************************
  * @par 修改日志:
  * | 日期       | 版本 | 作者   | 修改内容                                   |
  * |------------|------|--------|--------------------------------------------|
  * | 2026-06-17 | 1.0  | Dalin  | 初始创建                                   |
  *
  * @note        本文件遵循《嵌入式C代码规范V1.0》开发
  */

/* include -------------------------------------------------------------------------------------- */
#include "assert_report.h"
#include "board_select.h"

#ifndef ASSERT_REPORT_ENABLE_PRINTF
#define ASSERT_REPORT_ENABLE_PRINTF 1
#endif

#if ASSERT_REPORT_ENABLE_PRINTF
#include <stdio.h>
#endif

/* public function ------------------------------------------------------------------------------ */

/**
  * @brief  断言失败后进入死循环，供用户自定义错误处理
  * @retval 无
  */
static void assert_func(void)
{
	while (1)
	{
	}
}

/**
 * @brief  断言失败时调用的内部回调函数
 * @param  file      断言所在文件名
 * @param  location  断言所在行号
 */
uint8_t user_assert(uint8_t *file, uint32_t location)
{
#if ASSERT_REPORT_ENABLE_PRINTF
	printf("[error] Assert failure!\r\n");
	printf("[error] Location: %s %d.\r\n", file, location);
#else
	(void)file;
	(void)location;
#endif

	return 1;
}
