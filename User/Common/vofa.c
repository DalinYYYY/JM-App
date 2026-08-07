/**
 * @file vofa.c
 * @brief VOFA+ 上位机通信：justFloat 数据上传与下行协议解析
 * 
 * @author dalin (dalinyy@163.com)
 * @version 1.0
 * @date 2026-06-10
 * 
 * @copyright Copyright (c) 2026 Robot Tech.co, Ltd. All rights reserved.
 * 
 * @par 修改日志:
 * <table>
 * <tr><th>日期</th><th>版本</th><th>作者</th><th>修改内容</th></tr>
 * <tr><td>2026-06-10</td><td>1.0</td><td>yangsl</td><td>初始创建</td></tr>
 * </table>
 * 
 * @note 本文件遵循《嵌入式C代码规范V1.0》开发
 */

#include "vofa.h"
#include <stdlib.h>

justFloat_t vofa_frame = {.tail[0] = 0x00, .tail[1] = 0x00, .tail[2] = 0x80, .tail[3] = 0x7f};

/* 上传 justFloat 格式数据（当前为占位实现，待接入实际串口发送） */
void vofa_upload(void *data, uint8_t len)
{
	(void)data;
	(void)len;
}

/* 下行协议解析（当前为占位实现，待实现） */
void vofa_rcv_unpack(uint8_t *data_buf, uint8_t len)
{
	(void)data_buf;
	(void)len;
}

void vofa_receive_data(void)
{
	/* 占位实现，待接入串口接收处理 */
}
