/**
 * @file vofa.c
 * @brief 
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
/* 上传just_float格式数据 */
void vofa_upload(void *data, uint8_t len)
{
#if 0
    uint8_t *buf = data;
    memcpy(&vofa_frame, buf, len);
    drv_usart_send(DEV_VOFA, (uint8_t *)&vofa_frame.fdata[0], len, 1000);
    drv_usart_send(DEV_VOFA, (uint8_t *)&vofa_frame.tail[0], 4, 1000);
#elif 0

	uint8_t *msg = (uint8_t *)malloc(len + 4);

	memcpy(msg, data, len);
	memcpy(&msg[len], &vofa_frame.tail[0], 4);
	drv_uart_dma_send(DEV_VOFA, (uint8_t *)msg, len + 4);

	free(msg);
#endif
}

/* 协议解析 */
void vofa_rcv_unpack(uint8_t *data_buf, uint8_t len)
{
}

void vofa_receive_data(void)
{
	// static int en = 0;
	// static uint8_t rx_buffer[64];
	// uint16_t rx_len = 0;

	// if (en == 0)
	// {
	//     usart_idle_init(DEV_VOFA, 64);
	//     en = 1;
	// }
	// else
	// {
	//     usart_idle_get_data(DEV_VOFA, rx_buffer, &rx_len);
	//     if (rx_len != 0)
	//     {
	//         vofa_rcv_unpack(rx_buffer, rx_len);
	//     }
	// }
}
