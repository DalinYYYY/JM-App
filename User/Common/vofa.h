
#ifndef __VOFA_H
#define __VOFA_H

#include <string.h>
#include <stdbool.h>
#include <stdint.h>

/* vofa上位机串口号 */
#if (VOFA_PORT == 0x01)
#define DEV_VOFA DRV_UART1
#elif (VOFA_PORT == 0x02)
#define DEV_VOFA DRV_UART2
#elif (VOFA_PORT == 0x03)
#define DEV_VOFA DRV_UART3
#elif (VOFA_PORT == 0x04)
#define DEV_VOFA DRV_UART4
#elif (VOFA_PORT == 0x05)
#define DEV_VOFA DRV_UART5
#elif (VOFA_PORT == 0x06)
#define DEV_VOFA DRV_UART6
#endif

#define CH_COUNT 25
#define SIZE (CH_COUNT * 4 + 4)

typedef enum
{
    DIR_CW = 0,  // 顺时针
    DIR_CCW = 1, // 逆时针
} direction_e;

typedef struct
{
    float fdata[CH_COUNT];
    unsigned char tail[4];
} justFloat_t;

void vofa_upload(void *data, uint8_t len);
void vofa_receive_data(void);
void vofa_rcv_unpack(uint8_t *data_buf, uint8_t len);

extern justFloat_t vofa_frame;

#endif /* __VOFA_H */
