# drv 驱动封装层代码模板

> 本文件是 `SKILL.md` 的参考资料，提供新驱动骨架、跨系列适配、DMA+空闲中断环形缓冲、软驱动完整实现等代码模板。所有模板从 `User/Driver/` 既有驱动提炼，命名与风格与项目一致。

## 1. 新驱动骨架模板

以一个假想的 `drv_xxx` 外设为例，展示完整的最小骨架。新增驱动时按此结构填充。

### 1.1 drv_config.h 使能项

在 Configuration Wizard 区块加入使能宏：

```c
// <c1>
// ENABLE DRIVER ---> XXX
#define USE_XXX_DRIVER
// </c1>
```

### 1.2 drv_xxx.h 公共头

```c
/**
 * @file        drv_xxx.h
 * @brief       XXX驱动接口，封装HAL的XXX运行期收发功能
 *
 * @author      Dalin (dalinyy@163.com)
 * @version     1.0
 * @date        2026-06-XX
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @par 修改日志:
 * | 日期       | 版本 | 作者   | 修改内容   |
 * |------------|------|--------|------------|
 * | 2026-06-XX | 1.0  | Dalin  | 初始创建   |
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 * @note        XXX初始化由CubeMX的MX_XXX_Init完成，此层仅封装运行期接口
 * @note        对外接口不暴露HAL类型，HAL句柄仅在drv_xxx.c内部使用
 */
#ifndef _DRV_XXX_H_
#define _DRV_XXX_H_

#include "drv_config.h"

#ifdef USE_XXX_DRIVER
#include <stdint.h>

/**
 * @brief XXX外设编号
 */
typedef enum
{
    DRV_XXX_INIT = 0,
    DRV_XXX1,
    DRV_XXX2,
    DRV_XXX_NUMBER_MAX
} xxxNumber_e;

/**
 * @brief XXX设备描述
 */
typedef struct DRV_XXX_
{
    xxxNumber_e hxxx;
} xxxDrv_t;

/**
 * @brief       发送数据(阻塞)
 * @param        drv               : XXX设备
 * @param        data              : 发送数据缓冲区
 * @param        len               : 数据长度
 * @param        timeout           : 超时时间 (单位: ms)
 * @return       : DRV_EOK成功，DRV_ERROR失败
 */
int drv_xxx_send(xxxDrv_t drv, uint8_t *data, uint16_t len, uint32_t timeout);

/**
 * @brief       接收数据(阻塞)
 */
int drv_xxx_recv(xxxDrv_t drv, uint8_t *data, uint16_t len, uint32_t timeout);

#endif /* USE_XXX_DRIVER */
#endif /* _DRV_XXX_H_ */
```

### 1.3 drv_xxx.c 实现

```c
/**
 * @file        drv_xxx.c
 * @brief       XXX驱动实现，封装HAL的XXX运行期收发功能
 * ...（头注释与 .h 一致）
 */
#include "drv_xxx.h"

#ifdef USE_XXX_DRIVER
#include "xxx.h"   /* HAL 头，仅在 .c 内 include */

__weak XXX_HandleTypeDef hxxx1;
__weak XXX_HandleTypeDef hxxx2;

/* 句柄查找表：以 xxxNumber_e 为索引，O(1) 定位 HAL 句柄 */
static XXX_HandleTypeDef *const s_xxx_map[DRV_XXX_NUMBER_MAX] = {
    [DRV_XXX1] = &hxxx1,
    [DRV_XXX2] = &hxxx2,
};

static inline XXX_HandleTypeDef *get_xxx_handle(xxxNumber_e xxx)
{
    if (xxx >= DRV_XXX_NUMBER_MAX)
        return NULL;
    return s_xxx_map[xxx];
}

int drv_xxx_send(xxxDrv_t drv, uint8_t *data, uint16_t len, uint32_t timeout)
{
    XXX_HandleTypeDef *hxxx = get_xxx_handle(drv.hxxx);
    if (hxxx == NULL || data == NULL)
        return DRV_ERROR;

    return (HAL_XXX_Transmit(hxxx, data, len, timeout) == HAL_OK) ? DRV_EOK : DRV_ERROR;
}

int drv_xxx_recv(xxxDrv_t drv, uint8_t *data, uint16_t len, uint32_t timeout)
{
    XXX_HandleTypeDef *hxxx = get_xxx_handle(drv.hxxx);
    if (hxxx == NULL || data == NULL)
        return DRV_ERROR;

    return (HAL_XXX_Receive(hxxx, data, len, timeout) == HAL_OK) ? DRV_EOK : DRV_ERROR;
}

#endif /* USE_XXX_DRIVER */
```

## 2. 三态收发模板（阻塞/IT/DMA）

带片选管理的外设（如 SPI）的三态收发模板：

```c
/* 阻塞：自动管理 CS */
int drv_xxx_send(xxxDrv_t drv, uint8_t *data, uint16_t len, uint32_t timeout)
{
    XXX_HandleTypeDef *h = get_xxx_handle(drv.hxxx);
    if (h == NULL) return DRV_ERROR;

    drv_xxx_cs_select(drv);
    HAL_StatusTypeDef st = HAL_XXX_Transmit(h, data, len, timeout);
    drv_xxx_cs_release(drv);

    return (st == HAL_OK) ? DRV_EOK : DRV_ERROR;
}

/* 中断：启动前拉低 CS，完成回调中释放 */
int drv_xxx_send_it(xxxDrv_t drv, uint8_t *data, uint16_t len)
{
    XXX_HandleTypeDef *h = get_xxx_handle(drv.hxxx);
    if (h == NULL) return DRV_ERROR;

    drv_xxx_cs_select(drv);
    if (HAL_XXX_Transmit_IT(h, data, len) != HAL_OK) {
        drv_xxx_cs_release(drv);   // 启动失败立即释放
        return DRV_ERROR;
    }
    return DRV_EOK;   // 成功时 CS 由用户在完成回调中释放
}

/* DMA：同 IT */
int drv_xxx_send_dma(xxxDrv_t drv, uint8_t *data, uint16_t len)
{
    XXX_HandleTypeDef *h = get_xxx_handle(drv.hxxx);
    if (h == NULL) return DRV_ERROR;

    drv_xxx_cs_select(drv);
    if (HAL_XXX_Transmit_DMA(h, data, len) != HAL_OK) {
        drv_xxx_cs_release(drv);
        return DRV_ERROR;
    }
    return DRV_EOK;
}
```

## 3. 跨系列适配模板

CAN/FDCAN 是典型跨系列外设。模板展示如何用统一结构体 + `#if defined()` 屏蔽差异。

### 3.1 统一报文结构体（公共头）

```c
/* drv_can.h */
typedef struct
{
    uint32_t id;
    uint8_t ide;       // 0标准帧，1扩展帧
    uint8_t rtr;       // 0数据帧，1远程帧
    uint8_t len;       // 经典CAN 0~8，FD 0~64
    uint8_t data[64];
} drvCanMsg_t;
```

### 3.2 系列相关实现（.c 内）

```c
/* drv_can.c */
#if defined(STM32F4)
__weak CAN_HandleTypeDef hcan1;
static CAN_HandleTypeDef *const s_can_map[DRV_CAN_NUMBER_MAX] = {
    [DRV_CAN1] = &hcan1,
};
#elif defined(STM32G4) || defined(STM32H7)
__weak FDCAN_HandleTypeDef hfdcan1;
static FDCAN_HandleTypeDef *const s_can_map[DRV_CAN_NUMBER_MAX] = {
    [DRV_CAN1] = &hfdcan1,
};
#endif

static inline void *get_can_handle(canNumber_e can)
{
    if (can < DRV_CAN1 || can >= DRV_CAN_NUMBER_MAX)
        return NULL;
    return s_can_map[can];
}

/* FD 专用：字节数转 DLC 宏（经典 CAN 0~8 直接相等） */
#if defined(STM32G4) || defined(STM32H7)
static uint32_t fdcan_len_to_dlc(uint8_t len)
{
    if (len <= 8)  return (uint32_t)len;
    if (len <= 12) return FDCAN_DLC_BYTES_12;
    if (len <= 16) return FDCAN_DLC_BYTES_16;
    if (len <= 20) return FDCAN_DLC_BYTES_20;
    if (len <= 24) return FDCAN_DLC_BYTES_24;
    if (len <= 32) return FDCAN_DLC_BYTES_32;
    if (len <= 48) return FDCAN_DLC_BYTES_48;
    return FDCAN_DLC_BYTES_64;
}
#endif

int drv_can_send(canNumber_e can, drvCanMsg_t *msg)
{
    void *h = get_can_handle(can);
    if (h == NULL || msg == NULL) return DRV_ERROR;

#if defined(STM32F4)
    if (msg->len > 8) return DRV_ERROR;
    CAN_TxHeaderTypeDef header = {0};
    header.IDE = msg->ide ? CAN_ID_EXT : CAN_ID_STD;
    header.StdId = msg->id;
    header.RTR = msg->rtr ? CAN_RTR_REMOTE : CAN_RTR_DATA;
    header.DLC = msg->len;
    uint32_t mailbox;
    if (HAL_CAN_AddTxMessage((CAN_HandleTypeDef *)h, &header, msg->data, &mailbox) != HAL_OK)
        return DRV_ERROR;
#elif defined(STM32G4) || defined(STM32H7)
    if (msg->len > 64) return DRV_ERROR;
    FDCAN_TxHeaderTypeDef header = {0};
    header.Identifier = msg->id;
    header.IdType = msg->ide ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    header.TxFrameType = msg->rtr ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
    header.DataLength = fdcan_len_to_dlc(msg->len);
    header.FDFormat = FDCAN_CLASSIC_CAN;
    if (HAL_FDCAN_AddMessageToTxFifoQ((FDCAN_HandleTypeDef *)h, &header, msg->data) != HAL_OK)
        return DRV_ERROR;
#endif
    return DRV_EOK;
}
```

### 3.3 系列相关 HAL 中断回调

```c
#if defined(STM32F4)
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    canNumber_e can_id = (hcan->Instance == CAN1) ? DRV_CAN1 : DRV_CAN2;
    drvCanMsg_t msg = {0};
    if (drv_can_recv(can_id, &msg) != DRV_EOK) return;
    if (user_can_rx_callback[can_id])
        user_can_rx_callback[can_id](can_id, &msg);
}
#elif defined(STM32G4) || defined(STM32H7)
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0) return;
    canNumber_e can_id = (hfdcan->Instance == FDCAN1) ? DRV_CAN1 : DRV_CAN2;
    drvCanMsg_t msg = {0};
    if (drv_can_recv(can_id, &msg) != DRV_EOK) return;
    if (user_can_rx_callback[can_id])
        user_can_rx_callback[can_id](can_id, &msg);
    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}
#endif
```

## 4. DMA + 空闲中断不定长接收模板

UART 的 DMA+空闲中断是较复杂的模式，涉及环形缓冲与读写指针。模板来自 `drv_usart.c`。

### 4.1 状态结构体

```c
typedef struct
{
    uint8_t  flag;          // 接收完成标志
    uint16_t len;           // 本次接收长度
    uint16_t max;           // 缓存容量
    uint16_t read_pos;      // 读指针
    uint16_t write_pos;     // 写指针(DMA 剩余计数反推)
    uint32_t init_count;    // 调试用：init 次数
    uint32_t idle_count;    // 调试用：空闲中断次数
    uint32_t get_count;     // 调试用：取数次数
    uint32_t rx_bytes;      // 调试用：累计接收字节
    int      last_error;    // 最后一次错误码
    uint8_t *p;             // 缓存指针
} idleData_t;
```

### 4.2 初始化 + 启动

```c
static idleData_t s_idle_rx[DRV_UART_NUMBER_MAX];
static uint8_t s_idle_rx_buf[DRV_UART_NUMBER_MAX][DMA_IDLE_LEN];

int usart_idle_init(usartNumber_e uart, uint16_t len)
{
    if (uart >= DRV_UART_NUMBER_MAX || len == 0u || len > DMA_IDLE_LEN)
        return DRV_ERROR;

    s_idle_rx[uart].p = s_idle_rx_buf[uart];
    s_idle_rx[uart].max = len;
    s_idle_rx[uart].read_pos = 0;
    s_idle_rx[uart].write_pos = 0;
    s_idle_rx[uart].flag = 0;
    memset(s_idle_rx[uart].p, 0, len);

    UART_HandleTypeDef *handle = get_usart_handle(uart);
    if (handle == NULL) { s_idle_rx[uart].p = NULL; return DRV_ERROR; }

    if (HAL_UART_Receive_DMA(handle, s_idle_rx[uart].p, s_idle_rx[uart].max) != HAL_OK)
        return DRV_ERROR;

    __HAL_UART_ENABLE_IT(handle, UART_IT_IDLE);   // 开空闲中断
    s_idle_rx[uart].init_count++;
    return DRV_EOK;
}
```

### 4.3 空闲中断处理（在 UART ISR 中调用）

```c
int drv_uart_idle(usartNumber_e uart)
{
    UART_HandleTypeDef *handle = get_usart_handle(uart);
    if (handle == NULL) return DRV_ERROR;

    if (__HAL_UART_GET_FLAG(handle, UART_FLAG_IDLE) != RESET)
    {
        __HAL_UART_CLEAR_IDLEFLAG(handle);
        s_idle_rx[uart].flag = 1;        // 置位完成标志
        s_idle_rx[uart].idle_count++;
    }
    return DRV_EOK;
}
```

### 4.4 取数据（环形缓冲，处理回绕）

```c
int usart_idle_get_data(usartNumber_e uart, uint8_t *data, uint16_t *len)
{
    UART_HandleTypeDef *handle = get_usart_handle(uart);
    DMA_HandleTypeDef *dma_ch = get_usart_dma_rx_ch(uart);
    if (handle == NULL || dma_ch == NULL || data == NULL || len == NULL
        || s_idle_rx[uart].p == NULL)
    {
        if (len) *len = 0;
        return DRV_ERROR;
    }

    *len = 0;
    s_idle_rx[uart].flag = 0;

    uint16_t max = s_idle_rx[uart].max;
    uint16_t read_pos = s_idle_rx[uart].read_pos;
    /* DMA 剩余计数反推写指针 */
    uint16_t write_pos = (uint16_t)(max - __HAL_DMA_GET_COUNTER(dma_ch));
    if (write_pos >= max) write_pos = 0;
    s_idle_rx[uart].write_pos = write_pos;

    uint16_t copy_len;
    if (write_pos >= read_pos)
    {
        copy_len = (uint16_t)(write_pos - read_pos);
        if (copy_len) memcpy(data, &s_idle_rx[uart].p[read_pos], copy_len);
    }
    else
    {
        /* 回绕：先拷尾部，再拷头部 */
        uint16_t first_len = (uint16_t)(max - read_pos);
        memcpy(data, &s_idle_rx[uart].p[read_pos], first_len);
        if (write_pos) memcpy(&data[first_len], s_idle_rx[uart].p, write_pos);
        copy_len = (uint16_t)(first_len + write_pos);
    }

    s_idle_rx[uart].read_pos = write_pos;   // 读指针追上写指针
    s_idle_rx[uart].len = copy_len;
    s_idle_rx[uart].rx_bytes += copy_len;
    *len = copy_len;
    return DRV_EOK;
}
```

## 5. 软驱动完整模板（回调注入）

软件 I2C 的完整实现模板，展示回调注入、构造函数、位级时序。

### 5.1 设备描述符与回调类型

```c
typedef enum { I2C_LOW = 0, I2C_HIGH } i2c_state_e;
typedef enum { NACK = 0u, ACK } i2c_ack_state_e;
typedef enum { IN = 0u, OUT, DIR_MAX } i2c_sda_dir_e;

typedef struct i2c_soft_drv
{
    i2c_id_e id;
    uint8_t  slave_address;
    /* GPIO 操作回调（由使用者注入） */
    uint8_t (*sda_read)(i2c_id_e id);
    void    (*sda_dir)(i2c_id_e id, i2c_sda_dir_e dir);
    void    (*sda)(i2c_id_e id, i2c_state_e level);
    void    (*scl)(i2c_id_e id, i2c_state_e level);
    /* 方法指针（构造时挂载） */
    int (*write_nbytes)(struct i2c_soft_drv *pobj, uint8_t reg, uint8_t *data, uint8_t len);
    int (*read_nbytes)(struct i2c_soft_drv *pobj, uint8_t reg, uint8_t *data, uint8_t len);
} i2c_soft_drv_t;
```

### 5.2 构造函数

```c
void drv_i2c_init(i2c_soft_drv_t *pobj, i2c_id_e id,
                  uint8_t (*sda_read)(i2c_id_e id),
                  void (*sda_dir)(i2c_id_e id, i2c_sda_dir_e dir),
                  void (*sda)(i2c_id_e id, i2c_state_e level),
                  void (*scl)(i2c_id_e id, i2c_state_e level),
                  uint8_t hw_addr)
{
    assert_report(pobj != NULL);
    memset(pobj, 0, sizeof(*pobj));
    pobj->id = id;
    pobj->sda_read = sda_read;
    pobj->sda_dir = sda_dir;
    pobj->sda = sda;
    pobj->scl = scl;
    pobj->slave_address = hw_addr;
    pobj->read_nbytes = drv_i2c_read_nbytes;
    pobj->write_nbytes = drv_i2c_write_nbytes;
}
```

### 5.3 位级时序（static 内部函数）

```c
static void _i2c_delay(void)
{
    volatile uint8_t i;          // volatile 防 -O 删除空循环
    for (i = 0; i < 10; i++) ;
}

static void i2c_start(i2c_soft_drv_t *i2c)
{
    i2c->sda_dir(i2c->id, OUT);
    i2c->sda(i2c->id, I2C_HIGH);
    i2c->scl(i2c->id, I2C_HIGH);
    _i2c_delay();
    i2c->sda(i2c->id, I2C_LOW);  // SCL 高时 SDA 下降沿 = 起始
    _i2c_delay();
    i2c->scl(i2c->id, I2C_LOW);
}

static void i2c_stop(i2c_soft_drv_t *i2c)
{
    i2c->sda_dir(i2c->id, OUT);
    i2c->scl(i2c->id, I2C_LOW);
    i2c->sda(i2c->id, I2C_LOW);
    _i2c_delay();
    i2c->scl(i2c->id, I2C_HIGH);
    i2c->sda(i2c->id, I2C_HIGH); // SCL 高时 SDA 上升沿 = 停止
    _i2c_delay();
}

static int i2c_wait_ack(i2c_soft_drv_t *i2c)
{
    uint8_t err = 0;
    i2c->sda_dir(i2c->id, IN);
    i2c->sda(i2c->id, I2C_HIGH);
    _i2c_delay();
    i2c->scl(i2c->id, I2C_HIGH);
    _i2c_delay();
    while (i2c->sda_read(i2c->id)) {
        if (++err > 250) { i2c_stop(i2c); return DRV_ERROR; }
    }
    i2c->scl(i2c->id, I2C_LOW);
    return DRV_EOK;
}
```

### 5.4 多字节读写

```c
int drv_i2c_write_nbytes(i2c_soft_drv_t *i2c, uint8_t reg, uint8_t *data, uint8_t len)
{
    int ret = DRV_EOK;
    i2c_start(i2c);
    i2c_send_byte(i2c, i2c->slave_address | 0);  // 写
    ret |= i2c_wait_ack(i2c);
    i2c_send_byte(i2c, reg);
    ret |= i2c_wait_ack(i2c);
    for (uint8_t i = 0; i < len; i++) {
        i2c_send_byte(i2c, data[i]);
        ret |= i2c_wait_ack(i2c);
    }
    i2c_stop(i2c);
    return ret;
}

int drv_i2c_read_nbytes(i2c_soft_drv_t *i2c, uint8_t reg, uint8_t *data, uint8_t len)
{
    if (len == 0) return DRV_ERROR;
    int ret = DRV_EOK;
    i2c_start(i2c);
    i2c_send_byte(i2c, i2c->slave_address | 0);  // 写（设寄存器）
    ret |= i2c_wait_ack(i2c);
    i2c_send_byte(i2c, reg);
    ret |= i2c_wait_ack(i2c);

    i2c_start(i2c);                               // 重复起始
    i2c_send_byte(i2c, i2c->slave_address | 1);   // 读
    ret |= i2c_wait_ack(i2c);
    for (uint8_t i = 0; i < len - 1; i++)
        data[i] = i2c_read_byte(i2c, ACK);
    data[len - 1] = i2c_read_byte(i2c, NACK);     // 最后一字节 NACK
    i2c_stop(i2c);
    return ret;
}
```

## 6. 子功能复用宿主句柄表模板

PWM 复用 TIM 的句柄表，头文件单独，实现并入 `drv_tim.c`：

```c
/* drv_tim_pwm.h —— 只声明 API，不重复声明类型 */
#include "drv_tim.h"   // 复用 timNumber_e, timChannel_e

int drv_pwm_start(timNumber_e tim, timChannel_e channel);
int drv_pwm_set_dutycycle(timNumber_e tim, timChannel_e channel, uint32_t duty);
```

```c
/* drv_tim.c 末尾 —— PWM 实现复用 static 的 get_tim_handle/get_tim_channel */
#ifdef USE_TIM_PWM_DRIVER
int drv_pwm_start(timNumber_e tim, timChannel_e channel)
{
    TIM_HandleTypeDef *h = get_tim_handle(tim);
    if (h == NULL) return DRV_ERROR;
    HAL_TIM_PWM_Start(h, get_tim_channel(channel));
    return DRV_EOK;
}

int drv_pwm_set_dutycycle(timNumber_e tim, timChannel_e channel, uint32_t duty)
{
    TIM_HandleTypeDef *h = get_tim_handle(tim);
    if (h == NULL) return DRV_ERROR;
    __HAL_TIM_SET_COMPARE(h, get_tim_channel(channel), duty);  // 热路径直写寄存器
    return DRV_EOK;
}
#endif
```

## 7. RTOS 封装模板（不透明句柄）

RTOS 封装层用 `void *` 不透明句柄对外，内部转换：

```c
/* drv_rtos.h —— 句柄类型用 void *，不暴露 FreeRTOS/CMSIS-OS 类型 */
typedef void *drv_rtos_thread_handle_t;
typedef void *drv_rtos_sem_handle_t;

#define DRV_RTOS_WAIT_FOREVER 0xFFFFFFFFU

drv_rtos_thread_handle_t drv_rtos_thread_create(const char *name,
                                                drv_rtos_thread_func_t func,
                                                drv_rtos_priority_e priority,
                                                uint32_t stack_size,
                                                void const *arg);
int drv_rtos_sem_wait(drv_rtos_sem_handle_t sem, uint32_t timeout_ms);
```

```c
/* drv_rtos.c —— 内部转换 */
static inline uint32_t to_wait(uint32_t timeout_ms)
{
    return (timeout_ms == DRV_RTOS_WAIT_FOREVER) ? osWaitForever : timeout_ms;
}

int drv_rtos_sem_wait(drv_rtos_sem_handle_t sem, uint32_t timeout_ms)
{
    if (sem == NULL) return DRV_ERROR;
    return (osSemaphoreWait((osSemaphoreId)sem, to_wait(timeout_ms)) == osOK)
           ? DRV_EOK : DRV_ERROR;
}
```

## 8. DWT 高精度计时模板

裸寄存器 + `static inline`，不依赖 HAL/型号：

```c
/* drv_dwt_timer.h */
#define DWT_LAR_UNLOCK  (uint32_t)0xC5ACCE55
#define DWT_LAR         (*(volatile uint32_t *)0xE0000FB0)
#define DWT_CR          (*(volatile uint32_t *)0xE0001000)
#define DWT_CYCCNT      (*(volatile uint32_t *)0xE0001004)
#define DEM_CR          (*(volatile uint32_t *)0xE000EDFC)
#define DEM_CR_TRCENA   (1UL << 24)
#define DWT_CR_CYCCNTENA (1UL << 0)

void drv_dwt_timer_init(void);

/* inline 消除调用开销，供高频延时直接读 */
static inline uint32_t drv_dwt_timer_get_ticks(void)
{
    return DWT_CYCCNT;
}
```

```c
/* drv_delay.c —— 基于 DWT 的 us/ms 阻塞延时 */
void drv_delay_us(uint32_t us)
{
    uint32_t ticks = drv_dwt_timer_get_ticks();
    uint32_t wait = us * (SystemCoreClock / 1000000U);
    while ((drv_dwt_timer_get_ticks() - ticks) < wait) ;
}
```
