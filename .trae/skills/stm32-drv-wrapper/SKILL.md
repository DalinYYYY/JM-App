---
name: stm32-drv-wrapper
description: "用于STM32 HAL驱动封装层(drv_xxx)的开发、重构与评审。涵盖句柄查找表、HAL类型屏蔽、跨系列抽象、阻塞/IT/DMA三态API、软驱动回调注入。当新增/修改drv_gpio/spi/can/uart/tim/adc等外设驱动，或讨论'如何封装HAL''drv层该怎么写''对外接口该暴露什么'时使用，即使用户没说'封装层'也应触发。"
---

# STM32 HAL 驱动封装层规范 (drv_xxx)

## 概述

本规范约束 `User/Driver/` 目录下的 STM32 HAL 驱动封装层。该层位于 CubeMX 生成代码（`gpio.h/spi.h/usart.h` 等）与应用/算法层之间，对外提供与 HAL 类型无关、跨系列统一的运行期接口。

核心设计原则：

- **对外不暴露 HAL 类型**：公共头不 `#include "xxx.h"`（HAL 头），不出现 `SPI_HandleTypeDef`、`GPIO_TypeDef*` 等 HAL 类型；HAL 句柄仅在 `.c` 内部使用。
- **初始化与运行期分离**：外设初始化由 CubeMX 的 `MX_XXX_Init` 完成，drv 层只封装运行期收发/配置/回调。
- **跨系列统一**：F4/G4/H7 等系列差异（如 CAN vs FDCAN）在 `.c` 内用 `#if defined()` 屏蔽，对外保持同一套类型与 API。
- **O(1) 句柄定位**：用 `static const` 指针查找表 + designated initializer 把外设编号枚举映射到 HAL 句柄，避免 `switch`/`if-else` 链。

本规范是 `c-oop-coding-standard` 的驱动层特化：模块边界、命名、`const`、生命周期遵循 C-OOP 通用规则；本文件补充 HAL 封装层特有的句柄表、枚举映射、跨系列抽象、三态 API 等模式。

## 快速使用

| 场景 | 先做什么 | 需要读取 |
|------|----------|----------|
| 新增外设驱动 | 确定外设编号枚举、设备描述符、是否跨系列 | 本文件 + `references/drv-patterns.md` 的"新驱动骨架模板" |
| 重构既有驱动 | 保留公共 API，把 switch 链改成查找表 | 本文件 + `references/drv-patterns.md` |
| 评审 drv 代码 | 按 HAL 屏蔽、句柄表、边界检查、跨系列、实时性逐项检查 | `references/review-checklist.md` |
| 跨系列适配 | 在 `.c` 内用 `#if defined()` 分支，对外加统一结构体 | 本文件"跨系列抽象"一节 |
| 写软驱动 | 用回调注入 GPIO 操作，不依赖 HAL | 本文件"软驱动模式"一节 |
| 小改动 | 遵守命名、`DRV_EOK`/`DRV_ERROR`、NULL 检查 | 通常只需本文件 |

## 工作流程

1. **确定外设角色**：是单一系列外设，还是需要跨 F4/G4/H7 统一？是否有阻塞/中断/DMA 多种传输方式？是否需要接收回调？
2. **使能驱动**：在 `drv_config.h` 的 Configuration Wizard 中打开对应 `USE_XXX_DRIVER` 宏。
3. **定义公共类型**（`drv_xxx.h`）：外设编号枚举（带 `DRV_XXX_INIT` 占位和 `DRV_XXX_NUMBER_MAX` 边界）、通道/模式枚举、设备描述符 `xxxDrv_t`、初始化参数 `xxxInit_t`、回调函数类型。公共头不 include HAL 头。
4. **实现句柄查找表**（`drv_xxx.c`）：`__weak` 声明 HAL 句柄 → `static const` 指针数组 → `static inline` getter 做边界检查返回 NULL。
5. **实现枚举映射**：HAL 位域宏（如 `TIM_CHANNEL_x`、`ADC_CHANNEL_x`）用查找表；简单枚举（如 GPIO 模式）用 `switch`。
6. **实现运行期 API**：阻塞/IT/DMA 三态接口统一命名 `_it`/`_dma` 后缀；返回 `DRV_EOK`/`DRV_ERROR`。
7. **跨系列差异屏蔽**：在 `.c` 内用 `#if defined(STM32F4)` / `#elif defined(STM32G4) || defined(STM32H7)` 分支，对外用统一结构体（如 `drvCanMsg_t`）屏蔽报文头差异。
8. **回调注册**：中断接收类外设提供 `drv_xxx_register_rx_callback`，内部维护 `static` 回调表，在弱覆盖的 HAL 中断回调中转发。

## 文件组织

```text
drv_config.h        全局配置：DRV_EOK/DRV_ERROR、USE_XXX_DRIVER 使能宏、MCU 型号选择
drv_xxx.h           公共 API 与类型（不暴露 HAL 类型）
drv_xxx.c           实现：__weak 句柄、查找表、HAL 调用、跨系列分支
drv_xxx_soft.h      软件模拟外设（回调注入，不依赖 HAL）
drv_xxx_soft.c      软驱动实现
drv_tim_pwm.h       子功能头（复用 drv_tim 的句柄表，实现并入 drv_tim.c）
```

子功能（如 PWM 复用 TIM 的句柄表）可单独一个 `.h`，但实现并入宿主驱动的 `.c`，以复用 `static` 的 getter，避免对外暴露 HAL 类型。参见 `drv_tim_pwm.h` 与 `drv_tim.c` 的关系。

## 命名规范

文件与函数使用小写蛇形，带 `drv_` 前缀：

```text
drv_gpio.c / drv_spi.c / drv_can.c
drv_gpio_write() / drv_spi_send_dma() / drv_can_register_rx_callback()
```

类型命名（`_t` 后缀，`_e` 后缀用于枚举类型名）：

```text
xxxNumber_e        外设编号枚举（DRV_SPI1, DRV_SPI2...DRV_SPI_NUMBER_MAX）
xxxChannel_e       通道枚举（TIM_CH1...TIM_CH_MAX）
xxxDrv_t           设备描述符（含外设编号、片选引脚等）
xxxInit_t          运行期初始化参数
drvXxxMsg_t        跨系列统一报文（屏蔽 HAL 报文头差异）
xxx_rx_callback_t  接收回调函数类型
```

枚举常量使用大写蛇形带 `DRV_` 前缀（`DRV_PIN_HIGH`、`DRV_CAN1`），与宏风格一致；枚举的**类型名**用小写加 `_e` 后缀（`gpioPin_e`），区分"常量值"与"类型"。

错误码全局统一，定义在 `drv_config.h`：

```c
#define DRV_EOK    (0)   // 无错误
#define DRV_ERROR  (1)   // 通用错误
```

## 封装规则（核心）

### 对外不暴露 HAL 类型

公共头 `drv_xxx.h` **不** include `"xxx.h"`（HAL 头），不出现任何 HAL 类型。HAL 句柄只在 `drv_xxx.c` 内部 `#include "xxx.h"` 后使用。

```c
/* drv_adc.h —— 正确：注释说明不 include HAL 头 */
/* 注意：本头不include "adc.h"，对外接口不暴露HAL类型 */

/* drv_spi.h —— 正确：设备描述符用编号枚举，不持有 SPI_HandleTypeDef */
typedef struct DRV_SPI_ {
    spiNumber_e hspi;   // 外设编号，不是 HAL 句柄
    gpioDrv_t cs;       // 片选引脚
} spiDrv_t;
```

反例（禁止）：

```c
/* drv_spi.h —— 错误：公共头暴露 HAL 类型 */
#include "spi.h"
typedef struct {
    SPI_HandleTypeDef *hspi;  // 禁止：HAL 类型泄漏到公共 API
} spiDrv_t;
```

### 初始化与运行期分离

外设初始化（时钟、GPIO 复用、波特率、NVIC）由 CubeMX 的 `MX_XXX_Init` 完成。drv 层只封装运行期接口：收发、重配置、启停、回调注册。

```c
/* drv_gpio.h —— 上电初始化由 CubeMX 完成，此接口仅用于运行期动态重配置 */
void drv_gpio_init(gpioDrv_t drv, gpioInit_t init);
```

例外：CAN 的过滤器配置、启动、中断使能涉及运行期语义，放在 `drv_can_init` 中。

## 句柄查找表模式（核心 pattern）

把外设编号枚举映射到 HAL 句柄的标准做法：`__weak` 声明 + `static const` 指针数组 + designated initializer + `static inline` getter。

```c
/* drv_spi.c */
__weak SPI_HandleTypeDef hspi1;
__weak SPI_HandleTypeDef hspi2;
__weak SPI_HandleTypeDef hspi3;

/* 以 spiNumber_e 为索引，O(1) 定位 HAL 句柄 */
static SPI_HandleTypeDef *const s_spi_map[DRV_SPI_NUMBER_MAX] = {
    [DRV_SPI1] = &hspi1,
    [DRV_SPI2] = &hspi2,
    [DRV_SPI3] = &hspi3,
};

static inline SPI_HandleTypeDef *get_spi_handle(spiNumber_e spi)
{
    if (spi >= DRV_SPI_NUMBER_MAX)
        return NULL;
    return s_spi_map[spi];
}
```

要点：

- `__weak` 声明让 drv 层可独立编译；实际定义由 CubeMX 生成的 `spi.c` 提供，链接时覆盖弱符号。
- `static const` 指针数组进 Flash 只读，不占 RAM。
- designated initializer `[DRV_SPI1] = &hspi1` 让枚举值与句柄显式绑定，新增外设时只需加一行。
- getter 做边界检查返回 NULL，调用方统一处理非法编号。
- 未编译进的外设对应项为 NULL（如该封装无 GPIOF，用 `#ifdef GPIOF` 条件编译）。

通道/速率等 HAL 位域宏（非连续值）同样用查找表：

```c
/* TIM_CHANNEL_x 是位域宏（1,2,4,8），非连续序号，必须用查找表 */
static const uint32_t s_tim_channel_map[TIM_CH_MAX] = {
    [TIM_CH1] = TIM_CHANNEL_1,
    [TIM_CH2] = TIM_CHANNEL_2,
    [TIM_CH3] = TIM_CHANNEL_3,
    [TIM_CH4] = TIM_CHANNEL_4,
    [TIM_CH_ALL] = TIM_CHANNEL_ALL,
};
```

简单枚举（连续值）可用 `switch` 映射，如 `get_gpio_mode()`。

## 跨系列抽象

跨 F4/G4/H7 的外设（如 CAN/FDCAN）在 `.c` 内用 `#if defined()` 分支屏蔽差异，对外用统一结构体。

```c
/* drv_can.c —— 对外用 drvCanMsg_t 统一报文，差异在 .c 内屏蔽 */
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

int drv_can_send(canNumber_e can, drvCanMsg_t *msg)
{
    void *h = get_can_handle(can);
    if (h == NULL || msg == NULL)
        return DRV_ERROR;

#if defined(STM32F4)
    /* F4: 经典 CAN，len 上限 8 */
    if (msg->len > 8) return DRV_ERROR;
    CAN_TxHeaderTypeDef header = {0};
    header.StdId = msg->id;
    header.DLC = msg->len;
    HAL_CAN_AddTxMessage((CAN_HandleTypeDef *)h, &header, msg->data, &mailbox);
#elif defined(STM32G4) || defined(STM32H7)
    /* G4/H7: FDCAN，len 上限 64，需把字节数转 DLC 宏 */
    if (msg->len > 64) return DRV_ERROR;
    FDCAN_TxHeaderTypeDef header = {0};
    header.Identifier = msg->id;
    header.DataLength = fdcan_len_to_dlc(msg->len);
    HAL_FDCAN_AddMessageToTxFifoQ((FDCAN_HandleTypeDef *)h, &header, msg->data);
#endif
    return DRV_EOK;
}
```

统一结构体 `drvCanMsg_t` 屏蔽 `CAN_RxHeaderTypeDef` 与 `FDCAN_RxHeaderTypeDef` 的字段差异。HAL 中断回调也需按系列弱覆盖：

```c
#if defined(STM32F4)
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) { ... }
#elif defined(STM32G4) || defined(STM32H7)
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t ITs) { ... }
#endif
```

MCU 型号由 `drv_config.h` 的 `FLASH_MCU` 配置项经 Configuration Wizard 选择，展开为 `STM32F4`/`STM32G4` 等宏。

## API 分层：阻塞 / IT / DMA

同一外设的收发接口按传输方式分三态，命名统一后缀：

| 后缀 | 语义 | 阻塞 | 参数 |
|------|------|------|------|
| 无后缀 | 阻塞传输 | 是 | `(drv, data, len, timeout)` |
| `_it` | 中断传输 | 否 | `(drv, data, len)` |
| `_dma` | DMA 传输 | 否 | `(drv, data, len)` |

```c
int drv_spi_send(spiDrv_t drv, uint8_t *data, uint16_t len, uint32_t timeout);
int drv_spi_send_it(spiDrv_t drv, uint8_t *data, uint16_t len);
int drv_spi_send_dma(spiDrv_t drv, uint8_t *data, uint16_t len);
```

非阻塞传输的片选/资源管理约定：

- 启动前拉低 CS；启动失败立即释放 CS。
- 成功时 CS 由用户在传输完成回调中释放（drv 层无法感知中断/DMA 完成时机）。

```c
int drv_spi_send_dma(spiDrv_t drv, uint8_t *data, uint16_t len)
{
    hspi = get_spi_handle(drv.hspi);
    if (hspi == NULL) return DRV_ERROR;

    drv_spi_cs_select(drv);                 // 启动前拉低 CS
    if (HAL_SPI_Transmit_DMA(hspi, data, len) != HAL_OK) {
        drv_spi_cs_release(drv);            // 启动失败立即释放
        return DRV_ERROR;
    }
    return DRV_EOK;                         // 成功时 CS 由用户在回调中释放
}
```

## 回调注册模式

中断接收类外设（CAN、UART 空闲中断）提供回调注册接口，内部维护 `static` 回调表，在弱覆盖的 HAL 中断回调中转发。

```c
/* drv_can.c */
static can_rx_callback_t user_can_rx_callback[DRV_CAN_NUMBER_MAX] = {NULL};

int drv_can_register_rx_callback(canNumber_e can, can_rx_callback_t callback)
{
    if (can < DRV_CAN1 || can >= DRV_CAN_NUMBER_MAX)
        return DRV_ERROR;
    user_can_rx_callback[can] = callback;
    return DRV_EOK;
}

/* 弱覆盖 HAL 中断回调，转发给用户回调 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t ITs)
{
    if ((ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0) return;
    canNumber_e can_id = (hfdcan->Instance == FDCAN1) ? DRV_CAN1 : DRV_CAN2;

    drvCanMsg_t msg = {0};
    if (drv_can_recv(can_id, &msg) != DRV_EOK) return;
    if (user_can_rx_callback[can_id])
        user_can_rx_callback[can_id](can_id, &msg);
}
```

## 软驱动模式（drv_xxx_soft）

软件模拟外设（I2C、SPI）通过回调注入 GPIO 操作，与具体 MCU/HAL 完全解耦。构造函数注入回调，方法指针可挂入对象。

```c
/* drv_i2c_soft.h —— 设备描述符含回调指针和方法指针 */
typedef struct i2c_soft_drv {
    i2c_id_e id;
    uint8_t slave_address;
    uint8_t (*sda_read)(i2c_id_e id);     // 回调：读 SDA
    void (*sda_dir)(i2c_id_e id, i2c_sda_dir_e dir);  // 回调：设 SDA 方向
    void (*sda)(i2c_id_e id, i2c_state_e level);      // 回调：设 SDA 电平
    void (*scl)(i2c_id_e id, i2c_state_e level);      // 回调：设 SCL 电平
    int (*write_nbytes)(struct i2c_soft_drv *pobj, uint8_t reg, uint8_t *data, uint8_t len);
    int (*read_nbytes)(struct i2c_soft_drv *pobj, uint8_t reg, uint8_t *data, uint8_t len);
} i2c_soft_drv_t;

/* 构造函数：注入回调，挂载方法 */
void drv_i2c_init(i2c_soft_drv_t *pobj, i2c_id_e id,
                  uint8_t (*sda_read)(i2c_id_e id),
                  void (*sda_dir)(i2c_id_e id, i2c_sda_dir_e dir),
                  void (*sda)(i2c_id_e id, i2c_state_e level),
                  void (*scl)(i2c_id_e id, i2c_state_e level),
                  uint8_t hw_addr)
{
    assert_report(pobj != NULL);
    memset(pobj, 0, sizeof(i2c_soft_drv_t));
    pobj->id = id;
    pobj->sda_read = sda_read;
    pobj->sda_dir = sda_dir;
    pobj->sda = sda;
    pobj->scl = scl;
    pobj->slave_address = hw_addr;
    pobj->read_nbytes = drv_i2c_read_nbytes;   // 挂载方法
    pobj->write_nbytes = drv_i2c_write_nbytes;
}
```

时序用 `volatile` 计数延时防止被 `-O` 优化删除：

```c
static void _i2c_delay(void)
{
    volatile uint8_t i;
    for (i = 0; i < 10; i++) ;
}
```

## 实时约束

- 热路径（PWM 占空比、ADC 读取）直接操作寄存器宏（`__HAL_TIM_SET_COMPARE`），不触发 HAL 重配置。
- `static inline` getter 消除调用开销，供高频路径直接调用。
- DWT 周期计数器用裸寄存器地址，`static inline` 读取，不依赖 HAL/具体型号（Cortex-M0/M0+ 无 DWT，不适用）。
- ISR 与主循环共享的标志位（如 `idleData_t.flag`）必须考虑中断安全；`volatile` 不保证原子性，多字节共享量需配合关中断。

## 跨系列运行期自适应

FLASH 几何（容量/单双 Bank/页大小）不按型号宏硬编码，由 HAL 运行期值自适应，兼容各容量后缀（CB/CC/CE）：

```c
/* drv_flash_g4.h —— 几何参数全部来自 HAL 运行期值 */
/*   FLASH_SIZE      - 由芯片 FLASHSIZE 寄存器读出的实际容量 */
/*   FLASH_BANK_SIZE - 单 Bank=FLASH_SIZE，双 Bank=FLASH_SIZE/2 */
/*   FLASH_PAGE_SIZE - 双 Bank 2KB/页，单 Bank 4KB/页 */
u8 drv_g4_flash_is_dualbank(void);   // 运行期读 DBANK 选项位
u32 drv_g4_flash_total_size(void);   // 来自 FLASHSIZE 寄存器
```

## 常见反模式

| 反模式 | 修正 |
|--------|------|
| 公共头 include HAL 头，暴露 `SPI_HandleTypeDef*` | 公共头只用编号枚举，HAL 头在 `.c` 内 include |
| 用 `switch(spi){case DRV_SPI1: return &hspi1; ...}` 定位句柄 | 改用 `static const` 指针查找表 + designated initializer |
| 把 HAL 位域宏（`TIM_CHANNEL_x`）当连续值直接 `(1<<channel)` | 用查找表映射，因为 HAL 宏是非连续位域 |
| drv 层重复做 CubeMX 已完成的外设初始化 | drv 层只封装运行期接口，初始化交给 `MX_XXX_Init` |
| 跨系列差异泄漏到公共 API（如暴露 `CAN_RxHeaderTypeDef`） | 用统一结构体 `drvCanMsg_t` 屏蔽，差异在 `.c` 内 `#if defined()` |
| 非阻塞传输启动失败未释放 CS | 启动失败立即 `drv_spi_cs_release`；成功时由用户在回调释放 |
| 软驱动直接调用 `HAL_GPIO_WritePin` | 通过回调注入 GPIO 操作，保持与 HAL 解耦 |
| FLASH 几何按型号宏硬编码（`#ifdef STM32G474` 写死页大小） | 运行期读 HAL 值/寄存器自适应 |
| 回调直接在 HAL 中断里做业务逻辑 | HAL 中断回调只转发给用户注册的回调，业务在回调中处理 |
| 方法指针逐个内联进软驱动对象、每实例存一份 | 收进共享方法表或构造时统一挂载，对象只存状态 |

## 参考资料

新增/重构驱动时读取 `references/drv-patterns.md`。该文件包含新驱动骨架模板、DMA+空闲中断环形缓冲、软驱动完整实现等代码示例。

评审 drv 代码时读取 `references/review-checklist.md`。如果只需回答命名、错误码或句柄表的小问题，优先使用本文件，不要加载不必要的 reference。
