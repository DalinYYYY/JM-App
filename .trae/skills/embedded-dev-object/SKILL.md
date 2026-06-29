---
name: embedded-dev-object
description: "用于User/Devices设备对象层(dev_xxx)的开发、重构与评审。涵盖配置表驱动装配、OOP设备对象(状态+方法指针)、板级隔离(board_select)、抽象接口(dev_encoder_t)、组合装配(dev_motor_t)、通信设备start/poll/report模式、CRC/滤波/标定算法。当新增/修改dev_mt6701/dev_motor/dev_half_bridge/dev_commun_uart等设备，或讨论'设备层该怎么写''怎么换编码器型号''dev对象怎么组装''板级配置怎么隔离'时使用，即使用户没说'设备对象'也应触发。"
---

# 嵌入式设备对象层规范 (dev_xxx)

## 概述

本规范约束 `User/Devices/` 目录下的设备对象层。该层位于驱动封装层（`drv_xxx`）之上，应用/算法层之下，把具体硬件芯片（MT6701/MT6835/半桥/相电流 ADC 等）封装成带状态、带方法的设备对象 `dev_xxx_t`，供上层以面向对象方式调用。

核心设计原则：

- **配置表驱动装配**：硬件资源映射（SPI 编号、CS 引脚、ADC 通道、定时器通道等）由板级 `dev_config_board.inc` 的 `const xxx_config_t xxx_list[]` 表提供，设备对象按 `id` 索引取配置，与代码解耦。
- **板级隔离**：`board_select.h` 选择板子（V1/SFOC），板级 `.h` 使能 `USE_DEV_XXX`，板级 `.inc` 填配置表；同一设备代码可在多板复用。
- **OOP 设备对象**：`dev_xxx_t` 结构体持有状态字段 + 方法指针（`update`/`set_zero`/`start`/`stop` 等），构造函数 `dev_xxx_init(pobj, id)` 装配方法。优先用共享 `static const ops` 表（见"共享 ops 表模式"），方法指针内联仅用于需按实例绑定不同实现的场景。
- **抽象接口适配**：同类设备（如编码器）用抽象接口 `dev_encoder_t`（`ctx` + 方法指针）屏蔽具体型号，控制层只依赖抽象接口。
- **组合装配**：复杂设备（如 `dev_motor_t`）组合多个子设备（编码器 + 多圈 + FOC + 半桥 + 相电流），`init` 中逐一初始化并装配。
- **领域边界**：驱动/传感器只携带自身固有参数，不塞上层领域参数（编码器只出机械角，极对数归电机层）。详见"设备抽象领域边界"一节。
- **封装**：禁止无关模块直接写另一个模块的内部状态，需要修改时新增窄接口。

本规范是 `c-oop-coding-standard` 在设备层的特化，也是 `stm32-drv-wrapper` 的上层：drv 层封装 HAL，dev 层封装芯片/功能模块。模块边界、命名、`const`、生命周期遵循 C-OOP 通用规则；本文件补充设备层特有的配置表、板级隔离、抽象接口、组合装配等模式。

## 快速使用

| 场景 | 先做什么 | 需要读取 |
|------|----------|----------|
| 新增设备驱动 | 确定 id 枚举、配置表结构、设备对象状态与方法 | 本文件 + `references/dev-patterns.md` 的"新设备骨架模板" |
| 换芯片型号（如 MT6701→MT6835） | 写新芯片 dev_xxx，再在 dev_motor 用抽象接口适配 | 本文件"抽象接口适配"一节 + `references/dev-patterns.md` |
| 新增板子 | 在 `Board/XXX/Config/` 加 `dev_config_board.h/.inc` | 本文件"板级隔离"一节 |
| 评审 dev 代码 | 按配置表、OOP 对象、板级隔离、实时性逐项检查 | `references/review-checklist.md` |
| 写通信设备 | 用 start/on_rx_idle/poll/report 四段式 | 本文件"通信设备模式"一节 |
| 组合多设备 | 在 dev_motor_init 中逐一 init 并装配抽象接口 | 本文件"组合装配"一节 |
| 小改动 | 遵守命名、`DEV_EOK`/`DEV_ERROR`、`assert_report` | 通常只需本文件 |

## 工作流程

1. **确定设备角色**：是单一芯片驱动（编码器/EEPROM）、功能模块（半桥/相电流/电源监控）、通信粘合层（commun_uart/vesc）、还是组合设备（motor）？
2. **使能设备**：在板级 `dev_config_board.h` 加 `USE_DEV_XXX`，在 `dev_config.h` 加设备常量宏（如增益、分辨率）。
3. **定义公共类型**（`dev_xxx.h`）：id 枚举（`XXX_ID_1=0`、`XXX_ID_MAX`）、配置表结构 `xxx_config_t`（含 `name[20]` + 硬件资源）、设备对象 `dev_xxx_t`（状态 + 方法指针）。`extern const xxx_config_t xxx_list[XXX_ID_MAX];`。
4. **填板级配置表**（`Board/XXX/Config/dev_config_board.inc`）：用 designated initializer 填 `xxx_list[]`。
5. **实现设备对象**（`dev_xxx.c`）：`#if defined(USE_DEV_XXX)` 包裹；构造函数 `dev_xxx_init(pobj, id)` 做 `memset` + 装配方法指针；方法实现通过 `xxx_list[pobj->id]` 取硬件资源调用 drv 层。
6. **抽象接口适配**（若同类设备有多种型号）：定义抽象接口（如 `dev_encoder_t`），写适配函数把具体芯片绑定到抽象接口。
7. **组合装配**（若是组合设备）：在 init 中逐一初始化子设备，装配抽象接口，注入外部回调。

## 文件组织

```text
User/Config/board_select.h              板子选择宏（JM_BOARD_V1 / JM_BOARD_SFOC）
Board/<board>/Config/dev_config_board.h  板级设备使能宏（USE_DEV_XXX）
Board/<board>/Config/dev_config_board.inc 板级配置表（xxx_list[] 定义）
User/Devices/dev_config.h               全局设备常量（增益/分辨率/缓冲大小）
User/Devices/dev_config.c               定义 JM_BOARD_CONFIG_DEFINE_TABLES 后 include 板级 .inc
User/Devices/dev_xxx.h                  公共 API 与类型（id 枚举/配置表/设备对象）
User/Devices/dev_xxx.c                  实现：构造函数 + 方法实现
```

设备对象与配置表的关系：

```text
dev_xxx.h  声明 extern const xxx_config_t xxx_list[XXX_ID_MAX];
dev_xxx.c  按 xxx_list[pobj->id] 取硬件资源
Board/XXX/Config/dev_config_board.inc  定义 xxx_list[]（板级硬件映射）
dev_config.c  #define JM_BOARD_CONFIG_DEFINE_TABLES + #include 板级 .inc
```

## 命名规范

文件与函数使用小写蛇形，带 `dev_` 前缀：

```text
dev_mt6701.c / dev_half_bridge.c / dev_commun_uart.c
dev_mt6701_init() / dev_half_bridge_start() / dev_commun_uart_poll()
```

类型命名（`_t` 后缀，`_e` 后缀用于枚举类型名）：

```text
xxx_id_e              设备编号枚举（MT6701_ID_1, LED_ID_1...XXX_ID_MAX）
xxx_config_t          配置表结构（name[20] + 硬件资源，const，板级填表）
dev_xxx_t             设备对象（状态字段 + 方法指针）
xxx_state_e           状态枚举（LED_ON/OFF, BRIDGE_LOW/HIGH）
xxx_polarity_e        极性枚举（LED_ACTIVE_LOW/HIGH）
xxx_rx_callback_t     回调函数类型
```

设备对象的方法指针命名用动词（不带 `dev_` 前缀，因为是对象方法）：

```text
pobj->update(pobj)              刷新数据
pobj->start(pobj)               启动设备
pobj->stop(pobj)                停止设备
pobj->set_zero_angle(pobj, deg) 设置零点
pobj->get_mechanical_angle(pobj) 取机械角度
pobj->set_3pwm(pobj, c1, c2, c3) 设置三相 PWM
pobj->set_output_enable(pobj, en) 软急停开关
```

错误码定义在 `dev_config.h`：

```c
#define DEV_EOK     0
#define DEV_ERROR   1
#define DEV_ENABLE  1
#define DEV_DISABLE 0
```

## 配置表模式（核心 pattern）

把硬件资源映射从代码中抽出到板级配置表，是设备层的核心模式。

### 配置表结构

每个设备在 `.h` 声明配置表结构，含调试用 `name[20]` + 硬件资源字段：

```c
/* dev_mt6701.h */
typedef struct {
    char name[20];
    spiDrv_t spi_num;       // SPI 外设编号（drv 层类型）
    gpioDrv_t csn;          // 片选引脚（drv 层类型）
} mt6701_config_t;

extern const mt6701_config_t mt6701_list[MT6701_ID_MAX];
```

```c
/* dev_half_bridge.h */
typedef struct {
    char name[20];
    timNumber_e tim;        // 所属定时器
    timChannel_e channel[4]; // U/V/W + ADC 触发通道
} dev_half_bridge_config_t;

extern const dev_half_bridge_config_t half_bridge_list[BRIDGE_ID_MAX];
```

### 板级配置表定义

配置表在板级 `.inc` 文件用 designated initializer 填充，按 id 枚举索引：

```c
/* Board/SFOC/Config/dev_config_board.inc */
#ifdef JM_BOARD_CONFIG_DEFINE_TABLES
#if defined(USE_DEV_MT6701)
const mt6701_config_t mt6701_list[MT6701_ID_MAX] = {
    [MT6701_ID_1] = {
        .name = "MT6701_1",
        .spi_num = {.hspi = DRV_SPI1},
        .csn = {.gpiox = DRV_GPIOA, .pin = DRV_PIN_4, .ste = DRV_PIN_LOW},
    },
};
#endif

#if defined(USE_DEV_HALF_BRIDGE)
const dev_half_bridge_config_t half_bridge_list[BRIDGE_ID_MAX] = {
    [BRIDGE_DEV1] = {
        .name = "HALF_BRIDGE_1",
        .tim = DRV_TIM1,
        .channel = {TIM_CH1, TIM_CH2, TIM_CH3, TIM_CH4},
    },
};
#endif
#endif /* JM_BOARD_CONFIG_DEFINE_TABLES */
```

### 配置表汇总入口

`dev_config.c` 定义宏后 include 板级 `.inc`，把所有配置表集中编译：

```c
/* dev_config.c */
#define JM_BOARD_CONFIG_DEFINE_TABLES
#include "dev_config.h"
#include "dev_mt6701.h"
#include "dev_half_bridge.h"
/* ... 其他设备头 ... */
#include JM_BOARD_DEV_CONFIG_INC   // 展开为 "../../Board/SFOC/Config/dev_config_board.inc"
```

### 设备对象按 id 取配置

方法实现通过 `xxx_list[pobj->id]` 取硬件资源：

```c
static int dev_half_bridge_start(struct dev_half_bridge *pobj)
{
    const dev_half_bridge_config_t *cfg = &half_bridge_list[pobj->id];
    drv_pwm_start(cfg->tim, cfg->channel[PHASE_U]);
    /* ... */
}
```

要点：

- 配置表是 `const`，进 Flash 只读，不占 RAM。
- `name[20]` 字段供调试打印识别设备实例。
- designated initializer `[MT6701_ID_1] = {...}` 让 id 与配置显式绑定。
- 换板子只改 `.inc` 文件，设备代码不动。
- 配置表用 `#if defined(USE_DEV_XXX)` 包裹，未使能的设备不编译配置表。

## OOP 设备对象模式（核心 pattern）

设备对象 = 状态字段 + 方法指针，构造函数装配方法。

### 设备对象结构

```c
/* dev_mt6701.h */
typedef struct dev_mt6701 {
    mt6701_id_e id;            // 设备编号
    uint8_t raw_buf[4];         // SPI 原始帧
    uint32_t raw;               // 14bit 原始角度
    float mechanical_angle;     // 机械角度 °
    uint8_t mg_state;           // 磁场状态
    uint8_t crc_check;          // CRC 校验结果
    uint16_t err_cnt;           // 连续坏帧计数
    float offset;               // 零点偏移
    mt6701_dir_e dir;           // 方向

    /* public 方法指针 */
    void (*update)(struct dev_mt6701 *pobj);
    bool (*set_zero_angle)(struct dev_mt6701 *pobj, float deg);
    float (*get_zero_angle)(struct dev_mt6701 *pobj);
    void (*calibrate_zero)(struct dev_mt6701 *pobj);
    void (*set_dir)(struct dev_mt6701 *pobj, mt6701_dir_e dir);
    mt6701_dir_e (*get_dir)(struct dev_mt6701 *pobj);
} dev_mt6701_t;
```

### 构造函数

```c
void dev_mt6701_init(dev_mt6701_t *pobj, mt6701_id_e dev_id)
{
    assert_report(pobj != NULL);
    memset(pobj, 0, sizeof(dev_mt6701_t));   // 清零所有状态

    pobj->id = dev_id;
    pobj->offset = 0.0F;
    pobj->dir = MT6701_DIR_CW;

    /* 装配方法指针 */
    pobj->update = dev_mt6701_handle;
    pobj->set_zero_angle = mt6701_set_zero_angle;
    pobj->get_zero_angle = mt6701_get_zero_angle;
    pobj->calibrate_zero = mt6701_calibrate_zero;
    pobj->set_dir = mt6701_set_dir;
    pobj->get_dir = mt6701_get_dir;
}
```

要点：

- 调用者提供对象存储 `pobj`（栈/静态分配，不用 malloc）。
- `memset` 清零保证无未初始化字段。
- 构造函数只装配接口与参数，**不启动硬件**（硬件启动在 `start` 方法中）。
- 方法实现是 `static` 函数，只通过方法指针对外暴露。
- `assert_report(pobj != NULL)` 做参数校验。
- **新设备建议返回状态码**（`int dev_xxx_init(...)` 返回 `DEV_EOK`/`DEV_ERROR`）：C 无异常，半初始化对象一旦被当可用对象用可能驱动硬件误动作；返回状态码强制调用者使用前先检查。既有 `void` 返回的设备保持兼容，新设备优先返回状态码。

### 调用方式

```c
dev_mt6701_t mt6701;
dev_mt6701_init(&mt6701, MT6701_ID_1);   // 构造
mt6701.update(&mt6701);                   // 调方法
float angle = mt6701.get_zero_angle(&mt6701);
mt6701.set_zero_angle(&mt6701, 0.0F);
```

### 共享 ops 表模式（推荐）

上面的 `dev_mt6701_t` 把方法指针逐个内联进对象，每个实例各存 N 个指针——这是项目既有风格，简单直观但每实例多占 RAM。按 `c-oop-coding-standard` 要求，**当同一类型所有实例共享相同方法时，应改用共享 `static const ops` 表**：对象只持有一个 `const xxx_ops_t *ops` 指针指向那张表，表本身 `static const` 进 Flash 只读。

```c
/* dev_xxx.h —— 方法表类型 */
typedef struct {
    void  (*update)(struct dev_xxx *pobj);
    float (*get_value)(struct dev_xxx *pobj);
    void  (*set_zero)(struct dev_xxx *pobj, float zero);
    int   (*start)(struct dev_xxx *pobj);
    int   (*stop)(struct dev_xxx *pobj);
} xxx_ops_t;

/* dev_xxx.h —— 设备对象只持有一个 ops 指针 */
typedef struct dev_xxx {
    xxx_id_e id;
    xxx_state_e state;
    uint32_t raw;
    float value;
    float offset;
    const xxx_ops_t *ops;   // 指向共享方法表
} dev_xxx_t;

/* dev_xxx.c —— 共享方法表（static const，进 Flash） */
static const xxx_ops_t s_xxx_ops = {
    .update   = dev_xxx_update_impl,
    .get_value= dev_xxx_get_value_impl,
    .set_zero = dev_xxx_set_zero_impl,
    .start    = dev_xxx_start_impl,
    .stop     = dev_xxx_stop_impl,
};

/* dev_xxx.c —— 构造函数只挂 ops 指针 */
void dev_xxx_init(dev_xxx_t *pobj, xxx_id_e id)
{
    memset(pobj, 0, sizeof(*pobj));
    pobj->id = id;
    pobj->ops = &s_xxx_ops;   // 一行装配所有方法
}
```

两种模式的选择：

| 场景 | 用哪种 | 原因 |
|------|--------|------|
| 同类型所有实例共享相同方法 | **共享 ops 表**（推荐） | 每实例只花 1 个指针，表进 Flash 只读，不被意外改写 |
| 不同实例需绑定不同方法（如按宏选编码器型号） | **内联方法指针** | 灵活，构造时按条件装配不同实现 |
| 方法数很少（1~2 个）且实例极少 | 内联或 ops 均可 | 差异不大，选更简单的 |

项目现状：既有设备多用内联方法指针（因构造时可按宏条件装配不同实现，如 `dev_motor` 按编码器型号绑定不同 `update`）。**新增设备若所有实例共享相同方法，优先用共享 ops 表**；重构既有设备时，若内联方法指针未按实例差异化绑定，可逐步迁移到共享 ops 表省 RAM。

## 板级隔离模式

通过 `board_select.h` + 板级 `.h` + 板级 `.inc` 实现多板支持。

```c
/* User/Config/board_select.h */
#if defined(JM_BOARD_V1) && defined(JM_BOARD_SFOC)
#error "Define only one board macro."
#endif

#if defined(JM_BOARD_V1)
#define JM_BOARD_NAME "V1"
#include "../../Board/V1/Config/dev_config_board.h"
#define JM_BOARD_DEV_CONFIG_INC "../../Board/V1/Config/dev_config_board.inc"
#elif defined(JM_BOARD_SFOC)
#define JM_BOARD_NAME "SFOC"
#include "../../Board/SFOC/Config/dev_config_board.h"
#define JM_BOARD_DEV_CONFIG_INC "../../Board/SFOC/Config/dev_config_board.inc"
#else
#error "Define JM_BOARD_V1 or JM_BOARD_SFOC in the target options."
#endif
```

```c
/* Board/SFOC/Config/dev_config_board.h —— 板级设备使能 */
#define USE_DEV_HALF_BRIDGE
#define USE_DEV_MT6701
#define USE_DEV_POWER_MONITOR
#define USE_DEV_PHASE_CURRENT
#define USE_DEV_COMMUN_UART
```

板级 `.h` 决定哪些设备编译（`USE_DEV_XXX`），板级 `.inc` 决定设备连在哪些外设上（`xxx_list[]`）。换板子只改这两文件，设备代码不动。

## 抽象接口适配模式

同类设备（如编码器）有多种型号，用抽象接口屏蔽型号差异，控制层只依赖抽象接口。

### 抽象接口定义

```c
/* dev_motor.h —— 抽象编码器接口（与具体芯片型号无关） */
typedef struct dev_encoder {
    void *ctx;                // 指向具体编码器对象（dev_mt6701_t* / dev_mt6835_t*）
    float mechanical_angle;   // 最新机械角度，update 后刷新
    void (*update)(struct dev_encoder *pobj);
    float (*get_mechanical_angle)(struct dev_encoder *pobj);
} dev_encoder_t;
```

### 适配函数

为每种具体芯片写适配函数，把具体芯片的方法适配到抽象接口：

```c
/* dev_motor.c —— MT6701 适配 */
static void encoder_mt6701_update(struct dev_encoder *enc)
{
    dev_mt6701_t *chip = (dev_mt6701_t *)enc->ctx;
    chip->update(chip);                              // 调具体芯片方法
    enc->mechanical_angle = chip->mechanical_angle;  // 同步到抽象接口
}

/* dev_motor.c —— MT6835 适配 */
static void encoder_mt6835_update(struct dev_encoder *enc)
{
    dev_mt6835_t *chip = (dev_mt6835_t *)enc->ctx;
    chip->update(chip);
    enc->mechanical_angle = chip->get_mechanical_angle(chip);
}
```

### 编译期选型号

用宏选择编译哪套适配函数，切换型号只改宏：

```c
#define DEV_MOTOR_ENCODER_MT6701  1
#define DEV_MOTOR_ENCODER_MT6835  2
#define DEV_MOTOR_ENCODER_TYPE     DEV_MOTOR_ENCODER_MT6701

#if (DEV_MOTOR_ENCODER_TYPE == DEV_MOTOR_ENCODER_MT6701)
    /* 编译 MT6701 适配函数 + 装配 */
#elif (DEV_MOTOR_ENCODER_TYPE == DEV_MOTOR_ENCODER_MT6835)
    /* 编译 MT6835 适配函数 + 装配 */
#else
#error "未知的 DEV_MOTOR_ENCODER_TYPE"
#endif
```

## 设备抽象领域边界

驱动/传感器只携带自身固有的参数，不要塞上层领域的参数。一个模块的 cfg/state 应只包含"它这个角色本身需要的东西"；把上层（电机、控制、应用）领域的参数下放进底层驱动，会让驱动越界承担别的职责、丧失复用性，还常导致同一参数在两处各存一份、改一处忘另一处。

典型场景：

| 角色 | 应该输出/持有 | 不该持有（属上层） |
|------|--------------|-------------------|
| 磁编码器 | 机械角度（°/rad） | 极对数、电角度、齿数比 |
| 相电流采样 | 三相电流（A）、ADC 原始值 | PID 参数、电流环目标 |
| 半桥驱动 | PWM 比较值、ARR | 电流环输出（换算归控制层） |
| 电源监控 | 母线电压/电流/温度 | 过压阈值（保护策略归上层） |

正确做法：编码器只出机械角，`电角度 = 机械角 × 极对数` 的换算放到本就持有极对数的电机/控制层（`motion_param_t`）。

```c
/* 正确：dev_mt6701_t 只持有编码器固有参数 */
typedef struct dev_mt6701 {
    float mechanical_angle;   // 机械角度（编码器固有输出）
    float offset;             // 零点偏移（编码器固有）
    mt6701_dir_e dir;         // 方向（编码器固有）
    /* 没有 pole_pairs、没有 electrical_angle */
} dev_mt6701_t;

/* 正确：极对数在电机层 */
typedef struct dev_motor {
    uint8_t poles;            // 极对数（电机参数）
    dev_encoder_t encoder;    // 编码器只出机械角
    motion_param_t motor_param;  // 这里做 机械角×极对数→电角度 的换算
} dev_motor_t;
```

这条与"算法层不要持有 HAL 句柄"是一对镜像：前者防止底层细节往上层渗，这条防止上层概念往底层沉，两边都让每个模块守住自己的领域边界。

## 组合装配模式

复杂设备组合多个子设备，在构造函数中逐一初始化并装配。

```c
/* dev_motor.h —— 电机组合了编码器 + 多圈 + FOC + 半桥 + 相电流 */
typedef struct dev_motor {
    motor_id_e id;
    uint8_t poles;                       // 极对数

    focCurrent_t (*current_callback)(void);    // 外部回调
    float (*ele_radian_callback)(void);

    dev_encoder_t encoder;               // 抽象编码器
    dev_mt6701_t mt6701;                 // 具体芯片实体
    dev_mt6835_t mt6835;                 // 具体芯片实体
    motion_param_t motor_param;          // 角度/速度转化
    multiturn_t multiturn;               // 绝对多圈计数
    foc_t foc;                           // FOC
    dev_half_bridge_t half_bridge;       // 半桥
    dev_phase_current_t phase_current;   // 相电流
} dev_motor_t;
```

```c
/* dev_motor.c —— 组合装配 */
void dev_motor_init(dev_motor_t *pobj, motor_id_e id,
                    focCurrent_t (*current_callback)(void),
                    float (*ele_radian_callback)(void))
{
    memset(pobj, 0, sizeof(dev_motor_t));
    pobj->id = id;
    pobj->poles = dev_motor_get_poles(id);

    /* 1. 初始化编码器并装配抽象接口 */
#if (DEV_MOTOR_ENCODER_TYPE == DEV_MOTOR_ENCODER_MT6701)
    dev_mt6701_init(&pobj->mt6701, (mt6701_id_e)id);
    pobj->mt6701.set_zero_angle(&pobj->mt6701, 0.0F);
    pobj->encoder.ctx = &pobj->mt6701;
    pobj->encoder.update = encoder_mt6701_update;
    pobj->encoder.get_mechanical_angle = encoder_mt6701_get_mechanical_angle;
#endif

    /* 2. 初始化角度转化器 */
    motion_param_init(&pobj->motor_param, pobj->poles, 10, NULL);

    /* 3. 初始化多圈计数 */
    multiturn_init(&pobj->multiturn, &mt_cfg);

    /* 4. 初始化半桥 */
    dev_half_bridge_init(&pobj->half_bridge, (half_bridge_id_e)id);

    /* 5. 初始化相电流采样 */
    dev_phase_current_init(&pobj->phase_current, PHASE_CURRENT_GAIN, PHASE_CURRENT_SHUNT);
    pobj->phase_current.set_offset(&pobj->phase_current, (dev_current_i3axis_t){...});

    /* 6. 初始化 FOC，注入外部回调 */
    pobj->current_callback = current_callback;
    pobj->ele_radian_callback = ele_radian_callback;
    foc_init(&pobj->foc, pobj->current_callback, pobj->ele_radian_callback);
}
```

要点：

- 子设备对象作为字段嵌入组合设备，不用指针。
- 组合设备的 init 逐一调子设备 init。
- 抽象接口装配：把具体子设备绑定到抽象接口的 `ctx` + 方法指针。
- 外部回调（如电流采样、电角度）由组合设备注入到需要它的子模块（FOC）。

## 通信设备模式（start / on_rx_idle / poll / report）

通信粘合层设备（dev_commun_uart / dev_commun_vesc）统一用四段式 API：

```c
typedef struct dev_commun_uart {
    jm_proto_uart_t jm;          // 协议栈实例（私有，须为首成员）
    usartNumber_e uart;          // 串口编号
    uint8_t started;             // 接收已启动标志
    uint8_t rx_tmp[BUFFER_SIZE]; // 空闲取数临时缓冲
    int last_error;              // 最后错误码
    uint32_t poll_count;         // 调试用计数

    const jm_proto_ops_t *ops;   // 业务回调（应用注入）

    void (*set_ops)(struct dev_commun_uart *pobj, const jm_proto_ops_t *ops);
    int  (*start)(struct dev_commun_uart *pobj);       // 启动 DMA+空闲中断
    void (*on_rx_idle)(struct dev_commun_uart *pobj);  // 串口 ISR 中调用
    void (*poll)(struct dev_commun_uart *pobj);        // 主循环调用
    void (*report)(struct dev_commun_uart *pobj, uint8_t cmd, const uint8_t *body, uint16_t len);
} dev_commun_uart_t;
```

| 方法 | 调用上下文 | 职责 |
|------|-----------|------|
| `set_ops` | 初始化后 | 注入业务回调（电机控制/参数读写数据源） |
| `start` | 初始化后 | 启动空闲中断 + DMA 接收 |
| `on_rx_idle` | 串口 ISR | 检测 IDLE 标志，落本帧数据 |
| `poll` | 主循环/线程 | 取数据喂协议栈，自动回复 |
| `report` | 任意 | 主动上报一帧，无需上位机轮询 |

协议栈实例作为设备对象的首成员，方便上下转型。通信设备是"粘合层"：上接协议栈，下接 drv 层。

## 实时与安全模式

### volatile 与中断安全

ISR 与主循环/控制环共享的标志位、采样缓冲、状态变量必须加 `volatile`，否则编译器可能缓存旧值导致主循环读不到 ISR 写入的更新。但 `volatile` 不保证原子性，多字节共享量需配合关中断或原子访问。

```c
typedef struct dev_commun_uart {
    volatile uint8_t started;       // ISR 与主循环共享，须 volatile
    volatile uint32_t idle_irq_count;
    /* ... */
} dev_commun_uart_t;
```

### 热路径状态缓存

热路径避免 HAL 读取，在 `start` 时缓存：

```c
static int dev_half_bridge_start(struct dev_half_bridge *pobj)
{
    /* ... 启动 PWM ... */
    drv_tim_get_autoreload(cfg->tim, &pobj->autoreload);  // 缓存 ARR
}

static int dev_half_bridge_set_3pwm(struct dev_half_bridge *pobj, ...)
{
    uint16_t arr = pobj->autoreload;   // 用缓存的 ARR，免热路径 HAL 读
    ccr1 = (ccr1 > arr) ? arr : ccr1;  // 限幅
    /* ... */
}
```

### 软急停开关

`output_enable` 标志解耦软件故障与硬件 PWM：

```c
static int dev_half_bridge_set_3pwm(struct dev_half_bridge *pobj, ...)
{
    if (!pobj->output_enable) {
        ccr1 = ccr2 = ccr3 = 0;   // 软急停：强制三相 0 占空比
    }
    /* ... */
}
```

### 坏帧保持上次有效值

CRC 校验失败的帧丢弃，保持上一帧有效角度，避免单帧误码污染：

```c
static void dev_mt6701_get_machAngle(struct dev_mt6701 *pobj)
{
    if (pobj->crc_check != 0) {
        if (pobj->err_cnt < 0xFFFFu) pobj->err_cnt++;
        return;   // 坏帧：不更新 mechanical_angle
    }
    pobj->err_cnt = 0;
    /* ... 算角度 ... */
}
```

### 滤波状态对象化

一阶低通滤波的 `prev_current` 存在对象里，保证多实例可重入：

```c
typedef struct dev_adc_injected {
    /* ... */
    dev_current_f3axis_t current;       // 滤波后电流
    dev_current_f3axis_t prev_current;  // 上一拍（对象内状态）
} dev_phase_current_t;

#define _lpfilter(alpha, cur, prev) ((alpha)*(cur) + (1.0f-(alpha))*(prev))
```

### 自动校准

多次采样取均值标定零位：

```c
void calibrate_offset(struct dev_adc_injected *pobj, uint16_t samples)
{
    int32_t sum_a = 0, sum_b = 0, sum_c = 0;
    for (uint16_t i = 0; i < samples; i++) {
        update(pobj);
        sum_a += pobj->adc.a;
        /* ... */
    }
    pobj->offset.a = sum_a / samples;
    /* ... */
}
```

## 单例 extern 模式

全局唯一的设备用 `extern` 声明单例，免去调用方持有对象：

```c
/* dev_power_monitor.h */
extern dev_power_monitor_t dev_power_monitor;

/* dev_commun_uart.h */
extern dev_commun_uart_t dev_commun_uart;
```

单例仍需调 `init` 构造，但调用方直接用 `dev_power_monitor.update(&dev_power_monitor)`。

## 常见反模式

| 反模式 | 修正 |
|--------|------|
| 硬件引脚/通道硬编码在 `.c` 里（`drv_gpio_write(GPIOA, PIN_4, ...)`） | 抽到配置表 `xxx_list[id].csn`，按 id 取 |
| 设备用 `switch(id){case XXX_ID_1: ...}` 分支选资源 | 用配置表 + designated initializer |
| 设备对象用 `malloc` 分配 | 调用者提供存储，`dev_xxx_init(pobj, id)` 构造 |
| 构造函数里启动硬件（调 `HAL_xxx_Start`） | 构造只装配方法，硬件启动放 `start` 方法 |
| 构造函数返回 `void`，半初始化对象被当可用对象用 | 新设备 init 返回状态码，调用者使用前先检查 |
| 控制层直接调 `mt6701.update()`，换芯片要改控制层 | 用抽象接口 `dev_encoder_t`，控制层只依赖接口 |
| 换板子要改设备代码 | 板级差异抽到 `Board/XXX/Config/dev_config_board.inc` |
| 热路径每次读 HAL 取 ARR | `start` 时缓存到 `pobj->autoreload` |
| 滤波状态用 `static` 全局变量 | 存对象里 `pobj->prev_current`，保证多实例可重入 |
| CRC 坏帧用 0 或上次 raw 算角度 | 坏帧直接丢弃，保持上一帧有效角度 |
| 通信设备在 ISR 里做协议解析 | ISR 只 `on_rx_idle` 落数据，`poll` 在主循环解析 |
| 协议栈实例与设备对象分离 | 协议栈作为设备对象首成员，方便转型 |
| 软急停靠关 PWM 硬件 | 用 `output_enable` 软标志，强制 0 占空比，保留 PWM 时序 |
| 设备方法不检查 `pobj != NULL` | 入口 `assert_report(pobj != NULL)` |
| 方法指针逐个内联进对象、每实例各存一份（所有实例共享相同方法时） | 收进一张共享 `static const ops` 表，对象只留 `const ops *`；省 RAM，表进 Flash 只读 |
| 驱动/传感器持有上层领域参数（如编码器存 `pole_pairs`、直接输出电角度） | 驱动只出自身固有量（机械角）；上层换算（×极对数→电角度）放到拥有该参数的电机/控制层 |
| 外部模块直接写另一个设备的内部字段 | 新增窄接口或只读查询函数，禁止跨模块写状态 |
| ISR 与主循环共享的标志位未加 `volatile` | 共享量加 `volatile`；多字节量配合关中断 |

## 参考资料

新增/重构设备时读取 `references/dev-patterns.md`。该文件包含新设备骨架、配置表填充、抽象接口适配、组合装配、通信设备四段式、CRC/滤波/标定算法等代码示例。

评审 dev 代码时读取 `references/review-checklist.md`。如果只需回答命名、配置表或对象装配的小问题，优先使用本文件，不要加载不必要的 reference。

## 与其他 skill 的关系

| skill | 层次 | 关系 |
|-------|------|------|
| `c-oop-coding-standard` | 通用 C-OOP | 本规范遵循其模块边界、命名、`const`、生命周期规则 |
| `stm32-drv-wrapper` | drv 层（封装 HAL） | 本规范的下层：dev 层调 drv 层接口，不直接碰 HAL |
| `domain-embedded` | 嵌入式通用约束 | 本规范遵循其实时性、中断安全、无动态内存约束 |
