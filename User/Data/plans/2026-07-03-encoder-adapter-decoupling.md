# 编码器适配层解耦方案设计文档

> **方案名称**：方案A改进版 — 适配文件分离 + 实体外置
> **创建日期**：2026-07-03
> **状态**：设计完成，待用户确认后执行
> **目标**：将 `dev_motor` 从编码器型号耦合中彻底解耦，新增编码器只需加 1 个适配文件 + 2 行 `#elif`

---

## 1. 问题分析

当前 `dev_motor.h/c` 存在三个核心问题，违反开闭原则：

### 1.1 结构体膨胀 — 实体内嵌

[dev_motor.h](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/Devices/dev_motor.h) 的 `dev_motor_t` 内嵌了所有支持型号的芯片实体：

```c
typedef struct dev_motor {
    dev_encoder_t encoder;     // 抽象接口（好）
    dev_mt6701_t mt6701;      // ← 具体芯片实体（问题）
    dev_mt6835_t mt6835;      // ← 每加一种编码器就多一个字段
    ...
} dev_motor_t;
```

**问题**：加 AS5047 → 改结构体 → 所有引用 `dev_motor_t` 的代码重新编译；未使用的型号也占 RAM。

### 1.2 头文件依赖 — include 蔓延

`dev_motor.h` 直接 `#include` 了所有编码器芯片头：

```c
#include "dev_mt6701.h"    // ← 电机头不该认识具体编码器芯片
#include "dev_mt6835.h"
```

**问题**：任何包含 `dev_motor.h` 的文件都间接拉入芯片驱动头，形成不必要的依赖链。

### 1.3 适配函数堆积 — #if 叠加

[dev_motor.c](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/Devices/dev_motor.c) 内有 14 个 `static` 适配函数（7 个/芯片）+ `#if`/`#elif` 条件编译块：

```
encoder_mt6701_update / get_mechanical_angle / set_offset / get_offset / set_dir / get_dir / get_raw_deg  (7个)
encoder_mt6835_update / ...同上7个  (7个)
#if 装配块 ~30行
```

**问题**：新增编码器要在 `dev_motor.c` 里加 7 个函数 + 10 行装配代码，文件持续膨胀。

---

## 2. 设计目标

| 编号 | 目标 | 验收标准 |
|------|------|----------|
| G1 | `dev_motor_t` 不含任何具体芯片实体字段 | 结构体中无 `mt6701`/`mt6835` 字段 |
| G2 | `dev_motor.h` 不 `#include` 任何芯片驱动头 | 无 `dev_mt6701.h`/`dev_mt6835.h` |
| G3 | `dev_motor.c` 不含适配函数 | 无 `encoder_mt6701_*`/`encoder_mt6835_*` |
| G4 | 新增编码器不改 `dev_motor.h/c` | 仅加适配文件 + `dev_motor.c` 加 2 行 `#elif` |
| G5 | 抽象接口 `dev_encoder_t` 单一定义 | 消除 `.h`/`_virtual.h` 双份定义 |
| G6 | 编译通过 + 运行时角度/标定功能不变 | Keil 编译 0 error 0 warning |

---

## 3. 架构设计

### 3.1 重构前后对比

```
【重构前】                              【重构后】
┌─────────────────────────┐             ┌─────────────────────────────┐
│ dev_motor.h             │             │ dev_encoder.h (新)          │
│  #include dev_mt6701.h ←┼─问题        │  dev_encoder_t 抽象接口     │
│  #include dev_mt6835.h ←┼─问题        └──────────┬──────────────────┘
│  dev_encoder_t (内联定义)│                        │ include
│  dev_motor_t {          │             ┌──────────┴──────────────────┐
│    encoder; mt6701; ←───┼─问题        │ dev_motor.h (精简)          │
│    mt6835; ←────────────┼─问题        │  #include dev_encoder.h    │
│  }                      │             │  dev_motor_t { encoder; }  │
└────────┬────────────────┘             │    (无芯片实体字段)         │
         │ include                       └────────┬────────────────────┘
┌────────┴────────────────┐                      │ include
│ dev_motor.c             │             ┌────────┴────────────────────┐
│  14个适配函数 ←─────────┼─问题        │ dev_motor.c (精简)          │
│  #if 装配30行 ←─────────┼─问题        │  #include 适配头            │
│  dev_motor_init()       │             │  factory_call() ← 3行替代   │
└─────────────────────────┘             │  dev_motor_init()           │
                                       └────────┬────────────────────┘
                                                │ include
                           ┌─────────────────────┴──────────────┐
                           │                                      │
              ┌────────────┴───────────┐          ┌───────────────┴────────┐
              │ dev_encoder_mt6701.h/c  │          │ dev_encoder_mt6835.h/c│
              │  static s_mt6701[]     │          │  static s_mt6835[]     │
              │  7个适配函数           │          │  7个适配函数           │
              │  create() 工厂函数     │          │  create() 工厂函数     │
              └────────────────────────┘          └────────────────────────┘
```

### 3.2 核心设计决策

#### 决策1：提取 `dev_encoder.h` — 消除重复定义

`dev_encoder_t` 当前在 `dev_motor.h` 和 `dev_motor_virtual.h` 各定义一份（需手动保持一致）。提取到独立 `dev_encoder.h` 后，两处 `#include` 即可，单一定义源。

#### 决策2：实体外置 — 适配层持有 `static` 实体

不在 `dev_motor_t` 内嵌芯片实体，改由适配 `.c` 文件持有 `static` 数组：

```c
/* dev_encoder_mt6701.c */
static dev_mt6701_t s_mt6701[MT6701_ID_MAX];  // 单例，与 dev_power_monitor 风格一致
```

`dev_encoder_t.ctx` 指向 `&s_mt6701[id]`。项目单电机系统，`static` 单例不牺牲灵活性。

#### 决策3：工厂函数 — 一行装配全部接口

```c
void dev_encoder_mt6701_create(dev_encoder_t *enc, mt6701_id_e id);
```

调用者传入 `dev_encoder_t *` 和 `id`，函数内部完成：初始化实体 → 绑定 `ctx` → 装配 7 个方法指针。`dev_motor_init` 从 30 行 `#if` 装配缩减为 1 行工厂调用。

#### 决策4：`dev_motor.c` 保留最小 `#if` — 2行选型号

`dev_motor.c` 仍保留型号选择的 `#if`（2行 `#elif`），因为编译期必须知道链接哪个适配文件。这是不可消除的编译期分派，但代价极低。

---

## 4. 文件结构

### 4.1 新增文件（4个）

| 文件 | 职责 |
|------|------|
| `User/Devices/dev_encoder.h` | `dev_encoder_t` 抽象接口定义（从 dev_motor.h 提取） |
| `User/Devices/dev_encoder_mt6701.h` | MT6701 适配层公共 API（`create()` 声明） |
| `User/Devices/dev_encoder_mt6701.c` | MT6701 适配层实现（static 实体 + 7适配函数 + create） |
| `User/Devices/dev_encoder_mt6835.h` | MT6835 适配层公共 API |
| `User/Devices/dev_encoder_mt6835.c` | MT6835 适配层实现 |

> 注：`dev_encoder_mt6835` 为对称预留（当前板用 MT6701）。若当前不使用 MT6835 可只创建 MT6701 适配，MT6835 待需要时再加。

### 4.2 修改文件（3个）

| 文件 | 改动 |
|------|------|
| `User/Devices/dev_motor.h` | 删除 `#include dev_mt6701.h/dev_mt6835.h`；删除 `dev_encoder_t` 内联定义（改 include `dev_encoder.h`）；删除 `mt6701`/`mt6835` 实体字段 |
| `User/Devices/dev_motor.c` | 删除 14 个适配函数；删除 `#if` 装配块；替换为 `dev_encoder_xxx_create()` 工厂调用 |
| `User/MotorControl/CascadeControl/dev_motor_virtual.h` | 删除 `dev_encoder_t` 内联定义，改 `#include "dev_encoder.h"` |

### 4.3 不变文件

| 文件 | 原因 |
|------|------|
| `User/Devices/dev_mt6701.h/c` | 芯片驱动层不变，适配层调用它 |
| `User/Devices/dev_mt6835.h/c` | 同上 |
| `User/Devices/dev_config.c` | 已 include 芯片头用于配置表，无需改动 |
| `User/MotorCalibration/calib_hw.c` | 已通过 `m->encoder.*` 抽象层访问，无直接依赖 |
| `User/MotorCalibration/calib_level3_encoder.c` | 同上 |
| `User/MotorControl/CascadeControl/motor_loop.c` | 通过 `m->motor.encoder.*` 调用，API 不变 |

---

## 5. 任务分解

### Task 1：创建 `dev_encoder.h` — 提取抽象接口

**文件：**
- 创建：`User/Devices/dev_encoder.h`

**说明：** 从 `dev_motor.h` 提取 `dev_encoder_t` 定义到独立头文件，消除 `dev_motor.h` 与 `dev_motor_virtual.h` 的重复定义。

```c
/**
 * @file        dev_encoder.h
 * @brief       抽象编码器接口（与具体芯片型号无关）
 *
 * @details     控制层只面向本接口，不感知背后是 MT6701 / MT6835 / AS5047 等。
 *              适配层（dev_encoder_xxx.c）负责把具体芯片绑定到 ctx，
 *              并装配全部方法指针。
 *
 *              调用约定：先 update(self) 刷新，再读 self->mechanical_angle，
 *              或调 get_mechanical_angle(self)。
 *
 * @par 标定扩展接口（set_offset / set_dir / get_raw_deg）
 *              供 calib_hw / calib_level3 等标定模块使用，统一通过抽象层访问编码器。
 *              方向统一为 -1/1 约定（1=CW 正向, -1=CCW 反向），角度统一为 deg 单位。
 *              虚拟模式下这些方法可为 NULL（标定不在虚拟模式运行）。
 *
 * @note 本结构被 dev_motor.h 和 dev_motor_virtual.h 共同 include，
 *       保证真实/虚拟两种编译路径下布局一致。
 */
#ifndef __DEV_ENCODER_H__
#define __DEV_ENCODER_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 抽象编码器接口
 */
typedef struct dev_encoder
{
    void *ctx;                                                       /* 指向具体编码器对象（适配层静态实体）*/
    float mechanical_angle;                                          /* 最新机械角度(deg)，update 后刷新 */
    void (*update)(struct dev_encoder *pobj);                        /* 刷新角度 */
    float (*get_mechanical_angle)(struct dev_encoder *pobj);         /* 读取机械角度(deg) */
    /* 标定扩展接口：统一 -1/1 方向约定，统一 deg 单位 */
    void (*set_offset)(struct dev_encoder *pobj, float offset_deg);  /* 设置零点偏移(deg) */
    float (*get_offset)(struct dev_encoder *pobj);                  /* 读取零点偏移(deg) */
    void (*set_dir)(struct dev_encoder *pobj, int8_t dir);           /* 设置方向: 1=CW, -1=CCW */
    int8_t (*get_dir)(struct dev_encoder *pobj);                    /* 读取方向: 1/-1 */
    float (*get_raw_deg)(struct dev_encoder *pobj);                 /* 原始角度(deg)，未补偿 */
} dev_encoder_t;

#ifdef __cplusplus
}
#endif

#endif /* __DEV_ENCODER_H__ */
```

---

### Task 2：创建 `dev_encoder_mt6701.h/c` — MT6701 适配层

**文件：**
- 创建：`User/Devices/dev_encoder_mt6701.h`
- 创建：`User/Devices/dev_encoder_mt6701.c`

**说明：** 将 `dev_motor.c` 中的 7 个 MT6701 适配函数 + 实体初始化 + 接口装配逻辑迁移到独立文件，以 `static` 实体 + 工厂函数形式封装。

#### `dev_encoder_mt6701.h`

```c
/**
 * @file        dev_encoder_mt6701.h
 * @brief       MT6701 编码器适配层 — 把 dev_mt6701_t 适配到 dev_encoder_t 抽象接口
 *
 * @details     本文件持有 MT6701 的静态实体存储，通过 create() 工厂函数
 *              初始化实体并装配抽象接口的全部方法指针。
 *              调用后即可通过 enc->update(enc) 等抽象接口访问 MT6701。
 *
 *              新增编码器型号时，参照本文件创建 dev_encoder_<new>.h/c。
 */
#ifndef __DEV_ENCODER_MT6701_H__
#define __DEV_ENCODER_MT6701_H__

#include "dev_config.h"
#if defined(USE_DEV_MT6701)

#include "dev_encoder.h"
#include "dev_mt6701.h"  /* mt6701_id_e, dev_mt6701_t */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 创建 MT6701 编码器实例并装配抽象接口
 * @param enc  抽象编码器对象（由调用者提供存储，通常为 &dev_motor_t.encoder）
 * @param id   MT6701 设备编号
 * @details 内部初始化静态 MT6701 实体，绑定到 enc->ctx，
 *          并装配全部 7 个方法指针。
 */
void dev_encoder_mt6701_create(dev_encoder_t *enc, mt6701_id_e id);

#ifdef __cplusplus
}
#endif
#endif /* USE_DEV_MT6701 */
#endif /* __DEV_ENCODER_MT6701_H__ */
```

#### `dev_encoder_mt6701.c`

```c
/**
 * @file        dev_encoder_mt6701.c
 * @brief       MT6701 编码器适配层实现
 */
#include "dev_encoder_mt6701.h"

#if defined(USE_DEV_MT6701)

#include <string.h>
#include "assert_report.h"

/*============================================================================
 * 静态实体存储
 *   单例数组，按 id 索引。与 dev_power_monitor 全局单例风格一致。
 *   dev_encoder_t.ctx 指向 &s_mt6701[id]。
 *==========================================================================*/
static dev_mt6701_t s_mt6701[MT6701_ID_MAX];

/*============================================================================
 * MT6701 → 抽象编码器 适配函数
 *   把 dev_mt6701_t 的具体接口适配到 dev_encoder_t 抽象接口。
 *   对外只通过 dev_encoder_t 方法指针暴露，不直接调用。
 *==========================================================================*/

static void encoder_mt6701_update(struct dev_encoder *enc)
{
    dev_mt6701_t *chip = (dev_mt6701_t *)enc->ctx;
    chip->update(chip);
    enc->mechanical_angle = chip->mechanical_angle; /* mt6701 无 getter，直接读字段 */
}

static float encoder_mt6701_get_mechanical_angle(struct dev_encoder *enc)
{
    return enc->mechanical_angle;
}

static void encoder_mt6701_set_offset(struct dev_encoder *enc, float offset_deg)
{
    dev_mt6701_t *chip = (dev_mt6701_t *)enc->ctx;
    chip->offset = offset_deg;
}

static float encoder_mt6701_get_offset(struct dev_encoder *enc)
{
    dev_mt6701_t *chip = (dev_mt6701_t *)enc->ctx;
    return chip->offset;
}

static void encoder_mt6701_set_dir(struct dev_encoder *enc, int8_t dir)
{
    dev_mt6701_t *chip = (dev_mt6701_t *)enc->ctx;
    /* MT6701 枚举已统一为 -1/1，与抽象层约定一致，直接赋值 */
    chip->set_dir(chip, (mt6701_dir_e)dir);
}

static int8_t encoder_mt6701_get_dir(struct dev_encoder *enc)
{
    dev_mt6701_t *chip = (dev_mt6701_t *)enc->ctx;
    return (int8_t)chip->get_dir(chip);
}

static float encoder_mt6701_get_raw_deg(struct dev_encoder *enc)
{
    dev_mt6701_t *chip = (dev_mt6701_t *)enc->ctx;
    chip->update(chip); /* 确保 raw 字段已刷新 */
    return (float)chip->raw / MT6701_ANGLE_RESOLUTION * 360.0F;
}

/*============================================================================
 * 工厂函数：初始化实体 + 装配抽象接口
 *==========================================================================*/

void dev_encoder_mt6701_create(dev_encoder_t *enc, mt6701_id_e id)
{
    assert_report(enc != NULL);
    assert_report(id < MT6701_ID_MAX);

    /* 初始化静态实体 */
    dev_mt6701_init(&s_mt6701[id], id);

    /* 装配抽象接口 */
    enc->ctx = &s_mt6701[id];
    enc->mechanical_angle = 0.0F;
    enc->update = encoder_mt6701_update;
    enc->get_mechanical_angle = encoder_mt6701_get_mechanical_angle;
    enc->set_offset = encoder_mt6701_set_offset;
    enc->get_offset = encoder_mt6701_get_offset;
    enc->set_dir = encoder_mt6701_set_dir;
    enc->get_dir = encoder_mt6701_get_dir;
    enc->get_raw_deg = encoder_mt6701_get_raw_deg;
}

#endif /* USE_DEV_MT6701 */
```

---

### Task 3：创建 `dev_encoder_mt6835.h/c` — MT6835 适配层

**文件：**
- 创建：`User/Devices/dev_encoder_mt6835.h`
- 创建：`User/Devices/dev_encoder_mt6835.c`

**说明：** 与 Task 2 对称，迁移 MT6835 的 7 个适配函数。当前板未使用 MT6835（`USE_DEV_MT6835` 未定义），本 Task 可选——若暂不创建，Task 5 中去掉对应 `#elif` 分支即可。

#### `dev_encoder_mt6835.h`

```c
/**
 * @file        dev_encoder_mt6835.h
 * @brief       MT6835 编码器适配层 — 把 dev_mt6835_t 适配到 dev_encoder_t 抽象接口
 */
#ifndef __DEV_ENCODER_MT6835_H__
#define __DEV_ENCODER_MT6835_H__

#include "dev_config.h"
#if defined(USE_DEV_MT6835)

#include "dev_encoder.h"
#include "dev_mt6835.h"

#ifdef __cplusplus
extern "C" {
#endif

void dev_encoder_mt6835_create(dev_encoder_t *enc, mt6835_id_e id);

#ifdef __cplusplus
}
#endif
#endif /* USE_DEV_MT6835 */
#endif /* __DEV_ENCODER_MT6835_H__ */
```

#### `dev_encoder_mt6835.c`

```c
/**
 * @file        dev_encoder_mt6835.c
 * @brief       MT6835 编码器适配层实现
 */
#include "dev_encoder_mt6835.h"

#if defined(USE_DEV_MT6835)

#include <string.h>
#include "assert_report.h"

static dev_mt6835_t s_mt6835[MT6835_ID_MAX];

/* ---- MT6835 → 抽象编码器 适配函数 ---- */

static void encoder_mt6835_update(struct dev_encoder *enc)
{
    dev_mt6835_t *chip = (dev_mt6835_t *)enc->ctx;
    chip->update(chip);
    enc->mechanical_angle = chip->get_mechanical_angle(chip);
}

static float encoder_mt6835_get_mechanical_angle(struct dev_encoder *enc)
{
    return enc->mechanical_angle;
}

static void encoder_mt6835_set_offset(struct dev_encoder *enc, float offset_deg)
{
    dev_mt6835_t *chip = (dev_mt6835_t *)enc->ctx;
    chip->set_offset(chip, offset_deg);
}

static float encoder_mt6835_get_offset(struct dev_encoder *enc)
{
    dev_mt6835_t *chip = (dev_mt6835_t *)enc->ctx;
    return chip->offset;
}

static void encoder_mt6835_set_dir(struct dev_encoder *enc, int8_t dir)
{
    dev_mt6835_t *chip = (dev_mt6835_t *)enc->ctx;
    /* MT6835 约定：running_dir > 1 表示反向，适配到 -1/1 */
    chip->set_dir(chip, (dir < 0) ? 2 : 1);
}

static int8_t encoder_mt6835_get_dir(struct dev_encoder *enc)
{
    dev_mt6835_t *chip = (dev_mt6835_t *)enc->ctx;
    return (chip->running_dir > 1) ? -1 : 1;
}

static float encoder_mt6835_get_raw_deg(struct dev_encoder *enc)
{
    dev_mt6835_t *chip = (dev_mt6835_t *)enc->ctx;
    /* MT6835 21bit 原始角度转 deg */
    return (float)chip->get_mechanical_angle_raw(chip) / MT6835_ANGLE_RESOLUTION * 360.0F;
}

/* ---- 工厂函数 ---- */

void dev_encoder_mt6835_create(dev_encoder_t *enc, mt6835_id_e id)
{
    assert_report(enc != NULL);
    assert_report(id < MT6835_ID_MAX);

    dev_mt6835_init(&s_mt6835[id], id);

    enc->ctx = &s_mt6835[id];
    enc->mechanical_angle = 0.0F;
    enc->update = encoder_mt6835_update;
    enc->get_mechanical_angle = encoder_mt6835_get_mechanical_angle;
    enc->set_offset = encoder_mt6835_set_offset;
    enc->get_offset = encoder_mt6835_get_offset;
    enc->set_dir = encoder_mt6835_set_dir;
    enc->get_dir = encoder_mt6835_get_dir;
    enc->get_raw_deg = encoder_mt6835_get_raw_deg;
}

#endif /* USE_DEV_MT6835 */
```

---

### Task 4：重构 `dev_motor.h` — 精简结构体

**文件：**
- 修改：`User/Devices/dev_motor.h`

**改动点：**

1. 删除 `#include "dev_mt6701.h"` 和 `#include "dev_mt6835.h"`
2. 添加 `#include "dev_encoder.h"`
3. 删除内联的 `dev_encoder_t` 结构体定义（~13行）
4. 删除 `dev_motor_t` 中的 `dev_mt6701_t mt6701;` 和 `dev_mt6835_t mt6835;` 字段

**修改后完整文件头与结构体：**

```c
#ifndef __DEV_MOTOR_H__
#define __DEV_MOTOR_H__

#include <stdint.h>
#include "dev_encoder.h"        /* dev_encoder_t 抽象接口（替代内联定义）*/
#include "foc_core.h"
#include "dev_power_monitor.h"
#include "dev_half_bridge.h"
#include "motion_param.h"
#include "multiturn_counter.h"
#include "dev_motor_phase_current.h"

/*============================================================================
 * 编码器型号选择
 *   控制层只面向 dev_encoder_t 抽象接口，与型号无关；具体用哪颗芯片由本宏决定。
 *   适配层文件（dev_encoder_xxx.c）按本宏条件包含。
 *==========================================================================*/
#define DEV_MOTOR_ENCODER_MT6701 1
#define DEV_MOTOR_ENCODER_MT6835 2

#ifndef DEV_MOTOR_ENCODER_TYPE
#define DEV_MOTOR_ENCODER_TYPE DEV_MOTOR_ENCODER_MT6701
#endif

typedef enum
{
    DEV_MOTOR_1 = 0,
    // DEV_MOTOR_2,
    DEV_MOTOR_MAX,
} motor_id_e;

typedef struct
{
    char name[20];
    gpioDrv_t gpio;
} dev_motor_enable_config_t;

typedef struct
{
    float position_target;
    float velocity_target;
    float current_target;
} motor_ctrl_target_t;

/* dev_encoder_t 已移至 dev_encoder.h，此处不再重复定义 */

typedef struct dev_motor
{
    motor_id_e id;
    uint8_t poles;

    /* 外部输入回调函数 */
    focCurrent_t (*current_callback)(void);
    float (*ele_radian_callback)(void);

    motor_ctrl_target_t target;
    timNumber_e fsm_tim;

    /* public */
    dev_encoder_t encoder;             /* 抽象编码器（ctx 指向适配层静态实体）*/
    motion_param_t motor_param;        /* 角度/速度转化 */
    multiturn_t multiturn;             /* 绝对多圈计数 */
    foc_t foc;                         /* foc */
    dev_half_bridge_t half_bridge;     /* dev_half_bridge */
    dev_phase_current_t phase_current; /* adc for current */
} dev_motor_t;

void dev_motor_init(dev_motor_t *pobj, motor_id_e id,
                    focCurrent_t (*current_callback)(void),
                    float (*ele_radian_callback)(void));

void dev_motor_set_encoder_dir(dev_motor_t *pobj, int8_t dir);

#endif /* __DEV_MOTOR_H__ */
```

**关键变化：** `dev_motor_t` 不再包含 `mt6701`/`mt6835` 实体字段；芯片实体移至适配层 `static` 存储，通过 `encoder.ctx` 间接访问。

---

### Task 5：重构 `dev_motor.c` — 移除适配函数，改用工厂调用

**文件：**
- 修改：`User/Devices/dev_motor.c`

**改动点：**

1. 添加适配层头文件条件包含
2. 删除全部 14 个 `encoder_mt6701_*` / `encoder_mt6835_*` 适配函数（约 95 行）
3. 删除 `dev_motor_init` 内的 `#if`/`#elif` 装配块（约 30 行）
4. 替换为单行工厂函数调用

**修改后关键代码段：**

```c
#include "motor_loop_config.h"

#if (MOTOR_LOOP_ENABLE_DEV_DRIVER)

#include "dev_motor.h"
#include "assert_report.h"
#include "runtime_param.h"

/* 编码器适配层头（按型号条件包含，替代原 dev_motor.c 内联的适配函数）*/
#if (DEV_MOTOR_ENCODER_TYPE == DEV_MOTOR_ENCODER_MT6701)
#include "dev_encoder_mt6701.h"
#elif (DEV_MOTOR_ENCODER_TYPE == DEV_MOTOR_ENCODER_MT6835)
#include "dev_encoder_mt6835.h"
#endif

static dev_motor_enable_config_t motor_enable_list[DEV_MOTOR_MAX] = {
    {"MOTOR1_EN", {(gpioType_e)DRV_GPIOB, (gpioPin_e)DRV_PIN_2, (drvPinState_e)0}},
};

static void dev_motor_enable(void)
{
    drv_gpio_write(motor_enable_list[DEV_MOTOR_1].gpio, (drvPinState_e)1);
}

static void dev_motor_disable(void)
{
    drv_gpio_write(motor_enable_list[DEV_MOTOR_1].gpio, (drvPinState_e)0);
}

static float device_compensation(void)
{
    return 0.0F;
}

static uint8_t dev_motor_get_poles(motor_id_e id)
{
    uint8_t poles;
    switch (id)
    {
        case DEV_MOTOR_1: poles = 7; break;
        default: poles = 7; break;
    }
    return poles;
}

/* ===== 以下原 14 个适配函数已移至 dev_encoder_mt6701.c / dev_encoder_mt6835.c ===== */

void dev_motor_init(dev_motor_t *pobj,
                    motor_id_e id,
                    focCurrent_t (*current_callback)(void),
                    float (*ele_radian_callback)(void))
{
    assert_report(pobj != NULL);
    assert_report(current_callback != NULL);
    assert_report(ele_radian_callback != NULL);
    assert_report(id < DEV_MOTOR_MAX);
    memset(pobj, 0, sizeof(dev_motor_t));

    dev_motor_disable();

    pobj->id = id;
    pobj->poles = dev_motor_get_poles((motor_id_e)id);
    pobj->fsm_tim = DRV_TIM2;

    /* 创建编码器实例并装配抽象接口（1行替代原30行 #if 装配）*/
#if (DEV_MOTOR_ENCODER_TYPE == DEV_MOTOR_ENCODER_MT6701)
    dev_encoder_mt6701_create(&pobj->encoder, (mt6701_id_e)id);
#elif (DEV_MOTOR_ENCODER_TYPE == DEV_MOTOR_ENCODER_MT6835)
    dev_encoder_mt6835_create(&pobj->encoder, (mt6835_id_e)id);
#else
#error "未知的 DEV_MOTOR_ENCODER_TYPE，请在 dev_motor.h 选择支持的编码器型号"
#endif

    /* 从已标定参数加载编码器零位和方向 */
    {
        const encoder_param_t *enc_cfg = &usr.motor_param[(motor_num_e)id].encoder_param;
        pobj->encoder.set_offset(&pobj->encoder, enc_cfg->enc_offset);
        pobj->encoder.set_dir(&pobj->encoder, enc_cfg->enc_direction);
    }

    /* 以下初始化逻辑不变 */
    motion_param_init(&pobj->motor_param, pobj->poles, 10, NULL);

    multiturn_config_t mt_cfg;
    memset(&mt_cfg, 0, sizeof(mt_cfg));
    mt_cfg.mode = MULTITURN_MODE_SOFT;
    mt_cfg.device_compensation_callback = device_compensation;
    multiturn_init(&pobj->multiturn, &mt_cfg);

    dev_half_bridge_init(&pobj->half_bridge, (half_bridge_id_e)id);

    dev_phase_current_init(&pobj->phase_current, PHASE_CURRENT_GAIN, PHASE_CURRENT_SHUNT);
    pobj->phase_current.set_offset(&pobj->phase_current,
                                   (dev_current_i3axis_t){PHASE_CURRENT_ZERO_ADC,
                                                          PHASE_CURRENT_ZERO_ADC,
                                                          PHASE_CURRENT_ZERO_ADC});

    pobj->current_callback = current_callback;
    pobj->ele_radian_callback = ele_radian_callback;
    foc_init(&pobj->foc, pobj->current_callback, pobj->ele_radian_callback);

    dev_motor_enable();
}

void dev_motor_set_encoder_dir(dev_motor_t *pobj, int8_t dir)
{
    assert_report(pobj != NULL);
    if (dir != 1 && dir != -1)
    {
        return;
    }
    pobj->encoder.set_dir(&pobj->encoder, dir);
    motor_param_set_enc_direction(&usr.motor_param[(motor_num_e)pobj->id], dir);
}

#endif /* MOTOR_LOOP_ENABLE_DEV_DRIVER */
```

---

### Task 6：重构 `dev_motor_virtual.h` — 统一 include

**文件：**
- 修改：`User/MotorControl/CascadeControl/dev_motor_virtual.h`

**改动点：**

1. 删除内联的 `dev_encoder_t` 结构体定义（~13行）
2. 添加 `#include "dev_encoder.h"`

**修改后头文件片段：**

```c
#ifndef __DEV_MOTOR_VIRTUAL_H__
#define __DEV_MOTOR_VIRTUAL_H__

#include <stdint.h>
#include "dev_encoder.h"        /* dev_encoder_t 抽象接口（与 dev_motor.h 共享同一定义）*/
#include "foc_core.h"
#include "motion_param.h"
#include "multiturn_counter.h"

/* dev_encoder_t 已移至 dev_encoder.h，此处不再重复定义 */

typedef enum
{
    DEV_MOTOR_1 = 0,
    DEV_MOTOR_MAX,
} motor_id_e;

/* ... 其余内容不变 ... */
```

---

### Task 7：Keil 工程更新 + 编译验证

**步骤：**

1. **Keil 工程添加新源文件**（手动操作）
   - 将 `dev_encoder_mt6701.c` 添加到 Keil 工程的 `User/Devices` 分组
   - 若创建了 `dev_encoder_mt6835.c`，一并添加

2. **编译验证**
   - 全编译（Rebuild）
   - 预期：0 error, 0 warning
   - 若出现 `undefined symbol dev_encoder_mt6701_create`：检查 `.c` 是否加入工程、`USE_DEV_MT6701` 是否在板级配置中定义

3. **功能验证**
   - 烧录后确认：IDLE 态编码器角度正常刷新（`motor_loop_isr` step0）
   - 确认编码器零位标定、方向标定功能正常（`calib_level3_encoder.c`）
   - 确认 `dev_motor_set_encoder_dir` 运行时翻转方向正常

---

## 6. 扩展性示例：新增 AS5047 编码器

按本方案，新增 AS5047 编码器只需 3 步，**不修改 `dev_motor.h`**：

### 步骤1：创建芯片驱动（如已有则跳过）

```
User/Devices/dev_as5047.h  — dev_as5047_t 结构 + dev_as5047_init()
User/Devices/dev_as5047.c  — SPI 读取实现
```

### 步骤2：创建适配层

```
User/Devices/dev_encoder_as5047.h  — dev_encoder_as5047_create() 声明
User/Devices/dev_encoder_as5047.c  — static s_as5047[] + 7适配函数 + create()
```

### 步骤3：在 `dev_motor.h` 加型号宏 + `dev_motor.c` 加 `#elif`

```c
/* dev_motor.h */
#define DEV_MOTOR_ENCODER_AS5047 3
// 切换时：#define DEV_MOTOR_ENCODER_TYPE DEV_MOTOR_ENCODER_AS5047
```

```c
/* dev_motor.c */
#if (DEV_MOTOR_ENCODER_TYPE == DEV_MOTOR_ENCODER_MT6701)
    #include "dev_encoder_mt6701.h"
#elif (DEV_MOTOR_ENCODER_TYPE == DEV_MOTOR_ENCODER_MT6835)
    #include "dev_encoder_mt6835.h"
#elif (DEV_MOTOR_ENCODER_TYPE == DEV_MOTOR_ENCODER_AS5047)   /* 新增2行 */
    #include "dev_encoder_as5047.h"
#endif

// dev_motor_init 内：
#if (DEV_MOTOR_ENCODER_TYPE == DEV_MOTOR_ENCODER_MT6701)
    dev_encoder_mt6701_create(&pobj->encoder, (mt6701_id_e)id);
#elif (DEV_MOTOR_ENCODER_TYPE == DEV_MOTOR_ENCODER_MT6835)
    dev_encoder_mt6835_create(&pobj->encoder, (mt6835_id_e)id);
#elif (DEV_MOTOR_ENCODER_TYPE == DEV_MOTOR_ENCODER_AS5047)  /* 新增2行 */
    dev_encoder_as5047_create(&pobj->encoder, (as5047_id_e)id);
#endif
```

**对比重构前**：无需改 `dev_motor_t` 结构体、无需改 `dev_motor.h` include、无需在 `dev_motor.c` 写 7 个适配函数。

---

## 7. 风险评估与回滚方案

### 7.1 风险点

| 风险 | 等级 | 缓解措施 |
|------|------|----------|
| `static` 实体被多次 `create()` 重复初始化 | 低 | 单电机单 id，`dev_motor_init` 只调一次；`assert_report` 校验 id |
| Keil 未添加新 `.c` 导致链接错误 | 中 | Task 7 明确列出需添加的文件 |
| `dev_motor_virtual.h` 漏改导致布局不一致 | 中 | Task 6 统一 include `dev_encoder.h`，编译期即可发现 |
| 适配函数 `static` 可见性：原在 `dev_motor.c` 内 `static`，迁移后仍在适配 `.c` 内 `static` | 无 | 行为完全一致，仅文件位置变化 |

### 7.2 回滚方案

本次重构为纯结构整理，无算法/逻辑变更。回滚步骤：

1. `git revert` 回退本次提交
2. 恢复 Keil 工程文件（移除新增 `.c`）

---

## 8. 验证清单

- [ ] `dev_motor.h` 中无 `#include "dev_mt6701.h"` / `#include "dev_mt6835.h"`
- [ ] `dev_motor.h` 中 `dev_motor_t` 无 `mt6701` / `mt6835` 字段
- [ ] `dev_motor.c` 中无 `encoder_mt6701_*` / `encoder_mt6835_*` 函数
- [ ] `dev_motor.c` 中 `dev_motor_init` 编码器初始化不超过 5 行（`#if` + create + `#elif` + create + `#endif`）
- [ ] `dev_encoder.h` 被 `dev_motor.h` 和 `dev_motor_virtual.h` 共同 include
- [ ] `dev_encoder_mt6701.c` 内有 `static dev_mt6701_t s_mt6701[MT6701_ID_MAX]`
- [ ] Keil 全编译 0 error 0 warning
- [ ] 烧录后 IDLE 态角度正常刷新
- [ ] 编码器零位/方向标定功能正常

---

## 9. 执行顺序

```
Task 1 (dev_encoder.h)
  ├── Task 2 (dev_encoder_mt6701)
  ├── Task 3 (dev_encoder_mt6835)
  ├── Task 4 (dev_motor.h)
  └── Task 6 (dev_motor_virtual.h)
        │
        └── Task 5 (dev_motor.c)     ← 依赖 Task 2/3/4
              │
              └── Task 7 (编译验证)   ← 依赖全部
```

> 本方案为**设计文档**，暂不执行。用户确认方案后，按 Task 1 → Task 7 顺序执行。
