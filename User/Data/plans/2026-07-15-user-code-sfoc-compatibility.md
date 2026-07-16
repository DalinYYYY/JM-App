# User 代码跨工程（MKS ↔ SFOC）兼容性分析与优化方案

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 消除 User/ 目录代码在 MKS(STM32F405RGT6) 与 SFOC(STM32G473CBT6) 两个工程间切换时的兼容性缺陷，实现同一份 User 代码零修改双向编译通过。

**Architecture:** 以 Keil 工程预定义的 HAL 标准宏（`STM32F405xx` / `STM32G474xx` / `STM32G473xx`）为单一事实源，通过新建 `mcu_compat.h` 统一 User/ 下所有 MCU 相关条件编译分支，并覆盖 F1/F3/F4/G4/L4/H7 全系列；将 `drv_config.h` 的 `FLASH_MCU` 硬编码改为从 HAL 宏自动派生（L4 复用 G4 Flash 路径）；修复 `drv_usart.c` 硬编码 HAL 头（P0 阻塞，用 `main.h` 替代）；补齐 `drv_adc.c` 缺失的 F4 分支并统一为 `JM_MCU_*`；统一 `drv_can.c` 的系列宏并支持 F1；收敛 `user_interface.c` 对 CubeMX 生成头的直接依赖。板级差异继续通过 `Board/<board>/Config/` 覆盖机制隔离，User/ 业务层不感知板级。新增新系列时只需在 `mcu_compat.h` 加一行映射 + 在 `drv_config.h` 选对应 Flash 驱动，实现低成本扩展。

**Tech Stack:** STM32F405RGT6 / STM32G473CBT6 / HAL F4 & G4 / Keil MDK-ARM / C99 / C-OOP 设备对象模式 / 无主机端单元测试框架（验证方式：编译通过 + 调试器 Watch 观察）

---

## 一、兼容性现状评估

### 1.1 三块板与 MCU 映射

| 板宏 | 板目录 | MCU 型号 | Keil 预定义宏 | Keil 工程文件 | 预定义行 |
|---|---|---|---|---|---|
| `JM_BOARD_V1` | `Board/V1/` | STM32G473CBTx | `STM32G474xx` | `JointMotorApp.uvprojx` | 第 340 行 |
| `JM_BOARD_SFOC` | `Board/SFOC/` | STM32G473CBT6 | `STM32G474xx` | `sfoc.uvprojx` | 第 17 行 |
| `JM_BOARD_ODRIVE` | `Board/ODriveMKS/` | STM32F405RGT6 | `STM32F405xx` | `OdriveMini.uvprojx` | 第 340 行 |

> **说明**：SFOC 实际芯片为 STM32G473CBT6，但 Keil 预定义 `STM32G474xx`。G473 与 G474 同属 G4 系列，HAL 层完全兼容（寄存器集相同，仅外设数量差异），HAL 头 `stm32g4xx_hal.h` 对两者统一处理。

### 1.2 main.h 的 HAL 头包含链路（关键事实源）

每个板的 `main.h`（CubeMX 生成）第 30 行硬编码对应板的 HAL 头：

| 板 | main.h 路径 | 第 30 行 include | 引入的 HAL 头 |
|---|---|---|---|
| ODrive(MKS) | `Board/ODriveMKS/Inc/main.h` | `#include "stm32f4xx_hal.h"` | F4 HAL |
| SFOC | `Board/SFOC/Core/Inc/main.h` | `#include "stm32g4xx_hal.h"` | G4 HAL |

**User/ 下有 7 个文件直接 `#include "main.h"`**，通过 main.h 间接获得对应板的 HAL 类型定义：

| 文件 | 行号 | 是否有 MCU 条件保护 |
|---|---|---|
| `User/AppEntry/thread_management.c` | 20 | 无（但仅用 HAL_GetTick 等通用 API） |
| `User/AppEntry/control_irq.h` | 4 | 无（但仅用 HAL_GPIO 等） |
| `User/Config/system_config.h` | 4 | 无 |
| `User/AppServices/ThreadManager/thread_commun.c` | 26 | 无 |
| `User/Driver/drv_can.c` | 19 | `#ifdef USE_CAN_DRIVER`（当前未启用） |
| `User/Driver/drv_flash_f4.h` | 21 | `#ifdef USE_FLASH_F4_DRIVER`（F4 专属） |
| `User/Driver/drv_flash_g4.h` | 27 | `#ifdef USE_FLASH_G4_DRIVER`（G4 专属） |

**结论**：通过 main.h 间接获得 HAL 头的文件，跨工程自动适配（各板 main.h 含各自的 HAL 头），无需修改。

### 1.3 宏命名现状（兼容性根因分析）

User/ 下存在 **三种并行的 MCU 宏命名风格**，是兼容性问题的核心根因：

| 风格 | 形式 | 来源 | 示例文件 | 可靠性 |
|---|---|---|---|---|
| **HAL 标准（带 xx）** | `STM32F405xx` / `STM32G474xx` / `STM32G473xx` | Keil 工程选项预定义（`-D` 编译器参数） | `dev_dwt_counter.c:23-26` | ✅ 最可靠：所有翻译单元都成立，不依赖头文件包含顺序 |
| **HAL 系列宏（无 xx）** | `STM32F4` / `STM32G4` / `STM32H7` | CMSIS 设备头内部派生（不确定是否定义） | `drv_can.c:25,28,37,42...` `drv_adc.c:129,132` | ⚠️ 不可靠：依赖于 HAL 系列头（`stm32f4xx.h`/`stm32g4xx.h`）已被包含，CMSIS 不同版本定义行为不一致 |
| **drv_config.h 派生（无 xx）** | `STM32F405` / `STM32G474` | `drv_config.h` 的 `FLASH_MCU` 派生 | `drv_config.h:108-133` | ❌ 不匹配任何 HAL 标准宏，仅用于派生 `USE_FLASH_*_DRIVER` |

**风险详解**：

第 2 类（系列宏 `STM32F4`/`STM32G4`）的成立条件是：当前翻译单元**已经** `#include` 了 CMSIS 设备头（`stm32f4xx.h` 或 `stm32g4xx.h`，通常经 `main.h` → `stm32f4xx_hal.h` → `stm32f4xx.h` 链路引入）。

- `drv_can.c:19` 先 `#include "main.h"`，所以系列宏可能成立——但 `drv_can.c` 被 `#ifdef USE_CAN_DRIVER` 保护（当前未启用），不参与编译，风险潜伏。
- `drv_adc.c:21` 先 `#include "adc.h"`（CubeMX 生成，含 `main.h` → HAL 头），所以系列宏可能成立。但 F4 工程下 `#if defined(STM32G4)` 为假，`#elif defined(STM32F1) || defined(STM32F3)` 也为假（F4 既不是 F1 也不是 F3），**两个分支都不执行**，`HAL_ADCEx_Calibration_Start` 不被调用，ADC 未校准（隐藏功能缺陷）。

第 3 类（`drv_config.h` 派生宏 `STM32F405`/`STM32G474`）无 `xx` 后缀，不匹配任何 HAL 标准宏。经 Grep 全局搜索确认：**User/ 下没有任何文件用 `defined(STM32F405)` / `defined(STM32G474)` 做条件判断**（零引用），仅 `drv_config.h` 内部用于派生 `USE_FLASH_*_DRIVER`。

### 1.4 HAL 头硬编码现状

经 Grep 全局搜索 `stm32f4xx_hal.h` / `stm32g4xx_hal.h`，User/ 下共 **3 处** HAL 头直接包含：

| 文件 | 行号 | 内容 | 是否条件保护 | 影响 |
|---|---|---|---|---|
| `User/Devices/dev_dwt_counter.c` | 23-27 | `#if defined(STM32F405xx)` → f4 hal / `#elif defined(STM32G474xx)` → g4 hal | ✅ 已条件保护 | 无（参考样板） |
| `User/Driver/drv_usart.c` | 23 | `#include "stm32f4xx_hal.h"` | ❌ 硬编码 | **P0：SFOC(G4) 编译失败** |
| `User/Driver/drv_flash_f4.c` | 13-14 | `#include "stm32f4xx_hal_flash.h"` + `_ex.h` | ✅ `#ifdef USE_FLASH_F4_DRIVER` | 无（G4 下不编译） |
| `User/Driver/drv_flash_g4.c` | 21-23 | `#include "stm32g4xx_hal_flash.h"` + `_ex.h` + `_ramfunc.h` | ✅ `#ifdef USE_FLASH_G4_DRIVER` | 无（F4 下不编译） |

**结论**：仅 `drv_usart.c:23` 是未保护的硬编码，阻塞 SFOC 编译。`drv_flash_f4.c` / `drv_flash_g4.c` 的硬编码头已被各自驱动宏正确保护，互斥编译，安全。

### 1.5 兼容性问题分级清单（终版）

#### P0 阻塞 SFOC 编译（必须立即修复）

| # | 文件 | 行号 | 问题 | 影响 | 修复 Task |
|---|---|---|---|---|---|
| P0-1 | `User/Driver/drv_usart.c` | 23 | 硬编码 `#include "stm32f4xx_hal.h"` | SFOC(G4) 工程编译失败：找不到 F4 HAL 头或 include path 错误 | Task 2 |

#### P1 高风险（潜在编译/运行缺陷）

| # | 文件 | 行号 | 问题 | 影响 | 修复 Task |
|---|---|---|---|---|---|
| P1-1 | `User/Driver/drv_config.h` | 105 | `FLASH_MCU` 硬编码 `0x03`（F4） | 切 SFOC 需手动改 `0x06`，易遗忘；改错会导致 Flash 驱动选错分支 | Task 1 |
| P1-2 | `User/Driver/drv_config.h` | 107-134 | 派生宏 `STM32F405`/`STM32G474` 无 `xx` 后缀，不匹配 HAL 标准 | 派生宏本身不可用于 HAL 判断（零引用，仅内部派生用） | Task 1 |
| P1-3 | `User/Driver/drv_adc.c` | 129-135 | ADC 校准分支缺 F4（仅有 G4/L4/H7 和 F1/F3） | F4 工程下 `HAL_ADCEx_Calibration_Start` 不被调用，ADC 未校准（隐藏功能缺陷） | Task 3 |

#### P2 中风险（冗余或耦合，不影响功能但应优化）

| # | 文件 | 行号 | 问题 | 影响 | 修复 Task |
|---|---|---|---|---|---|
| P2-1 | `User/Driver/drv_usart.c` | 443-461 | `HAL_UART_ErrorCallback` 为 F4 DMA bug 修复，G4 上冗余 | G4 上执行无害（已有注释），仅增加代码冗余 | Task 2（注释增强） |
| P2-2 | `User/AppEntry/user_interface.c` | 32 | `#include "tim.h"` 直接依赖 CubeMX 生成头（带 TODO） | 真实电机模式下不需要 tim.h，收敛到条件编译内 | Task 5 |
| P2-3 | `User/Driver/drv_can.c` | 25,28,37,42,57,106,129,160,180,213,225,254,273 | 用 `STM32F4`/`STM32G4` 系列宏而非 HAL 标准宏 | CAN 驱动当前未启用（`USE_CAN_DRIVER` 注释），不阻塞；启用前需确认宏成立 | Task 4 |

#### P3 已良好隔离（无需修改，记录备查）

| 项 | 文件 | 行号 | 隔离机制 |
|---|---|---|---|
| 板级设备使能 | `dev_config_board.h`（SFOC/ODrive 各自） | 全文 | `USE_DEV_*` 系列宏按板定义 |
| DRV8301 模块 | `dev_drv8301.c/h` | .h:21 / .c:12 | 双重 `USE_DEV_DRV8301` 保护，SFOC 不定义则完全不参与编译 |
| EN_GATE 引脚 | `dev_motor.c` | 41-51 | `JM_BOARD_ODRIVE` 单点切换（PB12 vs PB2） |
| Flash 地址 | `motor_info_storage.h` / `dev_config_board.h` | storage.h:68-76 / board:43-51 | `#undef`+`#define` 板级覆盖默认值 |
| Flash F4/G4 驱动 | `dev_flash.c` | 75 | `USE_FLASH_F4_DRIVER` 双路径 |
| Flash 驱动文件 | `drv_flash_f4.c/h` / `drv_flash_g4.c/h` | 整文件 | `USE_FLASH_*_DRIVER` 互斥保护，硬编码 HAL 头在保护内安全 |
| DWT 计数器 | `dev_dwt_counter.c` | 23-27 | 已用 HAL 标准宏 `STM32F405xx`/`STM32G474xx` 条件包含（参考样板） |
| 编码器型号 | `dev_motor.c` / `motor_loop_config.h` | dev_motor:33-39,109-117 | `DEV_MOTOR_ENCODER_TYPE` 宏 + 工厂函数装配 |
| 电机参数 | `motor_profile.c/h` | 全文 | 纯参数配置，三板共用 |
| 标定参数 | `calib_config.h` | 全文 | 纯 `MOTOR_*` 派生宏，自动适配 |
| 7 个 main.h 引用 | 见 1.2 节表 | - | 各板 main.h 含各自的 HAL 头，自动适配 |
| drv_tim/spi/gpio/i2c | `drv_tim.c` / `drv_spi.c` / `drv_gpio.c` / `drv_i2c.c` | - | `TIM/SPI/I2C_HandleTypeDef` 跨 F4/G4 类型一致；drv_gpio 用 `#ifdef GPIOE` 等端口宏自适应 |

#### P4 已知运行期问题（单独议题，本方案不处理）

- F4 128KB 扇区擦除关中断时间过长（2~4 秒）可能导致 `motor_info_storage` 每次上电走首次上电路径（见项目记忆 2026-07-15）
- `motor_profile.sync_to_param` 未标定时用 Flash 0 覆盖默认值（已修复，加 `is_calibrated==1` 门控）

---

## 二、优化策略与设计原则

### 2.1 单一事实源原则

**以 Keil 工程预定义的 HAL 标准宏（带 `xx` 后缀）为唯一 MCU 判定依据**：
- F4 工程：`STM32F405xx`（由 `OdriveMini.uvprojx` 第 340 行 `-D` 预定义）
- G4 工程：`STM32G474xx`（由 `sfoc.uvprojx` / `JointMotorApp.uvprojx` 预定义）

**原因**：编译器 `-D` 选项预定义的宏在**所有翻译单元**都成立，不依赖任何头文件包含顺序，是最可靠的 MCU 判定方式。而 CMSIS 设备头内部派生的系列宏（`STM32F4`/`STM32G4`）依赖于 HAL 头先被包含，跨文件不可靠。

**实施**：新建 `mcu_compat.h`，将 HAL 标准宏映射为 `JM_MCU_F4` / `JM_MCU_G4` / `JM_MCU_H7` 统一判定宏。所有 User/ 下需要按 MCU 切换的代码统一引用 `JM_MCU_*`。

### 2.2 自动派生原则

`drv_config.h` 的 `FLASH_MCU` 手动配置项**删除**，改为从 HAL 标准宏自动派生 `USE_FLASH_F4_DRIVER` / `USE_FLASH_G4_DRIVER`，消除"切工程忘改配置"的人因风险。

### 2.3 最小改动原则

- 板级差异继续由 `Board/<board>/Config/dev_config_board.h` 覆盖，User/ 不感知板级
- 不重构已良好隔离的模块（P3 类）
- 不引入新的抽象层（YAGNI），`mcu_compat.h` 是唯一的抽象新增点
- `HAL_UART_ErrorCallback` 在 G4 上无害，仅增强注释，不改逻辑
- `dev_dwt_counter.c` 已用正确的 HAL 标准宏，保留不改（作为"直接用 HAL 标准宏"的参考样板）

### 2.4 双向验证原则

每个修改必须同时在 MKS(ODrive/F4) 与 SFOC(G4) 两个 Keil target 下编译通过，0 error。User/ 下 0 warning（CubeMX 生成的 `Board/*/Src/*.c` 外设级 warning 可忽略）。

### 2.5 可移植性原则

方案不仅解决 F4↔G4 双 target 兼容，还需为未来移植到其他 STM32 系列（F1/F3/L4/H7 等）降低成本：

- **mcu_compat.h 覆盖全系列**：F1/F3/F4/G4/L4/H7 一次性定义完整映射，新增系列只需加一行
- **Flash 驱动按"擦除/编程类型"分组**：G4 和 L4 的 Flash 接口一致（页擦除 + doubleword 编程），L4 直接复用 `drv_flash_g4.c`，无需新建文件
- **所有 MCU 条件分支统一用 `JM_MCU_*`**：禁止混用 `STM32F4`/`STM32G4` 系列宏和 `STM32F1`/`STM32F3` 系列宏
- **Flash 几何参数运行期自适应**：`drv_flash_g4.h:35-42` 已用 HAL 运行期值（FLASHSIZE 寄存器），自动适配各容量后缀；`drv_flash_f4.c` 扇区表当前硬编码 F405RG，未来移植到其他 F4 型号时需参数化（本方案标注为未来改进）
- **添加新系列的步骤标准化**：见第九章"可移植性设计指南"

---

## 三、Include 链路图（改造后）

```
Keil -D 预定义: STM32F405xx (MKS) / STM32G474xx (SFOC) / STM32F103xx (未来 F1) / ...
       │
       ├─→ mcu_compat.h (新增, 定义 JM_MCU_F1/F3/F4/G4/L4/H7 + JM_PERIPH_* 外设分组)
       │       │
       │       └─→ drv_config.h (按 Flash 擦除/编程类型派生 USE_FLASH_*_DRIVER)
       │               │
       │               ├─→ drv_usart.h → drv_usart.c (用 main.h 自动适配 HAL 头)
       │               ├─→ drv_adc.h → drv_adc.c (用 JM_PERIPH_ADC_CALIB_* 选校准分支)
       │               ├─→ drv_can.h → drv_can.c (用 JM_PERIPH_CAN_* 选 CAN/FDCAN 句柄)
       │               └─→ dev_flash.h → dev_flash.c (用 USE_FLASH_F4_DRIVER 双路径)
       │
       └─→ Board/<board>/Inc/main.h (各板 CubeMX 生成)
               │
               ├─→ stm32f4xx_hal.h (MKS) / stm32g4xx_hal.h (SFOC) / stm32f1xx_hal.h (未来 F1)
               │       │
               │       └─→ HAL 类型定义 (UART_HandleTypeDef / ADC_HandleTypeDef / ...)
               │
               └─→ 被 7 个 User/ 文件直接 #include (见 1.2 节)

Board/<board>/Config/dev_config_board.h (板级使能集 + 参数覆盖)
       │
       └─→ board_select.h (三选一互斥, 默认 fallback SFOC)
               │
               └─→ dev_config.h → USE_DEV_* 设备使能宏
                       │
                       └─→ dev_*.c/h (条件编译保护各设备模块)
```

---

## 四、文件结构

本方案涉及的文件改动清单（按依赖顺序）：

| 文件 | 改动类型 | 责任 | 任务 |
|---|---|---|---|
| `User/Config/mcu_compat.h` | **新建** | MCU 宏统一判定头（单一入口） | Task 1 |
| `User/Driver/drv_config.h` | 修改 | `FLASH_MCU` 自动派生 + include mcu_compat.h | Task 1 |
| `User/Driver/drv_usart.c` | 修改 | HAL 头改用 main.h + ErrorCallback 注释增强 | Task 2 |
| `User/Driver/drv_adc.c` | 修改 | 用 `JM_PERIPH_ADC_CALIB_*` 统一校准分支 + include mcu_compat.h | Task 3 |
| `User/Driver/drv_can.c` | 修改 | 统一为 `JM_PERIPH_CAN_*` 宏 + include mcu_compat.h | Task 4 |
| `User/AppEntry/user_interface.c` | 修改 | `tim.h` 依赖收敛到条件编译块 | Task 5 |

### 4.1 新建文件说明

**`User/Config/mcu_compat.h`**（MCU 兼容判定头）：集中定义 MCU 系列判定宏 `JM_MCU_F1`/`F3`/`F4`/`G4`/`L4`/`H7`，以及外设接口分组宏 `JM_PERIPH_CAN_CLASSIC`/`CAN_FD`、`JM_PERIPH_ADC_CALIB_DUAL_PARAM`/`SINGLE_PARAM`。放在 Config 目录与 `board_select.h`（板级选择）/ `system_config.h` 同层，符合"配置层"定位。所有 User/ 文件统一引用此头。避免每个文件各自 `#if defined(STM32F405xx)` 导致的命名漂移和遗漏。这是本方案唯一的抽象新增点，符合"统一入口"原则。新增 MCU 系列时只需在 mcu_compat.h 加一行 `#elif` 分支 + 加入对应外设分组即可，User/ 业务层无需修改。

---

## 五、Task 详细步骤

### Task 1: 创建 MCU 兼容判定头 + 改造 drv_config.h 自动派生

**Files:**
- Create: `User/Config/mcu_compat.h`
- Modify: `User/Driver/drv_config.h:6`（顶部加 include）
- Modify: `User/Driver/drv_config.h:97-134`（删除 FLASH_MCU 块，改为自动派生）

**依赖**：无（被 Task 3/4 依赖，必须先执行）

> **说明**：mcu_compat.h 放在 `User/Config/` 目录，与 `board_select.h`（板级选择）/ `system_config.h`（系统配置）同层，符合"配置层"定位。Keil 工程需确保 `User/Config/` 在 include path 中（已有，因 board_select.h 等已在同目录）。

- [ ] **Step 1: 创建 `User/Config/mcu_compat.h`**

```c
#ifndef __MCU_COMPAT_H__
#define __MCU_COMPAT_H__

/*
 * MCU 兼容判定头（单一入口）
 * 以 Keil 工程预定义的 HAL 标准宏（-D 编译器选项）为唯一事实源,
 * 统一 User/ 下的 MCU 条件编译。
 *
 * 所有需要按 MCU 切换的代码 #include 本头, 使用 JM_MCU_F4 / JM_MCU_G4 等判定,
 * 禁止直接使用 STM32F4 / STM32G4 系列宏（依赖 HAL 头包含顺序, 不可靠）
 * 或 STM32F405 / STM32G474 无后缀宏（不匹配 HAL 标准）。
 *
 * 支持系列（Flash 驱动类型）:
 *   - JM_MCU_F1:  STM32F103xx 等           → Flash: 页擦除+halfword (需 drv_flash_f1.c, 预留)
 *   - JM_MCU_F3:  STM32F302xx/F303xx 等    → Flash: 页擦除+halfword (需 drv_flash_f3.c, 预留)
 *   - JM_MCU_F4:  STM32F405xx/F407xx 等    → Flash: 扇区擦除+word   (drv_flash_f4.c)
 *   - JM_MCU_G4:  STM32G474xx/G473xx 等    → Flash: 页擦除+doubleword (drv_flash_g4.c)
 *   - JM_MCU_L4:  STM32L431xx/L476xx 等    → Flash: 页擦除+doubleword (复用 drv_flash_g4.c, 接口一致)
 *   - JM_MCU_H7:  STM32H745xx/H747xx 等    → Flash: 扇区擦除+256bit (需 drv_flash_h7.c, 预留)
 *
 * 添加新系列: 在下方加一行 #elif 分支, 在 drv_config.h 选对应 Flash 驱动即可。
 */

/*=== MCU 系列判定 ===*/
#if defined(STM32F103xB) || defined(STM32F103xC) || defined(STM32F103xD) || defined(STM32F103xE) || \
    defined(STM32F102xB) || defined(STM32F102xC)
#define JM_MCU_F1
#elif defined(STM32F302xC) || defined(STM32F302xE) || \
      defined(STM32F303xC) || defined(STM32F303xE) || defined(STM32F334x8)
#define JM_MCU_F3
#elif defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F415xx) || defined(STM32F417xx) || \
      defined(STM32F427xx) || defined(STM32F429xx) || defined(STM32F437xx) || defined(STM32F439xx) || \
      defined(STM32F411xE) || defined(STM32F401xC) || defined(STM32F401xE)
#define JM_MCU_F4
#elif defined(STM32G474xx) || defined(STM32G473xx) || defined(STM32G484xx) || \
      defined(STM32G431xx) || defined(STM32G441xx) || defined(STM32G491xx)
#define JM_MCU_G4
#elif defined(STM32L431xx) || defined(STM32L432xx) || defined(STM32L433xx) || \
      defined(STM32L452xx) || defined(STM32L476xx) || defined(STM32L475xx)
#define JM_MCU_L4
#elif defined(STM32H745xx) || defined(STM32H747xx) || defined(STM32H743xx) || defined(STM32H750xx)
#define JM_MCU_H7
#endif

#if !defined(JM_MCU_F1) && !defined(JM_MCU_F3) && !defined(JM_MCU_F4) && \
    !defined(JM_MCU_G4) && !defined(JM_MCU_L4) && !defined(JM_MCU_H7)
#error "Unknown MCU. Define STM32F405xx / STM32G474xx / STM32F103xx etc. in Keil target options."
#endif

/*=== 外设接口分组（跨系列共性, 供 drv_can/drv_adc 等使用） ===*/

/* CAN vs FDCAN: F1/F3/F4 用经典 CAN, G4/L4/H7 用 FDCAN */
#if defined(JM_MCU_F1) || defined(JM_MCU_F3) || defined(JM_MCU_F4)
#define JM_PERIPH_CAN_CLASSIC
#elif defined(JM_MCU_G4) || defined(JM_MCU_L4) || defined(JM_MCU_H7)
#define JM_PERIPH_CAN_FD
#endif

/* ADC 校准接口: G4/L4/H7 双参, F1/F3/F4 单参 */
#if defined(JM_MCU_G4) || defined(JM_MCU_L4) || defined(JM_MCU_H7)
#define JM_PERIPH_ADC_CALIB_DUAL_PARAM
#elif defined(JM_MCU_F1) || defined(JM_MCU_F3) || defined(JM_MCU_F4)
#define JM_PERIPH_ADC_CALIB_SINGLE_PARAM
#endif

#endif /* __MCU_COMPAT_H__ */
```

- [ ] **Step 2: 在 `drv_config.h` 第 6 行后加入 mcu_compat.h 的 include**

`drv_config.h` 当前第 4-6 行：

```c
#define _DRV_CONFIG_H_

#include <stdint.h>
```

修改为（第 6 行后插入一行）：

```c
#define _DRV_CONFIG_H_

#include <stdint.h>
#include "mcu_compat.h"
```

> **注意**：mcu_compat.h 不依赖任何其他头文件，无循环依赖风险。此处 include 一次后，所有 `#include "drv_config.h"` 的文件（drv_usart.h / drv_adc.h / drv_can.h / drv_flash_f4.h / drv_flash_g4.h 等）都间接获得 `JM_MCU_*` 宏。

- [ ] **Step 3: 删除 `drv_config.h` 第 97-134 行的 FLASH_MCU 块，替换为自动派生**

`drv_config.h` 当前第 97-136 行：

```c
// <o.0..3>FLASH MCU
//  <i> Default:STM32G474
//  <1=>STM32F103
//  <2=>STM32F429
//  <3=>STM32F405
//  <4=>STM32G431
//  <5=>STM32G491
//  <6=>STM32G474
#define FLASH_MCU 0x03

#if (FLASH_MCU == 0x01)
#define STM32F103
#elif (FLASH_MCU == 0x02)
#define STM32F429
#ifndef USE_FLASH_F4_DRIVER
#define USE_FLASH_F4_DRIVER
#endif
#elif (FLASH_MCU == 0x03)
#define STM32F405
#ifndef USE_FLASH_F4_DRIVER
#define USE_FLASH_F4_DRIVER
#endif
#elif (FLASH_MCU == 0x04)
#define STM32G431
#ifndef USE_FLASH_G4_DRIVER
#define USE_FLASH_G4_DRIVER
#endif
#elif (FLASH_MCU == 0x05)
#define STM32G491
#ifndef USE_FLASH_G4_DRIVER
#define USE_FLASH_G4_DRIVER
#endif
#elif (FLASH_MCU == 0x06)
#define STM32G474
#ifndef USE_FLASH_G4_DRIVER
#define USE_FLASH_G4_DRIVER
#endif
#endif

//***<<< end of configuration section >>>***
```

替换为（保留 Keil Configuration Wizard 标记）：

```c
// <h>FLASH MCU (auto-derived from HAL macro, do NOT edit manually)
//  <i> 由 Keil 预定义的 STM32F405xx / STM32G474xx 等自动派生 Flash 驱动选择
//  <i> 切换工程时无需修改此处, 由 board_select.h -> dev_config_board.h 链路保证
//  <i> Flash 驱动按"擦除/编程类型"分组:
//  <i>   - F4: 扇区擦除+word 编程         → drv_flash_f4.c
//  <i>   - G4/L4: 页擦除+doubleword 编程  → drv_flash_g4.c (L4 接口与 G4 一致, 复用)
//  <i>   - F1/F3: 页擦除+halfword 编程    → drv_flash_f1.c (预留, 需新建)
//  <i>   - H7: 扇区擦除+256bit 编程       → drv_flash_h7.c (预留, 需新建)

/* mcu_compat.h 已在文件顶部 include, 此处直接用 JM_MCU_* 派生 */
#if defined(JM_MCU_F4)
#ifndef USE_FLASH_F4_DRIVER
#define USE_FLASH_F4_DRIVER
#endif
#elif defined(JM_MCU_G4) || defined(JM_MCU_L4)
/* L4 的 Flash 接口与 G4 一致(页擦除 + 64-bit doubleword 编程), 直接复用 drv_flash_g4.c
 * drv_flash_g4.h:35-42 的 Flash 几何参数来自 HAL 运行期值, 自动适配各容量后缀 */
#ifndef USE_FLASH_G4_DRIVER
#define USE_FLASH_G4_DRIVER
#endif
#elif defined(JM_MCU_F1) || defined(JM_MCU_F3)
/* F1/F3 Flash: 页擦除 + 16-bit halfword 编程, 接口与 G4 不同, 需新建 drv_flash_f1.c */
#ifndef USE_FLASH_F1_DRIVER
#define USE_FLASH_F1_DRIVER
#endif
#elif defined(JM_MCU_H7)
/* H7 Flash: 扇区擦除 + 256-bit 编程, 双 Bank, 需新建 drv_flash_h7.c */
#ifndef USE_FLASH_H7_DRIVER
#define USE_FLASH_H7_DRIVER
#endif
#endif
// </h>

//***<<< end of configuration section >>>***
```

> **验证**：全局搜索确认 `defined(STM32F405)` / `defined(STM32G474)`（无 xx 后缀）零引用（已确认），删除派生宏安全。`USE_FLASH_F4_DRIVER` / `USE_FLASH_G4_DRIVER` 仍被派生，下游 `dev_flash.c` / `drv_flash_f4.c` / `drv_flash_g4.c` 行为不变。F1/F3/H7 的 `USE_FLASH_F1_DRIVER` / `USE_FLASH_H7_DRIVER` 为预留，当前无对应驱动文件，未来移植时新建。

- [ ] **Step 4: MKS(ODrive/F4) 编译验证**

打开 `Board/ODriveMKS/MDK-ARM/OdriveMini.uvprojx`，Project → Rebuild all target files。
预期：0 error。`USE_FLASH_F4_DRIVER` 被定义，`drv_flash_f4.c` 参与编译，`drv_flash_g4.c` 不参与。

- [ ] **Step 5: SFOC(G4) 编译验证**

打开 `Board/SFOC/MDK-ARM/sfoc.uvprojx`，Rebuild。
预期：0 error。`USE_FLASH_G4_DRIVER` 被定义，`drv_flash_g4.c` 参与编译，`drv_flash_f4.c` 不参与。

- [ ] **Step 6: 调试器确认派生宏**

MKS 板：调试器 Watch 观察 `JM_MCU_F4` 是否定义（预处理宏无法直接 Watch，改为检查编译输出中 `drv_flash_f4.o` 是否生成）。

---

### Task 2: 修复 drv_usart.c 硬编码 HAL 头（P0-1）+ 注释增强

**Files:**
- Modify: `User/Driver/drv_usart.c:23`（HAL 头改为 main.h）
- Modify: `User/Driver/drv_usart.c:434-442`（注释增强）

**依赖**：无（不依赖 Task 1 的 mcu_compat.h，可与 Task 1 并行）

**设计决策——为什么用 `main.h` 而非 `mcu_compat.h` 条件包含**：

经验证 HAL 头包含链路，`main.h` 是更优方案：

```
各板 main.h:30 → #include "stm32f4xx_hal.h" (ODrive) / "stm32g4xx_hal.h" (SFOC)
       ↑
dma.h:29 / usart.h:29 / adc.h:29 都 #include "main.h" (CubeMX 标准模板)
       ↑
drv_usart.c:21 #include "dma.h" 已间接包含 main.h → HAL 头
```

**关键事实**：
1. drv_usart.c 第 21 行 `#include "dma.h"` → dma.h:29 `#include "main.h"` → main.h:30 HAL 头，**第 23 行的 `#include "stm32f4xx_hal.h"` 本就完全冗余**
2. Grep 确认 User/Driver/ 下**仅 drv_usart.c 一处**显式包含 HAL 头；drv_adc.c / drv_spi.c / drv_tim.c / drv_gpio.c 都不显式包含，通过各自外设头（adc.h / spi.h / tim.h / gpio.h）间接获得
3. 用 `main.h` 替代后，与 drv_can.c:19 `#include "main.h"` 风格一致，且各板 main.h 自动适配对应 HAL 头，无需 MCU 宏判断

**与 mcu_compat.h 方案的对比**：

| 方案 | 改动量 | 依赖 | 风格一致性 |
|---|---|---|---|
| mcu_compat.h 条件包含 | 6 行（include + #if/#elif/#endif） | 依赖 Task 1 | 与其他驱动不一致（其他驱动不显式含 HAL 头） |
| **main.h 替代（本方案）** | **1 行**（改 1 个 include） | **无依赖** | **与 drv_can.c:19 一致** |

> **注意**：`main.h` 方案仅适用于"需要 HAL 类型定义"的场景（HAL 头包含类）。对于"需要按 MCU 切换逻辑"的场景（drv_adc.c 校准分支、drv_can.c 句柄类型、drv_config.h FLASH_MCU 派生），仍需 mcu_compat.h 的 `JM_MCU_*` 宏，因为 main.h 无法判断 MCU 型号。

- [ ] **Step 1: 修改第 23 行，将硬编码 HAL 头替换为 main.h**

`drv_usart.c` 当前第 20-25 行：

```c
#ifdef USE_USART_DRIVER
#include "dma.h"
#include "usart.h"
#include "stm32f4xx_hal.h"
#include <stdlib.h>
#include <string.h>
```

修改为：

```c
#ifdef USE_USART_DRIVER
#include "dma.h"
#include "usart.h"
#include "main.h"
#include <stdlib.h>
#include <string.h>
```

> **说明**：`main.h` 由各板 CubeMX 生成，第 30 行自动包含对应板的 HAL 头（ODrive→f4 HAL，SFOC→g4 HAL）。虽然第 21 行 `dma.h` 已间接包含 main.h，但显式 `#include "main.h"` 是防御性声明，与 drv_can.c:19 风格一致，避免依赖"dma.h 一定包含 main.h"的隐式链路。

- [ ] **Step 2: 增强 HAL_UART_ErrorCallback 的注释（第 434-442 行）**

`drv_usart.c` 当前第 434-442 行：

```c
/**
 * @brief       UART 错误回调(覆盖 HAL weak 实现)
 * @note        F4 HAL 在 DMA 模式下遇到 ORE/NE/FE 错误时, 会调用 UART_EndRxTransfer
 *              永久禁用 DMAR 位并中止 DMA RX stream, 导致接收彻底瘫痪。
 *              G4 HAL 的 __HAL_UART_CLEAR_IDLEFLAG 写 ICR 不读 DR, 错误概率极低;
 *              F4 HAL 该宏读 SR+DR, 与 DMA 竞争易触发 ORE。
 *              此回调在错误发生后重启 DMA 接收, 恢复通信。
 *              (DMA 此时已被 HAL 停止, 读 DR 不再与 DMA 竞争, 安全)
 */
```

替换为：

```c
/**
 * @brief       UART 错误回调(覆盖 HAL weak 实现)
 * @note        本回调针对 F4 HAL DMA bug 修复:
 *              F4 HAL 在 DMA 模式下遇到 ORE/NE/FE 错误时会调用 UART_EndRxTransfer
 *              永久禁用 DMAR 位并中止 DMA RX stream, 导致接收彻底瘫痪。
 *              G4 HAL 的 __HAL_UART_CLEAR_IDLEFLAG 写 ICR 不读 DR, 错误概率极低,
 *              本回调在 G4 上虽会执行但属冗余保护, 不影响功能。
 *              (DMA 此时已被 HAL 停止, 读 DR 不再与 DMA 竞争, 安全)
 * @sa          项目记忆 2026-07-15: F4 HAL __HAL_UART_CLEAR_IDLEFLAG 读 SR+DR 引发 ORE
 * @sa          drv_usart.c 第 23 行: HAL 头由 main.h 自动适配各板
 */
```

- [ ] **Step 3: MKS(F4) 编译验证**

Rebuild ODrive target。
预期：0 error。`HAL_UART_ErrorCallback` 正常链接，UART4 接收在 DMA 错误后可恢复。

- [ ] **Step 4: SFOC(G4) 编译验证**

Rebuild SFOC target。
预期：0 error。第 23 行 `main.h` → `stm32g4xx_hal.h`，不再因找不到 F4 HAL 头失败。

- [ ] **Step 5: 调试器验证 UART 接收（MKS 板）**

烧录 MKS 固件，上位机发送 0xEA 命令，调试器观察 `g_dev_commun_uart` 收到完整帧。
预期：UART4 RX 正常，无 ORE 永久禁用。

---

### Task 3: 补齐 drv_adc.c 的 F4 ADC 校准分支（P1-3）

**Files:**
- Modify: `User/Driver/drv_adc.c:18-21`（include 区加 mcu_compat.h）
- Modify: `User/Driver/drv_adc.c:128-135`（补 F4 分支）

**依赖**：Task 1（mcu_compat.h）

- [ ] **Step 1: 在 drv_adc.c include 区加入 mcu_compat.h**

`drv_adc.c` 当前第 18-21 行：

```c
#include "drv_adc.h"

#ifdef USE_ADC_DRIVER
#include "adc.h"
```

修改为：

```c
#include "drv_adc.h"

#ifdef USE_ADC_DRIVER
#include "mcu_compat.h"
#include "adc.h"
```

> **说明**：`drv_adc.h:21` 已 `#include "drv_config.h"`（含 mcu_compat.h），此处显式包含是冗余但安全的。

- [ ] **Step 2: 修改第 128-135 行，用 JM_PERIPH_ADC_CALIB_* 宏统一**

`drv_adc.c` 当前第 128-135 行：

```c
	/* G4/L4/H7系列校准接口为双参(句柄+单端/差分)，签名跨型号一致 */
#if defined(STM32G4) || defined(STM32L4) || defined(STM32H7)
	uint32_t mode = get_adc_mode(DRV_ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(handle, mode);
#elif defined(STM32F1) || defined(STM32F3)
	/* F1/F3系列校准接口为单参 */
	HAL_ADCEx_Calibration_Start(handle);
#endif
```

替换为（用 mcu_compat.h 的外设分组宏，不再逐系列列举）：

```c
	/* ADC 校准接口按 MCU 系列分组(mcu_compat.h 已定义):
	 * - JM_PERIPH_ADC_CALIB_DUAL_PARAM:   G4/L4/H7, 双参(句柄+单端/差分模式)
	 * - JM_PERIPH_ADC_CALIB_SINGLE_PARAM: F1/F3/F4, 单参(句柄) */
#if defined(JM_PERIPH_ADC_CALIB_DUAL_PARAM)
	uint32_t mode = get_adc_mode(DRV_ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(handle, mode);
#elif defined(JM_PERIPH_ADC_CALIB_SINGLE_PARAM)
	HAL_ADCEx_Calibration_Start(handle);
#endif
```

> **注意**：
> - `get_adc_mode` 函数（第 90-114 行）内部用 `#if defined(ADC_SINGLE_ENDED)` 判断，F4 下 `ADC_SINGLE_ENDED` 宏不存在则 `mode = 0`。但 `get_adc_mode` 仅在双参分支内被调用，F4 下不调用，无影响。
> - 用 `JM_PERIPH_ADC_CALIB_*` 宏后，**新增 MCU 系列时无需改 drv_adc.c**，只需在 mcu_compat.h 把新系列加入对应分组即可。

- [ ] **Step 3: MKS(F4) 编译验证**

Rebuild ODrive target。
预期：0 error。F4 走单参分支 `HAL_ADCEx_Calibration_Start(handle)`，链接成功。

- [ ] **Step 4: 调试器验证 ADC 校准生效（MKS 板）**

烧录 MKS 固件，上电后施加已知电压（如 3.3V 分压），调试器观察：
- `phase_current.offset.a` / `phase_current.offset.b` 接近 2048
- `dev_power_monitor` 的 vbus 读数与万用表一致

预期：ADC 已校准，读数线性。

- [ ] **Step 5: SFOC(G4) 编译验证**

Rebuild SFOC target。
预期：0 error。G4 走双参分支 `HAL_ADCEx_Calibration_Start(handle, mode)`，行为不变。

---

### Task 4: 统一 drv_can.c 的 MCU 宏为 JM_PERIPH_CAN_*（P2-3）

**Files:**
- Modify: `User/Driver/drv_can.c:18-20`（include 区加 mcu_compat.h）
- Modify: `User/Driver/drv_can.c:25,28,37,42,57,106,129,160,180,213,225,254,273`（宏替换）

**依赖**：Task 1（mcu_compat.h）

- [ ] **Step 1: 在 drv_can.c include 区加入 mcu_compat.h**

`drv_can.c` 当前第 18-20 行：

```c
#include "drv_can.h"
#include "main.h"
#include <string.h>
```

修改为：

```c
#include "drv_can.h"
#include "main.h"
#include "mcu_compat.h"
#include <string.h>
```

> **说明**：`drv_can.h:24` 已 `#include "drv_config.h"`（含 mcu_compat.h），此处显式包含是冗余但安全的。`main.h`（第 19 行）已间接包含 HAL 头，但 `JM_MCU_*` 基于 Keil `-D` 预定义宏，不依赖 main.h。

- [ ] **Step 2: 全局替换 MCU 宏（用 JM_PERIPH_CAN_* 外设分组宏）**

对 `drv_can.c` 执行以下替换（使用 `replace_all` 语义）：

**模式 A**（第 25, 37, 106, 160, 213, 254 行 — 经典 CAN 分支）：

```c
#if defined(STM32F4)
```
→
```c
#if defined(JM_PERIPH_CAN_CLASSIC)
```

> 替换后 F1/F3/F4 自动走此分支（经典 CAN `CAN_HandleTypeDef`），无需逐系列列举。

**模式 B**（第 28, 42, 129, 180, 225, 273 行 — FDCAN 分支）：

```c
#elif defined(STM32G4) || defined(STM32H7)
```
→
```c
#elif defined(JM_PERIPH_CAN_FD)
```

> 替换后 G4/L4/H7 自动走此分支（FDCAN `FDCAN_HandleTypeDef`）。

**模式 C**（第 57 行 — FDCAN 专属逻辑）：

```c
#if defined(STM32G4) || defined(STM32H7)
```
→
```c
#if defined(JM_PERIPH_CAN_FD)
```

> **验证**：替换后共 13 处，逐行核对行号 25, 28, 37, 42, 57, 106, 129, 160, 180, 213, 225, 254, 273。
>
> **可移植性优势**：用 `JM_PERIPH_CAN_*` 后，**新增 MCU 系列时无需改 drv_can.c**，只需在 mcu_compat.h 把新系列加入对应分组即可（如新增 STM32F0 也用经典 CAN，加到 `JM_PERIPH_CAN_CLASSIC` 分组即可）。

- [ ] **Step 3: 确认 CAN 驱动当前未启用**

读取 `drv_config.h` 第 58 行确认 `USE_CAN_DRIVER` 仍为注释状态（`//#define USE_CAN_DRIVER`）。此 Task 为未来启用 CAN 驱动做兼容性准备，当前 drv_can.c 不参与编译，不影响构建。

- [ ] **Step 4: MKS(F4) 编译验证**

Rebuild ODrive target。
预期：0 error（CAN 驱动未启用，drv_can.c 不参与编译，仅语法校验）。

- [ ] **Step 5: SFOC(G4) 编译验证**

Rebuild SFOC target。
预期：0 error。

- [ ] **Step 6: （可选）临时启用 USE_CAN_DRIVER 验证宏正确性**

在 `drv_config.h` 临时取消第 58 行 `#define USE_CAN_DRIVER` 注释，Rebuild 两个 target：
- MKS(F4)：确认走 `CAN_HandleTypeDef hcan1/hcan2` 分支
- SFOC(G4)：确认走 `FDCAN_HandleTypeDef hfdcan1/hfdcan2` 分支

验证后恢复注释。

---

### Task 5: 收敛 user_interface.c 对 tim.h 的直接依赖（P2-2）

**Files:**
- Modify: `User/AppEntry/user_interface.c:32`（tim.h 条件包含）
- Modify: `User/AppEntry/user_interface.c:79-82`（htim2/htim5 调用已门控，无需改）

**依赖**：无（独立于 Task 1-4）

- [ ] **Step 1: 确认 MOTOR_LOOP_ENABLE_DEV_DRIVER 当前值**

读取 `User/MotorControl/CascadeControl/motor_loop_config.h` 第 41-42 行：

```c
#ifndef MOTOR_LOOP_ENABLE_DEV_DRIVER
#define MOTOR_LOOP_ENABLE_DEV_DRIVER 1u
```

当前为 `1u`（真实电机模式，MKS 板），`htim2`/`htim5` 不被使用（第 79-82 行在 `#if (MOTOR_LOOP_ENABLE_DEV_DRIVER == 0u)` 内）。

- [ ] **Step 2: 将 tim.h 的 include 收敛到条件编译块内**

`user_interface.c` 当前第 32 行：

```c
#include "tim.h" // TODO: 避免直接依赖具体外设头，改为抽象接口（如 timer.h），或通过 control_irq.c 传入时钟频率等参数实现解耦
```

替换为：

```c
#if (MOTOR_LOOP_ENABLE_DEV_DRIVER == 0u)
/* 虚拟电机模式: 依赖 htim2/htim5 触发周期中断
 * (真实电机模式 MOTOR_LOOP_ENABLE_DEV_DRIVER==1 由 ADC 注入中断驱动, 不需要 tim.h) */
#include "tim.h"
#endif
```

- [ ] **Step 3: 确认第 79-82 行和 94-104 行已在条件编译内**

读取 `user_interface.c` 第 79-82 行：

```c
#if (MOTOR_LOOP_ENABLE_DEV_DRIVER == 0u)
	HAL_TIM_Base_Start_IT(&htim2); /* TODO:启动定时器更新中断，进入 user_control 调周期执行 */
	HAL_TIM_Base_Start_IT(&htim5); /* TODO:启动定时器更新中断，进入 motor_virtual_loop 调周期执行 */
#endif
```

第 94-104 行（`motor_virtual_loop`）：

```c
#if (MOTOR_LOOP_ENABLE_DEV_DRIVER == 0u)
	dev_dwt_counter_stop(...);
	...
	motor_loop_isr();
	...
	dev_dwt_counter_stop(...);
#endif
```

确认：`htim2`/`htim5` 的使用已被 `#if (MOTOR_LOOP_ENABLE_DEV_DRIVER == 0u)` 门控，与 Step 2 的 tim.h include 门控一致。✅

- [ ] **Step 4: MKS(F4, 真实电机模式) 编译验证**

确认 `MOTOR_LOOP_ENABLE_DEV_DRIVER == 1u`，Rebuild ODrive target。
预期：0 error。`tim.h` 不被包含（`#if == 0u` 为假），`htim2`/`htim5` 调用被排除。

- [ ] **Step 5: SFOC(G4) 编译验证**

Rebuild SFOC target。
预期：0 error。

- [ ] **Step 6: （可选）虚拟电机模式回归验证**

临时在 `motor_loop_config.h` 设 `#define MOTOR_LOOP_ENABLE_DEV_DRIVER 0u`，Rebuild，确认 `tim.h` 被包含且 `htim2`/`htim5` 调用存在。验证后恢复为 `1u`。

---

### Task 6: 全量双向编译验证 + 兼容性矩阵确认

**Files:**
- 无文件修改，仅验证

**依赖**：Task 1-5 全部完成

- [ ] **Step 1: MKS(ODrive/F4) 全量 Rebuild**

打开 `Board/ODriveMKS/MDK-ARM/OdriveMini.uvprojx`，Project → Rebuild all target files。
预期输出：

```
".\Objects\OdriveMini\OdriveMini.axf" - 0 Error(s), 0 Warning(s).
```

若 warning 仅来自 CubeMX 生成的 `Board/ODriveMKS/Src/*.c`（外设初始化），可忽略；User/ 下 0 warning。

- [ ] **Step 2: SFOC(G4) 全量 Rebuild**

打开 `Board/SFOC/MDK-ARM/sfoc.uvprojx`，Rebuild。
预期：

```
".\Objects\sfoc\sfoc.axf" - 0 Error(s), 0 Warning(s).
```

- [ ] **Step 3: 确认 DRV8301 模块在 SFOC 下不参与编译**

SFOC 编译输出列表中搜索 `dev_drv8301.o`：
预期：**不存在**（`USE_DEV_DRV8301` 未定义，`dev_drv8301.c` 编译为空）。

- [ ] **Step 4: 确认 Flash 驱动分支正确**

| Target | 预期参与编译 | 预期不参与 |
|---|---|---|
| MKS(F4) | `drv_flash_f4.o` | `drv_flash_g4.o` |
| SFOC(G4) | `drv_flash_g4.o` | `drv_flash_f4.o` |

在各自 `MDK-ARM/Objects/<target>/` 目录下确认 `.o` 文件存在性。

- [ ] **Step 5: 确认 Flash 地址宏正确覆盖**

| Target | `MOTORINFO_FLASH_START_ADDR` | 来源 |
|---|---|---|
| MKS(F4) | `0x080E0000U` | `Board/ODriveMKS/Config/dev_config_board.h:47` 覆盖 |
| SFOC(G4) | `0x0804F000U` | `motor_info_storage.h:68-69` 默认值 |

调试器观察 `g_motor_info_storage.dev_flash.start_addr` 应为上表对应值。

- [ ] **Step 6: 确认 EN_GATE 引脚正确切换**

| Target | EN_GATE 引脚 | 来源 |
|---|---|---|
| MKS(F4) | PB12 | `dev_motor.c:44` `JM_BOARD_ODRIVE` 分支 |
| SFOC(G4) | PB2 | `dev_motor.c:49` else 分支 |

- [ ] **Step 7: 确认 ADC 校准分支正确**

| Target | 预期分支 | HAL 接口 |
|---|---|---|
| MKS(F4) | `#elif defined(JM_PERIPH_ADC_CALIB_SINGLE_PARAM)` | `HAL_ADCEx_Calibration_Start(handle)` 单参 |
| SFOC(G4) | `#if defined(JM_PERIPH_ADC_CALIB_DUAL_PARAM)` | `HAL_ADCEx_Calibration_Start(handle, mode)` 双参 |

- [ ] **Step 8: 确认 UART HAL 头正确包含**

| Target | drv_usart.c:23 包含的 HAL 头 |
|---|---|
| MKS(F4) | `stm32f4xx_hal.h` |
| SFOC(G4) | `stm32g4xx_hal.h` |

- [ ] **Step 9: 生成兼容性矩阵记录**

```
双向编译验证通过:
- MKS(STM32F405xx):
  drv_flash_f4.o ✓ / dev_drv8301.o ✓ / EN_GATE=PB12 ✓
  Flash@0x080E0000 ✓ / ADC单参校准 ✓ / UART=F4 HAL ✓
- SFOC(STM32G474xx):
  drv_flash_g4.o ✓ / dev_drv8301.o ✗(不参与) / EN_GATE=PB2 ✓
  Flash@0x0804F000 ✓ / ADC双参校准 ✓ / UART=G4 HAL ✓
```

---

## 六、自检清单（Self-Review）

### 6.1 Spec 覆盖

| 问题编号 | 对应 Task | 状态 | 备注 |
|---|---|---|---|
| P0-1 drv_usart.c 硬编码 HAL 头 | Task 2 | ✅ | 用 `main.h` 替代（HAL 头包含类，不依赖 mcu_compat.h） |
| P1-1 FLASH_MCU 硬编码 | Task 1 Step 3 | ✅ | 改为从 HAL 宏自动派生 |
| P1-2 派生宏无 xx 后缀 | Task 1 Step 3 | ✅ | 删除派生宏，零引用确认安全 |
| P1-3 drv_adc.c 缺 F4 分支 | Task 3 | ✅ | 用 `JM_PERIPH_ADC_CALIB_*` 统一，支持 F1/F3/F4 单参 + G4/L4/H7 双参 |
| P2-1 ErrorCallback G4 冗余 | Task 2 Step 2 | ✅ | 注释增强，不改逻辑 |
| P2-2 user_interface.c tim.h | Task 5 | ✅ | 收敛到 `#if == 0u` 条件块 |
| P2-3 drv_can.c 宏命名 | Task 4 | ✅ | 13 处统一为 `JM_PERIPH_CAN_*`，支持 F1/F3/F4 经典 CAN + G4/L4/H7 FDCAN |
| P3 良好隔离模块 | 不修改 | ✅ | 10 项记录备查 |
| P4 F4 扇区擦除时长 | 本方案不处理 | ⏭️ | 单独议题 |

### 6.2 类型/宏一致性检查

| 宏/类型 | 定义位置 | 引用位置 | 一致性 |
|---|---|---|---|
| `JM_MCU_F1`/`F3`/`F4`/`G4`/`L4`/`H7` | Task 1: `mcu_compat.h` | 全局 MCU 判定（drv_config.h Flash 派生） | ✅ 一致 |
| `JM_PERIPH_CAN_CLASSIC` / `JM_PERIPH_CAN_FD` | Task 1: `mcu_compat.h` | Task 4: drv_can.c（替代逐系列列举） | ✅ 一致 |
| `JM_PERIPH_ADC_CALIB_DUAL_PARAM` / `SINGLE_PARAM` | Task 1: `mcu_compat.h` | Task 3: drv_adc.c（替代逐系列列举） | ✅ 一致 |
| `USE_FLASH_F4_DRIVER` / `USE_FLASH_G4_DRIVER` | Task 1: drv_config.h 派生 | `dev_flash.c:75` / `dev_flash.h:34-40` / `drv_flash_f4.h:19` / `drv_flash_g4.h:25` | ✅ 一致（未改动引用方） |
| `USE_FLASH_F1_DRIVER` / `USE_FLASH_H7_DRIVER` | Task 1: drv_config.h 派生（预留） | 无（未来 drv_flash_f1.c / drv_flash_h7.c） | ✅ 预留 |
| `MOTOR_LOOP_ENABLE_DEV_DRIVER` | `motor_loop_config.h:42`（默认 `1u`） | Task 5: user_interface.c | ✅ 未改动定义方 |
| `USE_DEV_DRV8301` | `Board/ODriveMKS/Config/dev_config_board.h:11` | `user_interface.c:25,47` / `dev_drv8301.c:12` / `dev_drv8301.h:21` | ✅ 未改动（已保护） |
| `JM_BOARD_ODRIVE` | Keil 预定义 | `board_select.h:34` / `dev_motor.c:41` | ✅ 未改动 |
| `HAL_UART_ErrorCallback` | HAL weak 实现 | drv_usart.c:443 覆盖 | ✅ 未改动签名 |

### 6.3 占位符扫描

- [x] 无 TBD / TODO / "implement later"（Task 5 保留原 TODO 的语义但改为条件编译实现）
- [x] 无 "add error handling" 等空泛描述
- [x] 每个代码步骤含完整代码块
- [x] 每个验证步骤含预期输出
- [x] 每个文件改动含精确行号

### 6.4 遗漏检查

| 检查项 | 结果 |
|---|---|
| 是否有其他文件硬编码 HAL 头？ | 已搜索：仅 drv_usart.c:23（P0）、dev_dwt_counter.c:23-27（已保护）、drv_flash_f4.c:13-14（已保护）、drv_flash_g4.c:21-23（已保护） |
| 是否有其他文件用系列宏 STM32F4/G4？ | 已搜索：仅 drv_can.c（13 处，Task 4）和 drv_adc.c（2 处，Task 3） |
| 是否有其他文件用无后缀宏 STM32F405/G474？ | 已搜索：零引用 ✅ |
| 是否有其他文件用 main.h 引入 HAL 依赖？ | 7 个文件，均通过各板 main.h 自动适配 ✅ |
| drv_tim/spi/gpio/i2c 是否有 MCU 依赖？ | 无，TIM/SPI/I2C_HandleTypeDef 跨 F4/G4 类型一致 ✅ |
| control_irq.c 是否有 MCU 依赖？ | 无，仅 include 通用头 ✅ |
| motor_info_storage.c 是否有 MCU 依赖？ | 无，Flash 地址通过板级 #undef+#define 覆盖 ✅ |

---

## 七、执行顺序与依赖

```
Task 1 (mcu_compat.h + drv_config.h)    ← 基础, 无依赖
   ↓ 被依赖
Task 3 (drv_adc.c)                      ← 依赖 mcu_compat.h (MCU 逻辑分支)
Task 4 (drv_can.c)                      ← 依赖 mcu_compat.h (MCU 逻辑分支)
   ↓
Task 2 (drv_usart.c)                    ← 独立! 用 main.h 替代, 不依赖 mcu_compat.h
Task 5 (user_interface.c)               ← 独立, tim.h 条件包含
   ↓
Task 6 (全量双向验证)                    ← 依赖 Task 1-5 全部完成
```

**依赖关系说明**：
- Task 1 是 Task 3/4 的前置（提供 mcu_compat.h 的 JM_MCU_* 宏）
- Task 2 改用 main.h 后**不再依赖 Task 1**，可与 Task 1 并行
- Task 5 独立，随时可执行
- Task 2/3/4/5 之间无依赖，可全部并行
- Task 6 必须最后执行

---

## 八、回滚预案

若任一 Task 导致回归，手动还原对应文件即可（不涉及 git commit）：

1. 还原 Task 1 后，Task 3/4 的 `#include "mcu_compat.h"` 会编译失败（文件不存在），需一并还原 Task 3/4
2. Task 2（main.h 替代）和 Task 5（tim.h 条件包含）无依赖，可独立还原
3. **还原后状态**：
   - `drv_usart.c` 恢复硬编码 `#include "stm32f4xx_hal.h"`（仅 MKS 可编译，SFOC 不可）
   - `drv_config.h` 恢复 `FLASH_MCU 0x03` 手动配置
   - `drv_adc.c` 恢复缺失 F4 分支
   - `drv_can.c` 恢复 `STM32F4`/`STM32G4` 系列宏
   - `user_interface.c` 恢复无条件 `#include "tim.h"`

**回滚验证**：还原后 MKS(F4) target Rebuild，0 error。

---

## 九、验证记录（Task 6 完成后填写）

| 验证项 | MKS(F4) 结果 | SFOC(G4) 结果 | 日期 | 验证人 |
|---|---|---|---|---|
| Rebuild 0 error | ☐ 通过 | ☐ 通过 | | |
| drv_flash_*.o 正确 | ☐ f4 参与 | ☐ g4 参与 | | |
| dev_drv8301.o | ☐ 参与 | ☐ 不参与 | | |
| EN_GATE 引脚 | ☐ PB12 | ☐ PB2 | | |
| Flash 地址 | ☐ 0x080E0000 | ☐ 0x0804F000 | | |
| ADC 校准 | ☐ F4 单参 | ☐ G4 双参 | | |
| UART HAL 头 | ☐ stm32f4xx_hal.h | ☐ stm32g4xx_hal.h | | |
| UART 接收 | ☐ 正常 | ☐ 正常 | | |

---

## 十、可移植性设计指南

本方案从可移植性角度设计了三层抽象，使移植到新 STM32 系列的成本最小化。

### 10.1 三层可移植性架构

```
┌─────────────────────────────────────────────────────────────┐
│ 第 1 层: mcu_compat.h — MCU 系列判定 + 外设接口分组          │
│   JM_MCU_F1/F3/F4/G4/L4/H7  ← 加一行 #elif 即可新增系列     │
│   JM_PERIPH_CAN_CLASSIC/FD   ← 按外设接口共性分组            │
│   JM_PERIPH_ADC_CALIB_*      ← 按校准接口签名分组             │
├─────────────────────────────────────────────────────────────┤
│ 第 2 层: drv_config.h — Flash 驱动选择                      │
│   按"擦除/编程类型"派生 USE_FLASH_*_DRIVER                   │
│   F4→扇区+word, G4/L4→页+doubleword, F1/F3→页+halfword      │
├─────────────────────────────────────────────────────────────┤
│ 第 3 层: drv_flash_f4.c / drv_flash_g4.c — Flash 驱动实现    │
│   接口签名完全对齐(drv_flash_read/write/clear)              │
│   G4 几何参数来自 HAL 运行期值, 自动适配容量后缀             │
└─────────────────────────────────────────────────────────────┘
```

### 10.2 移植到新 STM32 系列的步骤

**场景 A：移植到已覆盖的系列（如换用 STM32L476）**

1. Keil 工程选项预定义 `STM32L476xx`
2. mcu_compat.h 自动判定 `JM_MCU_L4` + `JM_PERIPH_CAN_FD` + `JM_PERIPH_ADC_CALIB_DUAL_PARAM`
3. drv_config.h 自动派生 `USE_FLASH_G4_DRIVER`（L4 复用 G4 Flash 路径）
4. **无需修改任何 User/ 代码**，直接 Rebuild

**场景 B：移植到未覆盖的系列（如 STM32F103）**

1. Keil 工程选项预定义 `STM32F103xC`
2. mcu_compat.h 已覆盖 → `JM_MCU_F1` + `JM_PERIPH_CAN_CLASSIC` + `JM_PERIPH_ADC_CALIB_SINGLE_PARAM`
3. drv_config.h 已派生 `USE_FLASH_F1_DRIVER`（预留）
4. **需新建** `drv_flash_f1.c/h`（页擦除 + halfword 编程），接口对齐 `drv_flash_read/write/clear`
5. Flash 地址等板级参数在 `Board/<new>/Config/dev_config_board.h` 覆盖
6. `drv_can.c` / `drv_adc.c` **无需修改**（已用 `JM_PERIPH_*` 分组宏）

**场景 C：移植到完全新的系列（如 STM32F0）**

1. 在 mcu_compat.h 加一行 `#elif` 分支定义 `JM_MCU_F0`
2. 在 mcu_compat.h 外设分组中加入 F0（F0 用经典 CAN + 单参 ADC 校准）
3. 在 drv_config.h 加 Flash 驱动选择（F0 页擦除 + halfword，复用 `USE_FLASH_F1_DRIVER` 或新建）
4. **User/ 业务层代码无需修改**

### 10.3 各 STM32 系列 Flash 驱动映射

| 系列 | Flash 驱动宏 | 驱动文件 | 擦除类型 | 编程粒度 | 状态 |
|---|---|---|---|---|---|
| F1 | `USE_FLASH_F1_DRIVER` | `drv_flash_f1.c`（预留） | 页擦除 1KB/2KB | 16-bit halfword | 需新建 |
| F3 | `USE_FLASH_F1_DRIVER` | `drv_flash_f1.c`（预留） | 页擦除 2KB | 16-bit halfword | 需新建（与 F1 接口一致） |
| **F4** | `USE_FLASH_F4_DRIVER` | `drv_flash_f4.c` | 扇区擦除 16KB~128KB | 32-bit word | ✅ 已实现 |
| **G4** | `USE_FLASH_G4_DRIVER` | `drv_flash_g4.c` | 页擦除 2KB/4KB | 64-bit doubleword | ✅ 已实现 |
| L4 | `USE_FLASH_G4_DRIVER` | `drv_flash_g4.c`（复用） | 页擦除 2KB | 64-bit doubleword | ✅ 复用 G4 驱动 |
| H7 | `USE_FLASH_H7_DRIVER` | `drv_flash_h7.c`（预留） | 扇区擦除 128KB | 256-bit (32-byte) | 需新建 |

> **G4/L4 复用可行性**：`drv_flash_g4.h:35-42` 明确注释"FLASH 几何参数全部来自 HAL 运行期值，不按型号宏硬编码"，`FLASH_SIZE`/`FLASH_BANK_SIZE`/`FLASH_PAGE_SIZE` 均由 HAL 按 DBANK 选项位和 FLASHSIZE 寄存器给出，自动适配各容量后缀。L4 的 Flash HAL 接口与 G4 完全一致（`FLASH_TYPEERASE_PAGES` + `FLASH_TYPEPROGRAM_DOUBLEWORD`），可直接复用。

### 10.4 外设接口分组映射

| 分组宏 | 包含系列 | 用于 |
|---|---|---|
| `JM_PERIPH_CAN_CLASSIC` | F1, F3, F4 | drv_can.c: 经典 CAN (`CAN_HandleTypeDef`) |
| `JM_PERIPH_CAN_FD` | G4, L4, H7 | drv_can.c: FDCAN (`FDCAN_HandleTypeDef`) |
| `JM_PERIPH_ADC_CALIB_DUAL_PARAM` | G4, L4, H7 | drv_adc.c: `HAL_ADCEx_Calibration_Start(handle, mode)` |
| `JM_PERIPH_ADC_CALIB_SINGLE_PARAM` | F1, F3, F4 | drv_adc.c: `HAL_ADCEx_Calibration_Start(handle)` |

> **设计优势**：用外设分组宏（`JM_PERIPH_*`）而非 MCU 系列宏（`JM_MCU_*`）做外设分支，是因为外设接口的共性跨系列（G4 和 L4 的 ADC 校准签名一致，但 F4 不同）。新增 MCU 系列时，只需在 mcu_compat.h 的分组宏中加入该系列，drv_can.c / drv_adc.c 等驱动文件**无需修改**。

### 10.5 已知可移植性技术债

| 项 | 当前状态 | 影响 | 改进方向 |
|---|---|---|---|
| `drv_flash_f4.c:22-35` 扇区表硬编码 F405RG | 移植到 F407VG/F429ZI 需改扇区表 | 仅影响 F4 系列内不同型号 | 改为从 HAL 运行期值或板级配置传入扇区表 |
| `drv_flash_f4.c:13-14` 硬编码 `stm32f4xx_hal_flash.h` | 已在 `USE_FLASH_F4_DRIVER` 保护内，安全 | 无（G4 下不编译） | 保留现状 |
| `dev_dwt_counter.c:23-27` 用 HAL 标准宏 | 已正确，但未用 `JM_MCU_*` 统一 | 无（功能正常） | 未来统一为 `JM_MCU_*` 保持一致性 |
| `drv_flash_f1.c` / `drv_flash_h7.c` 未创建 | F1/F3/H7 移植时需新建 | 仅影响新系列移植 | 按需创建，接口对齐 drv_flash_read/write/clear |

---

## 十一、未来改进建议（不在本方案范围）

1. **P4：F4 128KB 扇区擦除时长优化**：考虑将 motor_info 存储改用 Sector 3（16KB，若确认代码不占用），或实现"先写后擦"的双扇区轮转，降低单次擦除阻塞时间。需单独评估代码占用。
2. **drv_flash_f4.c 扇区表参数化**：当前硬编码 F405RG 的 12 扇区表，移植到 F407VG/F429ZI 需修改。可改为从板级配置或 HAL 运行期值获取扇区表，提升 F4 系列内可移植性。
3. **dev_dwt_counter.c 统一**：当前已用 HAL 标准宏（`STM32F405xx`/`STM32G474xx`）直接判断，工作正常。未来可统一为 `JM_MCU_*` 保持一致性，但当前不改动（最小改动原则）。
4. **自动化 CI**：引入 GitHub Actions / 本地脚本，在两个 Keil target 上自动 Rebuild，防止兼容性回归。
5. **drv_config.h 的 FLASH_MCU 配置向导清理**：当前注释保留了 FLASH_MCU 的 Configuration Wizard 标记但改为自动派生，未来可清理注释块使其不再显示在 Keil 配置向导中。
