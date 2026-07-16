# JointMotor → ODrive Mini (STM32F405RGT6) 最小迁移实施计划 v3

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将 JointMotor 工程从 STM32G474(SFOC) 最小化迁移到 MKS ODrive Mini V1.0(STM32F405RGT6+DRV8301+AS5047P)，User/ 算法/业务层零改动。

**Architecture:** 板级隔离 + 条件编译。所有硬件差异通过 `Board/ODriveMKS/Config/dev_config_board.{h,inc}` 板级配置表 + 少量 `#ifndef`/`#if` 分支吸收；Flash 底层换 `drv_flash_f4`（单扇区无磨损均衡）；编码器走 AS5047P 适配层；DRV8301 走设备对象模式，init 调用放 main.c。

**Tech Stack:** STM32F405RGT6 / HAL F4 / FreeRTOS / Keil MDK-ARM / C99 / C-OOP 设备对象模式

---

## 〇、已确认决策

| # | 决策项 | 结论 |
|---|---|---|
| 1 | DRV8301 nSCS | **PC13**（与 AS5047P 共享 SPI3，CS 独立） |
| 2 | AS5047P | J4 外接，CS=**PB2**，SPI3（16-bit, CPHA=2EDGE=Mode1） |
| 3 | IC 计算 | `DRV8301_TWO_PHASE_CURRENT` 条件编译 |
| 4 | Flash 驱动选择 | 沿用 `drv_config.h` 的 `FLASH_MCU` 机制（0x03→F4） |
| 5 | Flash 存储策略 | **固定 Flash 尾部，不磨损均衡**。F405RGT6 1MB → Sector 11 (0x080E0000, 128KB) |
| 6 | DRV8301 | 按设备对象模式，init 调用放 main.c（user_init 之前） |

---

## 一、引脚映射（从 ioc 确认）

| 功能 | MCU 引脚 | ioc 标号 | 外设/模式 |
|---|---|---|---|
| AS5047P CS | PB2 | GPIO_6 | GPIO_Output |
| AS5047P SPI | PC10/PC11/PC12 | SPI3_SCK/MISO/MOSI | SPI3 (16-bit, 2.625M, CPHA=2EDGE) |
| DRV8301 nSCS | PC13 | M0_nCS(原) | GPIO_Output |
| DRV8301 SPI | 共享 SPI3 | — | 同上 |
| EN_GATE | PB12 | EN_GATE | GPIO_Output |
| nFAULT | PD2 | nFAULT | GPIO_Input |
| M0 SO1 | PC0 | M0_IB | ADC1 注入 IN10 |
| M0 SO2 | PC1 | M0_IC | ADC1 注入 IN11 |
| VBUS | PA6 | VBUS_S | ADC1 规则 IN6 |
| UART4 | PA0/PA1 | GPIO_1/GPIO_2 | UART4 |
| TIM1 PWM | PA8-10/PB13-15 | M0_AH~CL | TIM1 CH1-3 + CH1-3N |

---

## 二、文件结构

### User/ 修改（8 个，全部小改）

| 文件 | 责任 | 改动 |
|---|---|---|
| `User/Config/board_select.h` | 板级开关 | 加 `JM_BOARD_ODRIVE` 分支 |
| `User/Driver/drv_config.h` | 驱动开关 | `USE_FLASH_G4_DRIVER` 注释掉，从 `FLASH_MCU` 派生 |
| `User/Devices/dev_config.h` | 设备参数 | 关键参数加 `#ifndef` 保护 |
| `User/Devices/dev_flash.h` | Flash 设备 | `#include "drv_flash_g4.h"` 改为条件 include |
| `User/Devices/dev_flash.c` | Flash 磨损均衡 | F4 条件：单扇区无磨损均衡写路径 |
| `User/AppServices/ParamService/motor_info_storage.h` | Flash 地址 | 3 个地址宏加 `#ifndef` 保护 |
| `User/Devices/dev_motor_phase_current.c` | 相电流 | IC 计算加 `DRV8301_TWO_PHASE_CURRENT` 分支 |
| `User/Devices/dev_motor.h` | 电机对象 | 加 `DEV_MOTOR_ENCODER_AS5047` 枚举 |
| `User/Devices/dev_motor.c` | 电机初始化 | 编码器分支 + 使能引脚条件编译 |
| `User/Devices/dev_config.c` | 配置表编译单元 | 加 `#include "dev_as5047.h"` + `#include "dev_drv8301.h"` |

### User/ 新增（8 个）

| 文件 | 责任 |
|---|---|
| `User/Driver/drv_flash_f4.h` | F4 Flash 接口（签名同 drv_flash_g4.h）+ F4 缺失宏补全 |
| `User/Driver/drv_flash_f4.c` | F4 Flash 实现（扇区擦除 + 32-bit 字编程） |
| `User/Devices/dev_as5047.h` | AS5047P 芯片驱动头 |
| `User/Devices/dev_as5047.c` | AS5047P 芯片驱动实现 |
| `User/Devices/dev_encoder_as5047.h` | AS5047→dev_encoder_t 适配层头 |
| `User/Devices/dev_encoder_as5047.c` | 适配层实现 |
| `User/Devices/dev_drv8301.h` | DRV8301 设备对象头 |
| `User/Devices/dev_drv8301.c` | DRV8301 设备对象实现 |

### Board/ODriveMKS/ 新增

- `Config/dev_config_board.{h,inc}`：板级配置表
- `Core/*`：CubeMX 重新生成 F4 HAL
- `Core/Src/main.c`：USER CODE 区段调 `dev_drv8301_init()`
- `MDK-ARM/*`：F405 启动文件 + 链接脚本（代码区限制在 0x080E0000 前）

---

## 三、任务分解

---

### Task 1: board_select.h 加 ODrive 分支

**Files:**
- Modify: `User/Config/board_select.h` (全文 29 行)

- [ ] **Step 1: 替换 board_select.h 全文**

```c
#ifndef __BOARD_SELECT_H__
#define __BOARD_SELECT_H__

/*
 * Select exactly one board macro in the MDK project:
 *   JM_BOARD_V1
 *   JM_BOARD_SFOC
 *   JM_BOARD_ODRIVE
 */
#if defined(JM_BOARD_V1) && defined(JM_BOARD_SFOC)
#error "Define only one board macro."
#endif
#if defined(JM_BOARD_V1) && defined(JM_BOARD_ODRIVE)
#error "Define only one board macro."
#endif
#if defined(JM_BOARD_SFOC) && defined(JM_BOARD_ODRIVE)
#error "Define only one board macro."
#endif

#ifndef JM_BOARD_SFOC
#ifndef JM_BOARD_ODRIVE
#define JM_BOARD_SFOC
#endif
#endif

#if defined(JM_BOARD_V1)
#define JM_BOARD_NAME "V1"
#include "../../Board/V1/Config/dev_config_board.h"
#define JM_BOARD_DEV_CONFIG_INC "../../Board/V1/Config/dev_config_board.inc"
#elif defined(JM_BOARD_SFOC)
#define JM_BOARD_NAME "SFOC"
#include "../../Board/SFOC/Config/dev_config_board.h"
#define JM_BOARD_DEV_CONFIG_INC "../../Board/SFOC/Config/dev_config_board.inc"
#elif defined(JM_BOARD_ODRIVE)
#define JM_BOARD_NAME "ODRIVE"
#include "../../Board/ODriveMKS/Config/dev_config_board.h"
#define JM_BOARD_DEV_CONFIG_INC "../../Board/ODriveMKS/Config/dev_config_board.inc"
#else
#error "Define JM_BOARD_V1 / JM_BOARD_SFOC / JM_BOARD_ODRIVE in the target options."
#endif

#endif /* __BOARD_SELECT_H__ */
```

- [ ] **Step 2: 编译验证 SFOC 板未受影响**

Keil 中保持 `JM_BOARD_SFOC` 预定义，编译 SFOC 工程。
Expected: 0 error。

- [ ] **Step 3: Commit**

```bash
git add User/Config/board_select.h
git commit -m "feat(board): add JM_BOARD_ODRIVE branch in board_select.h"
```

---

### Task 2: drv_config.h — Flash 驱动从 FLASH_MCU 派生

**Files:**
- Modify: `User/Driver/drv_config.h:26-34` (USE_FLASH_G4_DRIVER 硬编码段)
- Modify: `User/Driver/drv_config.h:92-114` (FLASH_MCU 条件段)

- [ ] **Step 1: 注释掉硬编码的 USE_FLASH_G4_DRIVER**

将第 26-34 行替换为：

```c
// <c1>
// ENABLE DRIVER ---> FLASH
//#define USE_FLASH_DRIVER
// </c>

// <c1>
// ENABLE DRIVER ---> FLASH_G4 (由 FLASH_MCU 自动派生, 勿手动开启)
//#define USE_FLASH_G4_DRIVER
// </c>

// <c1>
// ENABLE DRIVER ---> FLASH_F4 (由 FLASH_MCU 自动派生, 勿手动开启)
//#define USE_FLASH_F4_DRIVER
// </c>
```

- [ ] **Step 2: 在 FLASH_MCU 条件段派生驱动宏**

将第 92-114 行替换为：

```c
// <o.0..3>FLASH MCU
//  <i> Default:STM32G474
//  <1=>STM32F103
//  <2=>STM32F429
//  <3=>STM32F405
//  <4=>STM32G431
//  <5=>STM32G491
//  <6=>STM32G474
#define FLASH_MCU 0x06

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
```

- [ ] **Step 3: 编译验证 SFOC（FLASH_MCU=0x06 → USE_FLASH_G4_DRIVER 自动派生）**

编译 SFOC 工程。
Expected: 0 error。

- [ ] **Step 4: Commit**

```bash
git add User/Driver/drv_config.h
git commit -m "feat(drv): derive FLASH driver from FLASH_MCU, add F4 path"
```

---

### Task 3: dev_config.h — 参数加 #ifndef 保护

**Files:**
- Modify: `User/Devices/dev_config.h:26-62`

- [ ] **Step 1: 替换 PM / HALF_BRIDGE / PHASE_CURRENT 段**

将第 26-62 行替换为：

```c
#if defined(USE_DEV_POWER_MONITOR)
#define PM_VREF          (3.3f)
#define PM_RESOLUTION    (4096.0f)
#ifndef PM_VBUS_RATIO
#define PM_VBUS_RATIO    (11.0f) /* 板级分压网络 11:1 (SFOC 默认) */
#endif
#define PM_IBUS_RATIO    (2.0f)  /* 1/(gain*shunt) = 1/(50*0.01) */
#define PM_IBUS_OFFSET_V (1.65f) /* INA199B1 REF=VREF/2 */
#define PM_TEMP_RATIO    (10.0f)
#ifndef PM_IBUS_SOURCE
#define PM_IBUS_SOURCE (0)
#endif
#ifndef PM_IBUS_LPF_ALPHA
#define PM_IBUS_LPF_ALPHA (0.05f)
#endif
#endif

#if defined(USE_DEV_HALF_BRIDGE)
#ifndef HALF_BRIDGE_ADC_TRIG_CCR
#define HALF_BRIDGE_ADC_TRIG_CCR (8480u)
#endif
#endif

#if defined(USE_DEV_PHASE_CURRENT)
#ifndef PHASE_CURRENT_GAIN
#define PHASE_CURRENT_GAIN       (50.0f)
#endif
#ifndef PHASE_CURRENT_SHUNT
#define PHASE_CURRENT_SHUNT      (0.01f)
#endif
#define PHASE_CURRENT_VREF       (3.3f)
#define PHASE_CURRENT_RESOLUTION (4096.0f)
#define PHASE_CURRENT_LPF_ALPHA  (0.9f)
#ifndef PHASE_CURRENT_ZERO_ADC
#define PHASE_CURRENT_ZERO_ADC (2048u)
#endif
#endif
```

- [ ] **Step 2: 编译验证 SFOC**

编译 SFOC 工程。Expected: 0 error。

- [ ] **Step 3: Commit**

```bash
git add User/Devices/dev_config.h
git commit -m "feat(dev): guard board-overridable params with #ifndef"
```

---

### Task 4: dev_flash.h — 条件 include Flash 驱动头

**Files:**
- Modify: `User/Devices/dev_flash.h:34`

- [ ] **Step 1: 替换硬编码 include**

将第 34 行：

```c
#include "drv_flash_g4.h"
```

替换为：

```c
#if defined(USE_FLASH_G4_DRIVER)
#include "drv_flash_g4.h"
#elif defined(USE_FLASH_F4_DRIVER)
#include "drv_flash_f4.h"
#endif
```

- [ ] **Step 2: 编译验证 SFOC（USE_FLASH_G4_DRIVER → include drv_flash_g4.h）**

编译 SFOC 工程。Expected: 0 error。

- [ ] **Step 3: Commit**

```bash
git add User/Devices/dev_flash.h
git commit -m "feat(dev_flash): conditional include g4/f4 flash driver header"
```

---

### Task 5: motor_info_storage.h — Flash 地址加 #ifndef 保护

**Files:**
- Modify: `User/AppServices/ParamService/motor_info_storage.h:59-68`

- [ ] **Step 1: 包裹地址宏**

将第 59-68 行替换为：

```c
/* ===== motor_info Flash 存储地址定义 =====
 * 默认值针对 STM32G474CB 128KB 双Bank Flash，Bank2 末尾 4KB：
 *   存储区 0x0804F000，4KB，页 2KB，2 扇区 A/B 轮转磨损均衡。
 * ODrive(F405RG 1MB) 板在 dev_config_board.h 中覆盖为 Sector 11(0x080E0000, 128KB)，
 * 单扇区无磨损均衡。
 * @note 须在 Keil 链接脚本中将代码区限制在存储区起始地址之前。*/
#ifndef MOTORINFO_FLASH_START_ADDR
#define MOTORINFO_FLASH_START_ADDR 0x0804F000U
#endif
#ifndef MOTORINFO_FLASH_TOTAL_SIZE
#define MOTORINFO_FLASH_TOTAL_SIZE 0x00001000U /* 4KB */
#endif
#ifndef MOTORINFO_FLASH_PAGE_SIZE
#define MOTORINFO_FLASH_PAGE_SIZE  2048U       /* 2KB，双Bank页大小 */
#endif
```

- [ ] **Step 2: Commit**

```bash
git add User/AppServices/ParamService/motor_info_storage.h
git commit -m "feat(storage): guard Flash addr macros with #ifndef for board override"
```

---

### Task 6: dev_motor_phase_current.c — IC 合成分支

**Files:**
- Modify: `User/Devices/dev_motor_phase_current.c:66-73`

- [ ] **Step 1: 替换 dev_phase_current_get_value**

将第 66-73 行替换为：

```c
/* 读取三相注入组原始ADC值(按各相推导出的 id/rank) */
static void dev_phase_current_get_value(struct dev_adc_injected *pobj)
{
	assert_report(pobj != NULL);
	pobj->adc.a = drv_adc_injected_get_value(pobj->src[ADCX_IA].id, pobj->src[ADCX_IA].rank);
	pobj->adc.b = drv_adc_injected_get_value(pobj->src[ADCX_IB].id, pobj->src[ADCX_IB].rank);
#if defined(DRV8301_TWO_PHASE_CURRENT)
	/* DRV8301 仅输出 2 路相电流(SO1/SO2), IC 由基尔霍夫电流定律合成 */
	pobj->adc.c = -(pobj->adc.a + pobj->adc.b);
#else
	pobj->adc.c = drv_adc_injected_get_value(pobj->src[ADCX_IC].id, pobj->src[ADCX_IC].rank);
#endif
}
```

- [ ] **Step 2: 编译验证 SFOC（不定义 DRV8301_TWO_PHASE_CURRENT，走原路径）**

编译 SFOC 工程。Expected: 0 error。

- [ ] **Step 3: Commit**

```bash
git add User/Devices/dev_motor_phase_current.c
git commit -m "feat(phase_current): add DRV8301 2-phase IC synthesis branch"
```

---

### Task 7: dev_motor.h — 加 AS5047 编码器枚举

**Files:**
- Modify: `User/Devices/dev_motor.h:36-41`

- [ ] **Step 1: 替换编码器枚举段**

将第 36-41 行替换为：

```c
#define DEV_MOTOR_ENCODER_MT6701 1
#define DEV_MOTOR_ENCODER_MT6835 2
#define DEV_MOTOR_ENCODER_AS5047 3

#ifndef DEV_MOTOR_ENCODER_TYPE
#define DEV_MOTOR_ENCODER_TYPE DEV_MOTOR_ENCODER_MT6701
#endif
```

- [ ] **Step 2: Commit**

```bash
git add User/Devices/dev_motor.h
git commit -m "feat(motor): add DEV_MOTOR_ENCODER_AS5047 enum"
```

---

### Task 8: dev_motor.c — AS5047 分支 + 使能引脚条件编译

**Files:**
- Modify: `User/Devices/dev_motor.c:33-41, 99-105`

- [ ] **Step 1: 加 AS5047 适配层头**

将第 33-37 行替换为：

```c
#if (DEV_MOTOR_ENCODER_TYPE == DEV_MOTOR_ENCODER_MT6701)
#include "dev_encoder_mt6701.h"
#elif (DEV_MOTOR_ENCODER_TYPE == DEV_MOTOR_ENCODER_MT6835)
#include "dev_encoder_mt6835.h"
#elif (DEV_MOTOR_ENCODER_TYPE == DEV_MOTOR_ENCODER_AS5047)
#include "dev_encoder_as5047.h"
#endif
```

- [ ] **Step 2: 使能引脚条件编译**

将第 39-41 行替换为：

```c
#if defined(JM_BOARD_ODRIVE)
/* ODrive Mini: EN_GATE = PB12 (DRV8301) */
static dev_motor_enable_config_t motor_enable_list[DEV_MOTOR_MAX] = {
	{"MOTOR1_EN", {(gpioType_e)DRV_GPIOB, (gpioPin_e)DRV_PIN_12, (drvPinState_e)0}},
};
#else
/* SFOC/V1: EN = PB2 */
static dev_motor_enable_config_t motor_enable_list[DEV_MOTOR_MAX] = {
	{"MOTOR1_EN", {(gpioType_e)DRV_GPIOB, (gpioPin_e)DRV_PIN_2, (drvPinState_e)0}},
};
#endif
```

- [ ] **Step 3: 加 AS5047 工厂调用**

将第 99-105 行替换为：

```c
#if (DEV_MOTOR_ENCODER_TYPE == DEV_MOTOR_ENCODER_MT6701)
	dev_encoder_mt6701_create(&pobj->encoder, (mt6701_id_e)id);
#elif (DEV_MOTOR_ENCODER_TYPE == DEV_MOTOR_ENCODER_MT6835)
	dev_encoder_mt6835_create(&pobj->encoder, (mt6835_id_e)id);
#elif (DEV_MOTOR_ENCODER_TYPE == DEV_MOTOR_ENCODER_AS5047)
	dev_encoder_as5047_create(&pobj->encoder, (as5047_id_e)id);
#else
#error "未知的 DEV_MOTOR_ENCODER_TYPE，请在 dev_motor.h 选择支持的编码器型号"
#endif
```

- [ ] **Step 4: 编译验证 SFOC（走 MT6701 路径 + PB2 使能）**

编译 SFOC 工程。Expected: 0 error。

- [ ] **Step 5: Commit**

```bash
git add User/Devices/dev_motor.c
git commit -m "feat(motor): add AS5047 encoder branch + ODrive EN_GATE pin"
```

---

### Task 9: dev_config.c — 加 AS5047/DRV8301 头文件

**Files:**
- Modify: `User/Devices/dev_config.c:8-16`

- [ ] **Step 1: 在 include 列表中添加新设备头**

将第 8-16 行替换为：

```c
#include "dev_led.h"
#include "dev_mt6701.h"
#include "dev_mt6835.h"
#include "dev_as5047.h"
#include "dev_drv8301.h"
#include "dev_half_bridge.h"
#include "dev_eeprom.h"
#include "dev_power_monitor.h"
#include "dev_motor_phase_current.h"
#include "dev_commun_vesc.h"
#include "dev_commun_uart.h"
```

> **说明**：`dev_as5047.h` 和 `dev_drv8301.h` 内部由 `#if defined(USE_DEV_AS5047)` / `#if defined(USE_DEV_DRV8301)` 保护，SFOC 不定义这些宏时头文件为空，不影响 SFOC 编译。

- [ ] **Step 2: 编译验证 SFOC**

编译 SFOC 工程。Expected: 0 error。

- [ ] **Step 3: Commit**

```bash
git add User/Devices/dev_config.c
git commit -m "feat(dev_config): include dev_as5047.h and dev_drv8301.h"
```

---

### Task 10: 新增 drv_flash_f4.h

**Files:**
- Create: `User/Driver/drv_flash_f4.h`

- [ ] **Step 1: 写头文件**

```c
/**
 * @file        drv_flash_f4.h
 * @brief       STM32F4内部FLASH驱动接口（单Bank扇区擦除/读/写）
 *
 * @note        接口签名与 drv_flash_g4.h 完全对齐，dev_flash.c / motor_info_storage.c
 *              业务层无需改动。F405RG(1MB) 扇区划分：
 *                Sector 0~3: 16KB  (0x08000000~0x0800FFFF)
 *                Sector 4:   64KB  (0x08010000~0x0801FFFF)
 *                Sector 5~11: 128KB (0x08020000~0x080FFFFF)
 *              编程粒度 32-bit word (FLASH_TYPEPROGRAM_WORD)。
 *              本驱动无内部页缓冲：write 直接擦除目标扇区 + 字编程，
 *              上层 dev_flash.c 在 F4 下走单扇区无磨损均衡路径(跳过整页读回)。
 */
#ifndef __FLASH_F4_H
#define __FLASH_F4_H

#include "drv_config.h"

#ifdef USE_FLASH_F4_DRIVER

#include "main.h"

/* 简写整型别名（与 drv_flash_g4.h 一致，供 drv_flash_f4/dev_flash 共用） */
typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

#define STM32_FLASH_BASE FLASH_BASE /* 0x08000000 */
#define FLASH_WAITETIME 50000U

#define FLASH_NULL_U8  0xff
#define FLASH_NULL_U16 0xffff
#define FLASH_NULL_U32 0xffffffff
#define FLASH_NULL_U64 0xffffffffffffffff

/* F4 HAL 没有 FLASH_FLAG_ALL_ERRORS 宏（G4 专有），这里用 F4 的错误标志组合补全 */
#ifndef FLASH_FLAG_ALL_ERRORS
#define FLASH_FLAG_ALL_ERRORS (FLASH_FLAG_EOP    | FLASH_FLAG_OPERR  | \
                               FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | \
                               FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR)
#endif

#define FLASH_ERR_OK             0U
#define FLASH_ERR_ADDR_OUT_RANGE 1U
#define FLASH_ERR_BUSY           2U
#define FLASH_ERR_ERASE_FAILED   3U
#define FLASH_ERR_BANK_INVALID   4U
#define FLASH_ERR_WRITE_VERIFY   5U
#define FLASH_ERR_PARAM          6U

u8  drv_f4_flash_is_dualbank(void);
u32 drv_f4_flash_total_size(void);
u32 drv_f4_flash_page_size(void);
u8  drv_f4_flash_get_bank(u32 addr);

u8 drv_f4_flash_erase_sector(const u32 addr, u8 len, u8 bank);
u8 drv_f4_flash_read(const u32 addr, u64 *pdata64, u32 len_64);
u8 drv_f4_flash_write(const u32 addr, u64 *pdata64, u32 len_64, u8 bank);
u8 drv_f4_flash_write_buffer(const u32 addr, u64 *pdata64, u32 len_64, u8 bank);

u8 drv_flash_read(const u32 addr, u64 *pdata64, u32 len_64);
u8 drv_flash_write(const u32 addr, u64 *pdata64, u32 len_64);
u8 drv_flash_clear(const u32 addr, u8 len, u8 bank);

#endif /* USE_FLASH_F4_DRIVER */
#endif /* __FLASH_F4_H */
```

- [ ] **Step 2: Commit**

```bash
git add User/Driver/drv_flash_f4.h
git commit -m "feat(drv): add drv_flash_f4.h interface (mirrors drv_flash_g4 + F4 flag compat)"
```

---

### Task 11: 新增 drv_flash_f4.c

**Files:**
- Create: `User/Driver/drv_flash_f4.c`

- [ ] **Step 1: 写实现（F405RG 1MB，12 扇区）**

```c
/**
 * @file        drv_flash_f4.c
 * @brief       STM32F4 内部 FLASH 驱动实现（单 Bank 扇区擦除 + 32-bit 字编程）
 *
 * @note        F405RG(1MB) 共 12 个扇区：
 *                Sector 0~3: 16KB, Sector 4: 64KB, Sector 5~11: 128KB
 *              write 假设上层传入整段数据(从扇区首地址开始),
 *              内部擦除该扇区 + 按 32-bit 字编程, 无需页缓冲。
 */
#include "drv_flash_f4.h"

#ifdef USE_FLASH_F4_DRIVER
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_flash_ex.h"

/* F405RG(1MB) 扇区表：起始地址 + 大小 */
typedef struct {
	u32 base;
	u32 size;
} f4_sector_t;

static const f4_sector_t s_sectors[12] = {
	{0x08000000U, 0x4000U},   /* Sector 0:  16KB */
	{0x08004000U, 0x4000U},   /* Sector 1:  16KB */
	{0x08008000U, 0x4000U},   /* Sector 2:  16KB */
	{0x0800C000U, 0x4000U},   /* Sector 3:  16KB */
	{0x08010000U, 0x10000U},  /* Sector 4:  64KB */
	{0x08020000U, 0x20000U},  /* Sector 5:  128KB */
	{0x08040000U, 0x20000U},  /* Sector 6:  128KB */
	{0x08060000U, 0x20000U},  /* Sector 7:  128KB */
	{0x08080000U, 0x20000U},  /* Sector 8:  128KB */
	{0x080A0000U, 0x20000U},  /* Sector 9:  128KB */
	{0x080C0000U, 0x20000U},  /* Sector 10: 128KB */
	{0x080E0000U, 0x20000U},  /* Sector 11: 128KB (Flash 尾部, 存储区) */
};

/* 根据地址找扇区号 0~11, 越界返回 0xFF */
static u8 addr_to_sector(u32 addr)
{
	for (u8 i = 0; i < 12; i++)
	{
		if (addr >= s_sectors[i].base && addr < s_sectors[i].base + s_sectors[i].size)
			return i;
	}
	return 0xFFU;
}

u8 drv_f4_flash_is_dualbank(void) { return 0U; }
u32 drv_f4_flash_total_size(void) { return 0x100000U; } /* F405RG 1MB */
u32 drv_f4_flash_page_size(void)  { return 0x20000U; }  /* 末段扇区 128KB */
u8 drv_f4_flash_get_bank(u32 addr)
{
	if (addr < FLASH_BASE || addr >= FLASH_BASE + drv_f4_flash_total_size())
		return 0xFFU;
	return 1U; /* F4 单 Bank */
}

u8 drv_f4_flash_erase_sector(const u32 addr, u8 len, u8 bank)
{
	(void)bank; /* F4 单 Bank */
	if (len == 0U)
		return FLASH_ERR_PARAM;

	u8 start_sec = addr_to_sector(addr);
	if (start_sec == 0xFFU || (u16)start_sec + len > 12U)
		return FLASH_ERR_ADDR_OUT_RANGE;
	/* 要求 addr 扇区对齐 */
	if (addr != s_sectors[start_sec].base)
		return FLASH_ERR_ADDR_OUT_RANGE;

	FLASH_EraseInitTypeDef er;
	u32 err;
	er.TypeErase = FLASH_TYPEERASE_SECTORS;
	er.VoltageRange = FLASH_VOLTAGE_RANGE_3; /* 2.7~3.6V */
	er.Sector = start_sec;
	er.NbSectors = len;

	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
	HAL_StatusTypeDef hal_ret = HAL_FLASHEx_Erase(&er, &err);
	HAL_FLASH_Lock();
	return (hal_ret == HAL_OK) ? FLASH_ERR_OK : FLASH_ERR_ERASE_FAILED;
}

u8 drv_f4_flash_read(const u32 addr, u64 *pdata64, u32 len_64)
{
	if (pdata64 == NULL || len_64 == 0U || (addr % 8U) != 0U)
		return FLASH_ERR_PARAM;
	if (drv_f4_flash_get_bank(addr) == 0xFFU)
		return FLASH_ERR_ADDR_OUT_RANGE;
	u32 end_addr = addr + len_64 * 8U;
	if (drv_f4_flash_get_bank(end_addr - 1U) == 0xFFU)
		return FLASH_ERR_ADDR_OUT_RANGE;
	for (u32 i = 0; i < len_64; i++)
		pdata64[i] = *(__IO uint64_t *)(addr + i * 8U);
	return FLASH_ERR_OK;
}

/* write: 擦除包含 addr 的扇区(若 addr 为扇区首地址), 再按 32-bit 字编程。
 * 假设上层 dev_flash.c 在 F4 下传 flag+data(非整扇区), 直接擦除+编程。 */
u8 drv_f4_flash_write(const u32 addr, u64 *pdata64, u32 len_64, u8 bank)
{
	(void)bank;
	if (pdata64 == NULL || len_64 == 0U || (addr % 8U) != 0U)
		return FLASH_ERR_PARAM;
	if (drv_f4_flash_get_bank(addr) == 0xFFU)
		return FLASH_ERR_ADDR_OUT_RANGE;

	u8 sec = addr_to_sector(addr);
	if (sec == 0xFFU)
		return FLASH_ERR_ADDR_OUT_RANGE;
	u32 sec_end = s_sectors[sec].base + s_sectors[sec].size;
	u32 end_addr = addr + len_64 * 8U;
	if (end_addr > sec_end)
		return FLASH_ERR_ADDR_OUT_RANGE;

	/* 1. 若 addr 为扇区首地址, 先擦除该扇区 */
	if (addr == s_sectors[sec].base)
	{
		u8 r = drv_f4_flash_erase_sector(addr, 1, 1);
		if (r != FLASH_ERR_OK)
			return r;
	}

	/* 2. 解锁 + 32-bit 字编程(u64 拆成 2 个 u32) */
	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
	if (FLASH_WaitForLastOperation(FLASH_WAITETIME) != HAL_OK)
	{
		__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
		HAL_FLASH_Lock();
		return FLASH_ERR_BUSY;
	}

	for (u32 i = 0; i < len_64; i++)
	{
		u64 v = pdata64[i];
		u32 w0 = (u32)(v & 0xFFFFFFFFU);
		u32 w1 = (u32)(v >> 32);
		u32 a0 = addr + i * 8U;
		u32 a1 = a0 + 4U;

		for (u8 try = 0; try < 10; try++)
		{
			if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, a0, w0) == HAL_OK)
				if (*(__IO uint32_t *)a0 == w0)
					break;
			__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
		}
		if (*(__IO uint32_t *)a0 != w0)
		{
			HAL_FLASH_Lock();
			return FLASH_ERR_WRITE_VERIFY;
		}
		for (u8 try = 0; try < 10; try++)
		{
			if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, a1, w1) == HAL_OK)
				if (*(__IO uint32_t *)a1 == w1)
					break;
			__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
		}
		if (*(__IO uint32_t *)a1 != w1)
		{
			HAL_FLASH_Lock();
			return FLASH_ERR_WRITE_VERIFY;
		}
	}
	HAL_FLASH_Lock();
	return FLASH_ERR_OK;
}

u8 drv_f4_flash_write_buffer(const u32 addr, u64 *pdata64, u32 len_64, u8 bank)
{
	return drv_f4_flash_write(addr, pdata64, len_64, bank);
}

u8 drv_flash_read(const u32 addr, u64 *pdata64, u32 len_64)
{
	u8 ret;
	__disable_irq();
	ret = drv_f4_flash_read(addr, pdata64, len_64);
	__enable_irq();
	return ret;
}

u8 drv_flash_write(const u32 addr, u64 *pdata64, u32 len_64)
{
	if (drv_f4_flash_get_bank(addr) == 0xFFU)
		return FLASH_ERR_ADDR_OUT_RANGE;
	u8 ret;
	__disable_irq();
	ret = drv_f4_flash_write(addr, pdata64, len_64, 1U);
	__enable_irq();
	return ret;
}

u8 drv_flash_clear(const u32 addr, u8 len, u8 bank)
{
	u8 ret;
	__disable_irq();
	ret = drv_f4_flash_erase_sector(addr, len, bank);
	__enable_irq();
	return ret;
}

#endif /* USE_FLASH_F4_DRIVER */
```

- [ ] **Step 2: Commit**

```bash
git add User/Driver/drv_flash_f4.c
git commit -m "feat(drv): add drv_flash_f4.c impl (12 sectors, 32-bit word program)"
```

---

### Task 12: dev_flash.c — F4 单扇区无磨损均衡写路径

**Files:**
- Modify: `User/Devices/dev_flash.c:62-115`

**背景**：F4 尾部扇区 128KB，G4 路径的整页 read-modify-write 需 128KB 缓冲 → RAM 溢出。F4 下用单扇区无磨损均衡：直接擦除扇区 + 写 flag+data，无需整页读回。

- [ ] **Step 1: 替换 dev_flash_write 全函数体**

将第 62-115 行（`int dev_flash_write` 全函数）替换为：

```c
int dev_flash_write(struct dev_flash *pobj, u32 offset, u64 *data, u16 size)
{
	if (pobj == NULL || data == NULL || !pobj->inited)
		return DEV_ERROR;
	if ((offset % WORD_SIZE) != 0U)
		return DEV_ERROR;

	uint32_t usable = pobj->page_size - WORD_SIZE;
	if ((uint64_t)offset + (uint64_t)size * WORD_SIZE > usable)
		return DEV_ERROR;

#if defined(STM32F405xx)
	/* F4: 单扇区无磨损均衡。直接擦除扇区 + 写 flag+data, 无需整页读回。
	 * motor_info 每次写全量(~1KB=128 u64), page_buf 只需容纳 flag+data。 */
	static u64 f4_buf[257U]; /* flag(1) + 最多 256 个 u64 数据 = 2056 字节 */
	if (size > 256U)
		return DEV_ERROR;

	uint32_t new_seq = pobj->last_sequence + 1U;
	f4_buf[0] = FLASH_FLAG_MAKE(new_seq);
	for (uint16_t i = 0; i < size; i++)
		f4_buf[1U + i] = data[i];

	/* 写入起始地址 = start_addr(扇区首), drv_flash_f4_write 会先擦除该扇区 */
	u8 ret = drv_flash_write(pobj->start_addr, f4_buf, (u16)(1U + size));
	if (ret != FLASH_ERR_OK)
		return DEV_ERROR;

	pobj->last_sector = 0U;
	pobj->last_sequence = new_seq;
	return DEV_EOK;
#else
	/* G4: 读改写 + 磨损均衡 */
	uint32_t page_u64 = pobj->page_size / WORD_SIZE;
	static u64 page_buf[0x1000U / sizeof(u64)]; /* 4KB / 8 = 512 u64 */
	if (pobj->page_size > sizeof(page_buf))
		return DEV_ERROR;

	uint32_t cur_addr = pobj->start_addr + pobj->last_sector * pobj->page_size;
	if (drv_flash_read(cur_addr, page_buf, page_u64) != FLASH_ERR_OK)
		return DEV_ERROR;

	uint32_t data_idx = (WORD_SIZE + offset) / WORD_SIZE;
	for (uint16_t i = 0; i < size; i++)
		page_buf[data_idx + i] = data[i];

	uint32_t new_seq = pobj->last_sequence + 1U;
	page_buf[0] = FLASH_FLAG_MAKE(new_seq);

	uint32_t next = (pobj->last_sector + 1U) % pobj->sector_count;
	uint32_t write_addr = pobj->start_addr + next * pobj->page_size;
	u8 ret = drv_flash_write(write_addr, page_buf, page_u64);
	if (ret != FLASH_ERR_OK)
		return DEV_ERROR;

	pobj->last_sector = next;
	pobj->last_sequence = new_seq;
	return DEV_EOK;
#endif
}
```

- [ ] **Step 2: 编译验证 SFOC（走 #else G4 路径）**

编译 SFOC 工程。Expected: 0 error。

- [ ] **Step 3: Commit**

```bash
git add User/Devices/dev_flash.c
git commit -m "feat(dev_flash): add F4 single-sector no-wear-leveling write path"
```

---

### Task 13: 新增 dev_as5047.h

**Files:**
- Create: `User/Devices/dev_as5047.h`
- 参考: `dev_mt6701.h`

- [ ] **Step 1: 写头文件**

```c
/**
 * @file        dev_as5047.h
 * @brief       AS5047P 磁编码器(14bit SPI): 角度/校验/零点/方向
 *
 * @details     AS5047P 16-bit SPI 帧:
 *   命令帧: bit15=R/W(1=read), bit14=parity, bit13:0=addr
 *   数据帧: bit15=EF(错误标志), bit14=parity, bit13:0=DATA(14bit角度)
 *   读 ANGLECOM(0x3FFF): 发 0xFFFF(含parity), 再发 dummy, 第二帧收角度。
 *   SPI Mode 1(CPOL=0, CPHA=1), MSB first, 16-bit data size。
 *
 * @note        结构对齐 dev_mt6701.h, 手动 CS 控制(spiDrv_t.cs={0})。
 */
#ifndef __DEV_AS5047_H_
#define __DEV_AS5047_H_

#include "dev_config.h"
#if defined(USE_DEV_AS5047)

#include "drv_spi.h"
#include "drv_gpio.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AS5047_ANGLE_RESOLUTION (1 << 14)   /* 2^14 = 16384 */
#define AS5047_READ_ANGLE_CMD   0xFFFFU     /* 读 ANGLECOM(0x3FFF) + parity = 0xFFFF */

typedef enum {
	AS5047_ID_1 = 0,
	AS5047_ID_MAX,
} as5047_id_e;

typedef enum {
	AS5047_DIR_CW  = 1,
	AS5047_DIR_CCW = -1,
} as5047_dir_e;

/* 资源配置(在 dev_config_board.inc 的 as5047_list 填表) */
typedef struct {
	char name[20];
	spiDrv_t spi_num;   /* SPI 外设(cs 字段不使用, 填 {0}) */
	gpioDrv_t csn;      /* 片选引脚(手动控制) */
} as5047_config_t;

extern const as5047_config_t as5047_list[AS5047_ID_MAX];

typedef struct dev_as5047 {
	as5047_id_e id;
	uint16_t raw;            /* 14bit 原始角度 */
	float mech_angle_org;    /* 未补偿原始角度, ° */
	float mechanical_angle;  /* 最终机械角度, ° [0,360) */
	uint8_t  parity_err;     /* 奇偶校验错误标志 */
	uint8_t  ef;             /* AS5047P 错误标志位(EF) */
	uint16_t err_cnt;        /* 连续错误计数 */
	float offset;            /* 零点偏移, ° */
	as5047_dir_e dir;

	/* public */
	void (*update)(struct dev_as5047 *pobj);
	void (*set_dir)(struct dev_as5047 *pobj, as5047_dir_e dir);
	as5047_dir_e (*get_dir)(struct dev_as5047 *pobj);
} dev_as5047_t;

void dev_as5047_init(dev_as5047_t *pobj, as5047_id_e dev_id);

#ifdef __cplusplus
}
#endif
#endif /* USE_DEV_AS5047 */
#endif /* __DEV_AS5047_H_ */
```

- [ ] **Step 2: Commit**

```bash
git add User/Devices/dev_as5047.h
git commit -m "feat(dev): add dev_as5047.h header (AS5047P chip driver)"
```

---

### Task 14: 新增 dev_as5047.c

**Files:**
- Create: `User/Devices/dev_as5047.c`
- 参考: `dev_mt6701.c`（手动 CS + spiDrv_t.cs={0} 模式）

- [ ] **Step 1: 写实现**

```c
/**
 * @file        dev_as5047.c
 * @brief       AS5047P 磁编码器驱动实现
 *
 * @note        SPI 时序: 16-bit data size, len=1 表示一次 16-bit 传输。
 *   读角度(2-transfer):
 *     1. CSN LOW → 发 0xFFFF(读ANGLECOM命令) → 收 garbage → CSN 保持 LOW
 *     2. 发 0x0000(dummy) → 收 16-bit 角度帧 → CSN HIGH
 *   角度帧: bit15=EF, bit14=parity, bit13:0=14bit角度
 *
 *   奇偶校验: AS5047P 使用偶校验, 计算范围 = bit15 + bit13:0 (不含 bit14 本身)。
 *   即 15bit 数据中 1 的个数为偶数时 parity=0, 奇数时 parity=1。
 */
#include "dev_as5047.h"

#if defined(USE_DEV_AS5047)

#include <string.h>
#include <math.h>
#include "assert_report.h"

/* 角度归一化到 [0,360) */
static inline float as5047_norm360(float deg)
{
	deg = fmodf(deg, 360.0F);
	return (deg < 0.0F) ? (deg + 360.0F) : deg;
}

/* 偶校验计算: 对 15bit 数据(bit15 + bit13:0)计算, parity 使总数(含 parity)为偶数。
 * 输入 = frame & 0xBFFF (保留 bit15, 清 bit14, 保留 bit13:0) */
static uint8_t calc_even_parity_15bit(uint16_t v15)
{
	uint8_t p = 0;
	for (uint8_t i = 0; i < 15; i++)
		p ^= (uint8_t)((v15 >> i) & 1U);
	return p; /* 期望的 parity 值: 使 15bit 数据 + parity = 偶数个 1 */
}

/* SPI 16-bit 收发(CS 由调用方手动控制, spiDrv_t.cs={0}) */
static void as5047_spi_xfer(spiDrv_t dev, uint8_t *tx, uint8_t *rx)
{
	drv_spi_transfer(dev, tx, rx, 1, 1000); /* len=1: 一次 16-bit 传输 */
}

/* 读取角度(2-transfer 协议) */
static uint16_t as5047_read_angle(dev_as5047_t *pobj)
{
	const as5047_config_t *cfg = &as5047_list[pobj->id];
	spiDrv_t dev = {.hspi = cfg->spi_num.hspi, .cs = {0}}; /* cs={0}: 驱动不操作 CS */

	uint8_t cmd[2]   = {0xFF, 0xFF}; /* 0xFFFF big-endian */
	uint8_t dummy[2] = {0x00, 0x00};
	uint8_t rx1[2]   = {0};
	uint8_t rx2[2]   = {0};

	/* CSN LOW → 发命令 → 发 dummy 收数据 → CSN HIGH */
	drv_gpio_write(cfg->csn, DRV_PIN_LOW);
	as5047_spi_xfer(dev, cmd, rx1);    /* 发读命令, 收 garbage */
	as5047_spi_xfer(dev, dummy, rx2);  /* 发 dummy, 收角度帧 */
	drv_gpio_write(cfg->csn, DRV_PIN_HIGH);

	return ((uint16_t)rx2[0] << 8) | rx2[1];
}

static void dev_as5047_update(struct dev_as5047 *pobj)
{
	uint16_t frame = as5047_read_angle(pobj);

	/* 角度帧: bit15=EF(错误标志), bit14=parity(偶校验), bit13:0=14bit角度 */
	uint8_t ef         = (uint8_t)((frame >> 15) & 1U);  /* bit15 = EF */
	uint8_t parity_bit = (uint8_t)((frame >> 14) & 1U);  /* bit14 = parity */
	uint16_t angle14   = frame & 0x3FFFU;                 /* bit13:0 = 角度 */

	/* 偶校验: 计算 bit15 + bit13:0 (15bit, 不含 bit14)
	 * mask = 0xBFFF = bit15 + bit13:0 (bit14 清零) */
	uint8_t expect_p = calc_even_parity_15bit(frame & 0xBFFFU);
	pobj->parity_err = (parity_bit != expect_p) ? 1U : 0U;
	pobj->ef = ef;

	if (pobj->parity_err || ef)
	{
		if (pobj->err_cnt < 0xFFFFU)
			pobj->err_cnt++;
		return; /* 坏帧不更新角度 */
	}
	pobj->err_cnt = 0;
	pobj->raw = angle14;

	/* 原始角度 → 去偏移 → 方向 → 归一化 */
	float deg = (float)angle14 * 360.0F / (float)AS5047_ANGLE_RESOLUTION;
	pobj->mech_angle_org = deg;
	deg = as5047_norm360(deg - pobj->offset);
	if (pobj->dir == AS5047_DIR_CCW)
		deg = as5047_norm360(360.0F - deg);
	pobj->mechanical_angle = deg;
}

static void dev_as5047_set_dir(struct dev_as5047 *pobj, as5047_dir_e dir)
{
	assert_report(pobj != NULL);
	if (dir == AS5047_DIR_CW || dir == AS5047_DIR_CCW)
		pobj->dir = dir;
}

static as5047_dir_e dev_as5047_get_dir(struct dev_as5047 *pobj)
{
	assert_report(pobj != NULL);
	return pobj->dir;
}

void dev_as5047_init(dev_as5047_t *pobj, as5047_id_e dev_id)
{
	assert_report(pobj != NULL);
	assert_report(dev_id < AS5047_ID_MAX);
	memset(pobj, 0, sizeof(dev_as5047_t));
	pobj->id = dev_id;
	pobj->dir = AS5047_DIR_CW;
	pobj->offset = 0.0F;
	pobj->update  = dev_as5047_update;
	pobj->set_dir = dev_as5047_set_dir;
	pobj->get_dir = dev_as5047_get_dir;
}

#endif /* USE_DEV_AS5047 */
```

> **注意**：AS5047P 的偶校验计算范围为 bit15(EF) + bit13:0(角度数据)，不含 bit14(parity 位本身)。若硬件实测校验失败，请优先确认校验范围是否包含 bit15。

- [ ] **Step 2: Commit**

```bash
git add User/Devices/dev_as5047.c
git commit -m "feat(dev): add dev_as5047.c impl (14bit angle + parity + direction)"
```

---

### Task 15: 新增 dev_encoder_as5047.h + .c

**Files:**
- Create: `User/Devices/dev_encoder_as5047.h`
- Create: `User/Devices/dev_encoder_as5047.c`
- 参考: `dev_encoder_mt6701.h/.c`

- [ ] **Step 1: 写适配层头**

```c
/**
 * @file        dev_encoder_as5047.h
 * @brief       AS5047P → dev_encoder_t 抽象适配层
 */
#ifndef __DEV_ENCODER_AS5047_H__
#define __DEV_ENCODER_AS5047_H__

#include "dev_config.h"
#if defined(USE_DEV_AS5047)

#include "dev_encoder.h"
#include "dev_as5047.h"

#ifdef __cplusplus
extern "C" {
#endif

void dev_encoder_as5047_create(dev_encoder_t *enc, as5047_id_e id);

#ifdef __cplusplus
}
#endif
#endif /* USE_DEV_AS5047 */
#endif /* __DEV_ENCODER_AS5047_H__ */
```

- [ ] **Step 2: 写适配层实现**

```c
/**
 * @file        dev_encoder_as5047.c
 * @brief       AS5047P 编码器适配层实现
 */
#include "dev_encoder_as5047.h"

#if defined(USE_DEV_AS5047)

#include <string.h>
#include "assert_report.h"

static dev_as5047_t s_as5047[AS5047_ID_MAX];

static void encoder_as5047_update(struct dev_encoder *enc)
{
	dev_as5047_t *chip = (dev_as5047_t *)enc->ctx;
	chip->update(chip);
	enc->mechanical_angle = chip->mechanical_angle;
}

static float encoder_as5047_get_mechanical_angle(struct dev_encoder *enc)
{
	return enc->mechanical_angle;
}

static void encoder_as5047_set_offset(struct dev_encoder *enc, float offset_deg)
{
	dev_as5047_t *chip = (dev_as5047_t *)enc->ctx;
	chip->offset = offset_deg;
}

static float encoder_as5047_get_offset(struct dev_encoder *enc)
{
	dev_as5047_t *chip = (dev_as5047_t *)enc->ctx;
	return chip->offset;
}

static void encoder_as5047_set_dir(struct dev_encoder *enc, int8_t dir)
{
	dev_as5047_t *chip = (dev_as5047_t *)enc->ctx;
	chip->set_dir(chip, (as5047_dir_e)dir);
}

static int8_t encoder_as5047_get_dir(struct dev_encoder *enc)
{
	dev_as5047_t *chip = (dev_as5047_t *)enc->ctx;
	return (int8_t)chip->get_dir(chip);
}

static float encoder_as5047_get_raw_deg(struct dev_encoder *enc)
{
	dev_as5047_t *chip = (dev_as5047_t *)enc->ctx;
	chip->update(chip);
	return (float)chip->raw * 360.0F / (float)AS5047_ANGLE_RESOLUTION;
}

void dev_encoder_as5047_create(dev_encoder_t *enc, as5047_id_e id)
{
	assert_report(enc != NULL);
	assert_report(id < AS5047_ID_MAX);

	dev_as5047_init(&s_as5047[id], id);

	enc->ctx = &s_as5047[id];
	enc->mechanical_angle = 0.0F;
	enc->update = encoder_as5047_update;
	enc->get_mechanical_angle = encoder_as5047_get_mechanical_angle;
	enc->set_offset = encoder_as5047_set_offset;
	enc->get_offset = encoder_as5047_get_offset;
	enc->set_dir = encoder_as5047_set_dir;
	enc->get_dir = encoder_as5047_get_dir;
	enc->get_raw_deg = encoder_as5047_get_raw_deg;
}

#endif /* USE_DEV_AS5047 */
```

- [ ] **Step 3: Commit**

```bash
git add User/Devices/dev_encoder_as5047.h User/Devices/dev_encoder_as5047.c
git commit -m "feat(dev): add dev_encoder_as5047 adapter layer"
```

---

### Task 16: 新增 dev_drv8301.h

**Files:**
- Create: `User/Devices/dev_drv8301.h`

- [ ] **Step 1: 写头文件**

```c
/**
 * @file        dev_drv8301.h
 * @brief       DRV8301 栅极驱动器设备对象(SPI 配置 + 状态读取)
 *
 * @details     DRV8301 内置 2 路电流运放 + Buck + OCP。
 *   SPI 帧 16-bit: bit15=R/W(0=write,1=read), bit14:12=addr, bit11=0, bit10:0=data
 *   寄存器: 0x00=CTRL1, 0x01=CTRL2
 *   读寄存器: 发读命令 → 发 dummy → 第二帧收数据(2-transfer)
 *   SPI Mode 1(CPOL=0, CPHA=1), 16-bit data size, MSB first
 *
 * @note        设备对象模式: 静态单例 + 方法指针。
 *   init 仅做 SPI 寄存器配置, 不操作 EN_GATE(由 motor_enable_list 通过
 *   dev_motor_enable/disable 控制 EN_GATE 引脚电平)。
 *   调用时序: main.c 调 dev_drv8301_init → dev_motor_init 中 dev_motor_disable
 *   (EN_GATE 低) → FOC 初始化 → dev_motor_enable(EN_GATE 高, 复位 fault latch)。
 */
#ifndef __DEV_DRV8301_H_
#define __DEV_DRV8301_H_

#include "dev_config.h"
#if defined(USE_DEV_DRV8301)

#include "drv_spi.h"
#include "drv_gpio.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	DRV8301_ID_1 = 0,
	DRV8301_ID_MAX,
} drv8301_id_e;

#define DRV8301_REG_CTRL1 0x00U
#define DRV8301_REG_CTRL2 0x01U

typedef struct {
	char name[20];
	spiDrv_t spi_num;       /* SPI 外设(cs 字段不使用, 填 {0}) */
	gpioDrv_t nscs;         /* nSCS 片选(手动控制) */
	gpioDrv_t nfault;       /* nFAULT 输入 (PD2), 低有效 */
	uint16_t ctrl1_value;   /* CTRL1 寄存器原始值 */
	uint16_t ctrl2_value;   /* CTRL2 寄存器原始值 */
} drv8301_config_t;

extern const drv8301_config_t drv8301_list[DRV8301_ID_MAX];

typedef struct dev_drv8301 {
	drv8301_id_e id;
	uint16_t ctrl1;
	uint16_t ctrl2;
	uint8_t  fault;

	/* public */
	void     (*init)(struct dev_drv8301 *pobj);
	uint16_t (*read_reg)(struct dev_drv8301 *pobj, uint8_t addr);
	void     (*write_reg)(struct dev_drv8301 *pobj, uint8_t addr, uint16_t val);
	uint8_t  (*get_fault)(struct dev_drv8301 *pobj);
} dev_drv8301_t;

void dev_drv8301_init(dev_drv8301_t *pobj, drv8301_id_e id);
extern dev_drv8301_t g_dev_drv8301;

#ifdef __cplusplus
}
#endif
#endif /* USE_DEV_DRV8301 */
#endif /* __DEV_DRV8301_H_ */
```

> **CTRL1/CTRL2 寄存器值说明**（需对照 DRV8301 datasheet 确认）：
> - CTRL1 bit10:9 = GAIN (00=10, 01=20, 10=40, 11=80 V/V)
> - CTRL1 bit8 = DC_CAL_CH1, bit7 = DC_CAL_CH2
> - CTRL2 bit7 = PWM_MODE (0=6PWM, 1=4PWM)
> - CTRL2 bit5:4 = OCP_MODE (00=current limit, 01=auto retry, 10=OCP latch, 11=OCP off)
> - CTRL2 bit2:1 = GATE_CURRENT (00=0.25A, 01=0.70A, 10=1.17A, 11=1.70A)

- [ ] **Step 2: Commit**

```bash
git add User/Devices/dev_drv8301.h
git commit -m "feat(dev): add dev_drv8301.h device object header"
```

---

### Task 17: 新增 dev_drv8301.c

**Files:**
- Create: `User/Devices/dev_drv8301.c`

- [ ] **Step 1: 写实现**

```c
/**
 * @file        dev_drv8301.c
 * @brief       DRV8301 设备对象实现
 *
 * @note        SPI 16-bit, 2-transfer 读协议(同 AS5047P):
 *   读: CSN LOW → 发读命令(收 garbage) → 发 dummy(收数据) → CSN HIGH
 *   写: CSN LOW → 发写命令(收 status) → CSN HIGH
 *   len=1 表示一次 16-bit 传输(SPI data size=16bit)。
 */
#include "dev_drv8301.h"

#if defined(USE_DEV_DRV8301)

#include <string.h>
#include "assert_report.h"

dev_drv8301_t g_dev_drv8301;

/* 组装命令字 */
static uint16_t make_write_cmd(uint8_t addr, uint16_t data)
{
	return ((uint16_t)addr << 12) | (data & 0x07FFU);
}
static uint16_t make_read_cmd(uint8_t addr)
{
	return 0x8000U | ((uint16_t)addr << 12);
}

/* SPI 16-bit 收发(CS 由调用方手动控制) */
static void drv8301_spi_xfer(spiDrv_t dev, uint16_t cmd, uint16_t *rx)
{
	uint8_t tx[2] = {(uint8_t)(cmd >> 8), (uint8_t)(cmd & 0xFF)};
	uint8_t r[2] = {0};
	drv_spi_transfer(dev, tx, r, 1, 1000); /* len=1: 一次 16-bit */
	*rx = ((uint16_t)r[0] << 8) | r[1];
}

static void dev_drv8301_init_impl(struct dev_drv8301 *pobj)
{
	assert_report(pobj != NULL);
	const drv8301_config_t *cfg = &drv8301_list[pobj->id];
	spiDrv_t dev = {.hspi = cfg->spi_num.hspi, .cs = {0}};

	/* 写 CTRL1 */
	pobj->ctrl1 = cfg->ctrl1_value;
	uint16_t cmd = make_write_cmd(DRV8301_REG_CTRL1, pobj->ctrl1);
	uint16_t rx;
	drv_gpio_write(cfg->nscs, DRV_PIN_LOW);
	drv8301_spi_xfer(dev, cmd, &rx);
	drv_gpio_write(cfg->nscs, DRV_PIN_HIGH);

	/* 写 CTRL2 */
	pobj->ctrl2 = cfg->ctrl2_value;
	cmd = make_write_cmd(DRV8301_REG_CTRL2, pobj->ctrl2);
	drv_gpio_write(cfg->nscs, DRV_PIN_LOW);
	drv8301_spi_xfer(dev, cmd, &rx);
	drv_gpio_write(cfg->nscs, DRV_PIN_HIGH);
}

static uint16_t dev_drv8301_read_reg(struct dev_drv8301 *pobj, uint8_t addr)
{
	const drv8301_config_t *cfg = &drv8301_list[pobj->id];
	spiDrv_t dev = {.hspi = cfg->spi_num.hspi, .cs = {0}};
	uint16_t rx1, rx2;

	/* 2-transfer 读: 发读命令 → 发 dummy 收数据 */
	drv_gpio_write(cfg->nscs, DRV_PIN_LOW);
	drv8301_spi_xfer(dev, make_read_cmd(addr), &rx1);
	drv8301_spi_xfer(dev, 0x0000U, &rx2);
	drv_gpio_write(cfg->nscs, DRV_PIN_HIGH);

	return rx2 & 0x07FFU;
}

static void dev_drv8301_write_reg(struct dev_drv8301 *pobj, uint8_t addr, uint16_t val)
{
	const drv8301_config_t *cfg = &drv8301_list[pobj->id];
	spiDrv_t dev = {.hspi = cfg->spi_num.hspi, .cs = {0}};
	uint16_t rx;

	drv_gpio_write(cfg->nscs, DRV_PIN_LOW);
	drv8301_spi_xfer(dev, make_write_cmd(addr, val), &rx);
	drv_gpio_write(cfg->nscs, DRV_PIN_HIGH);
}

static uint8_t dev_drv8301_get_fault(struct dev_drv8301 *pobj)
{
	const drv8301_config_t *cfg = &drv8301_list[pobj->id];
	/* nFAULT 低有效: 0=故障, 1=正常 */
	pobj->fault = (drv_gpio_read(cfg->nfault) == DRV_PIN_LOW) ? 1U : 0U;
	return pobj->fault;
}

void dev_drv8301_init(dev_drv8301_t *pobj, drv8301_id_e id)
{
	assert_report(pobj != NULL);
	assert_report(id < DRV8301_ID_MAX);
	memset(pobj, 0, sizeof(dev_drv8301_t));
	pobj->id = id;
	pobj->init      = dev_drv8301_init_impl;
	pobj->read_reg  = dev_drv8301_read_reg;
	pobj->write_reg = dev_drv8301_write_reg;
	pobj->get_fault = dev_drv8301_get_fault;
}

#endif /* USE_DEV_DRV8301 */
```

- [ ] **Step 2: Commit**

```bash
git add User/Devices/dev_drv8301.c
git commit -m "feat(dev): add dev_drv8301.c device object impl (SPI config + fault read)"
```

---

### Task 18: 新增 dev_config_board.h（ODrive 板使能集 + 参数覆盖）

**Files:**
- Create: `Board/ODriveMKS/Config/dev_config_board.h`
- 参考: `Board/SFOC/Config/dev_config_board.h`

- [ ] **Step 1: 写头文件**

```c
#ifndef __DEV_CONFIG_BOARD_ODRIVE_H__
#define __DEV_CONFIG_BOARD_ODRIVE_H__

/* ODrive Mini (STM32F405RGT6 + DRV8301 + AS5047P) 板级使能集与参数覆盖 */

#define USE_DEV_PHASE_CURRENT
#define USE_DEV_HALF_BRIDGE
#define USE_DEV_POWER_MONITOR
#define USE_DEV_FLASH
#define USE_DEV_AS5047
#define USE_DEV_DRV8301
#define USE_DEV_COMMUN_UART
/* 不启用: USE_DEV_MT6701 / USE_DEV_MT6835 / USE_DEV_RGB_LED / USE_DEV_DWT_COUNTER */

/* DRV8301 仅 2 路相电流输出, IC 由基尔霍夫合成 */
#define DRV8301_TWO_PHASE_CURRENT

/* 编码器型号选 AS5047 */
#define DEV_MOTOR_ENCODER_TYPE DEV_MOTOR_ENCODER_AS5047

/* ===== 相电流参数(0.5mΩ + DRV8301 GAIN=40) ===== */
#undef  PHASE_CURRENT_GAIN
#define PHASE_CURRENT_GAIN       (40.0f)
#undef  PHASE_CURRENT_SHUNT
#define PHASE_CURRENT_SHUNT      (0.0005f)

/* ===== 母线电压分压: ODrive R43=18K, R44=1K → 19:1 ===== */
#undef  PM_VBUS_RATIO
#define PM_VBUS_RATIO            (19.0f)

/* ===== 母线电流来源: 三相合成(ODrive 无独立 IBUS 硬件) ===== */
#undef  PM_IBUS_SOURCE
#define PM_IBUS_SOURCE           (1)

/* ===== ADC 触发比较值: F4 TIM1 Period=8400, CCR=8380(ARR-20) =====
 * G4 默认 8480(ARR=8500), F4 必须覆盖否则 CCR>ARR 导致 ADC 永不触发 */
#undef  HALF_BRIDGE_ADC_TRIG_CCR
#define HALF_BRIDGE_ADC_TRIG_CCR (8380u)

/* ===== Flash 存储: Sector 11 (0x080E0000, 128KB), Flash 尾部, 单扇区无磨损均衡 ===== */
#undef  MOTORINFO_FLASH_START_ADDR
#define MOTORINFO_FLASH_START_ADDR 0x080E0000U
#undef  MOTORINFO_FLASH_TOTAL_SIZE
#define MOTORINFO_FLASH_TOTAL_SIZE 0x00020000U /* 128KB */
#undef  MOTORINFO_FLASH_PAGE_SIZE
#define MOTORINFO_FLASH_PAGE_SIZE  0x00020000U /* 128KB, F4 Sector */

#include "dev_config_board.inc"

#endif /* __DEV_CONFIG_BOARD_ODRIVE_H__ */
```

- [ ] **Step 2: Commit**

```bash
git add Board/ODriveMKS/Config/dev_config_board.h
git commit -m "feat(board): add ODrive dev_config_board.h (enable set + param overrides)"
```

---

### Task 19: 新增 dev_config_board.inc（ODrive 板配置表）

**Files:**
- Create: `Board/ODriveMKS/Config/dev_config_board.inc`
- 参考: `Board/SFOC/Config/dev_config_board.inc`

- [ ] **Step 1: 写配置表**

```c
#ifdef JM_BOARD_CONFIG_DEFINE_TABLES

#if defined(USE_DEV_AS5047)
const as5047_config_t as5047_list[AS5047_ID_MAX] = {
    [AS5047_ID_1] = {
        .name = "AS5047_1",
        .spi_num = {.hspi = DRV_SPI3},
        .csn = {.gpiox = DRV_GPIOB, .pin = DRV_PIN_2, .ste = DRV_PIN_LOW},  /* PB2 = GPIO_6 */
    },
};
#endif

#if defined(USE_DEV_DRV8301)
const drv8301_config_t drv8301_list[DRV8301_ID_MAX] = {
    [DRV8301_ID_1] = {
        .name = "DRV8301_1",
        .spi_num = {.hspi = DRV_SPI3},  /* 共享 SPI3 */
        .nscs = {.gpiox = DRV_GPIOC, .pin = DRV_PIN_13, .ste = DRV_PIN_LOW},  /* PC13 */
        .nfault = {.gpiox = DRV_GPIOD, .pin = DRV_PIN_2, .ste = DRV_PIN_LOW},  /* PD2 */
        /* CTRL1: GAIN=40V/V(bit10:9=10b → 0x0400), DC_CAL off
         * CTRL2: 6PWM mode(bit7=0), OCP=current limit(bit5:4=00), GATE_CURRENT=1.7A(bit2:1=11b → 0x0006)
         * @note 寄存器值需对照 DRV8301 datasheet 最终确认 */
        .ctrl1_value = 0x0400,  /* GAIN=40V/V */
        .ctrl2_value = 0x0006,  /* GATE_CURRENT=1.7A, 6PWM, OCP=current limit */
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

#if defined(USE_DEV_POWER_MONITOR)
const dev_power_monitor_config_t power_monitor_list[PM_CH_MAX] = {
    [PM_VBUS] = {.name = "VBUS", .id = DRV_ADC_1, .channel = DRV_ADC_CH6},   /* PA6 */
    /* PM_IBUS 不配置硬件通道: PM_IBUS_SOURCE=1 时由三相电流合成, 不读 ADC */
};
#endif

#if defined(USE_DEV_PHASE_CURRENT)
const dev_phase_current_config_t phase_current_list[ADCX_INX_INJECTED_MAX] = {
    [ADCX_IA] = {.name = "M0_IA", .id = DRV_ADC_1, .channel = DRV_ADC_CH10},  /* PC0 SO1 */
    [ADCX_IB] = {.name = "M0_IB", .id = DRV_ADC_1, .channel = DRV_ADC_CH11},  /* PC1 SO2 */
    [ADCX_IC] = {.name = "M0_IC_PHANTOM", .id = DRV_ADC_1, .channel = DRV_ADC_CH10},  /* 占位, IC 由合成 */
};
#endif

#if defined(USE_DEV_COMMUN_UART)
const dev_commun_uart_config_t commun_uart_list[JM_UART_COMM_ID_MAX] = {
    [JM_UART_COMM_ID_1] = {
        .name = "JM_UART_1",
        .uart = DRV_UART4,
        .motor_id = 1,
    },
};
#endif

#endif /* JM_BOARD_CONFIG_DEFINE_TABLES */
```

- [ ] **Step 2: Commit**

```bash
git add Board/ODriveMKS/Config/dev_config_board.inc
git commit -m "feat(board): add ODrive dev_config_board.inc (pin/channel tables)"
```

---

### Task 20: main.c 调 dev_drv8301_init

**Files:**
- Modify: `Board/ODriveMKS/Core/Src/main.c` (CubeMX 生成后)

- [ ] **Step 1: 在 main.c 顶部 USER CODE BEGIN Includes 加 include**

```c
/* USER CODE BEGIN Includes */
#include "dev_drv8301.h"
/* USER CODE END Includes */
```

- [ ] **Step 2: 在 main() 的 USER CODE BEGIN 2 区段加 DRV8301 初始化**

在 `MX_GPIO_Init` / `MX_SPI3_Init` / `MX_ADC1_Init` / `MX_TIM1_Init` 之后、`user_init()` 之前插入：

```c
  /* USER CODE BEGIN 2 */
  dev_drv8301_init(&g_dev_drv8301, DRV8301_ID_1);
  g_dev_drv8301.init(&g_dev_drv8301);   /* SPI 配置 DRV8301 寄存器 */
  /* USER CODE END 2 */
```

- [ ] **Step 3: Commit（待 CubeMX 生成后）**

```bash
git add Board/ODriveMKS/Core/Src/main.c
git commit -m "feat(board): init DRV8301 before user_init in main.c"
```

---

### Task 21: IOC 配置 + CubeMX 重新生成 + Keil 工程 + 编译验证

**Files:**
- Modify: `Board/ODriveMKS/OdriveMini.ioc`
- Modify: `Board/ODriveMKS/MDK-ARM/*.uvprojx`

- [ ] **Step 1: 按 PORTING_TASK_BRIEF 清单修改 IOC**

关键项：
- **SPI3**: Full-Duplex Master, 16-bit Data Size, CPOL=Low, **CPHA=2 Edge (Mode 1)**, BaudRatePrescaler=32 (168MHz/32=5.25MHz), FirstBit=MSB。AS5047P 和 DRV8301 共用 SPI3，两者均需 Mode 1 + 16-bit。
- **ADC1 注入组**: 2 通道 IN10(PC0) + IN11(PC1)，触发源 T1_CC4，采样时间 47.5 cycles
- **TIM1**: CenterAligned1, **Period=8400** (168MHz/8400/2=10kHz), RCR=0, CH4 CCR=8380
- **PC13** 配 GPIO_Output（DRV8301 nSCS），初始 High
- **PB12** 配 GPIO_Output（EN_GATE），初始 Low
- **PD2** 配 GPIO_Input（nFAULT），上拉
- 禁用 USB_DEVICE / FDCAN1 / TIM8 / TIM3 / TIM4
- UART4 波特率 2000000

- [ ] **Step 2: CubeMX 重新生成代码到 Board/ODriveMKS/Core/**

Toolchain = MDK-ARM。

- [ ] **Step 3: Keil 工程配置**

- Device: STM32F405RGTx
- 预定义宏: `USE_HAL_DRIVER, STM32F405xx, JM_BOARD_ODRIVE`
- `FLASH_MCU=0x03`（在 drv_config.h 的配置向导中设置）
- 链接脚本: IROM1 start=0x08000000, size=0x0E0000（代码区限制在 Sector 11 前，896KB 给代码）
- 添加 User/ 新增文件到工程：drv_flash_f4.c, dev_as5047.c, dev_encoder_as5047.c, dev_drv8301.c

- [ ] **Step 4: 虚拟电机模式编译验证**

设 `MOTOR_LOOP_ENABLE_DEV_DRIVER=0`，编译。
Expected: 0 error。

- [ ] **Step 5: 真实驱动模式编译验证**

设 `MOTOR_LOOP_ENABLE_DEV_DRIVER=1`，编译。
Expected: 0 error。

- [ ] **Step 6: Commit**

```bash
git add Board/ODriveMKS/
git commit -m "build(board): Keil project configured for F405 + JM_BOARD_ODRIVE"
```

---

## 四、自检清单

### 4.1 Spec 覆盖核对

| 需求 | 对应 Task |
|---|---|
| board_select 加 ODrive | Task 1 |
| Flash 驱动 F4 路径 | Task 2, 4, 10, 11, 12 |
| 参数板级覆盖 | Task 3, 5, 18 |
| 2 路相电流 IC 合成 | Task 6, 18(DRV8301_TWO_PHASE_CURRENT) |
| AS5047 编码器 | Task 7, 8, 13, 14, 15 |
| DRV8301 设备对象 | Task 16, 17, 20 |
| 板级配置表 | Task 18, 19 |
| dev_config.c 新 includes | Task 9 |
| IOC + CubeMX + Keil | Task 21 |

### 4.2 类型/API 一致性

- `spiDrv_t` = {`spiNumber_e hspi`, `gpioDrv_t cs`}，cs={0} 时驱动不操作 CS（已从 drv_spi.h 确认）
- `drv_spi_transfer(spiDrv_t drv, uint8_t *tx, uint8_t *rx, uint16_t len, uint32_t timeout)` — len=1 对应 16-bit SPI 一次传输
- `drv_gpio_write(gpioDrv_t drv, drvPinState_e ste)` — 值传递（已从 drv_gpio.h 确认）
- `drv_gpio_read(gpioDrv_t drv)` → `drvPinState_e`
- `gpioDrv_t` = {`gpioType_e gpiox`, `gpioPin_e pin`, `drvPinState_e ste`}
- `dev_encoder_t` 全部 8 个方法指针字段（已从 dev_encoder.h 确认）
- `drv_flash_read/write/clear` 签名 G4/F4 对齐（已从 drv_flash_g4.h 确认）
- `dev_flash_t` 结构 + `WORD_SIZE=8` + `FLASH_FLAG_MAKE/IS_VALID/SEQ`（已从 dev_flash.h 确认）
- `DRV_ADC_CH6 / CH10 / CH11` 枚举存在（已从 drv_adc.h 确认）

### 4.3 与 v2 计划的优化点（v3 新增 8 项）

| # | 优化项 | 说明 |
|---|---|---|
| 12 | **HALF_BRIDGE_ADC_TRIG_CCR 覆盖** | v2 遗漏。ODrive TIM1 Period=8400, G4 默认 CCR=8480 > ARR → ADC 永不触发。Task 18 中 `#undef` + `#define (8380u)` 覆盖 |
| 13 | **PM_IBUS_SOURCE=1** | v2 遗漏。ODrive 无 IBUS 硬件，v2 中 PM_IBUS 占位复用 VBUS 通道会导致 IBUS 读数=VBUS 电压(无意义)。改为三相合成 |
| 14 | **AS5047P EF/parity 变量交换修复** | v2 代码 `parity_bit=(frame>>15)` 取的是 EF 不是 parity，`ef=(frame>>14)` 取的是 parity 不是 EF。v3 修正：`ef=(frame>>15)`, `parity_bit=(frame>>14)` |
| 15 | **AS5047P 奇偶校验范围修正** | v2 用 `frame & 0x7FFF`(bit14:0) 计算校验，含 parity 位本身导致逻辑错误。v3 改为 `frame & 0xBFFF`(bit15+bit13:0, 不含 bit14) |
| 16 | **FLASH_FLAG_ALL_ERRORS 宏补全** | v2 遗漏。F4 HAL 无此宏(G4 专有)，drv_flash_f4.c 编译会报错。v3 在 drv_flash_f4.h 中用 F4 的 6 个错误标志组合 `#ifndef` 补全 |
| 17 | **SPI3 配置明确写入 IOC 清单** | v2 Task 21 仅引用 PORTING_TASK_BRIEF，未明确 SPI3 需 16-bit Full-Duplex Mode 1。AS5047P/DRV8301 均需此模式，配错则 SPI 通信失败 |
| 18 | **drv8301_config_t 移除 en_gate 字段** | v2 中 en_gate 字段从未被 dev_drv8301.c 使用(EN_GATE 由 motor_enable_list 控制)。移除避免死数据误导 |
| 19 | **DRV8301 CTRL2 OCP 模式标注** | v2 注释仅写 "GATE_CURRENT=1.7A, 6PWM"，v3 补充 OCP=current limit mode(bit5:4=00)，方便硬件实测时调整 |

### 4.4 已知风险（需硬件实测）

1. **AS5047P 奇偶校验范围**：当前按 bit15+bit13:0 计算(不含 bit14)。若硬件实测校验失败，尝试改为仅 bit13:0(14bit) 或 bit15+bit14:0(含parity自校验=结果应为0)
2. **DRV8301 CTRL1/CTRL2 寄存器值**：`0x0400` / `0x0006` 需对照 datasheet 确认，特别是 OCP 模式(current limit vs latch-off)
3. **SPI3 共享时序**：AS5047P 和 DRV8301 共享 SPI3，nCS 隔离时序需确认无冲突
4. **PC13 引脚**：原 ioc 标号 M0_nCS（板载 AS5047 位置），现用于 DRV8301 nSCS，确认物理连通
5. **TIM1 死区时间**：IOC 中 TIM1 DeadTime 设为 0（DRV8301 内部处理死区），需确认 DRV8301 的 OCP 能否覆盖硬件死区保护

---

## 五、改动量汇总

| 类别 | 文件数 | 说明 |
|---|---|---|
| User/ 修改 | 8 | ~70 行（宏/条件编译/枚举分支/Flash 条件路径） |
| User/ 新增 | 8 | drv_flash_f4 + dev_as5047 + dev_encoder_as5047 + dev_drv8301 |
| Board/ODriveMKS/ | 多个 | CubeMX 生成 + 板级配置表 + main.c + Keil |
| **User/ 算法/业务逻辑** | **0** | **FOC/PID/SVPWM/motor_loop/协议栈零改动** |

---

**文档结束。**
