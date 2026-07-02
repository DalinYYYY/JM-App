# 电机参数配置文件统一与宏切换 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 创建单一电机参数配置文件 `motor_profile.h`，通过一个宏 `MOTOR_PROFILE` 切换不同电机型号，并将该配置同步到标定模块（calib_config.h）、控制环参数（motor_param.c）、协议持久化参数（motor_info.c）的默认值，消除占位值（R=0.1Ω 等非真实值）。

**Architecture:** 新增 `User/Config/motor_profile.h` 作为电机电气身份参数（R/Ld/Lq/flux/kt/pole_pairs/电流/电压/转速/转矩/惯量）的唯一真相源。该文件定义 `MOTOR_PROFILE` 切换宏与每个型号的 `MOTOR_*` 参数宏。三个消费方 `#include` 此文件并用 `MOTOR_*` 宏替换硬编码字面量：`motor_param.c` 的 `g_default_config.motor_base`、`motor_info.c` 的 `motor_info_init` 电机标定段、`calib_config.h` 的结果合理性范围（按 `MOTOR_*` 的 ±50% 容差派生）。切换电机只需改 `MOTOR_PROFILE` 一行。

**Tech Stack:** STM32 HAL（Keil MDK-ARM）、C99 预处理器宏、关节电机参数体系（motor_param/motor_info 双轨）。

---

## 背景与关键事实（实现前必读）

### 当前问题：电机参数散落在 3 处且为占位值

| 文件 | 载体 | 生成方式 | 当前 R/Ld/Lq/flux/pole_pairs 默认值 |
|---|---|---|---|
| [motor_param.c:27-32](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/DataHub/motor_param.c#L27-L32) | `g_default_config.motor_base` | 自动生成（CSV→py） | 0.1Ω / 0.1mH / 0.12mH / 0.001Wb / 7 |
| [motor_info.c:51-58](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/DataHub/motor_info.c#L51-L58) | `motor_info_init` 逐字段赋值 | 自动生成（CSV→py） | 0.1Ω / 0.1mH / 0.12mH / 0.001Wb / 7 |
| [calib_config.h:100-112](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorCalibration/calib_config.h#L100-L112) | `CALIB_CFG_*_MIN/MAX` 宏 | 手写 | 范围与电机无关（通用宽范围） |

**问题 1**：R=0.1Ω / L=0.1mH / flux=0.001Wb 是**通用占位值**，不是 GM4820H 真实值（PDF：R=3.6Ω, L=4.8mH, flux≈0.02Wb）。控制环用这些占位值会导致 FOC 电流环解耦失效。

**问题 2**：无电机型号切换机制。要换电机必须手动改 3 个文件的多处字面量，易遗漏。

### GM4820H 真实参数（来自 PDF）

| 参数 | 值 | 来源 |
|---|---|---|
| 极对数 | 7 | PDF: 12N14P |
| 相电阻 R | 3.6 Ω | PDF: Ri 3.6Ω |
| 相电感 L | 4.8 mH | PDF: 4.8mH/Q6.9 |
| 磁链 flux | 0.02 Wb | KV=66 反算: 60/(2π·66·7)≈0.0206 |
| 转矩常数 kt | 0.21 Nm/A | FOC 一致: 1.5·pp·flux = 1.5·7·0.02 |
| 堵转电流 | 3.7 A | PDF |
| 推荐电压 | 24 V | PDF: 7.4-24V |
| 空载转速 | 1550 RPM | PDF: 24V@1550RPM |
| 额定转矩 | 0.2 Nm | PDF |
| 反电动势常数 ke | 0.14 V/(rad/s) | = flux·pole_pairs |
| 绕组 | WYE，SPMSM（表贴式，Ld≈Lq） | PDF |

### 设计边界：motor_profile.h 只含电机电气身份

**放入 motor_profile.h**（电机本体参数，换电机时变）：
- 电气：R, Ld, Lq, flux, kt, ke, pole_pairs
- 额定：rated_current, peak_current, max_speed, rated_voltage, rated_speed_rpm, rated_torque, peak_torque, inertia

**不放入 motor_profile.h**（板级/编码器/减速器硬件，换电机不变）：
- `pwm_freq_hz` / `foc_freq_hz`（板级驱动器，留在 motor_param.c）
- `enc_lines` / `enc_type` / `enc_offset` / `enc_direction`（编码器硬件 MT6701，留在各文件）
- `dead_time_ns`（功率器件，留在各文件）
- `gear_ratio`（减速器，机械配置，留在各文件）
- PID 增益（调试参数，留在各文件）

### 自动生成文件的注意事项

`motor_param.c` 和 `motor_info.c` 头部标注"自动生成文件，请勿手动修改"。本计划仍需手动编辑这两个文件的**电机电气身份字段**，将其改为引用 `MOTOR_*` 宏。这意味着：
- 重新运行 CSV 生成脚本会覆盖这些改动
- 长期方案是更新生成脚本（`motor_param_generate_v9.py` / `motor_info_generate.py`）让其输出 `MOTOR_*` 宏，但**本计划不包含脚本修改**（超出范围）
- 作为权宜之计，编辑后在文件头注释追加一行说明"电机电气身份字段已改为 motor_profile.h 宏驱动，重新生成时需保留 `#include "motor_profile.h"` 与 `MOTOR_*` 引用"

---

## File Structure

| 文件 | 职责 | 改动类型 |
|---|---|---|
| `User/Config/motor_profile.h` | 电机电气身份参数唯一真相源 + `MOTOR_PROFILE` 切换宏 | **新建** |
| `User/MotorCalibration/calib_config.h` | 标定参数；结果合理性范围改为从 `MOTOR_*` 派生 | 修改 |
| `User/DataHub/motor_param.c` | `g_default_config.motor_base` 字段改用 `MOTOR_*` 宏 | 修改 |
| `User/DataHub/motor_info.c` | `motor_info_init` 电机标定电气字段改用 `MOTOR_*` 宏 | 修改 |

**不改动**：`motor_param.h`、`motor_info.h`、`calib_validate.h`、`calib_level2_motor.c`、`calib_level3_encoder.c`、CSV 生成脚本。

---

## Task 1: 创建 motor_profile.h

**Files:**
- Create: `User/Config/motor_profile.h`

- [ ] **Step 1: 创建 motor_profile.h 文件**

写入以下完整内容：

```c
#ifndef __MOTOR_PROFILE_H__
#define __MOTOR_PROFILE_H__

/* ===================== 电机参数配置文件（唯一真相源）=====================
 * 所有电机电气身份参数集中于此。三个消费方通过 #include 引用：
 *   - User/DataHub/motor_param.c   （控制环运行时默认值）
 *   - User/DataHub/motor_info.c    （协议持久化参数默认值）
 *   - User/MotorCalibration/calib_config.h（标定结果合理性范围）
 *
 * 切换电机型号：修改下面的 MOTOR_PROFILE 宏定义为对应的型号编号。
 * 新增电机型号：在下面追加 #define MOTOR_PROFILE_XXX N，并补一个
 *               #elif 分支填写该型号的全部 MOTOR_* 参数。
 *
 * 说明：本文件只含电机电气身份参数（换电机时变的量）。
 *       板级参数（pwm_freq/enc_lines/dead_time）、减速器、PID 增益
 *       不在此文件，留在各自原文件。
 */

/* ===================== 电机型号选择（修改此行切换）===================== */
#define MOTOR_PROFILE_GM4820H 1
#define MOTOR_PROFILE_DEMO    2 /* 示例占位，演示多型号切换 */
#define MOTOR_PROFILE         MOTOR_PROFILE_GM4820H

/* ===================== 各型号参数 ===================== */
#if MOTOR_PROFILE == MOTOR_PROFILE_GM4820H
/* GM4820H 无刷云台电机（参数来源：GM4820H参数_2024.pdf）
 * 结构 12N14P / WYE / SPMSM（表贴式，Ld≈Lq）
 * 时间常数 τ = L/R = 4.8mH/3.6Ω = 1.33ms */
#define MOTOR_NAME            "GM4820H"
#define MOTOR_R               3.6f      /* 相电阻(Ω) PDF: Ri */
#define MOTOR_LD              4.8e-3f   /* d轴电感(H) PDF: 4.8mH */
#define MOTOR_LQ              4.8e-3f   /* q轴电感(H) SPMSM: Ld≈Lq */
#define MOTOR_FLUX            0.02f     /* 磁链(Wb) KV=66反算: 60/(2π·66·7) */
#define MOTOR_KT              0.21f     /* 转矩常数(Nm/A) = 1.5·pp·flux */
#define MOTOR_KE              0.14f     /* 反电动势常数(V/(rad/s)) = flux·pp */
#define MOTOR_POLE_PAIRS      7         /* 极对数 PDF: 12N14P */
#define MOTOR_RATED_CURRENT   0.6f      /* 额定电流(A) 保守 */
#define MOTOR_PEAK_CURRENT    3.7f      /* 峰值电流(A) PDF: 堵转 */
#define MOTOR_MAX_SPEED       200.0f    /* 最大转速(rad/s) ~1900RPM */
#define MOTOR_RATED_VOLTAGE   24.0f     /* 额定电压(V) PDF: 推荐 */
#define MOTOR_RATED_SPEED_RPM 1550.0f   /* 额定转速(rpm) PDF: 24V */
#define MOTOR_RATED_TORQUE    0.2f      /* 额定转矩(Nm) PDF */
#define MOTOR_PEAK_TORQUE     0.5f      /* 峰值转矩(Nm) */
#define MOTOR_INERTIA         1e-5f     /* 转子惯量(kg·m²) 估算 */

#elif MOTOR_PROFILE == MOTOR_PROFILE_DEMO
/* 示例：演示如何添加第二个电机型号（占位，非真实参数）*/
#define MOTOR_NAME            "DEMO"
#define MOTOR_R               1.0f
#define MOTOR_LD              1e-3f
#define MOTOR_LQ              1e-3f
#define MOTOR_FLUX            0.01f
#define MOTOR_KT              0.1f
#define MOTOR_KE              0.07f
#define MOTOR_POLE_PAIRS      7
#define MOTOR_RATED_CURRENT   1.0f
#define MOTOR_PEAK_CURRENT    5.0f
#define MOTOR_MAX_SPEED       300.0f
#define MOTOR_RATED_VOLTAGE   24.0f
#define MOTOR_RATED_SPEED_RPM 2000.0f
#define MOTOR_RATED_TORQUE    1.0f
#define MOTOR_PEAK_TORQUE     3.0f
#define MOTOR_INERTIA         1e-5f

#else
#error "未知 MOTOR_PROFILE，请在 motor_profile.h 中定义有效的电机型号编号"
#endif

#endif /* __MOTOR_PROFILE_H__ */
```

- [ ] **Step 2: 验证文件已创建**

Run: 用 Read 工具读取 `d:\AAWorkSpace\001_JointMotor\SW\JointMotor\User\Config\motor_profile.h`
Expected: 文件存在，内容与上面一致，`MOTOR_PROFILE` 定义为 `MOTOR_PROFILE_GM4820H`

- [ ] **Step 3: 不提交**

---

## Task 2: 重构 calib_config.h，结果合理性范围从 MOTOR_* 派生

**Files:**
- Modify: `User/MotorCalibration/calib_config.h:1-2`（加 include）
- Modify: `User/MotorCalibration/calib_config.h:100-112`（结果合理性范围段）

- [ ] **Step 1: 在 calib_config.h 顶部 include motor_profile.h**

用 Edit，old_string：
```c
#ifndef __CALIB_CONFIG_H__
#define __CALIB_CONFIG_H__

/* ===================== 标定集中配置 =====================
```

new_string：
```c
#ifndef __CALIB_CONFIG_H__
#define __CALIB_CONFIG_H__

#include "motor_profile.h" /* 电机电气身份参数（R/Ld/Lq/flux/pole_pairs），用于派生标定结果合理性范围 */

/* ===================== 标定集中配置 =====================
```

- [ ] **Step 2: 替换结果合理性范围段为从 MOTOR_* 派生**

用 Edit，old_string（匹配当前 100-112 行）：
```c
/* ===================== 结果合理性范围（calib_validate.h 用）=====================
 * 按 GM4820H 参数（R=3.6Ω, L=4.8mH, flux=0.017Wb, pp=7）设定，
 * 覆盖云台/关节类电机典型范围，兼顾误测拦截与误触发规避。*/
#define CALIB_CFG_R_MIN_OHM      0.1f    /* R 相电阻下限(Ω)，拦截短路(<0.1Ω) */
#define CALIB_CFG_R_MAX_OHM      50.0f   /* R 相电阻上限(Ω)，拦截断路(>50Ω) */
#define CALIB_CFG_LD_MIN_H       0.1e-3f /* Ld d轴电感下限(H) = 0.1mH */
#define CALIB_CFG_LD_MAX_H       50e-3f  /* Ld d轴电感上限(H) = 50mH */
#define CALIB_CFG_LQ_MIN_H       0.1e-3f /* Lq q轴电感下限(H) = 0.1mH */
#define CALIB_CFG_LQ_MAX_H       50e-3f  /* Lq q轴电感上限(H) = 50mH */
#define CALIB_CFG_FLUX_MIN_WB    1e-3f   /* flux 磁链下限(Wb) = 1mWb */
#define CALIB_CFG_FLUX_MAX_WB    0.1f    /* flux 磁链上限(Wb) = 100mWb */
#define CALIB_CFG_POLE_PAIRS_MIN 1       /* 极对数下限 */
#define CALIB_CFG_POLE_PAIRS_MAX 14      /* 极对数上限（覆盖 2N-28P 电机）*/
```

new_string：
```c
/* ===================== 结果合理性范围（calib_validate.h 用）=====================
 * 从 motor_profile.h 的 MOTOR_* 参数派生，容差 ±50%。
 * 切换电机型号时自动适配，无需手动调整。
 * 拦截短路/断路/异常值，兼顾误测拦截与误触发规避。*/
#define CALIB_CFG_R_MIN_OHM      (MOTOR_R * 0.5f)       /* R 下限 = 标称×0.5 */
#define CALIB_CFG_R_MAX_OHM      (MOTOR_R * 2.0f)       /* R 上限 = 标称×2.0 */
#define CALIB_CFG_LD_MIN_H       (MOTOR_LD * 0.5f)      /* Ld 下限 */
#define CALIB_CFG_LD_MAX_H       (MOTOR_LD * 2.0f)      /* Ld 上限 */
#define CALIB_CFG_LQ_MIN_H       (MOTOR_LQ * 0.5f)      /* Lq 下限 */
#define CALIB_CFG_LQ_MAX_H       (MOTOR_LQ * 2.0f)      /* Lq 上限 */
#define CALIB_CFG_FLUX_MIN_WB    (MOTOR_FLUX * 0.5f)    /* flux 下限 */
#define CALIB_CFG_FLUX_MAX_WB    (MOTOR_FLUX * 2.0f)    /* flux 上限 */
#define CALIB_CFG_POLE_PAIRS_MIN 1                      /* 极对数下限（通用）*/
#define CALIB_CFG_POLE_PAIRS_MAX 14                     /* 极对数上限（覆盖 2N-28P）*/
```

- [ ] **Step 3: 编译验证 calib_config.h 语法**

Run: `& "C:\APP\Code\MDK\CORE\UV4\UV4.exe" -b "d:\AAWorkSpace\001_JointMotor\SW\JointMotor\Board\SFOC\MDK-ARM\sfoc.uvprojx" -o "d:\AAWorkSpace\001_JointMotor\SW\JointMotor\Board\SFOC\MDK-ARM\build_t2.log" -j0`
Expected: 0 Error（motor_profile.h 在 User/Config 目录，需确认 include 路径已包含。若报找不到 motor_profile.h，需检查 Keil 工程 include 路径是否含 User/Config）

- [ ] **Step 4: 若编译报 "cannot find motor_profile.h"**

检查 Keil 工程 include 路径：用 Grep 搜索 `User/Config` 是否在 .uvprojx 的 IncludePath 中。
若缺失，需在 Keil IDE 中添加 `..\..\User\Config` 到 IncludePath（两个工程都要加）。
验证命令：用 Grep 在 .uvprojx 文件搜 `User\\Config` 或 `User/Config`。

- [ ] **Step 5: 不提交**

---

## Task 3: 更新 motor_param.c，motor_base 字段改用 MOTOR_* 宏

**Files:**
- Modify: `User/DataHub/motor_param.c:12-14`（加 include）
- Modify: `User/DataHub/motor_param.c:25-45`（motor_base 初始化）

- [ ] **Step 1: 在 motor_param.c 加 include motor_profile.h**

用 Edit，old_string：
```c
#include "motor_param.h"
#include <stdio.h>
#include <string.h>
#include <errno.h>
```

new_string：
```c
#include "motor_param.h"
#include "motor_profile.h" /* 电机电气身份参数（R/Ld/Lq/flux 等）宏驱动 */
#include <stdio.h>
#include <string.h>
#include <errno.h>
```

- [ ] **Step 2: 替换 g_default_config.motor_base 段**

用 Edit，old_string（匹配 25-45 行）：
```c
		.motor_base =
			{
							 .r = 0.1f,
							 .ld = 0.0001f,
							 .lq = 0.00012f,
							 .flux = 0.001f,
							 .kt = 0.1f,
							 .pole_pairs = 7,
							 .rated_current = 0.6f,
							 .peak_current = 1.50f,
							 .max_speed = 2000.0f,
							 .dead_time_ns = 500.0f,
							 .rated_voltage = 12.0f,
							 .rated_speed_rpm = 1000.0f,
							 .rated_torque = 2.0f,
							 .peak_torque = 6.0f,
							 .inertia = 1e-05f,
							 .ke = 0.01f,
							 .pwm_freq_hz = 10000,
							 .foc_freq_hz = 10000,
							 },
```

new_string：
```c
		.motor_base =
			{
							 .r = MOTOR_R,
							 .ld = MOTOR_LD,
							 .lq = MOTOR_LQ,
							 .flux = MOTOR_FLUX,
							 .kt = MOTOR_KT,
							 .pole_pairs = MOTOR_POLE_PAIRS,
							 .rated_current = MOTOR_RATED_CURRENT,
							 .peak_current = MOTOR_PEAK_CURRENT,
							 .max_speed = MOTOR_MAX_SPEED,
							 .dead_time_ns = 500.0f,
							 .rated_voltage = MOTOR_RATED_VOLTAGE,
							 .rated_speed_rpm = MOTOR_RATED_SPEED_RPM,
							 .rated_torque = MOTOR_RATED_TORQUE,
							 .peak_torque = MOTOR_PEAK_TORQUE,
							 .inertia = MOTOR_INERTIA,
							 .ke = MOTOR_KE,
							 .pwm_freq_hz = 10000,
							 .foc_freq_hz = 10000,
							 },
```

**注意**：`dead_time_ns`、`pwm_freq_hz`、`foc_freq_hz` 保留原字面量（板级参数，非电机身份）。

- [ ] **Step 3: 在文件头注释追加宏驱动说明**

用 Edit，old_string：
```c
 * @warning 【自动生成文件，请勿手动修改】
 *          本文件由脚本 generate_config_header_v9.py 根据配置表自动生成，
 *          任何手动改动都会在下次运行脚本时被覆盖。
 *          如需修改参数定义，请编辑源 CSV 配置表后重新生成。
 */
```

new_string：
```c
 * @warning 【自动生成文件，请勿手动修改】
 *          本文件由脚本 generate_config_header_v9.py 根据配置表自动生成，
 *          任何手动改动都会在下次运行脚本时被覆盖。
 *          如需修改参数定义，请编辑源 CSV 配置表后重新生成。
 *
 * @note   motor_base 段的电机电气身份字段（R/Ld/Lq/flux/kt/pole_pairs/
 *         rated_current/peak_current/max_speed/rated_voltage/rated_speed_rpm/
 *         rated_torque/peak_torque/inertia/ke）已改为 motor_profile.h 宏驱动。
 *         重新生成本文件时需保留 #include "motor_profile.h" 与 MOTOR_* 引用，
 *         或更新生成脚本让其输出 MOTOR_* 宏。
 */
```

- [ ] **Step 4: 编译验证**

Run: sfoc 工程编译
Expected: 0 Error

- [ ] **Step 5: 不提交**

---

## Task 4: 更新 motor_info.c，电机标定电气字段改用 MOTOR_* 宏

**Files:**
- Modify: `User/DataHub/motor_info.c:12-14`（加 include）
- Modify: `User/DataHub/motor_info.c:50-59`（motor_calib 电气字段）

- [ ] **Step 1: 在 motor_info.c 加 include motor_profile.h**

用 Edit，old_string（匹配 motor_info.c 顶部的 include 段，先 Read 确认确切内容）：

先 Read `d:\AAWorkSpace\001_JointMotor\SW\JointMotor\User\DataHub\motor_info.c` 第 1-16 行确认 include 段。

然后在 `#include "motor_info.h"` 后追加 `#include "motor_profile.h"`。

- [ ] **Step 2: 替换 motor_calib 电气字段段**

用 Edit，old_string（匹配 50-59 行）：
```c
	cfg->blocks.motor_calib.is_calibrated = 0U;
	cfg->blocks.motor_calib.pole_pairs = 7U;
	cfg->blocks.motor_calib.motor_type = 0U;
	cfg->blocks.motor_calib.direction = 0U;
	cfg->blocks.motor_calib.phase_resistance = 0.1f;
	cfg->blocks.motor_calib.phase_inductance_d = 0.0001f;
	cfg->blocks.motor_calib.phase_inductance_q = 0.00012f;
	cfg->blocks.motor_calib.flux_linkage = 0.001f;
	cfg->blocks.motor_calib.torque_constant = 0.1f;
	cfg->blocks.motor_calib.rotor_inertia = 0.00001f;
```

new_string：
```c
	cfg->blocks.motor_calib.is_calibrated = 0U;
	cfg->blocks.motor_calib.pole_pairs = (uint32_t)MOTOR_POLE_PAIRS;
	cfg->blocks.motor_calib.motor_type = 0U;
	cfg->blocks.motor_calib.direction = 0U;
	cfg->blocks.motor_calib.phase_resistance = MOTOR_R;
	cfg->blocks.motor_calib.phase_inductance_d = MOTOR_LD;
	cfg->blocks.motor_calib.phase_inductance_q = MOTOR_LQ;
	cfg->blocks.motor_calib.flux_linkage = MOTOR_FLUX;
	cfg->blocks.motor_calib.torque_constant = MOTOR_KT;
	cfg->blocks.motor_calib.rotor_inertia = MOTOR_INERTIA;
```

**注意**：`motor_type`、`direction`、`is_calibrated` 保留原值（非电机身份）。后续 `gear_ratio`、`enc_*`、`pwm_freq_hz`、`dead_time_ns`、`shunt_resistance`、`current_amp_gain` 等保留原值（板级/编码器/硬件参数）。

- [ ] **Step 3: 在文件头注释追加宏驱动说明**

用 Edit，在 motor_info.c 的 `@warning` 段后追加 `@note`（与 Task 3 Step 3 同样模式）：

old_string：
```c
 * @warning 【自动生成文件，请勿手动修改】
```
（先 Read 确认 motor_info.c 头部确切注释文本）

new_string：在对应 @warning 段后追加：
```c
 *
 * @note   motor_calib 段的电机电气身份字段（pole_pairs/phase_resistance/
 *         phase_inductance_d/phase_inductance_q/flux_linkage/torque_constant/
 *         rotor_inertia）已改为 motor_profile.h 宏驱动。
 *         重新生成本文件时需保留 #include "motor_profile.h" 与 MOTOR_* 引用。
 */
```

- [ ] **Step 4: 编译验证**

Run: sfoc 工程编译
Expected: 0 Error。注意 `MOTOR_POLE_PAIRS` 是 int，赋值给 uint32_t 需强制转换（已在代码中 `(uint32_t)`）

- [ ] **Step 5: 不提交**

---

## Task 5: 确认 Keil 工程 include 路径含 User/Config

**Files:**
- Check: `Board/SFOC/MDK-ARM/sfoc.uvprojx`
- Check: `Board/V1/MDK-ARM/JointMotorApp.uvprojx`

- [ ] **Step 1: 检查 sfoc 工程 include 路径**

用 Grep 在 `Board/SFOC/MDK-ARM/sfoc.uvprojx` 搜 `User\\Config` 或 `User/Config` 或 `Config`：
- 若已存在 → 跳过 Step 2
- 若不存在 → 执行 Step 2

- [ ] **Step 2: 若缺失，在 Keil IDE 中添加 User/Config 到 include 路径**

由于 .uvprojx 是 XML，直接编辑易出错。推荐：
1. 告知用户在 Keil IDE 中打开工程 → Options for Target → C/C++ → Include Paths → 添加 `..\..\User\Config`
2. 或用 Edit 工具在 .uvprojx 的 `<IncludePath>` 标签末尾追加 `;..\..\User\Config`（需先 Read 确认当前 IncludePath 内容与相对路径基准）

两个工程（sfoc + V1）都要检查并添加。

- [ ] **Step 3: 编译两个工程验证 include 路径生效**

Run: 两个工程并行编译
Expected: 0 Error（无 "cannot find motor_profile.h"）

- [ ] **Step 4: 不提交**

---

## Task 6: 双工程编译验证 + 宏展开验证

**Files:**
- 无文件改动，仅验证

- [ ] **Step 1: 编译 sfoc 工程**

Run: `& "C:\APP\Code\MDK\CORE\UV4\UV4.exe" -b "d:\AAWorkSpace\001_JointMotor\SW\JointMotor\Board\SFOC\MDK-ARM\sfoc.uvprojx" -o "d:\AAWorkSpace\001_JointMotor\SW\JointMotor\Board\SFOC\MDK-ARM\build_t6.log" -j0`
Expected: 0 Error, ≤14 Warning（与基线一致）

- [ ] **Step 2: 编译 V1 工程**

Run: `& "C:\APP\Code\MDK\CORE\UV4\UV4.exe" -b "d:\AAWorkSpace\001_JointMotor\SW\JointMotor\Board\V1\MDK-ARM\JointMotorApp.uvprojx" -o "d:\AAWorkSpace\001_JointMotor\SW\JointMotor\Board\V1\MDK-ARM\build_t6.log" -j0`
Expected: 4 Error（预存 motor_info_* 链接错误，与基线一致），0 个 motor_profile 相关错误

- [ ] **Step 3: 验证宏切换生效（手动验证）**

临时将 motor_profile.h 的 `#define MOTOR_PROFILE MOTOR_PROFILE_GM4820H` 改为 `MOTOR_PROFILE_DEMO`，重新编译 sfoc：
Expected: 0 Error（DEMO 分支参数不同但都能编译通过，证明切换宏生效）

验证完成后改回 `MOTOR_PROFILE_GM4820H`。

- [ ] **Step 4: 删除中间 log 文件**

用 DeleteFile 删除 `build_t2.log`、`build_t6.log`（两个工程各一个）。

- [ ] **Step 5: 不提交（用户审核后自己 add）**

---

## Self-Review

**1. Spec coverage:**
- "单独用一个配置文件保存" → Task 1 创建 motor_profile.h ✅
- "同步到标定" → Task 2 calib_config.h 派生 validate 范围 ✅
- "同步到电机参数默认配置" → Task 3 motor_param.c motor_base 用 MOTOR_* ✅
- "同步到 motor_info 默认配置" → Task 4 motor_info.c motor_calib 用 MOTOR_* ✅
- "通过一个宏切换不同电机参数" → Task 1 MOTOR_PROFILE 宏 + Task 6 Step 3 验证切换 ✅
- Keil include 路径 → Task 5 ✅

**2. Placeholder scan:**
- 无 TBD/TODO
- 无"add error handling"——每个 Edit 都有确切 old/new_string
- Task 4 Step 1 / Step 3 标注"先 Read 确认"是因为 motor_info.c 头部注释确切文本未在探索中完整捕获，需实现时读取——这是合理的（避免猜测文本导致 Edit 失败），不算占位符

**3. Type consistency:**
- `MOTOR_R` / `MOTOR_LD` / `MOTOR_LQ` / `MOTOR_FLUX` / `MOTOR_KT` / `MOTOR_KE` — float，Task 1 定义，Task 2/3/4 使用一致
- `MOTOR_POLE_PAIRS` — int，Task 1 定义；Task 3 赋值给 `uint8_t pole_pairs`（隐式转换 OK）；Task 4 赋值给 `uint32_t`（显式 `(uint32_t)` 转换）
- `MOTOR_RATED_CURRENT` / `MOTOR_PEAK_CURRENT` / `MOTOR_MAX_SPEED` / `MOTOR_RATED_VOLTAGE` / `MOTOR_RATED_SPEED_RPM` / `MOTOR_RATED_TORQUE` / `MOTOR_PEAK_TORQUE` / `MOTOR_INERTIA` — float，Task 1 定义，Task 3 使用一致
- `calib_validate.h` 的 `calib_validate_r(ld/lq/flux/pole_pairs)` 签名不变，仅范围常量值改变 ✅

**已知限制与后续工作：**
1. **CSV 生成脚本未同步**：重新运行 `motor_param_generate_v9.py` / `motor_info_generate.py` 会覆盖 Task 3/4 的改动。后续需更新脚本让其输出 `MOTOR_*` 宏（超出本计划范围）。
2. **板级参数不一致未修复**：motor_param.c 与 motor_info.c 在 `enc_lines`（4000 vs 16384）、`enc_type`（0 vs 1）、`pwm_freq_hz`（10000 vs 20000）存在不一致，这些是板级/编码器参数，不在 motor_profile.h 范围内，需单独任务处理。
3. **标定测试电压未派生**：calib_config.h 的 L2 测试电压（R=1.5V, Ld=2.0V 等）仍为固定值，未从 MOTOR_* 派生。这些是标定过程参数（非电机身份），保留固定值更清晰。切换电机型号时若需调测试电压，手动改 calib_config.h 即可。
4. **DEMO 型号参数为占位**：motor_profile.h 的 MOTOR_PROFILE_DEMO 分支是演示用，非真实电机参数。

---

## Execution Handoff

**Plan complete and saved to `User/Data/plans/2026-07-02-motor-profile-config.md`. Two execution options:**

**1. Subagent-Driven（推荐）** - 每个 Task 派新 subagent，任务间 review，快速迭代

**2. Inline Execution** - 当前会话执行，批量+检查点

选哪种？
