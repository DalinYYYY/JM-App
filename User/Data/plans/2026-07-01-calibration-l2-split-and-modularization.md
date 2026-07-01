# 标定 L2 子模式拆分与模块化重构实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将 L2 电机电气身份标定中"R/Ld/Lq/flux 辨识"（当前合并为单一子模式 3）拆分为 4 个独立子模式（R / Ld / Lq / flux），并提取共享硬件访问层 `calib_hw.c/h` 与集中配置文件 `calib_config.h`，降低模块耦合、消除散落的文件静态全局变量，为后续填充 L1/L2 真实算法做好结构准备（不实现真实算法）。

**Architecture:** 三条独立改动线：(1) 新增 `calib_config.h` 集中所有标定可调参数（电压/时间/采样数）；(2) 新增 `calib_hw.c/h` 将"标定电压会话"（替换电角度回调、施加 dq 电压、读编码器）封装为 `calib_hw_session_t` 结构体 + 配套函数，L3/L2 共享；(3) `calib_types.h` 将 `CALIB_L2_RL_FLUX=3` 拆为 `CALIB_L2_RESISTANCE=3` / `CALIB_L2_INDUCTANCE_D=4` / `CALIB_L2_INDUCTANCE_Q=5` / `CALIB_L2_FLUX_LINKAGE=6`，`calib_level2_motor.c` 与 `calib_level7_auto.c` 同步更新。协议层无需改动（submode 已是 payload[0] 透传）。

**Tech Stack:** STM32G4 HAL, 嵌入式 C (C99), Keil MDK-ARM, C-OOP ops 函数表模式, 现有 jm_proto 协议栈

---

## 结构影响分析（响应需求第 2 条）

用户明确"其他标定模式暂时不用代码实现，但要考虑增加这些标定模式是否会影响软件结构"。下表逐项分析每个未实现子模式未来填充真实算法时所需的改动面：

| 未来要实现的子模式 | 需要新增/修改的文件 | 是否需要动协议层/管理器 | 结构准备动作 |
|---|---|---|---|
| L1.1 ADC偏置 | `calib_level1_driver.c`（填充 `poll_adc_offset`）+ 读 `motor->phase_current.calibrate_offset` | 否 | 无（桩已就位） |
| L1.2 ADC增益 | `calib_level1_driver.c` + `calib_config.h` 加增益标定参数 | 否 | 无 |
| L1.3 电流传感器 | 同 L1.1（`dev_phase_current_t.calibrate_offset`） | 否 | 无 |
| L1.4 温度传感器 | `calib_level1_driver.c` + 读 `dev_power_monitor.get_temp_*` | 否 | 无 |
| L1.5 母线电压 | `calib_level1_driver.c` + 读 `dev_power_monitor.get_vbus` | 否 | 无 |
| L1.6 死区特性 | `calib_level1_driver.c` + `calib_hw_apply_voltage` 施加测试电压 | 否 | **需 calib_hw**（Task 2） |
| L2.1 相序识别 | `calib_level2_motor.c` + `calib_hw_apply_voltage` | 否 | **需 calib_hw** |
| L2.2 极对数 | `calib_level2_motor.c` + `calib_hw_apply_voltage` | 否 | **需 calib_hw** |
| L2.3 R 辨识 | `calib_level2_motor.c` + `calib_hw_apply_voltage` + `motor_param_set_r` | 否 | **需 calib_hw** + 拆分（Task 4/5） |
| L2.4 Ld 辨识 | 同上，写 `motor_param_set_ld` | 否 | 同上 |
| L2.5 Lq 辨识 | 同上，写 `motor_param_set_lq` | 否 | 同上 |
| L2.6 flux 辨识 | 同上，写 `motor_param_set_flux` + 需电机转动 | 否 | 同上 |

**结论：** 协议层（`jm_proto_ops.c` / `jm_cmd_def.h`）和管理器（`calib_mgr.c`）完全无需改动——submode 是 payload[0] 透传，新子模式只需在 `calib_types.h` 加 `#define` + 对应 level 文件加 `case`。唯一结构性缺口是 **L2/L1 的电压施加类标定需要 L3 现有的 `apply_voltage` / `calib_enter` / `calib_exit` 辅助函数，但这些函数当前是 `calib_level3_encoder.c` 的 `static` 函数无法共享**。本计划 Task 2 将其提取为 `calib_hw.c/h`，一次性补齐这个结构缺口。

---

## 文件结构

### 新增文件

| 文件 | 职责 |
|------|------|
| `User/MotorCalibration/calib_config.h` | 集中所有标定可调参数（电压/时间/采样数），各 level 模块 `#include` 引用，避免魔法数散落 |
| `User/MotorCalibration/calib_hw.h` | 标定硬件访问层接口：`calib_hw_session_t` 结构体 + enter/exit/apply_voltage/get_encoder_* 声明 |
| `User/MotorCalibration/calib_hw.c` | 标定硬件访问层实现：封装"替换电角度回调 + 施加 dq 电压 + 读编码器"为可重用会话 |

### 修改文件

| 文件 | 修改内容 |
|------|----------|
| `User/MotorCalibration/calib_types.h` | L2 子模式段：删除 `CALIB_L2_RL_FLUX`，新增 4 个常量（R/Ld/Lq/flux） |
| `User/MotorCalibration/calib_level2_motor.c` | start/poll switch 扩展为 6 个 case；新增 4 个桩 `poll_*()`；引入 `calib_hw_session_t` 字段备用 |
| `User/MotorCalibration/calib_level3_encoder.c` | 删除文件内 `apply_voltage`/`calib_enter`/`calib_exit`/`s_orig_ele_cb`/`s_calib_ele_angle`，改用 `calib_hw_*`；魔法数改用 `calib_config.h`；剩余状态合并为单一 `s_l3` 结构体 |
| `User/MotorCalibration/calib_level7_auto.c` | `s_sequence[]` 中 `{CALIB_LEVEL2_MOTOR, CALIB_L2_RL_FLUX}` 一行拆为 4 行（R/Ld/Lq/flux） |
| `User/MotorCalibration/docs/标定状态机说明.md` | L2 子模式数 3→6；新增 calib_hw/calib_config 文件说明；扩展指南补充新子模式 |
| `Board/V1/MDK-ARM/JointMotorApp.uvprojx` | MotorCalibration 组新增 `calib_hw.c` 文件条目 |
| `Board/SFOC/MDK-ARM/sfoc.uvprojx` | 同上 |

### 不修改的文件（重要：证明结构稳定）

- `calib_mgr.c` / `calib_mgr.h` — 管理器逻辑不变，ops 函数表签名不变
- `calib_level1_driver.c` / `calib_level4_torque.c` / `calib_level5_nonlinear.c` / `calib_level6_system.c` — 保持现有桩实现
- `jm_proto_ops.c` / `jm_cmd_def.h` — 协议层完全不变（submode 透传）
- `system_state.c` / `system_state.h` — 状态机集成点不变
- `motor_param.h` / `motor_param.c` — 参数结构不变（R/Ld/Lq/flux setter 已存在）

---

## 验证策略（嵌入式无单元测试框架）

本项目为 Keil MDK-ARM 嵌入式固件，无 pytest/host 测试框架。每个任务的验证 = **编译通过（0 error，0 新增 warning）+ 行为保持（重构前后 diff 审查）**。最终 Task 9 做双工程全量编译验证。

---

### Task 1: 创建 calib_config.h 集中配置文件

**Files:**
- Create: `User/MotorCalibration/calib_config.h`

**目的：** 响应需求第 3 条"为标定增加一个配置文件，统一管理标定的配置"。将散落在各 level 模块 `#define` 中的电压/时间/采样数魔法数集中到一个头文件，后续调参只需改一处。

- [ ] **Step 1: 创建 calib_config.h**

```c
#ifndef __CALIB_CONFIG_H__
#define __CALIB_CONFIG_H__

/* ===================== 标定集中配置 =====================
 * 所有标定可调参数（电压/时间/采样数）集中于此。
 * 各 level 模块 #include 引用，避免魔法数散落。
 * 修改参数只需改本文件，无需动算法源码。
 *
 * 时间相关 TICK 数约定：控制环频率 10kHz（dt=100us），
 * 秒数 × 10000 得到 TICK。若控制环频率改动，
 * 仅需修改下面的 CALIB_TICKS_PER_SEC 宏。*/

#define CALIB_TICKS_PER_SEC 10000.0f

/* ===================== L1 驱动硬件底层参数（预留）===================== */
#define CALIB_CFG_L1_ADC_OFFSET_SAMPLES  1000  /* ADC偏置采样次数（取平均）*/
#define CALIB_CFG_L1_VBUS_SAMPLE_COUNT   500   /* 母线电压采样次数 */

/* ===================== L2 电机电气身份参数（预留）===================== */
#define CALIB_CFG_L2_R_TEST_VOLTAGE_V    0.5f   /* R辨识：施加的DC测试电压(V) */
#define CALIB_CFG_L2_R_TEST_TIME_S       1.0f   /* R辨识：稳态等待时间(s) */
#define CALIB_CFG_L2_R_SAMPLE_COUNT      200    /* R辨识：电流采样次数 */
#define CALIB_CFG_L2_LD_TEST_VOLTAGE_V   2.0f   /* Ld辨识：d轴阶跃电压(V) */
#define CALIB_CFG_L2_LD_TEST_TIME_S      0.005f /* Ld辨识：阶跃持续时间(s)，观测di/dt */
#define CALIB_CFG_L2_LQ_TEST_VOLTAGE_V   2.0f   /* Lq辨识：q轴阶跃电压(V) */
#define CALIB_CFG_L2_LQ_TEST_TIME_S      0.005f /* Lq辨识：阶跃持续时间(s) */
#define CALIB_CFG_L2_FLUX_SPIN_VOLTAGE_V 3.0f   /* flux辨识：驱动电压(V) */
#define CALIB_CFG_L2_FLUX_SPIN_TIME_S    2.0f   /* flux辨识：稳态转动时间(s) */
#define CALIB_CFG_L2_FLUS_SPEED_RAD_S    10.0f  /* flux辨识：目标转速(rad/s) */

/* ===================== L3 编码器校准参数（已实现，从 calib_level3_encoder.c 迁移）===================== */
#define CALIB_CFG_L3_ALIGN_VOLTAGE_V     1.5f   /* d轴对齐电压(V) */
#define CALIB_CFG_L3_ALIGN_TIME_S        2.0f   /* 对齐稳定等待时间(s) */
#define CALIB_CFG_L3_ALIGN_TICKS         (uint32_t)(CALIB_CFG_L3_ALIGN_TIME_S * CALIB_TICKS_PER_SEC)
#define CALIB_CFG_L3_SAMPLE_COUNT        100    /* 零位标定采样次数（取平均滤波）*/
#define CALIB_CFG_L3_DIR_VOLTAGE_V       0.5f   /* 方向测试uq电压(V) */
#define CALIB_CFG_L3_DIR_TIME_S          1.0f   /* 方向测试持续时间(s) */
#define CALIB_CFG_L3_DIR_TICKS           (uint32_t)(CALIB_CFG_L3_DIR_TIME_S * CALIB_TICKS_PER_SEC)

#endif /* __CALIB_CONFIG_H__ */
```

- [ ] **Step 2: 验证头文件可被独立包含**

Run（在 Keil 工程中临时任意 .c 文件加 `#include "calib_config.h"` 后编译，或直接做全量编译预检）:
- 此步无独立命令，Task 9 会做全量编译验证。本步仅确认文件已创建。

- [ ] **Step 3: Commit**

```bash
git add User/MotorCalibration/calib_config.h
git commit -m "feat(calib): add calib_config.h centralizing calibration tunable params"
```

---

### Task 2: 创建 calib_hw.h / calib_hw.c 共享硬件访问层

**Files:**
- Create: `User/MotorCalibration/calib_hw.h`
- Create: `User/MotorCalibration/calib_hw.c`

**目的：** 响应需求第 2、3 条。当前 `calib_level3_encoder.c` 中的 `apply_voltage`/`calib_enter`/`calib_exit`/`get_encoder_raw_deg`/`get_encoder_mech_angle` 是 `static` 函数，无法被 L1/L2 复用。提取为独立模块，L2 的 R/Ld/Lq/flux 未来实现时可直接调用 `calib_hw_apply_voltage()`。

**设计要点：**
- `calib_hw_session_t` 封装一次标定电压会话的上下文（电机指针 + 原始回调 + 强制电角度）。
- 模块内单一 `s_active` 指针指向当前活动会话——系统单电机 + 管理器已保证"同时只有一个标定在跑"，此约束可接受。
- `calib_hw_enter` 替换 `motor->ele_radian_callback` 与 `motor->foc.ele_radian_callback`；`calib_hw_exit` 恢复并撤销 PWM。

- [ ] **Step 1: 创建 calib_hw.h**

```c
#ifndef __CALIB_HW_H__
#define __CALIB_HW_H__

#include <stdbool.h>
#include "calib_types.h" /* 引入 dev_motor 前向声明与 motor_param_t */

/* ===================== 标定硬件访问会话 =====================
 * 封装"标定期间替换电角度回调 + 直接施加 dq 电压"的上下文。
 * 每个 level 模块在 start() 时持有一个本地 session，poll() 中传给 calib_hw_* 函数。
 * 系统单电机 + calib_mgr 保证同时只有一个标定活动，s_active 单例约束可接受。*/
typedef struct
{
	struct dev_motor *motor;          /* 目标电机设备 */
	float (*orig_ele_cb)(void);       /* 保存的原始电角度回调，exit 时恢复 */
	float forced_ele_angle;           /* 标定期间强制使用的电角度(rad) */
} calib_hw_session_t;

/* 进入标定电压会话：保存并替换电角度回调，强制电角度=0 */
void calib_hw_enter(calib_hw_session_t *s, struct dev_motor *m);

/* 退出标定电压会话：撤销 PWM 输出，恢复原始电角度回调 */
void calib_hw_exit(calib_hw_session_t *s);

/* 施加 dq 电压（绕过电流环 PI，直接操作 FOC 链路 + SVPWM + 半桥）
 * @param theta 强制使用的电角度(rad)，用于标定期间固定电角度 */
void calib_hw_apply_voltage(calib_hw_session_t *s, float ud, float uq, float theta);

/* 撤销电压输出（PWM 三相置零）*/
void calib_hw_apply_zero(struct dev_motor *m);

/* 获取 MT6701 原始角度(°) [0,360)，不含 offset/dir 补偿 */
float calib_hw_get_encoder_raw_deg(struct dev_motor *m);

/* 获取编码器当前机械角度(°) [0,360)，含 offset/dir 补偿 */
float calib_hw_get_encoder_mech_angle(struct dev_motor *m);

#endif /* __CALIB_HW_H__ */
```

- [ ] **Step 2: 创建 calib_hw.c**

```c
/**
 * @file calib_hw.c
 * @brief 标定共享硬件访问层：电压会话 + 编码器读取
 * @note 从 calib_level3_encoder.c 提取，供 L1/L2/L3 共享。
 *       单电机系统约束：s_active 同一时刻只指向一个会话。
 */
#include "calib_hw.h"
#include "calib_mgr.h"   /* calib_mgr_get_io() —— abort 时取 motor 用 */
#include "dev_motor.h"
#include "dev_mt6701.h"

/* ===================== 模块私有：当前活动会话（单例）===================== */
static calib_hw_session_t *s_active = NULL;

/* ===================== 标定期间电角度回调（返回强制值）===================== */
static float calib_hw_ele_radian_cb(void)
{
	return s_active ? s_active->forced_ele_angle : 0.0f;
}

/* ===================== 接口实现 ===================== */
void calib_hw_enter(calib_hw_session_t *s, struct dev_motor *m)
{
	s->motor = m;
	s->orig_ele_cb = m->ele_radian_callback;
	s->forced_ele_angle = 0.0f;
	s_active = s;
	m->ele_radian_callback = calib_hw_ele_radian_cb;
	m->foc.ele_radian_callback = calib_hw_ele_radian_cb;
}

void calib_hw_exit(calib_hw_session_t *s)
{
	if (s->motor == NULL)
		return;
	calib_hw_apply_zero(s->motor);
	if (s->orig_ele_cb != NULL)
	{
		s->motor->ele_radian_callback = s->orig_ele_cb;
		s->motor->foc.ele_radian_callback = s->orig_ele_cb;
	}
	s_active = NULL;
}

void calib_hw_apply_voltage(calib_hw_session_t *s, float ud, float uq, float theta)
{
	struct dev_motor *m = s->motor;
	s->forced_ele_angle = theta;
	m->foc.set_udq(&m->foc, ud, uq);
	m->foc.inverse_park(&m->foc);
	m->foc.pfsvpwm(&m->foc);
	m->half_bridge.set_3pwm(&m->half_bridge,
	                        (uint32_t)(PWM_PERIOD * m->foc.svpwm.ta),
	                        (uint32_t)(PWM_PERIOD * m->foc.svpwm.tb),
	                        (uint32_t)(PWM_PERIOD * m->foc.svpwm.tc));
}

void calib_hw_apply_zero(struct dev_motor *m)
{
	m->half_bridge.set_3pwm(&m->half_bridge, 0, 0, 0);
}

float calib_hw_get_encoder_raw_deg(struct dev_motor *m)
{
	dev_mt6701_t *enc = &m->mt6701;
	enc->update(enc);
	return (float)enc->raw / MT6701_ANGLE_RESOLUTION * 360.0F;
}

float calib_hw_get_encoder_mech_angle(struct dev_motor *m)
{
	m->encoder.update(&m->encoder);
	return m->encoder.mechanical_angle;
}
```

- [ ] **Step 3: 验证编译（calib_hw.c 单独编译无 error）**

由于此时还没有 .uvprojx 工程条目（Task 7 才加），本步先确认源码语法正确：检查 `dev_motor.h`、`dev_mt6701.h`、`foc_core.h` 中被引用的成员（`mt6701`、`encoder`、`foc.set_udq`、`foc.inverse_park`、`foc.pfsvpwm`、`foc.svpwm.ta/tb/tc`、`half_bridge.set_3pwm`、`ele_radian_callback`、`foc.ele_radian_callback`、`PWM_PERIOD`、`MT6701_ANGLE_RESOLUTION`）均存在——这些在 `calib_level3_encoder.c` 现有代码中已被使用，确认无误。

- [ ] **Step 4: Commit**

```bash
git add User/MotorCalibration/calib_hw.h User/MotorCalibration/calib_hw.c
git commit -m "feat(calib): extract calib_hw shared hardware access layer from L3"
```

---

### Task 3: 重构 calib_level3_encoder.c 使用 calib_hw + calib_config

**Files:**
- Modify: `User/MotorCalibration/calib_level3_encoder.c`

**目的：** 验证 Task 2 的 `calib_hw` 层在已有可用代码（L3 零位/方向标定）上能正确工作，同时消除 L3 文件内的 5 个散落静态全局变量（`s_orig_ele_cb`、`s_calib_ele_angle`、`s_tick`、`s_sample_cnt`、`s_angle_sum`、`s_dir_start_angle`），合并为单一 `s_l3` 状态结构体。魔法数改用 `calib_config.h`。

**行为保持要求：** 重构后零位标定与方向标定的状态机逻辑、电角度回调替换方式、PWM 施加方式、编码器读取方式必须与原实现完全等价。Diff 审查重点：
1. `calib_enter` → `calib_hw_enter(&s_l3.session, m)`，行为：保存回调 + 替换为 `calib_hw_ele_radian_cb` + 强制电角度=0。✅ 等价
2. `apply_voltage(m, ud, uq, theta)` → `calib_hw_apply_voltage(&s_l3.session, ud, uq, theta)`，行为：写 forced_ele_angle + set_udq + inverse_park + pfsvpwm + set_3pwm。✅ 等价
3. `calib_exit` → `calib_hw_exit(&s_l3.session)`，行为：apply_zero + 恢复回调。✅ 等价
4. `get_encoder_raw_deg` / `get_encoder_mech_angle` → `calib_hw_get_encoder_raw_deg(m)` / `calib_hw_get_encoder_mech_angle(m)`。✅ 等价

- [ ] **Step 1: 用如下完整内容替换 calib_level3_encoder.c**

```c
/**
 * @file calib_level3_encoder.c
 * @brief L3 编码器校准（零位/方向/线性度/正余弦/多圈零点）
 * @note 重构后：硬件访问通过 calib_hw 共享层，参数通过 calib_config.h 集中管理。
 *       零位标定（d轴对齐法）与方向标定（施加uq观测角度变化）为真实实现；
 *       线性度/正余弦/多圈零点保持桩实现。
 *
 * @par 零位标定原理
 *   施加 ud 电压（强制电角度=0），转子磁场与 A 相绕组对齐。
 *   稳定后读取编码器原始角度作为 offset（机械零点对应的编码器读数）。
 *
 * @par 方向标定原理
 *   施加正向 uq 电压（电角度=0 时产生正向力矩），观测编码器角度变化方向。
 *   角度增大→CW（正向），角度减小→CCW（反向）。
 */
#include "calib_types.h"
#include "calib_config.h"
#include "calib_mgr.h"
#include "calib_hw.h"
#include "dev_motor.h"
#include "dev_mt6701.h"
#include "motor_param.h"

/* ===================== 模块私有状态（合并为单一结构体）===================== */
static struct
{
	uint8_t              submode;
	uint8_t              step;          /* 标定步骤状态机 */
	uint32_t             tick;          /* 周期计数器 */
	uint32_t             sample_cnt;    /* 采样计数器 */
	float                angle_sum;     /* 角度采样累加和 */
	float                dir_start_angle; /* 方向测试起始角度 */
	calib_hw_session_t   session;       /* 标定电压会话（替换电角度回调 + 施加电压）*/
} s_l3;

/* ===================== 零位标定状态机 =====================
 * STEP 0: 初始化，施加 ud 电压（电角度强制为0），转子开始对齐
 * STEP 1: 等待转子稳定对齐（CALIB_CFG_L3_ALIGN_TICKS 个周期）
 * STEP 2: 多次采样编码器原始角度取平均
 * STEP 3: 写入 offset 到 encoder_param 和 dev_mt6701，撤销电压，完成
 * ========================================================== */
static calib_state_e poll_zero_offset(void)
{
	const calib_io_t *io = calib_mgr_get_io();
	dev_motor_t *m = io->motor;

	switch (s_l3.step)
	{
		case 0: /* 施加 d 轴对齐电压 */
			calib_hw_enter(&s_l3.session, m);
			calib_hw_apply_voltage(&s_l3.session, CALIB_CFG_L3_ALIGN_VOLTAGE_V, 0.0f, 0.0f);
			s_l3.tick = 0;
			s_l3.step = 1;
			return CALIB_STATE_RUNNING;

		case 1: /* 等待转子稳定对齐 */
			calib_hw_apply_voltage(&s_l3.session, CALIB_CFG_L3_ALIGN_VOLTAGE_V, 0.0f, 0.0f);
			if (++s_l3.tick < CALIB_CFG_L3_ALIGN_TICKS)
				return CALIB_STATE_RUNNING;
			s_l3.tick = 0;
			s_l3.sample_cnt = 0;
			s_l3.angle_sum = 0.0f;
			s_l3.step = 2;
			return CALIB_STATE_RUNNING;

		case 2: /* 多次采样编码器原始角度 */
			calib_hw_apply_voltage(&s_l3.session, CALIB_CFG_L3_ALIGN_VOLTAGE_V, 0.0f, 0.0f);
			s_l3.angle_sum += calib_hw_get_encoder_raw_deg(m);
			if (++s_l3.sample_cnt < CALIB_CFG_L3_SAMPLE_COUNT)
				return CALIB_STATE_RUNNING;
			s_l3.step = 3;
			return CALIB_STATE_RUNNING;

		case 3: /* 写入标定结果，完成 */
		{
			float avg_deg = s_l3.angle_sum / (float)CALIB_CFG_L3_SAMPLE_COUNT;
			/* 写入 dev_mt6701 运行时（offset = 对齐位置的原始角度，使 mech_angle=0）*/
			m->mt6701.offset = avg_deg;
			m->mt6701.dir = MT6701_DIR_CW; /* 零位标定先置 CW，方向由后续方向标定确定 */
			/* 写入 motor_param_t（持久化），enc_offset 用计数值 */
			int32_t raw_counts = (int32_t)(avg_deg / 360.0F * MT6701_ANGLE_RESOLUTION);
			motor_param_set_enc_offset(io->param, raw_counts);
			motor_param_set_enc_direction(io->param, 1); /* 1=CW */
			motor_param_set_elec_angle_bias(io->param, 0.0f);
			calib_hw_exit(&s_l3.session);
			return CALIB_STATE_DONE;
		}

		default:
			calib_hw_exit(&s_l3.session);
			return CALIB_STATE_FAILED;
	}
}

/* ===================== 方向标定状态机 =====================
 * STEP 0: 确保零位已标定，施加正向 uq 电压（电角度=0）
 * STEP 1: 持续施加 uq，等待 CALIB_CFG_L3_DIR_TICKS 个周期让电机转动
 * STEP 2: 采样角度变化方向，判定 CW/CCW
 * STEP 3: 写入 direction，撤销电压，完成
 *
 * @note 方向标定前提：零位已标定（offset 已写入）。
 *       若未标定，先执行零位标定再执行方向标定。
 * ================================================================== */
static calib_state_e poll_direction(void)
{
	const calib_io_t *io = calib_mgr_get_io();
	dev_motor_t *m = io->motor;

	switch (s_l3.step)
	{
		case 0: /* 施加正向 uq 电压，记录起始角度 */
			calib_hw_enter(&s_l3.session, m);
			/* 电角度=0 时 uq>0 产生正向力矩（q轴超前d轴90°，即α轴方向）*/
			s_l3.dir_start_angle = calib_hw_get_encoder_mech_angle(m);
			calib_hw_apply_voltage(&s_l3.session, 0.0f, CALIB_CFG_L3_DIR_VOLTAGE_V, 0.0f);
			s_l3.tick = 0;
			s_l3.step = 1;
			return CALIB_STATE_RUNNING;

		case 1: /* 持续施加 uq，等待电机转动 */
			calib_hw_apply_voltage(&s_l3.session, 0.0f, CALIB_CFG_L3_DIR_VOLTAGE_V, 0.0f);
			if (++s_l3.tick < CALIB_CFG_L3_DIR_TICKS)
				return CALIB_STATE_RUNNING;
			s_l3.step = 2;
			return CALIB_STATE_RUNNING;

		case 2: /* 采样当前角度，判定方向 */
		{
			float end_angle = calib_hw_get_encoder_mech_angle(m);
			float delta = end_angle - s_l3.dir_start_angle;
			/* 处理 0/360 跳变：若 delta 绝对值 >180，说明跨越了 0° 边界 */
			if (delta > 180.0f)
				delta -= 360.0f;
			else if (delta < -180.0f)
				delta += 360.0f;

			mt6701_dir_e dir;
			int8_t enc_dir;
			if (delta > 1.0f) /* 角度增大 → 正向 CW */
			{
				dir = MT6701_DIR_CW;
				enc_dir = 1;
			}
			else if (delta < -1.0f) /* 角度减小 → 反向 CCW */
			{
				dir = MT6701_DIR_CCW;
				enc_dir = -1;
			}
			else /* 角度几乎无变化，可能电机未转动 */
			{
				calib_hw_exit(&s_l3.session);
				return CALIB_STATE_FAILED;
			}

			/* 写入 dev_mt6701 运行时 */
			m->mt6701.dir = dir;
			/* 写入 motor_param_t（持久化） */
			motor_param_set_enc_direction(io->param, enc_dir);
			s_l3.step = 3;
			return CALIB_STATE_RUNNING;
		}

		case 3: /* 撤销电压，完成 */
			calib_hw_exit(&s_l3.session);
			return CALIB_STATE_DONE;

		default:
			calib_hw_exit(&s_l3.session);
			return CALIB_STATE_FAILED;
	}
}

/* ===================== 其余子模式（桩）===================== */
static calib_state_e poll_linearity(void)
{
	return CALIB_STATE_DONE;
}
static calib_state_e poll_sincos(void)
{
	return CALIB_STATE_DONE;
}
static calib_state_e poll_multiturn_zero(void)
{
	return CALIB_STATE_DONE;
}

/* ===================== ops 接口 ===================== */
static bool calib_level3_start(uint8_t submode, motor_param_t *param, float dt)
{
	(void)param;
	(void)dt;

	s_l3.submode = submode;
	s_l3.step = 0;
	s_l3.tick = 0;
	s_l3.sample_cnt = 0;
	s_l3.angle_sum = 0.0f;
	s_l3.session.motor = NULL;
	s_l3.session.orig_ele_cb = NULL;
	s_l3.session.forced_ele_angle = 0.0f;

	switch (submode)
	{
		case CALIB_L3_ZERO_OFFSET:
		case CALIB_L3_DIRECTION:
		case CALIB_L3_LINEARITY:
		case CALIB_L3_SINCOS:
		case CALIB_L3_MULTITURN_ZERO:
			return true;
		default:
			return false;
	}
}

static calib_state_e calib_level3_poll(void)
{
	switch (s_l3.submode)
	{
		case CALIB_L3_ZERO_OFFSET: return poll_zero_offset();
		case CALIB_L3_DIRECTION: return poll_direction();
		case CALIB_L3_LINEARITY: return poll_linearity();
		case CALIB_L3_SINCOS: return poll_sincos();
		case CALIB_L3_MULTITURN_ZERO: return poll_multiturn_zero();
		default: return CALIB_STATE_FAILED;
	}
}

static void calib_level3_abort(void)
{
	const calib_io_t *io = calib_mgr_get_io();
	if (io != NULL && io->motor != NULL)
		calib_hw_exit(&s_l3.session);
	s_l3.step = 0;
}

const calib_level_ops_t calib_level3_ops = {
	.start = calib_level3_start,
	.poll = calib_level3_poll,
	.abort = calib_level3_abort,
};
```

- [ ] **Step 2: Diff 审查——确认行为等价**

逐函数对比原 `calib_level3_encoder.c` 与新版本：
- `poll_zero_offset`: 4 个 step 的状态转移、tick 计数、采样累加、offset 写入完全一致；仅将 `apply_voltage` → `calib_hw_apply_voltage`、`get_encoder_raw_deg` → `calib_hw_get_encoder_raw_deg`、`calib_enter` → `calib_hw_enter`、`calib_exit` → `calib_hw_exit`。
- `poll_direction`: 4 个 step 完全一致；方向判定阈值（±1.0°）、跳变处理（±180°）、CW/CCW 写入一致。
- `calib_level3_start`/`poll`/`abort`: switch 分支一致；abort 行为一致（调 exit + 清 step）。

- [ ] **Step 3: Commit**

```bash
git add User/MotorCalibration/calib_level3_encoder.c
git commit -m "refactor(calib): L3 use calib_hw + calib_config, consolidate file-statics into s_l3"
```

---

### Task 4: 拆分 L2 子模式常量（calib_types.h）

**Files:**
- Modify: `User/MotorCalibration/calib_types.h:38-42`

**目的：** 响应需求第 1 条。`CALIB_L2_RL_FLUX=3` 合并了 4 个物理上独立的辨识（R 用 DC 法、Ld/Lq 用阶跃响应法、flux 用反电势法），拆为 4 个独立子模式。submode 编号 3→6 连续，不影响协议层（payload[0] 透传）。

- [ ] **Step 1: 替换 L2 子模式段**

定位 `calib_types.h` 第 38-42 行：
```c
/* ===================== L2 子模式: 电机电气身份 ===================== */
#define CALIB_L2_PHASE_SEQ		1	/* 相序识别 */
#define CALIB_L2_POLE_PAIRS	2	/* 极对数 */
#define CALIB_L2_RL_FLUX		3	/* R/Ld/Lq/flux辨识 */
```

替换为：
```c
/* ===================== L2 子模式: 电机电气身份 =====================
 * 原 CALIB_L2_RL_FLUX(3) 拆分为 4 个独立子模式：
 * R/Ld/Lq/flux 物理上需不同测试方法（DC法/阶跃响应/反电势法），
 * 拆分后可独立触发与重试。submode 编号 3-6 连续，协议层 payload[0] 透传无需改动。*/
#define CALIB_L2_PHASE_SEQ			1	/* 相序识别 */
#define CALIB_L2_POLE_PAIRS			2	/* 极对数 */
#define CALIB_L2_RESISTANCE			3	/* R 相电阻辨识（DC法）*/
#define CALIB_L2_INDUCTANCE_D		4	/* Ld d轴电感辨识（阶跃响应）*/
#define CALIB_L2_INDUCTANCE_Q		5	/* Lq q轴电感辨识（阶跃响应）*/
#define CALIB_L2_FLUX_LINKAGE		6	/* flux 磁链辨识（反电势法）*/
```

- [ ] **Step 2: 全仓搜索 CALIB_L2_RL_FLUX 引用，确认无遗漏**

Run: 在 `d:\AAWorkSpace\001_JointMotor\SW\JointMotor` 下用 Grep 工具搜索 `CALIB_L2_RL_FLUX`，应只剩 `calib_level2_motor.c` 与 `calib_level7_auto.c` 两处引用（Task 5、Task 6 处理）。若发现其他引用，一并更新。

Expected: 命中 `calib_level2_motor.c`（start/poll switch）+ `calib_level7_auto.c`（s_sequence 表）。

- [ ] **Step 3: Commit**

```bash
git add User/MotorCalibration/calib_types.h
git commit -m "refactor(calib): split CALIB_L2_RL_FLUX into R/Ld/Lq/flux submodes (3-6)"
```

---

### Task 5: 更新 calib_level2_motor.c（新子模式 + calib_hw 备用）

**Files:**
- Modify: `User/MotorCalibration/calib_level2_motor.c`

**目的：** 扩展 L2 的 start/poll switch 到 6 个 case，新增 4 个桩 `poll_*()` 函数（R/Ld/Lq/flux），引入 `calib_hw_session_t` 字段为未来真实算法准备。响应需求第 1 条（拆分）+ 第 3 条（模块化，复用 calib_hw）。

- [ ] **Step 1: 用如下完整内容替换 calib_level2_motor.c**

```c
/**
 * @file calib_level2_motor.c
 * @brief L2 电机电气身份辨识（相序/极对数/R/Ld/Lq/flux）
 * @note L2 子模式 1-2（相序/极对数）与 3-6（R/Ld/Lq/flux）当前为桩实现。
 *       真实算法填充 poll_*() 函数即可，硬件访问通过 calib_hw 共享层，
 *       参数通过 calib_config.h 集中管理，结果写入 motor_param_set_r/ld/lq/flux/pole_pairs。
 *
 * @par 各子模式算法（未来实现参考）
 *   - 相序识别：施加 ud，观测三相电流相序方向
 *   - 极对数：施加 ud 旋转一周，数电周期数 = 极对数
 *   - R：施加 DC 电压 ud，稳态后 R = ud / id
 *   - Ld：施加 ud 阶跃，观测 di/dt，Ld = (ud - R*id) / (did/dt)
 *   - Lq：施加 uq 阶跃，观测 diq/dt，Lq = uq / (diq/dt)（近似）
 *   - flux：开环驱动电机稳速转动，flux = (uq - R*iq) / ω
 */
#include "calib_types.h"
#include "calib_config.h"
#include "calib_mgr.h"
#include "calib_hw.h"
#include "dev_motor.h"
#include "motor_param.h"

/* ===================== 模块私有状态（合并为单一结构体）===================== */
static struct
{
	uint8_t             submode;
	uint8_t             step;          /* 标定步骤状态机（未来真实算法用）*/
	calib_hw_session_t  session;       /* 标定电压会话（未来施加测试电压用）*/
} s_l2;

/* ===================== 各子模式桩实现 =====================
 * 当前直接返回 DONE（与 L1 桩约定一致，见标定状态机说明.md §8.3）。
 * 真实算法填充时替换函数体，状态机用 s_l2.step 推进。*/
static calib_state_e poll_phase_seq(void)
{
	return CALIB_STATE_DONE;
}
static calib_state_e poll_pole_pairs(void)
{
	return CALIB_STATE_DONE;
}
static calib_state_e poll_resistance(void)
{
	/* TODO: 真实实现
	 * STEP 0: calib_hw_enter + calib_hw_apply_voltage(ud=CFG_L2_R_TEST_VOLTAGE, 0, 0)
	 * STEP 1: 等待 CALIB_CFG_L2_R_TEST_TIME_S 稳态
	 * STEP 2: 采样 CALIB_CFG_L2_R_SAMPLE_COUNT 次 id 取平均
	 * STEP 3: R = ud / id_avg; motor_param_set_r(param, R); calib_hw_exit
	 */
	return CALIB_STATE_DONE;
}
static calib_state_e poll_inductance_d(void)
{
	/* TODO: 真实实现——ud 阶跃响应，Ld = (ud - R*id) / (did/dt) */
	return CALIB_STATE_DONE;
}
static calib_state_e poll_inductance_q(void)
{
	/* TODO: 真实实现——uq 阶跃响应，Lq = uq / (diq/dt) */
	return CALIB_STATE_DONE;
}
static calib_state_e poll_flux_linkage(void)
{
	/* TODO: 真实实现——开环稳速转动，flux = (uq - R*iq) / ω */
	return CALIB_STATE_DONE;
}

/* ===================== ops 接口 ===================== */
static bool calib_level2_start(uint8_t submode, motor_param_t *param, float dt)
{
	(void)param;
	(void)dt;

	s_l2.submode = submode;
	s_l2.step = 0;
	s_l2.session.motor = NULL;
	s_l2.session.orig_ele_cb = NULL;
	s_l2.session.forced_ele_angle = 0.0f;

	switch (submode)
	{
		case CALIB_L2_PHASE_SEQ:
		case CALIB_L2_POLE_PAIRS:
		case CALIB_L2_RESISTANCE:
		case CALIB_L2_INDUCTANCE_D:
		case CALIB_L2_INDUCTANCE_Q:
		case CALIB_L2_FLUX_LINKAGE:
			return true;
		default:
			return false;
	}
}

static calib_state_e calib_level2_poll(void)
{
	switch (s_l2.submode)
	{
		case CALIB_L2_PHASE_SEQ:     return poll_phase_seq();
		case CALIB_L2_POLE_PAIRS:    return poll_pole_pairs();
		case CALIB_L2_RESISTANCE:    return poll_resistance();
		case CALIB_L2_INDUCTANCE_D:  return poll_inductance_d();
		case CALIB_L2_INDUCTANCE_Q:  return poll_inductance_q();
		case CALIB_L2_FLUX_LINKAGE:  return poll_flux_linkage();
		default: return CALIB_STATE_FAILED;
	}
}

static void calib_level2_abort(void)
{
	const calib_io_t *io = calib_mgr_get_io();
	if (io != NULL && io->motor != NULL)
		calib_hw_exit(&s_l2.session);
	s_l2.step = 0;
}

const calib_level_ops_t calib_level2_ops = {
	.start = calib_level2_start,
	.poll = calib_level2_poll,
	.abort = calib_level2_abort,
};
```

- [ ] **Step 2: 确认无 CALIB_L2_RL_FLUX 残留**

Run: Grep 搜索 `CALIB_L2_RL_FLUX` in `User/MotorCalibration/calib_level2_motor.c`，应 0 命中。

- [ ] **Step 3: Commit**

```bash
git add User/MotorCalibration/calib_level2_motor.c
git commit -m "feat(calib): L2 expand to 6 submodes with calib_hw session, R/Ld/Lq/flux stubs"
```

---

### Task 6: 更新 calib_level7_auto.c 序列表

**Files:**
- Modify: `User/MotorCalibration/calib_level7_auto.c:16-34`

**目的：** L7 全自动序列中原 `{CALIB_LEVEL2_MOTOR, CALIB_L2_RL_FLUX}` 一行已失效（常量被删除）。拆为 4 行，对应 R→Ld→Lq→flux 的物理执行顺序（R 是 Ld/Lq 计算的输入，flux 需要电机转动放最后）。

- [ ] **Step 1: 替换 s_sequence[] 数组定义**

定位 `calib_level7_auto.c` 第 16-34 行的 `s_sequence[]` 数组：
```c
static const struct
{
	uint8_t level;
	uint8_t submode;
} s_sequence[] = {
	{CALIB_LEVEL1_DRIVER,    CALIB_L1_ADC_OFFSET    },
	{CALIB_LEVEL1_DRIVER,    CALIB_L1_ADC_GAIN      },
	{CALIB_LEVEL1_DRIVER,    CALIB_L1_CURRENT_SENSOR},
	{CALIB_LEVEL2_MOTOR,     CALIB_L2_PHASE_SEQ     },
	{CALIB_LEVEL2_MOTOR,     CALIB_L2_POLE_PAIRS    },
	{CALIB_LEVEL2_MOTOR,     CALIB_L2_RL_FLUX       },
	{CALIB_LEVEL3_ENCODER,   CALIB_L3_ZERO_OFFSET   },
	{CALIB_LEVEL3_ENCODER,   CALIB_L3_DIRECTION     },
	{CALIB_LEVEL4_TORQUE,    CALIB_L4_KT            },
	{CALIB_LEVEL5_NONLINEAR, CALIB_L5_COGGING       },
	{CALIB_LEVEL5_NONLINEAR, CALIB_L5_FRICTION      },
	{CALIB_LEVEL6_SYSTEM,    CALIB_L6_INERTIA       },
	{CALIB_LEVEL6_SYSTEM,    CALIB_L6_PID_AUTOTUNE  },
};
```

替换为：
```c
static const struct
{
	uint8_t level;
	uint8_t submode;
} s_sequence[] = {
	{CALIB_LEVEL1_DRIVER,    CALIB_L1_ADC_OFFSET    },
	{CALIB_LEVEL1_DRIVER,    CALIB_L1_ADC_GAIN      },
	{CALIB_LEVEL1_DRIVER,    CALIB_L1_CURRENT_SENSOR},
	{CALIB_LEVEL2_MOTOR,     CALIB_L2_PHASE_SEQ     },
	{CALIB_LEVEL2_MOTOR,     CALIB_L2_POLE_PAIRS    },
	{CALIB_LEVEL2_MOTOR,     CALIB_L2_RESISTANCE    },  /* R 先做，Ld/Lq 计算需用 R */
	{CALIB_LEVEL2_MOTOR,     CALIB_L2_INDUCTANCE_D  },
	{CALIB_LEVEL2_MOTOR,     CALIB_L2_INDUCTANCE_Q  },
	{CALIB_LEVEL2_MOTOR,     CALIB_L2_FLUX_LINKAGE  },  /* flux 需电机转动，放最后 */
	{CALIB_LEVEL3_ENCODER,   CALIB_L3_ZERO_OFFSET   },
	{CALIB_LEVEL3_ENCODER,   CALIB_L3_DIRECTION     },
	{CALIB_LEVEL4_TORQUE,    CALIB_L4_KT            },
	{CALIB_LEVEL5_NONLINEAR, CALIB_L5_COGGING       },
	{CALIB_LEVEL5_NONLINEAR, CALIB_L5_FRICTION      },
	{CALIB_LEVEL6_SYSTEM,    CALIB_L6_INERTIA       },
	{CALIB_LEVEL6_SYSTEM,    CALIB_L6_PID_AUTOTUNE  },
};
```

序列长度从 13 步变为 16 步（L2 段 3→6）。`L7_SEQ_LEN` 由 `sizeof(s_sequence)/sizeof(s_sequence[0])` 自动计算，无需手改。

- [ ] **Step 2: 确认无 CALIB_L2_RL_FLUX 残留**

Run: Grep 搜索 `CALIB_L2_RL_FLUX` 全仓，应 0 命中（所有引用已在 Task 4/5/6 清除）。

Expected: 0 命中。

- [ ] **Step 3: Commit**

```bash
git add User/MotorCalibration/calib_level7_auto.c
git commit -m "refactor(calib): L7 sequence expand L2 from 3 to 6 steps (R/Ld/Lq/flux ordered)"
```

---

### Task 7: 更新 Keil 工程文件（添加 calib_hw.c）

**Files:**
- Modify: `Board/V1/MDK-ARM/JointMotorApp.uvprojx`
- Modify: `Board/SFOC/MDK-ARM/sfoc.uvprojx`

**目的：** 将 `calib_hw.c` 加入两个 Keil 工程的 MotorCalibration 组。`calib_config.h` 与 `calib_hw.h` 是头文件，无需加入工程（通过 include path 已可见）。

- [ ] **Step 1: 在 JointMotorApp.uvprojx 的 MotorCalibration 组添加 calib_hw.c**

定位 `JointMotorApp.uvprojx` 第 626-630 行（`calib_level7_auto.c` 条目之后、`</Files>` 之前）：
```xml
            <File>
              <FileName>calib_level7_auto.c</FileName>
              <FileType>1</FileType>
              <FilePath>..\..\..\User\MotorCalibration\calib_level7_auto.c</FilePath>
            </File>
          </Files>
```

替换为：
```xml
            <File>
              <FileName>calib_level7_auto.c</FileName>
              <FileType>1</FileType>
              <FilePath>..\..\..\User\MotorCalibration\calib_level7_auto.c</FilePath>
            </File>
            <File>
              <FileName>calib_hw.c</FileName>
              <FileType>1</FileType>
              <FilePath>..\..\..\User\MotorCalibration\calib_hw.c</FilePath>
            </File>
          </Files>
```

- [ ] **Step 2: 在 sfoc.uvprojx 的 MotorCalibration 组添加 calib_hw.c**

在 `Board/SFOC/MDK-ARM/sfoc.uvprojx` 中执行相同操作：找到 `calib_level7_auto.c` 的 `<File>` 条目，在其后、`</Files>` 之前插入相同的 `calib_hw.c` `<File>` 块。

- [ ] **Step 3: Commit**

```bash
git add Board/V1/MDK-ARM/JointMotorApp.uvprojx Board/SFOC/MDK-ARM/sfoc.uvprojx
git commit -m "build(calib): add calib_hw.c to both Keil projects"
```

---

### Task 8: 更新标定状态机说明文档

**Files:**
- Modify: `User/MotorCalibration/docs/标定状态机说明.md`

**目的：** 文档同步实际代码——L2 子模式数 3→6，新增 calib_hw/calib_config 文件说明，L7 序列步数 13→16。

- [ ] **Step 1: 更新文件结构小节（§2）**

定位 `标定状态机说明.md` 第 11-23 行的文件结构代码块，在 `calib_level7_auto.c` 行后、闭合 ``` 前插入两行：

原：
```
├── calib_level7_auto.c     # L7 自动化集成（1 子模式，序列编排）
```
改为：
```
├── calib_level7_auto.c     # L7 自动化集成（1 子模式，序列编排）
├── calib_hw.h              # 标定共享硬件访问层接口（电压会话 + 编码器读取）
├── calib_hw.c              # 标定共享硬件访问层实现（L1/L2/L3 复用）
├── calib_config.h          # 标定集中配置（电压/时间/采样数，避免魔法数散落）
```

- [ ] **Step 2: 更新 7 级 24 子模式定义表（§4）**

定位 `标定状态机说明.md` 第 77 行：
```
| L2 | 0x91 | 电机电气身份 | 1=相序识别 / 2=极对数 / 3=R/Ld/Lq/flux辨识 |
```
改为：
```
| L2 | 0x91 | 电机电气身份 | 1=相序识别 / 2=极对数 / 3=R / 4=Ld / 5=Lq / 6=flux |
```

并将表标题"7 级 24 子模式"改为"7 级 27 子模式"（L2 由 3 增至 6，总数 24+3=27）。同步更新 §1 概述中"7 级 24 子模式"为"7 级 27 子模式"。

- [ ] **Step 3: 更新 L7 全自动序列（§7）**

定位 `标定状态机说明.md` 第 152-163 行的 L7 序列代码块：
```
L1.ADC_OFFSET → L1.ADC_GAIN → L1.CURRENT_SENSOR
→ L2.PHASE_SEQ → L2.POLE_PAIRS → L2.RL_FLUX
→ L3.ZERO_OFFSET → L3.DIRECTION
→ L4.KT
→ L5.COGGING → L5.FRICTION
→ L6.INERTIA → L6.PID_AUTOTUNE
```
改为：
```
L1.ADC_OFFSET → L1.ADC_GAIN → L1.CURRENT_SENSOR
→ L2.PHASE_SEQ → L2.POLE_PAIRS → L2.RESISTANCE → L2.INDUCTANCE_D → L2.INDUCTANCE_Q → L2.FLUX_LINKAGE
→ L3.ZERO_OFFSET → L3.DIRECTION
→ L4.KT
→ L5.COGGING → L5.FRICTION
→ L6.INERTIA → L6.PID_AUTOTUNE
```

并将"共 13 步"改为"共 16 步"。

- [ ] **Step 4: 更新扩展指南（§8.1）——补充新子模式示例**

定位 `标定状态机说明.md` §8.1 末尾，在"无需修改其他文件"段后追加一段：

```markdown
**新增 L2 子模式拆分后的真实算法填充流程**（以 R 辨识为例）：

1. `calib_config.h`：确认/调整 `CALIB_CFG_L2_R_TEST_VOLTAGE_V` / `_R_TEST_TIME_S` / `_R_SAMPLE_COUNT`
2. `calib_level2_motor.c`：在 `poll_resistance()` 函数体内替换桩，用 `calib_hw_enter` / `calib_hw_apply_voltage` / `calib_hw_get_*` / `calib_hw_exit` 实现状态机，结果通过 `motor_param_set_r(io->param, R)` 写入
3. 无需修改协议层、管理器、其他 level 文件
```

- [ ] **Step 5: Commit**

```bash
git add User/MotorCalibration/docs/标定状态机说明.md
git commit -m "docs(calib): sync L2 split (3→6 submodes), calib_hw/calib_config, L7 16 steps"
```

---

### Task 9: 双工程全量编译验证

**Files:** 无修改，仅验证

**目的：** 确认所有改动在两个 Keil 工程中均能编译通过，0 error，且无新增 warning（与 `Board/V1/MDK-ARM/build_calib.log` 基线对比）。

- [ ] **Step 1: 编译 JointMotorApp 工程（V1 板）**

在 Keil MDK 中打开 `Board/V1/MDK-ARM/JointMotorApp.uvprojx`，执行 Rebuild。

或者命令行（若配置了 fromelf/UV4）：
```
UV4 -b Board\V1\MDK-ARM\JointMotorApp.uvprojx -o Board\V1\MDK-ARM\build_calib_v2.log -j0
```

Expected:
- `0 Error(s)` 
- Warning 数 ≤ 基线 `build_calib.log` 中的 warning 数（不应有新增 warning）

若出现 `CALIB_L2_RL_FLUX undeclared`：说明有遗漏引用，回到 Task 4/5/6 排查。
若出现 `calib_hw.c: cannot open source input file`：说明 Task 7 的 .uvprojx 编辑有误。
若出现 `s_active multiply defined`：说明 calib_hw.c 被重复加入工程。

- [ ] **Step 2: 编译 sfoc 工程（SFOC 板）**

在 Keil MDK 中打开 `Board/SFOC/MDK-ARM/sfoc.uvprojx`，执行 Rebuild。

Expected: 同 Step 1。

- [ ] **Step 3: 记录编译结果**

将两个工程的编译输出末尾（`Program Size: Code=... RO=... RW=... ZI=...` 与 `0 Error(s), N Warning(s)` 行）追加到 `Board/V1/MDK-ARM/build_calib_v2.log`，作为本次重构的基线。

- [ ] **Step 4: Commit 编译日志**

```bash
git add Board/V1/MDK-ARM/build_calib_v2.log
git commit -m "build(calib): record post-refactor compile baseline (0 errors, both projects)"
```

---

## 自检清单

### 1. 需求覆盖

| 用户需求 | 覆盖任务 | 状态 |
|---|---|---|
| 1. R/Ld/Lq/flux 拆为 0x91 下多个子模式 | Task 4（拆常量）+ Task 5（拆 dispatch）+ Task 6（L7 序列） | ✅ |
| 2. 其他标定模式不实现，但考虑结构影响 | 顶部"结构影响分析"表 + Task 2（calib_hw 提取，为 L1/L2 未来算法铺路） | ✅ |
| 3. 模块化、降耦合、减全局变量、加配置文件 | Task 1（calib_config.h）+ Task 2（calib_hw 共享层）+ Task 3（L3 重构消除 8 个散落 static） | ✅ |
| L1 子模式保持桩（不实现） | 不修改 calib_level1_driver.c | ✅ |
| L3 零位/方向已有实现保持可用 | Task 3 重构后行为等价（diff 审查步骤） | ✅ |
| 协议层不动 | 不修改 jm_proto_ops.c / jm_cmd_def.h | ✅ |
| 管理器不动 | 不修改 calib_mgr.c / calib_mgr.h | ✅ |
| 双工程编译通过 | Task 9 | ✅ |
| 文档同步 | Task 8 | ✅ |

### 2. 占位符扫描

- ✅ 无 "TBD" / "implement later" / "fill in details"
- ✅ 代码块内 `/* TODO: 真实实现 */` 注释——这是代码内对桩的标注（与现有 `calib_level1_driver.c:53`、`calib_level7_auto.c:46` 一致），非计划占位符
- ✅ 所有步骤含完整代码或确切命令
- ✅ Task 9 的 expected 输出含具体错误诊断分支

### 3. 类型一致性

- `calib_hw_session_t` 在 Task 2 (calib_hw.h) 定义，Task 3 (L3 `s_l3.session`) 与 Task 5 (L2 `s_l2.session`) 引用——字段名 `motor` / `orig_ele_cb` / `forced_ele_angle` 一致 ✅
- `calib_hw_enter` / `calib_hw_exit` / `calib_hw_apply_voltage` / `calib_hw_apply_zero` / `calib_hw_get_encoder_raw_deg` / `calib_hw_get_encoder_mech_angle` 在 Task 2 声明，Task 3/5 调用——签名一致 ✅
- `CALIB_L2_RESISTANCE` / `CALIB_L2_INDUCTANCE_D` / `CALIB_L2_INDUCTANCE_Q` / `CALIB_L2_FLUX_LINKAGE` 在 Task 4 (calib_types.h) 定义，Task 5 (L2 dispatch) 与 Task 6 (L7 序列) 引用——命名一致 ✅
- `CALIB_CFG_L3_*` 宏在 Task 1 (calib_config.h) 定义，Task 3 (L3) 引用——命名一致 ✅
- `s_l3` / `s_l2` 结构体字段（`submode` / `step` / `tick` / `sample_cnt` / `angle_sum` / `dir_start_angle`）在 Task 3 内部使用，无跨文件引用 ✅
- `motor_param_set_r` / `set_ld` / `set_lq` / `set_flux` / `set_pole_pairs` / `set_enc_offset` / `set_enc_direction` / `set_elec_angle_bias` 均在 `motor_param.h` 中存在（研究确认）✅

---

## 执行交付

计划已保存至 `User/Data/plans/2026-07-01-calibration-l2-split-and-modularization.md`。两种执行方式：

**1. Subagent-Driven（推荐）** — 每个 Task 派发独立 subagent，任务间两阶段 review，迭代快

**2. Inline Execution** — 在当前会话内按 executing-plans 批量执行，带检查点

**选哪种？**
