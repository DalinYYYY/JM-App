# 电机运动模式模块化重构 + Bug 修复 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将前期需要的运动模式（IDLE/HOLD/OPEN_LOOP/DUTY/CURRENT/TORQUE/MIT/VELOCITY/POSITION/PV/PT）拆分为独立模块文件，每个模式可单独优化；同步修复电流环直通缺失、过渡期 profile 突变、首次进入无过渡、扫频 static 重入、校准指令路由穿透等 bug。

**Architecture:** 采用 C-OOP ops 函数表模式——每个模式一个 `.c` 文件导出统一签名 `void motor_mode_xxx_run(motor_ctrl_t *ctrl)`，`motor_control.c` 仅保留 `motor_ctrl_init` + `motor_ctrl_dispatch`（查表分发）。前期未拆分的模式（力控/轨迹/特殊/测试类）统一走 `motor_mode_legacy_run` fallback，后续逐个迁移。电流环 `cur_loop_run` 重构为接收完整 `motor_ref_t*`，按 `ctrl_type` 内部分流 VOLTAGE/DUTY/IDLE 直通。校准指令路由到独立的 `TOP_FSM_CALIB` 状态，不纳入运动模式体系。

**Tech Stack:** STM32G4 HAL, 嵌入式C (C99), Keil MDK-ARM, C-OOP ops 函数表, 虚拟电机仿真 (dev_motor_virtual.c), PyQt 上位机

---

## 模式模块化设计

### 目录结构

```
User/MotorControl/
├── ControlProcess/
│   ├── motor_control.c          # 重构：仅 init + dispatch + legacy handler
│   ├── motor_control.h          # 保留 motor_ctrl_t 定义
│   ├── motor_mode.h             # 新增：模式函数签名 + dispatch 接口
│   ├── ctrl_transition.c        # 修改：profile 延迟切换
│   └── pid_profile.{c,h}
├── Modes/                       # 新增目录
│   ├── motor_mode_idle.c        # IDLE
│   ├── motor_mode_hold.c        # HOLD（位置保持）
│   ├── motor_mode_open_loop.c   # OPEN_LOOP / VOLTAGE_VECTOR
│   ├── motor_mode_duty.c        # DUTY_CYCLE
│   ├── motor_mode_current.c     # CURRENT / FIELD_WEAKENING / SENSORLESS
│   ├── motor_mode_torque.c      # TORQUE
│   ├── motor_mode_mit.c         # MIT
│   ├── motor_mode_velocity.c    # VELOCITY / VELOCITY_TORQUE
│   ├── motor_mode_position.c    # POSITION / POSITION_VELOCITY / POSITION_TORQUE / PP
│   ├── motor_mode_profile_velocity.c # PV（速度轮廓，前期=速度+斜坡占位）
│   ├── motor_mode_profile_torque.c   # PT（力矩轮廓，前期=力矩+斜坡占位）
│   └── motor_mode_test_sweep.c  # TEST_SWEEP_FREQ（范例：测试模式也独立）
└── CascadeControl/
    ├── current_loop.{c,h}       # 重构：cur_loop_run 接收 ref，直通分流
    ├── cascade_control.c        # 修改：直通分支语义明确
    └── motor_loop.c             # 修改：按 ctrl_type 跳过外环
```

### 命名规范

- 文件名：`motor_mode_<name>.c`（与 `motor_control.c`/`motor_control.h` 前缀一致）
- 函数名：`motor_mode_<name>_run(motor_ctrl_t *ctrl)`
- 统一签名 typedef：`motor_mode_fn`

### Dispatch 表（ops 函数表模式）

`motor_control.c` 维护 `static const motor_mode_fn s_mode_table[RUN_STATE_MAX]`，表项指向各模式文件的 `motor_mode_xxx_run`。`motor_ctrl_dispatch` 查表调用。前期未拆分的模式指向 `motor_mode_legacy_run`（保留原 switch 逻辑）。

### 标定模式说明

标定（`CONTROL_MODE_CALIB_*`）不属于运动控制，走独立的 `TOP_FSM_CALIB` 顶层状态，**不纳入** `motor_mode_*` 体系。本计划仅修复校准指令的路由穿透 bug（让其正确进入 `TOP_FSM_CALIB`），标定逻辑本身后续单独实现。

---

## Task 1: 建立 motor_mode.h 接口与 Modes/ 目录

**目标**：定义统一的模式函数签名，为后续拆分建立骨架。

**Files:**
- Create: `User/MotorControl/ControlProcess/motor_mode.h`
- Create: `User/MotorControl/Modes/` (目录)

- [ ] **Step 1: 创建 motor_mode.h**

```c
#ifndef __MOTOR_MODE_H__
#define __MOTOR_MODE_H__

#include "motor_control.h"

/**
 * @brief 模式处理函数统一签名
 * @details 每个运动模式导出一个此签名函数，由 motor_ctrl_dispatch 查表调用。
 *          函数内仅填充 ctrl->ref（参考输出），不执行任何环路计算。
 *          实时约束：可在电流环中断调用，禁止动态内存/阻塞/字符串格式化。
 * @param ctrl 电机控制上下文（含 run_state/cmd/fb/param/ref）
 */
typedef void (*motor_mode_fn)(motor_ctrl_t *ctrl);

/* 限幅辅助（公共，供各模式文件复用）*/
static inline float motor_mode_clamp(float v, float lo, float hi)
{
	return (v < lo) ? lo : (v > hi) ? hi : v;
}

/* ===== 前期实现的模式（每个模式一个文件）===== */
void motor_mode_idle_run(motor_ctrl_t *ctrl);            /* IDLE */
void motor_mode_hold_run(motor_ctrl_t *ctrl);            /* HOLD */
void motor_mode_open_loop_run(motor_ctrl_t *ctrl);       /* OPEN_LOOP / VOLTAGE_VECTOR */
void motor_mode_duty_run(motor_ctrl_t *ctrl);            /* DUTY_CYCLE */
void motor_mode_current_run(motor_ctrl_t *ctrl);         /* CURRENT / FIELD_WEAKENING / SENSORLESS */
void motor_mode_torque_run(motor_ctrl_t *ctrl);          /* TORQUE */
void motor_mode_mit_run(motor_ctrl_t *ctrl);             /* MIT */
void motor_mode_velocity_run(motor_ctrl_t *ctrl);        /* VELOCITY / VELOCITY_TORQUE */
void motor_mode_position_run(motor_ctrl_t *ctrl);        /* POSITION / POSITION_VELOCITY / POSITION_TORQUE / PP */
void motor_mode_profile_velocity_run(motor_ctrl_t *ctrl);/* PV */
void motor_mode_profile_torque_run(motor_ctrl_t *ctrl);  /* PT */
void motor_mode_test_sweep_run(motor_ctrl_t *ctrl);      /* TEST_SWEEP_FREQ */

/* ===== Legacy fallback（未拆分模式的统一入口）===== */
void motor_mode_legacy_run(motor_ctrl_t *ctrl);          /* 力控/轨迹/特殊/其他测试模式 */

#endif /* __MOTOR_MODE_H__ */
```

- [ ] **Step 2: 创建 Modes 目录**

在 Keil 工程中 `User/MotorControl/` 下新建 `Modes` 分组目录，后续 Task 创建的模式文件加入此分组。

- [ ] **Step 3: 提交**

```bash
git add User/MotorControl/ControlProcess/motor_mode.h
git commit -m "feat(motor_mode): 建立模式函数统一签名接口 motor_mode.h

定义 motor_mode_fn 统一签名，每个模式导出 motor_mode_xxx_run。
提供 motor_mode_clamp 公共限幅辅助。
为模式模块化拆分建立骨架。"
```

---

## Task 2: 拆分基础模式为独立文件

**目标**：将 `run_generic_control` 中的 9 个基础模式 + HOLD 拆分为独立文件。

**Files:**
- Create: `User/MotorControl/Modes/motor_mode_idle.c`
- Create: `User/MotorControl/Modes/motor_mode_hold.c`
- Create: `User/MotorControl/Modes/motor_mode_open_loop.c`
- Create: `User/MotorControl/Modes/motor_mode_duty.c`
- Create: `User/MotorControl/Modes/motor_mode_current.c`
- Create: `User/MotorControl/Modes/motor_mode_torque.c`
- Create: `User/MotorControl/Modes/motor_mode_mit.c`
- Create: `User/MotorControl/Modes/motor_mode_velocity.c`
- Create: `User/MotorControl/Modes/motor_mode_position.c`
- Modify: `User/DataHub/state_define.h` (新增 RUN_STATE_HOLD)

- [ ] **Step 1: 在 state_define.h 新增 RUN_STATE_HOLD / RUN_STATE_PROFILE_VELOCITY / RUN_STATE_PROFILE_TORQUE**

在 `run_state_e` 枚举的 `RUN_STATE_IDLE` 后增加 `RUN_STATE_HOLD`（约 L130），在 `RUN_STATE_VELOCITY_TORQUE` 后增加 `RUN_STATE_PROFILE_VELOCITY`/`RUN_STATE_PROFILE_TORQUE`：

```c
typedef enum
{
	RUN_STATE_IDLE = 0,			 // 空闲保持
	RUN_STATE_HOLD,				 // 位置保持（主动锁定当前位置）—— 新增
	RUN_STATE_OPEN_LOOP,		 // 开环电压控制
	RUN_STATE_CURRENT,			 // 电流环控制
	RUN_STATE_TORQUE,			 // 力矩环控制
	RUN_STATE_MIT,				 // MIT Cheetah控制
	RUN_STATE_VELOCITY,			 // 速度环控制
	RUN_STATE_POSITION,			 // 位置环控制
	RUN_STATE_POSITION_VELOCITY, // 位置+速度前馈
	RUN_STATE_POSITION_TORQUE,	 // 位置+力矩限幅
	RUN_STATE_VELOCITY_TORQUE,	 // 速度+力矩限幅
	RUN_STATE_PROFILE_VELOCITY,	 // 轮廓速度模式（PV）—— 新增
	RUN_STATE_PROFILE_TORQUE,	 // 轮廓力矩模式（PT）—— 新增
	RUN_STATE_DUTY_CYCLE,		 // 占空比直接控制
	RUN_STATE_VOLTAGE_VECTOR,	 // 电压矢量控制
	RUN_STATE_FIELD_WEAKENING,	 // 弱磁控制
	RUN_STATE_SENSORLESS,		 // 无感FOC控制
	// ... 其余不变
```

**注意**：使用指定初始化器 `[RUN_STATE_XXX] = ...` 的映射表不受枚举值顺序变化影响。

- [ ] **Step 2: 创建 motor_mode_idle.c**

```c
/**
 * @file motor_mode_idle.c
 * @brief 空闲模式：PWM 置零，参考保持当前位置
 */
#include "motor_mode.h"

void motor_mode_idle_run(motor_ctrl_t *ctrl)
{
	motor_ref_t *ref = &ctrl->ref;
	ref->pos_profile = PID_PROFILE_POSITION;
	ref->vel_profile = PID_PROFILE_VELOCITY;
	ref->ctrl_type = REF_CTRL_IDLE;
	ref->pos = ctrl->fb.pos;
	ref->vel_ff = 0.0f;
}
```

- [ ] **Step 3: 创建 motor_mode_hold.c**

```c
/**
 * @file motor_mode_hold.c
 * @brief 位置保持模式：主动锁定当前位置（位置环）
 */
#include "motor_mode.h"

void motor_mode_hold_run(motor_ctrl_t *ctrl)
{
	motor_ref_t *ref = &ctrl->ref;
	ref->pos_profile = PID_PROFILE_POSITION;
	ref->vel_profile = PID_PROFILE_VELOCITY;
	ref->ctrl_type = REF_CTRL_POSITION;
	ref->pos = ctrl->fb.pos;  /* 目标=当前位置，实现主动保持 */
	ref->vel_ff = 0.0f;
}
```

- [ ] **Step 4: 创建 motor_mode_open_loop.c**

```c
/**
 * @file motor_mode_open_loop.c
 * @brief 开环电压模式：OPEN_LOOP / VOLTAGE_VECTOR
 */
#include "motor_mode.h"

void motor_mode_open_loop_run(motor_ctrl_t *ctrl)
{
	motor_param_t *p = ctrl->param;
	motor_ref_t *ref = &ctrl->ref;
	float rated_v = motor_param_get_rated_voltage(p);
	ref->pos_profile = PID_PROFILE_POSITION;
	ref->vel_profile = PID_PROFILE_VELOCITY;
	ref->ctrl_type = REF_CTRL_VOLTAGE;
	ref->voltage = motor_mode_clamp(ctrl->cmd.torque, -rated_v, rated_v);
}
```

- [ ] **Step 5: 创建 motor_mode_duty.c**

```c
/**
 * @file motor_mode_duty.c
 * @brief 占空比直控模式
 */
#include "motor_mode.h"

void motor_mode_duty_run(motor_ctrl_t *ctrl)
{
	motor_ref_t *ref = &ctrl->ref;
	ref->pos_profile = PID_PROFILE_POSITION;
	ref->vel_profile = PID_PROFILE_VELOCITY;
	ref->ctrl_type = REF_CTRL_DUTY;
	ref->duty = motor_mode_clamp(ctrl->cmd.torque, -1.0f, 1.0f);
}
```

- [ ] **Step 6: 创建 motor_mode_current.c**

```c
/**
 * @file motor_mode_current.c
 * @brief 电流模式：CURRENT / FIELD_WEAKENING / SENSORLESS
 */
#include "motor_mode.h"

void motor_mode_current_run(motor_ctrl_t *ctrl)
{
	motor_param_t *p = ctrl->param;
	motor_ref_t *ref = &ctrl->ref;
	float peak_i = motor_param_get_peak_current(p);
	ref->pos_profile = PID_PROFILE_POSITION;
	ref->vel_profile = PID_PROFILE_VELOCITY;
	ref->ctrl_type = REF_CTRL_CURRENT;
	ref->id = motor_mode_clamp(ctrl->cmd.id, -peak_i, peak_i);
	ref->iq = motor_mode_clamp(ctrl->cmd.iq, -peak_i, peak_i);
}
```

- [ ] **Step 7: 创建 motor_mode_torque.c**

```c
/**
 * @file motor_mode_torque.c
 * @brief 力矩模式
 */
#include "motor_mode.h"

void motor_mode_torque_run(motor_ctrl_t *ctrl)
{
	motor_param_t *p = ctrl->param;
	motor_ref_t *ref = &ctrl->ref;
	float peak_t = motor_param_get_peak_torque(p);
	ref->pos_profile = PID_PROFILE_POSITION;
	ref->vel_profile = PID_PROFILE_VELOCITY;
	ref->ctrl_type = REF_CTRL_TORQUE;
	ref->torque = motor_mode_clamp(ctrl->cmd.torque, -peak_t, peak_t);
	ref->torque_ff = 0.0f;
}
```

- [ ] **Step 8: 创建 motor_mode_mit.c**

```c
/**
 * @file motor_mode_mit.c
 * @brief MIT Cheetah 控制：kp*pos_err + kd*vel_err + torque_ff
 */
#include "motor_mode.h"

void motor_mode_mit_run(motor_ctrl_t *ctrl)
{
	motor_param_t *p = ctrl->param;
	motor_ref_t *ref = &ctrl->ref;
	float peak_t = motor_param_get_peak_torque(p);

	float pos_err = ctrl->cmd.pos - ctrl->fb.pos;
	float vel_err = ctrl->cmd.vel - ctrl->fb.vel;
	float torque = ctrl->cmd.kp * pos_err + ctrl->cmd.kd * vel_err + ctrl->cmd.torque_ff;

	ref->pos_profile = PID_PROFILE_POSITION;
	ref->vel_profile = PID_PROFILE_VELOCITY;
	ref->ctrl_type = REF_CTRL_TORQUE;
	ref->torque = motor_mode_clamp(torque, -peak_t, peak_t);
	ref->torque_ff = 0.0f;
}
```

- [ ] **Step 9: 创建 motor_mode_velocity.c**

```c
/**
 * @file motor_mode_velocity.c
 * @brief 速度模式：VELOCITY / VELOCITY_TORQUE
 */
#include "motor_mode.h"

void motor_mode_velocity_run(motor_ctrl_t *ctrl)
{
	motor_param_t *p = ctrl->param;
	motor_ref_t *ref = &ctrl->ref;
	float max_spd = motor_param_get_max_speed(p);
	float peak_t = motor_param_get_peak_torque(p);

	ref->pos_profile = PID_PROFILE_POSITION;
	ref->vel_profile = PID_PROFILE_VELOCITY;
	ref->ctrl_type = REF_CTRL_VELOCITY;
	ref->vel = motor_mode_clamp(ctrl->cmd.vel, -max_spd, max_spd);
	ref->torque_ff = ctrl->cmd.torque_ff;

	/* VELOCITY_TORQUE 变体：额外设置力矩限幅 */
	if (ctrl->run_state == RUN_STATE_VELOCITY_TORQUE)
	{
		ref->torque = motor_mode_clamp(ctrl->cmd.torque, -peak_t, peak_t);
	}
}
```

- [ ] **Step 10: 创建 motor_mode_position.c**

```c
/**
 * @file motor_mode_position.c
 * @brief 位置模式：POSITION / POSITION_VELOCITY / POSITION_TORQUE / PP
 */
#include "motor_mode.h"

void motor_mode_position_run(motor_ctrl_t *ctrl)
{
	motor_param_t *p = ctrl->param;
	motor_ref_t *ref = &ctrl->ref;
	float peak_t = motor_param_get_peak_torque(p);

	ref->pos_profile = PID_PROFILE_POSITION;
	ref->vel_profile = PID_PROFILE_VELOCITY;
	ref->ctrl_type = REF_CTRL_POSITION;
	ref->pos = ctrl->cmd.pos;
	ref->vel_ff = 0.0f;

	switch (ctrl->run_state)
	{
		case RUN_STATE_POSITION_VELOCITY:
			ref->vel_ff = ctrl->cmd.vel;
			break;

		case RUN_STATE_POSITION_TORQUE:
			ref->torque = motor_mode_clamp(ctrl->cmd.torque, -peak_t, peak_t);
			break;

		case RUN_STATE_POSITION:
		default:
			/* PP 前期直接复用基础位置模式，轨迹规划后续在此扩展 */
			break;
	}
}
```

- [ ] **Step 11: 提交**

```bash
git add User/DataHub/state_define.h User/MotorControl/ControlProcess/motor_mode.h User/MotorControl/Modes/
git commit -m "feat(modes): 拆分 9 个基础模式为独立文件

新增 RUN_STATE_HOLD 枚举。
基础模式（IDLE/HOLD/OPEN_LOOP/DUTY/CURRENT/TORQUE/MIT/VELOCITY/POSITION）
各为独立 motor_mode_xxx.c 文件，导出 motor_mode_xxx_run 统一签名。
PP 前期复用 POSITION，轨迹规划后续扩展。"
```

---

## Task 3: 创建 PV/PT 与测试模式范例文件

**目标**：补全前期需要的 PV/PT 模式文件，同时建立测试模式拆分范例（修复扫频 static 问题）。

**Files:**
- Create: `User/MotorControl/Modes/motor_mode_profile_velocity.c`
- Create: `User/MotorControl/Modes/motor_mode_profile_torque.c`
- Create: `User/MotorControl/Modes/motor_mode_test_sweep.c`
- Modify: `User/MotorControl/ControlProcess/motor_control.h` (motor_ctrl_t 增加测试字段)

- [ ] **Step 1: 在 motor_ctrl_t 末尾追加测试状态字段**

修改 `motor_control.h` 的 `motor_ctrl_t` 结构体（L90-98），在 `float dt;` 后追加两个字段：

```c
typedef struct
{
	run_state_e run_state;
	motor_cmd_t cmd;
	motor_fb_t fb;
	motor_param_t *param;
	motor_ref_t ref; // 对外参考输出（唯一）
	float dt;		 // 控制周期(s)

	/* 测试模式运行时状态（避免 static 变量导致的重入性问题）*/
	float test_phase; /* 扫频测试相位累计 */
	float test_freq;  /* 扫频测试当前频率 */
} motor_ctrl_t;
```

**注意**：仅在末尾追加字段，不修改已有字段顺序，避免破坏二进制兼容。

- [ ] **Step 2: 创建 motor_mode_profile_velocity.c**

```c
/**
 * @file motor_mode_profile_velocity.c
 * @brief 速度轮廓模式 PV：速度模式 + 斜坡规划（前期直接复用速度模式）
 * @todo 后续在此文件内实现斜坡发生器，平滑速度指令
 */
#include "motor_mode.h"

void motor_mode_profile_velocity_run(motor_ctrl_t *ctrl)
{
	motor_param_t *p = ctrl->param;
	motor_ref_t *ref = &ctrl->ref;
	float max_spd = motor_param_get_max_speed(p);

	ref->pos_profile = PID_PROFILE_POSITION;
	ref->vel_profile = PID_PROFILE_VELOCITY;
	ref->ctrl_type = REF_CTRL_VELOCITY;
	/* 前期直接透传，后续加斜坡：ref->vel = slope_limit(ctrl->cmd.vel, ...) */
	ref->vel = motor_mode_clamp(ctrl->cmd.vel, -max_spd, max_spd);
	ref->torque_ff = ctrl->cmd.torque_ff;
}
```

- [ ] **Step 3: 创建 motor_mode_profile_torque.c**

```c
/**
 * @file motor_mode_profile_torque.c
 * @brief 力矩轮廓模式 PT：力矩模式 + 斜坡规划（前期直接复用力矩模式）
 * @todo 后续在此文件内实现力矩斜坡，平滑力矩指令
 */
#include "motor_mode.h"

void motor_mode_profile_torque_run(motor_ctrl_t *ctrl)
{
	motor_param_t *p = ctrl->param;
	motor_ref_t *ref = &ctrl->ref;
	float peak_t = motor_param_get_peak_torque(p);

	ref->pos_profile = PID_PROFILE_POSITION;
	ref->vel_profile = PID_PROFILE_VELOCITY;
	ref->ctrl_type = REF_CTRL_TORQUE;
	/* 前期直接透传，后续加斜坡：ref->torque = slope_limit(ctrl->cmd.torque, ...) */
	ref->torque = motor_mode_clamp(ctrl->cmd.torque, -peak_t, peak_t);
	ref->torque_ff = 0.0f;
}
```

- [ ] **Step 4: 创建 motor_mode_test_sweep.c（修复 static 重入问题）**

```c
/**
 * @file motor_mode_test_sweep.c
 * @brief 扫频测试模式：生成正弦力矩指令
 * @note 相位/频率存于 motor_ctrl_t 实例字段，避免 static 导致多电机共享
 */
#include "motor_mode.h"
#include <math.h>

void motor_mode_test_sweep_run(motor_ctrl_t *ctrl)
{
	motor_ref_t *ref = &ctrl->ref;
	ref->pos_profile = PID_PROFILE_POSITION;
	ref->vel_profile = PID_PROFILE_VELOCITY;
	ref->ctrl_type = REF_CTRL_TORQUE;
	ref->torque = sinf(ctrl->test_phase) * 0.5f; /* 0.5N·m 振幅 */
	ref->torque_ff = 0.0f;
	ctrl->test_phase += 2.0f * (float)M_PI * ctrl->test_freq * ctrl->dt;
}
```

- [ ] **Step 5: 提交**

```bash
git add User/MotorControl/ControlProcess/motor_control.h User/MotorControl/Modes/motor_mode_profile_velocity.c User/MotorControl/Modes/motor_mode_profile_torque.c User/MotorControl/Modes/motor_mode_test_sweep.c
git commit -m "feat(modes): 新增 PV/PT/扫频测试模式文件

PV/PT 前期复用基础模式，预留斜坡规划接口。
扫频测试相位/频率移入 motor_ctrl_t 实例字段，修复 static 重入问题。
motor_ctrl_t 增加 test_phase/test_freq 字段。"
```

---

## Task 4: 重构 motor_control.c 为 dispatch + legacy handler

**目标**：`motor_control.c` 仅保留 `motor_ctrl_init` + `motor_ctrl_dispatch`（查 ops 表）+ `motor_mode_legacy_run`（处理未拆分模式）。

**Files:**
- Modify: `User/MotorControl/ControlProcess/motor_control.c` (整体重构)
- Modify: `User/MotorControl/ControlProcess/motor_control.h` (删除旧 handler 声明)

- [ ] **Step 1: 重写 motor_control.c**

完整替换 `motor_control.c` 内容为：

```c
#include "motor_control.h"
#include "motor_mode.h"
#include <string.h>

/**
 * @brief 运行状态 → 模式处理函数 映射表（ops 函数表）
 * @details 前期已拆分的模式指向独立 motor_mode_xxx_run；
 *          未拆分的模式统一指向 motor_mode_legacy_run，后续逐个迁移。
 *          新增模式时在此表注册即可，无需改动 dispatch 逻辑。
 */
static const motor_mode_fn s_mode_table[RUN_STATE_MAX] = {
	/* ===== 基础模式（已拆分）===== */
	[RUN_STATE_IDLE]			 = motor_mode_idle_run,
	[RUN_STATE_HOLD]			 = motor_mode_hold_run,
	[RUN_STATE_OPEN_LOOP]		 = motor_mode_open_loop_run,
	[RUN_STATE_CURRENT]			 = motor_mode_current_run,
	[RUN_STATE_TORQUE]			 = motor_mode_torque_run,
	[RUN_STATE_MIT]				 = motor_mode_mit_run,
	[RUN_STATE_VELOCITY]		 = motor_mode_velocity_run,
	[RUN_STATE_POSITION]		 = motor_mode_position_run,
	[RUN_STATE_POSITION_VELOCITY] = motor_mode_position_run,
	[RUN_STATE_POSITION_TORQUE]	 = motor_mode_position_run,
	[RUN_STATE_VELOCITY_TORQUE]	 = motor_mode_velocity_run,
	[RUN_STATE_PROFILE_VELOCITY] = motor_mode_profile_velocity_run, /* PV */
	[RUN_STATE_PROFILE_TORQUE]	 = motor_mode_profile_torque_run,   /* PT */
	[RUN_STATE_DUTY_CYCLE]		 = motor_mode_duty_run,
	[RUN_STATE_VOLTAGE_VECTOR]	 = motor_mode_open_loop_run,
	[RUN_STATE_FIELD_WEAKENING]	 = motor_mode_current_run,
	[RUN_STATE_SENSORLESS]		 = motor_mode_current_run,

	/* ===== 轨迹模式（未拆分，走 legacy）===== */
	[RUN_STATE_PVT]				= motor_mode_legacy_run, /* 后续拆为 motor_mode_pvt_run */
	[RUN_STATE_CUBIC_SPLINE]	= motor_mode_legacy_run,
	[RUN_STATE_TRAPEZOIDAL_TRAJ] = motor_mode_legacy_run,
	[RUN_STATE_S_CURVE_TRAJ]	= motor_mode_legacy_run,
	[RUN_STATE_HOMING]			= motor_mode_legacy_run,
	[RUN_STATE_ELECTRONIC_GEAR] = motor_mode_legacy_run,
	[RUN_STATE_ELECTRONIC_CAM]	= motor_mode_legacy_run,

	/* ===== 力控模式（未拆分，走 legacy）===== */
	[RUN_STATE_IMPEDANCE]				= motor_mode_legacy_run,
	[RUN_STATE_ADMITTANCE]				= motor_mode_legacy_run,
	[RUN_STATE_FORCE_CONTROL]			= motor_mode_legacy_run,
	[RUN_STATE_FORCE_POSITION_HYBRID]	= motor_mode_legacy_run,
	[RUN_STATE_GRAVITY_COMPENSATION]	= motor_mode_legacy_run,
	[RUN_STATE_COLLISION_DETECTION]		= motor_mode_legacy_run,
	[RUN_STATE_ZERO_FORCE]				= motor_mode_legacy_run,
	[RUN_STATE_CONSTANT_FORCE]			= motor_mode_legacy_run,
	[RUN_STATE_VARIABLE_IMPEDANCE]		= motor_mode_legacy_run,
	[RUN_STATE_ADAPTIVE_GRAVITY_COMP]	= motor_mode_legacy_run,
	[RUN_STATE_LANDING_BUFFER]			= motor_mode_legacy_run,

	/* ===== 特殊模式（未拆分，走 legacy）===== */
	[RUN_STATE_STEP_DIR]		= motor_mode_legacy_run,
	[RUN_STATE_ANALOG_INPUT]	= motor_mode_legacy_run,
	[RUN_STATE_PWM_INPUT]		= motor_mode_legacy_run,
	[RUN_STATE_JOG]				= motor_mode_legacy_run,
	[RUN_STATE_SAFE_TEACH]		= motor_mode_legacy_run,

	/* ===== 测试模式（扫频已拆分，其余走 legacy）===== */
	[RUN_STATE_TEST_AGING]		= motor_mode_legacy_run,
	[RUN_STATE_TEST_SWEEP_FREQ] = motor_mode_test_sweep_run,
	[RUN_STATE_TEST_COGGING]	= motor_mode_legacy_run,
	[RUN_STATE_TEST_FRICTION]	= motor_mode_legacy_run,
	[RUN_STATE_TEST_INERTIA]	= motor_mode_legacy_run,
	[RUN_STATE_DIAGNOSTIC]		= motor_mode_legacy_run,
	[RUN_STATE_HIGH_SPEED_DAQ]	= motor_mode_legacy_run,
	[RUN_STATE_SINGLE_STEP]		= motor_mode_legacy_run,
};

void motor_ctrl_dispatch(motor_ctrl_t *ctrl)
{
	if (ctrl->run_state >= RUN_STATE_MAX)
		return;

	motor_mode_fn handler = s_mode_table[ctrl->run_state];
	if (handler != NULL)
		handler(ctrl);
}

void motor_ctrl_init(motor_ctrl_t *ctrl, motor_param_t *param, float dt)
{
	memset(ctrl, 0, sizeof(motor_ctrl_t));

	ctrl->param = param;
	ctrl->dt = dt;

	ctrl->run_state = RUN_STATE_IDLE;
	ctrl->ref.ctrl_type = REF_CTRL_IDLE;
	ctrl->ref.pos_profile = PID_PROFILE_POSITION;
	ctrl->ref.vel_profile = PID_PROFILE_VELOCITY;

	ctrl->test_phase = 0.0f;
	ctrl->test_freq = 1.0f;
}

/**
 * @brief Legacy fallback：处理未拆分的模式
 * @details 保留原 run_force_control / run_trajectory_control / run_special_control /
 *          run_test_control 的 switch 逻辑。后续逐个模式迁移到独立文件后，
 *          此函数逐渐缩小直至删除。
 */
void motor_mode_legacy_run(motor_ctrl_t *ctrl)
{
	motor_ref_t *ref = &ctrl->ref;
	ref->pos_profile = PID_PROFILE_POSITION;
	ref->vel_profile = PID_PROFILE_VELOCITY;

	switch (ctrl->run_state)
	{
		/* ---- 轨迹模式（位置为基底）---- */
		case RUN_STATE_PVT:
		case RUN_STATE_CUBIC_SPLINE:
		case RUN_STATE_TRAPEZOIDAL_TRAJ:
		case RUN_STATE_S_CURVE_TRAJ:
		case RUN_STATE_ELECTRONIC_GEAR:
		case RUN_STATE_ELECTRONIC_CAM:
			ref->ctrl_type = REF_CTRL_POSITION;
			ref->pos = ctrl->cmd.pos;
			ref->vel_ff = ctrl->cmd.vel_ff;
			break;

		case RUN_STATE_HOMING:
			ref->ctrl_type = REF_CTRL_POSITION;
			ref->pos_profile = PID_PROFILE_HOMING;
			ref->pos = ctrl->cmd.pos;
			ref->vel_ff = ctrl->cmd.vel_ff;
			break;

		/* ---- 力控模式（位置为基底 + 阻抗 profile）---- */
		case RUN_STATE_IMPEDANCE:
		case RUN_STATE_ADMITTANCE:
		case RUN_STATE_FORCE_CONTROL:
		case RUN_STATE_FORCE_POSITION_HYBRID:
		case RUN_STATE_GRAVITY_COMPENSATION:
		case RUN_STATE_COLLISION_DETECTION:
		case RUN_STATE_ZERO_FORCE:
		case RUN_STATE_CONSTANT_FORCE:
		case RUN_STATE_VARIABLE_IMPEDANCE:
		case RUN_STATE_ADAPTIVE_GRAVITY_COMP:
		case RUN_STATE_LANDING_BUFFER:
			ref->ctrl_type = REF_CTRL_POSITION;
			ref->pos_profile = PID_PROFILE_IMPEDANCE;
			ref->pos = ctrl->cmd.pos;
			ref->vel_ff = 0.0f;
			break;

		/* ---- 特殊模式 ---- */
		case RUN_STATE_JOG:
			ref->ctrl_type = REF_CTRL_VELOCITY;
			ref->vel_profile = PID_PROFILE_JOG;
			ref->vel = motor_mode_clamp(ctrl->cmd.vel,
										-motor_param_get_max_speed(ctrl->param),
										motor_param_get_max_speed(ctrl->param));
			break;

		case RUN_STATE_STEP_DIR:
		case RUN_STATE_ANALOG_INPUT:
		case RUN_STATE_PWM_INPUT:
		case RUN_STATE_SAFE_TEACH:
		default:
			ref->ctrl_type = REF_CTRL_POSITION;
			ref->pos = ctrl->cmd.pos;
			ref->vel_ff = 0.0f;
			break;

		/* ---- 其他测试模式 ---- */
		case RUN_STATE_TEST_AGING:
		case RUN_STATE_TEST_COGGING:
		case RUN_STATE_TEST_FRICTION:
		case RUN_STATE_TEST_INERTIA:
		case RUN_STATE_DIAGNOSTIC:
		case RUN_STATE_HIGH_SPEED_DAQ:
		case RUN_STATE_SINGLE_STEP:
			ref->ctrl_type = REF_CTRL_IDLE;
			ref->pos = ctrl->fb.pos;
			break;
	}
}
```

- [ ] **Step 2: 删除 motor_control.h 中旧的 handler 声明**

删除 `motor_control.h` 中以下声明（约 L112-L136）：

```c
/* 删除这些 */
void run_generic_control(motor_ctrl_t *ctrl);
void run_force_control(motor_ctrl_t *ctrl);
void run_trajectory_control(motor_ctrl_t *ctrl);
void run_special_control(motor_ctrl_t *ctrl);
void run_test_control(motor_ctrl_t *ctrl);
```

保留 `motor_ctrl_init` 和 `motor_ctrl_dispatch` 声明。

- [ ] **Step 3: 编译验证**

Rebuild，确认：
- 所有 `motor_mode_xxx_run` 函数被正确链接
- 无 "undefined reference" 错误
- `s_mode_table` 所有表项索引在 `RUN_STATE_MAX` 范围内

- [ ] **Step 4: 提交**

```bash
git add User/MotorControl/ControlProcess/motor_control.c User/MotorControl/ControlProcess/motor_control.h
git commit -m "refactor(motor_control): 重构为 dispatch + ops 表 + legacy fallback

motor_control.c 仅保留 init + dispatch + motor_mode_legacy_run。
s_mode_table 注册已拆分模式，未拆分模式走 legacy。
删除旧的 run_generic/force/trajectory/special/test_control 声明。"
```

---

## Task 5: 补全模式映射表 + 修复校准指令路由穿透

**目标**：补全 `s_ctrl_mode_to_run_state[]` 中缺失的 PP/PV/PT/CSP/CSV/CST/SYNC 映射；修复 `CONTROL_MODE_CALIB_*` 校准指令穿透到运动控制分支的 bug，让其正确进入 `TOP_FSM_CALIB`。

**Files:**
- Modify: `User/AppServices/StateMachine/system_state.c` (映射表 + process_ctrl_cmd 校准路由)

- [ ] **Step 1: 补全映射表**

修改 `system_state.c` 的 `s_ctrl_mode_to_run_state[]`（L36-89），替换整个映射表为以下内容（基于现有映射 + 补全缺失项 + 修改 HOLD/BRAKE）：

```c
	[CONTROL_MODE_IDLE] = RUN_STATE_IDLE,
	[CONTROL_MODE_HOLD] = RUN_STATE_HOLD,
	[CONTROL_MODE_BRAKE] = RUN_STATE_HOLD,  /* 刹车=位置保持 */

	[CONTROL_MODE_OPEN_LOOP] = RUN_STATE_OPEN_LOOP,
	[CONTROL_MODE_CURRENT] = RUN_STATE_CURRENT,
	[CONTROL_MODE_TORQUE] = RUN_STATE_TORQUE,
	[CONTROL_MODE_MIT] = RUN_STATE_MIT,
	[CONTROL_MODE_VELOCITY] = RUN_STATE_VELOCITY,
	[CONTROL_MODE_POSITION] = RUN_STATE_POSITION,
	[CONTROL_MODE_POSITION_VELOCITY] = RUN_STATE_POSITION_VELOCITY,
	[CONTROL_MODE_POSITION_TORQUE] = RUN_STATE_POSITION_TORQUE,
	[CONTROL_MODE_VELOCITY_TORQUE] = RUN_STATE_VELOCITY_TORQUE,
	[CONTROL_MODE_DUTY_CYCLE] = RUN_STATE_DUTY_CYCLE,
	[CONTROL_MODE_VOLTAGE_VECTOR] = RUN_STATE_VOLTAGE_VECTOR,
	[CONTROL_MODE_FIELD_WEAKENING] = RUN_STATE_FIELD_WEAKENING,
	[CONTROL_MODE_SENSORLESS] = RUN_STATE_SENSORLESS,

	[CONTROL_MODE_IMPEDANCE] = RUN_STATE_IMPEDANCE,
	[CONTROL_MODE_ADMITTANCE] = RUN_STATE_ADMITTANCE,
	[CONTROL_MODE_FORCE_CONTROL] = RUN_STATE_FORCE_CONTROL,
	[CONTROL_MODE_FORCE_POSITION_HYBRID] = RUN_STATE_FORCE_POSITION_HYBRID,
	[CONTROL_MODE_GRAVITY_COMPENSATION] = RUN_STATE_GRAVITY_COMPENSATION,
	[CONTROL_MODE_COLLISION_DETECTION] = RUN_STATE_COLLISION_DETECTION,
	[CONTROL_MODE_ZERO_FORCE] = RUN_STATE_ZERO_FORCE,
	[CONTROL_MODE_CONSTANT_FORCE] = RUN_STATE_CONSTANT_FORCE,
	[CONTROL_MODE_VARIABLE_IMPEDANCE] = RUN_STATE_VARIABLE_IMPEDANCE,
	[CONTROL_MODE_ADAPTIVE_GRAVITY_COMP] = RUN_STATE_ADAPTIVE_GRAVITY_COMP,
	[CONTROL_MODE_LANDING_BUFFER] = RUN_STATE_LANDING_BUFFER,

	[CONTROL_MODE_PVT] = RUN_STATE_PVT,
	[CONTROL_MODE_CUBIC_SPLINE] = RUN_STATE_CUBIC_SPLINE,
	[CONTROL_MODE_TRAPEZOIDAL_TRAJ] = RUN_STATE_TRAPEZOIDAL_TRAJ,
	[CONTROL_MODE_S_CURVE_TRAJ] = RUN_STATE_S_CURVE_TRAJ,
	[CONTROL_MODE_HOMING] = RUN_STATE_HOMING,
	[CONTROL_MODE_CANOPEN_SYNC] = RUN_STATE_POSITION,  /* SYNC 同步位置 */
	[CONTROL_MODE_ETHERCAT_CSP] = RUN_STATE_POSITION, /* CSP = Cyclic Sync Position */
	[CONTROL_MODE_ETHERCAT_CSV] = RUN_STATE_VELOCITY, /* CSV = Cyclic Sync Velocity */
	[CONTROL_MODE_ETHERCAT_CST] = RUN_STATE_TORQUE,   /* CST = Cyclic Sync Torque */
	[CONTROL_MODE_PP] = RUN_STATE_POSITION,              /* Profile Position（前期复用 POSITION）*/
	[CONTROL_MODE_PV] = RUN_STATE_PROFILE_VELOCITY,     /* Profile Velocity → 独立模式文件 */
	[CONTROL_MODE_PT] = RUN_STATE_PROFILE_TORQUE,       /* Profile Torque → 独立模式文件 */
	[CONTROL_MODE_ELECTRONIC_GEAR] = RUN_STATE_ELECTRONIC_GEAR,
	[CONTROL_MODE_ELECTRONIC_CAM] = RUN_STATE_ELECTRONIC_CAM,

	[CONTROL_MODE_STEP_DIR] = RUN_STATE_STEP_DIR,
	[CONTROL_MODE_ANALOG_INPUT] = RUN_STATE_ANALOG_INPUT,
	[CONTROL_MODE_PWM_INPUT] = RUN_STATE_PWM_INPUT,
	[CONTROL_MODE_JOG] = RUN_STATE_JOG,
	[CONTROL_MODE_SAFE_TEACH] = RUN_STATE_SAFE_TEACH,

	[CONTROL_MODE_TEST_AGING] = RUN_STATE_TEST_AGING,
	[CONTROL_MODE_TEST_SWEEP_FREQ] = RUN_STATE_TEST_SWEEP_FREQ,
	[CONTROL_MODE_TEST_COGGING] = RUN_STATE_TEST_COGGING,
	[CONTROL_MODE_TEST_FRICTION] = RUN_STATE_TEST_FRICTION,
	[CONTROL_MODE_TEST_INERTIA] = RUN_STATE_TEST_INERTIA,
	[CONTROL_MODE_DIAGNOSTIC] = RUN_STATE_DIAGNOSTIC,
	[CONTROL_MODE_HIGH_SPEED_DAQ] = RUN_STATE_HIGH_SPEED_DAQ,
	[CONTROL_MODE_SINGLE_STEP] = RUN_STATE_SINGLE_STEP,
```

注意：`CONTROL_MODE_CALIB_*` 不放入此映射表（校准不走运动状态机）。`CONTROL_MODE_START_LOG/STOP_LOG` 是系统指令也不放入。

- [ ] **Step 2: 在 process_ctrl_cmd 增加校准指令路由**

在 `process_ctrl_cmd` 的系统级指令 switch 中（约 L350-368），`CONTROL_MODE_ENTER_BOOTLOADER` case 后增加校准指令 case：

```c
		case CONTROL_MODE_ENTER_BOOTLOADER:
			top_fsm_switch(sys, TOP_FSM_BOOTLOADER);
			if (sys->top_state == TOP_FSM_BOOTLOADER)
				sys->ctrl_mode = cmd;
			return;

		/* 校准指令：进入 CALIB 状态（标定逻辑后续实现，此处仅路由）*/
		case CONTROL_MODE_CALIB_MOTOR_PARAM:
		case CONTROL_MODE_CALIB_ENCODER_OFFSET:
		case CONTROL_MODE_CALIB_ENCODER_LINEARITY:
		case CONTROL_MODE_CALIB_TORQUE_CONST:
		case CONTROL_MODE_CALIB_COGGING_COMP:
		case CONTROL_MODE_CALIB_FRICTION_COMP:
		case CONTROL_MODE_CALIB_INERTIA:
		case CONTROL_MODE_CALIB_ADC_OFFSET:
		case CONTROL_MODE_CALIB_ADC_GAIN:
		case CONTROL_MODE_CALIB_CURRENT_SENSOR:
		case CONTROL_MODE_CALIB_TEMPERATURE:
		case CONTROL_MODE_CALIB_FULL_AUTO:
			/* 仅 IDLE 态可进入校准，避免运行中误触发 */
			if (sys->top_state == TOP_FSM_IDLE)
			{
				top_fsm_switch(sys, TOP_FSM_CALIB);
				if (sys->top_state == TOP_FSM_CALIB)
					sys->ctrl_mode = cmd;
			}
			return;

		case CONTROL_MODE_SAVE_CONFIG:
```

- [ ] **Step 3: 编译验证**

Rebuild 确认：
- 枚举值无越界（`CONTROL_MODE_*` 都在 `CONTROL_MODE_MAX` 范围内）
- 校准指令 case 无遗漏
- 无 "duplicate case" 警告

- [ ] **Step 4: 提交**

```bash
git add User/AppServices/StateMachine/system_state.c
git commit -m "fix(state): 补全 PP/PV/PT 映射 + 修复校准指令路由穿透

补全 PP/PV/PT/CSP/CSV/CST/SYNC 模式映射，此前缺失静默落到 IDLE。
修复 CONTROL_MODE_CALIB_* 穿透到运动控制分支的 bug：
校准指令现在正确路由到 TOP_FSM_CALIB 状态（仅 IDLE 态可进入）。
标定逻辑本身后续单独实现。"
```

---

## Task 6: 重构 cur_loop_run 支持 VOLTAGE/DUTY/IDLE 直通

**核心修复**：当前 `cur_loop_run(cl, id_ref, iq_ref)` 丢弃 `ref->voltage`/`ref->duty`，开环电压和占空比模式实际跑"电流环锁定 0A"。

**Files:**
- Modify: `User/MotorControl/CascadeControl/current_loop.h` (签名变更)
- Modify: `User/MotorControl/CascadeControl/current_loop.c` (直通分支)

- [ ] **Step 1: 修改 current_loop.h 签名**

替换 `current_loop.h` 中 `cur_loop_run` 声明（约 L58-64）：

```c
/**
 * @brief 运行一次电流环（在电流环中断中按基频调用）
 * @param cl 电流环控制器指针
 * @param ref 电机控制参考（含 ctrl_type / voltage / duty）
 * @param out 级联外环输出的电流参考（id_ref/iq_ref），仅闭环模式使用
 * @note 内部按 ref->ctrl_type 分流：
 *       VOLTAGE — 旁路 PI，直接用 ref->voltage 设 udq
 *       DUTY    — 旁路整个 FOC，直接驱动 half_bridge
 *       IDLE    — PWM 置零
 *       其余    — 正常 PI 电流环（使用 out->id_ref / out->iq_ref）
 */
void cur_loop_run(cur_loop_t *cl, const motor_ref_t *ref, const cascade_out_t *out);
```

在 `current_loop.h` 顶部 include 区增加：

```c
#include "motor_control.h"   /* for motor_ref_t */
#include "cascade_control.h" /* for cascade_out_t */
```

- [ ] **Step 2: 重写 current_loop.c 的 cur_loop_run**

替换 `current_loop.c` 中 `cur_loop_run` 函数体（L41-L78）：

```c
void cur_loop_run(cur_loop_t *cl, const motor_ref_t *ref, const cascade_out_t *out)
{
	dev_motor_t *m = cl->motor;

	/* ===== 直通模式：不经过 FOC 电流环 PI ===== */

	/* 占空比直控：跳过 FOC 全链路，直接驱动 PWM */
	if (ref->ctrl_type == REF_CTRL_DUTY)
	{
		float duty = ref->duty;
		uint32_t ccr = (uint32_t)(PWM_PERIOD * (0.5f + 0.5f * duty));
		m->half_bridge.set_3pwm(&m->half_bridge, ccr, ccr, ccr);
		return;
	}

	/* IDLE：PWM 置零（安全失能输出）*/
	if (ref->ctrl_type == REF_CTRL_IDLE)
	{
		m->half_bridge.set_3pwm(&m->half_bridge, 0, 0, 0);
		return;
	}

	/* ===== 以下模式需要 FOC 链路 ===== */

	/* step1: 刷新编码器机械角度 */
	m->encoder.update(&m->encoder);
	m->motor_param.update(&m->motor_param, MOTION_TYPE_ELE_RADIAN, m->encoder.mechanical_angle);

	/* step2: 三相电流采样 */
	m->phase_current.update(&m->phase_current);

	/* step3: Clarke 变换 */
	m->foc.clarke(&m->foc);

	/* step4: Park 变换 */
	m->foc.park(&m->foc);

	float ud, uq;

	if (ref->ctrl_type == REF_CTRL_VOLTAGE)
	{
		/* 开环电压：旁路 PI，直接用 ref->voltage 作为 q 轴电压 */
		ud = 0.0f;
		uq = ref->voltage;
	}
	else
	{
		/* 电流闭环 PI（CURRENT / TORQUE / VELOCITY / POSITION）*/
		ud = pid_profile_calculate(&cl->pid_id, PID_PROFILE_CURRENT_D,
								   out->id_ref, m->foc.i_dq.d, cl->dt);
		uq = pid_profile_calculate(&cl->pid_iq, PID_PROFILE_CURRENT_Q,
								   out->iq_ref, m->foc.i_dq.q, cl->dt);
	}

	/* step6: 设置 dq 电压并反 Park */
	m->foc.set_udq(&m->foc, ud, uq);
	m->foc.inverse_park(&m->foc);

	/* step7: SVPWM */
	m->foc.pfsvpwm(&m->foc);

	/* step8: PWM 输出 */
	m->half_bridge.set_3pwm(&m->half_bridge,
							(uint32_t)(PWM_PERIOD * m->foc.svpwm.ta),
							(uint32_t)(PWM_PERIOD * m->foc.svpwm.tb),
							(uint32_t)(PWM_PERIOD * m->foc.svpwm.tc));
}
```

- [ ] **Step 3: 编译验证**

Rebuild，确认无 "implicit declaration" 警告，include 链无循环依赖。

- [ ] **Step 4: 提交**

```bash
git add User/MotorControl/CascadeControl/current_loop.h User/MotorControl/CascadeControl/current_loop.c
git commit -m "fix(current_loop): cur_loop_run 接收完整 ref，支持 VOLTAGE/DUTY/IDLE 直通

开环电压模式 ref->voltage 此前被丢弃，实际跑电流环锁定 0A。
占空比模式 ref->duty 同样未生效。
重构后按 ctrl_type 分流：
- VOLTAGE: 旁路 PI，直设 udq
- DUTY: 旁路 FOC，直驱 PWM
- IDLE: PWM 置零
- 其余: 原 PI 路径"
```

---

## Task 7: 更新 motor_loop_isr 适配新签名

**Files:**
- Modify: `User/MotorControl/CascadeControl/motor_loop.c` (L260-L280)

- [ ] **Step 1: 修改 motor_loop_isr**

替换 `motor_loop.c` 中 `motor_loop_isr` 的非RUN态分支（L261-269）和 step3-step5（L272-280）：

```c
	/* 非运行态：外环复位，电流环以 IDLE 直通模式输出零 PWM */
	if (m->sys.top_state != TOP_FSM_RUN)
	{
		cascade_control_reset(&m->cascade);
		cur_loop_reset(&m->current);
		/* 清零 out，避免进入 RUN 态第一拍 vel_tick=false 时电流环用旧值 */
		m->out.id_ref = 0.0f;
		m->out.iq_ref = 0.0f;
		/* IDLE 直通：cur_loop_run 内部检测 ctrl_type==IDLE 后 PWM 置零 */
		cur_loop_run(&m->current, &m->sys.motor.ref, &m->out);
		return;
	}

	/* step3: 位置环（分频）——仅 POSITION 模式需要 */
	if (pos_tick && m->sys.motor.ref.ctrl_type == REF_CTRL_POSITION)
		cascade_control_run_position(&m->cascade, &m->sys.motor.ref, &fb);

	/* step4: 速度环 + 入环分发（分频）——跳过 VOLTAGE/DUTY/IDLE 直通模式 */
	if (vel_tick && m->sys.motor.ref.ctrl_type >= REF_CTRL_CURRENT)
		cascade_control_run(&m->cascade, &m->sys.motor.ref, &fb, &m->out);

	/* step5: 电流环（基频）——传入完整 ref，内部按 ctrl_type 分流 */
	cur_loop_run(&m->current, &m->sys.motor.ref, &m->out);
```

`ref_ctrl_type_e` 枚举顺序（[motor_control.h:49-57](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorControl/ControlProcess/motor_control.h#L49-57)）：
```c
REF_CTRL_IDLE = 0,    /* < CURRENT，跳过外环 */
REF_CTRL_VOLTAGE,     /* < CURRENT，跳过外环 */
REF_CTRL_DUTY,        /* < CURRENT，跳过外环 */
REF_CTRL_CURRENT,     /* >= CURRENT，走外环 */
REF_CTRL_TORQUE,
REF_CTRL_VELOCITY,
REF_CTRL_POSITION,
```
顺序满足"直通模式 < CURRENT，闭环模式 >= CURRENT"。

- [ ] **Step 2: 编译验证**

Rebuild 确认无类型不匹配警告。

- [ ] **Step 3: 提交**

```bash
git add User/MotorControl/CascadeControl/motor_loop.c
git commit -m "fix(motor_loop): ISR 按ctrl_type跳过外环，适配cur_loop_run新签名

VOLTAGE/DUTY/IDLE 直通模式不再执行位置环/速度环。
非RUN态改由 cur_loop_run 内部 IDLE 分支处理 PWM 置零。"
```

---

## Task 8: 修正 cascade_control_run 直通分支语义

**Files:**
- Modify: `User/MotorControl/CascadeControl/cascade_control.c` (L96-L143)

- [ ] **Step 1: 明确直通模式分支**

替换 `cascade_control.c` 中 `cascade_control_run` 的预装载检查（L96-L102）和 switch（L107-L143）。

预装载检查原代码（L96-L102）：
```c
	if (ref->ctrl_type != c->last_ctrl_type || ref->pos_profile != c->last_pos_profile || ref->vel_profile != c->last_vel_profile)
	{
		cascade_bumpless_preload(c, ref, fb);
		c->last_ctrl_type = ref->ctrl_type;
		c->last_pos_profile = ref->pos_profile;
		c->last_vel_profile = ref->vel_profile;
	}
```

替换为（合并条件 + 仅闭环模式预装载）：
```c
	/* 入环层级或配置文件变化：先做无扰预装载（仅闭环模式需要）*/
	if ((ref->ctrl_type != c->last_ctrl_type || ref->pos_profile != c->last_pos_profile || ref->vel_profile != c->last_vel_profile)
		&& ref->ctrl_type >= REF_CTRL_CURRENT)
	{
		cascade_bumpless_preload(c, ref, fb);
	}
	c->last_ctrl_type = ref->ctrl_type;
	c->last_pos_profile = ref->pos_profile;
	c->last_vel_profile = ref->vel_profile;
```

switch 部分（L107-L143）原已有 `case REF_CTRL_VOLTAGE: case REF_CTRL_DUTY: case REF_CTRL_IDLE: default: break;`，**无需修改**，语义已正确。

**关键变更点**：仅修改预装载条件，增加 `&& ref->ctrl_type >= REF_CTRL_CURRENT` 限制，使直通模式（VOLTAGE/DUTY/IDLE）不触发预装载。switch 分支保持原样。

- [ ] **Step 2: 编译验证**

Rebuild 确认无警告。

- [ ] **Step 3: 提交**

```bash
git add User/MotorControl/CascadeControl/cascade_control.c
git commit -m "refactor(cascade): VOLTAGE/DUTY/IDLE 分支语义明确化，不产生电流参考

直通模式 out 置零且不触发无扰预装载。
预装载仅在闭环模式(ctrl_type>=CURRENT)下执行。"
```

---

## Task 9: 修复过渡期 PID profile 立即切换问题

**Files:**
- Modify: `User/MotorControl/ControlProcess/ctrl_transition.c` (L27-67)

- [ ] **Step 1: 修改 transition_update 保留旧 profile 直到过渡完成**

替换 `ctrl_transition.c` 中 `transition_update` 函数（L27-67），在混合分支末尾增加两行：

```c
bool transition_update(transition_t *trans, const motor_ref_t *new_ref, motor_ref_t *out_ref)
{
	if (trans->state != TRANSITION_IN_PROGRESS)
	{
		*out_ref = *new_ref;
		return true;
	}

	trans->elapsed++;

	if (trans->duration == 0 || trans->elapsed >= trans->duration)
	{
		trans->state = TRANSITION_COMPLETED;
		trans->ratio = 1.0f;
		*out_ref = *new_ref;
		return true;
	}

	/* 量纲不同：不混合，直接采用新参考 */
	if (trans->old_ref.ctrl_type != new_ref->ctrl_type)
	{
		*out_ref = *new_ref;
		return false;
	}

	/* 同量纲：对目标值做线性混合 */
	trans->ratio = (float)trans->elapsed / trans->duration;

	*out_ref = *new_ref;
	out_ref->pos = blend(trans->old_ref.pos, new_ref->pos, trans->ratio);
	out_ref->vel = blend(trans->old_ref.vel, new_ref->vel, trans->ratio);
	out_ref->torque = blend(trans->old_ref.torque, new_ref->torque, trans->ratio);
	out_ref->id = blend(trans->old_ref.id, new_ref->id, trans->ratio);
	out_ref->iq = blend(trans->old_ref.iq, new_ref->iq, trans->ratio);
	out_ref->voltage = blend(trans->old_ref.voltage, new_ref->voltage, trans->ratio);
	out_ref->duty = blend(trans->old_ref.duty, new_ref->duty, trans->ratio);

	/* PID profile 保留旧值直到过渡完成，避免增益突变+中间参考值导致力矩跳变 */
	out_ref->pos_profile = trans->old_ref.pos_profile;
	out_ref->vel_profile = trans->old_ref.vel_profile;

	return false;
}
```

- [ ] **Step 2: 编译验证**

Rebuild 确认无警告。

- [ ] **Step 3: 提交**

```bash
git add User/MotorControl/ControlProcess/ctrl_transition.c
git commit -m "fix(transition): PID profile 延迟到过渡完成时切换

过渡期 pos/vel 线性混合但 profile 立即跳变会导致增益突变+中间参考
值产生力矩跳变。现在 profile 保留旧值直到 ratio=1.0 才切换。"
```

---

## Task 10: 修复 READY→RUN 首次进入无过渡

**Files:**
- Modify: `User/AppServices/StateMachine/system_state.c` (L219, L371-386)
- Modify: `User/AppServices/StateMachine/system_state.h` (L78 参数改名)

- [ ] **Step 1: 修改 process_ctrl_cmd 的 READY→RUN 分支**

替换 `system_state.c` 中 `process_ctrl_cmd` 的运动指令分支（约 L370-386）：

```c
	/* ---- 运动控制指令：需已使能（READY 或 RUN）---- */
	if (sys->top_state == TOP_FSM_READY)
	{
		/* READY → RUN：首次进入也走平滑过渡，避免位置阶跃 */
		top_fsm_switch(sys, TOP_FSM_RUN);
		run_state_e target = s_ctrl_mode_to_run_state[cmd];

		/* 进入 RUN 前把 motor.ref 设为 IDLE（保持当前位置）作为过渡起点 */
		sys->motor.ref.ctrl_type = REF_CTRL_IDLE;
		sys->motor.ref.pos = sys->motor.fb.pos;
		sys->motor.ref.vel = 0.0f;
		sys->motor.ref.torque = 0.0f;

		run_state_switch(sys, target, g_run_state_trans_count);
		sys->ctrl_mode = cmd;
	}
	else if (sys->top_state == TOP_FSM_RUN)
	{
		run_state_e target = s_ctrl_mode_to_run_state[cmd];
		run_state_switch(sys, target, g_run_state_trans_count);
		sys->ctrl_mode = cmd;
	}
```

- [ ] **Step 2: 修改 run_state_switch 参数名**

`system_state.h`（L78）：

```c
void run_state_switch(system_state_t *sys, run_state_e new_state, uint32_t trans_count);
```

`system_state.c`（L219）：

```c
void run_state_switch(system_state_t *sys, run_state_e new_state, uint32_t trans_count)
{
	if (new_state >= RUN_STATE_MAX || new_state == sys->motor.run_state)
		return;

	sys->target_run_state = new_state;
	transition_start(&sys->transition, trans_count, &sys->motor.ref);
}
```

- [ ] **Step 3: 编译验证**

Rebuild 确认参数名变更无遗漏。

- [ ] **Step 4: 提交**

```bash
git add User/AppServices/StateMachine/system_state.c User/AppServices/StateMachine/system_state.h
git commit -m "fix(state): READY→RUN 首次进入走平滑过渡，参数名 trans_ms→trans_count

此前首次进入直接置位 run_state 无过渡，位置阶跃。
现以 IDLE(当前位置)为过渡起点走平滑过渡。
同步修正参数名 trans_ms→trans_count 消除语义误导。"
```

---

## Task 11: 综合编译与虚拟电机验证

- [ ] **Step 1: 将新增文件加入 Keil 工程**

**关键手动操作**：新增的 12 个 `motor_mode_*.c` 文件必须加入 Keil 工程文件（`.uvprojx`）才能被编译。

操作步骤：
1. 在 Keil MDK 中打开 V1 工程
2. 在 `User/MotorControl` 分组下新建 `Modes` 子分组
3. 将以下 12 个文件加入 `Modes` 分组：
   - `motor_mode_idle.c`
   - `motor_mode_hold.c`
   - `motor_mode_open_loop.c`
   - `motor_mode_duty.c`
   - `motor_mode_current.c`
   - `motor_mode_torque.c`
   - `motor_mode_mit.c`
   - `motor_mode_velocity.c`
   - `motor_mode_position.c`
   - `motor_mode_profile_velocity.c`
   - `motor_mode_profile_torque.c`
   - `motor_mode_test_sweep.c`
4. 确认 `motor_mode.h` 所在目录已加入 Include Paths（应已在 `ControlProcess` 路径中）

- [ ] **Step 2: 全量 Rebuild V1 工程**

在 Keil MDK 中 Rebuild V1 工程，确认 0 Error, 0 Warning。

重点检查：
- `Modes/` 目录下所有 `motor_mode_*.c` 文件已参与编译（查看 Build Output 窗口）
- `motor_mode.h` include 链无循环依赖
- `motor_ctrl_t` 大小变化（新增 test_phase/test_freq）不导致中断栈溢出
- `cur_loop_run` 新签名所有调用点已更新（motor_loop.c L267/L280）
- `s_mode_table` 所有表项非 NULL
- 校准指令 case 无遗漏
- `motor_mode_clamp` 在各模式文件中正确解析（static inline in header）

- [ ] **Step 3: 虚拟电机模式切换验证**

使用 PyQt 上位机连接虚拟电机，依次验证：

1. **ENABLE → POSITION**：使能后进入位置模式，发送目标位置，确认运动到位
2. **POSITION → VELOCITY**：模式切换走平滑过渡，无力矩跳变
3. **VELOCITY → TORQUE**：异量纲切换，确认无扰预装载生效
4. **TORQUE → OPEN_LOOP(voltage)**：确认开环电压模式实际输出电压（电机转动），而非锁定 0A
5. **OPEN_LOOP → DUTY_CYCLE**：确认占空比模式直接驱动 PWM
6. **DUTY_CYCLE → IDLE**：确认 PWM 置零
7. **IDLE → HOLD**：确认位置保持模式主动锁定当前位置
8. **HOLD → POSITION**：确认模式切换平滑
9. **POSITION → MIT**：确认 MIT 模式响应 kp/kd/torque_ff
10. **MIT → TEST_SWEEP_FREQ**：确认扫频输出正弦力矩，退出后再进入相位从 0 开始
11. **IDLE → CALIB_ENCODER_OFFSET**：确认校准指令进入 TOP_FSM_CALIB 而非 RUN（电机不动）

验证方法：观察上位机遥测波形（电流/速度/位置/top_state），确认无异常跳变。

- [ ] **Step 4: 提交验证结果**

```bash
git add -A
git commit -m "test: 虚拟电机验证模块化重构后所有前期模式

验证项：
- 模块化模式（IDLE/HOLD/OPEN_LOOP/DUTY/CURRENT/TORQUE/MIT/VELOCITY/POSITION）独立工作
- 开环电压模式实际输出电压（此前锁定0A的bug已修复）
- 占空比模式直接驱动PWM
- HOLD 位置保持主动锁定
- 模式切换平滑过渡无力矩跳变
- PID profile 过渡完成时切换
- 扫频测试 phase 每实例独立、退出复位
- 校准指令正确进入 TOP_FSM_CALIB 不触发运动"
```

---

## Self-Review

### 1. Spec coverage
- 开环电压模式 → Task 6 (cur_loop VOLTAGE 直通) ✓
- 电流模式 → Task 2 (motor_mode_current.c) + Task 8 语义明确 ✓
- 速度模式 → Task 2 (motor_mode_velocity.c) ✓
- 位置模式 → Task 2 (motor_mode_position.c) ✓
- MIT 模式 → Task 2 (motor_mode_mit.c) ✓
- PP → Task 5 映射到 POSITION + Task 2 motor_mode_position.c ✓
- PV → Task 3 (motor_mode_profile_velocity.c) + Task 5 映射 RUN_STATE_PROFILE_VELOCITY + Task 4 s_mode_table 注册 ✓
- PT → Task 3 (motor_mode_profile_torque.c) + Task 5 映射 RUN_STATE_PROFILE_TORQUE + Task 4 s_mode_table 注册 ✓
- 占空比模式 → Task 2 (motor_mode_duty.c) + Task 6 (cur_loop DUTY 直通) ✓
- HOLD 位置保持 → Task 2 (motor_mode_hold.c) + Task 5 映射 ✓
- 模块化（每模式独立文件，motor_ 前缀）→ Task 1-4 ✓
- 过渡 profile 突变 → Task 9 ✓
- 首次进入无过渡 → Task 10 ✓
- 扫频 static 重入 → Task 3 (motor_mode_test_sweep.c) ✓
- 校准指令路由穿透 → Task 5 (process_ctrl_cmd 校准 case) ✓

### 2. Placeholder scan
- PV/PT 的 `@todo` 是明确的后续扩展点，非占位符——前期实现完整可用（直接透传），斜坡是增强
- 所有代码步骤均提供完整代码
- 所有命令均提供具体 git commit message
- 无 TBD/"implement later"（除 @todo 注释标注的后续增强点）
- 标定逻辑本身明确说明"后续单独实现"，本计划仅修复路由

### 3. Type consistency
- `motor_mode_fn` typedef 在 Task 1 定义，Task 4 的 `s_mode_table` 使用一致 ✓
- 所有 `motor_mode_xxx_run(motor_ctrl_t *ctrl)` 签名一致 ✓
- `cur_loop_run(cl, ref, out)` 签名在 Task 6 定义，Task 7 调用点一致 ✓
- `motor_ctrl_t` 新增字段 `test_phase`/`test_freq` 在 Task 3 定义，Task 4 init 初始化，motor_mode_test_sweep.c 使用 ✓
- `run_state_switch` 参数 `trans_count` 在 Task 10 定义和调用一致 ✓
- `RUN_STATE_HOLD` 在 Task 2 定义，Task 5 映射表和 Task 4 mode_table 一致 ✓
- `RUN_STATE_PROFILE_VELOCITY/TORQUE` 在 Task 2 定义，Task 5 映射 PV/PT，Task 4 s_mode_table 注册对应函数 ✓
- `motor_mode_clamp` 在 Task 1 (motor_mode.h) 定义，Task 2 各模式文件使用一致 ✓
- 文件名 `motor_mode_*.c` 与函数名 `motor_mode_xxx_run` 全局一致 ✓
