# 同模式目标值渐变与过渡模块重构（Intra-Mode Ref Smoothing & Transition Mgr Refactor）实现方案

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在不破坏现有"模式切换过渡"的前提下，为同一运行模式内的目标值突变（如 POSITION 模式从 0 rad 跳到 10 rad）增加参考层线性渐变，消除电机因目标阶跃产生的抽动；同时借机重构过渡模块边界，把"何时 start / 何时 update / 何时 force_complete"的策略从 `system_state.c` 收敛到独立的 `ctrl_transition_mgr` 模块，提升模块化与可扩展性。

**Architecture:** 两阶段递进：
1. **重构阶段**：新增 `ctrl_transition_mgr.c/h`（过渡调度管理器），封装所有过渡策略（模式切换过渡、同模式渐变、应急终止）。`transition_t` 实例与 `ref_smooth_cfg_t` 配置从 `system_state_t` / 全局变量迁入 `transition_mgr_t` 内部（**不保留全局 `g_ref_smooth_cfg`，配置完全内聚于 mgr**）。`system_state.c` 不再直接操作 `transition_*`，改为调用 `transition_mgr_*` API。
2. **同模式渐变阶段**：在 `transition_mgr_step()` 内部，无模式切换过渡时检测同模式目标突变，超阈值时以"上一拍输出"为 `old_ref` 启动渐变，复用 `transition_update` 的 blend 逻辑。**是否渐变以 `run_state`（运行子状态）白名单判定，而非 `ctrl_type`**——因为 MIT 与 TORQUE 模式都输出 `REF_CTRL_TORQUE`（见 [motor_mode_mit.c:19](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorControl/Modes/motor_mode_mit.c#L19)、[motor_mode_torque.c:14](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorControl/Modes/motor_mode_torque.c#L14)），`ctrl_type` 无法区分二者。MIT（反馈型柔顺控制）、HOLD（目标跟随反馈）、开环/占空比等模式不启用本机制。

**配置两层语义：**
- **enable 总开关**：`mgr.smooth_cfg.enable = false` 时同模式渐变完全跳过，恢复改前行为（便于对比测试与应急回退）。
- **过渡时长两种模式二选一**：
  - **速率模式（推荐）**：配置 `pos_rate`（rad/s）、`vel_rate`（rad/s²）等"每秒最大变化量"，启动时按 `duration = ceil(|delta| / (rate × dt))` 自动计算过渡拍数，变化量大则过渡长、变化量小则过渡短，更符合物理直觉。
  - **固定时长模式**：配置 `smooth_duration`（调用次数），所有突变统一过渡时长。`rate` 字段为 0 时回退到此模式。

**关键前提（经代码核对）：**
- **控制周期 dt = 100µs**（电流环周期，`current_freq_hz = 10kHz`）。`transition_mgr_init` 接收 dt 并存入 `mgr.smooth_cfg` / mgr 内部，rate 模式据此算 duration。dt 必须支持运行期可配置（见 Task 8 / Task 9）。
- **判据用 `run_state` 而非 `ctrl_type`**：白名单只放行纯参考型闭环模式（POSITION 系列 / VELOCITY / TORQUE / CURRENT），排除 MIT（`RUN_STATE_MIT`，反馈型）、HOLD（目标每拍跟随 `fb.pos`）、OPEN_LOOP / DUTY / VOLTAGE（直控）。
- **`transition_t` 的 `COMPLETED` 态需显式复位为 `IDLE`**：模式切换过渡与同模式渐变共用同一 `transition_t` 实例，过渡完成后引擎停在 `COMPLETED`，`ref_smooth_check` 入口必须把 `COMPLETED` 复位为 `IDLE` 并跳过本拍检测，否则模式切换完成那拍会误触发一次同模式渐变。

**Tech Stack:** C99、Keil MDK（STM32G474）、现有 `ctrl_transition` 引擎、Host GCC 单元测试（可选）、虚拟电机模式集成验证。

---

## 背景与根因

### 现有过渡机制（模式切换过渡）

- `transition_t`（[ctrl_transition.h](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorControl/ControlProcess/ctrl_transition.h)）工作在参考层，对 `motor_ref_t` 做线性 blend。
- `transition_start()` 在 `run_state_switch()`（[system_state.c:236](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/AppServices/StateMachine/system_state.c#L236)）中调用，**仅当目标 run_state 与当前不同**时启动。
- `motor_control_loop()`（[system_state.c:250](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/AppServices/StateMachine/system_state.c#L250)）的 if 分支每拍 dispatch 目标模式、blend 输出，过渡完成后正式切到 target_run_state。
- 过渡时长以 `motor_control_loop` 调用次数计，全局 `g_run_state_trans_count = 1000`。

### 问题根因

1. **同模式目标突变不触发过渡**：[system_state.c:238](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/AppServices/StateMachine/system_state.c#L238) `run_state_switch` 在 `new_state == sys->motor.run_state` 时直接 return，不调用 `transition_start`。
2. **模式 handler 直接赋值**：如 [motor_mode_position.c:16](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorControl/Modes/motor_mode_position.c#L16) `ref->pos = ctrl->cmd.pos`，目标阶跃直达 PID。
3. **PID 立即响应大误差** → 力矩饱和/电流冲击 → 电机抽动。

### 现有模块边界问题（重构动机）

调研过渡模块所有调用点（见下方调用关系图），发现：

1. **过渡触发逻辑散落在 system_state.c**：`transition_start` 在 `run_state_switch`、`process_ctrl_cmd`（READY→RUN 分支）两处调用，`transition_force_complete` 在 `top_fsm_switch` 两处调用，触发条件分散在状态机各处。
2. **`transition_t` 实例归属模糊**：作为 `system_state_t` 成员（[system_state.h:40](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/AppServices/StateMachine/system_state.h#L40)），但状态机自己调用 update，语义上既是拥有者又是使用者。
3. **新增同模式渐变会让 system_state.c 更臃肿**：若直接在 `motor_control_loop` else 分支插入 ref_smooth 逻辑，状态机层承担过渡细节，违反单一职责。
4. **配置全局散落**：`g_ref_smooth_cfg` 作为全局变量，多实例不可扩展。

### 现有调用关系

```
入口层:  jm_proto_ops.c ──motor_loop_set_cmd──→ motor_loop.c
                       │                           │
                       │              motor_control_loop/process_ctrl_cmd
                       ▼                           ▼
状态机层:                    system_state.c (内含 transition_t 成员)
                       │  5 处直接调用 transition_*
                       ▼
过渡引擎:              ctrl_transition.c (纯计算: init/start/update/force_complete)

                       system_state.c ──motor_ctrl_dispatch──→ motor_control.c ──→ motor_mode_*.c
                                       (写 sys.motor.ref 阶跃)
```

`system_state.c` 直接调用过渡引擎 5 处：`init`(1) + `force_complete`(2) + `start`(1) + `update`(1)。

### 数据流回顾

```
协议层 app_set_mode() 写 sys.motor.cmd.{pos,vel,...}
  → motor_loop_set_cmd() → process_ctrl_cmd()
    → 若 RUN 态且同模式: run_state_switch() 直接 return（不启动过渡）
  → motor_loop_isr() → motor_control_loop()
    → else 分支: motor_ctrl_dispatch() → motor_mode_xxx_run()
      → ref->pos = ctrl->cmd.pos  （阶跃！）
```

## 设计方案

### 核心思路（两阶段递进）

**阶段1：模块重构**——新增 `ctrl_transition_mgr`，收拢过渡策略。

**阶段2：同模式渐变**——在 `transition_mgr_step()` else 分支内实现 ref_smooth，逻辑不变但归属 mgr。

### 重构后目标架构

```
入口层:  jm_proto_ops.c ──motor_loop_set_cmd──→ motor_loop.c
                       │                           │
                       │              motor_control_loop/process_ctrl_cmd
                       ▼                           ▼
状态机层:                    system_state.c (瘦身: 仅状态流转)
                       │  3 处调用 transition_mgr_*
                       ▼
过渡调度层:              ctrl_transition_mgr.c (新增: 策略封装)
                       │  内含 transition_t + ref_smooth_cfg
                       ├─→ ctrl_transition.c (引擎, 不变)
                       └─→ motor_ctrl_dispatch (写 raw_ref)

                       transition_mgr_step ──mixed_ref──→ cascade_control.c
```

### 模块职责对照

| 模块 | 重构前 | 重构后 |
|------|--------|--------|
| [ctrl_transition.c](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorControl/ControlProcess/ctrl_transition.c) | 纯计算引擎 | **不变** |
| **ctrl_transition_mgr.c**（新增） | — | 过渡策略封装：模式切换过渡 + 同模式渐变 + 应急终止 |
| [system_state.c](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/AppServices/StateMachine/system_state.c) | 直接调 `transition_*` 5 处 | 调 `transition_mgr_*` 3 处，瘦身 |
| [motor_control.c](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorControl/ControlProcess/motor_control.c) | dispatch | **不变** |

### 设计要点

- **enable 总开关**：`mgr.smooth_cfg.enable = false` 直接跳过整个 ref_smooth 逻辑，恢复改前行为。便于对比测试、应急回退、特定工况临时禁用。
- **复用 blend**：`transition_update` 已对同 ctrl_type 的 pos/vel/torque/id/iq/ud/voltage/duty 做线性混合，直接复用。
- **old_ref = 上一拍实际输出**：突变启动时以 `prev_ref`（上拍 mixed_ref 或 raw_ref）为起点，保证参考连续。
- **过渡中不重启**：`transition_ref_smooth_check` 在 `state == IN_PROGRESS` 时直接 return false，让当前渐变完成（朝最新 raw_ref 混合，因为每拍 dispatch 产生新 raw_ref）。
- **COMPLETED 态显式复位**：`state == COMPLETED` 时把它复位为 `IDLE` 并 return false（本拍不检测）。这一步解决"模式切换过渡与同模式渐变共用同一 `transition_t`"带来的误触发——模式切换刚完成那拍不会被当成同模式突变启动渐变。
- **按 `run_state` 白名单启用（关键修正）**：判据从 `ctrl_type` 改为 `run_state`。仅 `RUN_STATE_POSITION / POSITION_VELOCITY / POSITION_TORQUE / VELOCITY / TORQUE / CURRENT` 启用。**排除 `RUN_STATE_MIT`**——MIT 输出 `REF_CTRL_TORQUE`，与纯 TORQUE 模式的 `ctrl_type` 相同，用 `ctrl_type` 无法区分，且 MIT 的 `ref->torque = kp·pos_err + kd·vel_err + torque_ff` 是依赖实时反馈的闭环量，对其做参考层 blend 无物理意义并破坏刚度/阻尼特性。同时排除 HOLD（目标跟随 `fb.pos`）、OPEN_LOOP / DUTY / VOLTAGE（直控）。`ctrl_type` 仅用于选择比较哪个字段（pos/vel/torque/id-iq）。
- **量纲变化不介入**：`ctrl_type` 变化交给模式切换过渡处理，ref_smooth 检测到 `raw_ref->ctrl_type != prev_ref->ctrl_type` 直接 return false。
- **过渡时长两种模式**：
  - 速率模式：`rate > 0` 时按 `duration = ceil(|delta| / (rate × dt))` 自动算，dt 取自 `mgr->dt`（默认 100µs，运行期可改）。不同量纲独立 rate（pos_rate 单位 rad/s、vel_rate 单位 rad/s²、torque_rate 单位 Nm/s、current_rate 单位 A/s）。多字段同时突变时取最大 duration（保证最慢字段也平滑）。
  - 固定时长模式：`rate == 0` 回退到 `smooth_duration`，所有突变统一时长。
- **配置内聚**：`ref_smooth_cfg` 从全局 `g_ref_smooth_cfg` 迁入 `transition_mgr_t.smooth_cfg`，多实例可扩展。运行期通过 `mgr.smooth_cfg` 字段访问。
- **system_state 瘦身**：`motor_control_loop` 的 if/else 分支 + ref_smooth 逻辑全部下沉到 `transition_mgr_step()`，状态机只剩一行调用。

### 与现有过渡机制的关系

| 场景 | 触发点（重构后） | 引擎 | old_ref 来源 |
|------|--------|------|-------------|
| 模式切换（POSITION→VELOCITY） | `transition_mgr_on_mode_switch` | `transition_t` | 源模式当前 ref |
| READY→RUN 首次进 RUN | `transition_mgr_on_mode_switch` | `transition_t` | IDLE ref（保持当前位置） |
| 退出 RUN / 进 FAULT | `transition_mgr_on_top_fsm_change` | — | force_complete 立即结束 |
| **同模式目标突变（本方案）** | `transition_mgr_step` else 分支 | `transition_t` | 上一拍实际输出 ref |

所有过渡类型共用同一 `transition_mgr_t` 实例（内含 `transition_t`），互斥触发。

## File Structure

| 文件 | 职责 | 改动类型 |
|------|------|----------|
| `User/MotorControl/ControlProcess/ctrl_transition.h` | 新增 `ref_smooth_cfg_t` 结构、`transition_ref_smooth_check` 声明 | 修改（追加） |
| `User/MotorControl/ControlProcess/ctrl_transition.c` | 实现 `transition_ref_smooth_check`、`ref_smooth_cfg_init_defaults` | 修改（追加） |
| **`User/MotorControl/ControlProcess/ctrl_transition_mgr.h`**（新建） | `transition_mgr_t` 结构、`transition_mgr_*` API 声明 | **新建** |
| **`User/MotorControl/ControlProcess/ctrl_transition_mgr.c`**（新建） | 过渡策略封装：模式切换过渡 + 同模式渐变 + 应急终止；`transition_mgr_step` 统一入口 | **新建** |
| `User/AppServices/StateMachine/system_state.h` | 成员 `transition_t transition` → `transition_mgr_t trans_mgr` | 修改 |
| `User/AppServices/StateMachine/system_state.c` | 5 处 `transition_*` 调用替换为 3 处 `transition_mgr_*`；`motor_control_loop` 瘦身为单行 `transition_mgr_step` | 修改 |
| `tests/test_ref_smooth.c`（新建） | Host GCC 单元测试（引擎层 + mgr 层） | 新建 |
| `tests/host_stubs/`（新建，按需） | 隔离 HAL 依赖的空 stub 头文件 | 新建（仅当 host 编译缺依赖时） |

**不改动**：[ctrl_transition.c](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorControl/ControlProcess/ctrl_transition.c) 引擎核心逻辑（仅追加 ref_smooth 函数）；所有 `motor_mode_*.c` 模式 handler 保持 `ref->pos = ctrl->cmd.pos` 直接赋值，渐变在更上层统一处理（DRY）。

### ctrl_transition_mgr 接口设计

```c
/* ctrl_transition_mgr.h */
#ifndef CTRL_TRANSITION_MGR_H
#define CTRL_TRANSITION_MGR_H

#include "ctrl_transition.h"
#include "motor_control.h"

/* 前向声明打破与 system_state.h 的循环依赖（需先给 system_state_t 加 tag，见 Task 3） */
typedef struct system_state_s system_state_t;

typedef struct
{
    transition_t trans;           /* 过渡引擎实例（从 system_state_t 迁入） */
    ref_smooth_cfg_t smooth_cfg;  /* 同模式渐变配置（完全内聚，不再有全局 g_ref_smooth_cfg） */
    run_state_e target_run_state; /* 模式切换过渡的目标 run_state */
    float dt;                     /* 控制周期(s)，供 ref_smooth rate 模式算 duration，运行期可改 */
} transition_mgr_t;

/**
 * @brief 初始化过渡管理器
 * @param mgr 管理器指针
 * @param dt 控制周期(s)，默认 100µs(1e-4)。存入 mgr->dt，运行期可改
 */
void transition_mgr_init(transition_mgr_t *mgr, float dt);

/**
 * @brief 顶层状态变化通知（替代 top_fsm_switch 内的 transition_force_complete）
 * @details 退出 RUN 或进入 FAULT/SAFETY 时调用，立即结束任何进行中的过渡
 * @param mgr 管理器指针
 * @param new_state 即将进入的顶层状态
 */
void transition_mgr_on_top_fsm_change(transition_mgr_t *mgr, top_fsm_e new_state);

/**
 * @brief 模式切换请求（替代 run_state_switch 内的 transition_start）
 * @param mgr 管理器指针
 * @param new_state 目标运行子状态
 * @param trans_count 过渡时长(调用次数)
 * @param cur_ref 当前参考（作为过渡起点 old_ref）
 */
void transition_mgr_on_mode_switch(transition_mgr_t *mgr,
                                   run_state_e new_state,
                                   uint32_t trans_count,
                                   const motor_ref_t *cur_ref);

/**
 * @brief 每拍步进（替代 motor_control_loop 的 if/else 分支 + 同模式渐变逻辑）
 * @details 统一入口：模式切换过渡中则 blend；否则 dispatch + ref_smooth 检测 + 渐变。
 *          结果写入 sys->motor.ref。
 * @param mgr 管理器指针
 * @param sys 系统状态（读 run_state/dt，写 motor.ref）
 */
void transition_mgr_step(transition_mgr_t *mgr, system_state_t *sys);

#endif /* CTRL_TRANSITION_MGR_H */
```

### transition_mgr_step 内部逻辑（统一 if/else）

```c
void transition_mgr_step(transition_mgr_t *mgr, system_state_t *sys)
{
    if (mgr->trans.state == TRANSITION_IN_PROGRESS)
    {
        /* 模式切换过渡中：dispatch 目标模式生成 new_ref */
        run_state_e old = sys->motor.run_state;
        sys->motor.run_state = mgr->target_run_state;
        motor_ctrl_dispatch(&sys->motor);
        motor_ref_t new_ref = sys->motor.ref;
        sys->motor.run_state = old;

        /* blend */
        motor_ref_t mixed_ref;
        bool done = transition_update(&mgr->trans, &new_ref, &mixed_ref);
        sys->motor.ref = mixed_ref;

        if (done)
            sys->motor.run_state = mgr->target_run_state;
    }
    else
    {
        /* 无模式切换过渡：dispatch 当前 + 同模式渐变检测 */
        motor_ref_t prev_ref = sys->motor.ref;
        motor_ctrl_dispatch(&sys->motor);

        /* 判据用 run_state（白名单），dt 用 mgr->dt（运行期可配） */
        transition_ref_smooth_check(&mgr->trans, sys->motor.run_state,
                                    &sys->motor.ref, &prev_ref,
                                    &mgr->smooth_cfg, mgr->dt);

        if (mgr->trans.state == TRANSITION_IN_PROGRESS)
        {
            motor_ref_t mixed_ref;
            transition_update(&mgr->trans, &sys->motor.ref, &mixed_ref);
            sys->motor.ref = mixed_ref;
        }
    }
}
```

### system_state.c 调用点变化对照

| 位置 | 重构前 | 重构后 |
|------|--------|--------|
| `system_state_init` | `transition_init(&sys->transition)` | `transition_mgr_init(&sys->trans_mgr, dt)` |
| `top_fsm_switch` 退出 RUN | `transition_force_complete(&sys->transition)` | `transition_mgr_on_top_fsm_change(&sys->trans_mgr, new_state)` |
| `top_fsm_switch` 进 FAULT/SAFETY | `transition_force_complete(&sys->transition)` | `transition_mgr_on_top_fsm_change(&sys->trans_mgr, new_state)` |
| `run_state_switch` | `transition_start(&sys->transition, ...)` | `transition_mgr_on_mode_switch(&sys->trans_mgr, ...)` |
| `motor_control_loop` if/else 分支 | `transition_update` + dispatch | `transition_mgr_step(&sys->trans_mgr, sys)`（一行） |

`motor_control_loop` 瘦身后：

```c
void motor_control_loop(system_state_t *sys)
{
    fault_check(sys);

    if (sys->top_state == TOP_FSM_CALIB)
    {
        sys->calib_state = calib_mgr_poll();
        sys->motor.ref.ctrl_type = REF_CTRL_IDLE;
        return;
    }

    if (sys->top_state != TOP_FSM_RUN)
    {
        sys->motor.ref.ctrl_type = REF_CTRL_IDLE;
        return;
    }

    /* 所有过渡策略收敛到一行 */
    transition_mgr_step(&sys->trans_mgr, sys);
}
```

---

## Task 1: 新建 ctrl_transition_mgr.h —— 过渡管理器接口声明

**Files:**
- Create: `User/MotorControl/ControlProcess/ctrl_transition_mgr.h`

- [ ] **Step 1: 创建头文件**

```c
/* ctrl_transition_mgr.h */
#ifndef CTRL_TRANSITION_MGR_H
#define CTRL_TRANSITION_MGR_H

#include "ctrl_transition.h"
#include "motor_control.h"

/* 前向声明打破与 system_state.h 的循环依赖。
 * 前提：system_state.h 需把匿名 typedef 改为带 tag 的
 *      `typedef struct system_state_s { ... } system_state_t;`（见 Task 3 Step 1）。*/
typedef struct system_state_s system_state_t;

/**
 * @brief 过渡管理器：收拢所有过渡策略（模式切换过渡 + 同模式渐变 + 应急终止）
 * @details transition_t 实例与 ref_smooth_cfg_t 配置均内聚于此结构，
 *          system_state.c 不再直接操作 transition_*，改为调用 transition_mgr_*。
 *          不再保留全局 g_ref_smooth_cfg，配置完全由 mgr->smooth_cfg 持有。
 */
typedef struct
{
	transition_t trans;		   /* 过渡引擎实例（从 system_state_t 迁入） */
	ref_smooth_cfg_t smooth_cfg; /* 同模式渐变配置（完全内聚） */
	run_state_e target_run_state; /* 模式切换过渡的目标 run_state */
	float dt;					 /* 控制周期(s)，默认 100µs，供 rate 模式算 duration，运行期可改 */
} transition_mgr_t;

/**
 * @brief 初始化过渡管理器
 * @param mgr 管理器指针
 * @param dt 控制周期(s)，默认 100µs(1e-4)，存入 mgr->dt
 */
void transition_mgr_init(transition_mgr_t *mgr, float dt);

/**
 * @brief 顶层状态变化通知（替代 top_fsm_switch 内的 transition_force_complete）
 * @details 退出 RUN 或进入 FAULT/SAFETY 时调用，立即结束任何进行中的过渡
 * @param mgr 管理器指针
 * @param new_state 即将进入的顶层状态
 */
void transition_mgr_on_top_fsm_change(transition_mgr_t *mgr, top_fsm_e new_state);

/**
 * @brief 模式切换请求（替代 run_state_switch 内的 transition_start）
 * @param mgr 管理器指针
 * @param new_state 目标运行子状态
 * @param trans_count 过渡时长(调用次数)
 * @param cur_ref 当前参考（作为过渡起点 old_ref）
 */
void transition_mgr_on_mode_switch(transition_mgr_t *mgr,
								   run_state_e new_state,
								   uint32_t trans_count,
								   const motor_ref_t *cur_ref);

/**
 * @brief 每拍步进（替代 motor_control_loop 的 if/else 分支 + 同模式渐变逻辑）
 * @details 统一入口：模式切换过渡中则 blend；否则 dispatch + ref_smooth 检测 + 渐变。
 *          结果写入 sys->motor.ref。
 * @param mgr 管理器指针
 * @param sys 系统状态（读 run_state/dt，写 motor.ref）
 */
void transition_mgr_step(transition_mgr_t *mgr, system_state_t *sys);

#endif /* CTRL_TRANSITION_MGR_H */
```

- [ ] **Step 2: 提交**

```bash
git add User/MotorControl/ControlProcess/ctrl_transition_mgr.h
git commit -m "feat(transition): add ctrl_transition_mgr header"
```

---

## Task 2: 新建 ctrl_transition_mgr.c —— 实现 mgr API（仅模式切换过渡，无 ref_smooth）

**Files:**
- Create: `User/MotorControl/ControlProcess/ctrl_transition_mgr.c`

**说明**：本 Task 仅实现模式切换过渡逻辑（等价迁移现有行为），ref_smooth 在 Task 6 接入。这样可先做回归测试确认重构无回归，再加新功能。

- [ ] **Step 1: 创建实现文件（不含 ref_smooth）**

```c
/* ctrl_transition_mgr.c */
#include "ctrl_transition_mgr.h"
#include <string.h>

void transition_mgr_init(transition_mgr_t *mgr, float dt)
{
	memset(mgr, 0, sizeof(*mgr));
	transition_init(&mgr->trans);
	ref_smooth_cfg_init_defaults(&mgr->smooth_cfg);
	mgr->target_run_state = RUN_STATE_IDLE;
	mgr->dt = (dt > 0.0f) ? dt : 1.0e-4f; /* 默认 100µs，供 rate 模式算 duration */
}

void transition_mgr_on_top_fsm_change(transition_mgr_t *mgr, top_fsm_e new_state)
{
	/* 退出 RUN 或进入 FAULT/SAFETY：立即结束过渡 */
	(void)new_state;
	transition_force_complete(&mgr->trans);
}

void transition_mgr_on_mode_switch(transition_mgr_t *mgr,
								   run_state_e new_state,
								   uint32_t trans_count,
								   const motor_ref_t *cur_ref)
{
	if (new_state >= RUN_STATE_MAX)
		return;
	mgr->target_run_state = new_state;
	transition_start(&mgr->trans, trans_count, cur_ref);
}

void transition_mgr_step(transition_mgr_t *mgr, system_state_t *sys)
{
	if (mgr->trans.state == TRANSITION_IN_PROGRESS)
	{
		/* 模式切换过渡中：dispatch 目标模式生成 new_ref */
		run_state_e old = sys->motor.run_state;
		sys->motor.run_state = mgr->target_run_state;
		motor_ctrl_dispatch(&sys->motor);
		motor_ref_t new_ref = sys->motor.ref;
		sys->motor.run_state = old;

		/* blend */
		motor_ref_t mixed_ref;
		bool done = transition_update(&mgr->trans, &new_ref, &mixed_ref);
		sys->motor.ref = mixed_ref;

		if (done)
			sys->motor.run_state = mgr->target_run_state;
	}
	else
	{
		/* 无模式切换过渡：直接 dispatch（ref_smooth 在 Task 6 接入） */
		motor_ctrl_dispatch(&sys->motor);
	}
}
```

- [ ] **Step 2: 提交**

```bash
git add User/MotorControl/ControlProcess/ctrl_transition_mgr.c
git commit -m "feat(transition): implement transition_mgr (mode switch only)"
```

---

## Task 3: system_state 迁移到 transition_mgr（结构体 + 调用点替换）

**Files:**
- Modify: `User/AppServices/StateMachine/system_state.h`（成员替换）
- Modify: `User/AppServices/StateMachine/system_state.c`（5 处调用点替换）

**说明**：本 Task 完成迁移后行为与改前**完全等价**（仅模式切换过渡，无 ref_smooth），用于回归测试。ref_smooth 在 Task 6 接入。

- [ ] **Step 1: 修改 system_state.h（加 tag + 成员替换 + include）**

**1a. 给 `system_state_t` 加 struct tag**（前向声明的前提）。现有为匿名 typedef（[system_state.h:34](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/AppServices/StateMachine/system_state.h#L34)）：
```c
typedef struct
{
	top_fsm_e top_state;		  /*!< 顶层有限状态机状态 */
	...
} system_state_t;
```
改为带 tag（tag 名与 `ctrl_transition_mgr.h` 前向声明的 `system_state_s` 一致）：
```c
typedef struct system_state_s
{
	top_fsm_e top_state;		  /*!< 顶层有限状态机状态 */
	...
} system_state_t;
```

**1b. 成员替换**（[system_state.h:40](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/AppServices/StateMachine/system_state.h#L40)）：
```c
	transition_t transition;	  /*!< 状态过渡器 */
```
替换为：
```c
	transition_mgr_t trans_mgr; /*!< 过渡管理器（内含 transition_t + ref_smooth_cfg） */
```

**1c. include 调整**：在顶部 include 区，把 `#include "ctrl_transition.h"` 替换为：
```c
#include "ctrl_transition_mgr.h"
```
（`ctrl_transition_mgr.h` 已 include `ctrl_transition.h`，无需重复。注意 mgr 头文件对 `system_state_t` 只用前向声明，不 include `system_state.h`，循环依赖已打破。）

- [ ] **Step 2: 修改 system_state_init**

定位锚点（[system_state.c:156](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/AppServices/StateMachine/system_state.c#L156)）：
```c
	transition_init(&sys->transition);
```

替换为：
```c
	transition_mgr_init(&sys->trans_mgr, dt);
```

- [ ] **Step 3: 修改 top_fsm_switch 两处 force_complete**

定位锚点1（[system_state.c:184](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/AppServices/StateMachine/system_state.c#L184)，退出 RUN）：
```c
		transition_force_complete(&sys->transition);
```
替换为：
```c
		transition_mgr_on_top_fsm_change(&sys->trans_mgr, new_state);
```

定位锚点2（[system_state.c:217](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/AppServices/StateMachine/system_state.c#L217)，进 FAULT/SAFETY）：
```c
		transition_force_complete(&sys->transition);
```
替换为：
```c
		transition_mgr_on_top_fsm_change(&sys->trans_mgr, new_state);
```

- [ ] **Step 4: 修改 run_state_switch**

定位锚点（[system_state.c:242](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/AppServices/StateMachine/system_state.c#L242)）：
```c
	sys->target_run_state = new_state;
	transition_start(&sys->transition, trans_count, &sys->motor.ref);
```

替换为：
```c
	sys->target_run_state = new_state;
	transition_mgr_on_mode_switch(&sys->trans_mgr, new_state, trans_count, &sys->motor.ref);
```

- [ ] **Step 5: 修改 motor_control_loop（瘦身）**

定位锚点（[system_state.c:270-292](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/AppServices/StateMachine/system_state.c#L270)）：
```c
	// 运行态：处理模式切换的平滑过渡
	if (sys->transition.state == TRANSITION_IN_PROGRESS)
	{
		// 生成目标状态参考（过渡起点已记录在 transition 内）
		run_state_e old_state = sys->motor.run_state;
		sys->motor.run_state = sys->target_run_state;
		motor_ctrl_dispatch(&sys->motor);
		motor_ref_t new_ref = sys->motor.ref;
		sys->motor.run_state = old_state;

		// 参考层混合输出
		motor_ref_t mixed_ref;
		bool trans_done = transition_update(&sys->transition, &new_ref, &mixed_ref);
		sys->motor.ref = mixed_ref;

		// 过渡完成后正式切换运行状态
		if (trans_done)
			sys->motor.run_state = sys->target_run_state;
	}
	else
	{
		// 无过渡：直接生成当前状态参考
		motor_ctrl_dispatch(&sys->motor);
	}
```

替换为：
```c
	/* 运行态：所有过渡策略收敛到 transition_mgr_step */
	transition_mgr_step(&sys->trans_mgr, sys);
```

- [ ] **Step 6: 删除冗余 sys->target_run_state 字段**

现有代码 `sys->target_run_state` 共 3 处引用（已核对）：
- 定义 [system_state.h:41](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/AppServices/StateMachine/system_state.h#L41) `run_state_e target_run_state;`
- 写 [system_state.c:241](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/AppServices/StateMachine/system_state.c#L241) `sys->target_run_state = new_state;`
- 读 [system_state.c:274](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/AppServices/StateMachine/system_state.c#L274)、[:286](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/AppServices/StateMachine/system_state.c#L286)（均在 Step 5 已被 `transition_mgr_step` 替换的 if 分支内，删除后不再引用）

重构后目标 run_state 完全由 `mgr->target_run_state` 维护（`transition_mgr_on_mode_switch` 内设置）。**简化方案**：删除 `system_state_t` 的 `target_run_state` 成员，删除 `run_state_switch` 中的 `sys->target_run_state = new_state;` 行（该值已通过 `transition_mgr_on_mode_switch` 的参数传入 mgr）。删除后全工程 grep 确认无 `sys->target_run_state` 残留。

> 注意：Step 5 替换 `motor_control_loop` if/else 分支后，原读取 `sys->target_run_state` 的两处已随分支删除；此 Step 只需清理定义与 `run_state_switch` 的写入。

- [ ] **Step 7: 编译验证**

Keil 编译工程，预期 0 error 0 warning。

- [ ] **Step 8: 回归测试（虚拟电机模式）**

烧录/仿真，验证：
1. IDLE→ENABLE→READY→POSITION 0→RUN：模式切换过渡正常，电机平滑启动
2. RUN(POSITION)→VELOCITY：模式切换过渡正常
3. RUN→STOP→READY：退出 RUN，过渡 force_complete
4. 急停 ESTOP：进 SAFETY，过渡立即结束

**预期**：所有行为与改前完全一致（无 ref_smooth，目标突变仍阶跃）。

- [ ] **Step 9: 提交**

```bash
git add User/AppServices/StateMachine/system_state.h User/AppServices/StateMachine/system_state.c
git commit -m "refactor(state): migrate to transition_mgr (behavior-equivalent)"
```

---

## Task 4: 扩展 ctrl_transition.h —— 新增渐变配置与 API 声明

**Files:**
- Modify: `User/MotorControl/ControlProcess/ctrl_transition.h`（在文件末尾 `#endif` 前追加）

- [ ] **Step 1: 追加 ref_smooth 配置结构与 API 声明**

在 `ctrl_transition.h` 的 `void transition_force_complete(transition_t *trans);` 之后、`#endif` 之前追加：

```c
/**
 * @brief 同模式目标值渐变配置
 * @details 当同一运行模式内目标值突变超过阈值时，启动参考层线性渐变，
 *          避免目标阶跃导致 PID 大误差冲击。
 *
 * 配置语义：
 *  - enable: 总开关，false 时完全跳过 ref_smooth 逻辑
 *  - *_thresh: 触发阈值，突变绝对值超过此值才启动渐变；置 0 表示该字段不检测
 *  - 过渡时长二选一：
 *    * 速率模式（优先）：对应 rate 字段 > 0 时，duration = ceil(|delta| / (rate × dt))
 *    * 固定时长模式：所有 rate == 0 时，duration = smooth_duration
 *
 * rate 单位为"每秒最大变化量"：pos_rate=rad/s, vel_rate=rad/s²,
 * torque_rate=Nm/s, current_rate=A/s。
 */
typedef struct
{
	bool enable;			 /* 总开关，false 完全禁用同模式渐变 */
	uint32_t smooth_duration; /* 固定时长模式：渐变时长(transition_update 调用次数) */

	/* 触发阈值 */
	float pos_thresh;		 /* 位置目标突变阈值(rad) */
	float vel_thresh;		 /* 速度目标突变阈值(rad/s) */
	float torque_thresh;	 /* 力矩目标突变阈值(N·m) */
	float current_thresh;	 /* 电流目标突变阈值(A)，用于 id/iq */
	float voltage_thresh;	 /* 电压目标突变阈值(V)，用于 ud/voltage */
	float duty_thresh;		 /* 占空比突变阈值 */

	/* 速率模式：> 0 时按速率自动算 duration，覆盖 smooth_duration */
	float pos_rate;		 /* 位置变化速率上限(rad/s) */
	float vel_rate;		 /* 速度变化速率上限(rad/s²) */
	float torque_rate;	 /* 力矩变化速率上限(N·m/s) */
	float current_rate;	 /* 电流变化速率上限(A/s) */
} ref_smooth_cfg_t;

/* 配置完全内聚于 transition_mgr_t.smooth_cfg，不提供全局实例。
 * 运行期调参通过 sys->trans_mgr.smooth_cfg 字段访问（见 Task 8 / Task 9）。*/

/**
 * @brief 用默认值初始化渐变配置
 * @param cfg 配置指针
 * @note 默认: enable=true, smooth_duration=500,
 *             pos_thresh=0.1rad, vel_thresh=1rad/s, torque_thresh=0.1Nm,
 *             current_thresh=0.5A, voltage_thresh=1V, duty_thresh=0.1,
 *             pos_rate=50rad/s, vel_rate=500rad/s²,
 *             torque_rate=20Nm/s, current_rate=100A/s
 *       （rate 模式默认启用，覆盖 smooth_duration）
 */
void ref_smooth_cfg_init_defaults(ref_smooth_cfg_t *cfg);

/**
 * @brief 检测同模式内目标值突变，超阈值时启动参考层渐变
 * @param trans 过渡引擎
 * @param run_state 当前运行子状态（白名单判据，用于排除 MIT/HOLD/直控模式）
 * @param raw_ref 本拍 dispatch 输出的原始参考（目标）
 * @param prev_ref 上一拍实际输出的参考（渐变起点）
 * @param cfg 渐变配置
 * @param dt 控制周期(s)，用于 rate 模式算 duration
 * @return true 已启动渐变; false 未启动（差异在阈值内或不适用）
 * @note - cfg->enable=false 直接返回 false
 *       - trans->state == TRANSITION_IN_PROGRESS：过渡中不重启，返回 false
 *       - trans->state == TRANSITION_COMPLETED：复位为 IDLE 并返回 false（本拍不检测，
 *         避免模式切换过渡刚完成那拍被误判为同模式突变）
 *       - 判据用 run_state 白名单：仅 POSITION/POSITION_VELOCITY/POSITION_TORQUE/
 *         VELOCITY/TORQUE/CURRENT 启用；MIT/HOLD/OPEN_LOOP/DUTY/VOLTAGE 等返回 false。
 *         （MIT 与 TORQUE 的 ctrl_type 同为 REF_CTRL_TORQUE，必须用 run_state 区分）
 *       - ctrl_type 与 prev_ref 不一致时返回 false（交由模式切换过渡处理）
 *       - ctrl_type 仅用于选择比较字段（pos/vel/torque/id-iq）
 *       - rate 模式：duration = max(ceil(|delta_i| / (rate_i × dt)))，至少为 1
 *       - 固定时长模式（所有 rate==0）：duration = cfg->smooth_duration
 */
bool transition_ref_smooth_check(transition_t *trans,
								 run_state_e run_state,
								 const motor_ref_t *raw_ref,
								 const motor_ref_t *prev_ref,
								 const ref_smooth_cfg_t *cfg,
								 float dt);
```

- [ ] **Step 2: 提交**

```bash
git add User/MotorControl/ControlProcess/ctrl_transition.h
git commit -m "feat(transition): add ref_smooth config & API declaration"
```

---

## Task 5: 实现 transition_ref_smooth_check

**Files:**
- Modify: `User/MotorControl/ControlProcess/ctrl_transition.c`（在文件末尾追加）
- Test: `tests/test_ref_smooth.c`（Task 7 编写）

- [ ] **Step 1: 在 ctrl_transition.c 顶部 include 区确认无遗漏**

`ctrl_transition.c` 已 `#include "ctrl_transition.h"` 与 `<string.h>`，无需新增 include（`ref_smooth_cfg_t` 与 `motor_ref_t` 已通过 `ctrl_transition.h` → `motor_control.h` 可见）。

- [ ] **Step 2: 在文件末尾追加实现**

在 `transition_force_complete` 函数之后追加：

```c
/* ===== 同模式目标值渐变 ===== */

void ref_smooth_cfg_init_defaults(ref_smooth_cfg_t *cfg)
{
	cfg->enable = true;
	cfg->smooth_duration = 500;
	cfg->pos_thresh = 0.1f;
	cfg->vel_thresh = 1.0f;
	cfg->torque_thresh = 0.1f;
	cfg->current_thresh = 0.5f;
	cfg->voltage_thresh = 1.0f;
	cfg->duty_thresh = 0.1f;
	/* 速率模式默认启用（>0 即生效），覆盖 smooth_duration */
	cfg->pos_rate = 50.0f;		/* 50 rad/s：5 rad 突变 → 0.1s 过渡 */
	cfg->vel_rate = 500.0f;		/* 500 rad/s²：50 rad/s 突变 → 0.1s 过渡 */
	cfg->torque_rate = 20.0f;	/* 20 Nm/s */
	cfg->current_rate = 100.0f; /* 100 A/s */
}

static float ref_smooth_absf(float v)
{
	return (v < 0.0f) ? -v : v;
}

/**
 * @brief 按 rate 模式计算过渡时长(调用次数)
 * @details duration_i = ceil(|delta_i| / (rate_i × dt))，取各字段最大值。
 *          rate_i <= 0 表示该字段不参与速率计算（交由 smooth_duration 兜底）。
 *          所有 rate 都 <= 0 时返回 0，由调用方回退到 smooth_duration。
 */
static uint32_t ref_smooth_calc_duration_by_rate(const motor_ref_t *raw,
												 const motor_ref_t *prev,
												 const ref_smooth_cfg_t *cfg,
												 float dt)
{
	if (dt <= 0.0f)
		return 0;

	float max_needed = 0.0f;

	if (cfg->pos_rate > 0.0f)
	{
		float d = ref_smooth_absf(raw->pos - prev->pos) / (cfg->pos_rate * dt);
		if (d > max_needed)
			max_needed = d;
	}
	if (cfg->vel_rate > 0.0f)
	{
		float d = ref_smooth_absf(raw->vel - prev->vel) / (cfg->vel_rate * dt);
		if (d > max_needed)
			max_needed = d;
	}
	if (cfg->torque_rate > 0.0f)
	{
		float d = ref_smooth_absf(raw->torque - prev->torque) / (cfg->torque_rate * dt);
		if (d > max_needed)
			max_needed = d;
	}
	if (cfg->current_rate > 0.0f)
	{
		float d_id = ref_smooth_absf(raw->id - prev->id) / (cfg->current_rate * dt);
		float d_iq = ref_smooth_absf(raw->iq - prev->iq) / (cfg->current_rate * dt);
		float d = (d_id > d_iq) ? d_id : d_iq;
		if (d > max_needed)
			max_needed = d;
	}

	if (max_needed <= 0.0f)
		return 0;

	/* ceil，至少 1 */
	uint32_t dur = (uint32_t)(max_needed + 0.999f);
	return (dur < 1) ? 1 : dur;
}

/**
 * @brief run_state 白名单：仅纯参考型闭环模式启用同模式渐变
 * @details 排除 MIT（反馈型，ctrl_type 与 TORQUE 相同无法用 ctrl_type 区分）、
 *          HOLD（目标跟随 fb.pos）、OPEN_LOOP/DUTY/VOLTAGE（直控）。
 */
static bool ref_smooth_run_state_enabled(run_state_e s)
{
	switch (s)
	{
		case RUN_STATE_POSITION:
		case RUN_STATE_POSITION_VELOCITY:
		case RUN_STATE_POSITION_TORQUE:
		case RUN_STATE_VELOCITY:
		case RUN_STATE_TORQUE:
		case RUN_STATE_CURRENT:
			return true;
		default:
			return false;
	}
}

bool transition_ref_smooth_check(transition_t *trans,
								 run_state_e run_state,
								 const motor_ref_t *raw_ref,
								 const motor_ref_t *prev_ref,
								 const ref_smooth_cfg_t *cfg,
								 float dt)
{
	/* 总开关关闭：完全跳过 */
	if (!cfg->enable)
		return false;

	/* 过渡进行中：不重启，让现有渐变朝最新 raw_ref 继续混合 */
	if (trans->state == TRANSITION_IN_PROGRESS)
		return false;

	/* 过渡刚完成：复位为 IDLE，本拍不检测。
	 * 关键：模式切换过渡与同模式渐变共用同一 trans，若不在此复位，
	 * 模式切换完成那拍（state=COMPLETED）会继续往下走检测，可能误触发渐变。 */
	if (trans->state == TRANSITION_COMPLETED)
	{
		trans->state = TRANSITION_IDLE;
		return false;
	}

	/* run_state 白名单：排除 MIT/HOLD/直控模式 */
	if (!ref_smooth_run_state_enabled(run_state))
		return false;

	/* 量纲变化交给模式切换过渡处理 */
	if (raw_ref->ctrl_type != prev_ref->ctrl_type)
		return false;

	bool exceed = false;
	switch (raw_ref->ctrl_type)
	{
		case REF_CTRL_POSITION:
			if (ref_smooth_absf(raw_ref->pos - prev_ref->pos) > cfg->pos_thresh)
				exceed = true;
			break;
		case REF_CTRL_VELOCITY:
			if (ref_smooth_absf(raw_ref->vel - prev_ref->vel) > cfg->vel_thresh)
				exceed = true;
			break;
		case REF_CTRL_TORQUE:
			if (ref_smooth_absf(raw_ref->torque - prev_ref->torque) > cfg->torque_thresh)
				exceed = true;
			break;
		case REF_CTRL_CURRENT:
			if (ref_smooth_absf(raw_ref->id - prev_ref->id) > cfg->current_thresh ||
				ref_smooth_absf(raw_ref->iq - prev_ref->iq) > cfg->current_thresh)
				exceed = true;
			break;
		default:
			break;
	}

	if (!exceed)
		return false;

	/* 计算过渡时长：优先速率模式，回退固定时长 */
	uint32_t duration = ref_smooth_calc_duration_by_rate(raw_ref, prev_ref, cfg, dt);
	if (duration == 0)
		duration = cfg->smooth_duration;
	if (duration == 0)
		return false; /* 无效配置，不启动 */

	/* 以上一拍实际输出为起点启动渐变 */
	transition_start(trans, duration, prev_ref);
	return true;
}
```

- [ ] **Step 3: 提交**

```bash
git add User/MotorControl/ControlProcess/ctrl_transition.c
git commit -m "feat(transition): implement transition_ref_smooth_check"
```

---

## Task 6: ctrl_transition_mgr 接入 ref_smooth（同模式渐变生效）

**Files:**
- Modify: `User/MotorControl/ControlProcess/ctrl_transition_mgr.c`（仅改 `transition_mgr_step` else 分支）

**说明**：Task 1-3 完成重构（行为等价），Task 4-5 完成 ref_smooth 引擎层实现。本 Task 把 ref_smooth 接入 mgr 的 else 分支，同模式渐变正式生效。system_state.c 无需改动（已瘦身为一行 `transition_mgr_step`）。

- [ ] **Step 1: 修改 transition_mgr_step 的 else 分支**

定位锚点（Task 2 创建的代码）：
```c
	else
	{
		/* 无模式切换过渡：直接 dispatch（ref_smooth 在 Task 6 接入） */
		motor_ctrl_dispatch(&sys->motor);
	}
```

替换为：
```c
	else
	{
		/* 无模式切换过渡：dispatch + 同模式渐变检测 */
		motor_ref_t prev_ref = sys->motor.ref;
		motor_ctrl_dispatch(&sys->motor);

		/* 同模式目标值渐变：突变超阈值时启动参考层 blend。
		 * 判据用 sys->motor.run_state（白名单排除 MIT/HOLD/直控）。
		 * dt 取自 mgr->dt（运行期可配），供 rate 模式算 duration。 */
		transition_ref_smooth_check(&mgr->trans, sys->motor.run_state,
									&sys->motor.ref, &prev_ref,
									&mgr->smooth_cfg, mgr->dt);

		/* 渐变进行中：对参考做线性混合，消除目标阶跃 */
		if (mgr->trans.state == TRANSITION_IN_PROGRESS)
		{
			motor_ref_t mixed_ref;
			transition_update(&mgr->trans, &sys->motor.ref, &mixed_ref);
			sys->motor.ref = mixed_ref;
		}
	}
```

- [ ] **Step 2: 编译验证**

Keil 编译工程，预期 0 error 0 warning。

- [ ] **Step 3: 提交**

```bash
git add User/MotorControl/ControlProcess/ctrl_transition_mgr.c
git commit -m "feat(transition_mgr): integrate intra-mode ref smooth"
```

---

## Task 7: Host 单元测试

**Files:**
- Create: `tests/test_ref_smooth.c`
- Create（按需）: `tests/host_stubs/` 下的 stub 头文件

### 关于 HAL 依赖隔离

`ctrl_transition.c` → `ctrl_transition.h` → `motor_control.h` → `motor_param.h` / `pid_profile.h` / `state_define.h` / `utils.h`，传递依赖可能引入 STM32 HAL 头。Host 编译前先试一次；若报缺头文件，在 `tests/host_stubs/` 下创建同名空 stub（仅保留类型定义所需的最小内容），用 `-Itests/host_stubs` 优先覆盖。

- [ ] **Step 1: 首次 host 编译探测依赖**

```bash
gcc -c User/MotorControl/ControlProcess/ctrl_transition.c -IUser/MotorControl/ControlProcess -IUser/DataHub -IUser/AppServices/StateMachine -IUser/MotorControl/CascadeControl -o /tmp/ctrl_transition.o 2>&1 | head -40
```

观察缺哪些头，按需在 `tests/host_stubs/` 建 stub。

> **签名变更提醒**：`transition_ref_smooth_check` 现多一个 `run_state_e run_state` 参数（第 2 位）。下方所有用例的调用都需按 `check(&t, <run_state>, &raw, &prev, &cfg, TEST_DT)` 传入与该用例 `ctrl_type` 匹配的 run_state：
> - POSITION 系用例 → `RUN_STATE_POSITION`
> - VELOCITY 用例 → `RUN_STATE_VELOCITY`
> - TORQUE 用例 → `RUN_STATE_TORQUE`
> - CURRENT 用例 → `RUN_STATE_CURRENT`
> - DUTY 用例 → `RUN_STATE_DUTY_CYCLE`
> 并**新增两个用例**（用例14 MIT 排除、用例15 COMPLETED 复位），直接守护本次两处致命修正。

- [ ] **Step 2: 编写测试文件 tests/test_ref_smooth.c**

```c
#include "ctrl_transition.h"
#include <stdio.h>
#include <math.h>

/* 测试用控制周期：100µs（与固件电流环一致，current_freq_hz=10kHz） */
#define TEST_DT 0.0001f

static int tests_run = 0;
static int tests_failed = 0;

static void check(bool cond, const char *msg)
{
	tests_run++;
	if (!cond)
	{
		printf("FAIL: %s\n", msg);
		tests_failed++;
	}
	else
	{
		printf("ok  : %s\n", msg);
	}
}

/* 关闭 rate 模式，回到固定时长语义（供既有用例复用） */
static void cfg_disable_rate(ref_smooth_cfg_t *cfg)
{
	cfg->pos_rate = 0.0f;
	cfg->vel_rate = 0.0f;
	cfg->torque_rate = 0.0f;
	cfg->current_rate = 0.0f;
}

/* 用例1: 目标无变化不启动渐变 */
static void test_no_change_no_start(void)
{
	transition_t t;
	transition_init(&t);
	ref_smooth_cfg_t cfg;
	ref_smooth_cfg_init_defaults(&cfg);
	cfg_disable_rate(&cfg);

	motor_ref_t raw = {0}, prev = {0};
	raw.ctrl_type = REF_CTRL_POSITION;
	prev.ctrl_type = REF_CTRL_POSITION;
	raw.pos = 1.0f;
	prev.pos = 1.0f;

	bool started = transition_ref_smooth_check(&t, RUN_STATE_POSITION, &raw, &prev, &cfg, TEST_DT);
	check(!started, "no change: should not start");
	check(t.state == TRANSITION_IDLE, "no change: state idle");
}

/* 用例2: 位置目标突变超阈值启动渐变，old_ref=prev，duration=smooth_duration */
static void test_pos_exceed_starts(void)
{
	transition_t t;
	transition_init(&t);
	ref_smooth_cfg_t cfg;
	ref_smooth_cfg_init_defaults(&cfg);
	cfg_disable_rate(&cfg);

	motor_ref_t raw = {0}, prev = {0};
	raw.ctrl_type = REF_CTRL_POSITION;
	prev.ctrl_type = REF_CTRL_POSITION;
	prev.pos = 0.0f;
	raw.pos = 5.0f; /* > 0.1 阈值 */

	bool started = transition_ref_smooth_check(&t, RUN_STATE_POSITION, &raw, &prev, &cfg, TEST_DT);
	check(started, "pos exceed: should start");
	check(t.state == TRANSITION_IN_PROGRESS, "pos exceed: in progress");
	check(t.old_ref.pos == 0.0f, "pos exceed: old_ref=prev (0.0)");
	check(t.duration == cfg.smooth_duration, "pos exceed: duration=smooth_duration (rate disabled)");
}

/* 用例3: 渐变进行中不重启，old_ref 保持 */
static void test_in_progress_no_restart(void)
{
	transition_t t;
	transition_init(&t);
	ref_smooth_cfg_t cfg;
	ref_smooth_cfg_init_defaults(&cfg);
	cfg_disable_rate(&cfg);

	motor_ref_t raw = {0}, prev = {0};
	raw.ctrl_type = REF_CTRL_POSITION;
	prev.ctrl_type = REF_CTRL_POSITION;
	prev.pos = 0.0f;
	raw.pos = 5.0f;
	transition_ref_smooth_check(&t, RUN_STATE_POSITION, &raw, &prev, &cfg, TEST_DT);

	/* 再次调用：raw 又突变到 10，但应不重启 */
	motor_ref_t prev2 = prev;
	raw.pos = 10.0f;
	bool started = transition_ref_smooth_check(&t, RUN_STATE_POSITION, &raw, &prev2, &cfg, TEST_DT);
	check(!started, "in progress: no restart");
	check(t.old_ref.pos == 0.0f, "in progress: old_ref unchanged");
}

/* 用例4: 变化小于阈值不启动 */
static void test_below_threshold_no_start(void)
{
	transition_t t;
	transition_init(&t);
	ref_smooth_cfg_t cfg;
	ref_smooth_cfg_init_defaults(&cfg);
	cfg_disable_rate(&cfg);

	motor_ref_t raw = {0}, prev = {0};
	raw.ctrl_type = REF_CTRL_VELOCITY;
	prev.ctrl_type = REF_CTRL_VELOCITY;
	prev.vel = 0.0f;
	raw.vel = 0.5f; /* < 1.0 阈值 */

	bool started = transition_ref_smooth_check(&t, RUN_STATE_VELOCITY, &raw, &prev, &cfg, TEST_DT);
	check(!started, "below thresh: no start");
}

/* 用例5: DUTY 模式不启用渐变（run_state 白名单排除） */
static void test_duty_disabled(void)
{
	transition_t t;
	transition_init(&t);
	ref_smooth_cfg_t cfg;
	ref_smooth_cfg_init_defaults(&cfg);

	motor_ref_t raw = {0}, prev = {0};
	raw.ctrl_type = REF_CTRL_DUTY;
	prev.ctrl_type = REF_CTRL_DUTY;
	prev.duty = 0.0f;
	raw.duty = 0.9f;

	bool started = transition_ref_smooth_check(&t, RUN_STATE_DUTY_CYCLE, &raw, &prev, &cfg, TEST_DT);
	check(!started, "duty mode: disabled");
}

/* 用例14（新增·关键）: MIT 模式不渐变——与 TORQUE 同 ctrl_type，靠 run_state 区分 */
static void test_mit_disabled(void)
{
	transition_t t;
	transition_init(&t);
	ref_smooth_cfg_t cfg;
	ref_smooth_cfg_init_defaults(&cfg);

	/* MIT handler 输出 ctrl_type=REF_CTRL_TORQUE，与纯 TORQUE 模式相同 */
	motor_ref_t raw = {0}, prev = {0};
	raw.ctrl_type = REF_CTRL_TORQUE;
	prev.ctrl_type = REF_CTRL_TORQUE;
	prev.torque = 0.0f;
	raw.torque = 5.0f; /* 巨大突变 */

	/* run_state=MIT → 白名单排除，即便 ctrl_type=TORQUE 也不启动 */
	bool started = transition_ref_smooth_check(&t, RUN_STATE_MIT, &raw, &prev, &cfg, TEST_DT);
	check(!started, "MIT mode: disabled even with TORQUE ctrl_type");
	check(t.state == TRANSITION_IDLE, "MIT mode: state idle");

	/* 反向对照：同样的 ref，run_state=TORQUE 应启动 */
	transition_init(&t);
	bool started2 = transition_ref_smooth_check(&t, RUN_STATE_TORQUE, &raw, &prev, &cfg, TEST_DT);
	check(started2, "TORQUE mode: starts (contrast to MIT)");
}

/* 用例15（新增·关键）: COMPLETED 态调用 check 应复位为 IDLE 且不启动 */
static void test_completed_resets_to_idle(void)
{
	transition_t t;
	transition_init(&t);
	/* 模拟一次模式切换过渡完成：手动置 COMPLETED */
	t.state = TRANSITION_COMPLETED;

	ref_smooth_cfg_t cfg;
	ref_smooth_cfg_init_defaults(&cfg);
	cfg_disable_rate(&cfg);

	motor_ref_t raw = {0}, prev = {0};
	raw.ctrl_type = REF_CTRL_POSITION;
	prev.ctrl_type = REF_CTRL_POSITION;
	prev.pos = 0.0f;
	raw.pos = 5.0f; /* 即便有突变，本拍也不应启动（先复位 IDLE） */

	bool started = transition_ref_smooth_check(&t, RUN_STATE_POSITION, &raw, &prev, &cfg, TEST_DT);
	check(!started, "completed: no start on reset tick");
	check(t.state == TRANSITION_IDLE, "completed: reset to idle");

	/* 下一拍同样突变应正常启动 */
	bool started2 = transition_ref_smooth_check(&t, RUN_STATE_POSITION, &raw, &prev, &cfg, TEST_DT);
	check(started2, "completed: starts on next tick after reset");
}

/* 用例6: ctrl_type 变化不介入（交模式切换过渡） */
static void test_ctrl_type_change_skipped(void)
{
	transition_t t;
	transition_init(&t);
	ref_smooth_cfg_t cfg;
	ref_smooth_cfg_init_defaults(&cfg);

	motor_ref_t raw = {0}, prev = {0};
	prev.ctrl_type = REF_CTRL_POSITION;
	prev.pos = 0.0f;
	raw.ctrl_type = REF_CTRL_VELOCITY;
	raw.vel = 10.0f;

	/* run_state=POSITION（白名单内），但 raw/prev 的 ctrl_type 不一致 → 交模式切换过渡 */
	bool started = transition_ref_smooth_check(&t, RUN_STATE_POSITION, &raw, &prev, &cfg, TEST_DT);
	check(!started, "ctrl_type change: skipped");
}

/* 用例7: 电流环 id/iq 任一超阈值启动 */
static void test_current_id_iq(void)
{
	transition_t t;
	transition_init(&t);
	ref_smooth_cfg_t cfg;
	ref_smooth_cfg_init_defaults(&cfg);

	motor_ref_t raw = {0}, prev = {0};
	raw.ctrl_type = REF_CTRL_CURRENT;
	prev.ctrl_type = REF_CTRL_CURRENT;
	prev.id = 0.0f;
	prev.iq = 0.0f;
	raw.id = 0.1f;  /* < 0.5 */
	raw.iq = 2.0f;  /* > 0.5 */

	bool started = transition_ref_smooth_check(&t, RUN_STATE_CURRENT, &raw, &prev, &cfg, TEST_DT);
	check(started, "current iq exceed: start");
}

/* 用例8: blend 进展正确，首拍 ratio=1/N，末拍到目标 */
static void test_blend_progression(void)
{
	transition_t t;
	transition_init(&t);
	ref_smooth_cfg_t cfg;
	ref_smooth_cfg_init_defaults(&cfg);
	cfg_disable_rate(&cfg);
	cfg.smooth_duration = 10;

	motor_ref_t raw = {0}, prev = {0};
	raw.ctrl_type = REF_CTRL_POSITION;
	prev.ctrl_type = REF_CTRL_POSITION;
	prev.pos = 0.0f;
	raw.pos = 10.0f;
	transition_ref_smooth_check(&t, RUN_STATE_POSITION, &raw, &prev, &cfg, TEST_DT);

	motor_ref_t out;
	/* 第一拍: elapsed=1, ratio=0.1, out.pos = 0*0.9 + 10*0.1 = 1.0 */
	transition_update(&t, &raw, &out);
	check(fabsf(out.pos - 1.0f) < 1e-6f, "blend tick1: pos=1.0");

	/* 推进剩余 9 拍 */
	for (int i = 0; i < 9; i++)
		transition_update(&t, &raw, &out);
	check(t.state == TRANSITION_COMPLETED, "blend: completed");
	check(fabsf(out.pos - 10.0f) < 1e-6f, "blend final: pos=10.0");
}

/* 用例9: 渐变期间 raw 变化，blend 朝最新 raw 收敛 */
static void test_blend_tracks_new_raw(void)
{
	transition_t t;
	transition_init(&t);
	ref_smooth_cfg_t cfg;
	ref_smooth_cfg_init_defaults(&cfg);
	cfg_disable_rate(&cfg);
	cfg.smooth_duration = 10;

	motor_ref_t raw = {0}, prev = {0};
	raw.ctrl_type = REF_CTRL_POSITION;
	prev.ctrl_type = REF_CTRL_POSITION;
	prev.pos = 0.0f;
	raw.pos = 10.0f;
	transition_ref_smooth_check(&t, RUN_STATE_POSITION, &raw, &prev, &cfg, TEST_DT);

	motor_ref_t out;
	transition_update(&t, &raw, &out); /* tick1: ratio=0.1, out=1.0 */

	/* 第2拍起 raw 变为 20（渐变中不重启，但 new_ref 用最新 raw） */
	raw.pos = 20.0f;
	/* 推进 9 拍完成 */
	for (int i = 0; i < 9; i++)
		transition_update(&t, &raw, &out);
	check(t.state == TRANSITION_COMPLETED, "track: completed");
	check(fabsf(out.pos - 20.0f) < 1e-6f, "track: converge to new raw 20.0");
}

/* 用例10: enable=false 完全跳过（即使突变很大） */
static void test_enable_false_skip(void)
{
	transition_t t;
	transition_init(&t);
	ref_smooth_cfg_t cfg;
	ref_smooth_cfg_init_defaults(&cfg);
	cfg.enable = false;

	motor_ref_t raw = {0}, prev = {0};
	raw.ctrl_type = REF_CTRL_POSITION;
	prev.ctrl_type = REF_CTRL_POSITION;
	prev.pos = 0.0f;
	raw.pos = 100.0f; /* 巨大突变 */

	bool started = transition_ref_smooth_check(&t, RUN_STATE_POSITION, &raw, &prev, &cfg, TEST_DT);
	check(!started, "enable=false: skip even huge jump");
	check(t.state == TRANSITION_IDLE, "enable=false: state idle");
}

/* 用例11: rate 模式算 duration = ceil(|delta| / (rate × dt)) */
static void test_rate_mode_duration(void)
{
	transition_t t;
	transition_init(&t);
	ref_smooth_cfg_t cfg;
	ref_smooth_cfg_init_defaults(&cfg);
	/* 默认 pos_rate=50 rad/s, dt=0.0001s → rate×dt=0.005 rad/tick
	 * delta=5 rad → duration = 5/0.005 = 1000 */

	motor_ref_t raw = {0}, prev = {0};
	raw.ctrl_type = REF_CTRL_POSITION;
	prev.ctrl_type = REF_CTRL_POSITION;
	prev.pos = 0.0f;
	raw.pos = 5.0f;

	bool started = transition_ref_smooth_check(&t, RUN_STATE_POSITION, &raw, &prev, &cfg, TEST_DT);
	check(started, "rate mode: start");
	check(t.duration == 1000, "rate mode: duration=1000 (5/(50*0.0001))");

	/* 验证首拍 blend：ratio=1/1000=0.001, out=0.005 */
	motor_ref_t out;
	transition_update(&t, &raw, &out);
	check(fabsf(out.pos - 0.005f) < 1e-6f, "rate mode: tick1 pos=0.005");
}

/* 用例12: rate=0 回退固定时长模式 */
static void test_rate_zero_fallback_duration(void)
{
	transition_t t;
	transition_init(&t);
	ref_smooth_cfg_t cfg;
	ref_smooth_cfg_init_defaults(&cfg);
	cfg_disable_rate(&cfg);
	cfg.smooth_duration = 200;

	motor_ref_t raw = {0}, prev = {0};
	raw.ctrl_type = REF_CTRL_POSITION;
	prev.ctrl_type = REF_CTRL_POSITION;
	prev.pos = 0.0f;
	raw.pos = 5.0f;

	bool started = transition_ref_smooth_check(&t, RUN_STATE_POSITION, &raw, &prev, &cfg, TEST_DT);
	check(started, "rate zero: start");
	check(t.duration == 200, "rate zero: fallback to smooth_duration=200");
}

/* 用例13: rate 模式下小突变 duration 短，大突变 duration 长 */
static void test_rate_mode_scales_with_delta(void)
{
	/* 小突变 1 rad → duration = 1/0.005 = 200 */
	{
		transition_t t;
		transition_init(&t);
		ref_smooth_cfg_t cfg;
		ref_smooth_cfg_init_defaults(&cfg);
		motor_ref_t raw = {0}, prev = {0};
		raw.ctrl_type = REF_CTRL_POSITION;
		prev.ctrl_type = REF_CTRL_POSITION;
		raw.pos = 1.0f;
		bool started = transition_ref_smooth_check(&t, RUN_STATE_POSITION, &raw, &prev, &cfg, TEST_DT);
		check(started, "rate small: start");
		check(t.duration == 200, "rate small: duration=200");
	}
	/* 大突变 10 rad → duration = 10/0.005 = 2000 */
	{
		transition_t t;
		transition_init(&t);
		ref_smooth_cfg_t cfg;
		ref_smooth_cfg_init_defaults(&cfg);
		motor_ref_t raw = {0}, prev = {0};
		raw.ctrl_type = REF_CTRL_POSITION;
		prev.ctrl_type = REF_CTRL_POSITION;
		raw.pos = 10.0f;
		bool started = transition_ref_smooth_check(&t, RUN_STATE_POSITION, &raw, &prev, &cfg, TEST_DT);
		check(started, "rate large: start");
		check(t.duration == 2000, "rate large: duration=2000");
	}
}

int main(void)
{
	test_no_change_no_start();
	test_pos_exceed_starts();
	test_in_progress_no_restart();
	test_below_threshold_no_start();
	test_duty_disabled();
	test_ctrl_type_change_skipped();
	test_current_id_iq();
	test_blend_progression();
	test_blend_tracks_new_raw();
	test_enable_false_skip();
	test_rate_mode_duration();
	test_rate_zero_fallback_duration();
	test_rate_mode_scales_with_delta();
	test_mit_disabled();            /* 用例14：MIT 排除 */
	test_completed_resets_to_idle(); /* 用例15：COMPLETED 复位 */

	printf("\n==== %d/%d passed ====\n", tests_run - tests_failed, tests_run);
	return tests_failed ? 1 : 0;
}
```

- [ ] **Step 3: 编译并运行测试（预期 FAIL：因为还未接入 system_state，但 ctrl_transition 实现已完成，本步应 PASS）**

```bash
gcc tests/test_ref_smooth.c User/MotorControl/ControlProcess/ctrl_transition.c \
    -IUser/MotorControl/ControlProcess -IUser/DataHub \
    -IUser/AppServices/StateMachine -IUser/MotorControl/CascadeControl \
    -lm -o tests/test_ref_smooth 2>&1 | head -40
```

若有 HAL 头缺失，在 `tests/host_stubs/` 建空 stub 并追加 `-Itests/host_stubs`，重新编译。

运行：
```bash
./tests/test_ref_smooth
```

Expected: 全部用例 PASS（`==== N/N passed ====`，N 为实际 check 断言总数），退出码 0。含新增用例14（MIT 排除）、用例15（COMPLETED 复位）。

- [ ] **Step 4: 提交**

```bash
git add tests/test_ref_smooth.c tests/host_stubs/
git commit -m "test(transition): add host unit tests for ref_smooth"
```

---

## Task 8: 虚拟电机模式集成验证

**Files:** 无代码改动，仅运行验证

本任务在虚拟电机模式（`MOTOR_LOOP_ENABLE_DEV_DRIVER=0`，[dev_motor_virtual.c](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorControl/CascadeControl/dev_motor_virtual.c)）下验证端到端行为，确认同模式目标突变不再阶跃。

- [ ] **Step 1: 确认虚拟电机模式编译开关**

检查 `motor_loop.h` 或 select 头中 `MOTOR_LOOP_ENABLE_DEV_DRIVER` 当前值。若为 1（真硬件），临时切到 0 做虚拟验证（验证完再切回）。

- [ ] **Step 2: 编译并烧录/仿真**

Keil 编译工程，烧录或启动仿真。

- [ ] **Step 3: 场景A —— 位置模式目标突变（rate 模式，验证渐变生效）**

通过上位机/串口发送：
1. `ENABLE` → 进入 READY
2. `POSITION` 目标 0 rad → 进入 RUN，电机停于 0
3. 等待 1 秒（确保模式切换过渡完成）
4. 发送 `POSITION` 目标 5 rad（同模式目标突变）

**预期（rate 模式默认启用）**：`pos_rate=50 rad/s`，dt=电流环周期(100µs)，duration = 5 / (50 × 100e-6) = 1000 拍 = 100ms。电机在约 100ms 内从 0 线性过渡到 5 rad，无抽动。上位机遥测 0xC6 `position_rad` 应呈线性变化，斜率约 50 rad/s。

**对比**（改前）：同一操作电机会瞬间抽动（目标 5 rad 误差直接进位置环）。

- [ ] **Step 4: 场景B —— 小幅目标变化不触发渐变（验证阈值）**

1. 位置模式目标 0 rad
2. 发送 `POSITION` 目标 0.05 rad（< `pos_thresh=0.1`）

**预期**：不启动渐变，目标立即生效（0.05 rad 阶跃对机械冲击可忽略）。

- [ ] **Step 5: 场景C —— 连续突变（验证不重启 + 收敛到最新目标）**

1. 位置模式目标 0 rad
2. 发送目标 5 rad（启动渐变，duration≈1000 拍）
3. 渐变未完成时（<100ms 内）发送目标 8 rad

**预期**：渐变不重启，朝最新目标 8 rad 收敛，参考连续无跳变。

- [ ] **Step 6: 场景D —— 速度模式验证**

1. `VELOCITY` 目标 0 rad/s
2. 发送 `VELOCITY` 目标 50 rad/s（> `vel_thresh=1`）

**预期（rate 模式）**：`vel_rate=500 rad/s²`，duration = 50 / (500 × 100e-6) = 1000 拍 = 100ms。速度参考在 100ms 内从 0 线性渐变到 50 rad/s，加速度约 500 rad/s²。

- [ ] **Step 7: 场景E —— MIT / 占空比模式不渐变（验证白名单排除）**

**E1（关键·MIT）**：
1. `MIT` 模式，`kp/kd/torque_ff` 给定，目标 pos 0 rad
2. 发送 MIT 目标 pos 突变到 5 rad

**预期**：不启动渐变。MIT 输出 `ctrl_type=REF_CTRL_TORQUE`，与纯 TORQUE 相同，但 `run_state=RUN_STATE_MIT` 被白名单排除。电机按 MIT 柔顺特性响应（kp/kd 决定），**不叠加参考层 blend**。这是本次修正的核心验证点——若渐变错误启动，说明白名单判据未生效。

**E2（DUTY）**：
1. `DUTY` 模式占空比 0
2. 发送 `DUTY` 占空比 0.8

**预期**：不启动渐变，占空比立即生效（DUTY 为直控模式，`run_state=RUN_STATE_DUTY_CYCLE` 被排除）。

- [ ] **Step 8: 场景F —— enable=false 完全禁用（验证总开关）**

通过调试器把 `sys.trans_mgr.smooth_cfg.enable` 改为 `false`，然后：
1. 位置模式目标 0 rad
2. 发送 `POSITION` 目标 5 rad

**预期**：渐变不启动，目标立即生效（5 rad 阶跃直达 PID），电机抽动——与改前行为一致。验证 enable 开关可应急回退。

验证后把 `enable` 改回 `true`。

- [ ] **Step 9: 场景G —— rate 模式 duration 随突变大小缩放**

1. 位置模式目标 0 rad
2. 发送目标 1 rad（小突变）：duration = 1/(50×100e-6) = 200 拍 = 20ms
3. 等待完成，发送目标 10 rad（大突变）：duration = 10/(50×100e-6) = 2000 拍 = 200ms

**预期**：小突变过渡快（20ms），大突变过渡慢（200ms），变化速率恒为 50 rad/s。遥测 `position_rad` 斜率两次相同。

- [ ] **Step 10: 场景H —— 固定时长模式（rate=0 回退）**

通过调试器把 `sys.trans_mgr.smooth_cfg.pos_rate/vel_rate/torque_rate/current_rate` 全部改为 0，`smooth_duration` 改为 500，然后：
1. 位置模式目标 0 rad
2. 发送目标 5 rad

**预期**：duration = smooth_duration = 500 拍 = 50ms（固定时长，不随突变大小变化）。验证 rate=0 回退逻辑。

验证后恢复 rate 默认值。

- [ ] **Step 11: 记录验证结果**

在 `User/Data/plans/2026-07-07-intra-mode-ref-smoothing.md` 末尾追加"验证记录"小节，记录各场景实测波形/结论。

- [ ] **Step 12: 提交验证记录**

```bash
git add User/Data/plans/2026-07-07-intra-mode-ref-smoothing.md
git commit -m "docs(plan): record intra-mode ref smooth verification results"
```

---

## Task 9（可选）: 运行期可配置接口

**Files:**
- Modify: `User/Protocol/joint_proto/jm_proto_ops.c`（按需）
- Modify: `User/Protocol/joint_proto/jm_proto_ops.h`（按需）

若需要上位机运行期调整渐变阈值/时长/dt（调试不同电机/负载），新增一条配置命令。本任务为可选增强，首版可跳过。

- [ ] **Step 1: 评估必要性**

若调试中发现默认阈值（pos=0.1rad 等）对不同电机不通用，再实施本任务。否则保持编译期默认值 + 运行期调试器改 `sys.trans_mgr.smooth_cfg` / `sys.trans_mgr.dt` 即可。

- [ ] **Step 2: （如需）新增 0xXX 配置命令解析**

在 `jm_proto_ops.c` 的 `app_set_mode` 或独立 `app_set_param` 中新增对 `sys.trans_mgr.smooth_cfg` 字段及 `sys.trans_mgr.dt` 的读写（配置内聚于 mgr，无全局变量）。载荷布局参考 `joint_motor_param_index.csv`。dt 需支持运行期配置（默认 100µs）。

- [ ] **Step 3: 提交**

```bash
git add User/Protocol/joint_proto/
git commit -m "feat(proto): add runtime config for ref_smooth thresholds"
```

---

## 风险与注意事项

1. **过渡期间 cmd 频繁更新**：`transition_update` 每拍用最新 raw_ref 做 blend，old_ref 固定为启动时的 prev_ref。连续突变会朝最新目标收敛，参考始终连续。已在用例9覆盖。
2. **与模式切换过渡的交互**：若同模式渐变进行中又收到模式切换指令，`run_state_switch` 会调 `transition_start` 覆盖 old_ref（用当前 `sys->motor.ref`，即 mixed_ref），重启模式切换过渡。这是合理行为——模式切换优先级高于同模式渐变。
3. **PID profile 一致性**：同模式内 `pos_profile`/`vel_profile` 不变，`transition_update` 在过渡期间保留 old_ref 的 profile 值（[ctrl_transition.c:68-69](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorControl/ControlProcess/ctrl_transition.c#L68)），同模式下无影响。
4. **实时性**：`transition_ref_smooth_check` 仅含若干浮点比较 + 一次 `transition_start`（结构体拷贝），执行时间 µs 级，在电流环 ISR 内可接受。rate 模式计算仅多 4 次浮点除法 + max，开销可忽略。mgr 层多一次函数调用，开销可忽略。
5. **位置模式多圈与限幅**：渐变作用于 `ref->pos`（目标位置），下游 `cascade_control_run_position` 与位置限幅（±12.566 rad）仍生效，渐变不绕过限幅。
6. **rate 模式 duration 上界**：rate 模式下大突变会算出较长 duration（如 100 rad 突变 + pos_rate=50 → 4000 拍 = 200ms）。若担心过长，可：(a) 调大 pos_rate；(b) 增加 `smooth_max_duration` 上限字段（本方案未加，保持简洁，按需扩展）。rate 模式 duration 下界为 1（极小突变立即完成）。
7. **rate 单位严格按物理量纲**：pos_rate=rad/s、vel_rate=rad/s²、torque_rate=Nm/s、current_rate=A/s。配置时注意量纲匹配，否则 duration 计算错误。dt 取自 `mgr->dt`（默认 100µs，运行期可改），对应电流环周期 `1/current_freq_hz`（`current_freq_hz=10kHz`）。**注意**：dt 不再从 `sys->motor.dt` 读取，而是 mgr 自持一份（`transition_mgr_init` 传入），确保配置内聚。若两者需一致，`system_state_init` 传入的 dt 应与 `motor_ctrl_init` 相同。
8. **enable=false 完全旁路**：enable=false 时 `transition_ref_smooth_check` 直接返回 false，不触碰 transition 引擎，sys->motor.ref 保持 dispatch 原值（阶跃）。模式切换过渡不受影响（走 mgr 的 if 分支）。
9. **默认阈值需实测微调**：pos=0.1rad（≈5.7°）为保守起点，若减速比大（gear_ratio=100）输出端角度更小，可能需调小阈值。rate 默认值（pos_rate=50 rad/s 等）对应 5 rad→100ms、50 rad/s→100ms，适合中惯量负载；大惯量需调小 rate，小惯量可调大。建议 Task 8 场景验证后据实调整 `ref_smooth_cfg_init_defaults`。
10. **重构回归风险**：Task 3 完成 mgr 迁移后行为必须与改前完全等价（仅模式切换过渡，无 ref_smooth）。务必在 Task 3 Step 8 做回归测试（4 个场景）确认无回归，再进入 Task 6 接入 ref_smooth。若回归失败，问题定位在 mgr 迁移，而非 ref_smooth 新逻辑，缩小排查范围。
11. **target_run_state 归属**：重构后 `target_run_state` 从 `system_state_t` 迁入 `transition_mgr_t`。检查 system_state.c/h 是否有其他位置引用 `sys->target_run_state`，若有需同步改为 `sys->trans_mgr.target_run_state` 或删除冗余字段。
12. **mgr 与 system_state.h 循环依赖（已确认需加 tag）**：`ctrl_transition_mgr.h` 需用 `system_state_t`（`transition_mgr_step` 参数），`system_state.h` 需用 `transition_mgr_t`（成员）。打破循环：`ctrl_transition_mgr.h` **不 include** `system_state.h`，仅前向声明 `typedef struct system_state_s system_state_t;`；`system_state.h` 正常 include `ctrl_transition_mgr.h`。**关键前提**：现有 `system_state_t` 是匿名 struct typedef（[system_state.h:34](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/AppServices/StateMachine/system_state.h#L34)，`typedef struct { ... } system_state_t;`，**无 tag**），匿名 struct 无法前向声明。必须先改为 `typedef struct system_state_s { ... } system_state_t;`（Task 3 Step 1a），tag 名与前向声明一致。

13. **MIT 与 TORQUE 共用 ctrl_type（本次核心修正）**：MIT（[motor_mode_mit.c:19](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorControl/Modes/motor_mode_mit.c#L19)）与 TORQUE（[motor_mode_torque.c:14](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorControl/Modes/motor_mode_torque.c#L14)）都输出 `REF_CTRL_TORQUE`，**用 `ctrl_type` 无法区分**。原方案的 `ctrl_type` 范围判据会把 MIT 也纳入渐变，违背设计意图且破坏 MIT 柔顺特性。修正为 `run_state` 白名单判据（`ref_smooth_run_state_enabled`）。若未来新增反馈型模式（如阻抗/导纳），须同步确认其 run_state 不在白名单内。

14. **COMPLETED 态复位（本次核心修正）**：模式切换过渡与同模式渐变共用同一 `transition_t`。`transition_update` 完成后引擎停在 `TRANSITION_COMPLETED`（引擎自身从不转回 IDLE，见 [ctrl_transition.c:38-44](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorControl/ControlProcess/ctrl_transition.c#L38)）。若 `ref_smooth_check` 只判断 `IN_PROGRESS`，则模式切换完成那拍（state=COMPLETED）会继续走检测逻辑，`prev_ref`(mixed) 与新 dispatch 的 `raw` 差异可能误触发一次同模式渐变。修正：`check` 入口显式把 `COMPLETED` 复位为 `IDLE` 并 return false（本拍不检测）。用例15 守护此逻辑。

15. **HOLD 模式明确排除**：HOLD（[motor_mode_hold.c:13](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorControl/Modes/motor_mode_hold.c#L13)）目标 `ref->pos = ctrl->fb.pos` 每拍跟随反馈，本就是"锁定当前位置"，不应叠加渐变（否则会与反馈跟随冲突）。白名单不含 `RUN_STATE_HOLD`。

## 验证清单

**重构阶段（Task 1-3）**
- [ ] `system_state_t` 已加 tag `system_state_s`，前向声明打破循环依赖，编译无循环报错
- [ ] `sys->target_run_state` 字段已删除，全工程 grep 无残留
- [ ] Task 3 编译 0 error 0 warning
- [ ] Task 3 回归测试 4 场景全过（IDLE→RUN / 模式切换 / 退出 RUN / 急停）
- [ ] 行为与改前完全等价（无 ref_smooth，目标突变仍阶跃）

**ref_smooth 阶段（Task 4-6）**
- [ ] 无全局 `g_ref_smooth_cfg`，配置内聚于 `mgr.smooth_cfg`
- [ ] dt 由 `mgr.dt` 持有（默认 100µs），运行期可改
- [ ] Task 6 编译 0 error 0 warning

**测试与集成（Task 7-8）**
- [ ] Host 单测全部通过（含 enable=false、rate 模式、**用例14 MIT 排除、用例15 COMPLETED 复位**）
- [ ] 虚拟电机场景 A-H 全部符合预期
- [ ] **场景 E1：MIT 模式目标突变不启动渐变（核心修正验证）**
- [ ] 真硬件烧录后位置模式目标突变无抽动
- [ ] 小幅目标变化不受渐变影响（响应及时）
- [ ] enable=false 时行为与改前一致（应急回退有效）
- [ ] rate 模式 duration 随突变大小正确缩放（dt=100µs）
- [ ] rate=0 回退固定时长模式正确
- [ ] 模式切换过渡行为未受影响（回归测试）
- [ ] **模式切换过渡完成那拍不误触发同模式渐变（COMPLETED 复位验证）**
- [ ] MIT/DUTY/OPEN_LOOP/HOLD 模式行为未受影响
