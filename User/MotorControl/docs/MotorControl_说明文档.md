# MotorControl 模块说明文档

> 路径：`User/MotorControl/`
> 适用版本：当前主线（截至 2026-07-13）
> 范围：仅描述本文件夹内的 4 个子模块（CascadeControl / ControlProcess / Modes / PidManager）及其与上下游的接口契约

---

## 1. 概述

### 1.1 模块定位

`MotorControl` 是关节电机固件的**运动控制核心层**，承担以下职责：

- 实现 PMSM 的 FOC 三环级联控制（位置环 → 速度环 → 电流环）
- 提供 12+ 种控制模式（IDLE / DUTY / CURRENT / TORQUE / VELOCITY / POSITION / MIT / OPEN_LOOP / PV / PT / TEST_SWEEP / HOLD）
- 管理模式分发、平滑过渡、无扰切换
- 管理 PID 参数的三级回退加载（Flash → AUTOTUNE → DEFAULT）与运行期调试
- 提供虚拟电机模型，支持无硬件闭环调试

### 1.2 在系统中的位置

```
┌─────────────────────────────────────────────────────────┐
│  AppServices / StateMachine (system_state.c)            │  上层状态机
│  AppServices / ParamService (jm_proto_ops.c)            │  协议回调
└───────────────────────┬─────────────────────────────────┘
                        │ motor_loop_set_cmd / motor_control_loop
                        ▼
┌─────────────────────────────────────────────────────────┐
│                  MotorControl (本模块)                   │
│  ┌──────────────────────────────────────────────────┐   │
│  │ ControlProcess  (模式分发 + 过渡引擎)            │   │
│  └──────────────────────┬───────────────────────────┘   │
│                         │ motor_ref_t                   │
│  ┌──────────────────────▼───────────────────────────┐   │
│  │ CascadeControl  (位置环+速度环 + 电流环 + ISR)   │   │
│  └──────────────────────┬───────────────────────────┘   │
│  ┌──────────────────────┴───────────────────────────┐   │
│  │ PidManager      (三环独立 source + AUTOTUNE)     │   │
│  └──────────────────────┬───────────────────────────┘   │
│  ┌──────────────────────┴───────────────────────────┐   │
│  │ Modes           (12 个模式处理函数)              │   │
│  └──────────────────────────────────────────────────┘   │
└───────────────────────┬─────────────────────────────────┘
                        │ foc / phase_current / half_bridge / encoder
                        ▼
┌─────────────────────────────────────────────────────────┐
│  Devices (dev_motor / dev_encoder / dev_phase_current)  │  设备抽象层
│  Drv (drv_tim / drv_adc / drv_gpio / drv_spi)           │  HAL 驱动层
└─────────────────────────────────────────────────────────┘
```

### 1.3 子模块一览

| 子模块 | 路径 | 文件数 | 职责 |
|---|---|---|---|
| CascadeControl | `CascadeControl/` | 11 | 三环级联算法、电流环 FOC、控制中断编排、虚拟电机 |
| ControlProcess | `ControlProcess/` | 7 | 模式分发表、模式接口规范、过渡引擎、过渡管理器 |
| Modes | `Modes/` | 12 | 12 个独立模式处理函数（仅填充 `motor_ref_t`） |
| PidManager | `PidManager/` | 7 | 三环独立 source 加载、AUTOTUNE 理论估计、运行期 profile |

---

## 2. 模块架构与调用关系

### 2.1 整体调用链

```
                    上层 (main / system_state / jm_proto_ops)
                                      │
                                      │ motor_loop_set_cmd()
                                      ▼
                    ┌──────── motor_loop_isr() ────────┐
                    │           (CascadeControl)        │
                    │                                   │
        ┌───────────┴───────────┐           ┌──────────┴──────────┐
        │  反馈解算 + 状态机    │           │   三环分频调度      │
        │  motor_loop_update_   │           │                     │
        │    feedback()         │           │  pos_tick → cascade_control_run_position()
        │  motor_control_loop() │           │  vel_tick → cascade_control_run()
        │                       │           │  每拍    → cur_loop_run()
        └───────────┬───────────┘           └──────────┬──────────┘
                    │                                  │
                    │                                  ▼
                    │                    ┌──────────────────────────┐
                    │                    │ cur_loop_run() FOC 链路  │
                    │                    │  phase_current.update() │
                    │                    │  foc.clarke()/park()    │
                    │                    │  motor_pid_profile_     │
                    │                    │    calculate() (d/q PI) │
                    │                    │  foc.inverse_park()     │
                    │                    │  foc.pfsvpwm()          │
                    │                    │  half_bridge.set_3pwm() │
                    │                    └──────────────────────────┘
                    ▼
              motor_loop_sync_runtime() → usr.motor_state[M1]
```

### 2.2 数据结构层级

| 结构体 | 定义位置 | 角色 |
|---|---|---|
| `system_state_t` | `AppServices/StateMachine/system_state.h:34` | 顶层状态机，含 `top_state`、`motor`、`trans_mgr` |
| `motor_ctrl_t` | `ControlProcess/motor_control.h:91` | 控制核心，含 `run_state`、`cmd`、`fb`、`ref`、`param` |
| `motor_ref_t` | `ControlProcess/motor_control.h:65` | 模式层唯一输出，含 `ctrl_type` + 各目标值 + profile |
| `motor_loop_t` | `CascadeControl/motor_loop.h:37` | 三环集成上下文，含 `motor`、`sys`、`cascade`、`current` |
| `cascade_ctrl_t` | `CascadeControl/cascade_control.h:66` | 级联控制器，含位置/速度环 PID 状态 |
| `cur_loop_t` | `CascadeControl/current_loop.h:35` | 电流环上下文，含 d/q 轴 PID |

---

## 3. CascadeControl 子模块（级联控制层）

### 3.1 文件清单

| 文件 | 职责 |
|---|---|
| [cascade_control.c](../CascadeControl/cascade_control.c) / [.h](../CascadeControl/cascade_control.h) | 三环级联算法（位置环 + 速度环 + 入环分发） |
| [motor_loop.c](../CascadeControl/motor_loop.c) / [.h](../CascadeControl/motor_loop.h) | 三环集成与控制中断入口 `motor_loop_isr` |
| [current_loop.c](../CascadeControl/current_loop.c) / [.h](../CascadeControl/current_loop.h) | FOC 电流环完整链路 |
| [motor_loop_config.h](../CascadeControl/motor_loop_config.h) | 频率分频、虚拟电机开关 |
| [dev_motor_virtual.c](../CascadeControl/dev_motor_virtual.c) / [.h](../CascadeControl/dev_motor_virtual.h) | 虚拟电机 PMSM 物理模型 |
| [dev_motor_select.h](../CascadeControl/dev_motor_select.h) | 真实/虚拟设备头文件选择器 |
| [dev_motor_stub.h](../CascadeControl/dev_motor_stub.h) | 设备层占位头（历史遗留，未被 select 引用） |

### 3.2 cascade_control — 级联算法

**职责**：纯算法模块，不依赖硬件。输入 `motor_ref_t` 与反馈，输出 dq 轴电流参考 `id_ref` / `iq_ref`。

**核心入口** `cascade_control_run()` 按 `ref->ctrl_type` 分发：

| ctrl_type | 入环层级 | 处理逻辑 | 行号 |
|---|---|---|---|
| `REF_CTRL_POSITION` | 位置环 + 速度环 | 位置环 PID 输出 vel_setpoint → 速度环 PID → iq_ref | `cascade_control.c:111-116` |
| `REF_CTRL_VELOCITY` | 速度环 | 速度环 PID + 力矩前馈 `iq_ff = torque_ff/kt` | `cascade_control.c:119-125` |
| `REF_CTRL_TORQUE` | 电流环 | `iq_ref = torque / kt` | `cascade_control.c:128-130` |
| `REF_CTRL_CURRENT` | 电流环 | 直接用 `ref->id` / `ref->iq` | `cascade_control.c:133-136` |
| `REF_CTRL_VOLTAGE` / `DUTY` / `IDLE` | 旁路 | 电流参考置零 | `cascade_control.c:139-143` |

**无扰切换**（bumpless transfer）：`cascade_control.c:96-103` 检测 `ctrl_type`/`pos_profile`/`vel_profile` 变化时调用 `cascade_bumpless_preload()` 对新入环 PID 预装载积分（速度环用当前 iq 反推，位置环用当前 vel 反推）。

### 3.3 motor_loop — 控制中断编排

**核心接口**：
```c
void motor_loop_init(float current_freq_hz);   // motor_loop.h:57  传入电流环频率(Hz)
void motor_loop_isr(void);                     // motor_loop.h:63  三环控制中断入口
void motor_loop_set_cmd(ctrl_mode_e cmd);      // motor_loop.h:69  下发上层控制指令
motor_loop_t *motor_loop_get(void);            // motor_loop.h:75  获取全局上下文
void motor_loop_set_encoder_dir(int8_t dir);   // motor_loop.h:83  运行时翻转编码器方向
```

**初始化流程**（`motor_loop.c:61-136`）：
1. `motor_param_init()` 加载默认参数 → `motor_profile_apply_param` 覆盖电气身份
2. `motor_profile_sync_to_param` 同步 Flash 标定参数
3. **PID 三级回退加载**：`motor_pid_load_boot` → `motor_pid_load_source_from_flash` → `motor_pid_load`
4. 计算三环周期：`dt_current = 1/freq`、`dt_velocity = dt_current * VEL_DIV`、`dt_position = dt_current * POS_DIV`
5. `motor_pid_profile_init` → `dev_motor_init`（真实/虚拟）
6. 运动解算频率 = 速度环频率，选用 PLL 观测器
7. 虚拟模式下 `virtual_motor_set_period`
8. `system_state_init` → `cascade_control_init` → `cur_loop_init`
9. `phase_current.start` → `half_bridge.start` → `HAL_Delay(2)`
10. `cur_loop_calibrate_offset` → `drv_tim_start_it` 启动控制定时器

### 3.4 motor_loop_isr 执行流程

`motor_loop.c:273-355`，按顺序：

| 步骤 | 行号 | 内容 |
|---|---|---|
| 0. 分频判断 | 279-284 | `vel_tick = (++vel_cnt >= VEL_DIV)`、`pos_tick = (++pos_cnt >= POS_DIV)` |
| 1. 刷新编码器与电角度 | 287-288 | `encoder.update` + `motor_param.update(ELE_RADIAN)`，所有模式统一执行 |
| 2. 解算运动反馈 | 291 | `motor_loop_update_feedback()` 组织 `cascade_fb_t` 并同步到 `sys.motor.fb` |
| 3. 遥测同步错开位置环 | 294-301 | `sync_pending` 时本拍执行 `motor_loop_sync_runtime`；位置拍挂起到下一拍 |
| 4. 状态机生成参考 | 304-305 | `motor_control_loop(&sys)` + `motor_loop_sync_state` |
| 5. CALIB 态分支 | 310-314 | `TOP_FSM_CALIB` 时仅刷新 `phase_current.update` 后 `return` |
| 6. 非运行态分支 | 317-327 | `top_state != TOP_FSM_RUN` 时复位外环+电流环、清零 out、`cur_loop_run`(IDLE)后 return |
| 7. 位置环 | 330-331 | `pos_tick && ctrl_type==POSITION` 时 `cascade_control_run_position` |
| 8. 速度环 + 入环分发 | 334-335 | `vel_tick && ctrl_type>=CURRENT` 时 `cascade_control_run` |
| 9. 电流环 | 338 | `cur_loop_run()` 每拍执行 |
| 10. 母线电流合成 | 340-354 | 仅真实电机 + `USE_DEV_POWER_MONITOR` + `PM_IBUS_SOURCE==1` 时执行 |

### 3.5 current_loop — FOC 电流环

**FOC 链路 8 步**（`current_loop.c:74-132`）：
1. 三相电流采样 `phase_current.update`
2. Clarke 变换 `foc.clarke`
3. Park 变换 `foc.park`
4. 计算 ud/uq（VOLTAGE 直通 / PI 闭环）
5. `foc.set_udq` + 反 Park `foc.inverse_park`
6. SVPWM `foc.pfsvpwm`
7. PWM 输出 `half_bridge.set_3pwm`

**`cur_loop_run` 分支**（`current_loop.c:48-133`）：

| ctrl_type | 行号 | 处理 |
|---|---|---|
| `REF_CTRL_DUTY` | 56-62 | 占空比直控，跳过 FOC 链路，`ccr = PWM_PERIOD * (0.5 + 0.5*duty)` |
| `REF_CTRL_IDLE` | 65-69 | PWM 置零 |
| `REF_CTRL_VOLTAGE` | 85-103 | 旁路 PI，直接用 `ref->ud`/`ref->voltage`；真实电机模式下除以 `vbus` 归一化 |
| 其余（CURRENT/TORQUE/VELOCITY/POSITION） | 104-119 | 电流闭环 PI，真实电机模式下 PI 输出电压除以 Vbus 归一化 |

> **SVPWM 归一化要点**：`foc_core.c` 的 `foc_svpwm` 中 `Ts=1.0`（归一化周期），直接把电压值当占空比时间，缺少 `/Vbus` 因子。修复方案：在 `calib_hw_apply_voltage` 和 `current_loop.c` 的 `REF_CTRL_VOLTAGE` 分支中对真实电机做 `ud/uq /= dev_power_monitor.vbus` 归一化；虚拟电机不归一化（直接用 ud/uq 推进物理模型）。

### 3.6 关键配置（motor_loop_config.h）

| 宏 | 值 | 行号 | 说明 |
|---|---|---|---|
| `MOTOR_LOOP_ENABLE_DEV_DRIVER` | `1u` | 42 | 1=真实驱动，0=虚拟电机（默认真实） |
| `MOTOR_LOOP_VEL_DIV` | `5u` | 54 | 速度环分频：每 5 个电流环中断执行一次 |
| `MOTOR_LOOP_POS_DIV` | `10u` | 59 | 位置环分频：每 10 个电流环中断执行一次 |
| 编译期约束 | — | 62-64 | `POS_DIV % VEL_DIV != 0` 时 `#error` |

**控制频率**（电流环 10kHz 示例）：

| 环路 | 频率 | 周期 |
|---|---|---|
| 电流环 | 10 kHz（中断基频） | 100 µs |
| 速度环 | 2 kHz（10k / 5） | 500 µs |
| 位置环 | 1 kHz（10k / 10） | 1 ms |

### 3.7 虚拟电机（dev_motor_virtual）

**启用条件**：`MOTOR_LOOP_ENABLE_DEV_DRIVER == 0`

**物理模型**（`dev_motor_virtual.c:76-110`，dq PMSM 前向欧拉积分）：
- 电气：`did/dt = (ud - Rs*id + we*Lq*iq) / Ld`、`diq/dt = (uq - Rs*iq - we*Ld*id - we*flux) / Lq`
- 转矩：`Te = 1.5 * poles * (flux*iq + (Ld-Lq)*id*iq)`（含磁阻转矩项）
- 机械：`domega/dt = (Te - Tload - tfric) / inertia`
- 反馈：`theta_e = wrap_2pi(poles * theta_m)`

**一拍延迟建模**：`virt_half_bridge_set_3pwm`（`cur_loop_run` 末尾）才用本拍 `foc.u_dq` 推进物理模型，下一拍 `encoder.update`/`phase_current.update` 输出新状态，符合真实数字控制"本拍电压→下拍反馈"特性。

**子步细分**（`virtual_motor_set_period` L235-250）：保证子步长不超过电气时间常数 `tau_e = Ld/Rs` 的 1/10，n 上限 16。

---

## 4. ControlProcess 子模块（控制过程层）

### 4.1 文件清单

| 文件 | 职责 |
|---|---|
| [motor_control.c](../ControlProcess/motor_control.c) / [.h](../ControlProcess/motor_control.h) | 顶层控制核心、模式分发表、`motor_ctrl_t`/`motor_ref_t`/`motor_cmd_t`/`motor_fb_t` |
| [motor_mode.h](../ControlProcess/motor_mode.h) | 模式处理函数统一签名 `motor_mode_fn` |
| [ctrl_transition.c](../ControlProcess/ctrl_transition.c) / [.h](../ControlProcess/ctrl_transition.h) | 过渡引擎：init/start/update/force_complete + ref_smooth |
| [ctrl_transition_mgr.c](../ControlProcess/ctrl_transition_mgr.c) / [.h](../ControlProcess/ctrl_transition_mgr.h) | 过渡管理器：每拍步进 `transition_mgr_step` 统一入口 |

### 4.2 状态机层级

```
top_state (TOP_FSM_*)         ← 外层/粗粒度状态机（9 个状态）
   │
   └─ 仅在 TOP_FSM_RUN 态下：
        run_state (RUN_STATE_*) ← 内层/细粒度运行模式（约 45 个）
           │
           └─ s_mode_table[run_state] 分发到 motor_mode_*_run
```

**顶层状态**（`state_define.h:10-23`）：INIT / SAFETY / FAULT / IDLE / READY / RUN / CALIB / CONFIG / BOOTLOADER

**三段式使能流程**：
```
IDLE --ENABLE-->  READY --运动指令--> RUN
RUN  --STOP---->  READY --DISABLE--> IDLE
```

**转移许可表**（`system_state.c:116-126`）：FAULT 与 SAFETY 可从任意状态进入（最高优先级），不受许可表限制。

### 4.3 参考控制类型 ref_ctrl_type_e

`motor_control.h:48-57`，告知下游三环从哪一级入环：

| 枚举 | 含义 | 下游行为 |
|---|---|---|
| `REF_CTRL_IDLE = 0` | 空闲 | 输出保持/置零 |
| `REF_CTRL_VOLTAGE` | 开环电压 | dq 电压直送 |
| `REF_CTRL_DUTY` | 占空比直控 | 跳过 FOC |
| `REF_CTRL_CURRENT` | 电流环入 | 直接 id/iq |
| `REF_CTRL_TORQUE` | 电流环入 | torque 换算为 iq |
| `REF_CTRL_VELOCITY` | 速度环入 | 速度环 PID |
| `REF_CTRL_POSITION` | 位置环入 | 位置环 PID |

### 4.4 模式分发表

**位置**：`motor_control.c:11-69`，`static const motor_mode_fn s_mode_table[RUN_STATE_MAX]`

**设计要点**：
- 数组索引 = `run_state_e` 枚举值；数组元素 = 函数指针
- C99 designated initializer `[RUN_STATE_XXX] = func`，未赋值槽位为 NULL
- `motor_ctrl_dispatch`（`motor_control.c:71-79`）仅做边界检查 + 查表 + 调用，无 switch/case
- **新增模式只需在表内注册一行**，无需改动 dispatch 函数

**完整映射表**：

| run_state | 处理函数 | 行号 |
|---|---|---|
| `RUN_STATE_IDLE` | `motor_mode_idle_run` | 13 |
| `RUN_STATE_HOLD` | `motor_mode_hold_run` | 14 |
| `RUN_STATE_OPEN_LOOP` / `VOLTAGE_VECTOR` | `motor_mode_open_loop_run` | 15, 27 |
| `RUN_STATE_CURRENT` / `FIELD_WEAKENING` / `SENSORLESS` | `motor_mode_current_run` | 16, 28, 29 |
| `RUN_STATE_TORQUE` | `motor_mode_torque_run` | 17 |
| `RUN_STATE_MIT` | `motor_mode_mit_run` | 18 |
| `RUN_STATE_VELOCITY` / `VELOCITY_TORQUE` | `motor_mode_velocity_run` | 19, 23 |
| `RUN_STATE_POSITION` / `POSITION_VELOCITY` / `POSITION_TORQUE` | `motor_mode_position_run` | 20, 21, 22 |
| `RUN_STATE_PROFILE_VELOCITY` | `motor_mode_profile_velocity_run` | 24 |
| `RUN_STATE_PROFILE_TORQUE` | `motor_mode_profile_torque_run` | 25 |
| `RUN_STATE_DUTY_CYCLE` | `motor_mode_duty_run` | 26 |
| `RUN_STATE_TEST_SWEEP_FREQ` | `motor_mode_test_sweep_run` | 62 |
| 轨迹/力控/特殊/其他测试模式 | `motor_mode_legacy_run` | 32-68 |

### 4.5 过渡引擎（ctrl_transition）

**状态机**：
```c
typedef enum {
    TRANSITION_IDLE = 0,
    TRANSITION_IN_PROGRESS,
    TRANSITION_COMPLETED
} transition_state_e;
```

**混合策略**（`transition_update` `ctrl_transition.c:27-72`）：

| 场景 | 处理 |
|---|---|
| 非过渡态 | 直接透传 `new_ref` |
| 计数到位 / `duration==0` | 标记 COMPLETED，`ratio=1.0`，透传 |
| **异量纲切换**（`ctrl_type` 变化） | **不混合**，直接采用新参考；无扰切换交由下游三环预装载积分 |
| **同量纲混合** | `ratio = elapsed / duration`，对 8 个标量字段线性 blend：`pos/vel/torque/id/iq/ud/voltage/duty` |

**关键约束**：PID profile 保留旧值直到过渡完成（`ctrl_transition.c:67-69`），避免增益突变叠加中间参考值导致力矩跳变。

### 4.6 同模式目标值渐变（ref_smooth）

`transition_ref_smooth_check`（`ctrl_transition.c:176-244`）处理同模式内目标值突变：

**白名单**（仅以下模式启用）：POSITION / POSITION_VELOCITY / POSITION_TORQUE / VELOCITY / TORQUE / CURRENT

**排除原因**：
- MIT：反馈型，`ctrl_type` 与 TORQUE 相同无法区分
- HOLD：目标跟随 `fb.pos`
- OPEN_LOOP / DUTY / VOLTAGE：直控

**触发阈值**（默认）：
- `pos_thresh=0.1rad`、`vel_thresh=1rad/s`、`torque_thresh=0.1Nm`、`current_thresh=0.5A`
- 速率限制：`pos_rate=50rad/s`、`vel_rate=500rad/s²`、`torque_rate=20Nm/s`、`current_rate=100A/s`

### 4.7 过渡管理器（ctrl_transition_mgr）

`transition_mgr_step`（`ctrl_transition_mgr.c:33-72`）是**唯一**的每拍步进入口：

| 状态 | 处理 |
|---|---|
| 过渡进行中 | 临时改 `run_state` 为 `target_run_state` 调 dispatch 生成 `new_ref`，再 `transition_update` 混合；完成后才真正提交 `run_state` |
| 稳态 | 正常 dispatch + ref_smooth 检测 + 必要时混合 |

**全局过渡时长**：`g_run_state_trans_count = 1000`（以 `motor_control_loop` 调用次数计，非毫秒）。

---

## 5. Modes 子模块（控制模式层）

### 5.1 接口规范

**统一签名**（`motor_mode.h:13`）：
```c
typedef void (*motor_mode_fn)(motor_ctrl_t *ctrl);
```

**关键约束**（`motor_mode.h:9-11`）：
- 模式函数内**只填充 `ctrl->ref`（参考输出），不执行任何环路计算**
- 可在电流环中断调用，禁止动态内存 / 阻塞 / 字符串格式化

**无 start/stop 回调**：模式本身是无状态、纯函数式的。生命周期由 `ctrl_transition_mgr` 承载。

### 5.2 模式对照表

| 模式文件 | 函数 | ctrl_type | 控制目标 | 限幅参数 | 行号 |
|---|---|---|---|---|---|
| `motor_mode_idle.c` | idle_run | IDLE | PWM 置零，跟踪当前位置 | — | 7 |
| `motor_mode_hold.c` | hold_run | POSITION | 主动锁定当前位置（`pos=fb.pos`） | — | 7 |
| `motor_mode_duty.c` | duty_run | DUTY | 占空比直控 | ±1.0（硬编码） | 7 |
| `motor_mode_current.c` | current_run | CURRENT | d/q 轴电流目标 | ±peak_current | 7 |
| `motor_mode_torque.c` | torque_run | TORQUE | 力矩直控 | ±peak_torque | 7 |
| `motor_mode_velocity.c` | velocity_run | VELOCITY | 速度环目标 + 力矩前馈 | ±max_speed | 7 |
| `motor_mode_position.c` | position_run | POSITION | 位置环目标（3 分支） | POSITION_TORQUE: ±peak_t | 7 |
| `motor_mode_mit.c` | mit_run | TORQUE | MIT 阻抗律 | ±peak_torque | 7 |
| `motor_mode_open_loop.c` | open_loop_run | VOLTAGE | 开环电压 | ±rated_voltage | 7 |
| `motor_mode_profile_velocity.c` | profile_velocity_run | VELOCITY | PV：速度+斜坡规划（TODO） | ±max_speed | 8 |
| `motor_mode_profile_torque.c` | profile_torque_run | TORQUE | PT：力矩+斜坡规划（TODO） | ±peak_torque | 8 |
| `motor_mode_test_sweep.c` | test_sweep_run | TORQUE | 扫频测试（正弦力矩） | ±0.5（硬编码振幅） | 9 |

### 5.3 各模式使用的控制环

实际控制环计算在下游 `cascade_control.c:89-145`，依据 `ref->ctrl_type` 选择入环层级：

| 模式 | 位置环 | 速度环 | 电流环 |
|---|---|---|---|
| idle / duty / open_loop | — | — | 旁路（电压/占空比直通） |
| current | — | — | 直接 id/iq |
| torque / MIT / PT / test_sweep | — | — | torque/kt → iq |
| velocity / PV | — | 速度环 PID | 速度环输出 → iq |
| position / hold | 位置环 PID | 速度环 PID | → iq |

### 5.4 模式差异点

**PV/PT vs 普通 velocity/torque**：
- 当前实现**完全相同**（前期透传），仅预留斜坡/轮廓规划接口（`motor_mode_profile_velocity.c:5,17` 标注 TODO）
- 设计意图：PV/PT 在指令到达 ref 前先经过斜率限制器，平滑速度/力矩指令阶跃

**velocity vs velocity_torque**：
- `motor_mode_velocity.c:21-24`：VELOCITY_TORQUE 变体在普通速度模式基础上**额外设置 `ref->torque`**（用 peak_t 限幅）

**position 的三分支**（`motor_mode_position.c:19-33`）：
- `POSITION`：纯位置模式
- `POSITION_VELOCITY`：叠加 `vel_ff = cmd.vel`（速度前馈）
- `POSITION_TORQUE`：额外设 `ref->torque`（力矩限幅，peak_t）

**idle vs hold**：
- 两者都设 `ref->pos = fb.pos`
- `idle.c:12` 设 `ctrl_type = REF_CTRL_IDLE` → 下游不跑任何环，PWM 置零
- `hold.c:12` 设 `ctrl_type = REF_CTRL_POSITION` → 下游跑位置环，**主动**锁定当前位置（有电流输出）

### 5.5 MIT 阻抗控制实现

`motor_mode_mit.c:13-21`：
```c
float pos_err = ctrl->cmd.pos - ctrl->fb.pos;     // 行13
float vel_err = ctrl->cmd.vel - ctrl->fb.vel;     // 行14
float torque = ctrl->cmd.kp * pos_err
             + ctrl->cmd.kd * vel_err
             + ctrl->cmd.torque_ff;               // 行15
ref->ctrl_type = REF_CTRL_TORQUE;                 // 行19
ref->torque = motor_mode_clamp(torque, -peak_t, peak_t);  // 行20
```

**关键点**：
- `kp` / `kd` 来自**运行时指令** `ctrl->cmd.kp` / `ctrl->cmd.kd`（每条命令可变），符合 MIT Cheetah 协议
- 限幅用 **peak_torque**，不是 `iq_max`（`iq_max` 仅用于 legacy 阻抗模式）
- MIT 律 `τ = kp·Δθ + kd·Δω + τ_ff` 本质是弹簧-阻尼+前馈的阻抗控制

### 5.6 test_sweep 扫频实现

`motor_mode_test_sweep.c:15-17`：
```c
ref->torque = sinf(ctrl->test_phase) * 0.5f;                              // 行15
ctrl->test_phase += 2.0f * (float)M_PI * ctrl->test_freq * ctrl->dt;     // 行17
```

- **相位累加**：每个控制周期推进 `2π·f·dt` 弧度
- **状态存储于实例字段**（`motor_ctrl_t` 成员 `test_phase`/`test_freq`，非 static），避免多电机共享相位
- **频率可变**：`test_freq` 运行期可修改，相位连续累加，改频时不跳变，适合扫频（chirp）测试
- **振幅硬编码**：0.5 N·m，不可配置
- **初始化**：`test_phase=0.0f`、`test_freq=1.0f`（默认 1Hz）

### 5.7 peak_current 限流生效范围

**两层限流**：

| 层级 | 生效模式 | 限流对象 |
|---|---|---|
| 模式层直接限幅 | 仅 `current` 模式 | `ref->id` / `ref->iq` |
| 下游级联最终限流 | torque / MIT / velocity / position / hold / PV / PT / test_sweep | iq_ref（`cascade_control.c:93` 取 peak_i 作为最终钳位） |
| 不限流 | open_loop / duty / idle | 走电压/占空比直通，电流参考置零 |

---

## 6. PidManager 子模块（PID 管理器）

### 6.1 文件清单

| 文件 | 职责 |
|---|---|
| [motor_pid_manager.h](../PidManager/motor_pid_manager.h) | 门面头文件，汇总三个子头 |
| [motor_pid_load.c](../PidManager/motor_pid_load.c) / [.h](../PidManager/motor_pid_load.h) | 三环独立 source 加载 + 三级回退 |
| [motor_pid_autotune.c](../PidManager/motor_pid_autotune.c) / [.h](../PidManager/motor_pid_autotune.h) | 零极点对消法理论估计 |
| [motor_pid_profile.c](../PidManager/motor_pid_profile.c) / [.h](../PidManager/motor_pid_profile.h) | 运行期 profile 管理（`s_motor_pid_profiles` 数组） |

### 6.2 三环独立 source 机制

**核心设计**：每个环（电流/速度/位置）各自维护一个 `pid_source_e`，互不干扰。切换电流环 source 不影响速度/位置环。

**`pid_source_e` 枚举**（`motor_pid_load.h:28-34`）：

| 值 | 名称 | 数据来源 | 持久化 |
|----|------|----------|--------|
| 0 | DEFAULT | `motor_param.c` 编译期常量 | 是（代码内） |
| 1 | FLASH | Flash 中 `ControlParam_t` 工程调试值 | 是 |
| 2 | AUTOTUNE | 理论估计值（先写入 `ControlParam_t`） | 是（写入后需 0xEA 固化） |
| 3 | DEBUG | 0xA5 下发的实时值 | **否**（重启消失） |

**`pid_ring_e` 枚举**（`motor_pid_load.h:39-45`）：

| 值 | 名称 | 说明 |
|----|------|------|
| 0 | PID_RING_CURRENT | 电流环（D/Q 双轴合并） |
| 1 | PID_RING_VELOCITY | 速度环 |
| 2 | PID_RING_POSITION | 位置环 |

> 注意：`pid_ring_e` 只有 3 个值（电流环合并 D/Q），与 `motor_pid_profile_id_e` 的 4 值映射（D/Q 分开）不同。

### 6.3 pid_source_mask 位域布局

字段定义位于 `DataHub/motor_info.h:166`，位域访问宏位于 `motor_pid_load.h:104-107`：

| 位段 | 环 | SHIFT | 含义 |
|------|----|----|------|
| bit[3:0] | 电流环 | 0 | 0=DEFAULT, 1=FLASH, 2=AUTOTUNE, 3=DEBUG |
| bit[7:4] | 速度环 | 4 | 同上 |
| bit[11:8] | 位置环 | 8 | 同上 |
| bit[31:12] | 保留 | — | 未使用 |

**关键特性**：DEBUG 值**不写入 mask**（`jm_proto_ops.c:293-299` 仅当 `source != PID_SOURCE_DEBUG` 才写 mask），重启自动回退。

### 6.4 三级回退加载流程

`motor_pid_load_boot(param, info)` 在 `motor_loop_init` 中调用一次，三环各自独立执行：

```
┌─────────────────────────────────────────────────────────────┐
│ 对每个环 (CURRENT / VELOCITY / POSITION) 独立执行:          │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│   Step 1: Flash 字段范围检查                                │
│   ├─ flash_current_valid(ctl)?   [L110-113]                 │
│   │   kp_ld/kp_lq ∈ (0, 1000), ki_ld/ki_lq ∈ (0, 100000)    │
│   ├─ flash_velocity_valid(ctl)?  [L115-118]                 │
│   │   kp_s ∈ (0, 10000), ki_s ∈ (0, 100000)                 │
│   └─ flash_position_valid(ctl)?  [L120-124]                 │
│       kp_p ∈ (0, 10000)（位置环纯比例，ki 可为 0）          │
│                  │                                          │
│           通过  │  不通过                                   │
│         ┌───────┴────────┐                                  │
│         ▼                ▼                                  │
│   source=FLASH      Step 2: AUTOTUNE 估计                   │
│   写入 param        motor_pid_autotune_current/velocity/    │
│   从 ctl            position(info, 0, ...)                  │
│                      返回 0?                                │
│                      │                                      │
│                 成功  │  失败 (-2 辨识未就绪)                │
│                ┌─────┴──────┐                               │
│                ▼            ▼                               │
│          source=AUTOTUNE  source=DEFAULT                    │
│          写入 param       保留默认值                         │
│          从估计值         (不覆盖)                           │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

> **重要说明**：代码中**不存在** `pid_flash_valid_magic` 字段或 `0xC0DE2014` 常量。PID Flash 有效性检查采用**字段范围检查**（三个 `flash_*_valid` 函数）。整个 `motor_info_t` 的 Flash 校验由 `motor_info_storage.c:54-68` 的四重校验完成：`PARAM_MAGIC = 0x53455256`（"SERVO"）+ config_version + CRC32 + 字段范围校验。

### 6.5 AUTOTUNE 理论估计公式

**默认带宽**（`motor_pid_autotune.c:15-17`）：
- 电流环：1000 Hz
- 速度环：100 Hz
- 位置环：20 Hz

**电流环**（零极点对消法，`motor_pid_autotune.c:38-69`）：
```
ωc = 2π × bandwidth_hz
Kp_d = ωc × Ld        Kp_q = ωc × Lq
Ki_d = ωc × R         Ki_q = ωc × R
rated_current = current_lim × 0.6
integral_limit = Kp_q × rated_current × 1.5
```

**速度环**（二阶最佳阻尼 ξ=0.707，`motor_pid_autotune.c:71-102`）：
```
ωc = 2π × bandwidth_hz
Kp = J × ωc / Kt
Ki = J × ωc² / (4 × Kt)
integral_limit = current_lim × 0.5
```

**位置环**（纯比例，`motor_pid_autotune.c:104-122`）：
```
Kp = 2π × bandwidth_hz   (带宽 = Kp)
Ki = 0
integral_limit = 0
```

**辨识数据就绪检查**（`motor_pid_autotune.c:23-36`）：
- `is_calibrated == 1`
- `phase_resistance >= 0.001`
- `phase_inductance_d >= 0.00001`
- 速度环额外：`rotor_inertia >= 1e-7` 且 `torque_constant >= 0.01`

**事务性应用** `motor_pid_autotune_apply`（`motor_pid_autotune.c:124-158`）：任一环失败则全部不写入，返回错误码。

### 6.6 运行期 profile（motor_pid_profile）

**`motor_pid_profile_id_e` 枚举**（`motor_pid_profile.h:12-23`）：

| 值 | 名称 | 用途 |
|----|------|------|
| 0 | CURRENT_D | d 轴电流环 |
| 1 | CURRENT_Q | q 轴电流环 |
| 2 | VELOCITY | 速度环 |
| 3 | POSITION | 位置环 |
| 4 | IMPEDANCE | 阻抗控制（legacy） |
| 5 | HOMING | 回零模式（legacy） |
| 6 | JOG | 点动模式（legacy） |
| 7 | TEST | 测试模式 |

**`s_motor_pid_profiles[MOTOR_PID_PROFILE_MAX]`**（`motor_pid_profile.c:5`）：全局唯一的运行期 PID 参数表，ISR 直接读取此表执行 PID 计算。

**默认值示例**：
- `current_d`：kp=5.5, ki=1500, output_limit=24, integral_limit=200, flags=ANTI_WINDUP
- `current_q`：kp=5.5, ki=1500, output_limit=48, integral_limit=10, flags=ANTI_WINDUP
- `velocity`：kp=0.08, ki=0.8, output_limit=20, integral_limit=200
- `position`：kp=10, ki=0, output_limit=30

### 6.7 DEBUG 模式工作机制

```
1. 上位机发 0xA1: ring_select=某环, source=3(DEBUG)
   ├─ 状态检查: 必须 IDLE 态
   ├─ motor_pid_set_source(ring, DEBUG)  // 写 RAM s_ring_source
   ├─ source==DEBUG → 不写 pid_source_mask (不持久化)
   └─ motor_pid_reload()
       ├─ motor_pid_load case DEBUG: break (不覆盖 motor_param_t)
       └─ motor_param_t 未变 → profile 保持原值

2. 上位机发 0xA5: ring=0~3, param_type=1~7, value4=4B
   ├─ ring → src_ring 映射:
   │     ring 0/1 (D/Q) → PID_RING_CURRENT
   │     ring 2 (速度)   → PID_RING_VELOCITY
   │     ring 3 (位置)   → PID_RING_POSITION
   ├─ 检查 motor_pid_get_source(src_ring) == DEBUG, 否则 NACK(STATE_DENY)
   ├─ motor_pid_profile_set_param(ring, param_type, value4)
   │   → 直接 memcpy 写入 s_motor_pid_profiles[ring].param
   └─ ISR 下一拍生效 (无需 reload)

3. 退出 DEBUG:
   a. 0xA1 切回 source=FLASH/AUTOTUNE/DEFAULT
      → motor_pid_reload → profile 从 motor_param_t 恢复 → 调试值丢失
   b. 断电重启 → 自动回 DEFAULT/FLASH/AUTOTUNE
```

---

## 7. 协议接口

### 7.1 PID 管理命令（0xA0~0xAF）

> **重要修正**：协议命令实际为 `0xA0~0xA6`，非计划文档中的 `0x9A/0x9B/0x9C`。`0x9A/0x9B/0x9C` 仅出现在计划文档文件名中，落地时改为 `0xA0~0xA6`。

| 命令 | 函数 | 载荷 | 行为 | 状态限制 |
|------|------|------|------|----------|
| `0xA0` | `app_pid_autotune` | `ring_select:u8, cur_bw:f32, vel_bw:f32, pos_bw:f32` | 事务计算三环 → 写 ControlParam_t → 按 ring_select 设 source=AUTOTUNE → reload | IDLE |
| `0xA1` | `app_pid_source_set` | `ring_select:u8(0~2), source:u8(0~3)` | 设 s_ring_source → 非 DEBUG 写 pid_source_mask → reload | IDLE |
| `0xA2` | `app_pid_source_get` | 无输入，输出 3 字节 | 返回 cur/vel/pos 三环 source | 无 |
| `0xA5` | `app_pid_param_set` | `ring:u8(0~3), param_type:u8(1~7), value4:4B` | 检查对应环==DEBUG → 直接写 s_motor_pid_profiles | DEBUG 模式 |
| `0xA6` | `app_pid_param_get` | `ring:u8(0~3), param_type:u8(1~7)` | 返回 4B profile 值 | 无 |

### 7.2 两套 ring 映射

代码中存在**两套不同的 ring 映射**，分别用于不同命令：

**`pid_ring_e`**（用于 0xA0/0xA1/0xA2 source 管理）：

| 值 | 含义 |
|----|------|
| 0 | 电流环（D/Q 双轴合并） |
| 1 | 速度环 |
| 2 | 位置环 |

**`motor_pid_profile_id_e`**（用于 0xA5/0xA6 参数读写）：

| 值 | 含义 |
|----|------|
| 0 | D 轴电流环 |
| 1 | Q 轴电流环 |
| 2 | 速度环 |
| 3 | 位置环 |

**0xA0 ring_select 语义**：`0=电流, 1=速度, 2=位置, 3=全部`。

### 7.3 0xA5/0xA6 param_type 编码

| param_type | 字段 | 数据类型 |
|------------|------|----------|
| 1 | kp | float |
| 2 | ki | float |
| 3 | kd | float |
| 4 | output_limit | float |
| 5 | integral_limit | float |
| 6 | output_filter_alpha | float |
| 7 | flags | uint32_t |

`value4` 为 4 字节小端，float 字段按 `memcpy` 转换。

---

## 8. 关键设计要点

### 8.1 模式层与环路层解耦

- **模式层**（Modes）：纯函数式，仅填充 `motor_ref_t`，不执行任何环路计算
- **环路层**（CascadeControl）：依据 `ref->ctrl_type` 选择入环层级
- **过渡层**（ControlProcess）：管理模式切换的平滑过渡，不直接操作环路

### 8.2 三环独立 source

- `s_ring_source[3]` 数组让三环 source 互不干扰
- 上电时 Flash 字段范围检查 → AUTOTUNE 理论估计 → DEFAULT 默认值，逐级 fallback
- DEBUG 模式不持久化，重启自动消失

### 8.3 无扰切换（Bumpless Transfer）

- 模式切换过渡：同量纲线性混合，异量纲直接切换（交由下游预装载积分）
- 同模式目标值渐变：ref_smooth 白名单模式启用，按速率限制计算时长
- 下游三环检测 `ctrl_type` 变化后预装载积分

### 8.4 遥测同步错开位置环

`motor_loop_isr` 把 `motor_loop_sync_runtime` 从位置拍挂起到下一拍执行，避开位置环重负载拍，降低 ISR 抖动。

### 8.5 一拍延迟建模（虚拟电机）

虚拟电机在 `virt_half_bridge_set_3pwm`（`cur_loop_run` 末尾）才用本拍 `foc.u_dq` 推进物理模型，下一拍 `encoder.update`/`phase_current.update` 输出新状态，符合真实数字控制"本拍电压→下拍反馈"的一拍延迟特性。

### 8.6 SVPWM 归一化

`foc_core.c` 的 `foc_svpwm` 中 `Ts=1.0`（归一化周期），直接把电压值当占空比时间，缺少 `/Vbus` 因子。修复方案：真实电机模式下对 `ud/uq` 除以 `dev_power_monitor.vbus` 归一化；虚拟电机不归一化。

---

## 9. 附录：文件清单

### 9.1 CascadeControl（11 文件）

| 文件 | 行数 | 主要内容 |
|---|---|---|
| `cascade_control.c/h` | — | 级联算法、`cascade_ctrl_t`、入环分发 |
| `motor_loop.c/h` | — | `motor_loop_isr`、`motor_loop_init`、三环分频调度 |
| `current_loop.c/h` | — | FOC 电流环 8 步链路、`cur_loop_t` |
| `motor_loop_config.h` | 64 | `MOTOR_LOOP_ENABLE_DEV_DRIVER`、`VEL_DIV`、`POS_DIV` |
| `dev_motor_virtual.c/h` | — | dq PMSM 物理模型、子步细分 |
| `dev_motor_select.h` | 22 | 真实/虚拟头文件选择器 |
| `dev_motor_stub.h` | — | 设备层占位头（历史遗留，未被引用） |

### 9.2 ControlProcess（7 文件）

| 文件 | 行数 | 主要内容 |
|---|---|---|
| `motor_control.c/h` | — | `motor_ctrl_t`/`motor_ref_t`/`motor_cmd_t`/`motor_fb_t`、`s_mode_table`、`motor_ctrl_dispatch`、legacy fallback |
| `motor_mode.h` | — | `motor_mode_fn` 签名、`motor_mode_clamp` |
| `ctrl_transition.c/h` | — | `transition_t`、`transition_state_e`、`ref_smooth_cfg_t` |
| `ctrl_transition_mgr.c/h` | — | `transition_mgr_t`、`transition_mgr_step` |

### 9.3 Modes（12 文件）

| 文件 | 行数 | 函数 |
|---|---|---|
| `motor_mode_idle.c` | ~20 | `motor_mode_idle_run` |
| `motor_mode_hold.c` | ~20 | `motor_mode_hold_run` |
| `motor_mode_duty.c` | ~20 | `motor_mode_duty_run` |
| `motor_mode_current.c` | ~20 | `motor_mode_current_run` |
| `motor_mode_torque.c` | ~20 | `motor_mode_torque_run` |
| `motor_mode_velocity.c` | ~30 | `motor_mode_velocity_run` |
| `motor_mode_position.c` | ~40 | `motor_mode_position_run` |
| `motor_mode_mit.c` | ~25 | `motor_mode_mit_run` |
| `motor_mode_open_loop.c` | ~20 | `motor_mode_open_loop_run` |
| `motor_mode_profile_velocity.c` | ~20 | `motor_mode_profile_velocity_run` |
| `motor_mode_profile_torque.c` | ~20 | `motor_mode_profile_torque_run` |
| `motor_mode_test_sweep.c` | ~20 | `motor_mode_test_sweep_run` |

### 9.4 PidManager（7 文件）

| 文件 | 行数 | 主要内容 |
|---|---|---|
| `motor_pid_manager.h` | 26 | 门面头文件，汇总三个子头 |
| `motor_pid_load.h` | 141 | `pid_source_e`/`pid_ring_e`、mask 位域宏、API 声明 |
| `motor_pid_load.c` | 221 | 三级回退、source 管理、reload 实现 |
| `motor_pid_autotune.h` | 74 | `autotune_result_t`、三环估计 API 声明 |
| `motor_pid_autotune.c` | 158 | 零极点对消法公式、事务性 apply |
| `motor_pid_profile.h` | 150 | `motor_pid_profile_id_e`、profile API 声明 |
| `motor_pid_profile.c` | 300 | `s_motor_pid_profiles` 数组、默认值、单字段读写 API |

---

## 10. 关联文档与代码引用

- 状态机定义：`User/DataHub/state_define.h`（`run_state_e` / `top_fsm_e` / `ctrl_mode_e`）
- 状态机实现：`User/AppServices/StateMachine/system_state.c/h`（顶层状态机 + 转移许可表 + 指令映射表）
- 协议层实现：`User/AppServices/ParamService/jm_proto_ops.c`（0xA0~0xA6 命令回调）
- 电机参数：`User/DataHub/motor_param.h/c`（`motor_param_t`、`peak_current`/`peak_torque`/`max_speed` getter）
- 电机信息存储：`User/AppServices/ParamService/motor_info_storage.c/h`（Flash 四重校验）
- 电机配置：`User/Config/motor_profile.c/h`（默认参数）
- FOC 核心：`foc_core.c`（Clarke/Park/SVPWM）
