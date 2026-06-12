# 系统状态机逻辑图

> 对应实现：`system_state.c` / `system_state.h` / `state_define.h` / `ctrl_transition.h`
> 本文档采用 Mermaid 语法，可在支持 Mermaid 的编辑器（VSCode + Markdown Preview Mermaid Support 插件、GitHub、Typora 等）中直接渲染。

---

## 1. 第一层：顶层主状态机 (top_fsm_e)

第一层决定「系统处于什么阶段、伺服是否使能」。FAULT / SAFETY 为抢占式状态，可从任意状态进入（不查许可表）。

```mermaid
stateDiagram-v2
    direction TB
    [*] --> INIT : 上电

    INIT --> IDLE : system_state_init 自动切换

    state "IDLE 待机/伺服失能" as IDLE
    state "READY 就绪/已使能/电机不动" as READY
    state "RUN 运行态(运行子状态生效)" as RUN
    state "CALIB 校准" as CALIB
    state "CONFIG 配置" as CONFIG
    state "BOOTLOADER 固件升级" as BOOT
    state "FAULT 故障" as FAULT
    state "SAFETY 急停(最高优先级)" as SAFETY

    IDLE --> READY : ENABLE 上使能
    READY --> RUN : 运动指令(首次直接置位,无过渡)
    RUN --> READY : STOP (保持使能)
    READY --> IDLE : DISABLE 下使能 (RUN 经 READY 逐级回退)

    IDLE --> BOOT : ENTER_BOOTLOADER
    BOOT --> IDLE

    IDLE --> CALIB : (*许可表开放,当前无指令触发)
    CALIB --> IDLE
    IDLE --> CONFIG : (*许可表开放,当前无指令触发)
    CONFIG --> IDLE

    note right of SAFETY
        ESTOP: 任意状态 → SAFETY
        仅响应 CLEAR_FAULT → IDLE
    end note
    note right of FAULT
        fault_check 触发(当前为空桩)
        仅响应 CLEAR_FAULT → IDLE
    end note

    SAFETY --> IDLE : CLEAR_FAULT 解除急停
    FAULT --> IDLE : CLEAR_FAULT 清除故障
```

### 1.1 转移许可表 `s_top_fsm_allowed[from][to]`

| From \ To | INIT | IDLE | READY | RUN | CALIB | CONFIG | BOOT |
|-----------|:----:|:----:|:-----:|:---:|:-----:|:------:|:----:|
| INIT      |      |  ✓   |       |     |       |        |      |
| IDLE      |      |      |  ✓    |     |  ✓    |   ✓    |  ✓   |
| READY     |      |  ✓   |       |  ✓  |       |        |      |
| RUN       |      |      |  ✓    |     |       |        |      |
| FAULT     |      |  ✓   |       |     |       |        |      |
| SAFETY    |      |  ✓   |       |     |       |        |      |
| CALIB     |      |  ✓   |       |     |       |        |      |
| CONFIG    |      |  ✓   |       |     |       |        |      |
| BOOT      |      |  ✓   |       |     |       |        |      |

> - FAULT / SAFETY 不在表中：由 `top_fsm_transition_allowed()` 单独放行，任意状态可进入。
> - `(*)` IDLE→CALIB / IDLE→CONFIG 在许可表中开放，但 `process_ctrl_cmd()` 当前没有指令会触发进入这两个状态。

---

## 2. 指令处理优先级 (process_ctrl_cmd)

控制指令按固定优先级处理，急停最高。

```mermaid
flowchart TD
    A[process_ctrl_cmd 收到 cmd] --> B{cmd >= CONTROL_MODE_MAX ?}
    B -- 是 --> Z[忽略, 返回]
    B -- 否 --> C{cmd == ESTOP ?}
    C -- 是 --> C1[切换 SAFETY 并返回]
    C -- 否 --> D{top_state == FAULT ?}
    D -- 是 --> D1{cmd == CLEAR_FAULT ?}
    D1 -- 是 --> D2[切换 IDLE]
    D1 -- 否 --> Z
    D -- 否 --> E{top_state == SAFETY ?}
    E -- 是 --> E1{cmd == CLEAR_FAULT ?}
    E1 -- 是 --> E2[切换 IDLE 解除急停]
    E1 -- 否 --> Z
    E -- 否 --> F{系统级指令?<br/>IDLE/DISABLE/ENABLE/STOP/<br/>BOOTLOADER/SAVE/RESET}
    F -- 是 --> F1[执行对应顶层切换并返回]
    F -- 否 --> G{运动指令: 当前已使能?}
    G -- top_state==READY --> G1[切换 RUN<br/>run_state=映射目标<br/>首次直接置位]
    G -- top_state==RUN --> G2[run_state_switch<br/>启动平滑过渡]
    G -- 其他状态 --> Z2[忽略]
```

---

## 3. 第二层：运行子状态机 (run_state_e) — 仅 RUN 态内有效

第二层决定「使能后电机用哪种控制律运动」。仅当 `top_state == TOP_FSM_RUN` 时由 `motor_control_loop()` 执行。

```mermaid
stateDiagram-v2
    direction LR
    [*] --> RUN_STATE_IDLE : 进入RUN首次置位

    state "运行子状态 motor.run_state" as RS {
        RUN_STATE_IDLE --> 基础控制
        RUN_STATE_IDLE --> 高级力控
        RUN_STATE_IDLE --> 轨迹同步
        RUN_STATE_IDLE --> 特殊应用
        RUN_STATE_IDLE --> 测试诊断

        state "基础控制 (开环/电流/力矩/MIT/速度/位置/复合/占空比/电压矢量/弱磁/无感)" as 基础控制
        state "高级力控 (阻抗/导纳/力控/力位混合/重力补偿/碰撞/零力/恒力/变阻抗/自适应/落地缓冲)" as 高级力控
        state "轨迹同步 (PVT/三次样条/梯形/S曲线/回零/电子齿轮/电子凸轮)" as 轨迹同步
        state "特殊应用 (脉冲方向/模拟量/PWM/点动/安全示教)" as 特殊应用
        state "测试诊断 (老化/扫频/齿槽/摩擦/惯量/诊断/高速采集/单步)" as 测试诊断
    }

    note right of RS
        RUN 态内切换运动指令:
        run_state_switch() 启动 transition 平滑过渡
        过渡时长 = g_run_state_trans_count (调用次数计)
    end note

    RS --> [*] : STOP/DISABLE/ESTOP/FAULT 归零IDLE并强制结束过渡
```

### 3.1 控制指令 → 运行子状态映射 `s_ctrl_mode_to_run_state`

| 控制指令 (ctrl_mode_e)        | 运行子状态 (run_state_e)        |
|-------------------------------|---------------------------------|
| IDLE / HOLD / BRAKE           | RUN_STATE_IDLE                  |
| OPEN_LOOP                     | RUN_STATE_OPEN_LOOP             |
| CURRENT                       | RUN_STATE_CURRENT               |
| TORQUE                        | RUN_STATE_TORQUE                |
| MIT                           | RUN_STATE_MIT                   |
| VELOCITY                      | RUN_STATE_VELOCITY              |
| POSITION                      | RUN_STATE_POSITION              |
| POSITION_VELOCITY             | RUN_STATE_POSITION_VELOCITY     |
| POSITION_TORQUE               | RUN_STATE_POSITION_TORQUE       |
| VELOCITY_TORQUE               | RUN_STATE_VELOCITY_TORQUE       |
| DUTY_CYCLE                    | RUN_STATE_DUTY_CYCLE            |
| VOLTAGE_VECTOR                | RUN_STATE_VOLTAGE_VECTOR        |
| FIELD_WEAKENING               | RUN_STATE_FIELD_WEAKENING       |
| SENSORLESS                    | RUN_STATE_SENSORLESS            |
| IMPEDANCE ~ LANDING_BUFFER    | 对应 RUN_STATE_* (高级力控)      |
| PVT ~ ELECTRONIC_CAM          | 对应 RUN_STATE_* (轨迹同步)      |
| STEP_DIR ~ SAFE_TEACH         | 对应 RUN_STATE_* (特殊应用)      |
| TEST_* / DIAGNOSTIC / DAQ ... | 对应 RUN_STATE_* (测试诊断)      |

> **映射缺口（复核注意）**：`CANOPEN_SYNC`、`ETHERCAT_CSP/CSV/CST`、`PP/PV/PT`、`TEST_CURRENT_LOOP`、`TEST_VELOCITY_LOOP` 在 `ctrl_mode_e` 中已定义，但映射表中未配置，默认值为 0 → `RUN_STATE_IDLE`，会出现「进 RUN 但电机不动」的静默退化。

---

## 4. 平滑过渡机制 (ctrl_transition)

`motor_control_loop()` 在 RUN 态处理运动模式切换的平滑过渡。

```mermaid
flowchart TD
    A[motor_control_loop] --> B[fault_check]
    B --> C{top_state == RUN ?}
    C -- 否 --> C1[ref.ctrl_type = IDLE<br/>不生成运动参考, 返回]
    C -- 是 --> D{transition.state == IN_PROGRESS ?}
    D -- 是 --> E[临时切到 target_run_state<br/>dispatch 生成 new_ref<br/>恢复 old_state]
    E --> F[transition_update 参考层混合<br/>输出 mixed_ref]
    F --> G{过渡完成?}
    G -- 是 --> G1[正式切换 run_state = target_run_state]
    G -- 否 --> G2[继续过渡]
    D -- 否 --> H[motor_ctrl_dispatch<br/>直接生成当前状态参考]
```

**过渡两种语义（由 `ctrl_type` 量纲决定）：**

- **同 `ctrl_type`（量纲一致）**：对目标参考值做线性混合，输出连续无跳变。
- **异 `ctrl_type`（量纲不同）**：不混合，直接切到新参考，由下游三环检测 `ctrl_type` 变化后预装载积分实现无扰切换。

---

## 5. 两层关系总览

```mermaid
flowchart TB
    subgraph L1["第一层 顶层FSM (谁能动)"]
        direction LR
        I1[INIT] --> I2[IDLE] --> I3[READY]
        I3 <--> I4[RUN]
        SAF[SAFETY/FAULT 抢占]
    end

    subgraph L2["第二层 运行子FSM (怎么动) — 仅 RUN 激活"]
        direction LR
        R1[run_state: 电流/位置/力矩/阻抗/轨迹/测试...]
        R2[切换走 transition 平滑过渡]
        R1 --- R2
    end

    I4 -- "top_state==RUN 时第二层生效" --> L2
    L2 -- "非RUN态: ref.ctrl_type 强制IDLE, 子状态不生效" --> I2
```

---

## 6. 复核备注（实现与文档差异）

| 编号 | 问题 | 位置 | 影响 |
|:----:|------|------|------|
| ① | CALIB / CONFIG 顶层态无指令可进入 | `process_ctrl_cmd` | 校准/配置功能进不去 |
| ② | 部分已定义指令未配映射表，静默退化为 IDLE | `s_ctrl_mode_to_run_state` | 进RUN但不动，无拒绝反馈 |
| ③ | HOLD / BRAKE 映射到 RUN_STATE_IDLE，会推进 RUN 态 | 映射表 | 语义需确认 |
| ④ | `fault_check` 为空桩，无自动故障检测 | `fault_check` | 与头文件注释不符 |
| ⑤ | 文件头注释 FAULT/SAFETY 顺序与枚举(SAFETY先)不一致 | `system_state.c:4` | 仅文档误导，功能无影响 |
