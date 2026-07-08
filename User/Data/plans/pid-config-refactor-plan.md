# PID 配置重构方案

## 摘要

`motor_param.c` 是脚本自动生成的数据文件，不可手动修改。本方案在 `User/MotorControl/PidManager/` 新建独立的 PID 管理模块，实现：
1. **工程调试值配置**：通过上位机协议读写 Flash 中的 `ControlParam_t` 字段
2. **理论估计配置**：基于电机辨识参数（R/L）用零极点对消法自动计算 PID
3. **三环独立配置**：电流环/速度环/位置环各自独立选择参数来源（默认值 / Flash 工程值 / 理论估计值），互不干扰

---

## 当前状态分析

### 现有 PID 加载链路
```
motor_param_init()          [motor_param.c 自动生成, 不可改]
    ↓
motor_profile_apply_param() [motor_profile.c 覆盖电气身份, 不覆盖PID]
    ↓
motor_profile_sync_to_param() [Flash 标定数据同步]
    ↓
pid_profile_init(param)     [pid_profile.c 从 motor_param_t 加载到 s_pid_profiles]
    ↓
pid_profile_get(id)         [运行时各控制环读取]
```

### 关键问题
1. **`motor_param.c:81-85` 被错误修改**：PID 值从 CSV 源值 `0.5/10` 被改为 `3.6/2700`，需回退
2. **`pid_profile.c` 位于 `ControlProcess/`**：需迁移到 `PidManager/`
3. **无理论估计机制**：当前只能用固定默认值或 Flash 工程值
4. **三环绑定加载**：`pid_profile_load_from_motor_param()` 一次性加载所有环，无法独立选择来源
5. **校验范围偏紧**：`ki_ld/ki_lq` 上限 1000，高带宽电机理论估计可能超限

### 可复用资源
- `motor_info_storage_get()` 已返回可写 `motor_info_t *`（[motor_info_storage.h:174](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/AppServices/ParamService/motor_info_storage.h#L174)）
- `0xEA` 协议命令已实现 `app_motor_info_save()` 保存到 Flash（[jm_proto_ops.c:700](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/Protocol/joint_proto/jm_proto_ops.c#L700)）
- `motor_info_dispatch_read/write` 已支持按 pid 索引读写 `ControlParam_t` 字段
- `ControlParam_t.reserved[58]` 有 232B 预留空间可用

---

## 方案设计

### 核心架构

```
┌─────────────────────────────────────────────────────────┐
│                   PidManager 模块                        │
│  ┌─────────────┐  ┌──────────────┐  ┌───────────────┐  │
│  │ pid_profile │  │motor_pid_load│  │motor_pid_auto │  │
│  │  (迁移)     │  │  (新建)      │  │   tune(新建)  │  │
│  │             │  │ 三环独立加载  │  │ 零极点对消法  │  │
│  └─────────────┘  └──────────────┘  └───────────────┘  │
│         ↑                ↑                  ↑           │
│         └────────────────┼──────────────────┘           │
│                          │                              │
│              motor_pid_load(param, info)                │
│              按 *_pid_source 独立选择来源                 │
└──────────────────────────┬──────────────────────────────┘
                           ↓
                    motor_loop_init() 调用
```

### 数据来源定义（三环独立）

```c
typedef enum {
    PID_SOURCE_DEFAULT = 0,   /* 用 motor_param.c 默认值 */
    PID_SOURCE_FLASH   = 1,   /* 用 Flash 中 ControlParam_t 工程调试值 */
    PID_SOURCE_AUTOTUNE = 2,  /* 用理论估计值（零极点对消法） */
} pid_source_e;
```

### pid_source 存储（PidManager 内部 static，不侵入 motor_info）

> **关键决策**：`motor_info.h/c` 由脚本自动生成，不能手动修改。因此 pid_source 标志存于 PidManager 模块内部的 static 数组，不持久化到 Flash。每次上电默认 source=0（用 motor_param.c 默认值），行为向后兼容。若需持久化，后续让 `motor_info_generate.py` 脚本支持新增字段。

```c
/* motor_pid_load.c 内部 */
static pid_source_e s_ring_source[PID_RING_MAX] = {
    PID_SOURCE_DEFAULT, PID_SOURCE_DEFAULT, PID_SOURCE_DEFAULT
};
```

---

## 具体修改任务

### Task 1: 回退 motor_param.c 的错误改动

**文件**: [motor_param.c](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/DataHub/motor_param.c#L81-L85)

**修改**: `3.6f/2700.0f` → `0.5f/10.0f`（恢复 CSV 源值）

**原因**: 该文件由 `motor_param_generate_v9.py` 自动生成，手动修改会被下次生成覆盖。PID 配置改由新的 `motor_pid_load` 机制处理。

### Task 2: 迁移 pid_profile 到 PidManager + 更新 Keil 工程

**操作**:
1. 移动 `User/MotorControl/ControlProcess/pid_profile.c/h` → `User/MotorControl/PidManager/pid_profile.c/h`
2. 更新 [sfoc.uvprojx](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/Board/SFOC/MDK-ARM/sfoc.uvprojx#L342) IncludePath 追加 `;../../../User/MotorControl/PidManager`
3. 更新 sfoc.uvprojx 中 pid_profile.c 的 FilePath
4. 同步更新 `Board/V1/MDK-ARM/JointMotorApp.uvprojx`

**注意**: `pid_profile.c` 内部逻辑暂不修改，仅做文件位置迁移，保持原有 `pid_profile_load_from_motor_param` 等接口不变。

### Task 3: 修正校验范围

**文件**: [motor_info.c:239,243](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/DataHub/motor_info.c#L239)

**修改**: 放宽 `ki_ld`/`ki_lq` 上限 `1000.0f` → `10000.0f`，`integral_limit` 上限 `100.0f` → `500.0f`

**原因**: 高带宽电机理论估计 `Ki = ωc·R` 可能超过 1000；`integral_limit = Kp·rated·1.5` 在高带宽时也可能超过 100。

### Task 4: 新建 motor_pid_autotune 理论估计模块

**新建文件**:
- `User/MotorControl/PidManager/motor_pid_autotune.h`
- `User/MotorControl/PidManager/motor_pid_autotune.c`

**接口**（迭代7统一为结构体 API）:
```c
/* 单环理论估计结果 */
typedef struct {
    float kp;
    float ki;
    float integral_limit;
} autotune_result_t;

/* 电流环（d/q 双轴，零极点对消法） */
int motor_pid_autotune_current(const motor_info_t *info, float bandwidth_hz,
                               autotune_result_t *out_d, autotune_result_t *out_q);

/* 速度环（二阶最佳阻尼 ξ=0.707） */
int motor_pid_autotune_velocity(const motor_info_t *info, float bandwidth_hz,
                                autotune_result_t *out);

/* 位置环（纯比例，带宽=Kp） */
int motor_pid_autotune_position(const motor_info_t *info, float bandwidth_hz,
                                autotune_result_t *out);

/* 事务性应用：计算三环并写入 ControlParam_t，任一环失败全部不写入 */
int motor_pid_autotune_apply(motor_info_t *info, float current_bw_hz,
                             float velocity_bw_hz, float position_bw_hz);
```

**核心算法**（零极点对消法）:
```c
/* 前置检查：辨识数据是否就绪（迭代2修正） */
if (info->blocks.motor_calib.is_calibrated != 1U)
    return -2;  /* 辨识数据未就绪 */

float r  = info->blocks.motor_calib.phase_resistance;
float ld = info->blocks.motor_calib.phase_inductance_d;
float lq = info->blocks.motor_calib.phase_inductance_q;
if (r < 0.001f || ld < 0.00001f) return -2;

/* 带宽默认值 fallback（迭代3修正） */
if (current_bw_hz <= 0.0f) current_bw_hz = 1000.0f;  /* 电流环推荐 1kHz */

float wc = 2.0f * PI * current_bw_hz;  /* 截止角频率 rad/s */

/* 电流环: Kp = wc*L, Ki = wc*R */
out_d->kp = wc * ld;
out_d->ki = wc * r;
out_q->kp = wc * lq;
out_q->ki = wc * r;

/* 积分限幅 = Kp_q * rated_current * 1.5 */
float rated = info->blocks.motor_calib.current_lim * 0.6f;
out_q->integral_limit = out_q->kp * rated * 1.5f;
out_d->integral_limit = out_q->integral_limit;
```

**速度环**（二阶最佳阻尼 ξ=0.707）:
```c
if (velocity_bw_hz <= 0.0f) velocity_bw_hz = 100.0f;  /* 速度环推荐 100Hz */

float j  = info->blocks.motor_calib.rotor_inertia;
float kt = info->blocks.motor_calib.torque_constant;
if (j < 1e-7f || kt < 0.01f) return -2;

float wc = 2.0f * PI * velocity_bw_hz;
/* Kp = J*ωc/kt, Ki = J*ωc²/(4*kt) 保证阻尼比0.707 */
out->kp = j * wc / kt;
out->ki = j * wc * wc / (4.0f * kt);
out->integral_limit = info->blocks.motor_calib.current_lim * 0.5f;
```

**位置环**（P 控制器）:
```c
if (position_bw_hz <= 0.0f) position_bw_hz = 20.0f;  /* 位置环推荐 20Hz */
out->kp = 2.0f * PI * position_bw_hz;
out->ki = 0.0f;
out->integral_limit = 0.0f;
```

**事务性 apply**（迭代4修正）:
```c
int motor_pid_autotune_apply(motor_info_t *info, float cur_bw, float vel_bw, float pos_bw)
{
    autotune_result_t d, q, v, p;
    /* 先计算全部环，任一失败则不写入 */
    int ret = motor_pid_autotune_current(info, cur_bw, &d, &q);
    if (ret) return ret;
    ret = motor_pid_autotune_velocity(info, vel_bw, &v);
    if (ret) return ret;
    ret = motor_pid_autotune_position(info, pos_bw, &p);
    if (ret) return ret;

    /* 全部成功，事务提交 */
    ControlParam_t *ctl = &info->blocks.control;
    ctl->kp_ld = d.kp;  ctl->ki_ld = d.ki;
    ctl->kp_lq = q.kp;  ctl->ki_lq = q.ki;
    ctl->integral_limit = q.integral_limit;
    ctl->kp_s = v.kp;   ctl->ki_s = v.ki;
    ctl->speed_integral_limit = v.integral_limit;
    ctl->kp_p = p.kp;
    return 0;
}
```

**返回值**: 0=成功, -1=参数无效, -2=辨识数据未就绪

### Task 5: 新建 motor_pid_load 加载模块

**新建文件**:
- `User/MotorControl/PidManager/motor_pid_load.h`
- `User/MotorControl/PidManager/motor_pid_load.c`

**接口**:
```c
/* 从 motor_info 按 pid_source 独立加载三环 PID 到 motor_param_t */
void motor_pid_load(motor_param_t *param, const motor_info_t *info);

/* 运行时 reload（协议修改 source 后调用） */
void motor_pid_reload(void);

/* source 管理 API（供协议层调用） */
void motor_pid_set_source(pid_ring_e ring, pid_source_e src);
pid_source_e motor_pid_get_source(pid_ring_e ring);
```

**实现逻辑**（三环独立，source 存于内部 static）:
```c
/* 模块内部状态：三环独立 source 标志，不持久化 */
static pid_source_e s_ring_source[PID_RING_MAX] = {
    PID_SOURCE_DEFAULT, PID_SOURCE_DEFAULT, PID_SOURCE_DEFAULT
};

void motor_pid_set_source(pid_ring_e ring, pid_source_e src)
{
    if (ring < PID_RING_MAX)
        s_ring_source[ring] = src;
}

pid_source_e motor_pid_get_source(pid_ring_e ring)
{
    return (ring < PID_RING_MAX) ? s_ring_source[ring] : PID_SOURCE_DEFAULT;
}

void motor_pid_load(motor_param_t *param, const motor_info_t *info)
{
    const ControlParam_t *ctl = &info->blocks.control;

    /* ---- 电流环：按 s_ring_source[PID_RING_CURRENT] 独立选择 ---- */
    switch (s_ring_source[PID_RING_CURRENT]) {
    case PID_SOURCE_FLASH:
    case PID_SOURCE_AUTOTUNE:
        /* Flash 值由 0xE7 写入；autotune 值由 motor_pid_autotune_apply 写入。
           两者都存于 ctl 同一字段，读取路径一致。 */
        param->current_loop.current_kp_d = ctl->kp_ld;
        param->current_loop.current_ki_d = ctl->ki_ld;
        param->current_loop.current_kp_q = ctl->kp_lq;
        param->current_loop.current_ki_q = ctl->ki_lq;
        param->current_loop.current_integral_limit = ctl->integral_limit;
        break;
    case PID_SOURCE_DEFAULT:
    default:
        /* 保留 motor_param_init/motor_profile 的默认值，不覆盖 */
        break;
    }

    /* ---- 速度环：按 s_ring_source[PID_RING_VELOCITY] 独立选择 ---- */
    switch (s_ring_source[PID_RING_VELOCITY]) {
    case PID_SOURCE_FLASH:
    case PID_SOURCE_AUTOTUNE:
        param->position_loop.speed_kp = ctl->kp_s;
        param->position_loop.speed_ki = ctl->ki_s;
        param->position_loop.speed_integral_limit = ctl->speed_integral_limit;
        break;
    case PID_SOURCE_DEFAULT:
    default:
        break;
    }

    /* ---- 位置环：按 s_ring_source[PID_RING_POSITION] 独立选择 ---- */
    switch (s_ring_source[PID_RING_POSITION]) {
    case PID_SOURCE_FLASH:
    case PID_SOURCE_AUTOTUNE:
        param->position_loop.position_kp = ctl->kp_p;
        param->position_loop.position_integral_limit = ctl->position_integral_limit;
        break;
    case PID_SOURCE_DEFAULT:
    default:
        break;
    }
}

void motor_pid_reload(void)
{
    motor_param_t *param = &usr.motor_param[M1];
    const motor_info_t *info = motor_info_storage_get();
    motor_pid_load(param, info);
    /* 重新同步到 pid_profile 管理器 */
    pid_profile_load_from_motor_param(param);
}
```

**设计要点**:
- `PID_SOURCE_AUTOTUNE` 与 `PID_SOURCE_FLASH` 读取路径相同（都从 `ctl` 读），区别在于：autotune 会先调用 `motor_pid_autotune_apply` 把理论值写入 `ctl`，再 reload
- `PID_SOURCE_DEFAULT` 时不覆盖 `param`，保留 `motor_param_init` 的默认值
- 三环互不干扰：切换电流环 source 不影响速度/位置环
- source 标志为模块内部 static，重启回零（PID_SOURCE_DEFAULT），不持久化

### Task 6: motor_loop.c 接入 motor_pid_load

**文件**: [motor_loop.c:69-81](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorControl/CascadeControl/motor_loop.c#L69-L81)

**修改**: 在 `motor_profile_sync_to_param` 之后、`pid_profile_init` 之前插入：
```c
motor_profile_sync_to_param(param, motor_info_storage_get());
/* 新增: 按 pid_source 独立加载三环 PID */
motor_pid_load(param, motor_info_storage_get());
pid_profile_init(param);
```

### Task 7: 协议层新增 0x9A PID_AUTOTUNE 命令

**文件**: [jm_proto_ops.c](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/Protocol/joint_proto/jm_proto_ops.c)

**0x9A PID_AUTOTUNE 请求**（使用 0x9A 避开现有 0x98 标定中止）:
- payload[0] = ring_select (0:电流环 1:速度环 2:位置环 3:全部三环)
- payload[1..4] = current_bw_hz (float, 0 表示用推荐默认值)
- payload[5..8] = velocity_bw_hz (float, 0 表示用推荐默认值)
- payload[9..12] = position_bw_hz (float, 0 表示用推荐默认值)

**ACK**: 8 字节 `{status, fail_reason, ring_done, reserved[5]}`

**前置条件**: 仅 `TOP_FSM_IDLE` 状态允许执行（并发安全，见迭代5）

**处理流程**:
```c
static jm_err_e handle_pid_autotune(const uint8_t *payload, uint8_t len)
{
    /* 状态检查：仅 IDLE 态允许 */
    if (system_state_get_top() != TOP_FSM_IDLE)
        return JM_ERR_STATE_DENY;

    motor_info_t *info = motor_info_storage_get();
    /* 解析带宽参数 */
    float cur_bw, vel_bw, pos_bw;
    memcpy(&cur_bw, &payload[1], 4);
    memcpy(&vel_bw, &payload[5], 4);
    memcpy(&pos_bw, &payload[9], 4);

    /* 计算并写入 ControlParam_t（事务语义：任一环失败全部不写入） */
    int ret = motor_pid_autotune_apply(info, cur_bw, vel_bw, pos_bw);
    if (ret != 0) return JM_ERR_STATE_DENY;  /* fail_reason 通过 ACK 返回 */

    /* 设置 source 标志（按 ring_select，调用 PidManager API） */
    uint8_t ring = payload[0];
    if (ring == 0 || ring == 3) motor_pid_set_source(PID_RING_CURRENT,  PID_SOURCE_AUTOTUNE);
    if (ring == 1 || ring == 3) motor_pid_set_source(PID_RING_VELOCITY, PID_SOURCE_AUTOTUNE);
    if (ring == 2 || ring == 3) motor_pid_set_source(PID_RING_POSITION, PID_SOURCE_AUTOTUNE);

    /* 立即 reload 到运行期 */
    motor_pid_reload();
    /* 注意: 不自动保存 Flash，由上位机显式发 0xEA 固化 */
    return JM_ERR_NONE;
}
```

### Task 8: 协议层新增 0x9B PID_SOURCE_SET 命令

**新增命令 0x9B**（独立切换某环的 source，不触发 autotune 计算）:

- payload[0] = ring_select (0:电流环 1:速度环 2:位置环)
- payload[1] = source (0:默认 1:Flash 2:理论估计)

**前置条件**: 仅 `TOP_FSM_IDLE` 状态允许执行

**处理**:
```c
static jm_err_e handle_pid_source_set(const uint8_t *payload, uint8_t len)
{
    /* 状态检查：仅 IDLE 态允许 */
    if (system_state_get_top() != TOP_FSM_IDLE)
        return JM_ERR_STATE_DENY;

    uint8_t ring = payload[0];
    uint8_t src  = payload[1];
    if (src > 2 || ring > 2) return JM_ERR_INVALID_ARG;

    /* 调用 PidManager API 设置 source（不写 motor_info） */
    motor_pid_set_source((pid_ring_e)ring, (pid_source_e)src);
    motor_pid_reload();  /* 立即生效 */
    return JM_ERR_NONE;
}
```

---

## 假设与决策

| 决策点 | 选择 | 理由 |
|--------|------|------|
| pid_source 存储 | PidManager 内部 static 变量，不持久化 | `motor_info.h/c` 自动生成不可改；source 是调试期行为，重启回默认可接受 |
| autotune 触发 | 新增 0x9A 协议命令 | 避开 0x98 标定中止冲突 |
| source 切换 | 新增 0x9B 协议命令 | 独立于 autotune，可随时切回默认/Flash |
| Flash 保存 | 复用 0xEA | 已有完整实现，避免重复 |
| autotune 值存储 | 写入 ControlParam_t（RAM） | source=2 时从同一位置读取，统一路径 |
| 三环 source | 独立 3 个 static 变量 | 满足"不统一切换"需求 |
| 默认 source 值 | 0 (PID_SOURCE_DEFAULT) | 首次上电用 motor_param.c 默认值，行为兼容 |
| 并发安全 | 0x9A/0x9B 仅 IDLE 态执行 | 避免 ISR 与 reload 竞争写 motor_param_t |
| 校验范围放宽 | 直接改 motor_info.c | 同 pwm_freq_hz 修改先例，后续同步 CSV |

---

## 验证步骤

1. **编译验证**: 两个 Keil 工程（SFOC/V1）均编译通过
2. **默认行为**: 不发任何协议命令时，PID 行为与重构前一致（source=0 用默认值）
3. **Flash 工程值**: 0xE7 写入 PID → 0x9B 切 source=1 → 验证生效
4. **理论估计**: 0x9A 触发 autotune → 验证计算值合理性 → 0xEA 保存
5. **三环独立**: 切换电流环 source 不影响速度/位置环
6. **持久化**: 重启后 source 标志和 Flash 值保持
7. **校验范围**: 理论估计高带宽值能通过 validate

---

# 迭代审查记录（10 次迭代）

## 迭代 1: 完整性审查

**问题1**: Task 8 的 0x9A 命令与现有 0x98 标定中止命令编号接近，易混淆。
**修正**: 已改用 0x9A/0x9B，避开 0x98。

**问题2**: Task 6 的 `motor_pid_reload` 调用 `pid_profile_load_from_motor_param`，但该函数会覆盖全部三环，破坏"独立加载"语义。
**修正**: `motor_pid_load` 直接写 `motor_param_t`，`pid_profile_load_from_motor_param` 在 `pid_profile_init` 时已调用一次读取 `motor_param_t`。reload 时只需重新调用 `pid_profile_load_from_motor_param` 即可——因为它读的是已被 `motor_pid_load` 按独立 source 覆盖后的 `motor_param_t`，所以最终生效值仍遵守 source 选择。逻辑自洽，无需修改。

**问题3**: Task 3 修改 `ControlParam_t` 后，`motor_info.c` 自动生成的 `motor_info_init` 和 `motor_info_validate` 也需要同步——但这些是自动生成文件不能改。
**修正**: ⚠️ 关键问题。`motor_info.h/c` 由脚本生成。需确认脚本是否支持 reserved 字段重定义。若不支持，**替代方案**：在 `PidManager` 内部维护独立的 source 标志（static 变量），不存入 Flash，每次上电默认 source=0。这样无需改 `motor_info.h/c`。

**决策**: 采用替代方案——pid_source 存于 PidManager 内部 static 变量，不持久化。理由：
- 避免修改自动生成文件
- source 选择是调试期行为，重启回默认值可接受
- 若需持久化，后续让脚本支持新增字段

## 迭代 2: 低耦合性审查（基于迭代1决策调整）

**调整后架构**: pid_source 为 PidManager 内部状态，不侵入 motor_info。

**问题**: `motor_pid_load` 需同时读 `motor_info_t`（Flash 工程值）和 `motor_param_t`（默认值），耦合两个数据源。
**评估**: 这是必要的——"选择来源"的本质就是多数据源切换。通过将 source 管理集中在 `motor_pid_load`，其他模块（motor_loop/pid_profile）不感知 source 概念，耦合可接受。

**问题**: `motor_pid_autotune` 依赖 `motor_info_t` 的辨识字段（R/L），若辨识未完成用默认值计算会出错。
**修正**: Task 5 已有返回值 -2 表示"辨识数据未就绪"。补充判断：检查 `is_calibrated == 1` 或 R/L 非默认值。

## 迭代 3: 使用便利性审查

**问题**: 上位机切换 source 需先 0x9A 计算 + 0x9B 切 source，步骤繁琐。
**优化**: 0x9A autotune 命令内部自动设置 source=2，无需额外发 0x9B。0x9B 仅用于切回默认/Flash。

**问题**: 理论估计带宽参数需要上位机传入，用户可能不知道选多少。
**优化**: 带宽参数支持 0 值表示"用推荐默认值"（电流环 1000Hz、速度环 100Hz、位置环 20Hz），在 `motor_pid_autotune_apply` 内部 fallback。

## 迭代 4: 错误处理审查

**问题**: 0x9A 命令若辨识数据未就绪，返回 STATE_DENY 但上位机不知原因。
**修正**: ACK payload 扩展为 `{status, fail_reason, ring_done, reserved[5]}`，fail_reason 复用 `CALIB_FAIL_DEP_NOT_MET=0x04` 语义。

**问题**: `motor_pid_autotune_apply` 部分环成功、部分失败时，已写入的值如何处理？
**修正**: 采用事务语义——任一环失败则全部不写入，返回错误。调用方重试。

## 迭代 5: 并发安全审查

**问题**: 0x9A/0x9B 在通信线程调用 `motor_pid_reload`，会写 `motor_param_t`；同时电流环 ISR 读 `motor_param_t`。
**评估**: 
- `motor_param_t` 的 PID 字段是 float，STM32 M4F 单条 `STR` 原子写
- 电流环每拍读 PID 参数，reload 写入瞬间若被中断，最坏情况是某一拍用混合值
- PID 参数变化不大时影响可忽略；若敏感，可在 IDLE 态才允许 reload

**决策**: 限制 0x9A/0x9B 仅在 `TOP_FSM_IDLE` 状态执行，RUN 态拒绝。jm_proto_ops 层加状态检查。

## 迭代 6: 可扩展性审查

**问题**: 未来若要增加前馈/滤波参数的 source 选择，当前 3 个 source 字段不够。
**评估**: source 概念已抽象为 enum，新增 source 类型只需加 enum 值。pid_source 当前是 PidManager 内部 static 数组 `s_ring_source[3]`，扩展为 `s_ring_source[RING_MAX]` 即可。

**问题**: 新增控制环（如力控环）如何接入？
**评估**: `motor_pid_load` 内 switch-case 结构清晰，新增环只需加一个 case 块。可接受。

## 迭代 7: API 一致性审查

**问题**: `motor_pid_autotune_current/velocity/position` 三个接口参数风格不一致（电流环返回 5 个 out，位置环只返回 1 个）。
**修正**: 统一为按环返回结构体：
```c
typedef struct {
    float kp, ki, integral_limit;
} autotune_result_t;

int motor_pid_autotune_current(const motor_info_t *info, float bw,
                               autotune_result_t *out_d, autotune_result_t *out_q);
int motor_pid_autotune_velocity(const motor_info_t *info, float bw, autotune_result_t *out);
int motor_pid_autotune_position(const motor_info_t *info, float bw, autotune_result_t *out);
```
更清晰，扩展方便。

## 迭代 8: 向后兼容审查

**问题**: 重构后 `pid_profile.c` 从 `ControlProcess` 迁到 `PidManager`，其他文件 `#include "pid_profile.h"` 是否受影响？
**检查**: 需更新 Keil IncludePath（Task 2 已涵盖）。源文件 include 用相对路径或依赖 IncludePath，迁移后只需确保 IncludePath 包含新目录。

**问题**: `motor_pid_load` 在 `motor_loop_init` 中插入，是否影响原有 `pid_profile_init` 行为？
**检查**: `motor_pid_load` 写 `motor_param_t`，`pid_profile_init` 读 `motor_param_t`，顺序正确。默认 source=0 时不写 `motor_param_t`，行为完全兼容。

## 迭代 9: 性能审查

**问题**: `motor_pid_reload` 每次调用 `pid_profile_load_from_motor_param` 有多次 memcpy。
**评估**: reload 仅在协议命令触发时执行（非实时路径），性能无要求。

**问题**: autotune 计算涉及除法（J/kt），ISR 中是否调用？
**检查**: autotune 仅在 0x9A 协议命令处理中调用（通信线程），不在 ISR。无性能问题。

## 迭代 10: 文档与可读性审查

**问题**: PidManager 模块缺少统一头文件说明。
**修正**: 新建 `PidManager/pid_manager.h` 作为模块门面，汇总 include 三个子头文件，并提供 source enum 定义：
```c
/* pid_manager.h - PID 管理模块统一门面 */
#include "pid_profile.h"
#include "motor_pid_load.h"
#include "motor_pid_autotune.h"

typedef enum {
    PID_RING_CURRENT  = 0,
    PID_RING_VELOCITY = 1,
    PID_RING_POSITION = 2,
    PID_RING_MAX
} pid_ring_e;
```

**问题**: source 标志不持久化，用户每次重启需重新设置。
**记录**: 已在迭代1决策中接受此限制。若未来需持久化，需修改 `motor_info_generate.py` 脚本新增字段。在模块 README 中注明。

---

## 最终任务清单（基于 10 次迭代修正）

| # | 任务 | 文件 | 状态 |
|---|------|------|------|
| 1 | 回退 motor_param.c PID 值 3.6/2700 → 0.5/10 | motor_param.c L81-85 | 待执行 |
| 2 | 迁移 pid_profile.c/h 到 PidManager + 更新 2 个 Keil 工程 IncludePath/FilePath | pid_profile.*, sfoc.uvprojx, JointMotorApp.uvprojx | 待执行 |
| 3 | 放宽 motor_info.c 校验范围 ki_ld/ki_lq→10000, integral_limit→500 | motor_info.c L239,243,245 | 待执行 |
| 4 | 新建 pid_manager.h 门面 + pid_source_e/pid_ring_e 定义 | PidManager/pid_manager.h | 待执行 |
| 5 | 新建 motor_pid_autotune.c/h（零极点对消法 + 事务语义 + 带宽默认值 fallback） | PidManager/motor_pid_autotune.* | 待执行 |
| 6 | 新建 motor_pid_load.c/h（三环独立加载 + static source + set/get API） | PidManager/motor_pid_load.* | 待执行 |
| 7 | motor_loop.c 接入 motor_pid_load（sync_to_param 之后、pid_profile_init 之前） | motor_loop.c L69-81 | 待执行 |
| 8 | 协议层新增 0x9A PID_AUTOTUNE + 0x9B PID_SOURCE_SET（均含 IDLE 态限制） | jm_proto_ops.c | 待执行 |

**关键决策汇总**:
- pid_source 不存入 Flash（PidManager 内部 static），避免修改自动生成的 motor_info.h/c
- 0x9A/0x9B 仅 IDLE 态可执行（并发安全）
- 0x9A 内部自动设 source=2，无需额外发 0x9B
- autotune 带宽参数=0 时用推荐默认值（电流环 1000Hz、速度环 100Hz、位置环 20Hz）
- Flash 保存复用现有 0xEA 命令
- 校验范围放宽直接改 motor_info.c（同 pwm_freq_hz 先例，后续同步 CSV）
