# Task 8 实施方案：协议层新增 0x9A PID_AUTOTUNE + 0x9B PID_SOURCE_SET

## 摘要

PID 配置重构 Task 1-7 已完成（motor_param.c 回退、pid_profile 迁移、motor_info.c 校验放宽、pid_manager.h 门面、motor_pid_autotune.c/h、motor_pid_load.c/h、motor_loop.c 接入）。本方案完成最后的 Task 8：在协议层新增两条命令，使上位机可触发理论估计（0x9A）和独立切换三环参数来源（0x9B）。

---

## 当前状态分析

### 已就绪的 PidManager API

| 接口 | 文件 | 作用 |
|------|------|------|
| `motor_pid_autotune_apply(info, cur_bw, vel_bw, pos_bw)` | [motor_pid_autotune.c:123](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorControl/PidManager/motor_pid_autotune.c#L123) | 事务性计算三环 PID 并写入 ControlParam_t，返回 0=成功/-1=参数无效/-2=辨识未就绪 |
| `motor_pid_set_source(ring, src)` | [motor_pid_load.c:19](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorControl/PidManager/motor_pid_load.c#L19) | 设置指定环的 source（不持久化） |
| `motor_pid_get_source(ring)` | [motor_pid_load.c:25](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorControl/PidManager/motor_pid_load.c#L25) | 查询指定环的 source |
| `motor_pid_reload()` | [motor_pid_load.c:84](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorControl/PidManager/motor_pid_load.c#L84) | 按 source 重新加载到 motor_param_t + pid_profile |

### 协议层 dispatch 机制

[jm_proto.c:548](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/Protocol/joint_proto/jm_proto.c#L548) 的 `if (cmd <= JM_CMD_SINGLE_STEP)` 分支将 0x00~0xB8 范围命令统一交给 `ops->set_mode` 回调，仅返回简单 ACK(1字节) 或 NACK(3字节)。

**关键约束**：0x9A 需要返回 8 字节详细 ACK（含 fail_reason），简单 ACK 无法满足。0x97 CALIB_QUERY 已有先例——在 dispatch 中直接拦截并返回自定义 ACK。

### 状态访问方式

代码中无 `system_state_get_top()` 函数。实际访问路径：`motor_loop_get()->sys.top_state`（[motor_loop.h:75](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorControl/CascadeControl/motor_loop.h#L75) 提供 `motor_loop_get()`，[system_state.h:36](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/AppServices/StateMachine/system_state.h#L36) 定义 `top_state` 字段）。jm_proto_ops.c 已 include motor_loop.h。

---

## 设计决策

### 拦截方式：ops 回调（低耦合）

| 方案 | 优点 | 缺点 |
|------|------|------|
| A. 直接在 jm_proto.c 调用 PidManager（同 0x97 先例） | 改动少 | jm_proto.c 需引入 4 个新 include，协议核心层耦合业务 |
| **B. 新增 ops 回调（推荐）** | **协议核心只解析+格式化，业务逻辑全在 jm_proto_ops.c** | 需改 jm_proto.h 结构体 |

选择 **方案 B**：在 `jm_proto_ops_t` 新增两个回调，jm_proto.c 仅做载荷解析/ACK 格式化，业务逻辑集中在 jm_proto_ops.c。符合 [jm_proto.h:20](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/Protocol/joint_proto/jm_proto.h#L20) 的架构注释："业务动作...通过 jm_proto_ops_t 回调注入, 不在本层实现"。

### ACK 格式

- **0x9A**：8 字节 `{status, fail_reason, ring_select_done, reserved[5]}`（同 0x97 风格）
  - status: 0=成功, 1=失败
  - fail_reason: 0=无, 1=辨识数据未就绪, 2=非IDLE态, 3=参数无效
  - ring_select_done: 成功时回显 ring_select，失败时=0
- **0x9B**：简单 ACK（1 字节 status=0），走 dispatch 通用 ACK 路径

### 并发安全

0x9A/0x9B 均限 `TOP_FSM_IDLE` 态执行（迭代5决策），避免 ISR 与 reload 竞争写 motor_param_t。

---

## 具体修改

### 修改 1: jm_cmd_def.h 新增命令码

**文件**: [jm_cmd_def.h:110](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/Protocol/joint_proto/jm_cmd_def.h#L110)

在 `JM_CMD_CALIB_ABORT = 0x98` 之后新增：

```c
JM_CMD_CALIB_ABORT = 0x98,  /* 中止标定 */

/* PID 管理 0x9A~0x9B: 三环独立参数来源管理
 * 0x9A: 触发理论估计(零极点对消法)并自动设 source=2, 仅IDLE态
 * 0x9B: 独立切换某环 source, 仅IDLE态 */
JM_CMD_PID_AUTOTUNE    = 0x9A,  /* PID 理论估计 */
JM_CMD_PID_SOURCE_SET  = 0x9B,  /* PID 来源切换 */
```

### 修改 2: jm_proto.h 新增 ops 回调

**文件**: [jm_proto.h:136](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/Protocol/joint_proto/jm_proto.h#L136)（`set_baudrate` 之后）

```c
jm_err_e (*set_baudrate)(uint8_t baud_code);

/* PID 理论估计(CMD 0x9A): 基于辨识参数计算三环PID写入ControlParam_t,
 * 自动设 source=AUTOTUNE 并 reload。仅IDLE态可执行。
 * ring_select: 0=电流环 1=速度环 2=位置环 3=全部三环
 * cur_bw/vel_bw/pos_bw: 各环带宽Hz, <=0用推荐默认值
 * out_fail_reason: 失败原因输出(0=无,1=辨识未就绪,2=非IDLE态,3=参数无效)
 * 返回 JM_ERR_OK 成功, 其余失败。可为 NULL(回 NACK)。*/
jm_err_e (*pid_autotune)(uint8_t ring_select, float cur_bw, float vel_bw, float pos_bw,
                         uint8_t *out_fail_reason);

/* PID 来源切换(CMD 0x9B): 独立设置某环参数来源, 立即 reload。仅IDLE态可执行。
 * ring_select: 0=电流环 1=速度环 2=位置环
 * source: 0=默认 1=Flash工程值 2=理论估计
 * 返回 JM_ERR_OK 成功, 其余失败。可为 NULL(回 NACK)。*/
jm_err_e (*pid_source_set)(uint8_t ring_select, uint8_t source);
} jm_proto_ops_t;
```

### 修改 3: jm_proto.c 拦截 0x9A/0x9B

**文件**: [jm_proto.c:544](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/Protocol/joint_proto/jm_proto.c#L544)（0x97 CALIB_QUERY 拦截之后、`cmd <= JM_CMD_SINGLE_STEP` 之前）

```c
/* 标定进度查询 0x97: ... (已有代码不变) */

/* PID 理论估计 0x9A: 触发 autotune 计算 + 自动设 source=2 + reload。
 * ACK: 8字节 {status, fail_reason, ring_select_done, reserved[5]} */
if (cmd == JM_CMD_PID_AUTOTUNE)
{
    uint8_t ring_select;
    float cur_bw, vel_bw, pos_bw;
    uint8_t fail_reason = 0;
    jm_err_e e;

    if (len < 13)  /* ring(1) + 3*float(12) */
        return reply_nack(proto, cmd, JM_ERR_LENGTH);
    if (proto->ops == NULL || proto->ops->pid_autotune == NULL)
        return reply_nack(proto, cmd, JM_ERR_UNSUPPORTED);

    ring_select = payload[0];
    memcpy(&cur_bw, &payload[1], 4);
    memcpy(&vel_bw, &payload[5], 4);
    memcpy(&pos_bw, &payload[9], 4);

    e = proto->ops->pid_autotune(ring_select, cur_bw, vel_bw, pos_bw, &fail_reason);
    /* 0x9A 总是回 8字节 ACK(成功/失败均回), 返回 JM_ERR_OK 避免调用方覆盖 reply。
     * 成败信息编码在 body[0](status) 和 body[1](fail_reason) 中, 同 0x97 先例。*/
    {
        uint8_t body[8] = {(e == JM_ERR_OK) ? 0u : 1u, fail_reason,
                           (e == JM_ERR_OK) ? ring_select : 0u, 0, 0, 0, 0, 0};
        reply_set(proto, cmd, body, sizeof(body));
        return JM_ERR_OK;
    }
}

/* PID 来源切换 0x9B: 独立设置某环 source, 立即 reload。简单 ACK。*/
if (cmd == JM_CMD_PID_SOURCE_SET)
{
    uint8_t ring_select, source;
    jm_err_e e;

    if (len < 2)
        return reply_nack(proto, cmd, JM_ERR_LENGTH);
    if (proto->ops == NULL || proto->ops->pid_source_set == NULL)
        return reply_nack(proto, cmd, JM_ERR_UNSUPPORTED);

    ring_select = payload[0];
    source = payload[1];
    e = proto->ops->pid_source_set(ring_select, source);
    return (e == JM_ERR_OK) ? reply_ack(proto, cmd, 0) : reply_nack(proto, cmd, e);
}

/* 其余 0x00~0xB8 控制/校准/诊断类: 统一交给 set_mode 回调 ... (已有代码不变) */
```

**注意**: 0x9A 无论成功/失败都返回 8 字节 ACK（含 status 和 fail_reason），dispatch 返回 JM_ERR_OK 避免调用方覆盖 reply。上位机通过 body[0](status) 判断成败，body[1](fail_reason) 获取失败原因。0x9B 走标准 ACK/NACK 路径。

### 修改 4: jm_proto_ops.c 实现回调

**文件**: [jm_proto_ops.c](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/Protocol/joint_proto/jm_proto_ops.c)

**4a. 新增 include**（在现有 include 区域，约 L38 之后）:

```c
#include "motor_pid_autotune.h"  /* motor_pid_autotune_apply */
#include "motor_pid_load.h"      /* motor_pid_set_source / motor_pid_reload */
```

jm_proto_ops.c 已 include `motor_loop.h`（L35）、`motor_info_storage.h`（L37），无需新增。

**4b. 新增 app_pid_autotune 函数**（在 `app_set_mode` 之后，约 L200 处）:

```c
/* ============================================================================
 *  PID 理论估计: CMD 0x9A  ->  pid_autotune
 *     基于辨识参数(R/L)用零极点对消法计算三环PID, 写入 ControlParam_t,
 *     按 ring_select 自动设 source=AUTOTUNE 并 reload。仅 IDLE 态可执行。
 * ==========================================================================*/
static jm_err_e app_pid_autotune(uint8_t ring_select, float cur_bw, float vel_bw, float pos_bw,
                                  uint8_t *out_fail_reason)
{
    if (out_fail_reason != NULL)
        *out_fail_reason = 0;

    /* 状态检查: 仅 IDLE 态允许(并发安全) */
    if (motor_loop_get()->sys.top_state != TOP_FSM_IDLE)
    {
        if (out_fail_reason != NULL)
            *out_fail_reason = 2;  /* 非 IDLE 态 */
        return JM_ERR_STATE_DENY;
    }

    /* ring_select 范围检查: 0~3 */
    if (ring_select > 3)
    {
        if (out_fail_reason != NULL)
            *out_fail_reason = 3;  /* 参数无效 */
        return JM_ERR_OUT_OF_RANGE;
    }

    /* 事务性计算并写入 ControlParam_t */
    motor_info_t *info = motor_info_storage_get();
    int ret = motor_pid_autotune_apply(info, cur_bw, vel_bw, pos_bw);
    if (ret != 0)
    {
        if (out_fail_reason != NULL)
            *out_fail_reason = (ret == -2) ? 1 : 3;  /* 1=辨识未就绪, 3=参数无效 */
        return JM_ERR_STATE_DENY;
    }

    /* 按 ring_select 自动设 source=AUTOTUNE */
    if (ring_select == 0 || ring_select == 3)
        motor_pid_set_source(PID_RING_CURRENT, PID_SOURCE_AUTOTUNE);
    if (ring_select == 1 || ring_select == 3)
        motor_pid_set_source(PID_RING_VELOCITY, PID_SOURCE_AUTOTUNE);
    if (ring_select == 2 || ring_select == 3)
        motor_pid_set_source(PID_RING_POSITION, PID_SOURCE_AUTOTUNE);

    /* 立即 reload 到运行期 */
    motor_pid_reload();

    /* 不自动保存 Flash, 由上位机显式发 0xEA 固化 */
    return JM_ERR_OK;
}
```

**4c. 新增 app_pid_source_set 函数**（紧接 app_pid_autotune 之后）:

```c
/* ============================================================================
 *  PID 来源切换: CMD 0x9B  ->  pid_source_set
 *     独立设置某环参数来源(默认/Flash/理论估计), 立即 reload。仅 IDLE 态可执行。
 * ==========================================================================*/
static jm_err_e app_pid_source_set(uint8_t ring_select, uint8_t source)
{
    /* 状态检查: 仅 IDLE 态允许 */
    if (motor_loop_get()->sys.top_state != TOP_FSM_IDLE)
        return JM_ERR_STATE_DENY;

    /* 参数范围检查 */
    if (ring_select > 2 || source > 2)
        return JM_ERR_OUT_OF_RANGE;

    /* 调用 PidManager API 设置 source(不写 motor_info) */
    motor_pid_set_source((pid_ring_e)ring_select, (pid_source_e)source);

    /* 立即 reload 生效 */
    motor_pid_reload();

    return JM_ERR_OK;
}
```

**4d. 注册到 s_app_ops 单例**（[jm_proto_ops.c:810](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/Protocol/joint_proto/jm_proto_ops.c#L810)）:

```c
static const jm_proto_ops_t s_app_ops = {
    .set_mode = app_set_mode,
    /* ... 已有字段不变 ... */
    .motor_info_reset = app_motor_info_reset,
    /* PID 管理 0x9A~0x9B */
    .pid_autotune   = app_pid_autotune,
    .pid_source_set = app_pid_source_set,
};
```

---

## 假设与决策

| 决策点 | 选择 | 理由 |
|--------|------|------|
| 拦截方式 | ops 回调 | 协议核心不耦合业务，符合 jm_proto.h 架构注释 |
| 0x9A ACK 格式 | 8 字节(含 fail_reason) | 迭代4决策：上位机需知失败原因 |
| 0x9A 失败时返回 | 8 字节 ACK(status=1), dispatch 返回 JM_ERR_OK | 避免调用方覆盖 reply；上位机通过 body[0]/[1] 判断成败 |
| 0x9B ACK 格式 | 简单 ACK/NACK | 仅切换 source，无需详细状态 |
| 状态访问 | `motor_loop_get()->sys.top_state` | 代码中无 `system_state_get_top()`，此为实际访问路径 |
| ring_select 编码 | 0/1/2=单环, 3=全部 | 同计划约定，0x9B 不支持 3(仅单环切换) |
| 0x9A 是否自动保存 Flash | 否 | 复用 0xEA 显式保存，避免误固化 |
| 错误码映射 | -2→辨识未就绪(fail_reason=1), -1→参数无效(fail_reason=3) | 区分失败原因 |

---

## 验证步骤

1. **编译验证**: 两个 Keil 工程（SFOC/V1）均编译通过，无 warning
2. **0x9B 基本功能**: IDLE 态发 0x9B{ring=0, src=1} → ACK → 验证电流环 PID 来自 Flash
3. **0x9B 三环独立**: 切换电流环 source 不影响速度/位置环（读取 motor_param 验证）
4. **0x9A 前置检查**: 未标定时发 0x9A → 8字节 ACK{1, 1, 0, ...}（辨识未就绪）
5. **0x9A 成功路径**: 标定完成后发 0x9A{ring=3, bw=0,0,0} → 8字节 ACK{0, 0, 3, ...} → 验证 ControlParam_t 已写入理论值 → 验证 motor_param_t 已 reload
6. **0x9A 状态限制**: RUN 态发 0x9A → 8字节 ACK{1, 2, 0, ...}（非IDLE态）
7. **0x9B 状态限制**: RUN 态发 0x9B → NACK(0x03 STATE_DENY)
8. **0x9B 参数越界**: 发 0x9B{ring=5, src=0} → NACK(0x02 OUT_OF_RANGE)
9. **Flash 固化**: 0x9A 成功后发 0xEA → 重启 → 验证 Flash 中理论值保持（source 回默认=0，需重新 0x9B 切换）
