# PID 命令码迁移 + 调试模式方案

## 摘要

1. **命令码迁移**：0x9A/0x9B/0x9C → 0xA0/0xA1/0xA2，0xA0~0xAF 专属 PID 段
2. **新增 DEBUG source**（值=3）：调试模式下下位机直接使用 0xA5 下发的参数，不被 reload 覆盖
3. **新增 0xA5（写PID参数）/ 0xA6（读PID参数）**：按环+参数类型读写，单字段 4 字节，实时生效

## 命令码分配（0xA0~0xAF 专属 PID）

| 命令码 | 名称 | 功能 | 原命令码 |
|--------|------|------|----------|
| 0xA0 | PID理论估计 | 触发autotune计算+设source=AUTOTUNE | 0x9A 迁移 |
| 0xA1 | PID来源切换 | 独立切换某环source（新增DEBUG=3） | 0x9B 迁移 |
| 0xA2 | 读PID来源 | 返回三环当前source | 0x9C 迁移 |
| 0xA3 | 保留 | — | — |
| 0xA4 | 保留 | — | — |
| 0xA5 | 写PID参数 | 实时下发单字段PID参数 | 新增 |
| 0xA6 | 读PID参数 | 读取单字段PID参数 | 新增 |
| 0xA7~0xAF | 保留 | — | — |

## DEBUG source 设计

### source 枚举扩展

```c
// motor_pid_load.h
typedef enum
{
    PID_SOURCE_DEFAULT  = 0,  /* motor_param.c 默认值 */
    PID_SOURCE_FLASH    = 1,  /* Flash ControlParam 工程值 */
    PID_SOURCE_AUTOTUNE = 2,  /* 理论估计值 */
    PID_SOURCE_DEBUG    = 3,  /* 调试值：直接用 0xA5 下发的参数 */
} pid_source_e;
```

### DEBUG 模式核心机制

```
正常模式 (DEFAULT/FLASH/AUTOTUNE):
  motor_pid_load() → 按 source 从对应数据源覆盖 motor_param_t
  motor_pid_profile_load_from_motor_param() → 同步到 s_motor_pid_profiles[]
  ISR 从 s_motor_pid_profiles[] 读取参数

DEBUG 模式:
  motor_pid_load() → case PID_SOURCE_DEBUG: break; (不覆盖 motor_param_t)
  0xA5 写入 → 直接写 s_motor_pid_profiles[ring].param.xxx (ISR 实际读取的数据源)
  ISR 从 s_motor_pid_profiles[] 读取参数 (读到的就是 0xA5 写入的值)
```

**关键点**：0xA5 直接写 `s_motor_pid_profiles[]`（控制环 ISR 实际读取的数据源），而非 `motor_param_t`。原因：
1. `motor_param_t` 中没有 `kd/output_limit/output_filter_alpha/flags` 字段，可调参数不全
2. `s_motor_pid_profiles[]` 包含完整 `pid_param_t`（kp/ki/kd/output_limit/integral_limit/output_filter_alpha/flags）
3. DEBUG 模式下 `motor_pid_load` 不覆盖，`motor_pid_profile_load_from_motor_param` 不调用，0xA5 写入的值不会被冲掉
4. 写入即生效（ISR 下一拍就用新值），无需 reload

### DEBUG 不持久化

- `pid_source_mask`（Flash Index=127）中**不存 DEBUG 值**
- 0xA1 切到 DEBUG 时只改 `s_ring_source[]`（RAM），不写 `pid_source_mask`
- 重启后自动回 DEFAULT/FLASH/AUTOTUNE（由 `motor_pid_load_boot` + `motor_pid_load_source_from_flash` 决定）
- `pid_source_from_mask` 的上界校验从 `<= AUTOTUNE` 放宽到 `<= DEBUG`（防止读出脏值异常）

## 0xA5 / 0xA6 命令设计

### 载荷格式

```
0xA5 (写PID参数):
  payload[0] = ring        (0=D轴电流环 1=Q轴电流环 2=速度环 3=位置环)
  payload[1] = param_type  (1=kp 2=ki 3=kd 4=output_limit 5=integral_limit 6=output_filter_alpha 7=flags)
  payload[2..5] = value    (4字节, float 或 uint32_t 按 param_type 解释)

0xA6 (读PID参数):
  请求: payload[0] = ring, payload[1] = param_type
  应答: payload[0] = ring, payload[1] = param_type, payload[2..5] = value (4字节)
```

### ring 映射（调试环编号，与 pid_ring_e 不同）

| ring 值 | 含义 | 对应 profile_id | 对应 source 环 |
|---------|------|----------------|---------------|
| 0 | 电流环 D 轴 | MOTOR_PID_PROFILE_CURRENT_D (0) | PID_RING_CURRENT |
| 1 | 电流环 Q 轴 | MOTOR_PID_PROFILE_CURRENT_Q (1) | PID_RING_CURRENT |
| 2 | 速度环 | MOTOR_PID_PROFILE_VELOCITY (2) | PID_RING_VELOCITY |
| 3 | 位置环 | MOTOR_PID_PROFILE_POSITION (3) | PID_RING_POSITION |

> 注：ring 0/1 都映射到 `PID_RING_CURRENT`（电流环整体进 DEBUG），但 0xA5 可分别调 D/Q 轴参数。

### param_type 映射

| param_type | 字段 | 数据类型 | 说明 |
|-----------|------|---------|------|
| 1 | kp | float | 比例增益 |
| 2 | ki | float | 积分增益 |
| 3 | kd | float | 微分增益 |
| 4 | output_limit | float | 输出限幅 |
| 5 | integral_limit | float | 积分限幅 |
| 6 | output_filter_alpha | float | 输出滤波系数 |
| 7 | flags | uint32_t | PID标志位（ANTI_WINDUP等） |

### 状态约束

- **0xA5（写）**：要求对应环 source=DEBUG，否则 NACK(STATE_DENY)
  - ring 0/1 → 检查 `motor_pid_get_source(PID_RING_CURRENT) == PID_SOURCE_DEBUG`
  - ring 2 → 检查 `PID_RING_VELOCITY`
  - ring 3 → 检查 `PID_RING_POSITION`
  - 不限制 top_fsm（允许运行态实时调参，这才是"调试"的意义）
- **0xA6（读）**：无状态限制，随时可读（读当前 profile 中的值）

### 0xA1（来源切换）扩展

- `source` 范围从 0~2 放宽到 0~3
- source=3(DEBUG) 时**不写** `pid_source_mask`（不持久化）
- source=0/1/2 时正常写 `pid_source_mask`（持久化）

## 变更清单

### 变更 1：命令码迁移 0x9A/9B/9C → 0xA0/A1/A2

**下位机**：
- `jm_cmd_def.h`：`JM_CMD_PID_AUTOTUNE = 0xA0`，`JM_CMD_PID_SOURCE_SET = 0xA1`，`JM_CMD_PID_SOURCE_GET = 0xA2`
- `jm_proto.c`：dispatch 分支中所有 `JM_CMD_PID_AUTOTUNE/SOURCE_SET/SOURCE_GET` 引用自动跟随枚举
- `jm_proto_ops.c`：0x9A/9B/9C 的 dispatch 分支判断自动跟随枚举
- CSV（下位机 + 上位机）：0x9A→0xA0, 0x9B→0xA1, 0x9C→0xA2

**上位机**：
- `cmd_def.py`：`PID_AUTOTUNE = 0xA0`，`PID_SOURCE_SET = 0xA1`，`PID_SOURCE_GET = 0xA2`
- `motor_client.py`：`_on_frame` 中 0x9A/9C 特判分支自动跟随枚举
- `pid_panel.py`：`on_ack`/`on_nack` 中 `JmCmd.PID_SOURCE_SET` 引用自动跟随
- `main_window.py`：`_on_pid_autotune_result`/`_on_pid_source_received` 引用自动跟随

### 变更 2：DEBUG source 枚举 + 位域校验

**文件**：`motor_pid_load.h`

- 枚举新增 `PID_SOURCE_DEBUG = 3`
- `pid_source_from_mask` 上界校验：`s <= PID_SOURCE_AUTOTUNE` → `s <= PID_SOURCE_DEBUG`

### 变更 3：motor_pid_load 新增 DEBUG 分支

**文件**：`motor_pid_load.c` 的 `motor_pid_load()` 函数

三环 switch 各新增：
```c
case PID_SOURCE_DEBUG:
    /* 不覆盖 motor_param_t，保留 0xA5 直接写入 s_motor_pid_profiles 的值 */
    break;
```

### 变更 4：0xA1(来源切换) 支持 DEBUG

**文件**：`jm_proto_ops.c` 的 `app_pid_source_set()`

- `source > 2` 校验放宽为 `source > 3`（或 `source > PID_SOURCE_DEBUG`）
- source=DEBUG 时不写 `pid_source_mask`：
```c
if (source != PID_SOURCE_DEBUG) {
    /* 写入 pid_source_mask (持久化) */
    mask = pid_source_to_mask(mask, ring, src);
    info->blocks.control.pid_source_mask = mask;
}
```

### 变更 5：新增 0xA5/0xA6 协议命令

**下位机**：

- `jm_cmd_def.h`：新增 `JM_CMD_PID_PARAM_SET = 0xA5`，`JM_CMD_PID_PARAM_GET = 0xA6`
- `jm_proto.h`：ops 新增两个函数指针：
  ```c
  jm_err_e (*pid_param_set)(uint8_t ring, uint8_t param_type, const uint8_t *value4);
  jm_err_e (*pid_param_get)(uint8_t ring, uint8_t param_type, uint8_t *out_value4);
  ```
- `jm_proto.c`：dispatch 新增 0xA5/0xA6 分支
  - 0xA5：解析 ring/param_type/value4，调 ops->pid_param_set，回 ACK{status:u8}
  - 0xA6：解析 ring/param_type，调 ops->pid_param_get，回 {ring:u8;param_type:u8;value:4B}
- `jm_proto_ops.c`：实现 `app_pid_param_set` / `app_pid_param_get`
  - `app_pid_param_set`：检查对应环 source==DEBUG → 调 `motor_pid_profile_set_param` → 回 OK
  - `app_pid_param_get`：调 `motor_pid_profile_get_param` → 回 4 字节值

**上位机**：
- `cmd_def.py`：`PID_PARAM_SET = 0xA5`，`PID_PARAM_GET = 0xA6`
- `motor_client.py`：新增 `pid_param_set(ring, param_type, value)` / `pid_param_get(ring, param_type)` + 信号
- `pid_panel.py`：调试区 UI（环选择 + 参数类型 + 值输入 + 读/写按钮）

### 变更 6：motor_pid_profile 新增单字段读写 API

**文件**：`motor_pid_profile.h/c`

新增：
```c
/* 按 profile_id + param_type 读写单个参数 (0xA5/0xA6 调用) */
int motor_pid_profile_set_param(uint8_t profile_id, uint8_t param_type, const uint8_t *value4);
int motor_pid_profile_get_param(uint8_t profile_id, uint8_t param_type, uint8_t *out_value4);
```

实现按 param_type switch 读写 `s_motor_pid_profiles[profile_id].param.kp/ki/kd/...`，float 用 memcpy，flags 直接拷贝。

### 变更 7：CSV 同步

- 下位机 CSV：0x9A/9B/9C 改为 0xA0/0xA1/0xA2，新增 0xA5/0xA6 行
- 上位机 CSV：同步

## 数据流图

```
                         ┌─────────────────────────────────┐
                         │         Flash (ControlParam_t)    │
                         │  kp_ld/ki_ld/kp_lq/ki_lq/...      │
                         │  pid_source_mask (Index=127)      │
                         └──────────┬──────────────────────┘
                                    │ 上电加载
                         ┌──────────▼──────────────────────┐
                         │      motor_info_t (RAM)          │
                         │  blocks.control.*                │
                         └──────────┬──────────────────────┘
                                    │ motor_pid_load_boot / motor_pid_load
                         ┌──────────▼──────────────────────┐
                         │      motor_param_t (运行期)       │
                         │  current_kp_d / speed_kp / ...   │
                         └──────────┬──────────────────────┘
                                    │ motor_pid_profile_load_from_motor_param
                                    │ (DEBUG模式跳过此步)
                         ┌──────────▼──────────────────────┐
  0xA5 写 ──────────────►│   s_motor_pid_profiles[] (ISR源)  │◄─── ISR 读取
  (仅DEBUG模式允许)       │  [0].param.kp/ki/kd/... (D轴)    │
                         │  [1].param.kp/ki/kd/... (Q轴)    │
                         │  [2].param.kp/ki/kd/... (速度)   │
                         │  [3].param.kp/ki/kd/... (位置)   │
                         └─────────────────────────────────┘
                                    │
  0xA6 读 ──────────────►│ (直接从此处读)
  (随时可读)              │
```

## source 切换与 DEBUG 交互时序

```
正常使用:
  1. 上电 → boot 三级回退 → source=FLASH/AUTOTUNE/DEFAULT
  2. 0xA1 切 source=FLASH → motor_pid_reload → profile 从 motor_param_t 同步
  3. 0xEA 保存 → pid_source_mask 固化到 Flash

调试使用:
  1. 0xA1 切 source=DEBUG → s_ring_source=DEBUG (不写mask) → motor_pid_reload
     → motor_pid_load case DEBUG: break (不覆盖)
     → motor_pid_profile_load_from_motor_param 仍执行 (从 motor_param_t 同步当前值作为起点)
  2. 0xA5 写 D轴kp=8.0 → 直接写 profiles[0].param.kp=8.0 → ISR 下一拍生效
  3. 0xA5 写 Q轴ki=2000 → 直接写 profiles[1].param.ki=2000 → ISR 下一拍生效
  4. 0xA6 读 D轴kp → 返回 8.0
  5. 调参满意后:
     a. 0xA1 切回 source=FLASH → motor_pid_reload → profile 从 Flash 恢复 (调试值丢失)
     b. 如需保存调试值: 先 0xE7 写入 ControlParam_t → 0xEA 固化 → 再切 source=FLASH
  6. 断电重启 → DEBUG 自动消失 (mask未存DEBUG) → 回 DEFAULT/FLASH/AUTOTUNE
```

## 验证

1. 命令码迁移后 0xA0/A1/A2 功能与原 0x9A/9B/9C 完全一致
2. 0xA1 切 DEBUG 后 0xA2 读取返回 source=3
3. DEBUG 模式下 0xA5 写入参数后电机响应立即变化
4. DEBUG 模式下 0xA6 读取返回 0xA5 写入的值
5. 非 DEBUG 模式下 0xA5 写入返回 NACK(STATE_DENY)
6. 断电重启后 source 不为 DEBUG
7. 0xA1 切 DEBUG 后 pid_source_mask 未改变（不持久化）
