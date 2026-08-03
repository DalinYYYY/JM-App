# CAN-DI 同 ID 设备发现与改址方案

## 1. 文档信息

| 项目 | 内容 |
|---|---|
| 文档状态 | 已实现，待真机总线验收 |
| 创建日期 | 2026-08-03 |
| 适用范围 | 关节电机下位机、PyQt 上位机、经典 CAN/CAN FD |
| 主要目标 | 在不改变现有 CAN ID 编码规范的前提下，发现并分别配置同一总线上 CAN ID 相同的设备 |

本文档既是协议定义，也是当前实现的维护说明。本文定义的 CAN-DI 仅用于设备发现、物理识别和 CAN ID 配置，不替代设备完整 UUID。

完整 96-bit UUID 继续通过现有 `READ_DEV_INFO(0xD0)` 命令读取。

---

## 2. 背景与问题

当前 CAN 扩展帧 ID 使用以下固定格式：

```text
CAN_ID = (CMD << 8) | MotorID
```

- `CMD` 位于 CAN ID 的 `[15:8]`。
- `MotorID` 位于 CAN ID 的 `[7:0]`。
- `MotorID=0` 为广播地址。
- 有效设备地址为 `1~127`。

当前上位机通过依次向节点 `1~127` 发送 `READ_DEV_INFO(0xD0)` 进行扫描。若同一总线上存在多台相同 MotorID 的设备，它们会使用相同仲裁 ID 返回多帧设备信息。

由此产生两类问题：

1. 多台设备同时使用相同仲裁 ID、发送不同数据时，CAN 数据段发生冲突，产生错误帧和重发。
2. 多台设备错峰发送合法分片时，上位机多帧重组器仍会按相同 CAN ID 将不同设备的分片组合，导致序号冲突、CRC 失败或数据内容杂糅。

因此，同 ID 设备的初始发现不能继续使用多帧 `READ_DEV_INFO`，必须使用不需要分片重组的单帧发现协议。

---

## 3. 设计目标

### 3.1 必须实现

1. 保持现有 `CAN_ID=(CMD<<8)|MotorID` 规范不变。
2. 保持现有正常控制命令和 DBC 解析方式不变。
3. 多台设备 MotorID 相同时，上位机能够显示每一台设备。
4. 发现响应必须为经典 CAN 单帧，不使用多帧重组。
5. 上位机能够按设备标识定向修改其中一台设备的 MotorID。
6. 设置完成后新 MotorID 立即生效，无需人工断电重启。
7. 设置完成后仍可通过 `0xD0` 读取完整 UUID、硬件版本和固件版本。
8. Flash 写入失败时不得切换运行时 MotorID。
9. SYNC、ESTOP 和现有控制协议行为不得受到影响。

### 3.2 不在 CAN-DI 中实现

1. CAN-DI 不承担完整 UUID 传输。
2. CAN-DI 不作为加密密钥或安全认证凭据。
3. CAN-DI 不替代现有 `READ_DEV_INFO(0xD0)`。
4. 本方案不改变 CAN 仲裁 ID 的字段布局。

---

## 4. 名词定义

| 名称 | 含义 |
|---|---|
| UUID | MCU 提供的完整 96-bit 唯一 ID，共 12 字节 |
| CAN-DI | CAN Device Identifier，面向发现和改址的压缩设备标识 |
| CAN_DI56 | 设置命令使用的 56-bit 设备选择标识，共 7 字节 |
| CAN_DI_GUARD | 扫描阶段使用的 8-bit 碰撞辅助检测值 |
| Fingerprint64 | `CAN_DI56 + CAN_DI_GUARD`，上位机扫描列表的设备键 |
| 扫描轮次 | 上位机发送一次发现广播，并在规定时间窗内收集响应的过程 |
| 时隙 | 同 ID 设备错开发送发现响应的时间窗口 |

---

## 5. CAN-DI 生成规则

### 5.1 UUID 字段来源

当前固件通过 `HAL_GetUIDw0()`、`HAL_GetUIDw1()` 和 `HAL_GetUIDw2()` 读取 96-bit UID，并按小端序保存为连续 12 字节：

```text
UID[0..3]   = WORD0，晶圆 X/Y 坐标
UID[4]      = WORD1[7:0]，晶圆编号 WAF_NUM
UID[5..11]  = LOT 批号
```

CAN-DI 计算直接使用上述 12 字节的原始存储顺序，不做 BCD 解码，不重新排列 WORD，不依赖 CPU 对整数的隐式字节序转换。

### 5.2 CAN_DI56

```text
CAN_DI56[0..3] = UID[0..3]
CAN_DI56[4]    = UID[4]
lot_crc        = CRC16_XMODEM(UID[5..11])
CAN_DI56[5]    = lot_crc & 0xFF
CAN_DI56[6]    = lot_crc >> 8
```

当前工程已有 `crc16_calc()`，算法为 CCITT/XMODEM、初值 `0x0000`。实现时必须复用该函数，禁止再引入另一套同名 CRC16 变体。

### 5.3 CAN_DI_GUARD

```text
uid_crc      = CRC16_XMODEM(UID[0..11])
CAN_DI_GUARD = (uid_crc >> 8) & 0xFF
```

`CAN_DI_GUARD` 不参与设置命令匹配，只用于上位机发现 `CAN_DI56` 的理论碰撞：

```text
CAN_DI56 相同且 GUARD 不同 -> 明确判定为 DI 冲突，禁止设置
```

### 5.4 性质和限制

- 同批次、同晶圆设备依靠 X/Y 坐标区分。
- 同批次、不同晶圆设备依靠 WAF_NUM 区分。
- 不同批次设备依靠 LOT 的 CRC16 区分。
- 芯片厂商只保证完整 96-bit UUID 唯一，不保证压缩后的 CAN-DI 绝对唯一。
- CAN-DI 是不可逆指纹，不能由 CAN-DI 还原完整 UUID。
- 上位机发现 DI 冲突时必须停止该冲突组的远程设置，改用物理隔离或后续扩展协议处理。

当前唯一实现入口如下，其他模块不得重复实现另一套压缩算法：

```c
void jm_proto_can_di_build(const uint8_t uid[12],
                           uint8_t di56[7],
                           uint8_t *guard);
```

---

## 6. 命令分配

以下命令已经完成全局占用检查，并同步到命令枚举、命令 CSV、DBC 和上位机枚举。

| CMD | 建议名称 | 方向 | 用途 |
|---:|---|---|---|
| `0xF4` | `JM_CMD_CAN_DI_DISCOVER` | Host -> All / Motor -> Host | 广播发现及单帧响应 |
| `0xF5` | `JM_CMD_CAN_DI_SET_ID` | Host -> All / Motor -> Host | 按 CAN-DI 设置 MotorID 及应答 |
| `0xF6` | `JM_CMD_CAN_DI_IDENTIFY` | Host -> All / Motor -> Host | 按 CAN-DI 触发物理设备指示 |

现有命令保持不变：

- `0xD0 READ_DEV_INFO`：读取完整 UUID 和版本。
- `0xF0 SET_CAN_ID`：对已具有唯一 MotorID 的设备设置地址，保持兼容。
- `0xF2 BROADCAST_SYNC`：保持原行为。
- `0xF3 SET_FD_MODE`：保持原行为。

三个新命令均使用普通单帧 ID，不设置 `JM_CAN_MULTI_FLAG`。

---

## 7. 单帧发现协议

### 7.1 发现请求

```text
CAN ID = (0xF4 << 8) | 0x00 = 0xF400
IDE    = Extended
RTR    = Data
DLC    = 8
```

| Byte | 字段 | 类型 | 说明 |
|---:|---|---|---|
| 0 | `protocol_version` | `u8` | 首版固定为 `1` |
| 1 | `round_id` | `u8` | 上位机扫描轮次，循环递增 |
| 2 | `slot_exp` | `u8` | 时隙数量为 `1 << slot_exp`，默认 `8` 即 256 |
| 3 | `slot_ms` | `u8` | 每时隙毫秒数，默认 `2` |
| 4~7 | `nonce` | `u32 LE` | 每轮随机数，禁止连续轮次复用 |

下位机参数校验：

```text
protocol_version == 1
slot_exp          范围 6~10，即 64~1024 时隙
slot_ms           范围 1~10 ms
```

参数非法时设备静默，不允许所有设备同时返回 NACK。

### 7.2 发现响应

```text
CAN ID = (0xF4 << 8) | current_motor_id
IDE    = Extended
RTR    = Data
DLC    = 8
```

| Byte | 字段 | 类型 | 说明 |
|---:|---|---|---|
| 0~6 | `CAN_DI56` | `bytes[7]` | 设备选择标识 |
| 7 | `CAN_DI_GUARD` | `u8` | 压缩碰撞辅助检测值 |

当前 MotorID 已包含在响应 CAN ID 的低 8 位中，不在数据区重复传输。

### 7.3 响应时隙计算

每台设备使用完整 UUID 和本轮 nonce 计算响应时隙：

```text
slot_input = UID[0..11] + nonce_le[0..3]
slot_hash  = CRC16_XMODEM(slot_input)
slot_count = 1 << slot_exp
slot       = slot_hash % slot_count
delay_ms   = slot * slot_ms
```

设备收到新发现请求后：

1. 取消尚未发送的旧轮次发现任务。
2. 保存新 `round_id`、nonce 和到期时间。
3. 在通信线程的周期 `poll()` 中等待到期。
4. 到期后发送一次发现响应。
5. 同一 nonce、同一轮次最多发送一次。

### 7.4 为什么仍需要时隙

本方案不改变 CAN ID 格式，因此相同 MotorID 的设备仍使用相同发现响应 ID。例如三台 ID=2 设备都返回 `0xF402`。

单帧消除了多帧重组杂糅，但如果多台设备在同一时刻发送 `0xF402` 且数据不同，仍会产生 CAN 数据段冲突。因此必须通过 UID 和 nonce 派生时隙，将响应时间错开。

### 7.5 扫描轮次策略

上位机默认执行：

```text
初始扫描：256 时隙，2 ms/时隙，至少 3 轮
单轮窗口：256 * 2 ms + 50 ms 保护时间 = 562 ms
```

自适应扫描：

1. 某设备仅在一轮出现：追加两轮 512 时隙扫描。
2. 扫描期间 CAN 错误计数增加：追加两轮 1024 时隙扫描。
3. 最多执行 7 轮，超出后提示扫描结果可能不完整。
4. 每轮必须使用新的随机 nonce。
5. 上位机按 Fingerprint64 合并结果，不按 MotorID 覆盖。

该方法是保持现有仲裁 ID 格式条件下的概率型防碰撞方案。它不能提供数学上的绝对枚举保证；若现场要求绝对保证，必须使用物理隔离、设备按键确认，或未来引入 UID 派生仲裁 ID 的专用协议域。

---

## 8. 单帧按 CAN-DI 设置 MotorID

### 8.1 设置请求

```text
CAN ID = (0xF5 << 8) | 0x00 = 0xF500
IDE    = Extended
RTR    = Data
DLC    = 8
```

| Byte | 字段 | 类型 | 说明 |
|---:|---|---|---|
| 0~6 | `target_can_di56` | `bytes[7]` | 目标设备 CAN-DI |
| 7 | `new_motor_id` | `u8` | 新地址，范围 `1~127` |

所有设备都接收该广播，但处理规则必须是：

1. 本机 CAN_DI56 不匹配：静默，不返回 NACK。
2. CAN_DI56 匹配：检查长度、新 ID、设备状态和 Flash 状态。
3. 只有匹配设备允许执行 Flash 写入。
4. 同一设备同一时刻只允许一个改址事务。

### 8.2 设置响应

响应必须在运行时 ID 切换前，使用旧 MotorID 发送：

```text
CAN ID = (0xF5 << 8) | old_motor_id
IDE    = Extended
RTR    = Data
DLC    = 8
```

| Byte | 字段 | 类型 | 说明 |
|---:|---|---|---|
| 0~6 | `CAN_DI56` | `bytes[7]` | 应答设备标识 |
| 7 | `status` | `u8` | 复用 `JM_ERR_*` 状态码 |

建议状态码：

| 状态 | 含义 |
|---:|---|
| `0x00 JM_ERR_OK` | Flash 写入成功，即将切换 |
| `0x02 JM_ERR_OUT_OF_RANGE` | 新 ID 不在 `1~127` |
| `0x03 JM_ERR_STATE_DENY` | 电机未失能或当前状态不允许 |
| `0x08 JM_ERR_FLASH` | 通用 Flash 错误 |
| `0x11 JM_ERR_FLASH_WRITE` | Flash 写入失败 |
| `0x12 JM_ERR_FLASH_VERIFY` | Flash 回读校验失败 |
| `0x0E JM_ERR_RATE_LIMIT` | 修改过于频繁 |

广播请求中只有匹配设备应答，所以失败也使用 `0xF5` 定向结果帧，不使用会被所有设备触发的通用广播 NACK。

### 8.3 立即生效时序

```text
收到 F5 广播
    -> 校验 CAN_DI56
    -> 校验 new_motor_id
    -> 确认电机失能/IDLE
    -> 写 Flash
    -> Flash 回读校验
    -> 使用旧 MotorID 发送 F5 ACK
    -> 等待发送邮箱真正清空
    -> 更新所有运行时 MotorID
    -> 重新配置单播硬件过滤器
    -> 使用新 MotorID 发送 F5 确认帧
    -> 恢复普通通信
```

必须同步更新：

```text
dev_commun_can.motor_id
dev_commun_can.jm.motor_id
dev_commun_can.jm.proto.motor_id
motor_info Flash 中的 can_id
CAN/FDCAN 单播硬件过滤器
软件目标地址过滤
```

Flash 写入失败时：

- 恢复 RAM 中的旧配置。
- 使用旧 MotorID 返回失败状态。
- 不修改运行时 ID。
- 不重新配置过滤器。

过滤器热切换失败时：

- 禁止继续处于“部分变量已更新”的状态。
- 优先回滚全部运行时变量和过滤器到旧 ID。
- 如果驱动无法可靠回滚，则执行受控系统复位，由已校验的 Flash 新 ID 在启动流程中重新加载。
- 上位机通过重新发现判断设备最终使用旧 ID 还是新 ID，不盲目重复写 Flash。

---

## 9. 物理设备识别

仅凭 CAN-DI 字符串无法让操作人员判断它对应哪台实体设备，因此同时实现物理指示命令。

### 9.1 识别请求

```text
CAN ID = (0xF6 << 8) | 0x00 = 0xF600
DLC    = 8
```

| Byte | 字段 | 说明 |
|---:|---|---|
| 0~6 | `target_can_di56` | 目标设备 |
| 7 | `duration_100ms` | 指示持续时间，0 表示停止 |

只有 CAN_DI56 匹配设备执行，其他设备静默。当前实现暂时关闭故障灯并让状态灯以
200 ms周期快闪；识别结束后恢复原状态映射，不修改故障锁存状态。

### 9.2 识别响应

```text
CAN ID = (0xF6 << 8) | current_motor_id
Data[0..6] = CAN_DI56
Data[7]    = status
```

---

## 10. 下位机实现架构

### 10.1 处理层级

CAN 接收 ISR 只能执行：

1. 基础长度和地址过滤。
2. 将物理帧复制到现有 RX 队列。
3. 更新必要的中断统计。

以下操作禁止在 ISR 中执行：

- CRC 计算和发现时隙调度。
- Flash 擦写和保存。
- 等待 ACK 发送完成。
- 停止、重配或启动 CAN 外设。
- LED 长时序控制。

F4/F5/F6 应由 `dev_commun_can.poll()` 中的 commissioning 状态机处理。

### 10.2 广播白名单

原广播白名单只允许 SYNC 和 ESTOP，当前已增加：

```text
JM_CMD_CAN_DI_DISCOVER
JM_CMD_CAN_DI_SET_ID
JM_CMD_CAN_DI_IDENTIFY
```

三个新命令虽然允许广播接收，但不能复用现有“广播执行后立即通用应答”的路径。

### 10.3 响应帧拦截

相同 MotorID 的设备会接收到彼此的 F4/F5/F6 响应。因此绑定层必须在进入 `jm_proto_can_feed()` 前识别并消费这些帧：

```text
CMD 为 F4/F5/F6 且 MotorID=0：作为请求处理
CMD 为 F4/F5/F6 且 MotorID!=0：视为其他设备响应，直接忽略
```

如果响应帧继续进入通用 dispatch，同 ID 的其他设备可能将响应误判为发给自己的请求并返回 NACK，造成新的总线冲突。

此外，`TELEMETRY(0xCA)` 和 `NACK(0xFE)` 是 Motor -> Host 单向帧，CAN协议入口必须
静默丢弃收到的这两类帧。否则同 ID设备会把其他节点的遥测误判为请求并产生 NACK，
随后继续对彼此的 NACK返回 NACK，形成持续占满总线的 `0xFE` 自激风暴。

空载荷查询的响应也必须在 CAN入口识别并丢弃，包括 `CALIB_QUERY(0x97)`、
`PID_SOURCE_GET(0xA2)`、`READ_xxx(0xC0~0xC9)` 和设备信息查询
`0xD0~0xD2`。例如 A2请求 DLC=0、响应 DLC=3；如果响应再次进入 dispatch，
两台同 ID设备会持续互发 `01 01 01`。上位机打开 CAN后必须先进行 F4发现，确认
所选 MotorID没有冲突后，才能发送这些普通查询。

### 10.4 当前状态数据

`dev_commun_can_t` 当前保存以下 commissioning 状态：

```c
uint8_t can_uid[12];
uint8_t can_di56[7];
uint8_t can_di_guard;
uint8_t can_di_valid;
uint8_t discover_pending;
uint8_t id_switch_pending;
uint8_t pending_new_id;
uint32_t discover_due_tick;
uint32_t commissioning_quiet_until;
```

### 10.5 驱动层接口

当前 `drv_can_send()` 成功只表示帧已进入发送邮箱或 FIFO，不表示帧已经完成总线发送。立即改址需要补充：

```c
int drv_can_wait_tx_idle(canNumber_e can, uint32_t timeout_ms);
int drv_can_reconfigure_dual_filter(canNumber_e can,
                                    const drvCanDualFilter_t *filter);
```

约束：

- 只能在线程或主循环调用，不能在 ISR 调用。
- `wait_tx_idle()` 必须有有限超时。
- 过滤器切换期间停止接收通知、停止外设、重配、启动并恢复通知。
- 广播过滤器继续保持 MotorID=0。

### 10.6 发现发送重试

当前支持的 V1、SFOC 和 ODriveMKS 板级 CAN初始化均配置
`AutoRetransmission = DISABLE`。因此每轮发现只提交一次响应；同一时隙发生碰撞后，
设备等待下一轮新的 nonce，不会在当前轮同步无限重发。

后续新增板型时必须继续保持 discovery 单次发送语义。如果新板型需要为普通控制帧启用
硬件自动重传，则应在驱动层提供 commissioning 专用 single-shot发送接口，不能直接
让 F4发现响应继承无限重传设置。

---

## 11. 上位机实现架构

### 11.1 Transport 层

CAN 发现模式需要接收任意 `1~127` 节点的 F4/F5/F6 响应。

处理原则：

- F4/F5/F6 始终按单帧解析。
- 不进入 `_rx_segments` 多帧缓存。
- 从 CAN ID 低 8 位提取当前 MotorID。
- 将接收时间归属到当前扫描轮次。
- 当前轮时间窗结束后丢弃迟到响应。

当前 Client 层信号：

```python
can_di_discovered(node_id, di56, guard)
can_di_set_result(node_id, di56, status)
can_di_identify_result(node_id, di56, status)
```

扫描轮次由主窗口当前时间窗归属，不放入设备响应载荷，也不作为信号参数传递。

### 11.2 Client 层

当前 Client 层接口：

```python
begin_can_scan()
discover_can_di(round_id, slot_exp, slot_ms, nonce)
set_can_id_by_di(di56, new_id)
identify_can_device(di56, duration_100ms)
end_can_scan()
```

原有 `query_dev_info()`、`probe_can_node()` 和 `cmd_set_can_id()` 保持兼容。

### 11.3 数据模型

扫描结果不能继续使用 MotorID 作为字典键。建议结构：

```python
@dataclass
class CanDiDevice:
    di56: bytes
    guard: int
    current_id: int
    seen_rounds: set[int]
    last_seen: float
    desired_id: int | None
    state: str
    error: str

device_key = di56 + bytes([guard])
devices: dict[bytes, CanDiDevice]
```

额外建立两个索引：

```text
current_id -> Fingerprint64列表，用于判断节点 ID 冲突
CAN_DI56   -> Guard集合，用于判断压缩标识冲突
```

### 11.4 设备管理窗口

现有扫描完成后的单选输入框改为非模态“CAN 设备管理”窗口，窗口在设置成功后保持打开。

建议列：

| 状态 | 当前 ID | CAN-DI | 出现轮次 | ID 冲突 | 新 ID | 操作 |
|---|---:|---|---:|---|---:|---|
| 在线 | 2 | `A1...8F` | 3/3 | ID 2 冲突 | 5 | 识别、设置 |
| 在线 | 2 | `B7...16` | 3/3 | ID 2 冲突 | 6 | 识别、设置 |
| 在线 | 2 | `C9...42` | 3/3 | ID 2 冲突 | 7 | 识别、设置 |

界面规则：

- 名称显示为 `CAN-DI`，不能显示为 UUID。
- 相同 MotorID 的设备分别显示，不覆盖。
- ID 冲突组禁止普通控制和多帧设备信息查询。
- DI56 冲突组禁止远程设置。
- 支持单台设置。
- 支持多选后自动分配未占用 ID。
- 设置成功立即刷新当前行和占用集合，不关闭窗口。

### 11.5 批量设置状态机

批量设置必须串行执行：

```text
待处理
  -> 发送设置命令
  -> 等待旧 ID ACK
  -> 等待运行时切换
  -> 接收新 ID 确认或重新发现验证
  -> 更新占用集合
  -> 处理下一台
```

建议状态：

```text
DISCOVERED
PENDING
WRITING
ACKED_OLD_ID
SWITCHING
VERIFIED
FAILED
TIMEOUT
```

批量示例：

```text
DI-A: 2 -> 5，验证成功
DI-B: 2 -> 6，验证成功
DI-C: 2 -> 7，验证成功
最后重新扫描至少 3 轮
```

设置前根据扫描结果建立已占用 ID 集合。每成功一台立即更新集合，目标 ID 已占用时禁止提交。

---

## 12. 完整 UUID 和版本读取流程

CAN-DI 扫描只用于建立：

```text
Fingerprint64 <-> current_motor_id
```

当同 ID 冲突解除后，上位机按新 MotorID 执行现有流程：

```text
选择新 MotorID
-> 发送 READ_DEV_INFO(0xD0)
-> 读取 hw_ver
-> 读取 fw_ver
-> 读取完整 UUID[12]
-> 校验响应中的 reported_id
```

因此 `0xD0` 的28字节应答格式、完整 UUID读取和版本显示逻辑不需要由 CAN-DI 协议替代。

---

## 13. 安全和异常处理

### 13.1 状态限制

- 仅允许在电机失能或系统 IDLE 状态设置 MotorID。
- 故障状态是否允许修改应由产品策略明确，默认建议禁止。
- 设置过程中禁止启动电机和切换控制模式。

### 13.2 扫描期间通信限制

发现重复 MotorID 后，上位机应停止该冲突 ID 的：

- `READ_DEV_INFO` 多帧查询。
- 普通反馈轮询。
- 遥测订阅和控制命令。

如设备正在周期上报遥测，进入 commissioning 扫描窗口时应暂停非安全类主动上报，避免重复 ID 遥测继续产生总线冲突。ESTOP 和 SYNC 不得被屏蔽。

### 13.3 超时恢复

设置 ACK 超时后：

1. 不立即重复写 Flash。
2. 同时检查旧 ID 和新 ID 的单帧确认。
3. 重新执行 CAN-DI 发现。
4. 根据相同 Fingerprint64 报告的实际 MotorID 恢复 UI 状态。

### 13.4 断电恢复

- Flash 保存完成前断电：设备继续使用旧 ID。
- Flash 保存完成后、运行时切换前断电：重启后加载新 ID。
- 上位机始终以重新发现结果为最终事实来源。

### 13.5 访问控制

CAN-DI 是设备标识，不是认证凭据。若后续需要防止现场误改，可增加物理配置使能、上电配置时间窗或会话授权，但不应把 CAN-DI 当作密码。

---

## 14. 兼容性

### 14.1 旧固件

- 旧固件不识别 F4/F5/F6，保持静默。
- 新上位机 F4 扫描无响应时可以回退旧版 `1~127` 单播扫描。
- 回退扫描只能处理 ID 唯一设备，必须明确提示无法识别同 ID 设备。

### 14.2 旧上位机

- 旧上位机仍可使用 `0xD0` 和 `0xF0` 管理 ID 唯一设备。
- 新固件不改变原命令行为。

### 14.3 CAN FD

- CAN-DI 协议始终使用 DLC=8 单帧。
- 不依赖 CAN FD 长帧。
- CAN FD 运行模式下仍按相同8字节定义发送，避免两套发现协议。

---

## 15. 代码改造清单

### 15.1 下位机

| 文件 | 修改内容 |
|---|---|
| `User/Protocol/joint_proto/jm_cmd_def.h` | 增加 F4/F5/F6 命令枚举和协议常量 |
| `User/Protocol/joint_proto/jm_proto_can.c/h` | 扩展广播白名单；避免通用广播应答 |
| `User/Devices/dev_commun_can.c/h` | 增加 commissioning 状态机、响应拦截、时隙调度和运行时改址 |
| `User/Driver/drv_can.c/h` | 增加 TX 完成等待、过滤器运行时重配、必要的有限重试能力 |
| `User/DataHub/runtime_param.c/h` | 复用现有 UUID 数据源，不改变完整 UUID读取逻辑 |
| `User/Protocol/joint_proto/jm_proto_ops.c/h` | 复用 Flash 原子保存和状态检查，明确立即生效接口边界 |
| `User/Tools/scripts/gen_can_dbc.py` | 增加 F4/F5/F6 DBC 描述 |
| `User/Protocol/docs/joint_motor_command_list.csv` | 增加命令记录 |

### 15.2 上位机

| 文件 | 修改内容 |
|---|---|
| `User/Tools/pyqt_gui/jmproto/cmd_def.py` | 增加命令枚举 |
| `User/Tools/pyqt_gui/transport/can_transport.py` | 增加任意节点 F4/F5/F6 单帧接收和发送接口 |
| `User/Tools/pyqt_gui/core/motor_client.py` | 增加 CAN-DI扫描、设置、识别信号和方法 |
| `User/Tools/pyqt_gui/ui/main_window.py` | 替换旧扫描流程，接入设备管理窗口 |
| `User/Tools/pyqt_gui/ui/` 新对话框 | 实现设备表格、冲突提示、自动分配和串行设置状态机 |
| `User/Tools/pyqt_gui/resources/joint_motor_command_list.csv` | 同步命令描述 |

---

## 16. 推荐实施顺序

1. 固化 CAN-DI 算法，并为固定 UUID 建立测试向量。
2. 定义 F4/F5/F6 命令和载荷，更新协议文档、CSV、DBC生成器。
3. 下位机实现 F4 单帧发现和时隙调度。
4. 上位机 transport/client 实现 F4 收发和按 Fingerprint64 聚合。
5. 使用多台相同 ID 设备验证发现稳定性。
6. 下位机实现 F5 Flash保存、旧 ID ACK和运行时过滤器切换。
7. 上位机实现单台设置、确认和重新发现验证。
8. 实现 F6 物理指示。
9. 实现设备管理窗口和串行批量设置。
10. 增加自动化测试、虚拟设备仿真和三台真机验收。

---

## 17. 测试计划

### 17.1 CAN-DI 算法测试

- 固定 UID 输入生成固定 CAN_DI56 和 Guard。
- C 与 Python 使用相同测试向量，输出完全一致。
- 验证 UID字节序不受主机端平台影响。
- 相同 LOT、不同坐标生成不同 CAN-DI。
- 不同 LOT、相同坐标和晶圆号通常生成不同 CAN-DI。

### 17.2 发现测试

- 单台设备正常发现。
- 3台设备全部为 ID=2，稳定显示3行。
- 10台设备混合唯一 ID和重复 ID。
- 人工制造同一轮相同时隙，后续 nonce轮次能够恢复。
- 发现帧不进入多帧重组器。
- 迟到响应不归入错误轮次。
- 扫描期间不会触发广播 NACK风暴。
- DI56相同、Guard不同的模拟设备被标记为不可设置。

### 17.3 设置测试

- 单台 `2 -> 5` 立即生效。
- 旧 ID不再响应，新 ID立即响应。
- 3台 `2/2/2 -> 5/6/7` 串行设置成功。
- 非目标 CAN-DI设备保持静默。
- ID越界返回明确状态。
- 电机运行时拒绝设置。
- Flash写入失败不切换运行时 ID。
- ACK丢失后重新发现能够确认最终 ID。
- 设置过程中断电，重启后 ID与已验证 Flash内容一致。
- 目标 ID已占用时上位机阻止设置。

### 17.4 回归测试

- 原 `0xF0 SET_CAN_ID` 行为不变。
- `0xD0 READ_DEV_INFO` 完整 UUID读取正常。
- 版本号解析和显示正常。
- SYNC和ESTOP广播行为正常。
- 普通单帧和多帧收发不受影响。
- CAN FD模式切换不受影响。
- Bus-Off恢复和通信丢失保护不受影响。

---

## 18. 验收标准

方案完成需同时满足：

1. 三台真实设备在同一总线上均设置为 ID=2。
2. 上位机至少连续10次扫描，每次均显示3个不同 Fingerprint64。
3. 扫描期间无持续错误帧、Bus-Off或多帧 CRC错误增长。
4. 用户能够逐行执行物理识别。
5. 用户能够串行设置为 ID=5、6、7。
6. 每台设置后无需人工重启即可用新 ID通信。
7. 旧 ID停止普通响应。
8. 最终重新扫描结果为三个不同 CAN-DI，分别对应5、6、7。
9. 分配唯一 ID后，三个设备均能通过 `0xD0`读取完整 UUID。
10. 原有控制、遥测、安全广播和 DBC解析全部通过回归测试。

---

## 19. 实现决策摘要

```text
不修改：CAN_ID = (CMD << 8) | MotorID
不修改：完整 UUID 由 READ_DEV_INFO(0xD0) 读取
新增：CAN-DI56，仅用于同 ID设备发现和定向设置
新增：F4 单帧广播发现
新增：UID + nonce 派生响应时隙
新增：F5 单帧按 CAN-DI 设置 ID
新增：F6 单帧物理设备识别
要求：Flash成功后、旧 ID ACK发送完成，再立即切换运行时 ID和过滤器
要求：上位机以 Fingerprint64 保存设备，以 MotorID建立冲突分组
要求：批量改址串行执行，每台都经过 ACK、切换和重新发现验证
```
