# 关节电机 CAN 通信功能说明

## 1. 架构概览

CAN 通信采用四层分层设计，自下而上为：

```
┌─────────────────────────────────────────────────┐
│  应用层 (jm_host_commun.c)                       │
│  · 遥测周期上报                                  │
│  · 通信中断降级处理                              │
│  · 命令收发调度                                  │
├─────────────────────────────────────────────────┤
│  设备粘合层 (dev_commun_can.c/h)                │
│  · CAN 收发桥接 (drvCanMsg_t ↔ jm_can_frame_t)  │
│  · 双过滤器初始化和中断注册                      │
│  · 诊断统计与错误处理                            │
├─────────────────────────────────────────────────┤
│  协议绑定层 (jm_proto_can.c/h)                  │
│  · 帧编码/解码 (ID 生成、命令提取)               │
│  · 多帧分包/重组 (含 CRC16 校验)                 │
│  · MIT/反馈帧定点压缩                            │
│  · 运行期 CAN FD 模式切换                        │
├─────────────────────────────────────────────────┤
│  驱动层 (drv_can.c/h)                           │
│  · HAL 封装 (经典 CAN / FDCAN 统一接口)          │
│  · 接收中断回调                                  │
│  · Bus-Off 恢复处理                              │
│  · TxFIFO 满时轮询等待                           │
└─────────────────────────────────────────────────┘
```

**关键设计原则**：
- 协议层与物理层解耦：`jm_proto_can` 不直接依赖 `drv_can`，通过函数指针 `jm_can_tx_fn` 注入发送能力
- CAN 与 UART 共用同一套命令分发逻辑 (`jm_proto_dispatch`)，仅编解码方式不同
- 零拷贝接收：ISR 中直接将 `drvCanMsg_t` 转换为 `jm_can_frame_t` 并 feed 协议层，无中间 ring buffer

---

## 2. CAN 帧格式

### 2.1 仲裁 ID 编码

CAN 帧使用 **29 位扩展帧**，ID 编码公式：

```
CAN_ID = (CMD << 8) | MOTOR_ID
```

- `CMD`：命令码（0x00~0xFE），占 ID[15:8]
- `MOTOR_ID`：电机地址（1~127），占 ID[7:0]
- `MOTOR_ID = 0` 为广播地址，所有电机均接收

示例：CMD=0xCB (SET_TELEMETRY)，MOTOR_ID=1 → CAN_ID=0xCB01

### 2.2 数据区编码

#### 单帧（载荷 ≤ 单帧上限）

数据区直接存放命令载荷，无控制字：

```
data[0..len-1] = payload
```

- 经典模式：单帧上限 8 字节
- FD 模式：单帧上限 64 字节

#### 多帧（载荷 > 单帧上限）

数据区首字节为控制字，后续为载荷片段：

```
data[0] = 控制字 (bit7=末帧标志, bit6:0=分包序号)
data[1..] = 载荷片段
```

控制字定义：

| 字段 | 位 | 含义 |
|------|-----|------|
| `JM_CAN_SEG_LAST` (0x80) | bit7 | 末帧标志 |
| `JM_CAN_SEG_SEQ_MASK` (0x7F) | bit6:0 | 分包序号 (0~126) |

每帧片段大小：
- 经典模式：7 字节 (8B - 1B 控制字)
- FD 模式：63 字节 (64B - 1B 控制字)

**末帧末尾追加 2 字节 CRC16 校验值**（小端序，CCITT/XMODEM 算法，初值 0x0000），覆盖整个载荷（不含控制字和 CRC 自身）。

#### 多帧接收规则

1. 序号从 0 开始，严格递增
2. 末帧置 bit7，收到末帧后 CRC16 校验
3. 校验通过：去除 CRC 后分发；校验失败：丢弃并计入 `rx_crc_fail_count`
4. 重组超时 200ms：超时未收到末帧则丢弃并计入 `rx_timeout_count`
5. 序号不连续（帧丢失或乱序）：丢弃整个多帧并计入 `rx_conflict_count`
6. 旧重组未完成又来 seq=0：记为冲突，覆盖旧缓冲

#### 多帧白名单

仅以下命令支持多帧重组（防止单帧载荷首字节与控制字歧义）：

| 命令 | 说明 |
|------|------|
| 0xE0 | 参数批量读取 |
| 0xE1 | 参数写 (变长, 写 char[16] 时 18B) |
| 0xE3 | 参数批量写 |
| 0x31 | 导纳控制 |
| 0x33 | 力位混合控制 |
| 0x38 | 变阻抗控制 |
| 0x50 | PVT 插补 |
| 0x51 | 三次样条 |
| 0x52 | 梯形轨迹 |
| 0x53 | S 型轨迹 |
| 0x76 | 扫频测试 |
| 0xA0 | PID 理论估计 |
| 0xE9 | 电机配置批量写 |

---

## 3. 定点压缩

为在经典 CAN 8 字节限制内传输浮点数据，协议层对特定命令进行定点压缩/解压。

### 3.1 MIT 控制帧 (0x13/0x30)

8 字节压缩格式，无控制字：

| 字段 | 位宽 | 范围 |
|------|------|------|
| pos | 16 | -12.5 ~ 12.5 rad |
| vel | 12 | -65.0 ~ 65.0 rad/s |
| kp | 12 | 0.0 ~ 500.0 Nm/rad |
| kd | 12 | 0.0 ~ 5.0 Nm·s/rad |
| tff | 12 | -50.0 ~ 50.0 Nm |

```
byte[0..1] = pos[15:0]
byte[2..3] = vel[11:0] << 4
byte[3..4] = kp[11:0] << 4
byte[5..6] = kd[11:0] << 4
byte[6..7] = tff[11:0] << 4
```

解压后恢复为 5 个 float32 (20 字节)，再走标准命令分发，使应用层对 CAN 和 UART 完全透明。

### 3.2 反馈帧 (0xC0)

8 字节压缩格式，由 `jm_fb_pack` 自动生成：

| 字段 | 位宽 | 范围 |
|------|------|------|
| pos | 16 | -12.5 ~ 12.5 rad |
| vel | 16 | -65.0 ~ 65.0 rad/s |
| torque | 16 | -50.0 ~ 50.0 Nm |
| temp_motor | 8 | -40.0 ~ 215.0 ℃ |
| fault_mask | 8 | 0~255 |

---

## 4. CAN FD 支持

### 4.1 硬件前提

- **MCU**：STM32G4/H7/L4 系列（内置 FDCAN 外设）
- **收发器**：需支持 CAN FD（如 CA-IS2062A），经典 CAN 收发器仅支持 8 字节帧
- **上位机**：PCAN-USB FD 或同等 FD 硬件
- **编译期**：`USE_CAN_FD_MODE=1` 且 `JM_PERIPH_CAN_FD` 已定义

### 4.2 运行期模式切换

系统上电默认 **经典 CAN 模式**（兼容所有上位机硬件），上位机可主动通过 `0xF3` 命令切换到 FD 长帧模式。

**切换流程**：

```
上位机                         下位机
  │                              │
  ├─ 0xF3 {enable=1} ──────────→│  (用当前模式发送)
  │                               ├─ 校验硬件能力
  │                               ├─ 组装 ACK{ack_enable, cap}
  │  ←── ACK{1, 1} ────────────┤  (用旧模式应答)
  │                               ├─ jm_proto_can_set_fd_mode(1)
  │                               ├─ dev_commun_can.use_fd_runtime = 1
  │                               └─ 后续帧自动以 FD 格式发送
  ├─ 收到 ACK, 切换自身 _fd_mode
  └─ 后续帧以 FD 格式收发
```

**关键注意事项**：

1. **ACK 用旧模式发出**：下位机在 `jm_proto_dispatch` 处理 0xF3 时，先组装应答帧（此时仍是旧模式），ACK 发送完成后再切换模式。这确保上位机一定能收到 ACK。

2. **纯软件操作**：运行期切换不重新初始化 FDCAN 外设。外设已在上电初始化时配置为 FD+BRS 模式，可同时发送 FD 帧和经典帧。切换仅影响 `jm_can_frame_t.is_fd` 字段，由 `drv_can_send` 据此设置 `FDFormat` 和 `BRS`。

3. **硬件不支持时**：`jm_proto_can_set_fd_mode` 返回 `cap=0`，ACK 中 `ack_enable=0`，上位机应据此判断切换失败。

4. **UART 模式**：`set_fd_mode` 回调始终返回 `cap=0`，上位机 `switch_fd_mode` 返回 `False`。

### 4.3 板级配置

在 `dev_config_board.h` 中配置：

```c
#define USE_DEV_COMMUN_CAN         // 启用 CAN 通信设备
#define USE_CAN_FD_MODE  1         // 1=启用 CAN FD, 0=仅经典 CAN

/* FDCAN 位定时 (FDCANCLK = PCLK1/2 = 85MHz) */
#define FDCAN_NOMINAL_BAUDRATE  1000000UL  // 标称段 1Mbps
#define FDCAN_DATA_BAUDRATE     8000000UL  // 数据段 8Mbps (仅 FD 帧)
#define FDCAN_NOMINAL_PRESCALER 5
#define FDCAN_NOMINAL_TS1       12
#define FDCAN_NOMINAL_TS2       4
#define FDCAN_DATA_PRESCALER    1
#define FDCAN_DATA_TS1          7
#define FDCAN_DATA_TS2          3
```

F4 系列使用经典 CAN（`JM_PERIPH_CAN_CLASSIC`），不支持 FD，`USE_CAN_FD_MODE` 被忽略。

---

## 5. 地址过滤

### 5.1 当前方案：双掩码硬件过滤

启动时从 `motor_info.device.can_id` 读取节点地址，并配置本机与广播两个硬件过滤器。仲裁 ID 的低 8 位是节点地址，因此掩码只比较低 8 位，CMD 和多帧标志不参与匹配。

```c
filter.unicast_id = pobj->motor_id;
filter.broadcast_id = 0;
filter.mask = 0xFF;
drv_can_init_dual_filter(pobj->can, &filter);
```

软件过滤由 `jm_proto_can_feed` 在入口处执行：

```c
dst = JM_CAN_GET_MOTOR_ID(frame->id);
if (dst != c->motor_id && dst != JM_CAN_BROADCAST_ID)
    return;  // 非本机且非广播帧，丢弃
```

### 5.2 驱动实现

`drv_can_init_dual_filter` 同时支持 FDCAN 和经典 CAN：

- 过滤器 0：`ID=local_id, mask=0xFF`
- 过滤器 1：`ID=0, mask=0xFF`
- 两个过滤器都路由到 FIFO0，上层无需区分来源
- FDCAN 全局拒绝未命中帧、标准帧和远程帧
- 经典 CAN 使用两个独立 Filter Bank，并比较 IDE/RTR 位

软件层继续校验本机/广播地址。广播白名单开放 `BROADCAST_SYNC`、`ESTOP` 和
CAN-DI commissioning 命令；F4/F5/F6由设备绑定层单独处理，不进入通用广播应答路径。

`TELEMETRY(0xCA)` 和 `NACK(0xFE)` 是 Motor -> Host 单向响应。下位机收到这两类
CAN帧时必须静默丢弃，禁止进入通用命令分发，防止同 ID设备之间形成 NACK反馈环。
空载荷查询（0x97、0xA2、0xC0~0xC8、0xD0~0xD2）的非空帧同样属于其他节点的
响应，也必须静默丢弃。0xC9 仅接受空载荷通用调试查询，任何非空载荷（包括旧版
`offset:u16 + count:u8` 波形请求）都必须静默丢弃。高速波形统一使用 0xB9/0xBA
TRACE 链路。上位机连接 CAN 后先执行 CAN-DI 发现，确认 ID唯一后再发送普通查询。

### 5.3 同 ID 设备发现与立即改址

同一总线上存在多个相同节点 ID 时，禁止使用 `READ_DEV_INFO(0xD0)` 多帧扫描，
否则不同设备的分片会使用相同仲裁 ID，造成冲突或重组数据杂糅。当前实现使用
CAN-DI commissioning 命令，且不改变现有仲裁 ID 编码：

```text
0xF4 CAN_DI_DISCOVER：广播发现，设备按 UID + nonce 派生时隙，返回 8 字节单帧
0xF5 CAN_DI_SET_ID：按 CAN_DI56 定向保存新 ID，旧 ID ACK 完成后热切换过滤器
0xF6 CAN_DI_IDENTIFY：按 CAN_DI56 触发目标设备状态灯快闪
```

发现结果使用 `CAN_DI56 + Guard` 作为列表键，不使用节点 ID 作为唯一键。因此多台
相同 ID 设备会保留为多行。批量改址由上位机串行执行，每台设备均等待旧 ID ACK和
新 ID 确认后再处理下一台，最后重新扫描验证。

完整载荷、压缩标识算法、异常处理和验收步骤见
《CAN_DI同ID设备发现与改址方案.md》。完整 96-bit UUID仍由 `0xD0` 读取。

---

## 6. 通信中断降级

当 CAN 链路断开或上位机停止发送时，下位机需要自主降级以保证安全。

### 6.1 配置项

| 宏 | 说明 |
|-----|------|
| `JM_CAN_LOSS_ACTION` | 0=IDLE 立即停机, 1=HOLD 位置保持, 2=TIMER 超时停机 |
| `JM_CAN_LOSS_TIMEOUT_MS` | TIMER 模式超时阈值 (ms) |

### 6.2 降级逻辑

`dev_commun_can.check_loss` 检查 `last_rx_tick` 与当前时间（`HAL_GetTick`）的差值：

- 超时 `JM_CAN_LOSS_TIMEOUT_MS` 时返回 1，由应用层自行切换 `CONTROL_MODE_IDLE` 停机
- 仅在 `JM_CAN_LOSS_ACTION == 2`（TIMER 模式）时生效，其余模式当前为预留（`#else` 分支空实现）
- 触发后重置 `last_rx_tick` 避免重复触发

> **注意**：开发期通常禁用通信中断降级（`check_loss` 调用被注释），量产期再启用，避免调试时频繁误触发。

---

## 7. 遥测周期上报

### 7.1 配置流程

1. 上位机发送 `0xCB SET_TELEMETRY {enable=1, mask, period_ms}` 启用
2. 下位机 `jm_host_commun_can_process` 按 `period_ms` 换算的 tick 频率主动推送 `0xCA TELEMETRY` 帧
3. 发送 `0xCB {enable=0}` 停止上报

### 7.2 分包注意事项

遥测帧载荷 = 2B mask + 按位拼接的数据组，长度取决于 `mask` 位掩码：

| mask 位 | 数据组 | 字节数 |
|---------|--------|--------|
| 0 (POS_VEL) | pos + vel | 8 |
| 1 (DQ) | id + iq | 8 |
| 2 (PHASE) | ia + ib + ic | 12 |
| 3 (BUS) | vbus + ibus + power | 12 |
| 4 (TEMP) | temp_fet + temp_motor | 8 |
| 5 (MULTITURN) | multiturn + single | 8 |
| 6 (TORQUE) | torque | 4 |
| 7 (FAULT) | fault_mask + warn_mask | 8 |
| 8 (STATE) | 4 状态字节 | 4 |
| 9 (DEBUG) | jm_dbg[N] | N×4 |

**典型场景**：mask=0x017F (POS_VEL+DQ+PHASE+BUS+TEMP+MULTITURN+TORQUE+STATE) = 66B，需分包 10 帧（经典模式：66B/7B=9.4 → 10 帧）。

**重要**：订阅 mask 时需评估 CAN 总线负载。经典 CAN 1Mbps 下，10 帧/周期在 200Hz 周期时可达约 40% 总线负载。

---

## 8. CRC16 校验

多帧协议使用 CRC16-CCITT/XMODEM 算法对载荷做完整性校验：

- 多项式：`0x1021`
- 初值：`0x0000`
- 输出：小端序 2 字节，追加在末帧末尾
- 校验范围：整个载荷（不含控制字和 CRC 自身）

下位机由 `crc16.c` 实现，上位机由 `jmproto/crc16.py` 实现（与下位机一致）。

**校验失败处理**：
- 下位机：丢弃重组数据，`rx_crc_fail_count++`
- 上位机：丢弃重组数据，`rx_crc_errors++`

---

## 9. TxFIFO 满处理

### 9.1 问题背景

STM32 FDCAN 的 TxFIFO 仅有 3 个邮箱深度。当多帧分包连续发送时（如 66B 遥测需 10 帧），前 3 帧进入邮箱后 TxFIFO 满，若直接返回失败则后续帧丢失。

### 9.2 解决方案

`drv_can_send` 在 TxFIFO 满时**轮询等待**（最多 1ms = 100×10μs），等待硬件发送完邮箱中的帧后腾出空间：

```c
uint32_t wait = 0u;
while (HAL_FDCAN_GetTxFifoFreeLevel(handle) == 0u)
{
    if (++wait > 100u)  // 1ms 超时
    {
        diag_inc(can, DIAG_OFF(tx_fail_count));
        return DRV_ERROR;
    }
    drv_delay_us(10);
}
```

> **注意**：此函数可能在 ISR 上下文中调用（如 CAN 接收中断中触发的应答），需确保 `drv_delay_us` 在 ISR 中可用，且超时时间不宜过长以免阻塞中断。

---

## 10. 上位机集成

### 10.1 模块结构

```
pyqt_gui/
├── transport/
│   └── can_transport.py    # CAN 传输层 (python-can 后端)
├── jmproto/
│   ├── cmd_def.py          # 命令码定义
│   └── crc16.py            # CRC16 计算 (与下位机一致)
└── core/
    └── motor_client.py     # 高层客户端 (cmd_switch_fd_mode)
```

### 10.2 初始化流程

```python
# 经典 CAN 模式
transport.open(channel="PCAN_USBBUS1", bitrate=1000000, motor_id=1, fd=False)

# CAN FD 模式 (需 PCAN-USB FD 硬件)
transport.open(channel="PCAN_USBBUS1", bitrate=1000000, motor_id=1, fd=True, data_bitrate=8000000)
```

### 10.3 FD 模式切换

```python
# 上位机主动切换到 FD 模式 (同步等待下位机 ACK)
ok = client.cmd_switch_fd_mode(enable=True)
if ok:
    print("已切换到 CAN FD 模式")
else:
    print("切换失败: 硬件不支持或超时")
```

切换逻辑由 `CanTransport.switch_fd_mode` 实现，内部通过 0xF3 命令同步等待 ACK，然后切换自身 `_fd_mode`。

### 10.4 多帧接收

`CanTransport._handle_multi_frame` 实现多帧重组，关键特性：

- **seq 连续性检查**：非连续帧丢弃整个重组（计入 `rx_seq_errors`）
- **CRC16 校验**：末帧 CRC 校验失败丢弃（计入 `rx_crc_errors`）
- **超时清理**：`_cleanup_stale_segments` 每 100ms 清理超时缓冲
- **首帧覆盖**：seq=0 的新帧无条件覆盖旧缓冲

### 10.5 性能优化

上位机 UI 层做了以下优化以避免卡顿：

- **setStyleSheet 缓存**：仅在颜色变化时重设样式表，避免 220 次/秒的样式重建
- **FOC 框图脏标记**：`set_values` 仅置脏标记，由定时器批量刷新
- **转子动画降频**：30fps（原 60fps），肉眼无差异但 paintEvent 减半
- **遥测队列批处理**：50ms 批量处理遥测帧，避免每帧触发 UI 刷新

---

## 11. 常见问题

### 11.1 遥测数据异常/状态乱跳

**现象**：上位机遥控模式下，反馈数据偶尔错乱，状态字段跳到 0。

**原因**：
1. 遥测载荷 >8B 时需多帧分包，TxFIFO 满导致中间帧丢失
2. 上位机不检查 seq 连续性，丢失帧后拼凑错误数据

**解决**：
1. 下位机 TxFIFO 满时轮询等待（已修复）
2. 上位机加 seq 连续性检查，不连续时丢弃（已修复）
3. 检查 CAN 总线负载，避免过多遥测组订阅

### 11.2 某命令 CAN 上无应答

**现象**：CAN 发送命令后无应答，但 UART 上正常。

**原因**：命令载荷 >8B 且不在多帧白名单中，下位机按单帧处理，载荷截断。

**解决**：将命令加入 `jm_proto_can.c` 的 `is_multi` 白名单，或切换到 FD 模式。

### 11.3 CAN FD 切换失败

**现象**：`cmd_switch_fd_mode(True)` 返回 `False`。

**排查步骤**：
1. 确认下位机 `USE_CAN_FD_MODE=1` 且 `JM_PERIPH_CAN_FD` 已定义
2. 确认上位机 PCAN-USB FD 硬件已连接，驱动版本 ≥4.x
3. 确认 CAN 收发器支持 FD（如 CA-IS2062A）
4. 查看上位机日志：`switch_fd_mode` 超时说明下位机未响应 0xF3

### 11.4 上位机 UI 卡顿

**现象**：实时波形和转子动画卡顿。

**原因**：多因子叠加，详见第 10.5 节。主要根因是 `setStyleSheet` 每帧调用导致 Qt 样式引擎反复重建。

**解决**：已做优化（见第 10.5 节），若仍有卡顿可进一步：
- 减少遥测订阅 mask（减少数据量和分包帧数）
- 增大遥测上报周期（降低数据传输频率）
- 关闭不需要的 UI 面板（减少重绘控件数）

---

## 12. 接口速查

### 12.1 下位机 API

| 函数 | 文件 | 说明 |
|------|------|------|
| `jm_proto_can_init` | jm_proto_can.c | 初始化 CAN 协议实例 |
| `jm_proto_can_feed` | jm_proto_can.c | 喂入收到的 CAN 帧 |
| `jm_proto_can_send` | jm_proto_can.c | 主动发送逻辑帧 |
| `jm_proto_can_set_fd_mode` | jm_proto_can.c | 切换 FD 模式 |
| `jm_proto_can_get_fd_mode` | jm_proto_can.c | 查询当前 FD 模式 |
| `jm_proto_can_di_build` | jm_proto_can.c | 从 96-bit UID生成 CAN_DI56和 Guard |
| `dev_commun_can_init` | dev_commun_can.c | 初始化 CAN 通信设备 |
| `dev_commun_can.start` | dev_commun_can.c | 启动 CAN 通信 |
| `dev_commun_can.report` | dev_commun_can.c | 主动上报数据帧 |
| `dev_commun_can.poll` | dev_commun_can.c | 周期维护（诊断刷新） |
| `drv_can_send` | drv_can.c | 底层发送 CAN 帧 |
| `drv_can_recv` | drv_can.c | 底层接收 CAN 帧 |
| `drv_can_init_dual_filter` | drv_can.c | 双过滤器初始化 |
| `drv_can_wait_tx_idle` | drv_can.c | 非阻塞/限时确认发送邮箱为空 |
| `drv_can_reconfigure_dual_filter` | drv_can.c | 运行期热切换单播与广播过滤器 |

### 12.2 上位机 API

| 方法 | 文件 | 说明 |
|------|------|------|
| `CanTransport.open` | can_transport.py | 打开 CAN 总线 |
| `CanTransport.send` | can_transport.py | 发送逻辑帧 |
| `CanTransport.send_to` | can_transport.py | 向指定节点或广播地址发送命令 |
| `CanTransport.switch_fd_mode` | can_transport.py | 切换 FD 模式 |
| `JmClient.cmd_switch_fd_mode` | motor_client.py | 高层 FD 切换接口 |
| `JmClient.set_telemetry` | motor_client.py | 配置遥测上报 |
| `JmClient.discover_can_di` | motor_client.py | 发送一轮 CAN-DI广播发现 |
| `JmClient.set_can_id_by_di` | motor_client.py | 按 CAN_DI56定向设置 ID |
| `JmClient.identify_can_device` | motor_client.py | 触发目标设备物理识别 |

### 12.3 CAN 管理命令

| 命令 | 码 | 说明 |
|------|-----|------|
| SET_CAN_ID | 0xF0 | 原子保存节点地址到 Flash，ACK 使用旧 ID，重启后生效 |
| SET_BAUDRATE | 0xF1 | 设置波特率 (0=1M, 1=500K, 2=250K, 3=125K) |
| BROADCAST_SYNC | 0xF2 | 广播同步（ID=0，所有电机同步执行） |
| SET_FD_MODE | 0xF3 | 运行期切换 CAN FD 模式 |
| CAN_DI_DISCOVER | 0xF4 | 广播单帧发现，同 ID设备按随机时隙响应 |
| CAN_DI_SET_ID | 0xF5 | 按 CAN_DI56定向设置并立即切换节点 ID |
| CAN_DI_IDENTIFY | 0xF6 | 按 CAN_DI56触发目标设备指示灯 |

---

## 13. 文件索引

| 文件 | 层 | 说明 |
|------|-----|------|
| `User/Driver/drv_can.h` | 驱动 | CAN/FDCAN 驱动接口 |
| `User/Driver/drv_can.c` | 驱动 | 收发实现 + TxFIFO 等待 |
| `User/Config/mcu_compat.h` | 配置 | MCU 系列判定 (JM_PERIPH_CAN_FD/CLASSIC) |
| `Board/*/Config/dev_config_board.h` | 配置 | 板级 CAN 配置 |
| `User/Protocol/joint_proto/jm_cmd_def.h` | 协议 | 命令码定义 + 功能特性位 |
| `User/Protocol/joint_proto/jm_proto_can.h` | 协议 | CAN 帧结构 + 压缩宏 + 接口 |
| `User/Protocol/joint_proto/jm_proto_can.c` | 协议 | 编解码/多帧/CRC/MIT 压缩 |
| `User/Protocol/joint_proto/jm_proto.h` | 协议 | 协议核心 + set_fd_mode 回调 |
| `User/Protocol/joint_proto/jm_proto.c` | 协议 | 命令分发 (含 0xF3 处理) |
| `User/Protocol/joint_proto/jm_proto_ops.c` | 业务 | set_fd_mode 回调实现 |
| `User/Devices/dev_commun_can.h` | 设备 | CAN 通信设备结构体 |
| `User/Devices/dev_commun_can.c` | 设备 | CAN 收发桥接 + 中断回调 |
| `User/AppServices/ParamService/jm_host_commun.c` | 应用 | 遥测周期上报逻辑 |
| `User/Common/crc16.h` | 公共 | CRC16 计算 |
| `User/Common/crc16.c` | 公共 | CRC16 实现 (CCITT/XMODEM) |
| `User/Protocol/docs/joint_motor_command_list.csv` | 文档 | 协议命令表 |
| `User/Protocol/docs/joint_motor_can.dbc` | 文档 | CAN DBC 文件 |
| `User/Protocol/docs/CAN_DI同ID设备发现与改址方案.md` | 文档 | CAN-DI协议、实现和验收说明 |
| `User/Tools/pyqt_gui/transport/can_transport.py` | 上位机 | python-can 传输层 |
| `User/Tools/pyqt_gui/core/motor_client.py` | 上位机 | 高层客户端 |
| `User/Tools/pyqt_gui/ui/can_device_manager.py` | 上位机 | 多设备发现、识别和批量改址窗口 |
| `User/Tools/pyqt_gui/jmproto/cmd_def.py` | 上位机 | 命令码定义 |
| `User/Tools/pyqt_gui/jmproto/crc16.py` | 上位机 | CRC16 计算 |
