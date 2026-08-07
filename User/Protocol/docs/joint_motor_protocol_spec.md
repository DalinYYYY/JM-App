# 关节电机通信协议说明

> 配套文件：
> - `joint_motor_command_list.csv` —— 命令总表（含预留命令码）
> - `joint_motor_param_index.csv` —— 参数索引表（86 个，param_id 0~85，与 `motor_param.h` 一一对应），位于 `User/Tools/pyqt_gui/resources/` 与 `User/Tools/motor_info_gen/`
> - `motor_info.csv` —— 电机配置参数表（含 CAN 配置字段），位于 `User/Tools/pyqt_gui/resources/` 与 `User/Tools/motor_info_gen/`
>
> 本协议同时覆盖**串口（USART）**与 **CAN/CAN FD**，逻辑命令码 `CMD` 两者共用，仅封装层不同。
>
> **协议版本**: 1.2（§AA1，由 0xD0 READ_DEV_INFO 应答携带，详见 `jm_cmd_def.h`）

## 1. 命令码 CMD 分区

| 区间 | 类别 | 说明 |
|------|------|------|
| 0x00–0x0F | 系统控制 | 使能/失能/停止/急停等，对应 `ctrl_mode_e` 基础指令 |
| 0x10–0x2F | 运动控制 | 开环/电流/力矩/MIT/速度/位置等闭环模式 |
| 0x30–0x4F | 高级力控 | 阻抗/导纳/力位混合/重力补偿等 |
| 0x50–0x6F | 轨迹同步 | PVT/样条/梯形/S型/回零/总线同步 |
| 0x70–0x7F | 特殊应用与测试 | 脉冲方向/点动/老化/扫频等 |
| 0x80–0x8F | 多电机同步(预留) | §AB2/§Q16 仅定义命令码不实现，回 NACK(NOT_SUPPORTED) |
| 0x90–0xAF | 校准 | 电机参数/编码器/力矩常数/ADC等辨识 |
| 0xB0–0xBF | 系统诊断 | 清障/Bootloader/日志/高速采集 |
| 0xC0–0xCB | 反馈查询 | 实时反馈读取/遥测 |
| 0xCC–0xCF | OTA预留 | §AK5/§Q17 仅定义命令码不实现，回 NACK(NOT_SUPPORTED) |
| 0xD0–0xDF | 设备信息 | 版本/UID/名称/心跳(0xD2) |
| 0xE0–0xEF | 参数读写 | 通用读写 + motor_info 配置读写(0xE6~0xEC) |
| 0xF0–0xFF | CAN管理与通用 | 改地址/波特率/广播/NACK |

> 0x00–0xB8 段的 CMD 值与 `state_define.h` 的 `ctrl_mode_e` 数值**完全一致**，固件解析时 `CMD` 可直接当控制模式用。
>
> §AA2 保留区规则：预留命令码（0x80~0x82 同步、0xCC~0xCF OTA）已定义但未实现 handler，固件回 NACK(err_code=0x13 NOT_SUPPORTED)。老固件收到未定义命令码回 NACK(err_code=0x01 UNSUPPORTED)。

## 2. 串口帧格式（复用 packer_parser）

```
+--------+--------+--------+--------+---------+------------------+--------+--------+
| STX_H  | STX_L  | LEN_H  | LEN_L  | HDR_CHK | CMD + DATA       | CRC_L  | CRC_H  |
| 0xA5   | 0x5A   |  长度高 |  长度低 |  头校验  | (LEN 字节)        |  CRC低  | CRC高   |
+--------+--------+--------+--------+---------+------------------+--------+--------+
```

- `LEN` = `CMD(1) + DATA(n)` 的总字节数（大端）。
- `HDR_CHK` = `(STX_H + STX_L + LEN_H + LEN_L) & 0xFF`。
- `CRC16` 覆盖**数据区（CMD+DATA）**，小端落帧（低字节在前）。算法须与上位机一致。
- 数据区首字节为 `CMD`，其后为该命令的载荷（见命令表「串口请求/应答载荷」列）。
- 多字节数值统一**小端**编码，浮点为 IEEE-754 `f32`。

### 串口数据区示例（位置环 0x15，目标 1.57 rad）
```
A5 5A 00 05 04 | 15 DB 0F C9 3F | CRC_L CRC_H
                 └CMD┘└─ 1.57f ─┘
```
LEN=5 (CMD 1 + DATA 4)，HDR_CHK=(0xA5+0x5A+0x00+0x05)&0xFF=0x04。

## 3. CAN 帧格式（扩展帧 29 位）

仲裁 ID 编码：

```
bit28 ........ bit8 | bit7 .... bit0
       CMD (高位)    |   电机ID (1~127)
ID = (CMD << 8) | MotorID
```

- **数据区直接是命令载荷**（不含 CMD，CMD 已在 ID 里），最多 8 字节。
- 电机ID=0 为**广播地址**，所有电机接收但不应答（用于同步，见 0xF2）。
- 应答帧：固件可在发送方向上对 ID 置一个方向标志位（如 bit27=1）区分上行/下行，或约定电机回复用同一 ID。本表 CAN_ID 列给的是 Host→Motor 方向。
- 载荷 >8 字节的命令（备注标「CAN需分包」）需分多帧：建议 DATA[0] 放分包序号/总数，或改用 ISO-TP/自定义分段。串口无此限制。

### CAN 数据区示例（位置环 0x15，电机ID=3，1.57 rad）
```
ID = 0x1500 | 0x03 = 0x1503 (扩展帧)
DATA = DB 0F C9 3F   (1.57f 小端, 4字节)
```

### CAN 多帧分包白名单

下位机 `jm_proto_can.c` 的 `is_multi` 白名单覆盖以下命令。白名单仅作用于**请求方向**（Host→Motor）：请求载荷 >8B 时启用多帧重组。**应答方向**（Motor→Host）的多帧由 `can_emit_payload` 自动处理（len>8 即分包），无需白名单。

| CMD | 名称 | 说明 |
|-----|------|------|
| 0xA0 | PID_AUTOTUNE | 请求 13B |
| 0xB9 | TRACE_CONFIG | 请求 13B，经典 CAN 需分包 |
| 0xE1 | PARAM_WRITE | 写 char[16] 时 18B |
| 0xE9 | MOTOR_INFO_WRITE_BULK | 变长请求 |
| 0xE2 | PARAM_READ_BULK | 应答变长 |
| 0xE3 | PARAM_WRITE_BULK | 请求变长 |
| 0x31 | ADMITTANCE | 请求 16B |
| 0x33 | FORCE_POSITION_HYBRID | 请求 12B |
| 0x38 | VARIABLE_IMPEDANCE | 请求 12B |
| 0x50 | PVT | 请求 12B |
| 0x51 | CUBIC_SPLINE | 请求 18B |
| 0x52 | TRAPEZOIDAL_TRAJ | 请求 12B |
| 0x53 | S_CURVE_TRAJ | 请求 16B |
| 0x76 | TEST_SWEEP_FREQ | 请求 12B |

> 不在白名单的命令在 CAN 上请求方向仅支持 ≤8B 单帧。CSV 备注列标注「CAN需分包」的命令中，请求方向需多帧的（如 0x31/0x50/0xA0/0xB9/0xE1 等）应在此清单内；仅应答方向需多帧的（如 0xBA/0xCA/0xD0/0xD1/0xE8）不在清单中是正常的，由 `can_emit_payload` 自动分包。0xC9 仅保留空载荷通用调试查询，不再承载波形数据。

## 4. MIT 控制帧定点压缩（0x13 / 0x30，CAN 专用 8 字节）

CAN 下 MIT 五参数压缩进 8 字节（64 bit），与达妙/CubeMars 习惯一致：

| 字段 | 位宽 | 范围(可配) | 说明 |
|------|------|-----------|------|
| pos  | 16 bit | ±12.5 rad | 目标位置 |
| vel  | 12 bit | ±65 rad/s | 目标速度 |
| kp   | 12 bit | 0~500 | 位置刚度 |
| kd   | 12 bit | 0~5 | 速度阻尼 |
| tff  | 12 bit | ±最大力矩 | 前馈力矩 |

定点转换：`raw = (value - min) / (max - min) * (2^bits - 1)`，反向还原。范围上限可由参数表（max_speed、iq_max、peak_torque 等）确定。
> 串口下 MIT 用 5 个 `f32`（20 字节）直传，精度更高，无需压缩。

### MIT 反馈帧（电机→上位机，8 字节）
| 字段 | 位宽 | 说明 |
|------|------|------|
| motor_id | 8 bit | 回复电机ID |
| pos | 16 bit | 实际位置 |
| vel | 12 bit | 实际速度 |
| torque | 12 bit | 实际力矩 |
| temp | 8 bit | 温度 |
| err | 8 bit | 故障码 |

## 5. 实时反馈帧（0xC0）

上位机周期轮询的主反馈。串口用 `f32` 全精度，CAN 用 16/8 bit 压缩适配 8 字节。
建议固件支持**定时主动上报**（无需轮询），上报周期通过 0xB5「开始日志」的 `rate_hz` 配置。

## 6. 参数读写（0xE0/0xE1）

- `param_id` 见 `joint_motor_param_index.csv`（0~82）。
- 读：请求 `{param_id:u16}` → 应答 `{param_id:u16, type:u8, value:bytes}`。
- 写：请求 `{param_id:u16, value:bytes}` → 应答 `{param_id:u16, status:u8}`，仅写入 RAM。
- `value` 字节数由参数类型决定（见索引表「字节数」列）。
- 写入后须用 0xE4 或 0xB3「保存配置」固化到 Flash，否则掉电丢失。
- `type` 编码：0=u8 1=i8 2=u16 3=i16 4=u32 5=i32 6=f32 7=char[]。

## 7. 错误码（NACK 0xFE 的 err_code）

NACK 载荷格式 (§H2): `[0xFE][orig_cmd][err_code][seq]` (4B)
- `orig_cmd`: 触发 NACK 的原命令码
- `err_code`: 见下表
- `seq`: 异步命令的序列号 (§S2)，同步命令填 0

| 码 | 含义 | 备注 |
|----|------|------|
| 0x00 | 成功(实际用ACK,不发NACK) | |
| 0x01 | CMD不支持 | 命令码区间不识别(老固件收到新命令) |
| 0x02 | 参数越界 | |
| 0x03 | 状态不允许(如未使能就发运动指令) | |
| 0x04 | param_id无效 | |
| 0x05 | CRC/校验错误 | |
| 0x06 | 长度错误 | |
| 0x07 | 只读参数不可写 | |
| 0x08 | Flash读写失败(通用) | |
| 0x09 | 处于故障态需先清障 | |
| 0x0A | 校准未完成/校准中 | |
| 0x0B | 异步已排队(§H1/§S2) | 即时 NACK 携带 seq |
| 0x0C | 忙(双通道主控被占用, §AH2) | |
| 0x0D | 鉴权失败(令牌不匹配, §Z1) | |
| 0x0E | 速率限制(命令频率超限, §Z2) | |
| 0x0F | 未找到(资源/文件/记录不存在) | |
| 0x10 | Flash擦除失败(§S1) | |
| 0x11 | Flash写入失败(§S1) | |
| 0x12 | Flash校验失败(读回不匹配, §S1) | |
| 0x13 | 命令未实现(预留命令如OTA/SYNC, §AA2) | 与 0x01 区别: 命令码已定义但未实现 |

> §H1 异步命令双时序协议:
> - 即时 NACK: 收到命令立即校验失败时返回 (err_code + seq=0)
> - 最终 ACK/NACK: 异步任务完成后返回 (err_code=OK/具体错误 + seq=递增)

## 8. 典型交互流程

1. 上电 → 上位机发 0xD0 读设备信息 → 确认在线与版本。
2. 配置阶段：0xE1 批量写参数 → 0xE4 保存。
3. 校准：0x9B 一键校准 → 轮询应答 progress 至完成。
4. 运行：0x04 上使能 → 选模式（如 0x15 位置环）下发目标 → 0xC0 周期读反馈。
5. 停止：0x06 停止运行 → 0x05 下使能。
6. 异常：收到 0xFE NACK 或 0xC8 故障码 → 0xB0 清障。

## 9. 实现建议

- 串口侧：`upacker_unpack` 收齐整帧后回调，回调内取 `data[0]` 作 CMD 分发；下行用 `upacker_pack` 封 CMD+DATA。
- CAN 侧：从仲裁 ID 拆出 CMD 与电机ID，DATA 直接是载荷；建立 `CMD → 处理函数` 跳转表，与串口共用同一套命令处理逻辑，仅封装层不同。
- 建议把命令处理做成与传输无关的 `cmd_dispatch(cmd, data, len, reply_buf)`，串口和 CAN 都调它，最大化复用。

## 10. 0xD0 READ_DEV_INFO 应答扩展格式

CSV 描述应答为 `{hw_ver:u32;fw_ver:u32;uid:bytes12}`（20B），实际固件返回 28B，后 8B 为扩展字段：

| 偏移 | 长度 | 字段 | 说明 |
|------|------|------|------|
| 0 | 4 | hw_ver | 硬件版本 |
| 4 | 4 | fw_ver | 固件版本 |
| 8 | 12 | uid | 96 位 UID |
| 20 | 1 | motor_id_def | 默认 CAN 地址 |
| 21 | 1 | proto_major | 协议主版本（当前 1） |
| 22 | 1 | proto_minor | 协议次版本（当前 1） |
| 23 | 1 | feat_lo | feature_flags 低字节 |
| 24 | 1 | feat_hi | feature_flags 高字节 |
| 25 | 3 | reserved | 保留 |

> 老上位机读前 20B 即可，后 8B 自动忽略。

## 11. 占位命令与 TODO 清单

以下命令为占位实现或预留，量产前需评估补全：

### 11.1 占位命令（仅切状态，不解析载荷）

| CMD | 名称 | 状态 |
|-----|------|------|
| 0xB0 | CLEAR_FAULT | 占位：仅切状态，不清 fault_mask |
| 0xB1 | DIAGNOSTIC | 占位：仅切状态，不解析载荷 |
| 0xB5 | START_LOG | 旧版命令，返回 NOT_SUPPORTED |
| 0xB6 | STOP_LOG | 旧版命令，返回 NOT_SUPPORTED |
| 0xB7 | HIGH_SPEED_DAQ | 旧版命令，返回 NOT_SUPPORTED |
| 0xB8 | SINGLE_STEP | 占位：仅切状态 |
| 0xB9 | TRACE_CONFIG | 配置/停止高速 TRACE：`enable:u8 + session:u16 + mask:u32 + rate:u32 + packet_samples:u8 + flags:u8`；ACK 返回 session、实际采样率、实际打包点数和缓冲容量。LIVE 与 TRACE 配置相互独立 |
| 0xBA | TRACE_DATA | 设备主动批量上报，无逐包 ACK；固定头25B含 session、sequence、first_sample_index、actual_rate、mask、count、channels、flags、overflow_count，之后为通道位序排列的 float32 样本；控制/ACK > LIVE > TRACE，TRACE 溢出丢旧保新并置 DISCONTINUITY |
| 0xD2 | HEARTBEAT | 占位：仅被动应答，未实现主动周期上报 |

### 11.2 安全隐患（量产前必须补全）

| CMD | 名称 | 问题 |
|-----|------|------|
| 0xB2 | ENTER_BOOTLOADER | 不校验 magic，任何 0xB2 命令都会切到 BOOTLOADER 状态 |
| 0xB4 | FACTORY_RESET | 不校验 magic，且不执行恢复出厂参数 |

### 11.3 预留命令（仅定义不实现，回 NACK）

- 0x80~0x82：多电机同步（SYNC/PRESET/TRIGGER）
- 0xCC~0xCF：OTA 升级（START/DATA/END/RESUME）

### 11.4 已知 TODO

- CAN 应用桥接层未实现（协议库已就绪，缺应用层 drv_can 桥接）
- 异步命令机制基础设施就绪但未使用（`jm_proto_reply_pending/ack_async/nack_async`）
- 鉴权机制仅 0xE1 实现，0xE4/0xF0/0xF1/0x96/0xEA 待补
- 批量写速率限制缺失（0xE3/0xE9，单次写 0xE1/0xE7 已有）
- FAULT_STATE(0x09) 错误码未使用（故障态拦截缺失）
- feature_flags 不上报 AUTH(bit1)/DUAL_ARB(bit4)
