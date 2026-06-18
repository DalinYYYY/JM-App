# VESC 串行通信协议详解

> 本文基于 [bldc](../) 固件源码整理，核心实现位于 [comm/](.) 目录。
> 适用于 USB-CDC（虚拟串口）、硬件 UART、蓝牙透传等所有走"逐字节流"的链路。

---

## 目录

1. [整体架构](#一整体架构)
2. [帧格式（Packet Layer）](#二帧格式packet-layer)
3. [发送端组帧流程](#三发送端组帧流程)
4. [接收端拆帧流程](#四接收端拆帧流程)
5. [CRC16 校验](#五crc16-校验)
6. [命令层（Command Layer）](#六命令层command-layer)
7. [Payload 数据编码规则](#七payload-数据编码规则)
8. [常用命令详解](#八常用命令详解)
9. [完整命令 ID 表](#九完整命令-id-表)
10. [关键常量速查](#十关键常量速查)
11. [交互示例](#十一交互示例)

---

## 一、整体架构

VESC 通信分为三层，职责清晰分离：

| 层级 | 文件 | 职责 |
|------|------|------|
| **传输层** | [comm_usb_serial.c](comm_usb_serial.c)、UART 驱动 | 实际收发字节流；USB 模拟 CDC 虚拟串口（VID `0x0483` / PID `0x5740`） |
| **帧层（Packet）** | [packet.c](packet.c) / [packet.h](packet.h) | 组帧/拆帧、起止字节、长度字段、CRC 校验、字节流重组与重同步 |
| **命令层（Command）** | [commands.c](commands.c) / [datatypes.h](../datatypes.h) | 解析 payload 首字节命令 ID 并分发，构造回复 |

数据流向：

```
接收： 物理字节流 ──逐字节──> packet_process_byte() ──完整帧──> commands_process_packet() ──按命令ID──> 各处理函数
发送： 应用数据 ──> commands_send_packet() ──> packet_send_packet() ──组帧──> send_func() ──> 物理发送
```

帧层是**无状态可复用**的：每条物理链路各持有一个 `PACKET_STATE_t` 实例（含独立收发缓冲与回调），互不干扰。这样同一套协议代码可同时跑在 USB、UART、蓝牙等多个接口上。

---

## 二、帧格式（Packet Layer）

这是协议的核心。一个完整帧的结构如下（见 [packet.c:41-73](packet.c#L41-L73)）：

```
+-----------+-------------------+---------------+-----------+----------+
|  Start    |  Length           |  Payload      |  CRC16    |  Stop    |
|  1 字节   |  1 / 2 / 3 字节    |  N 字节       |  2 字节   |  1 字节  |
+-----------+-------------------+---------------+-----------+----------+
     │              │                  │              │           │
     │              │                  │              │           └─ 固定 0x03
     │              │                  │              └─ crc16(Payload)，大端
     │              │                  └─ 命令ID(1) + 命令参数(N-1)
     │              └─ Payload 字节数，大端
     └─ 同时指示长度字段宽度：2→1字节, 3→2字节, 4→3字节
```

### 2.1 起始字节（Start Byte）—— 兼作长度字段宽度标志

起始字节的数值本身就编码了长度字段占几个字节（见 [packet.c:48-60](packet.c#L48-L60)）：

| 起始字节 | 帧类型 | 长度字段字节数 | payload 长度范围 |
|:--------:|:------:|:------------:|:----------------:|
| `0x02` (2) | 短帧 | 1 字节 | 1 ~ 255 |
| `0x03` (3) | 中帧 | 2 字节（大端）| 256 ~ 65535 |
| `0x04` (4) | 长帧 | 3 字节（大端）| 65536 ~ 16777215 |

> **巧妙设计**：起始字节的数值 `2/3/4` 恰好等于"起始字节 + 长度字段"的总字节数。代码中直接用 `data_start = buffer[0]` 作为 payload 在帧内的偏移量（见 [packet.c:165](packet.c#L165)）。

### 2.2 长度字段（Length）

- 大端字节序（高位在前）。
- 表示 **payload** 的字节数（不含起止字节、长度字段、CRC）。
- 固件默认 `PACKET_MAX_PL_LEN = 512`（见 [packet.h:27-29](packet.h#L27-L29)），可在编译期重定义。默认值下实际只用到短帧与中帧。

### 2.3 Payload（数据）

- 第 1 字节 = 命令 ID（`COMM_PACKET_ID` 枚举）。
- 其余字节 = 该命令的参数/数据，编码规则见[第七节](#七payload-数据编码规则)。

### 2.4 CRC16（2 字节）

- 算法：**CRC-16/XMODEM（CCITT）**，多项式 `0x1021`，初值 `0x0000`，输入/输出均不反转（见 [util/crc.c:26-65](../util/crc.c#L26-L65)）。
- 校验范围：**仅 Payload**，不含起止字节与长度字段（见 [packet.c:65](packet.c#L65)、[packet.c:233](packet.c#L233)）。
- 字节序：高字节在前（大端，见 [packet.c:66-67](packet.c#L66-L67)）。

### 2.5 停止字节（Stop Byte）

固定为 `0x03` (3)（写入见 [packet.c:68](packet.c#L68)，校验见 [packet.c:229](packet.c#L229)）。

---

## 三、发送端组帧流程

`packet_send_packet()`（见 [packet.c:41-73](packet.c#L41-L73)）：

1. 校验长度：`len == 0` 或 `len > PACKET_MAX_PL_LEN` 直接丢弃（见 [packet.c:42-44](packet.c#L42-L44)）。零长度帧不支持。
2. 根据 `len` 写入起始字节与长度字段（1/2/3 字节，大端）。
3. `memcpy` 拷贝 payload。
4. 计算 `crc16(payload)`，按大端写入 2 字节。
5. 写入停止字节 `0x03`。
6. 调用注册的 `send_func()` 一次性把整帧推给物理层。

---

## 四、接收端拆帧流程

`packet_process_byte()`（见 [packet.c:75-132](packet.c#L75-L132)）逐字节接收，采用**线性缓冲区**（非环形），满时用 `memmove` 平移对齐。核心机制：

### 4.1 `bytes_left` 快进优化

当已能确定"还差 N 个字节才能凑齐当前帧"时，后续字节直接累计、跳过解析，降低 CPU 占用（见 [packet.c:101-104](packet.c#L101-L104)）。

### 4.2 滑动重同步

`try_decode_packet()` 在缓冲区不同偏移处反复尝试解码，直到成功或数据耗尽（见 [packet.c:108-125](packet.c#L108-L125)）。返回值约定（见 [packet.c:150-154](packet.c#L150-L154)）：

| 返回值 | 含义 | 处理 |
|:------:|------|------|
| `>0` | 解码成功，值为整帧消耗字节数 = `len + data_start + 3` | 读指针前移该字节数 |
| `-1` | 结构非法 | 读指针 **+1** 后重试（丢弃 1 字节重新对齐）|
| `-2` | 数据不足 | 等待更多字节，设置 `bytes_left` |

### 4.3 多重合法性校验

判定一帧合法需同时满足（见 [try_decode_packet](packet.c#L155-L246)）：

- 起始字节 ∈ {2, 3, 4}（见 [packet.c:180](packet.c#L180)）。
- **长度编码最紧凑**：16 位长度必须 ≥ 255、24 位长度必须 ≥ 65535，否则判非法，防止歧义（见 [packet.c:203](packet.c#L203)、[packet.c:212](packet.c#L212)）。
- `len ≤ PACKET_MAX_PL_LEN`（见 [packet.c:218](packet.c#L218)）。
- 停止字节 == `0x03`（见 [packet.c:229](packet.c#L229)）。
- CRC 匹配（见 [packet.c:237](packet.c#L237)）。

全部通过才回调 `process_func()` 把 payload 交给命令层。

---

## 五、CRC16 校验

实现在 [util/crc.c](../util/crc.c)，查表法：

```c
unsigned short crc16_rolling(unsigned short cksum, unsigned char *buf, unsigned int len) {
    for (unsigned int i = 0; i < len; i++) {
        cksum = crc16_tab[(((cksum >> 8) ^ *buf++) & 0xFF)] ^ (cksum << 8);
    }
    return cksum;
}
unsigned short crc16(unsigned char *buf, unsigned int len) {
    return crc16_rolling(0, buf, len);  // 初值 0
}
```

参数总结：

| 参数 | 值 |
|------|-----|
| 多项式 | `0x1021` |
| 初值 | `0x0000` |
| 输入反转 | 否 |
| 输出反转 | 否 |
| 异或输出 | `0x0000` |
| 别名 | CRC-16/XMODEM、CRC-16/CCITT-FALSE 同族 |

> 注意：固件另有 `crc32`（STM32 硬件加速）用于固件升级等场景，与帧层无关。

---

## 六、命令层（Command Layer）

分发入口 `commands_process_packet()`（见 [commands.c:197-230](commands.c#L197-L230)）：

```c
packet_id = data[0];   // payload 首字节 = 命令 ID
data++;                // 之后是命令参数
len--;
...
switch (packet_id) { ... }
```

- 完整命令 ID 枚举见 [datatypes.h:955-1143](../datatypes.h#L955)，共 160 个（0 ~ 159）。
- 收到请求时记录 `reply_func`（回复回调），处理函数构造回复后调用 `commands_send_packet()` → 经帧层封帧返回。
- 部分命令（如 `COMM_EXT_NRF_*`）被当作独立通道处理（见 [commands.c:210-219](commands.c#L210-L219)）。

---

## 七、Payload 数据编码规则

多字节数值统一采用**大端序**（高位在前），实现于 [util/buffer.c](../util/buffer.c)。浮点数常以"定点缩放整数"方式传输（值 × scale 后存为整数），收端再除以同样的 scale 还原。

| 类型 | 字节数 | 编码方式 | 出处 |
|------|:------:|---------|------|
| `int8 / uint8` | 1 | 直接存放 | — |
| `int16 / uint16` | 2 | 大端 | [buffer.c:24-32](../util/buffer.c#L24-L32) |
| `int32 / uint32` | 4 | 大端 | [buffer.c:34-46](../util/buffer.c#L34-L46) |
| `int64 / uint64` | 8 | 大端 | [buffer.c:48-68](../util/buffer.c#L48-L68) |
| `float16` | 2 | `(int16)(值 × scale)`，大端 | — |
| `float32` | 4 | `(int32)(值 × scale)`，大端 | — |
| `float32_auto` | 4 | IEEE-754 紧凑编码（无需 scale）| — |
| 字符串 | 变长 | C 字符串，以 `\0` 结尾 | — |

**示例**：温度 25.6 ℃ 以 `float16, scale=10` 编码 → `256` → 字节 `01 00`。

> `scale` 的选择体现了精度与带宽的权衡：电流用 `1e2`（0.01A 精度），位置用 `1e6`（微度级），占空比用 `1e3`。这些约定在收发两端必须严格一致。

---

## 八、常用命令详解

### 8.1 控制类命令（无回复，需配合心跳）

这些命令仅下发、无回复，且都会调用 `timeout_reset()`。若停止发送，超时后电机自动停转，因此需周期性发送以维持运行。

| 命令 | ID | Payload（命令ID之后）| 还原公式 | 出处 |
|------|:--:|---------------------|---------|------|
| `COMM_SET_DUTY` | 5 | int32 | duty = v / 100000 | [commands.c:485](commands.c#L485) |
| `COMM_SET_CURRENT` | 6 | int32 | I = v / 1000 (A) | [commands.c:491](commands.c#L491) |
| `COMM_SET_CURRENT_BRAKE` | 7 | int32 | I = v / 1000 (A) | [commands.c:497](commands.c#L497) |
| `COMM_SET_RPM` | 8 | int32 | rpm = v | [commands.c:503](commands.c#L503) |
| `COMM_SET_POS` | 9 | int32 | pos = v / 1e6 (°) | [commands.c:509](commands.c#L509) |
| `COMM_SET_HANDBRAKE` | 10 | float32, scale=1e3 | — | [commands.c:515](commands.c#L515) |
| `COMM_SET_SERVO_POS` | 12 | float16, scale=1000 | — | [commands.c:535](commands.c#L535) |

**心跳**：`COMM_ALIVE` (30) 无参数，仅刷新超时计数器与关机定时（见 [commands.c:700-703](commands.c#L700-L703)）。上位机一般每 ~50ms 发一次。

**复位**：`COMM_REBOOT` (29) 无参数，保存备份数据后 `NVIC_SystemReset()`（见 [commands.c:695-698](commands.c#L695-L698)）。

### 8.2 `COMM_FW_VERSION` (ID=0) —— 固件版本查询

请求 payload 仅命令 ID 本身。回复结构（见 [commands.c:231-294](commands.c#L231-L294)）：

```
[00] 命令ID
[u8] 主版本号 FW_VERSION_MAJOR
[u8] 次版本号 FW_VERSION_MINOR
[str\0] 硬件名称 HW_NAME
[12B] STM32 UUID（双电机时第二电机 UUID 末字节+1）
[u8] pairing_done
[u8] 测试版本号 FW_TEST_VERSION_NUMBER
[u8] 硬件类型 HW_TYPE_VESC
[u8] 自定义配置数量
[u8] 相位滤波器标志（1=有 / 0=无）
[u8] QMLUI_HW 标志（0/1/2）
[u8] QMLUI_APP 标志（0/1/2）
[u8] nrf_flags
[str\0] 固件名称 FW_NAME
[u32] 硬件配置 CRC
```

### 8.3 `COMM_GET_VALUES` (ID=4) / `COMM_GET_VALUES_SELECTIVE` (ID=50)

读取实时运行数据。`SELECTIVE` 版本请求 payload 带一个 `uint32 mask`，按位选择需要的字段，回复中先回显该 mask（见 [commands.c:384-483](commands.c#L384-L483)）。非 selective 版相当于 `mask = 0xFFFFFFFF`（全选）。

字段按 bit 位顺序排列：

| bit | 字段 | 类型 / scale |
|:---:|------|-------------|
| 0 | FET（控制器）温度 | float16 / 10 |
| 1 | 电机温度 | float16 / 10 |
| 2 | 电机平均电流 | float32 / 100 |
| 3 | 输入平均电流 | float32 / 100 |
| 4 | id（直轴电流）| float32 / 100 |
| 5 | iq（交轴电流）| float32 / 100 |
| 6 | 当前占空比 | float16 / 1000 |
| 7 | 转速 ERPM | float32 / 1 |
| 8 | 输入电压 | float16 / 10 |
| 9 | 已用安时 | float32 / 10000 |
| 10 | 回充安时 | float32 / 10000 |
| 11 | 已用瓦时 | float32 / 10000 |
| 12 | 回充瓦时 | float32 / 10000 |
| 13 | 里程计 | int32 |
| 14 | 绝对里程计 | int32 |
| 15 | 故障码 | uint8 |
| 16 | PID 位置 | float32 / 1e6 |
| 17 | 控制器 ID | uint8 |
| 18 | 3 路 MOS 温度 | 3× float16 / 10 |
| 19 | vd | float32 / 1000 |
| 20 | vq | float32 / 1000 |
| 21 | 状态位（bit0=timeout, bit1=kill_sw）| uint8 |

### 8.4 `COMM_FORWARD_CAN` (ID=34) —— CAN 总线转发

把一帧 VESC 命令通过本机 CAN 总线转发给指定 ID 的从机，实现"一根串口控制整条 CAN 链路上的所有 VESC"（见 [commands.c:733-747](commands.c#L733-L747)）：

```
[22] 命令ID (COMM_FORWARD_CAN)
[u8] 目标 CAN ID
[...] 被转发的完整命令（命令ID + 参数），原样塞进 CAN 缓冲发送
```

双电机硬件上若目标 ID 是第二电机，则切换到电机线程 2 本地递归处理，而非走 CAN（见 [commands.c:736-741](commands.c#L736-L741)）。

### 8.5 `COMM_CUSTOM_APP_DATA` (ID=36) —— 自定义透传

把命令 ID 之后的原始数据交给应用层注册的 `appdata_func`（及 LispBM 处理），用于用户自定义协议（见 [commands.c:771-778](commands.c#L771-L778)）。

### 8.6 终端命令

- `COMM_TERMINAL_CMD` (20)：发送字符串终端命令，输出通过 `COMM_PRINT` 异步返回。
- `COMM_TERMINAL_CMD_SYNC` (64)：同步版本。

---

## 九、完整命令 ID 表

来源 [datatypes.h:955-1143](../datatypes.h#L955)（节选/分组）：

| ID | 名称 | 说明 |
|:--:|------|------|
| 0 | COMM_FW_VERSION | 固件版本/硬件信息 |
| 1 | COMM_JUMP_TO_BOOTLOADER | 跳转 Bootloader |
| 2-3 | COMM_ERASE/WRITE_NEW_APP_DATA | 固件升级（擦除/写入）|
| 4 | COMM_GET_VALUES | 读取实时值 |
| 5-12 | COMM_SET_DUTY … SET_SERVO_POS | 各类控制设定 |
| 13-18 | COMM_SET/GET_MCCONF / APPCONF | 电机/应用配置读写 |
| 19 | COMM_SAMPLE_PRINT | 采样数据 |
| 20 | COMM_TERMINAL_CMD | 终端命令 |
| 21 | COMM_PRINT | 文本输出（设备→上位机）|
| 22 | COMM_ROTOR_POSITION | 转子位置 |
| 24-28 | COMM_DETECT_* | 电机参数/编码器/霍尔检测 |
| 29 | COMM_REBOOT | 重启 |
| 30 | COMM_ALIVE | 心跳 |
| 31-33 | COMM_GET_DECODED_PPM/ADC/CHUK | 解码后的遥控输入 |
| 34 | COMM_FORWARD_CAN | CAN 转发 |
| 35 | COMM_SET_CHUCK_DATA | Nunchuk 数据 |
| 36 | COMM_CUSTOM_APP_DATA | 自定义透传 |
| 38-46 | COMM_GPD_* | 通用脉冲驱动 |
| 47 | COMM_GET_VALUES_SETUP | 整车实时值 |
| 48-49 | COMM_SET_MCCONF_TEMP* | 临时配置 |
| 50-51 | COMM_GET_VALUES_*_SELECTIVE | 按掩码选择字段 |
| 52-56 | COMM_EXT_NRF_* | NRF 无线 |
| 57-58 | COMM_DETECT_* | 磁链/FOC 全自动检测 |
| 59-62 | …_ALL_CAN / PING_CAN | 全 CAN 广播操作 |
| 63 | COMM_APP_DISABLE_OUTPUT | 禁用输出 |
| 65 | COMM_GET_IMU_DATA | IMU 数据 |
| 66-70 | COMM_BM_* | Bootloader/烧录 |
| 75-79 | COMM_PLOT_* / GET_DECODED_BALANCE | 绘图/平衡 |
| 84 | COMM_SET_CURRENT_REL | 相对电流 |
| 85 | COMM_CAN_FWD_FRAME | 原始 CAN 帧转发 |
| 86 | COMM_SET_BATTERY_CUT | 电池截止电压 |
| 87-89 | COMM_SET_BLE_* / CAN_MODE | 蓝牙/CAN 模式 |
| 90-91 | COMM_GET_IMU_CALIBRATION / MCCONF_TEMP | 标定/临时配置 |
| 92-95 | COMM_*_CUSTOM_CONFIG | 自定义配置 |
| 96-101 | COMM_BMS_* | 电池管理系统 |
| 110 | COMM_SET_ODOMETER | 设置里程 |
| 111-112 | COMM_PSW_* | 电源开关板 |
| 117-118 | COMM_GET_QML_UI_HW/APP | QML 界面资源 |
| 122-124 | COMM_IO_BOARD_* | IO 扩展板 |
| 128-129 | COMM_GET/RESET_STATS | 统计信息 |
| 130-139 | COMM_LISP_* | LispBM 脚本 |
| 140-144 | COMM_FILE_* | 文件系统操作 |
| 145-148 | COMM_LOG_* | 数据记录 |
| 150 | COMM_GET_GNSS | GNSS 定位 |
| 156 | COMM_SHUTDOWN | 关机 |
| 157 | COMM_FW_INFO | 固件信息 |
| 159 | COMM_MOTOR_ESTOP | 紧急停止 |

> 完整 160 个枚举请直接查阅 [datatypes.h](../datatypes.h#L955)。

---

## 十、关键常量速查

| 项目 | 值 | 出处 |
|------|-----|------|
| 起始字节 | `2` / `3` / `4`（指示长度字段宽度）| [packet.c:48-56](packet.c#L48-L56) |
| 停止字节 | `3`（`0x03`）| [packet.c:68](packet.c#L68) |
| CRC 算法 | CRC16-CCITT，poly `0x1021`，init `0x0000`，大端 | [crc.c:26-65](../util/crc.c#L26-L65) |
| CRC 范围 | 仅 Payload | [packet.c:65](packet.c#L65) |
| 最大 Payload | `512`（`PACKET_MAX_PL_LEN`，可重定义）| [packet.h:28](packet.h#L28) |
| 缓冲区大小 | `PACKET_MAX_PL_LEN + 8` | [packet.h:31](packet.h#L31) |
| 数值字节序 | 大端（Big-Endian）| [buffer.c:24](../util/buffer.c#L24) |
| 零长度帧 | 不支持 | [packet.c:42](packet.c#L42) |
| USB VID/PID | `0x0483` / `0x5740`（ST CDC）| [comm_usb_serial.c:28-29](comm_usb_serial.c#L28-L29) |

---

## 十一、交互示例

### 示例 1：查询固件版本

上位机发送 `COMM_FW_VERSION`（payload 仅 1 字节 `0x00`）：

```
帧：  02 01 00 [CRC_H] [CRC_L] 03
      │  │  │  └────────┘      └ 停止字节
      │  │  └ payload = COMM_FW_VERSION(0)
      │  └ 长度 = 1
      └ 起始字节（短帧）
```

CRC = `crc16({0x00}, 1)`。设备回复一帧 payload 以 `0x00` 开头，含版本号、硬件名、UUID 等（见 [8.2](#82-comm_fw_version-id0--固件版本查询)）。

### 示例 2：设置电流 10.0 A

`COMM_SET_CURRENT`(6)，参数 int32 = 10.0 × 1000 = 10000 = `0x00002710`：

```
payload： 06 00 00 27 10
          │  └──────────┘ int32 大端 = 10000
          └ 命令 ID

整帧：    02 05 06 00 00 27 10 [CRC_H] [CRC_L] 03
          │  │  └ payload(5字节) ─┘
          │  └ 长度 = 5
          └ 短帧起始
```

CRC 对 5 字节 payload `06 00 00 27 10` 计算。此命令无回复，需持续发 `COMM_ALIVE` 维持。

### 示例 3：通过 CAN 转发控制 ID=12 的从机设置占空比 0.5

`COMM_SET_DUTY`(5)，duty = 0.5 × 100000 = 50000 = `0x0000C350`，外层用 `COMM_FORWARD_CAN`(34) 包裹：

```
内层命令： 05 00 00 C3 50
外层payload： 22 0C 05 00 00 C3 50
             │  │  └ 被转发的完整命令
             │  └ 目标 CAN ID = 12 (0x0C)
             └ COMM_FORWARD_CAN(34=0x22)

整帧：    02 07 22 0C 05 00 00 C3 50 [CRC_H] [CRC_L] 03
```

---

## 附：伪代码（上位机发送一帧）

```python
def vesc_send(payload: bytes) -> bytes:
    n = len(payload)
    if n <= 255:
        header = bytes([2, n])
    elif n <= 65535:
        header = bytes([3, (n >> 8) & 0xFF, n & 0xFF])
    else:
        header = bytes([4, (n >> 16) & 0xFF, (n >> 8) & 0xFF, n & 0xFF])
    crc = crc16_ccitt(payload)          # poly=0x1021, init=0x0000
    return header + payload + bytes([(crc >> 8) & 0xFF, crc & 0xFF, 3])
```

> 接收端则逐字节喂入状态机，校验起止字节、长度、CRC 后取出 payload，首字节即命令 ID。



