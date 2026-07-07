# joint_proto 协议栈文件逻辑关系总图

> 本文档梳理 `joint_proto` 协议栈**从顶层线程入口到 HAL 驱动**的完整文件调用链与依赖关系，
> 涵盖 `Protocol/`、`Devices/`、`AppServices/`、`Common/`、`Driver/`、`DataHub/`、`MotorControl/`、
> `MotorCalibration/` 等多个目录。建议配合以下文档参阅：
> - [joint_proto_layer_diagram.md](joint_proto_layer_diagram.md)：协议四层架构总览
> - [joint_proto_解析流程说明.md](joint_proto_解析流程说明.md)：命令分发与载荷解析详解
>
> **渲染**:VS Code / Trae 安装 "Markdown Preview Mermaid Support" 插件，或推送至 GitHub 可直接渲染 Mermaid 图。

---

## 目录

- [1. 全栈文件调用关系总图](#1-全栈文件调用关系总图)
- [2. 文件分层与职责速查](#2-文件分层与职责速查)
- [3. RX 数据通路：字节流到电机动作](#3-rx-数据通路字节流到电机动作)
- [4. TX 数据通路：反馈/遥测到串口字节流](#4-tx-数据通路反馈遥测到串口字节流)
- [5. 逐文件调用关系详图](#5-逐文件调用关系详图)
  - [5.1 thread_commun.c — 通信线程入口](#51-thread_communc--通信线程入口)
  - [5.2 jm_host_commun.c — 通信服务接入层](#52-jm_host_communc--通信服务接入层)
  - [5.3 dev_commun_uart.c — 设备粘合层](#53-dev_commun_uartc--设备粘合层)
  - [5.4 jm_proto_uart.c — 串口绑定层](#54-jm_proto_uartc--串口绑定层)
  - [5.5 packer_parser.c — 组拆帧器](#55-packer_parserc--组拆帧器)
  - [5.6 jm_proto.c — 协议核心分发](#56-jm_protoc--协议核心分发)
  - [5.7 jm_proto_ops.c — 业务回调实现](#57-jm_proto_opsc--业务回调实现)
- [6. 头文件 include 依赖图](#6-头文件-include-依赖图)
- [7. 关键数据结构在各文件间的流转](#7-关键数据结构在各文件间的流转)
- [8. 回调注入链路](#8-回调注入链路)
- [9. 编译开关与条件包含](#9-编译开关与条件包含)

---

## 1. 全栈文件调用关系总图

下图展示 `joint_proto` 协议栈涉及的**全部文件**及其调用关系（实线=函数调用，虚线=回调注入，点线=头文件依赖）。

```mermaid
flowchart TB
    subgraph THREAD["AppServices/ThreadManager/ — 线程层"]
        TC["thread_commun.c<br/>━━━━━━━<br/>commun_thread()<br/>jm_host_commun_init()<br/>jm_host_commun_process()<br/>jm_host_commun_wait()"]
    end

    subgraph PARAMSVC["AppServices/ParamService/ — 通信服务接入层"]
        JHC["jm_host_commun.c<br/>━━━━━━━<br/>jm_host_commun_init/process/wait<br/>commun_uart_pack_telemetry<br/>commun_uart_push_telemetry"]
    end

    subgraph DEVS["Devices/ — 设备粘合层"]
        DCU["dev_commun_uart.c/.h<br/>━━━━━━━<br/>dev_commun_uart_init<br/>set_ops/start/poll/report/on_rx_idle"]
        DCV["dev_commun_vesc.c/.h<br/>(VESC 旁路, 不在本文档主链)"]
    end

    subgraph PROTO["Protocol/joint_proto/ — 协议核心"]
        JPU["jm_proto_uart.c/.h<br/>━━━━━━━<br/>jm_proto_uart_init/feed/send"]
        JPC["jm_proto_can.c/.h<br/>(CAN 旁路, 共用 dispatch)"]
        JP["jm_proto.c/.h<br/>━━━━━━━<br/>jm_proto_init<br/>jm_proto_dispatch"]
        JPO["jm_proto_ops.c/.h<br/>━━━━━━━<br/>jm_app_ops_get<br/>jm_app_get_feedback<br/>jm_app_telemetry_*"]
        JCD["jm_cmd_def.h<br/>(纯定义, 无实现)"]
        EX["example_jm_proto_uart.c<br/>(不参与编译, 仅示例)"]
    end

    subgraph PACKER["Protocol/packer_parser/ — 组拆帧器"]
        PP["packer_parser.c/.h<br/>━━━━━━━<br/>upacker_init/set_cb<br/>upacker_pack/unpack"]
    end

    subgraph COMMON["Common/ — 通用工具"]
        CRC["crc16.c/.h<br/>━━━━━━━<br/>crc16_calc()"]
        ASR["assert_report.c/.h<br/>assert_report()"]
    end

    subgraph DRV["Driver/ — HAL 驱动"]
        DU["drv_usart.c/.h<br/>━━━━━━━<br/>usart_idle_init<br/>usart_idle_get_data<br/>drv_usart_send<br/>drv_uart_idle"]
        DRT["drv_rtos.c/.h<br/>drv_rtos_sem_*/delay_ms"]
    end

    subgraph HUB["DataHub/ — 全局数据中心"]
        RP["runtime_param.c/.h<br/>━━━━━━━<br/>usr (全局实例)<br/>motor_state_t / motor_param_t"]
        MP["motor_param.c/.h<br/>━━━━━━━<br/>MOTOR_PARAM_PARAM_COUNT<br/>字段表工具"]
        MI["motor_info.c/.h<br/>━━━━━━━<br/>motor_info_t / 6 子块"]
        VER["version.h<br/>HW_/APP_/BOOT_ VERSION"]
    end

    subgraph CTRL["MotorControl/ — 控制业务"]
        ML["motor_loop.c/.h<br/>━━━━━━━<br/>motor_loop_get()<br/>motor_loop_set_cmd()"]
    end

    subgraph CALIB["MotorCalibration/ — 标定模块"]
        CM["calib_mgr.c/.h<br/>━━━━━━━<br/>calib_mgr_start<br/>calib_mgr_get_status<br/>calib_mgr_abort"]
    end

    subgraph CFG["Config/ — 配置层"]
        DC2["dev_config.c/.h<br/>USE_DEV_COMMUN_UART<br/>commun_uart_list[]"]
    end

    TC -->|"调用"| JHC
    JHC -->|"驱动设备"| DCU
    JHC -->|"读遥测状态/反馈"| JPO
    JPU -.->|"注入 ops"| DCU
    DCU -->|"feed/send"| JPU
    JPU -->|"dispatch"| JP
    JP -.->|"回调 ops->xxx"| JPO
    JPU -->|"组拆帧"| PP
    PP -->|"CRC16 校验"| CRC
    DCU -->|"串口收发"| DU
    JHC -->|"信号量/延时"| DRT
    TC -->|"VESC 旁路"| DCV

    JPO -->|"写控制目标"| ML
    JPO -->|"读活数据"| RP
    JPO -->|"查参数表"| MP
    JPO -->|"读写电机配置"| MI
    JPO -->|"版本号"| VER
    JPO -->|"标定控制"| CM
    JPO -->|"jm_dbg[]"| RP

    DCU -->|"配置表"| CFG
    DCU -->|"断言"| ASR
    JPU --> JCD
    JP --> JCD
    JPC --> JCD
    JPO --> JCD

    classDef thread fill:#e1f5ff,stroke:#0288d1
    classDef proto fill:#fff4e1,stroke:#f57c00
    classDef packer fill:#f3e5f5,stroke:#7b1fa2
    classDef dev fill:#e8f5e9,stroke:#388e3c
    classDef hub fill:#fce4ec,stroke:#c2185b
    classDef ctrl fill:#f1f8e9,stroke:#689f38
    class TC thread
    class JHC thread
    class JP,JPO,JPU,JPC,JCD,EX proto
    class PP packer
    class DCU,DCV dev
    class RP,MP,MI,VER hub
    class ML,CM ctrl
```

---

## 2. 文件分层与职责速查

按数据流方向（自顶向下为 RX 入口 → HAL，自底向上为业务回调），各文件分层如下：

| 层 | 文件 | 职责 | 关键导出 |
|----|------|------|----------|
| **L5 线程入口** | `thread_commun.c` | 通信线程主循环，调度 host | `commun_thread()` |
| **L4 服务接入** | `jm_host_commun.c` | 装配设备+注入ops+遥测分频打包 | `jm_host_commun_init/process/wait/notify_rx` |
| **L3 设备粘合** | `dev_commun_uart.c` | HAL↔协议栈桥接，DMA 取数与发送 | `dev_commun_uart_init` + 5 个方法指针 |
| **L2 串口绑定** | `jm_proto_uart.c` | packer 组拆帧 + 调 dispatch + 封帧回送 | `jm_proto_uart_init/feed/send` |
| **L2' 组拆帧** | `packer_parser.c` | 字节流状态机 + CRC16 校验 | `upacker_init/set_cb/pack/unpack` |
| **L2'' 通用工具** | `crc16.c` | CRC16 算法 | `crc16_calc()` |
| **L1 协议核心** | `jm_proto.c` | 传输无关的命令分发 + 应答组织 | `jm_proto_init/dispatch` |
| **L0 定义** | `jm_cmd_def.h` | 命令码/错误码/类型码/宏 | （纯宏与枚举） |
| **L3' 业务回调** | `jm_proto_ops.c` | ops 接口实现，对接电机控制/参数 | `jm_app_ops_get/get_feedback/telemetry_*` |
| **L6 HAL 驱动** | `drv_usart.c` | USART+DMA+IDLE 收发 | `usart_idle_init/get_data`、`drv_usart_send` |
| **L6' RTOS** | `drv_rtos.c` | 信号量/延时 | `drv_rtos_sem_*` |
| **L7 数据中心** | `runtime_param.c` / `motor_param.c` / `motor_info.c` / `version.h` | 活数据/参数表/配置/版本 | `usr`、`s_param_tbl`、`motor_info_*` |
| **L7' 控制业务** | `motor_loop.c` | 控制目标写入与状态机触发 | `motor_loop_get/set_cmd` |
| **L7'' 标定** | `calib_mgr.c` | 标定启动/查询/中止 | `calib_mgr_start/get_status/abort` |

---

## 3. RX 数据通路：字节流到电机动作

下图展示一帧命令从串口字节到触发电机动作的**完整调用栈**。每一步标注调用方→被调方→函数名。

```mermaid
sequenceDiagram
    participant IRQ as USART IDLE IRQ
    participant TC as thread_commun.c
    participant JHC as jm_host_commun.c
    participant DCU as dev_commun_uart.c
    participant JPU as jm_proto_uart.c
    participant PP as packer_parser.c
    participant CRC as crc16.c
    participant JP as jm_proto.c
    participant JPO as jm_proto_ops.c
    participant ML as motor_loop.c
    participant RP as runtime_param.c

    Note over IRQ: 硬件层: USART IDLE 触发
    IRQ->>DCU: on_rx_idle() (IRQ 上下文)
    DCU->>DCU: drv_uart_idle(uart) 锁存本帧长度

    Note over TC: 线程上下文
    TC->>JHC: jm_host_commun_process()
    JHC->>DCU: dev.poll()
    DCU->>DCU: usart_idle_get_data(uart, rx_tmp, &rx_len)
    DCU->>JPU: jm_proto_uart_feed(&jm, rx_tmp, rx_len)
    JPU->>PP: upacker_unpack(&packer, buf, len)
    loop 逐字节状态机
        PP->>PP: frame_decode(packer, byte)
        PP->>CRC: crc16_calc(data, flen) (CRC 阶段)
        CRC-->>PP: crc 值
    end
    PP->>JPU: cb(data, flen) [即 on_frame 回调]
    JPU->>JP: jm_proto_dispatch(&proto, cmd, payload, len)
    JP->>JP: 主分发决策树 (见解析文档§3)
    JP->>JPO: ops->set_mode(cmd, payload, len)
    JPO->>JPO: jm_rd_f32() 解析载荷
    JPO->>ML: motor_loop_get()->sys.motor.cmd = 目标量
    JPO->>ML: motor_loop_set_cmd((ctrl_mode_e)cmd)
    JPO-->>JP: JM_ERR_OK
    JP->>JP: reply_ack(cmd, 0)
    JP-->>JPU: reply_len=2, reply=[cmd,0]
    JPU->>PP: upacker_pack(&packer, reply, reply_len)
    PP->>PP: frame_encode() 组帧 + CRC
    PP->>JPU: send回调 [即 on_send]
    JPU->>DCU: jm_uart_tx(frame, len) [注入的发送函数]
    DCU->>DCU: drv_usart_send(uart, data, len, timeout)
    Note over DCU: DMA 发送应答帧
```

---

## 4. TX 数据通路：反馈/遥测到串口字节流

下图展示**周期遥测 0xCA**（电机→上位机，无应答）的调用栈，体现主动上报路径。

```mermaid
sequenceDiagram
    participant TC as thread_commun.c
    participant JHC as jm_host_commun.c
    participant JPO as jm_proto_ops.c
    participant RP as runtime_param.c
    participant DCU as dev_commun_uart.c
    participant JPU as jm_proto_uart.c
    participant PP as packer_parser.c
    participant DU as drv_usart.c

    TC->>JHC: jm_host_commun_process()
    JHC->>JHC: 遥测分频到点
    JHC->>JPO: jm_app_telemetry_enabled()
    JPO->>RP: 读静态 enable
    JPO-->>JHC: 1=已使能
    JHC->>JPO: jm_app_telemetry_mask()
    JPO-->>JHC: mask
    JHC->>JPO: jm_app_get_feedback(&fb)
    JPO->>RP: 读 usr.motor_state[M1]
    RP-->>JPO: motor_state_t
    JPO-->>JHC: jm_feedback_t fb
    JHC->>JHC: commun_uart_pack_telemetry(mask, &fb, o)
    Note over JHC: 按 jm_telemetry_bit_e 位序变长拼接<br/>帧体 = mask(u16) + 各组数据
    JHC->>DCU: dev.report(JM_CMD_TELEMETRY, o, n)
    DCU->>JPU: jm_proto_uart_send(&jm, cmd, body, len)
    JPU->>PP: upacker_pack(&packer, frame, 1+len)
    PP->>PP: frame_encode 组帧(帧头+长度+校验+CRC)
    PP->>JPU: on_send 回调
    JPU->>DCU: jm_uart_tx(frame, len)
    DCU->>DU: drv_usart_send(uart, data, len, timeout)
    Note over DU: DMA 发送遥测帧
```

---

## 5. 逐文件调用关系详图

### 5.1 thread_commun.c — 通信线程入口

**位置**：`User/AppServices/ThreadManager/thread_commun.c`

```mermaid
flowchart LR
    TC["thread_commun.c<br/>commun_thread()"]
    TC -->|"初始化"| JHCI["jm_host_commun_init()<br/>(jm_host_commun.c)"]
    TC -->|"周期处理"| JHCP["jm_host_commun_process()<br/>(jm_host_commun.c)"]
    TC -->|"等待/延时"| JHCW["jm_host_commun_wait()<br/>(jm_host_commun.c)"]
    TC -.->|"VESC 旁路"| DCV["dev_commun_vesc_*<br/>(dev_commun_vesc.c)"]
    TC -.->|"VOFA 旁路(已注释)"| VOFA["vofa_upload<br/>(vofa.c)"]
```

**调用关系**：
- `jm_host_commun_init()` → 装配设备 + 注入 ops + 启动接收
- `jm_host_commun_process()` → 每拍 poll + 遥测分频
- `jm_host_commun_wait(THREAD_DELAY_COMMUN)` → 信号量等待（RX 唤醒或超时）

**include 依赖**：`drv_rtos.h`、`runtime_param.h`、`thread_config.h`、`thread_commun.h`、`vofa.h`、`main.h`、`dev_commun_vesc.h`、`jm_host_commun.h`

---

### 5.2 jm_host_commun.c — 通信服务接入层

**位置**：`User/AppServices/ParamService/jm_host_commun.c`

```mermaid
flowchart LR
    JHC["jm_host_commun.c"]
    JHC -->|"init: 装配设备"| DCU_I["dev_commun_uart_init()<br/>(dev_commun_uart.c)"]
    JHC -->|"init: 注入 ops"| DCU_S["dev.set_ops(jm_app_ops_get())<br/>(dev_commun_uart.c)"]
    JHC -->|"init: 启动"| DCU_ST["dev.start()<br/>(dev_commun_uart.c)"]
    JHC -->|"process: poll"| DCU_P["dev.poll()<br/>(dev_commun_uart.c)"]
    JHC -->|"process: 遥测打包"| JPO_FB["jm_app_get_feedback()<br/>(jm_proto_ops.c)"]
    JHC -->|"process: 遥测门控"| JPO_TE["jm_app_telemetry_enabled/mask/period()<br/>(jm_proto_ops.c)"]
    JHC -->|"process: 上报"| DCU_R["dev.report()<br/>(dev_commun_uart.c)"]
    JHC -->|"wait: 信号量"| DRT["drv_rtos_sem_wait()<br/>(drv_rtos.c)"]
    JHC -->|"notify_rx: 释放信号量"| DRT2["drv_rtos_sem_release()<br/>(drv_rtos.c)"]
```

**核心逻辑**：
- `jm_host_commun_init()`：创建 RX 信号量 → 初始化设备 → 注入 `jm_app_ops_get()` 单例 → 启动 DMA 接收
- `jm_host_commun_process()`：调 `dev.poll()` 驱动命令分发 → 检查遥测使能 → 按 `period_ms` 分频 → 调 `commun_uart_push_telemetry()` 打包上报
- `commun_uart_pack_telemetry()`：按 `jm_telemetry_bit_e` 位序变长拼接 10 组数据

**include 依赖**：`runtime_param.h`、`thread_config.h`、`drv_rtos.h`、`dev_commun_uart.h`、`jm_proto_ops.h`

---

### 5.3 dev_commun_uart.c — 设备粘合层

**位置**：`User/Devices/dev_commun_uart.c`

```mermaid
flowchart LR
    DCU["dev_commun_uart.c"]
    DCU -->|"init: 读配置表"| CFG["commun_uart_list[id]<br/>(dev_config.c)"]
    DCU -->|"start: 协议栈装配"| JPU_I["jm_proto_uart_init()<br/>(jm_proto_uart.c)"]
    DCU -->|"start: 启动 DMA 接收"| DU_I["usart_idle_init()<br/>(drv_usart.c)"]
    DCU -->|"on_rx_idle: IRQ 锁存"| DU_IDL["drv_uart_idle()<br/>(drv_usart.c)"]
    DCU -->|"poll: 取数据"| DU_GET["usart_idle_get_data()<br/>(drv_usart.c)"]
    DCU -->|"poll: 喂协议栈"| JPU_F["jm_proto_uart_feed()<br/>(jm_proto_uart.c)"]
    DCU -->|"report: 主动上报"| JPU_S["jm_proto_uart_send()<br/>(jm_proto_uart.c)"]
    DCU -.->|"发送回调(被 JPU 回调)"| DU_TX["drv_usart_send()<br/>(drv_usart.c)"]
    DCU -.->|"断言"| ASR["assert_report()<br/>(assert_report.c)"]
```

**关键设计**：
- `jm_uart_tx()` 静态函数作为发送回调注入 `jm_proto_uart_init`，被 `jm_proto_uart.c` 的 `on_send` 回调
- `s_active` 模块级指针：因 packer 回调无上下文参数，用活动实例指针路由到正确串口（单链路设计）
- `poll()` 取数后立即 `jm_proto_uart_feed()`，**在同一线程上下文**完成 dispatch + 应答发送

**include 依赖**：`dev_config.h`（含 `USE_DEV_COMMUN_UART`）、`drv_usart.h`、`jm_proto_uart.h`、`assert_report.h`

---

### 5.4 jm_proto_uart.c — 串口绑定层

**位置**：`User/Protocol/joint_proto/jm_proto_uart.c`

```mermaid
flowchart LR
    JPU["jm_proto_uart.c"]
    JPU -->|"init: 初始化协议核心"| JP_I["jm_proto_init()<br/>(jm_proto.c)"]
    JPU -->|"init: 初始化 packer"| PP_I["upacker_init()<br/>(packer_parser.c)"]
    JPU -->|"init: 注册回调"| PP_CB["upacker_set_cb(on_frame, on_send)<br/>(packer_parser.c)"]
    JPU -->|"feed: 喂字节"| PP_UN["upacker_unpack()<br/>(packer_parser.c)"]
    JPU -.->|"feed 内回调 on_frame"| JP_D["jm_proto_dispatch()<br/>(jm_proto.c)"]
    JPU -.->|"feed 内回调 on_frame: 应答封帧"| PP_P["upacker_pack()<br/>(packer_parser.c)"]
    JPU -->|"send: 主动上报封帧"| PP_P2["upacker_pack()<br/>(packer_parser.c)"]
    JPU -.->|"packer 发送回调 on_send"| TX["jm_uart_tx_fn<br/>(dev_commun_uart.c 注入)"]
```

**回调闭环**：
1. `feed()` → `upacker_unpack()` → 逐字节状态机
2. 收齐整帧 → packer 调 `on_frame()` 回调
3. `on_frame()` → `jm_proto_dispatch()` → 业务处理 → 应答写入 `proto->reply`
4. `on_frame()` 检查 `reply_len > 0` → `upacker_pack()` 封帧
5. packer 封帧后调 `on_send()` 回调 → `jm_uart_tx_fn` 发出

**include 依赖**：`jm_proto.h`、`packer_parser.h`

---

### 5.5 packer_parser.c — 组拆帧器

**位置**：`User/Protocol/packer_parser/packer_parser.c`

```mermaid
flowchart LR
    PP["packer_parser.c"]
    PP -->|"CRC16 校验"| CRC["crc16_calc()<br/>(crc16.c)"]
    PP -.->|"整帧接收回调"| CB["PACKER_CB cb<br/>(jm_proto_uart.c 的 on_frame)"]
    PP -.->|"封包发送回调"| SEND["PACKER_CB send<br/>(jm_proto_uart.c 的 on_send)"]
```

**核心实现**：
- `frame_decode()`：8 态状态机（HEADER_HIGH → HEADER_LOW → LEN_HIGH → LEN_LOW → HEADER_CHK → DATA_RECV → CRC_LOW → CRC_HIGH）
- `frame_encode()`：组帧为 `[0xA5][0x5A][LEN_HI][LEN_LO][HDR_CHK][DATA...][CRC_LO][CRC_HI]`
- `send_buffer[SEND_BUF_SIZE]`：**单全局静态缓冲**（约束：每个上报节拍只发一帧，无发送忙查询）
- 头校验：`(STX_H + STX_L + LEN_HI + LEN_LO) & 0xFF`
- 数据校验：`crc16_calc(data, flen)`，小端落帧

**include 依赖**：`packer_parser.h`、`crc16.h`、`string.h`

> 注：`packer_parser` 是协议栈中**唯一依赖 `crc16.c`**的文件，CRC16 校验逻辑集中于此。

---

### 5.6 jm_proto.c — 协议核心分发

**位置**：`User/Protocol/joint_proto/jm_proto.c`

```mermaid
flowchart LR
    JP["jm_proto.c"]
    JP -.->|"回调 ops->set_mode"| JPO_SM["jm_proto_ops.c: app_set_mode<br/>(0x00~0xB8)"]
    JP -.->|"回调 ops->get_feedback"| JPO_GF["jm_proto_ops.c: app_get_feedback<br/>(0xC0~0xC8)"]
    JP -.->|"回调 ops->param_read/write/save/reset"| JPO_PR["jm_proto_ops.c: app_param_*<br/>(0xE0~0xE5)"]
    JP -.->|"回调 ops->motor_info_*"| JPO_MI["jm_proto_ops.c: app_motor_info_*<br/>(0xE6~0xEB)"]
    JP -.->|"回调 ops->get_dev_info/name"| JPO_DI["jm_proto_ops.c: app_get_dev_*<br/>(0xD0~0xD2)"]
    JP -.->|"回调 ops->get_debug"| JPO_DBG["jm_proto_ops.c: app_get_debug<br/>(0xC9)"]
    JP -.->|"回调 ops->set_telemetry"| JPO_TL["jm_proto_ops.c: app_set_telemetry<br/>(0xCB)"]
    JP -.->|"回调 ops->set_can_id/baudrate"| JPO_CAN["jm_proto_ops.c: app_set_can_*<br/>(0xF0~0xF1)"]
    JP -->|"标定查询直接调用"| CM["calib_mgr_get_status()<br/>(calib_mgr.c)"]
```

**关键设计**：
- `dispatch` 是**唯一入口**，串口/CAN 共用
- 应答组织三件套：`reply_set` / `reply_ack` / `reply_nack`（均写入 `proto->reply[]` 与 `reply_len`）
- `pack_feedback()` 把 `jm_feedback_t` 打包为 22B 全精度应答（仅 UART 路径用，CAN 路径自行重新压缩）
- 0x97 标定查询**绕过 ops**，直接调 `calib_mgr_get_status()` 返回 8B 详细状态

**include 依赖**：`jm_proto.h`、`calib_mgr.h`、`string.h`

---

### 5.7 jm_proto_ops.c — 业务回调实现

**位置**：`User/Protocol/joint_proto/jm_proto_ops.c`

```mermaid
flowchart LR
    JPO["jm_proto_ops.c"]
    JPO -->|"set_mode: 写控制目标"| ML["motor_loop.c<br/>motor_loop_get()->sys.motor.cmd<br/>motor_loop_set_cmd()"]
    JPO -->|"get_feedback: 读活数据"| RP["runtime_param.c<br/>usr.motor_state[M1]"]
    JPO -->|"param_read/write: 查表"| MP["motor_param.c<br/>s_param_tbl[id]<br/>MOTOR_PARAM_PARAM_COUNT"]
    JPO -->|"param_read/write: 读写实例"| RP2["runtime_param.c<br/>usr.motor_param[M1]"]
    JPO -->|"param_save: 弱符号落Flash"| FLASH["jm_app_param_storage_save()<br/>(可被驱动层强符号覆盖)"]
    JPO -->|"param_reset: 恢复默认"| MP2["motor_param.c<br/>motor_param_init()"]
    JPO -->|"motor_info_read/write"| MI["motor_info.c<br/>motor_info_dispatch_read/write()"]
    JPO -->|"motor_info_save"| MIS["motor_info_storage.c<br/>motor_info_storage_get()"]
    JPO -->|"get_dev_info: 版本号"| VER["version.h<br/>HW_/APP_/BOOT_ VERSION"]
    JPO -->|"get_dev_info: UID"| RP3["runtime_param.c<br/>usr.sys.info.device_uid"]
    JPO -->|"get_dev_name"| MP3["motor_param.c<br/>motor_param_get_motor_name()"]
    JPO -->|"get_debug: jm_dbg[]"| RP4["runtime_param.c<br/>jm_dbg[]"]
    JPO -->|"set_can_id"| MP4["motor_param.c<br/>motor_param_set_motor_id()"]
    JPO -->|"calib start/abort"| CM["calib_mgr.c<br/>calib_mgr_start/abort()"]
    JPO -->|"telemetry 状态(静态变量)"| ST["本文件静态<br/>s_tlm_enable/mask/period"]
    JPO -->|"motor_profile 覆盖"| MPR["motor_profile.h<br/>motor_profile_apply_param/info"]
```

**核心数据通路**：
- **参数读写双依赖**：`motor_param.c` 提供元数据（字段表 `s_param_tbl`：offset/type/size），`runtime_param.c` 提供活数据（`usr.motor_param[M1]` 实例内存）。读写时先查表得 offset，再按 offset 直接内存拷贝（Cortex-M 小端，与协议小端一致，无需转换）。
- **反馈读取**：直接从 `usr.motor_state[M1]` 的 `motion/electrical/power/thermal/fault` 子结构填充 `jm_feedback_t`。
- **标定控制**：`set_mode` 收到 0x90~0x96 时调 `calib_mgr_start(level, submode)`，失败按状态分类返回 BUSY/STATE_DENY/OUT_OF_RANGE。

**include 依赖**：`jm_proto_ops.h`、`runtime_param.h`、`motor_param.h`、`motor_profile.h`、`version.h`、`motor_loop.h`、`motor_info.h`、`motor_info_storage.h`、`calib_mgr.h`

---

## 6. 头文件 include 依赖图

下图展示协议栈相关头文件的依赖关系（自上而下为依赖方向）。

```mermaid
flowchart TB
    subgraph APP["应用层头文件"]
        JHC_H["jm_host_commun.h"]
        JPO_H["jm_proto_ops.h"]
    end

    subgraph DEV["设备层头文件"]
        DCU_H["dev_commun_uart.h"]
        DCV_H["dev_commun_vesc.h"]
    end

    subgraph PROTO_H["协议层头文件"]
        JPU_H["jm_proto_uart.h"]
        JPC_H["jm_proto_can.h"]
        JP_H["jm_proto.h"]
        JCD_H["jm_cmd_def.h"]
    end

    subgraph PACK_H["组拆帧头文件"]
        PP_H["packer_parser.h"]
    end

    subgraph DRV_H["驱动层头文件"]
        DU_H["drv_usart.h"]
        DRT_H["drv_rtos.h"]
    end

    subgraph COMMON_H["通用头文件"]
        CRC_H["crc16.h"]
        ASR_H["assert_report.h"]
    end

    subgraph HUB_H["数据中心头文件"]
        RP_H["runtime_param.h"]
        MP_H["motor_param.h"]
        MI_H["motor_info.h"]
        VER_H["version.h"]
    end

    subgraph CTRL_H["控制层头文件"]
        ML_H["motor_loop.h"]
        CM_H["calib_mgr.h"]
    end

    subgraph CFG_H["配置头文件"]
        DC_CFG["dev_config.h<br/>(USE_DEV_COMMUN_UART)"]
    end

    JHC_H --> DCU_H
    JHC_H --> JPO_H
    JHC_H --> DRT_H

    DCU_H --> DC_CFG
    DCU_H --> DU_H
    DCU_H --> JPU_H

    JPU_H --> JP_H
    JPU_H --> PP_H

    JPC_H --> JP_H

    JP_H --> JCD_H

    JPO_H --> JP_H

    PP_H --> CRC_H

    RP_H --> ML_H
    RP_H --> VER_H
    MP_H --> RP_H

    JPO_H -.->|"实现依赖(.c)"| RP_H
    JPO_H -.->|"实现依赖(.c)"| MP_H
    JPO_H -.->|"实现依赖(.c)"| MI_H
    JPO_H -.->|"实现依赖(.c)"| ML_H
    JPO_H -.->|"实现依赖(.c)"| CM_H

    JP_H -.->|"实现依赖(.c)"| CM_H
```

> 实线=头文件直接 include；虚线=.c 实现文件依赖（非头文件传递）。

---

## 7. 关键数据结构在各文件间的流转

下图展示协议栈中几个核心数据结构**在哪个文件产生、在哪个文件消费**。

```mermaid
flowchart LR
    subgraph SRC["数据产生方"]
        RP_GEN["runtime_param.c<br/>usr.motor_state[M1]"]
        RP_GEN2["runtime_param.c<br/>usr.motor_param[M1]"]
        RP_GEN3["runtime_param.c<br/>jm_dbg[]"]
        ML_GEN["motor_loop.c<br/>sys.motor.cmd"]
        CM_GEN["calib_mgr.c<br/>calib_status_t"]
        VER_GEN["version.h<br/>HW_/APP_"]
    end

    subgraph CONV["协议层转换"]
        JPO_C["jm_proto_ops.c<br/>填充/查表"]
        JP_C["jm_proto.c<br/>打包应答"]
        JHC_C["jm_host_commun.c<br/>遥测打包"]
    end

    subgraph OUT["输出方"]
        JPU_O["jm_proto_uart.c<br/>reply[] / feed"]
        PP_O["packer_parser.c<br/>帧编码"]
        DCU_O["dev_commun_uart.c<br/>DMA 收发"]
    end

    RP_GEN -->|"motor_state_t"| JPO_C
    RP_GEN2 -->|"motor_param_t 内存"| JPO_C
    RP_GEN3 -->|"jm_dbg[]"| JPO_C
    ML_GEN -->|"motor_cmd_t 写"| JPO_C
    CM_GEN -->|"calib_status_t"| JP_C
    VER_GEN -->|"版本号"| JPO_C

    JPO_C -->|"jm_feedback_t"| JP_C
    JPO_C -->|"jm_feedback_t"| JHC_C
    JPO_C -->|"param_id+offset"| JP_C

    JP_C -->|"reply[1+256]"| JPU_O
    JHC_C -->|"telemetry body[]"| DCU_O

    JPU_O -->|"frame[1+256]"| PP_O
    PP_O -->|"send_buffer[1031]"| DCU_O
```

**核心数据结构与流转**：

| 数据结构 | 产生方 | 消费方 | 流转路径 |
|---------|--------|--------|----------|
| `motor_state_t` | `runtime_param.c`（电流环 ISR 写） | `jm_proto_ops.c:app_get_feedback` | `usr.motor_state[M1]` → `jm_feedback_t` → `reply[]` |
| `motor_param_t` | `runtime_param.c`（默认值 + 协议写入） | `jm_proto_ops.c:app_param_*` | `usr.motor_param[M1]` ↔ `s_param_tbl` 查表读写 |
| `motor_cmd_t` | `jm_proto_ops.c:app_set_mode` 写 | `motor_loop.c` 电流环 ISR 读 | `sys.motor.cmd` 单生产者单消费者无锁 |
| `jm_feedback_t` | `jm_proto_ops.c` 填充 | `jm_proto.c:pack_feedback` / `jm_host_commun.c:pack_telemetry` | 中转结构，转成应答字节 |
| `reply[1+256]` | `jm_proto.c` 写 | `jm_proto_uart.c` 读 | dispatch 出参，绑定层封帧 |
| `calib_status_t` | `calib_mgr.c` | `jm_proto.c`（0x97 直接读） | 直接打包为 8B 应答 |
| `jm_dbg[]` | 任意模块写 | `jm_proto_ops.c:app_get_debug` / `jm_host_commun.c` | 调试观测点，免改协议 |

---

## 8. 回调注入链路

协议栈采用**回调注入**实现解耦。下图展示所有回调的注入点与被调方。

```mermaid
flowchart LR
    subgraph INJ["注入方"]
        JHC_I["jm_host_commun.c<br/>jm_host_commun_init()"]
        DCU_I["dev_commun_uart.c<br/>dev_commun_uart_start()"]
        JPU_I["jm_proto_uart.c<br/>jm_proto_uart_init()"]
    end

    subgraph CB["回调函数"]
        OPS["jm_proto_ops_t *ops<br/>━━━━━━━<br/>set_mode/get_feedback/<br/>param_*/motor_info_*/<br/>get_dev_*/get_debug/<br/>set_telemetry/set_can_*"]
        TX_UART["jm_uart_tx_fn<br/>━━━━━━━<br/>jm_uart_tx()<br/>(dev_commun_uart.c 静态)"]
        ON_FRAME["on_frame()<br/>━━━━━━━<br/>jm_proto_uart.c 静态<br/>解出 cmd+payload<br/>调 dispatch"]
        ON_SEND["on_send()<br/>━━━━━━━<br/>jm_proto_uart.c 静态<br/>调 jm_uart_tx_fn"]
        PP_CB["PACKER_CB cb/send<br/>━━━━━━━<br/>整帧接收/封包发送回调"]
    end

    subgraph CONS["被调方"]
        JPO_C["jm_proto_ops.c<br/>(ops 实现)"]
        JP_C["jm_proto.c<br/>(dispatch)"]
        PP_C["packer_parser.c<br/>(状态机)"]
        DU_C["drv_usart.c<br/>(DMA 发送)"]
    end

    JHC_I -->|"jm_app_ops_get()"| OPS
    JHC_I -.->|"经 dev.set_ops"| DCU_I
    DCU_I -->|"jm_proto_uart_init(ops)"| OPS
    DCU_I -->|"jm_proto_uart_init(tx)"| TX_UART
    JPU_I -->|"upacker_set_cb(on_frame, on_send)"| PP_CB

    OPS -.->|"被 dispatch 调用"| JPO_C
    TX_UART -.->|"被 on_send 调用"| DU_C
    ON_FRAME -.->|"被 packer cb 调用"| JP_C
    ON_SEND -.->|"被 packer send 调用"| DU_C
    PP_CB -.->|"内部触发"| PP_C
```

**注入顺序**（初始化时序）：

1. `jm_host_commun_init()` 创建信号量
2. `dev_commun_uart_init()` 读配置表填充 `pobj->uart/motor_id`
3. `dev.set_ops(jm_app_ops_get())` → 注入业务回调
4. `dev.start()` 内部调 `jm_proto_uart_init(&jm, ops, motor_id, jm_uart_tx)`：
   - 注入 `ops` 到 `jm_proto_t`
   - 注入 `jm_uart_tx` 到 `jm_proto_uart_t`（作为 packer 的 send 回调）
5. `upacker_set_cb(on_frame, on_send)` → 注入 packer 的接收/发送回调
6. `usart_idle_init()` → 启动 DMA + IDLE 中断

---

## 9. 编译开关与条件包含

协议栈通过编译开关控制特性裁剪：

| 宏 | 定义位置 | 作用 | 影响文件 |
|----|---------|------|---------|
| `USE_DEV_COMMUN_UART` | `dev_config.h`（板级） | 启用 joint_proto 串口通信链路 | `dev_commun_uart.c`、`jm_host_commun.c`、`thread_commun.c` |
| `USE_DEV_COMMUN_VESC` | `dev_config.h`（板级） | 启用 VESC Tool 兼容链路 | `dev_commun_vesc.c`、`thread_commun.c` |
| `USE_DATA_CHECK` | `packer_parser.h` | 启用 CRC16 + 头校验 | `packer_parser.c` |
| `USE_DYNAMIC_MEM` | `packer_parser.h` | packer 缓冲动态分配（默认静态） | `packer_parser.c` |
| `MAX_PACK_SIZE` | `packer_parser.h` | 单帧数据区上限（默认 1024） | `packer_parser.c/.h` |
| `JM_PAYLOAD_MAX` | `jm_proto.h` | 协议层载荷上限（256） | `jm_proto.c/.h` |
| `JM_DBG_CH` | `jm_proto_ops.h` 或配置 | 调试通道数量 | `jm_proto_ops.c`、`jm_host_commun.c` |

**典型板级配置**（`Board/V1/Config/dev_config_board.h`）：

```c
#define USE_DEV_COMMUN_UART   1   /* 启用 joint_proto 串口 */
#define USE_DEV_COMMUN_VESC   0   /* 禁用 VESC 兼容 */
```

---

## 附：文件清单与跨目录归属

| 目录 | 文件 | 职责 |
|------|------|------|
| `AppServices/ThreadManager/` | `thread_commun.c/.h` | 通信线程入口 |
| `AppServices/ParamService/` | `jm_host_commun.c/.h` | 通信服务接入 + 遥测打包 |
| `Devices/` | `dev_commun_uart.c/.h` | UART 设备粘合层 |
| `Devices/` | `dev_commun_vesc.c/.h` | VESC 设备粘合层（旁路） |
| `Protocol/joint_proto/` | `jm_proto.c/.h` | 协议核心 dispatch |
| `Protocol/joint_proto/` | `jm_proto_uart.c/.h` | 串口绑定层 |
| `Protocol/joint_proto/` | `jm_proto_can.c/.h` | CAN 绑定层（旁路） |
| `Protocol/joint_proto/` | `jm_proto_ops.c/.h` | 业务回调实现 |
| `Protocol/joint_proto/` | `jm_cmd_def.h` | 命令码/错误码/类型码 |
| `Protocol/joint_proto/` | `example_jm_proto_uart.c` | 接入示例（不编译） |
| `Protocol/packer_parser/` | `packer_parser.c/.h` | 组拆帧器 |
| `Common/` | `crc16.c/.h` | CRC16 算法 |
| `Common/` | `assert_report.c/.h` | 断言上报 |
| `Driver/` | `drv_usart.c/.h` | USART+DMA+IDLE 驱动 |
| `Driver/` | `drv_rtos.c/.h` | RTOS 信号量/延时 |
| `DataHub/` | `runtime_param.c/.h` | 活数据全局实例 |
| `DataHub/` | `motor_param.c/.h` | 参数表元数据 |
| `DataHub/` | `motor_info.c/.h` | MotorInfo 持久化配置 |
| `DataHub/` | `version.h` | 版本号 |
| `MotorControl/CascadeControl/` | `motor_loop.c/.h` | 控制目标写入 |
| `MotorCalibration/` | `calib_mgr.c/.h` | 标定管理器 |
| `Config/` | `dev_config.c/.h` | 设备配置表 + 编译开关 |
