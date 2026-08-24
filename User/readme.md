# User 文件夹结构说明

> 本文档依据实际代码结构整理（2026-07-02）。标注「规划中」的目录已建但暂无源文件，仅占位预留。

## 一、完整目录树

```
User/
├── AppEntry/                   # 系统入口层（初始化 + 主循环调度 + 中断入口）
├── AppServices/                # 系统级通用服务
│   ├── StateMachine/               # 全局主状态机
│   ├── ThreadManager/              # 线程/任务管理
│   └── ParamService/               # 上位机通信接入服务（承载 joint_proto 协议）
├── Common/                     # 通用工具层（无业务依赖）
├── Config/                     # 编译配置层（含电机型号档案 + 板级选择）
├── Data/                       # 资源数据（图片、文档、计划）
│   ├── docs/
│   ├── picture/
│   └── plans/
├── DataHub/                    # 全局数据中心（参数 + 运行数据 + 版本）
├── Devices/                    # 硬件设备抽象封装（含通信设备）
├── Driver/                     # 外设驱动抽象层（HAL 封装）
├── MotorAlgorithms/            # 纯算法库（FOC / 运动解算 / 多圈计数）
├── MotorCalibration/           # 电机标定模块（7 级 27 子模式）
├── MotorControl/               # 电机控制业务逻辑
│   ├── CascadeControl/             # 三环级联控制（电流环/外环/集成编排/虚拟电机）
│   ├── ControlProcess/             # 控制流程与过渡引擎
│   └── Modes/                      # 控制模式（一模式一文件 + 分发表）
├── Protocol/                   # 通信协议层（joint_proto / packer_parser）
└── Tools/                      # 离线工具（非 MCU 编译）
    ├── motor_param_gen/            # 参数表代码生成器（Python）
    ├── motor_info_gen/             # MotorInfo 持久化参数代码生成器（Python）
    └── pyqt_gui/                   # JM Studio 上位机（PyQt6）
```

## 二、各模块详细功能

### 1. `AppEntry/` — 系统入口层

**核心职责**：连接 CubeMX 生成代码与用户代码的桥梁，集中管理初始化、主循环调度与中断入口，不承载控制算法本身。

| 文件 | 核心功能 | 主要接口 |
| --- | --- | --- |
| `user_interface.c/h` | 用户接口层：硬件初始化、线程创建、主循环调度 | `user_init()`、`user_control()`、`motor_virtual_loop()` |
| `control_irq.c/h` | 控制相关中断入口（转发到业务层处理） | 中断回调入口 |
| `thread_management.c/h` | 线程管理入口封装 | 线程创建/调度入口 |

> 注：`motor_virtual_loop()` 仅在虚拟电机模式（`MOTOR_LOOP_ENABLE_DEV_DRIVER==0`）下编译启用。

---

### 2. `AppServices/` — 系统级通用服务层

**核心职责**：提供与具体业务无关的系统级基础设施，跨模块、全局性质的功能集中于此。

| 二级目录 | 状态 | 核心功能 | 包含文件 |
| --- | --- | --- | --- |
| `StateMachine/` | 已实现 | 全局主状态机，系统行为总指挥 | `system_state.c/h`（状态机核心，顶层主状态见 `DataHub/state_define.h`）、`system_state_fsm.md`（状态机说明） |
| `ThreadManager/` | 已实现 | 线程/任务统一管理（通信/控制/显示/空闲/周期任务） | `thread_config.h`、`thread_commun.c/h`、`thread_control.c/h`、`thread_display.c/h`、`thread_idle.c/h`、`thread_period.c/h` |
| `ParamService/` | 已实现 | 上位机通信接入层（承载 joint_proto 协议），从 `ThreadManager` 抽出 | `jm_host_commun.c/h`（自研 PyQt 上位机串口通信，遥控模式按 mask 主动上报 TELEMETRY）、`serialstudio_commun.c/h`（SerialStudio 接入层，已由 jm_host_commun 取代但保留兼容） |

---

### 3. `Common/` — 通用工具层

**核心职责**：所有模块的公共依赖，提供通用工具、PID 核心、CRC、断言与调试上送，不依赖业务模块。

**包含文件**：

- `utils.c/h`：通用数学与信号处理工具（角度归一化、快速三角函数、FFT、滤波、CRC32C 等，面向 M4F 单精度优化）
- `pid_core.c/h`：PID 控制器核心（运行时状态 + 算法配置标志）
- `crc16.c/h`：CRC16 校验算法（协议帧校验用）
- `assert_report.c/h`：断言与错误上报
- `vofa.c/h`：VOFA+ 上位机 JustFloat 数据上送与接收解析

---

### 4. `Config/` — 编译配置层

**核心职责**：存放编译期可配置宏、电机型号档案与板级选择，实现配置与代码分离。

**包含文件**：

- `system_config.h`：系统级配置（任务栈/优先级/频率、版本等）
- `control_config.h`：控制算法参数（环频率、PID 默认值、采样频率等）
- `protocol_config.h`：通信协议配置（CAN/UART 波特率、设备地址等）
- `debug_config.h`：调试配置（打印、断言、VOFA 开关等）
- `motor_profile.c/h`：电机型号参数库（唯一真相源）。集中各电机电气身份参数（R/Ld/Lq/flux/pole_pairs/KT/KE 等），三处消费方 `DataHub/motor_param.c`、`DataHub/motor_info.c`、`MotorCalibration/calib_config.h` 通过 `#include` 引用；型号选择在各板 `Board/<板名>/Config/motor_config_board.h` 的 `MOTOR_PROFILE_BOARD` 宏，未指定的板由 `motor_profile.h` 兜底为 GM4820H
- `board_select.h`：板级选择器。通过 `JM_BOARD_V1` / `JM_BOARD_SFOC` 宏二选一，引入 `Board/<板名>/Config/dev_config_board.h` 与 `motor_config_board.h`

---

### 5. `DataHub/` — 全局数据中心

**核心职责**：全局数据的统一存放与访问通道，各模块通过其交互参数与运行数据。

**包含文件**：

- `motor_param.c/h`：关节电机控制参数 API（由 `Tools/motor_param_gen` 生成，含 get/set/校验/打印）
- `motor_info.c/h`：MotorInfo 配置参数 API（1024B 整块 Flash 空间，由 `Tools/motor_info_gen` 生成，6 子块布局：SystemParam/MotorCalibParam/DeviceParam/ControlParam/ProtectCommParam/AdvancedAlgoParam，含整块 memcpy 读写与越界校验）
- `runtime_param.c/h`：运行期参数与数据（版本号联合体/结构体等）
- `state_define.h`：系统顶层主状态定义（优先级从高到低）
- `version.h`：版本信息定义

---

### 6. `Devices/` — 硬件设备抽象层

**核心职责**：对外部硬件设备做面向对象封装，提供统一设备接口，屏蔽寄存器细节。

| 文件 | 设备 |
| --- | --- |
| `dev_motor.c/h` | 电机实例化：编码器 + 多圈计数 + FOC + PWM + 相电流采样 |
| `dev_mt6701.c/h` | MT6701 磁编码器（14bit SSI/SPI）：角度/磁场状态/CRC/零点/方向 |
| `dev_mt6835.c/h` | MT6835 磁编码器（21bit SPI）：角度/零点读写/方向 |
| `dev_half_bridge.c/h` | 三相半桥 PWM 输出（互补输出 + ADC 注入同步触发） |
| `dev_motor_phase_current.c/h` | 三相相电流采样（ADC 注入组，与 PWM 同步） |
| `dev_power_monitor.c/h` | 电源监控（ADC 规则组 DMA：母线电压/电流/温度等） |
| `dev_dwt_counter.c/h` | DWT 周期计数设备封装 |
| `dev_commun_uart.c/h` | 关节电机串口通信设备（USART+DMA 空闲中断，承载 joint_proto，上接 `jm_proto_uart` 下接 `drv_usart`） |
| `dev_config.c/h` | 设备层统一接口：公共常量 + 板级选择器，板级映射在 `Board/<板名>/Config/` |
| `dev_eeprom.h` | EEPROM 设备：字节/块读写、跨页写、参数持久化（接口预留） |
| `dev_led.h` | LED 设备：单色/RGB 统一对象接口（接口预留） |

**文档子目录**：`Devices/docs/`（含 `dev_mt6701_说明.md`、`相电流采样_PWM触发ADC时序说明.md`）

**设计要点**：每个设备一组 `.c/.h`，上层只通过接口访问，不直接操作寄存器。通信设备（`dev_commun_uart`/`dev_commun_can`）作为粘合层，连接协议栈与底层 USART/CAN 驱动。

---

### 7. `Driver/` — 外设驱动抽象层

**核心职责**：对 CubeMX HAL 库做轻量封装，提供统一外设接口，隔离 MCU 厂商差异。

| 文件 | 外设 |
| --- | --- |
| `drv_adc.c/h` | ADC（规则/注入组、DMA） |
| `drv_can.c/h` | CAN（FDCAN）收发与过滤 |
| `drv_usart.c/h` | USART/UART |
| `drv_spi.c/h` / `drv_spi_soft.c/h` | 硬件 SPI / 软件模拟 SPI |
| `drv_i2c.c/h` / `drv_i2c_soft.c/h` | 硬件 I2C / 软件模拟 I2C |
| `drv_gpio.c/h` | GPIO |
| `drv_tim.c/h` / `drv_tim_pwm.h` | 定时器 / PWM |
| `drv_dwt_timer.c/h` | DWT 周期计数器，CPU 周期级高精度计时 |
| `drv_delay.c/h` | 延时 |
| `drv_flash_g4.c/h` | STM32G4 片内 Flash 读写 |
| `drv_rtos.c/h` | RTOS 接口封装（任务/延时/临界区/节拍，隔离具体 RTOS） |
| `drv_config.h` | 驱动层统一配置 |

---

### 8. `MotorAlgorithms/` — 纯算法库

**核心职责**：纯数学计算，不依赖硬件/OS/业务。输入数据、输出结果、无副作用，便于单测与 MBD 替换。

**包含文件**：

- `foc_core.c/h`：BLDC FOC 算法核心（Clarke/Park/反 Park/SVPWM）
- `motion_param.c/h`：运动参数解算（仅角度/速度，不含多圈计数）
- `multiturn_counter.c/h`：绝对多圈计数（从 `motion_param` 拆分独立）

---

### 9. `MotorCalibration/` — 电机标定模块

**核心职责**：独立的电机参数标定体系，按 7 级 27 子模式组织，与 `MotorControl/` 解耦。通过 `calib_mgr` 统一调度，各级别模块导出 `calib_level_ops_t` ops 函数表，按 level 索引查表调用。

**级别划分**：

| 级别 | 模块 | 子模式数 | 内容 |
| --- | --- | --- | --- |
| L1 | `calib_level1_driver.c` | 6 | 驱动硬件底层（ADC 偏置/增益/电流传感器/温度/母线电压/死区） |
| L2 | `calib_level2_motor.c` | 6 | 电机电气身份（相序/极对数/R/Ld/Lq/flux） |
| L3 | `calib_level3_encoder.c` | 5 | 编码器校准（零位/方向/线性度/正余弦/多圈零点） |
| L4 | `calib_level4_torque.c` | 1 | 转矩基础（力矩常数 KT） |
| L5 | `calib_level5_nonlinear.c` | 4 | 非线性补偿（齿槽/摩擦/死区/磁饱和） |
| L6 | `calib_level6_system.c` | 4 | 负载系统级（惯量/阻尼/回程间隙/PID 自整定） |
| L7 | `calib_level7_auto.c` | 1 | 全自动序列（16 步：L2 段按 R→Ld→Lq→flux 顺序） |

**包含文件**：

- `calib_mgr.c/h`：标定管理器。`calib_mgr_init` 注入 `dev_motor`/`motor_param`/`dt`，提供 `start`/`poll`/`abort`/`get_status` 接口；维护 `done_mask`（uint64 位图，bit=(level-1)*8+(submode-1)）记录各级子模式完成状态，供前置依赖检查
- `calib_hw.c/h`：标定共享硬件访问层。封装 `calib_hw_session_t` 会话（保存/恢复电角度回调 + 强制电角度），提供 `enter`/`exit`/`apply_voltage`/`apply_zero`/`get_encoder_raw_deg`/`get_encoder_mech_angle`；所有 level 模块复用此层而非各自实现
- `calib_config.h`：标定可调参数集中配置。所有电压/时间/采样数从此派生（测试电流 = 峰值电流×0.14，时间参数由 `MOTOR_TAU_S=Ld/R` 派生），切换电机型号自动适配
- `calib_types.h`：标定类型定义（状态枚举/失败原因码/级别常量/子模式常量/`calib_status_t`/`calib_level_ops_t` ops 表/`calib_io_t` 硬件访问接口/`done_mask` 位图宏/前置依赖表 `s_calib_dep_table`）
- `calib_step.h`：标定步骤定义（L7 全自动序列步骤枚举）
- `calib_validate.h`：标定结果合理性校验工具（范围/NaN 检查，结果写入 `motor_param` 前调用）

**文档子目录**：`MotorCalibration/docs/标定状态机说明.md`

> 设计要点：每个 level 模块将文件静态全局变量收敛到单个 `s_lN` 结构体（如 `s_l2`、`s_l3`），减少散落全局；标定结果经 `calib_validate_*` 校验后才写入 `motor_param`；硬件访问经 `calib_hw` 会话统一进出，避免与控制环冲突。

---

### 10. `MotorControl/` — 电机控制业务逻辑层

**核心职责**：电机控制业务流程与环路编排，不含保护逻辑与底层寄存器操作。

| 二级目录 | 状态 | 核心功能 | 包含文件 |
| --- | --- | --- | --- |
| `CascadeControl/` | 已实现 | 三环级联控制与设备接入 | `cascade_control.c/h`（位置环→速度环→电流参考，纯算法）、`current_loop.c/h`（FOC 电流环 + SVPWM + PWM）、`motor_loop.c/h`（三环集成/编排，统一中断入口）、`motor_loop_config.h`（控制配置，含 `MOTOR_LOOP_ENABLE_DEV_DRIVER` 真实/虚拟切换）、`dev_motor_select.h`（设备头选择器）、`dev_motor_virtual.c/h`（虚拟电机 dq 物理模型，在环仿真）、`dev_motor_stub.h`（屏蔽态桩） |
| `ControlProcess/` | 已实现 | 控制流程与过渡引擎 | `motor_control.c/h`（上层指令处理 + 模式分发表）、`ctrl_transition.c/h`（过渡引擎/状态平滑切换）、`pid_profile.c/h`（PID 参数配置档案）、`motor_mode.h`（模式处理函数统一签名 `motor_mode_fn` + 各模式声明） |
| `Modes/` | 已实现 | 控制模式实现，一模式一文件 | `motor_mode_idle.c`（IDLE）、`motor_mode_hold.c`（HOLD）、`motor_mode_open_loop.c`（OPEN_LOOP/VOLTAGE_VECTOR）、`motor_mode_duty.c`（DUTY_CYCLE）、`motor_mode_current.c`（CURRENT/FIELD_WEAKENING/SENSORLESS）、`motor_mode_torque.c`（TORQUE）、`motor_mode_mit.c`（MIT）、`motor_mode_velocity.c`（VELOCITY/VELOCITY_TORQUE）、`motor_mode_position.c`（POSITION/POSITION_VELOCITY/POSITION_TORQUE/PP）、`motor_mode_profile_velocity.c`（PV）、`motor_mode_profile_torque.c`（PT）、`motor_mode_test_sweep.c`（TEST_SWEEP_FREQ） |

> 设计要点：每个模式文件仅导出一个 `motor_mode_fn` 函数，只填充 `ctrl->ref`（参考输出），不执行环路计算，可在电流环中断调用。`motor_control.c` 维护模式分发表，按 `run_state` 查表派发。
>
> 真实电机与虚拟电机经 `dev_motor_select.h` + `MOTOR_LOOP_ENABLE_DEV_DRIVER` 二选一，API（`dev_motor_t`/`dev_motor_init`）对上层透明。

---

### 11. `Protocol/` — 通信协议层

**核心职责**：与上位机/外部设备的通信协议实现，硬件接口与协议逻辑分离。

| 二级目录 | 核心功能 | 包含文件 |
| --- | --- | --- |
| `joint_proto/` | 关节电机自研协议（传输无关） | `jm_proto.c/h`（CMD 分发 + 小端编解码助手，业务动作经 `jm_proto_ops_t` 回调注入）、`jm_cmd_def.h`（命令码定义）、`jm_proto_uart.c/h`（串口组帧/拆帧）、`jm_proto_can.c/h`（CAN 组帧/拆帧）、`jm_proto_ops.c/h`（业务回调实现：反馈/参数/控制）、`example_jm_proto_uart.c`（使用示例） |
| `packer_parser/` | 通用数据包解析器 | `packer_parser.c/h` |

**文档子目录**：`Protocol/docs/`（含 `joint_motor_command_list.csv` 命令清单、`joint_motor_param_index.csv` 参数索引、`joint_motor_protocol_spec.md` 协议规范、`joint_proto_layer_diagram.md` 分层图）

> 当前 VOFA+ 上送仍由 `Common/vofa.c/h` 承担；CAN 收发能力由 `Driver/drv_can.c/h` 提供。

---

### 12. `Tools/` — 离线工具

**核心职责**：在 PC 上运行的辅助工具，不参与 MCU 编译。

| 子目录 | 功能 |
| --- | --- |
| `motor_param_gen/` | 由 CSV 生成 `DataHub/motor_param.c/h`：含 get/set 接口、RO 只读字段、char 数组字符串版接口、越界校验（返回首个越界 CSV id），生成后自动按项目 `.clang-format` 格式化。包含 `motor_param_generate_v9.py`、`motor_param.csv` |
| `motor_info_gen/` | 由 CSV 生成 `DataHub/motor_info.c/h`：1024B Flash 整块布局（6 子块），含整块 memcpy 读写、字段偏移常量、越界校验。包含 `motor_info_generate.py`、`motor_info.csv`、`MotorInfo_readme.md` |
| `pyqt_gui/` | **JM Studio** 关节电机调试上位机（PyQt6）。协议/传输解耦（`jmproto` 零硬件依赖，`transport` 抽象基类可插拔串口/CAN/虚拟引擎）；数据驱动 UI（100 命令 + 83 参数由 CSV 渲染）；内置数字孪生引擎（dq 电气方程 + 机械方程 + 热模型 + 关节负载 + FOC + 状态机）；遥测订阅（10 通道按位掩码）；完整诊断（实时反馈 + 状态机 + 故障历史 + NACK 解码）。包含主程序 `main.py`、`core/`（motor_client）、`jmproto/`（协议层）、`transport/`（串口/CAN/虚拟引擎）、`ui/`（主窗口 + panels）、`tools/twin/`（独立孪生上位机）、`tools/waveform/`（独立波形采集上位机带 FFT/触发/导出）、`packaging/`（PyInstaller 打包脚本） |

---

## 三、模块依赖关系

```
AppEntry（初始化 + 主循环 + 中断入口）
    ↓
AppServices（StateMachine / ThreadManager / ParamService）
    ↓
MotorControl（CascadeControl / ControlProcess / Modes）
    ↓                                  ↑（标定态调用）
MotorCalibration（calib_mgr → level1~7）   │
    ↓                                        │
MotorAlgorithms（FOC / 运动解算 / 多圈计数）  │
    ↓                                        │
DataHub（motor_param / motor_info / runtime）│
    ↓                                        │
Devices（设备抽象，含 dev_commun_uart/can） ┘
    ↓
Driver（HAL 封装）
    ↓
CubeMX 生成的 HAL 库

横向支撑层（各层均可调用）：
Common（utils / pid_core / crc16 / assert / vofa）、Config（含 motor_profile / board_select）
Protocol（joint_proto）经 Devices/dev_commun_* 接入，供 AppServices/ParamService 使用
```

> 设备接入采用编译期切换：`MOTOR_LOOP_ENABLE_DEV_DRIVER==1` 走真实 `Devices/dev_motor.c`，`==0` 走 `CascadeControl/dev_motor_virtual.c`（虚拟在环仿真），两者实现同一套 `dev_motor_*` API，互斥编译避免符号冲突。
>
> 板级切换：`Config/board_select.h` 通过 `JM_BOARD_V1` / `JM_BOARD_SFOC` 宏二选一，引入对应 `Board/<板名>/Config/dev_config_board.h`，板级参数（pwm_freq/enc_lines/dead_time 等）与 `motor_profile.h` 的电机电气身份参数分离。
