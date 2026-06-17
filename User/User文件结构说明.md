# User 文件夹结构说明

> 本文档依据实际代码结构整理（2026-06-17）。标注「规划中」的目录已建但暂无源文件，仅占位预留。

## 一、完整目录树

```
User/
├── AppEntry/                   # 系统入口层（初始化 + 主循环调度 + 中断入口）
├── AppServices/                # 系统级通用服务
│   ├── StateMachine/               # 全局主状态机
│   ├── ThreadManager/              # 线程/任务管理
│   ├── ErrorManager/               # 全局错误管理（规划中）
│   ├── ParamService/               # 参数管理服务（规划中）
│   ├── SafetyManager/              # 软硬件保护逻辑（规划中）
│   └── Watchdog/                   # 系统监控服务（规划中）
├── Common/                     # 通用工具层（无业务依赖）
├── Config/                     # 编译配置层
├── Data/                       # 资源数据（图片、VOFA 配置等）
│   ├── picture/
│   └── vofaConfig/
├── DataHub/                    # 全局数据中心（参数 + 运行数据 + 版本）
├── Devices/                    # 硬件设备抽象封装
├── Driver/                     # 外设驱动抽象层（HAL 封装）
├── MotorAlgorithms/            # 纯算法库（FOC / 运动解算 / 多圈计数）
├── MotorControl/               # 电机控制业务逻辑
│   ├── CascadeControl/             # 三环级联控制（电流环/外环/集成编排/虚拟电机）
│   ├── ControlProcess/             # 控制流程与过渡引擎
│   ├── Calibration/                # 校准流程（规划中）
│   └── ControlModes/               # 控制模式（规划中）
├── Protocol/                   # 通信协议层（规划中）
├── Test/                       # 测试体系层（规划中）
│   ├── DebugTools/
│   ├── IntegrationTest/
│   └── UnitTest/
└── Tools/                      # 离线工具（非 MCU 编译）
    └── motor_param_gen/            # 参数表代码生成器（Python）
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
| `StateMachine/` | 已实现 | 全局主状态机，系统行为总指挥 | `system_state.c/h`（状态机核心，顶层主状态见 `DataHub/state_define.h`） |
| `ThreadManager/` | 已实现 | 线程/任务统一管理（通信/控制/显示/空闲/周期任务） | `thread_config.h`、`thread_commun.c/h`、`thread_control.c/h`、`thread_display.c/h`、`thread_idle.c/h`、`thread_period.c/h` |
| `ErrorManager/` | 规划中 | 全局错误收集、分级与上报 | — |
| `ParamService/` | 规划中 | 参数读写、校验、持久化 | — |
| `SafetyManager/` | 规划中 | 过流/过压/过温/堵转等保护 | — |
| `Watchdog/` | 规划中 | 任务监控、死锁检测、心跳 | — |

---

### 3. `Common/` — 通用工具层

**核心职责**：所有模块的公共依赖，提供通用工具、PID 核心、断言与调试上送，不依赖业务模块。

**包含文件**：

- `utils.c/h`：通用数学与信号处理工具（角度归一化、快速三角函数、FFT、滤波、CRC32C 等，面向 M4F 单精度优化）
- `pid_core.c/h`：PID 控制器核心（运行时状态 + 算法配置标志）
- `assert_report.c/h`：断言与错误上报
- `vofa.c/h`：VOFA+ 上位机 JustFloat 数据上送与接收解析

---

### 4. `Config/` — 编译配置层

**核心职责**：存放编译期可配置宏，实现配置与代码分离。

**包含文件**：

- `system_config.h`：系统级配置（任务栈/优先级/频率、版本等）
- `control_config.h`：控制算法参数（环频率、PID 默认值、采样频率等）
- `protocol_config.h`：通信协议配置（CAN/UART 波特率、设备地址等）
- `debug_config.h`：调试配置（打印、断言、VOFA 开关等）

---

### 5. `DataHub/` — 全局数据中心

**核心职责**：全局数据的统一存放与访问通道，各模块通过其交互参数与运行数据。

**包含文件**：

- `motor_param.c/h`：关节电机配置参数 API（由 `Tools/motor_param_gen` 生成，含 get/set/校验/打印）
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
| `dev_config.c/h` | 设备层统一硬件配置：使能开关 + 换算常量 |
| `dev_eeprom.h` | EEPROM 设备：字节/块读写、跨页写、参数持久化（接口预留） |
| `dev_led.h` | LED 设备：单色/RGB 统一对象接口（接口预留） |

**设计要点**：每个设备一组 `.c/.h`，上层只通过接口访问，不直接操作寄存器。

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

### 9. `MotorControl/` — 电机控制业务逻辑层

**核心职责**：电机控制业务流程与环路编排，不含保护逻辑与底层寄存器操作。

| 二级目录 | 状态 | 核心功能 | 包含文件 |
| --- | --- | --- | --- |
| `CascadeControl/` | 已实现 | 三环级联控制与设备接入 | `cascade_control.c/h`（位置环→速度环→电流参考，纯算法）、`current_loop.c/h`（FOC 电流环 + SVPWM + PWM）、`motor_loop.c/h`（三环集成/编排，统一中断入口）、`motor_loop_config.h`（控制配置，含 `MOTOR_LOOP_ENABLE_DEV_DRIVER` 真实/虚拟切换）、`dev_motor_select.h`（设备头选择器）、`dev_motor_virtual.c/h`（虚拟电机 dq 物理模型，在环仿真）、`dev_motor_stub.h`（屏蔽态桩） |
| `ControlProcess/` | 已实现 | 控制流程与过渡引擎 | `motor_control.c/h`（上层指令处理）、`ctrl_transition.c/h`（过渡引擎/状态平滑切换）、`pid_profile.c/h`（PID 参数配置档案） |
| `Calibration/` | 规划中 | 编码器/相序/电机参数校准 | — |
| `ControlModes/` | 规划中 | 电流/速度/位置/力矩模式 | — |

> 真实电机与虚拟电机经 `dev_motor_select.h` + `MOTOR_LOOP_ENABLE_DEV_DRIVER` 二选一，API（`dev_motor_t`/`dev_motor_init`）对上层透明。

---

### 10. `Protocol/` — 通信协议层（规划中）

**核心职责**：与上位机/外部设备的通信，硬件接口与协议逻辑分离。目录已预留，暂无源文件。

> 当前 VOFA+ 上送暂由 `Common/vofa.c/h` 承担；CAN 收发能力由 `Driver/drv_can.c/h` 提供。

---

### 11. `Test/` — 测试体系层（规划中）

**核心职责**：测试代码与调试工具，与业务代码分离，不进正式版本。

| 二级目录 | 核心功能 |
| --- | --- |
| `DebugTools/` | 调试工具（命令行、性能分析、日志） |
| `IntegrationTest/` | 集成测试（模块间交互） |
| `UnitTest/` | 单元测试（单函数） |

---

### 12. `Tools/` — 离线工具

**核心职责**：在 PC 上运行的辅助工具，不参与 MCU 编译。

| 文件 | 功能 |
| --- | --- |
| `motor_param_gen/motor_param_generate_v9.py` | 由 CSV 生成 `DataHub/motor_param.c/h`：含 get/set 接口、RO 只读字段、char 数组字符串版接口、越界校验（返回首个越界 CSV id），生成后自动按项目 `.clang-format` 格式化 |
| `motor_param_gen/motor_param.csv` | 参数表源数据（生成器输入） |

---

## 三、模块依赖关系

```
AppEntry（初始化 + 主循环 + 中断入口）
    ↓
AppServices（StateMachine / ThreadManager）
    ↓
MotorControl（CascadeControl / ControlProcess）
    ↓
MotorAlgorithms（FOC / 运动解算 / 多圈计数）
    ↓
DataHub（参数 / 运行数据）
    ↓
Devices（设备抽象）
    ↓
Driver（HAL 封装）
    ↓
CubeMX 生成的 HAL 库

横向支撑层（各层均可调用）：
Common（utils / pid_core / assert / vofa）、Config（编译配置）
```

> 设备接入采用编译期切换：`MOTOR_LOOP_ENABLE_DEV_DRIVER==1` 走真实 `Devices/dev_motor.c`，`==0` 走 `CascadeControl/dev_motor_virtual.c`（虚拟在环仿真），两者实现同一套 `dev_motor_*` API，互斥编译避免符号冲突。
