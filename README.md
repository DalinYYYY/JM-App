# JointMotor — 关节电机控制器固件

基于 STM32G474 的 BLDC 关节电机 FOC 控制固件。采用分层架构，支持真实硬件与虚拟电机在环仿真两种模式的编译期无缝切换。

## 硬件平台

| 项目 | 规格 |
| --- | --- |
| MCU | STM32G474CET6（Cortex-M4F，带 FPU / DSP） |
| 主频 | 170 MHz |
| Flash / RAM | 512 KB / 128 KB |
| 编码器 | MT6701（14bit）/ MT6835（21bit）磁编码器 |
| 功率级 | 三相半桥，互补 PWM + ADC 注入同步采样 |
| 通信 | FDCAN、USART |

## 软件环境

- **IDE / 工具链**：Keil MDK-ARM（工程 `Board/V1/MDK-ARM/JointMotorApp.uvprojx`）
- **配置工具**：STM32CubeMX（`Board/V1/JointMotorApp.ioc`）
- **RTOS**：FreeRTOS（CMSIS-OS V1，堆 16 KB）
- **代码风格**：`.clang-format`（项目根，参数生成器会自动套用）

## 核心特性

- **FOC 电流环**：Clarke / Park / 反 Park / SVPWM，单精度 FPU 优化
- **三环级联控制**：位置环 → 速度环 → 电流参考
- **虚拟电机在环仿真**：dq 物理模型复用真实 FOC 算法，无需硬件即可闭环验证
- **绝对多圈计数**：从运动解算中独立拆分
- **参数表代码生成**：CSV 一键生成带 get/set/校验的参数访问层
- **VOFA+ 上位机**：JustFloat 协议波形上送与指令接收

## 系统架构与状态机

### 软件框图

整体软件分层架构，自顶向下：用户接口层 → 通信驱动与协议解析 → 系统服务（状态机/参数/安全）→ 控制业务 → 算法 → 设备 → 驱动，底层由线程任务调度支撑。

![软件框图](User/Data/picture/软件框图.png)

### 主状态机

系统顶层状态机（对应 `User/DataHub/state_define.h` 的 `top_fsm_e`）：初始化（INIT）后进入待机（IDLE），经使能进入就绪（READY）与运行（RUN）；另有校准（CALIB）、配置（CONFIG）、固件升级（BOOTLOADER）等工作态，以及最高优先级的安全（SAFETY）与故障（FAULT）态。

![主状态机](User/Data/picture/主状态机.png)

### 运行模式状态机

运行态下的子模式切换（对应 `run_state_e` / `ctrl_mode_e`）：覆盖开环、电流、力矩、速度、位置、MIT、力控（阻抗/导纳/重力补偿等）、轨迹插补（PVT/样条/梯形/S 曲线）、回零等模式，由上位机指令驱动切换。

![运行模式状态机](User/Data/picture/运行模式状态机.png)

### 过渡模式

模式切换时的过渡引擎（对应 `User/MotorControl/ControlProcess/ctrl_transition.c/h`）：收到切换指令后先做合法性检查，合法则进入过渡态，保存并初始化状态、多拍平滑过渡，完成后进入新运行态；非法指令则退出过渡、维持原态。

![过渡模式](User/Data/picture/过渡模式.png)

## 目录结构

```
JointMotor_v0.1/
├── Core/                   # CubeMX 生成：HAL 初始化、main、中断向量、FreeRTOS 入口
├── Drivers/                # ST HAL 驱动与 CMSIS
├── Middlewares/            # FreeRTOS 等第三方中间件
├── Board/V1/MDK-ARM/       # Keil 工程文件
├── User/                   # 用户代码（核心，分层架构，详见 User/User文件结构说明.md）
│   ├── AppEntry/               # 入口层：初始化 + 主循环调度 + 中断入口
│   ├── AppServices/            # 系统服务：状态机、线程管理
│   ├── Common/                 # 通用工具：utils / pid_core / assert / vofa
│   ├── Config/                 # 编译期配置
│   ├── DataHub/                # 全局数据中心：参数 / 运行数据 / 版本
│   ├── Devices/                # 设备抽象：电机 / 编码器 / 半桥 / 采样 / 电源监控
│   ├── Driver/                 # 外设驱动封装（HAL 之上的统一接口）
│   ├── MotorAlgorithms/        # 纯算法：FOC / 运动解算 / 多圈计数
│   ├── MotorControl/           # 控制业务：CascadeControl / ControlProcess
│   ├── Protocol/               # 通信协议（规划中）
│   ├── Test/                   # 测试体系（规划中）
│   └── Tools/                  # 离线工具：参数表代码生成器
├── Board/V1/JointMotorApp.ioc # CubeMX 工程配置
├── .clang-format           # 代码格式规范
└── buildclean.bat          # 清理 Keil 编译中间产物
```

> `User/` 各模块的详细职责与文件清单见 [User/User文件结构说明.md](User/User文件结构说明.md)。

## 真实电机 / 虚拟电机切换

设备接入层采用**编译期二选一**，对上层完全透明（API 均为 `dev_motor_*`）：

| 宏 `MOTOR_LOOP_ENABLE_DEV_DRIVER` | 编译的实现 | 用途 |
| --- | --- | --- |
| `1` | `User/Devices/dev_motor.c`（真实硬件驱动） | 实机运行 |
| `0` | `User/MotorControl/CascadeControl/dev_motor_virtual.c`（dq 物理模型） | 在环仿真 |

切换只需修改 [User/MotorControl/CascadeControl/motor_loop_config.h](User/MotorControl/CascadeControl/motor_loop_config.h) 中的该宏，无需改动 Keil 工程文件。两份实现互斥编译，避免符号重复定义。

虚拟电机复用真实 `foc_core.c` 的 FOC 数学，保证仿真与实机跑同一套算法。

## 编译与烧录

1. 用 Keil MDK-ARM 打开 `Board/V1/MDK-ARM/JointMotorApp.uvprojx`
2. 选择目标 `JointMotorApp`，编译（F7）
3. 通过 ST-Link / J-Link 下载到 STM32G474

清理中间产物：运行根目录 `buildclean.bat`。

> 修改外设配置时，用 CubeMX 打开 `Board/V1/JointMotorApp.ioc` 重新生成；用户代码集中在 `User/`，CubeMX 重新生成不会覆盖。

## 参数表生成

电机参数访问层由 CSV 自动生成：

```bash
cd User/Tools/motor_param_gen
python motor_param_generate_v9.py
```

编辑 `motor_param.csv` 后运行，生成 `User/DataHub/motor_param.c/h`，自动包含 get/set 接口、只读（RO）字段保护、字符串版数组接口与越界校验，并按 `.clang-format` 格式化。
