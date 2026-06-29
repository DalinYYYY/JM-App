# JointMotor

基于 STM32G474 的关节电机控制器固件仓库，包含主控板固件、协议文档、参数生成工具，以及配套的 PyQt 上位机与虚拟电机调试工具。项目采用分层架构，支持真实硬件与虚拟电机在环仿真两种模式的编译期切换。

## 仓库概览

当前仓库不仅包含 MCU 固件，还同时维护了通信协议、参数表生成流程和 PC 端调试工具：

- **主固件工程**：`Board/V1`，目标芯片为 `STM32G474CETx`
- **参考板工程**：`Board/SFOC`，以子模块形式保留
- **用户代码主体**：`User/`，承载控制、算法、设备抽象与协议实现
- **上位机工具**：`User/Tools/pyqt_gui`，以子模块形式维护
- **协议文档与表格**：`User/Protocol/docs`

## 硬件平台

| 项目 | 规格 |
| --- | --- |
| MCU | STM32G474CET6 / STM32G474CETx（Cortex-M4F，带 FPU / DSP） |
| 主频 | 170 MHz |
| Flash / RAM | 512 KB / 128 KB |
| 编码器 | MT6701（14 bit）/ MT6835（21 bit）磁编码器 |
| 功率级 | 三相半桥，互补 PWM + ADC 注入同步采样 |
| 通信 | FDCAN、USART |

## 软件环境

- **IDE / 工具链**：Keil MDK-ARM，工程位于 `Board/V1/MDK-ARM/JointMotorApp.uvprojx`
- **Keil Pack**：`Keil.STM32G4xx_DFP.2.2.0`
- **编译器**：Arm Compiler 5（工程当前配置）
- **配置工具**：STM32CubeMX，工程位于 `Board/V1/JointMotorApp.ioc`
- **RTOS**：FreeRTOS（CMSIS-OS V1）
- **代码格式**：项目根目录 `.clang-format`

## 核心特性

- **FOC 电流环**：Clarke / Park / 反 Park / SVPWM，面向 M4F 单精度优化
- **级联控制结构**：位置环、速度环、电流环分层组织
- **虚拟电机在环仿真**：dq 物理模型复用真实 FOC 算法，无需硬件即可闭环验证
- **多圈位置解算**：运动参数与绝对多圈计数解耦
- **协议双通道**：统一命令空间，同时覆盖串口与 CAN
- **参数表代码生成**：CSV 自动生成 `motor_param.c/h` 访问层
- **调试工具完善**：配套 PyQt 上位机、数字孪生工具、波形工具

## 系统架构与状态机

### 软件框图

整体软件分层架构自顶向下为：用户入口层 -> 系统服务层 -> 控制业务层 -> 算法层 -> 设备抽象层 -> 驱动层，底层由 FreeRTOS 任务调度支撑。

![软件框图](User/Data/picture/软件框图.png)

### 主状态机

系统顶层状态机定义见 `User/DataHub/state_define.h` 中的 `top_fsm_e`。系统从 `INIT` 进入 `IDLE`，经使能进入 `READY` 与 `RUN`；另包含 `CALIB`、`CONFIG`、`BOOTLOADER` 等工作态，以及最高优先级的 `SAFETY` 与 `FAULT`。

![主状态机](User/Data/picture/主状态机.png)

### 运行模式状态机

运行态子模式定义见 `run_state_e` / `ctrl_mode_e`，覆盖开环、电流、力矩、速度、位置、MIT、阻抗 / 导纳、轨迹插补、回零等模式，由协议命令驱动切换。

![运行模式状态机](User/Data/picture/运行模式状态机.png)

### 过渡模式

模式切换过渡逻辑位于 `User/MotorControl/ControlProcess/ctrl_transition.c/h`。切换前先做合法性检查，合法则进入过渡态执行状态保存、初始化与平滑过渡，完成后切入目标模式；非法切换则维持原状态。

![过渡模式](User/Data/picture/过渡模式.png)

## 目录结构

```text
JointMotor/
├── Board/
│   ├── V1/                         # 当前主控板工程：CubeMX、HAL、FreeRTOS、Keil 工程
│   │   ├── Core/
│   │   ├── Drivers/
│   │   ├── Middlewares/
│   │   ├── MDK-ARM/
│   │   └── JointMotorApp.ioc
│   └── SFOC/                       # 参考板 / 历史板级工程（子模块）
├── User/                           # 用户代码主体
│   ├── AppEntry/                   # 入口层：初始化、调度、中断转发
│   ├── AppServices/                # 系统服务：状态机、线程管理、参数通信
│   ├── Common/                     # 通用工具：PID、CRC、断言、VOFA
│   ├── Config/                     # 编译期配置
│   ├── Data/                       # 图片等资源
│   ├── DataHub/                    # 全局数据中心：参数、运行数据、版本
│   ├── Devices/                    # 设备抽象：电机、编码器、半桥、采样、电源监控
│   ├── Driver/                     # HAL 之上的驱动封装
│   ├── MotorAlgorithms/            # 纯算法：FOC、运动解算、多圈计数
│   ├── MotorControl/               # 控制业务：级联控制、模式切换、虚拟电机
│   ├── Protocol/                   # 协议实现、文档、协议表格、串口 / CAN 封装
│   ├── Tools/
│   │   ├── motor_param_gen/        # 参数表代码生成器
│   │   └── pyqt_gui/               # PyQt 上位机（子模块）
│   └── User文件结构说明.md         # User 层详细职责说明
├── README.md
├── buildclean.bat                  # 清理 Keil 编译中间产物
├── .clang-format
├── .gitignore
└── .gitmodules
```

`User/` 各模块的详细职责与文件清单见 [User/User文件结构说明.md](User/User文件结构说明.md)。

## 克隆仓库

本仓库包含子模块，首次拉取请使用递归方式：

```bash
git clone --recursive <repo-url>
```

如果已经完成普通克隆，请执行：

```bash
git submodule update --init --recursive
```

当前子模块包括：

- `Board/SFOC`
- `User/Tools/pyqt_gui`

## 真实电机 / 虚拟电机切换

设备接入层采用编译期二选一，对上层保持统一的 `dev_motor_*` 接口：

| 宏 `MOTOR_LOOP_ENABLE_DEV_DRIVER` | 编译实现 | 用途 |
| --- | --- | --- |
| `1` | `User/Devices/dev_motor.c` | 真实硬件运行 |
| `0` | `User/MotorControl/CascadeControl/dev_motor_virtual.c` | 虚拟电机在环仿真 |

切换方式：

1. 打开 `User/MotorControl/CascadeControl/motor_loop_config.h`
2. 修改 `MOTOR_LOOP_ENABLE_DEV_DRIVER`
3. 重新编译工程

默认配置为：

```c
#define MOTOR_LOOP_ENABLE_DEV_DRIVER 1u
```

虚拟电机实现复用真实 `foc_core.c` 的 FOC 数学逻辑，从而保证仿真与实机尽量运行同一套控制算法。

## 编译与烧录

### 固件构建

1. 使用 Keil MDK-ARM 打开 `Board/V1/MDK-ARM/JointMotorApp.uvprojx`
2. 选择目标 `JointMotorApp`
3. 执行编译
4. 通过 ST-Link 或 J-Link 下载到目标板

### CubeMX 重新生成

当外设配置需要调整时：

1. 打开 `Board/V1/JointMotorApp.ioc`
2. 在 STM32CubeMX 中修改外设
3. 重新生成代码
4. 回到 Keil 工程重新编译

`Board/V1/Core`、`Board/V1/Drivers`、`Board/V1/Middlewares` 为 CubeMX / HAL 生成内容，用户业务代码主要位于 `User/`。

### 清理中间文件

在仓库根目录运行：

```bat
buildclean.bat
```

该脚本会清理 `Board/V1/MDK-ARM` 下的 `Objects`、`Listings`、`DebugConfig`、输出目录及若干临时文件。

## 通信协议

协议实现与文档位于 `User/Protocol/`：

- `docs/joint_motor_protocol_spec.md`：协议规范说明
- `docs/joint_motor_command_list.csv`：命令总表
- `docs/joint_motor_param_index.csv`：参数索引表
- `joint_proto/`：协议核心实现与串口 / CAN 封装
- `packer_parser/`：串口打包与解析器
- `serial_studio/`：Serial Studio 工具链与配置生成脚本
- `vesc_proto/`：VESC 兼容协议相关实现与资料

协议命令空间统一覆盖串口与 CAN，便于固件、上位机与调试工具共享同一套命令定义。

## 参数表生成

参数访问层由 CSV 自动生成，生成器位于 `User/Tools/motor_param_gen/`。

```bash
cd User/Tools/motor_param_gen
python motor_param_generate_v9.py
```

编辑 `motor_param.csv` 后重新执行脚本，将生成：

- `User/DataHub/motor_param.c`
- `User/DataHub/motor_param.h`

生成结果包含参数 get / set 接口、只读字段保护、数组字符串接口、边界校验，并会按 `.clang-format` 进行格式化。

## PyQt 上位机

配套上位机位于 `User/Tools/pyqt_gui/`，用于串口 / CAN / 虚拟孪生调试。

主要能力包括：

- 数据驱动的命令与参数面板
- 实时反馈、状态机、曲线与故障显示
- 数字孪生虚拟引擎
- 波形与附属工具

快速启动：

```bash
cd User/Tools/pyqt_gui
pip install -r requirements.txt
python main.py
```

当前主要 Python 依赖：

- `PyQt6`
- `pyserial`
- `pyqtgraph`
- `numpy`

更详细说明见 `User/Tools/pyqt_gui/README.md`。

## 相关文档

- `User/User文件结构说明.md`
- `User/Protocol/docs/joint_motor_protocol_spec.md`
- `User/Tools/pyqt_gui/README.md`
- `User/AppServices/StateMachine/system_state_fsm.md`
