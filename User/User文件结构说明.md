# User文件夹结构说明

## 一、完整目录树

```
User/
├── AppEntry/                   # 系统所有入口（主入口+中断入口）
├── AppServices/                # 系统级通用服务
│   ├── ErrorManager/               # 全局错误管理
│   ├── ParamService/               # 参数管理服务
│   ├── SafetyManager/              # 所有软硬件保护逻辑
│   ├── StateMachine/               # 全局状态机管理(主状态机)
│   ├── ThreadManager/              # 线程/任务管理
│   ├── BootManager/                # 固件升级管理
│   └── Watchdog/                   # 系统监控服务
├── Common/                     # 通用工具层
├── Config/                     # 编译配置层
├── Data/                       # 工程可能用到的图片, 工具配置等
├── Devices/                    # 硬件设备抽象封装
├── Driver/                     # 外设驱动抽象层
├── MotorAlgorithms/            # 纯算法库
├── MotorControl/               #  纯点击控制业务逻辑
│   ├── Calibration/                # 校准流程管理
│   ├── ControlModes/               # 各种控制模式实现
│   └── ControlMachine/             # 控制状态机管理
├── Protocol/                   # 通信协议层
├── DataHub/                    # 数据枢纽
└── Test/                       # 测试体系层
    ├── DebugTools/                 # 调试工具
    ├── IntegrationTest/            # 集成测试
    └── UnitTest/                   # 单元测试
```

## 二、各模块详细功能说

### 1. `AppEntry/` - 系统所有入口层

**核心职责**：连接CubeMX生成代码与用户代码的唯一桥梁，集中管理所有系统入口点（主入口+中断入口），**绝对不包含任何业务逻辑**。

#### 1.1 根目录文件

| 文件 | 核心功能 | 主要函数 |
| --- | --- | --- |
| `app_main.c` | 主入口函数，main.c中仅需调用这一个函数 | `void app_main(void)` |
| `system_boot.c/h` | 系统初始化顺序控制，按优先级执行各模块初始化 | `void system_boot_sequence(void)` |

#### 1.2 `Interrupts/` - 中断入口函数（按你的要求新增）

**核心职责**：只存放CubeMX生成的`stm32xx_it.c`中直接调用的中断入口函数，**不包含任何实际处理逻辑**。

**包含文件**：

- `current_loop_irq.c`：ADC采样完成中断入口（电流环）
- `timer_irq.c`：定时器更新中断入口
- `can_irq.c`：CAN接收中断入口
- `uart_irq.c`：UART接收中断入口
- `safety_timer_irq.c`：安全检查定时器中断入口


- 中断入口函数只做三件事：清标志、安全检查、调用对应业务层的处理函数
- 所有实际处理逻辑都在业务层实现，中断入口只做转发
- 这样CubeMX重新生成`stm32xx_it.c`时，只需将中断函数指向这里即可

---

### 2. `AppServices/` - 系统级通用服务层

**核心职责**：提供与具体业务无关的系统级通用服务，是整个系统的基础设施。所有跨模块、全局性质的功能都放在这里。

| 二级文件夹 | 核心功能 | 包含文件 |
| --- | --- | --- |
| `ErrorManager/` | 全局错误收集、处理与上报 | `error_code.h`（统一错误码定义）、`error_handler.c/h`（错误分级处理）、`error_log.c/h`（错误日志记录） |
| `ParamService/` | 参数统一读写、校验、同步与持久化 | `param_table.h`（参数表定义）、`param_database.c/h`（参数数据库实现）、`flash_storage.c/h`（Flash存储驱动） |
| `SafetyManager/` | 所有软硬件保护逻辑 | `safety_manager.h`（故障等级与阈值定义）、`hardware_protection.c/h`（过流/过压/过温保护）、`software_protection.c/h`（堵转/超速/通信超时保护）、`emergency_stop.c/h`（紧急停机逻辑）、`safety_check.c/h`（保护调度器） |
| `StateMachine/` | 全局状态机管理，系统行为总指挥 | `system_state.h`（系统状态枚举）、`state_transition.c/h`（状态切换规则与安全检查） |
| `ThreadManager/` | 线程/任务统一管理 | `thread_table.h`（所有任务定义表）、`thread_manager.c/h`（任务创建、挂起、恢复） |
| `Watchdog/` | 系统运行状态监控，死锁与卡死检测 | `task_watchdog.c/h`（任务运行状态监控）、`heartbeat.c/h`（与上位机的心跳通信） |

---

### 3. `Common/` - 通用工具层

**核心职责**：所有模块的公共依赖，提供通用的宏定义、数据类型和工具函数，**不依赖任何其他模块**。

**包含文件**：

- `common.h`：全局通用头文件，包含所有标准库头文件和通用宏
- `data_types.h`：自定义数据类型定义（结构体、枚举等）
- `utils.c/h`：通用工具函数（CRC校验、字节序转换、字符串处理等）
- `ring_buffer.c/h`：环形缓冲区实现（用于通信数据缓存）
- `math_utils.c/h`：通用数学函数（快速正弦余弦、查表插值等）

---

### 4. `Config/` - 编译配置层

**核心职责**：存放所有编译期可配置的宏定义，实现**配置与代码分离**，便于根据不同硬件和应用场景进行裁剪。

**包含文件**：

- `system_config.h`：系统级配置（任务栈大小、优先级，任务频率，软硬件版本等）
- `control_config.h`：控制算法参数（环频率、PID默认参数、采样频率等）
- `protocol_config.h`：通信协议配置（CAN波特率、UART波特率、设备地址等）
- `debug_config.h`：调试配置（是否开启打印、断言、VOFA等）

---

### 5. `Devices/` - 硬件设备抽象层

**核心职责**：对所有外部硬件设备进行面向对象的抽象封装，提供统一的设备操作接口，屏蔽硬件连接细节。

**设计要点**：

- 每个设备对应一个.c和.h文件，使用面向对象的思想实现统一的接口
- 上层代码只能通过接口函数访问设备，不能直接操作寄存器

---

### 6. `Driver/` - 外设驱动抽象层

**核心职责**：对CubeMX生成的HAL库进行轻量级封装，提供跨平台的统一外设操作接口，隔离不同MCU厂商的HAL库差异。

**包含文件**：

- `drv_can.c/h`：CAN外设封装
- `drv_uart.c/h`：UART外设封装
- `drv_adc.c/h`：ADC外设封装
- `drv_pwm.c/h`：PWM外设封装
- `drv_gpio.c/h`：GPIO外设封装
- `drv_timer.c/h`：定时器外设封装
- ··· 

**命名规则**：参考如下：

```c
#ifndef __DRV_CAN_H
#define __DRV_CAN_H

/* 函数声明 */
int drv_can_init(canNumber_e can, void *filter);
int drv_can_send_msg(canNumber_e can, void *txHeader, uint8_t *data, uint16_t size);
int drv_can_recv_msg(canNumber_e can, void *rxHeader, uint8_t *data, uint16_t size);
int drv_can_register_rx_callback(canNumber_e can, can_rx_callback_t callback);

#endif /* __DRV_CAN_H */
```



---

### 7. `MotorAlgorithms/` - 纯算法库

**核心职责**：存放所有纯数学计算的算法实现，不依赖任何硬件、操作系统和业务逻辑。输入是数据，输出是计算结果，没有任何副作用。

**设计要点**：

- 所有算法函数都是纯函数，没有全局变量
- 可以单独编译和测试，不依赖任何其他模块
- 后期可以直接替换为Simulink等MBD工具生成的算法代码

---

### 8. `MotorControl/` - 纯电机控制业务逻辑层

**核心职责**：只负责电机控制的业务流程和模式切换，不包含任何保护逻辑、算法实现和硬件操作。

| 二级文件夹 | 核心功能 | 包含文件 |
| --- | --- | --- |
| `Calibration/` | 各种校准功能的业务流程 | `encoder_calibration.c/h` `phase_sequence_calibration.c/h`、`motor_param_calibration.c/h` |
| `ControlModes/` | 各种控制模式的业务流程实现 | `current_mode.c/h`、`speed_mode.c/h`、`position_mode.c/h`、`torque_mode.c/h` |
| `ControlProcess/` | 电机启动、停止等完整业务流程 | `motor_startup.c/h`、`motor_stop.c/h` |

---

### 9. `Protocol/` - 通信协议层

**核心职责**：负责所有与上位机和外部设备的通信，实现硬件接口与协议逻辑的完全分离。

**对不同的硬件接口：**

- `uart_commun_interface.c/.h` ：UART协议接口
- `can_commun_interface.c/.h` ：UARTCAN协议接口
- `justfloat_protocol_core.c/.h` ：justfloat解析实现
- `can_protocol_core.c/.h` ：CAN解析实现
- `justfloat_protocol_callback.c/.h`：justfloat协议回调口
- `can_protocol_callback.c/.h`：CAN协议回调口

---

### 10. `DataHub/` - 全局数据中心

**核心职责**：所有全局变量的唯一存放位置，是各模块之间数据交互的唯一通道

**包含文件**：

- `mootor_data.c/h`：电机实时运行数据的定义与操作接口
- `motor_info.c/h`： 电机控制相关配置的定义与操作接口
- `exception_data.c/h`：异常数据相关定于与操作为接口

**设计要点**：

- MotorInfo数据严格进行4字节对齐
- 不允许在其他任何模块定义全局变量
- 所有数据访问都通过统一的get/set接口
- 对多任务访问的数据自动添加互斥锁保护

---

**4字节对齐**
```C
typedef struct {
    uint8_t pole_pairs;
    uint8_t reserved1[3]; // 填充至4字节
    float rated_current;
    float rated_voltage;
    float reduction_ratio;
    uint16_t encoder_resolution;
    uint8_t reserved2[2]; // 填充至4字节
} __attribute__((aligned(4))) motor_hardware_info_t;
```

### 11. `Test/` - 测试体系层

**核心职责**：存放所有测试代码和调试工具，与业务代码完全分离，不编译到正式版本中。

| 二级文件夹 | 核心功能 |
| --- | --- |
| `DebugTools/` | 调试工具（串口命令行、性能分析、日志系统等） |
| `IntegrationTest/` | 集成测试用例（针对模块间交互） |
| `UnitTest/` | 单元测试用例（针对单个函数） |

## 三、模块依赖关系图

```
AppEntry(主入口+中断入口)
    ↓
AppServices  ───────┐
    ↓               │
MotorControl        │
    ↓               │
MotorAlgorithms     │
    ↓               │
SystemData ←────────┘
    ↓
Devices
    ↓
Driver
    ↓
CubeMX生成的HAL库

横向支撑层（所有层均可调用）：
Common → Config → Protocol
```