# JointMotor → ODrive Mini (STM32F405RGT6) 移植任务书

> **用途**：本文件是新 AI 会话的任务交接文档。新窗口加载后，AI 应先读本文件 + 项目 memory，再开始实施。
> **创建日期**：2026-07-13
> **源工程路径**：`d:\AAWorkSpace\001_JointMotor\SW\JointMotor`
> **目标硬件**：MKS ODRIVE MINI V1.0（STM32F405RGT6 + DRV8301）

---

## 一、任务目标

将现有 JointMotor 电机控制工程从 **STM32G473/G474 (SFOC板)** 移植到 **MKS ODRIVE MINI V1.0 (STM32F405RGT6)** 上运行。

**核心原则**：最大化复用现有代码（80%+ 不动），通过修改 IOC 配置让硬件迁就代码逻辑，而非反向改造代码。

**关键约束**：
- 项目 memory 路径：`c:\Users\dal12\.trae-cn\memory\projects\-d-AAWorkSpace-001-JointMotor-SW-JointMotor\project_memory.md`
- 必须遵守 project_memory.md 中的 Hard Constraints（如 Flash 地址、encoder 方向约定、PID 三环独立 source 机制等）
- 所有思考过程和回答使用中文（user_profile.md 约定）

---

## 二、硬件平台对比

| 项目 | 源平台 (SFOC) | 目标平台 (ODrive Mini) |
|------|------|------|
| MCU | STM32G473CBTx | STM32F405RGT6 |
| 内核 | Cortex-M4F @170MHz | Cortex-M4F @168MHz |
| Flash | 128KB(双Bank带空洞) | 1MB(单Bank连续) |
| RAM | 128KB | 192KB(128K+64K CCM) |
| 封装 | LQFP48 | LQFP64 |
| HAL库 | STM32G4xx_HAL_Driver | STM32F4xx_HAL_Driver |
| 驱动芯片 | 外置半桥 | **DRV8301DCAR**（内置buck+2路电流运放） |
| 采样电阻 | 10mΩ (3路) | **0.5mΩ (2路)** |
| 编码器 | MT6701/MT6835 (SPI1) | AS5047 或外接 (SPI3) |

---

## 三、原理图关键发现（已验证）

原理图文件：`Board/ODriveMKS/MKS ODRIVE MINI V1.0 Schematic.pdf`

### 3.1 驱动芯片 DRV8301 特性（决定性约束）

- **仅 2 路电流采样输出**：SO1(M0_A相)、SO2(M0_B相)，第三相 IC 必须由 `IC = -(IA+IB)` 计算
- SPI 配置接口（SPI_SCK/MISO/MOSI + nSCS），可设置 PWM 模式/电流放大倍数/OCP 阈值
- nFAULT 故障输出、EN_GATE 使能、DC_CAL 电流校准
- 内置 5V Buck（L1 22uH 是 DRV8301 buck 电感）

### 3.2 ADC 通道映射（原理图网络标号）

| 网络名 | MCU引脚 | STM32F4 ADC通道 | 用途 |
|---|---|---|---|
| M0_SO1 | PC0 | ADC123_IN10 | M0 A相电流 (DRV8301 SO1) |
| M0_SO2 | PC1 | ADC123_IN11 | M0 B相电流 (DRV8301 SO2) |
| M1_IC | PC2 | ADC123_IN12 | M1 C相电流（仅双电机用） |
| M1_IB | PC3 | ADC12_IN13 | M1 B相电流（仅双电机用） |
| VBUS_S | PA6 | ADC12_IN6 | 母线电压采样 |
| AUX_TEMP | PA5 | ADC12_IN5 | AUX 温度 |
| M0_TEMP | PC5 | ADC12_IN15 | M0 温度 |
| M1_TEMP | PA4 | ADC12_IN4 | M1 温度 |

### 3.3 PWM 引脚（M0 电机）

| 网络名 | MCU引脚 | 外设 | 用途 |
|---|---|---|---|
| M0_AH | PA8 | TIM1_CH1 | A相上桥 |
| M0_BH | PA9 | TIM1_CH2 | B相上桥 |
| M0_CH | PA10 | TIM1_CH3 | C相上桥 |
| M0_AL | PB13 | TIM1_CH1N | A相下桥 |
| M0_BL | PB14 | TIM1_CH2N | B相下桥 |
| M0_CL | PB15 | TIM1_CH3N | C相下桥 |
| EN_GATE | PB12 | GPIO_Output | DRV8301使能 |
| nFAULT | PD2 | GPIO_Input | DRV8301故障输入(上拉) |

### 3.4 SPI 编码器

- SPI3: PC10=SCK, PC11=MISO, PC12=MOSI
- M0_nCS = PC13, M1_NCS = PC14（GPIO软件控制）
- 原理图有 AS5047 焊接位置（U8），也可外接编码器到 J6 接口

### 3.5 通信接口

- CAN: PB8=RX, PB9=TX (SN65HVD232)
- UART4: PA0=TX, PA1=RX (SharedStack，与 GPIO_1/GPIO_2 复用)
- USB: PA11=DM, PA12=DP (Type-C)
- 无 USART1 独立引脚（当前代码用 USART1 PB6/PB7，ODrive 无此走线）

### 3.6 辅助功能

- AUX_L/AUX_H: PB10/PB11 (TIM2_CH3/CH4 PWM)
- M0_ENC_A/B/Z: PB4/PB5/PC9 (TIM3 编码器接口，若用 ABZ 编码器)
- M1_ENC_A/B/Z: PB6/PB7/PC15 (TIM4 编码器接口)

---

## 四、当前代码架构（需了解的关键点）

### 4.1 分层结构

```
CubeMX HAL (Board/<板>/Core/Src/*.c)
    ↓ __weak HAL 句柄
drv_xxx 驱动层 (User/Driver/drv_*.c) - 句柄查找表封装
    ↓ drv_xxx 接口
dev_xxx 设备对象层 (User/Devices/dev_*.c) - OOP+配置表
    ↓ dev_motor_t 接口
MotorControl 控制层 (User/MotorControl/) - FOC/PID/SVPWM
    ↓
AppServices 业务层 (User/AppServices/) - FreeRTOS任务/协议栈
```

### 4.2 板级选择机制

- `User/Config/board_select.h` 通过宏 `JM_BOARD_SFOC` / `JM_BOARD_V1` / **`JM_BOARD_ODRIVE`(新增)** 切换
- 每块板子有独立的 `Board/<板>/Config/dev_config_board.{h,inc}`
- `.h` 定义使能设备集（`USE_DEV_*`），`.inc` 定义配置表实例（引脚/通道/句柄映射）

### 4.3 控制环触发链（关键！）

```
TIM1 中心对齐计数 → TIM1_CC4 比较匹配(上升沿)
   → 触发 ADC1 注入组转换(ADC_EXTERNALTRIGINJEC_T1_CC4)
   → 2路相电流采样完成(JDR1/JDR2)
   → ADC1_2_IRQHandler (G4) / ADC_IRQHandler (F4)
   → HAL_ADCEx_InjectedConvCpltCallback (= CURRENT_LOOP_IRQ_TASK)
   → motor_loop_isr() [电流10kHz / 速度2kHz / 位置1kHz 分频]
```

- ISR 入口文件：`User/AppEntry/control_irq.c`
- `CURRENT_LOOP_IRQ_TASK` 宏定义在 `User/Config/system_config.h` 第12行
- `UVW_CURRENT_U_HANDLE` = `hadc1`（system_config.h 第6行）
- motor_loop_isr 实现在 `User/MotorControl/CascadeControl/motor_loop.c`

### 4.4 关键配置参数

- `motor_loop_config.h`: `MOTOR_LOOP_VEL_DIV=5`, `MOTOR_LOOP_POS_DIV=10`（10kHz基频下→速度2kHz/位置1kHz）
- `dev_motor.h`: `DEV_MOTOR_ENCODER_TYPE = DEV_MOTOR_ENCODER_MT6701`
- `dev_config.h`: `HALF_BRIDGE_ADC_TRIG_CCR=8480`, `PHASE_CURRENT_SHUNT=0.01`, `PHASE_CURRENT_GAIN=50`, `PHASE_CURRENT_ZERO_ADC=2048`
- `motor_info_storage.h`: `MOTORINFO_FLASH_START_ADDR=0x0804F000`（G4 Bank2末尾，F4不可用）

### 4.5 Flash 存储子系统（必须重写）

三层架构：
1. `User/AppServices/ParamService/motor_info_storage.{c,h}` - 业务封装+CRC32
2. `User/Devices/dev_flash.{c,h}` - 通用页存储+磨损均衡（magic+sequence标志）
3. `User/Driver/drv_flash_g4.{c,h}` - **G4专属HAL封装，F4完全不可用**

G4 vs F4 Flash 差异：
- G4: 双Bank(带空洞), Page擦除(2KB/4KB), 64-bit doubleword编程
- F4: 单Bank(连续), Sector擦除(16KB~128KB大小不一), 32-bit word/halfword编程

F405RG Sector 划分：
- Sector 0~3: 各 16KB (0x08000000~0x0800FFFF)
- Sector 4: 64KB (0x08010000~0x0801FFFF)
- Sector 5~7: 各 128KB (0x08020000~0x0807FFFF)

---

## 五、严重不兼容项（必须处理）

### 5.1 Flash 存储子系统（彻底重写）★★★★★

**问题**：`drv_flash_g4.c` 使用 G4 特有 API：
- `FLASH->OPTR & FLASH_OPTR_DBANK`（F4无此选项位）
- `HAL_FLASHEx_Erase` + `FLASH_TYPEERASE_PAGES`（F4只有`FLASH_TYPEERASE_SECTORS`）
- `FLASH_TYPEPROGRAM_DOUBLEWORD`（F4用`FLASH_TYPEPROGRAM_WORD`）
- `STM32G4_FLASH_BANK2_BASE = 0x08040000`（F4上是普通代码地址）

**方案**：
- 新增 `User/Driver/drv_flash_f4.{c,h}`，接口与 `drv_flash_g4.c` 完全一致
- 修改 `motor_info_storage.h` 地址宏：
  - `MOTORINFO_FLASH_START_ADDR` → `0x08060000`（Sector 7起始，128KB）
  - `MOTORINFO_FLASH_PAGE_SIZE` → `16384`（最小Sector 16KB）
- Keil 链接脚本限制代码区在 `0x08060000` 之前

### 5.2 相电流采样架构（2路改造）★★★★★

**问题**：DRV8301 只输出 2 路相电流（SO1/SO2），当前代码假设 3 路独立采样。

**方案 A（推荐）**：
- IOC：PC0/PC1 配置为 ADC1 注入组 Rank1/2（IN10/IN11）
- `dev_config_board.inc` 的 `phase_current_list` 改为 2 路有效
- `dev_motor_phase_current.c` 修改 IC 计算：`ic = -(ia+ib)`（约3行改动）

**方案 B**：双 ADC 并行（dual regular simultaneous 模式），复杂度高，不推荐

### 5.3 采样电阻参数差异 ★★★★

- 当前: `PHASE_CURRENT_SHUNT=0.01` (10mΩ), `PHASE_CURRENT_GAIN=50`
- ODrive: R16/R17 = 0.5mΩ, DRV8301 内部增益可配置（默认 10/20/40/80 V/V）
- 需修改 `dev_config.h` 的 `PHASE_CURRENT_SHUNT` 和 `PHASE_CURRENT_GAIN`
- 计算公式：`I = (ADC - 2048) * VREF / 4096 / Gain / Shunt`

### 5.4 中断号名称差异 ★★★

| 用途 | G4 (当前) | F4 (ODrive) |
|---|---|---|
| ADC 注入完成 | `ADC1_2_IRQn` | `ADC_IRQn` |
| TIM1 更新 | `TIM1_UP_TIM16_IRQn` | `TIM1_UP_TIM10_IRQn` |
| TIM1 刹车 | `TIM1_BRK_TIM15_IRQn` | `TIM1_BRK_TIM9_IRQn` |
| DMA | `DMA1_Channel1_IRQn` | `DMA1_Stream0_IRQn` (Stream而非Channel) |

User/ 代码不直接引用中断号（都通过 HAL 回调），只需替换 `Board/ODriveMKS/Core/Src/stm32f4xx_it.c`。

### 5.5 编码器型号适配 ★★★★

- 若用 AS5047：需新增 `dev_encoder_as5047.c`（参考 `dev_encoder_mt6701.c` 结构），16-bit帧+PAR校验
- 若外接 MT6701：保持 `DEV_MOTOR_ENCODER_TYPE=DEV_MOTOR_ENCODER_MT6701`，仅改 SPI 引脚配置

---

## 六、IOC 配置修改清单

当前 IOC 文件：`Board/ODriveMKS/OdriveMini.ioc`

| 修改项 | 当前值 | 目标值 | 原因 |
|---|---|---|---|
| ADC1 注入通道数 | 1 (CH6) | **2 (IN10 + IN11)** | 采 SO1/SO2 两相电流 |
| ADC1 注入触发源 | T1_TRGO | **T1_CC4** | 保持代码现有触发逻辑 |
| ADC1 注入采样时间 | 3 cycles | **47.5 cycles** | DRV8301 SO 输出阻抗较高 |
| ADC2/ADC3 注入组 | 各1通道 | **全部删除** | 仅用 ADC1 |
| ADC2/ADC3 外设 | 启用 | **可禁用** | 简化 |
| TIM1 Period | 3500 | **8400** | 保持 10kHz PWM (168MHz/8400/2) |
| TIM1 RCR | 2 | **0** | 每周期触发ADC（10kHz控制环） |
| TIM1 CounterMode | CENTERALIGNED3 | **CENTERALIGNED1** | 对齐代码逻辑 |
| TIM1 CH4 | Output Compare No Output | **保持，CCR=8380** | ADC 触发 |
| TIM1 DeadTime | 20 | **0** | DRV8301 内部处理死区 |
| TIM1 OCMode | PWM2 | **PWM1 或保持PWM2** | 需与代码极性匹配 |
| SPI3 | Full Duplex 16-bit | **RX_ONLY 8-bit** | MT6701 是只读 |
| SPI3 BaudRatePrescaler | 16 | **32** | 168MHz/32=5.25MHz（MT6701上限15.6MHz） |
| UART4 波特率 | 默认115200 | **2000000** | 匹配代码协议（APB1=42MHz下误差0.8%） |
| PB12 | GPIO_Output EN_GATE | 保持 | 已正确 |
| PD2 | GPIO_Input nFAULT | 保持 | 接入故障检测 |
| FDCAN1 | 启用 | **可禁用** | 当前代码未用 CAN |
| USB_DEVICE | 启用 | **禁用** | 简化移植 |
| TIM2 CH3/CH4 | AUX PWM | **可禁用** | 当前用 TIM2 作 FSM |
| TIM8 | 启用 M1 PWM | **禁用** | 仅用 M0 |
| TIM3/TIM4 | 编码器接口 | **可禁用** | 用 SPI 编码器 |
| NVIC TimeBase | TIM14 | **保持或改 TIM1** | F4上TIM14是基本定时器 |

---

## 七、代码改动清单（最小化）

### 7.1 新增文件

| 文件 | 说明 | 工作量 |
|---|---|---|
| `Board/ODriveMKS/Config/dev_config_board.h` | ODrive板使能设备集 | 小 |
| `Board/ODriveMKS/Config/dev_config_board.inc` | ODrive板配置表（引脚/通道映射） | 中 |
| `Board/ODriveMKS/Core/*` | CubeMX 重新生成 F4 HAL 代码 | 大（CubeMX自动） |
| `User/Driver/drv_flash_f4.c` | F4 Flash HAL 封装（接口同 drv_flash_g4） | 大 |
| `User/Driver/drv_flash_f4.h` | F4 Flash 头文件 | 小 |
| `Board/ODriveMKS/MDK-ARM/startup_stm32f405xx.s` | F405 启动文件 | 提供 |
| `Board/ODriveMKS/MDK-ARM/stm32f405xx_flash.sct` | F405 链接脚本 | 提供 |
| **可选** `User/Devices/dev_encoder_as5047.c` | AS5047 适配层（若用 AS5047） | 中 |

### 7.2 修改文件

| 文件 | 改动内容 | 工作量 |
|---|---|---|
| `User/Config/board_select.h` | 新增 `JM_BOARD_ODRIVE` 分支 | 小 |
| `User/Devices/dev_motor_phase_current.c` | IC = -(IA+IB) 计算（约3行） | 小 |
| `User/AppServices/ParamService/motor_info_storage.h` | Flash 地址宏改为 F4 Sector 7 | 小 |
| `User/Config/system_config.h` | `UVW_CURRENT_U_HANDLE` 等宏确认（可能无需改） | 小 |
| `User/Devices/dev_config.h` | `PHASE_CURRENT_SHANT=0.0005`, `PHASE_CURRENT_GAIN` 按DRV8301配置 | 小 |
| `User/MotorControl/CascadeControl/motor_loop_config.h` | 确认分频系数（10kHz下无需改） | 小 |

### 7.3 dev_config_board.inc 配置表要点

```c
/* MT6701/AS5047 编码器 */
.mt6701_list[0] = {
    .spi_num = {.hspi = DRV_SPI3},                              // SPI1 → SPI3
    .csn = {.gpiox = DRV_GPIOC, .pin = DRV_PIN_13, .ste = DRV_PIN_LOW},  // PA4 → PC13
}

/* 半桥 */
.half_bridge_list[0] = {
    .tim = DRV_TIM1,
    .channel = {TIM_CH1, TIM_CH2, TIM_CH3, TIM_CH4},  // 保持
}

/* 电源监控 */
.power_monitor_list[PM_VBUS] = {.id = DRV_ADC_1, .channel = DRV_ADC_CH6},   // PA6
/* IBUS 可选：DRV8301无独立IBUS，用三相合成（PM_IBUS_SOURCE=1） */

/* 相电流（2路有效） */
.phase_current_list[ADCX_IA] = {.id = DRV_ADC_1, .channel = DRV_ADC_CH10},  // PC0 SO1
.phase_current_list[ADCX_IB] = {.id = DRV_ADC_1, .channel = DRV_ADC_CH11},  // PC1 SO2
.phase_current_list[ADCX_IC] = {.id = DRV_ADC_1, .channel = DRV_ADC_CH10},  // 占位，IC由计算得出

/* UART 通信 */
.commun_uart_list[0] = {
    .uart = DRV_UART4,   // USART1 → UART4
    .motor_id = 1,
}

/* motor EN */
.motor_enable_list[0] = {
    {"MOTOR1_EN", {DRV_GPIOB, DRV_PIN_12, 0}},  // PB2 → PB12 (EN_GATE)
}

/* 无 RGB LED，不定义 USE_DEV_RGB_LED */
```

### 7.4 可保持不变的代码（80%+）

- 全部 `User/MotorControl/` 算法层（FOC/PID/SVPWM/motor_loop）
- 全部 `User/AppServices/` 业务层（除 motor_info_storage 地址宏）
- 全部 `User/DataHub/`、`User/Common/` 工具层
- `drv_rtos.c`、`drv_dwt_timer.c`（内核相关，MCU无关）
- `drv_gpio.c`、`drv_tim.c`、`drv_spi.c`、`drv_usart.c`（HAL句柄查找表，自动适配F4句柄）
- `dev_motor.c`、`dev_encoder_mt6701.c`、`dev_mt6701.c` 等设备对象（仅配置表改）
- CMSIS-DSP 库（Cortex-M4F 兼容）

---

## 八、移植实施步骤

### 步骤 1：建立 ODriveMKS 工程骨架

- 参考 `Board/SFOC/` 目录结构，在 `Board/ODriveMKS/` 下创建 `Config/`、`Core/`、`Drivers/`、`MDK-ARM/`、`Middlewares/`
- 用 CubeMX 打开 `OdriveMini.ioc`，按第六节清单修改配置，重新生成代码到 `Core/`
- 芯片选 STM32F405RGTx，Toolchain 选 MDK-ARM

### 步骤 2：新增 drv_flash_f4.c/h

- 参考 `drv_flash_g4.c` 的接口签名（`drv_flash_read/write/clear`）
- 用 `HAL_FLASHEx_Erase` + `FLASH_TYPEERASE_SECTORS` 实现
- 用 `FLASH_TYPEPROGRAM_WORD` (32-bit) 替代 doubleword
- 页缓冲区大小改为 16KB（最小Sector）
- 修改 `motor_info_storage.h` 地址宏指向 Sector 7

### 步骤 3：新增 dev_config_board.{h,inc}

- 参考 `Board/SFOC/Config/dev_config_board.{h,inc}` 结构
- 按第七节 7.3 要点修改配置表
- `dev_config_board.h` 不定义 `USE_DEV_RGB_LED`，其他设备保持启用

### 步骤 4：修改 board_select.h

```c
#elif defined(JM_BOARD_ODRIVE)
  #include "dev_config_board.h"
  #define JM_BOARD_DEV_CONFIG_INC "dev_config_board.inc"
#endif
```

### 步骤 5：修改 dev_motor_phase_current.c

在 `update` 方法中把 IC 读取改为计算：
```c
pobj->adc.a = drv_adc_injected_get_value(pobj->src[ADCX_IA].id, pobj->src[ADCX_IA].rank);
pobj->adc.b = drv_adc_injected_get_value(pobj->src[ADCX_IB].id, pobj->src[ADCX_IB].rank);
pobj->adc.c = -(pobj->adc.a + pobj->adc.b);  // 基尔霍夫电流定律
```

### 步骤 6：适配 ISR 触发链

- `Board/ODriveMKS/Core/Src/stm32f4xx_it.c` 的 `ADC_IRQHandler` 调用 `HAL_ADC_IRQHandler(&hadc1)`
- HAL 内部派发到 `HAL_ADCEx_InjectedConvCpltCallback`（即 `CURRENT_LOOP_IRQ_TASK`）
- `control_irq.c` 判断 `hadc->Instance == UVW_CURRENT_U_HANDLE.Instance`（即 hadc1）进入 motor_loop_isr

### 步骤 7：修改 dev_config.h 参数

- `PHASE_CURRENT_SHUNT` = 0.0005（0.5mΩ）
- `PHASE_CURRENT_GAIN` = 40（DRV8301 默认增益，需确认 SPI 配置）
- `PM_VBUS_RATIO` 确认（ODrive 分压电阻 R43=18K, R44=1K → 19:1，当前 11.0）

### 步骤 8：Keil 工程配置

- 设备选 `STM32F405RGTx`
- 预定义宏: `USE_HAL_DRIVER, STM32F405xx, JM_BOARD_ODRIVE`
- 启动文件: `startup_stm32f405xx.s`
- 链接脚本: `stm32f405xx_flash.sct`，代码区限制在 `0x08060000` 之前
- 替换 HAL 库为 STM32F4xx_HAL_Driver
- 替换 CMSIS Device 为 STM32F405

### 步骤 9：确认编码器型号

- 若焊接 AS5047：新增 `dev_encoder_as5047.c`，修改 `DEV_MOTOR_ENCODER_TYPE`
- 若外接 MT6701：保持现有 MT6701 适配层，仅改 SPI 配置

### 步骤 10：编译验证

- 先用虚拟电机模式（`MOTOR_LOOP_ENABLE_DEV_DRIVER=0`）验证编译通过
- 再切到真实驱动模式（`=1`）验证硬件链接

---

## 九、关键约束和注意事项

### 9.1 必须遵守的 Hard Constraints（来自 project_memory.md）

1. **Virtual motor 目标位置单位**必须用电机侧弧度（非输出侧），与 `motor_control.c:190` 的 `ref->pos=cmd->pos` 一致（不应用 gear_ratio）
2. **位置限幅** ±12.566 弧度（电机侧）
3. **编码器刷新**必须在所有模式（IDLE/READY/DUTY）中执行，`encoder.update()` 和 `motor_param.update()` 放在 `motor_loop_isr` 开头
4. **`motor_param_init()`** 必须在 `motor_loop_init()` 中调用
5. **电流零偏校准**必须在 TIM1 和 ADC 注入组启动后进行，延时 2ms 等待有效采样
6. **相电流零偏**默认 2048（对应 1.65V REF）
7. **编码器方向** enc_direction 用 -1/1 约定（CW=1, CCW=-1）
8. **Flash 中 enc_direction=0** 必须回退为默认值 1
9. **PID 三环独立 source** 机制（DEFAULT/FLASH/AUTOTUNE/DEBUG）
10. **`pid_flash_valid_magic`** 验证值 `0xC0DE2014u`

### 9.2 ODrive 特有注意事项

1. **DRV8301 SPI 配置**：上电后需通过 SPI 配置 DRV8301 寄存器（PWM模式、增益、OCP阈值），否则驱动不工作。当前代码无此逻辑，需新增 `dev_drv8301.c` 或在 `dev_motor_init` 中加初始化
2. **nFAULT 接入**：PD2 接入 `system_state.c` 故障检测，DRV8301 过流/过温时拉低
3. **DC_CAL 校准**：DRV8301 有 DC_CAL 引脚可校准电流零偏，需确认硬件走线（原理图未明确标注）
4. **采样电阻 0.5mΩ**：比当前 10mΩ 小 20 倍，电流分辨率降低，需评估 ADC 噪声
5. **F405 没有 ADC 校准**：`HAL_ADCEx_Calibration_Start` 在 F4 上不支持，需删除或改用软件零偏校准
6. **F4 DMA 是 Stream 而非 Channel**：`DMA1_Stream0` 等命名，CubeMX 会自动生成
7. **CCM RAM**：F405 有 64KB CCM RAM（`0x10000000`），可放高频访问数据，但 FreeRTOS 任务栈不能放（无法 DMA 访问）

### 9.3 不要改动的代码

- `User/MotorControl/` 全部算法层
- `User/Common/pid_core.c`（PID 核心）
- `User/AppServices/ParamService/motor_info_storage.c`（业务逻辑，仅改地址宏）
- `User/Devices/dev_flash.c`（通用磨损均衡层，仅改 drv_flash 底层）

---

## 十、参考资料

- 源工程 SFOC 板配置：`Board/SFOC/Config/dev_config_board.{h,inc}`
- 源工程 SFOC CubeMX 配置：`Board/SFOC/sfoc.ioc`
- 源工程 SFOC HAL 代码：`Board/SFOC/Core/Src/*.c`
- 驱动层参考：`User/Driver/drv_flash_g4.c`（F4 版本的接口模板）
- 设备对象参考：`User/Devices/dev_encoder_mt6701.c`（AS5047 适配层模板）
- 控制环参考：`User/MotorControl/CascadeControl/motor_loop.c`
- ISR 入口参考：`User/AppEntry/control_irq.c`
- 项目 memory：`c:\Users\dal12\.trae-cn\memory\projects\-d-AAWorkSpace-001-JointMotor-SW-JointMotor\project_memory.md`

---

## 十一、新窗口启动指令模板

在新 AI 窗口发送以下指令启动任务：

```
请阅读 d:\AAWorkSpace\001_JointMotor\SW\JointMotor\Board\ODriveMKS\PORTING_TASK_BRIEF.md
了解 JointMotor → ODrive Mini (STM32F405RGT6) 移植任务的完整背景。

我的当前工作目录：d:\AAWorkSpace\001_JointMotor\SW\JointMotor

请先加载项目 memory（c:\Users\dal12\.trae-cn\memory\projects\-d-AAWorkSpace-001-JointMotor-SW-JointMotor\project_memory.md），
然后从【步骤 X】开始实施移植工作。

[具体任务说明，例如：]
- 帮我修改 Board/ODriveMKS/OdriveMini.ioc 配置文件（按第六节清单）
- 帮我新增 User/Driver/drv_flash_f4.c/h
- 帮我新增 Board/ODriveMKS/Config/dev_config_board.{h,inc}
- 帮我修改 board_select.h 加 ODrive 分支
```

---

**文档结束**
