# 主仓 Git 改动按功能分提交方案

## 目标
将主仓当前未提交的 29 个 modified 文件 + 9 个 untracked 文件(含 4 个新驱动 + 1 个新头)按功能拆分为独立的语义化提交，所有提交在新建分支 `develop_mks` 上完成。submodule（Board/ODriveMKS, Board/SFOC）内部改动不在本次范围。

## 当前改动总览

- modified: 29 个文件（356+ / 107-）
- untracked:
  - 新驱动: `User/Devices/dev_drv8301.c/h`、`User/Devices/dev_as5047.c/h`、`User/Devices/dev_encoder_as5047.c/h`、`User/Driver/drv_flash_f4.c/h`
  - 新头: `User/Config/mcu_compat.h`
  - 文档: `User/Data/docs/git-commit/SKILL.md`、`User/Data/plans/2026-07-13-odrive-mini-migration.md`、`User/Data/plans/2026-07-15-user-code-sfoc-compatibility.md`、`User/MotorControl/docs/MotorControl_说明文档.md`

## 提交方案（按依赖顺序）

### Step 0: 创建新分支
```bash
git checkout -b develop_mks
```
所有后续 commit 都在新分支上。当前 HEAD = e870df4（master）。

---

### Commit 1: `feat(mcu_compat): 新增跨系列 MCU 兼容判定头`

**目的**: 引入 `mcu_compat.h` 作为 MCU 系列判定的单一入口，替换散落在 drv_*.c 中的 `STM32F4 / STM32G4 / STM32H7` 硬编码宏。是后续 F4 移植的基础。

**文件**:
- 新增: `User/Config/mcu_compat.h` (JM_MCU_F1/F3/F4/G4/L4/H7 + JM_PERIPH_CAN_CLASSIC/FD + JM_PERIPH_ADC_CALIB_DUAL/SINGLE)
- 修改: `User/Driver/drv_config.h` — FLASH_MCU 由手动宏改为 `mcu_compat.h` 自动派生 USE_FLASH_F4_DRIVER/USE_FLASH_G4_DRIVER
- 修改: `User/Driver/drv_adc.c` — `STM32G4||STM32L4||STM32H7` → `JM_PERIPH_ADC_CALIB_DUAL_PARAM`，`STM32F1||STM32F3` → `JM_PERIPH_ADC_CALIB_SINGLE_PARAM`；ADC_SINGLE_ENDED/ADC_DIFFERENTIAL_ENDED 加 `#if defined` 守护
- 修改: `User/Driver/drv_can.c` — `STM32F4` → `JM_PERIPH_CAN_CLASSIC`，`STM32G4||STM32H7` → `JM_PERIPH_CAN_FD`
- 修改: `User/Devices/dev_dwt_counter.c` — 按 `STM32F405xx`/`STM32G474xx` 条件包含对应 HAL 头
- 修改: `User/Driver/drv_usart.c` — **仅** `#include "stm32g4xx_hal.h"` → `#include "main.h"` 这一行（用 `git add -p` 选择，其他改动留给 Commit 9）

```bash
git add User/Config/mcu_compat.h
git add User/Driver/drv_config.h User/Driver/drv_adc.c User/Driver/drv_can.c User/Devices/dev_dwt_counter.c
git add -p User/Driver/drv_usart.c   # 只选 #include "main.h" 那一行
```

---

### Commit 2: `feat(board): 新增 JM_BOARD_ODRIVE 板级选择与板级参数覆盖机制`

**目的**: 引入 ODrive Mini 板宏切换，并通过 `#ifndef` 守护机制允许 board 配置文件覆盖通用参数（PM_VBUS_RATIO、HALF_BRIDGE_ADC_TRIG_CCR、PHASE_CURRENT_* 等）。

**文件**:
- 修改: `User/Config/board_select.h` — 加 `JM_BOARD_ODRIVE` 选项与互斥检查；新增 ODrive board include 路径
- 修改: `User/Devices/dev_config.h` — `PM_VBUS_RATIO`、`HALF_BRIDGE_ADC_TRIG_CCR`、`PHASE_CURRENT_GAIN`、`PHASE_CURRENT_SHUNT`、`PHASE_CURRENT_ZERO_ADC` 全部加 `#ifndef` 守护；为 `HALF_BRIDGE_ADC_TRIG_CCR` 加 F4 Period=8400 注释

```bash
git add User/Config/board_select.h User/Devices/dev_config.h
```

---

### Commit 3: `feat(drv_flash): 新增 F4 Flash 驱动(扇区擦写)`

**目的**: 为 STM32F405RG 提供扇区擦写 Flash 驱动，跳过 G4 路径的整页读改写（128KB 缓冲会撑爆 RAM）。

**文件**:
- 新增: `User/Driver/drv_flash_f4.c` (205 行)
- 新增: `User/Driver/drv_flash_f4.h`
- 修改: `User/Devices/dev_flash.c` — `dev_flash_write` 加 `#if USE_FLASH_F4_DRIVER` 分支，单扇区擦写 + flag+data 一次写入
- 修改: `User/Devices/dev_flash.h` — 按 `USE_FLASH_G4_DRIVER`/`USE_FLASH_F4_DRIVER` 包含对应头
- 修改: `User/AppServices/ParamService/motor_info_storage.h` — `MOTORINFO_FLASH_START_ADDR`、`MOTORINFO_FLASH_TOTAL_SIZE`、`MOTORINFO_FLASH_PAGE_SIZE` 加 `#ifndef` 守护，允许 ODrive 板覆盖为 Sector 11 (0x080E0000, 128KB)

```bash
git add User/Driver/drv_flash_f4.c User/Driver/drv_flash_f4.h
git add User/Devices/dev_flash.c User/Devices/dev_flash.h
git add User/AppServices/ParamService/motor_info_storage.h
```

---

### Commit 4: `fix(motor_info_storage): 保存前阻断范围校验失败并增加回读校验`

**目的**: 修复"保存时绕过范围校验、下次上电必然 verify 失败重走默认路径"的隐患；并在写入后立即回读走与上电相同的校验链，当次就能区分擦写失败 vs 数据校验失败。

**文件**:
- 修改: `User/AppServices/ParamService/motor_info_storage.c` — `motorinfo_ops_save` 改 `(void)motor_info_validate` 为阻断式；写入后回读并调用 `motorinfo_verify`；`init` 路径 B3 捕获 save 返回值

```bash
git add User/AppServices/ParamService/motor_info_storage.c
```

---

### Commit 5: `feat(drv8301): 新增 DRV8301 门极驱动器与 2-shunt 相电流采样`

**目的**: 新增 DRV8301 SPI 驱动（CTRL1/CTRL2 配置 GAIN=40V/V、DC_CAL=0），并在相电流采样层支持 2-shunt 模式（合成相由基尔霍夫定律计算）。

**文件**:
- 新增: `User/Devices/dev_drv8301.c` (116 行)
- 新增: `User/Devices/dev_drv8301.h`
- 修改: `User/Devices/dev_config.c` — **仅** `#include "dev_drv8301.h"` 这一行（用 `git add -p`）
- 修改: `User/Devices/dev_motor_phase_current.c` — `start` 跳过合成相 rank 分配；`get_value` 加 `DRV8301_TWO_PHASE_SYNTH_PHASE` 分支用基尔霍夫合成
- 修改: `User/AppEntry/user_interface.c` — `hardware_init` 加 `#if defined(USE_DEV_DRV8301)` 包裹的 `dev_drv8301_init + g_dev_drv8301.init`；`tim.h` 改为 `MOTOR_LOOP_ENABLE_DEV_DRIVER==0` 条件包含

```bash
git add User/Devices/dev_drv8301.c User/Devices/dev_drv8301.h
git add -p User/Devices/dev_config.c          # 只选 dev_drv8301.h 那一行
git add User/Devices/dev_motor_phase_current.c
git add User/AppEntry/user_interface.c
```

---

### Commit 6: `feat(as5047): 新增 AS5047 编码器驱动并接入 dev_motor`

**目的**: 新增 AS5047 编码器底层驱动 + dev_motor 适配层，将其作为 `DEV_MOTOR_ENCODER_AS5047` 选项接入电机对象。

**文件**:
- 新增: `User/Devices/dev_as5047.c` (171 行)
- 新增: `User/Devices/dev_as5047.h`
- 新增: `User/Devices/dev_encoder_as5047.c`
- 新增: `User/Devices/dev_encoder_as5047.h`
- 修改: `User/Devices/dev_config.c` — **仅** `#include "dev_as5047.h"` 这一行（用 `git add -p`，与 Commit 5 拆分）
- 修改: `User/Devices/dev_motor.c` — **仅** AS5047 encoder 分支两处（`#elif DEV_MOTOR_ENCODER_AS5047` include + `dev_encoder_as5047_create` 调用），用 `git add -p`，**不选** ODrive EN_GATE 块
- 修改: `User/Devices/dev_motor.h` — 加 `DEV_MOTOR_ENCODER_AS5047 = 3`，默认改为 `DEV_MOTOR_ENCODER_AS5047`

```bash
git add User/Devices/dev_as5047.c User/Devices/dev_as5047.h
git add User/Devices/dev_encoder_as5047.c User/Devices/dev_encoder_as5047.h
git add -p User/Devices/dev_config.c          # 只选 dev_as5047.h 那一行
git add -p User/Devices/dev_motor.c            # 只选 AS5047 分支两处, 跳过 ODrive EN_GATE
git add User/Devices/dev_motor.h
```

---

### Commit 7: `feat(dev_motor): 支持 ODrive 板 EN_GATE 引脚(PB12)`

**目的**: 通过 `JM_BOARD_ODRIVE` 宏切换 EN_GATE 引脚：ODrive Mini = PB12，SFOC/V1 = PB2。

**文件**:
- 修改: `User/Devices/dev_motor.c` — **仅** `#if defined(JM_BOARD_ODRIVE)` EN_GATE 配置块（用 `git add -p`，与 Commit 6 拆分）

```bash
git add -p User/Devices/dev_motor.c            # 只选 ODrive EN_GATE 块
```

---

### Commit 8: `feat(power_monitor): 支持合成源通道并修正 DMA rank 分配`

**目的**: 引入 `ch_rank` 字段记录每个逻辑通道在其 ADC 的 DMA 序列中的 rank；合成源通道（无 ADC 配置）跳过校准与 DMA 启动，避免 `assert_report(0)`。

**文件**:
- 修改: `User/Devices/dev_power_monitor.c` — `start` 阶段分配 rank 并跳过合成源；`get_vbus`/`get_ibus` 用 `ch_rank[PM_*]` 索引
- 修改: `User/Devices/dev_power_monitor.h` — 结构体加 `ch_rank[PM_CH_MAX]`

```bash
git add User/Devices/dev_power_monitor.c User/Devices/dev_power_monitor.h
```

---

### Commit 9: `fix(drv_usart): 修复 F4 HAL UART ORE 错误导致接收永久瘫痪`

**目的**: 覆写 `HAL_UART_ErrorCallback`，在 F4 HAL 因 DMA 总线负载高触发 ORE/NE/FE 错误并永久禁用 DMAR 位时，清标志 + 停 DMA + 重启 DMA 接收 + 重新使能 IDLE 中断。G4 上为冗余保护不影响功能。

**文件**:
- 修改: `User/Driver/drv_usart.c` — **仅** `HAL_UART_ErrorCallback` 函数实现（用 `git add -p`，跳过头文件 include 那一行，那一行已在 Commit 1 提交）

```bash
git add -p User/Driver/drv_usart.c             # 只选 HAL_UART_ErrorCallback 函数体
```

---

### Commit 10: `feat(motor_profile): 新增 MKS 5010 360KV 电机配置`

**目的**: 新增 `MOTOR_PROFILE_5010_360` 选项，使用厂家参数（R=0.12Ω, L=50μH, 极对数=7, Imax=20A, 12~24V）。同步递增 `MOTOR_PROFILE_CONFIG_VERSION` 1→2 触发 Flash 参数重新初始化。

**文件**:
- 修改: `User/Config/motor_profile.h` — 加 `MOTOR_PROFILE_5010_360 = 2` 宏定义与参数块；`MOTOR_PROFILE_DEMO` 编号顺延为 3；`MOTOR_PROFILE_CONFIG_VERSION` 改为 2U

```bash
git add User/Config/motor_profile.h
```

---

### Commit 11: `fix(calib): 适配低电阻电机 R 标定(宽范围+电流限流+允许负电流)`

**目的**: 解决 MKS 5010 (R=0.12Ω) R 标定持续失败问题：
- R 校验范围由 `MOTOR_R × [0.1, 10]` 改为固定宽范围 `[0.01, 50]` Ω，避免低电阻电机死区非线性导致误拦
- 测试电流基准改为 `min(峰值 × 0.3, 1.5A)`，避免超出电源 2A 限流导致电压塌陷
- 电流方向检查由 `id < 0.001f` 改为 `fabsf(id) < 0.001f`，允许 DRV8301 板电流极性反转
- 增加 `r_id_high/r_d_id/r_d_v/r_result` 诊断字段便于调试器观察

**文件**:
- 修改: `User/MotorCalibration/calib_config.h` — 测试电流公式 + R 范围常量
- 修改: `User/MotorCalibration/calib_level2_motor.c` — `fabsf` 替换两处 + 诊断字段记录

```bash
git add User/MotorCalibration/calib_config.h User/MotorCalibration/calib_level2_motor.c
```

---

### Commit 12: `fix(motor_info): 放宽 torque_constant 下限支持低 KT 电机`

**目的**: MKS 5010 KV=360 反算 KT ≈ 0.398e-3 Nm/A，原下限 0.001 会误拦，改为 0.00001。同时同步 Tools 下自动生成源，保持 CSV → 生成器 → 数据文件链一致。

**文件**:
- 修改: `User/DataHub/motor_info.c` — `motor_info_validate` 与 `motor_info_set_torque_constant` 下限 0.001→0.00001
- 修改: `User/DataHub/motor_info.h` — 自动生成日期 2026-07-09 → 2026-07-15
- 修改: `User/Tools/motor_info_gen/motor_info.c` — 同 DataHub 副本
- 修改: `User/Tools/motor_info_gen/motor_info.h` — 同 DataHub 副本
- 修改: `User/Tools/motor_info_gen/motor_info.csv` — torque_constant min 列 0.001 → 0.00001

```bash
git add User/DataHub/motor_info.c User/DataHub/motor_info.h
git add User/Tools/motor_info_gen/motor_info.c User/Tools/motor_info_gen/motor_info.h User/Tools/motor_info_gen/motor_info.csv
```

---

### Commit 13: `style(control_irq): 修正电流环中断频率注释为 10KHZ`

**目的**: 仅注释修改，无逻辑变化。

**文件**:
- 修改: `User/AppEntry/control_irq.c` — `20KHZ` → `10KHZ`

```bash
git add User/AppEntry/control_irq.c
```

---

### Commit 14: `docs: 归档方案文档与 MotorControl 说明`

**目的**: 把本次开发期间产生的方案文档与说明文档一次性归档，避免散落 untracked。

**文件**:
- 新增: `User/Data/docs/git-commit/SKILL.md`
- 新增: `User/Data/plans/2026-07-13-odrive-mini-migration.md`
- 新增: `User/Data/plans/2026-07-15-user-code-sfoc-compatibility.md`
- 新增: `User/MotorControl/docs/MotorControl_说明文档.md`

```bash
git add User/Data/docs/git-commit/SKILL.md
git add User/Data/plans/2026-07-13-odrive-mini-migration.md
git add User/Data/plans/2026-07-15-user-code-sfoc-compatibility.md
git add User/MotorControl/docs/MotorControl_说明文档.md
```

---

## 假设与决策

- **不触碰 submodule**：Board/ODriveMKS、Board/SFOC 内部改动本次不处理，相关 submodule 指针保持原样。
- **新建分支 `develop_mks`**：所有提交在该分支上完成，不动 master。
- **commit message 用中文**：与现有历史风格一致（见 e870df4~f555230）。
- **Conventional Commits**：遵循 `<type>(<scope>): <description>` 格式。
- **依赖顺序**：mcu_compat → board_select → drv_flash → motor_info_storage fix → drv8301/as5047 驱动 → ODrive EN_GATE → power_monitor → drv_usart 修复 → motor_profile → calib → motor_info → control_irq style → docs。
- **CRLF 警告**：Windows 上 LF→CRLF 警告忽略，不影响内容。
- **`git add -p` 交互**：执行时按下方"分块选择指引"操作。

## `git add -p` 分块选择指引

3 个需要拆分的文件及其 hunk 选择：

### `User/Driver/drv_usart.c`（2 个 hunk）
- **Commit 1（mcu_compat）选**：`-#include "stm32g4xx_hal.h"` `+#include "main.h"` 这一行所在的 hunk
- **Commit 9（ORE 修复）选**：文件末尾新增的 `void HAL_UART_ErrorCallback(...)` 整个函数 hunk

### `User/Devices/dev_config.c`（2 个 hunk 相邻）
- diff 中两行 `+#include "dev_as5047.h"` 和 `+#include "dev_drv8301.h"` 通常在一个 hunk 内。如果 git 把它们合并为一个 hunk，使用 `git add -p` 的 `s`（split）选项拆分（要求两行间有空行作为分隔点）；若无法 split，则两行一起进 Commit 5（drv8301），Commit 6 不再包含 dev_config.c。

### `User/Devices/dev_motor.c`（2 个不相关 hunk）
- **Commit 6（AS5047）选**：
  - `#elif (DEV_MOTOR_ENCODER_TYPE == DEV_MOTOR_ENCODER_AS5047)` `#include "dev_encoder_as5047.h"` hunk
  - `dev_encoder_as5047_create(&pobj->encoder, (as5047_id_e)id);` hunk
- **Commit 7（ODrive EN_GATE）选**：
  - `#if defined(JM_BOARD_ODRIVE)` `static dev_motor_enable_config_t motor_enable_list...` hunk

## 验证步骤

每个 commit 完成后建议执行：
```bash
git log --oneline -1
git show --stat HEAD
```

全部完成后：
```bash
git log --oneline develop_mks -15   # 查看分支历史
git status                            # 应只剩 Board/ODriveMKS 和 Board/SFOC 的 submodule modified
git diff --stat                       # 确认仅剩 submodule
```

最终验证要点：
1. `git log develop_mks ^master` 应显示 14 个新 commit
2. `git status` 仅显示 `modified: Board/ODriveMKS` 和 `modified: Board/SFOC`（submodule，按约定本次不处理）
3. 每个提交都能独立 `git checkout <hash>` 编译通过（理想情况下；若不允许中途编译，至少保证逻辑自洽）

## 风险与回滚

- **风险**：`git add -p` 选错 hunk 会导致 commit 包含错误内容。
- **回滚**：单个 commit 出错可用 `git reset HEAD~1`（保留改动到工作区）后重新 add；若已 push 则用 `git revert`。
- **整体回滚**：`git checkout master && git branch -D develop_mks` 即可丢弃整条分支。
