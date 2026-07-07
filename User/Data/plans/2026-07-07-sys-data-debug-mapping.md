# sys_data_t 调试映射指针 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 `sys_data_t` 中增加指向 `dwt_timer` / `s_motor_loop` / `g_motor_info_storage` / `dev_power_monitor` / `dev_commun_uart` 五个已存在全局变量的**指针字段**，通过 `usr` 一个变量即可在调试器中观察全部数据，零数据复制、实时反映。

**Architecture:** 采用「指针映射 + 前向声明」方案。`sys_data_t` 内新增 5 个指针字段，指向已存在的全局变量（不持有数据、不复制数据）。为避免 `runtime_param.h`（DataHub 层）反向依赖 MotorControl/Devices/ParamService 层头文件：
- 前 3 个（`dwtTimer_t` / `motor_loop_t` / `motor_info_storage_t`）原为**匿名 struct**，需改为具名 struct（`typedef struct xxx_s {...} xxx_t;`）才能前向声明；
- 后 2 个（`dev_power_monitor_t` / `dev_commun_uart_t`）**已是具名 struct**（`typedef struct dev_power_monitor {...}` / `typedef struct dev_commun_uart {...}`），可直接前向声明，**无需改动原头文件**。

在 `runtime_param.h` 中用 `struct xxx *` 前向声明全部 5 个类型，仅在 `runtime_param.c` 中 include 真正的头文件并绑定地址。

**Tech Stack:** 嵌入式 C（Keil MDK / ARM Compiler），STM32G474，无主机端单元测试框架（验证方式：编译通过 + 调试器 Watch 窗口观察）。

---

## 方案对比（为什么选指针映射）

| 方案 | 数据复制 | 实时性 | 内存开销 | 分层影响 | 结论 |
|------|----------|--------|----------|----------|------|
| 手动逐字段复制到 sys_data_t | 是 | 需周期同步 | 翻倍 | 无 | 否决（用户明确不想复制）|
| 直接 include 头文件嵌入结构体 | 是（值拷贝）| 需同步 | 翻倍 | runtime_param.h 变重 | 否决 |
| 宏别名 `#define usr_xxx (xxx)` | 否 | 实时 | 0 | 无 | 不算"在 sys_data_t 内"，调试器仍需分别加变量 |
| **指针映射 + 前向声明** | **否** | **实时** | **5 个指针=20B** | **runtime_param.h 不增依赖** | **采纳** |

---

## 文件结构

| 文件 | 职责 | 改动 |
|------|------|------|
| `User/Devices/dev_dwt_counter.h` | DWT 计时器设备类型 + 接口 | 给 `dwtTimer_t` 的 struct 具名；补 `extern dwtTimer_t dwt_timer;` |
| `User/MotorControl/CascadeControl/motor_loop.h` | 三环控制上下文类型 + 接口 | 给 `motor_loop_t` 的 struct 具名 |
| `User/AppServices/ParamService/motor_info_storage.h` | motor_info Flash 存储设备类型 | 给 `motor_info_storage_t` 的 struct 具名 |
| `User/Devices/dev_power_monitor.h` | 电源监控设备类型 | **不改**（typedef 已具名 `struct dev_power_monitor`）|
| `User/Devices/dev_commun_uart.h` | UART 通信设备类型 | **不改**（typedef 已具名 `struct dev_commun_uart`）|
| `User/DataHub/runtime_param.h` | sys_data_t 定义 | 前向声明 5 个 struct；`sys_data_t` 末尾加 5 个指针字段 |
| `User/AppEntry/user_interface.c` | `hardware_init` 初始化流程 | include `dev_commun_uart.h`；在 `hardware_init` 末尾绑定 5 个指针 |

**设计要点：**
- 指针为**非 const**：保留调试时通过 `usr.p_xxx->field = value` 强制设值的能力（嵌入式调试常见需求）。
- 绑定在 `hardware_init()` 末尾完成（`user_interface.c`）：此时五个对象均已 init 完毕，地址与内容都就绪。原 `user_data_init()` 是死代码（从未被调用），不能放那里。
- 用 `motor_loop_get()` getter 绑定 `s_motor_loop`，避免直接访问带 `s_` 前缀的"伪私有"变量。
- 前向声明让 `runtime_param.h` 不引入任何新 include，保持 DataHub 层不反向依赖上层。
- `dev_power_monitor` / `dev_commun_uart` 的 typedef 原本就是具名 struct（`typedef struct dev_power_monitor {...}` / `typedef struct dev_commun_uart {...}`），前向声明 `struct dev_power_monitor` / `struct dev_commun_uart` 即可，零改动原头文件。

---

### Task 1: 给三个 typedef 具名 struct 并补 dwt_timer extern 声明

**Files:**
- Modify: `User/Devices/dev_dwt_counter.h:11-18`（struct 具名 + 补 extern）
- Modify: `User/MotorControl/CascadeControl/motor_loop.h:37-48`（struct 具名）
- Modify: `User/AppServices/ParamService/motor_info_storage.h:118-133`（struct 具名）

**说明：** C 语言中 `typedef struct { ... } xxx_t;`（匿名 struct）无法前向声明。改为 `typedef struct xxx_s { ... } xxx_t;`（具名 struct）后，可在其他头文件中用 `struct xxx_s *` 前向声明。这是最小改动，完全向后兼容（`xxx_t` 仍可用）。

- [ ] **Step 1: 改 `dev_dwt_counter.h` — struct 具名 + 补 extern 声明**

将 [dev_dwt_counter.h:11-18](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/Devices/dev_dwt_counter.h#L11-L18) 的：

```c
	typedef struct
	{
		uint32_t now_records[SYS_TIMER_RECORD_MAX_INDEX];	   /* 记录各索引对应的起始时刻(DWT_CYCCNT计数值) */
		uint32_t duration_records[SYS_TIMER_RECORD_MAX_INDEX]; /* 记录各索引对应的持续时长(时钟周期数) */
		float duration_us[SYS_TIMER_RECORD_MAX_INDEX];		   /* 记录各索引对应的持续时长(微秒) */
		uint32_t sys_freq_hz;								   /* 系统时钟频率，单位Hz */
		float ticks_to_us;									   /* 时钟周期数→微秒的换算系数(1e6/freq)，用于快速换算 */
	} dwtTimer_t;
```

改为：

```c
	typedef struct dwtTimer_s
	{
		uint32_t now_records[SYS_TIMER_RECORD_MAX_INDEX];	   /* 记录各索引对应的起始时刻(DWT_CYCCNT计数值) */
		uint32_t duration_records[SYS_TIMER_RECORD_MAX_INDEX]; /* 记录各索引对应的持续时长(时钟周期数) */
		float duration_us[SYS_TIMER_RECORD_MAX_INDEX];		   /* 记录各索引对应的持续时长(微秒) */
		uint32_t sys_freq_hz;								   /* 系统时钟频率，单位Hz */
		float ticks_to_us;									   /* 时钟周期数→微秒的换算系数(1e6/freq)，用于快速换算 */
	} dwtTimer_t;

	/* 全局实例（定义在 dev_dwt_counter.c），供调试映射指针绑定 */
	extern dwtTimer_t dwt_timer;
```

- [ ] **Step 2: 改 `motor_loop.h` — struct 具名**

将 [motor_loop.h:37-48](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorControl/CascadeControl/motor_loop.h#L37-L48) 的：

```c
typedef struct
{
	dev_motor_t motor;		// 底层电机设备（FOC/编码器/半桥/ADC）
	system_state_t sys;		// 上层状态机（模式管理 + 参考生成）
	cascade_ctrl_t cascade; // 三环级联外环（位置 + 速度）
	cur_loop_t current;		// 电流环
	cascade_out_t out;		// 级联输出（dq电流参考）

	uint32_t vel_cnt; // 速度环分频计数器（自增比较，避免取模）
	uint32_t pos_cnt; // 位置环分频计数器（自增比较，避免取模）
	bool sync_pending; // 遥测同步挂起：位置拍置位，下一拍执行以错开位置环负载
} motor_loop_t;
```

改为：

```c
typedef struct motor_loop_s
{
	dev_motor_t motor;		// 底层电机设备（FOC/编码器/半桥/ADC）
	system_state_t sys;		// 上层状态机（模式管理 + 参考生成）
	cascade_ctrl_t cascade; // 三环级联外环（位置 + 速度）
	cur_loop_t current;		// 电流环
	cascade_out_t out;		// 级联输出（dq电流参考）

	uint32_t vel_cnt; // 速度环分频计数器（自增比较，避免取模）
	uint32_t pos_cnt; // 位置环分频计数器（自增比较，避免取模）
	bool sync_pending; // 遥测同步挂起：位置拍置位，下一拍执行以错开位置环负载
} motor_loop_t;
```

- [ ] **Step 3: 改 `motor_info_storage.h` — struct 具名**

将 [motor_info_storage.h:118-133](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/AppServices/ParamService/motor_info_storage.h#L118-L133) 的：

```c
	typedef struct
	{
		/* 组合子设备：通用 Flash 设备（提供页擦写+磨损均衡） */
		dev_flash_t flash_dev;

		/* 全局唯一 motor_info 实例（外部经 get 方法取句柄） */
		motor_info_t motor_info;

		/* 状态标志 */
		bool inited;            /* Flash 存储服务初始化完成 */
		bool motor_info_loaded; /* motor_info 已加载默认+Flash+profile */

		/* 方法表指针（指向共享 static const ops，构造时装配） */
		const motor_info_storage_ops_t *ops;
	} motor_info_storage_t;
```

改为：

```c
	typedef struct motor_info_storage
	{
		/* 组合子设备：通用 Flash 设备（提供页擦写+磨损均衡） */
		dev_flash_t flash_dev;

		/* 全局唯一 motor_info 实例（外部经 get 方法取句柄） */
		motor_info_t motor_info;

		/* 状态标志 */
		bool inited;            /* Flash 存储服务初始化完成 */
		bool motor_info_loaded; /* motor_info 已加载默认+Flash+profile */

		/* 方法表指针（指向共享 static const ops，构造时装配） */
		const motor_info_storage_ops_t *ops;
	} motor_info_storage_t;
```

- [ ] **Step 4: Keil 编译验证**

Run: Keil MDK → Project → Build Target（或 `F7`）
Expected: 0 errors。若出现 `error: #70: incomplete type is not allowed`，说明某处仍用匿名 struct 引用——检查 Step 1-3 是否完整。

- [ ] **Step 5: Commit**

```bash
git add User/Devices/dev_dwt_counter.h User/MotorControl/CascadeControl/motor_loop.h User/AppServices/ParamService/motor_info_storage.h
git commit -m "refactor: 给 dwtTimer_t/motor_loop_t/motor_info_storage_t 的 struct 具名以支持前向声明"
```

---

### Task 2: 在 sys_data_t 中增加 5 个调试映射指针字段

**Files:**
- Modify: `User/DataHub/runtime_param.h:219-225`

**说明：** 用前向声明 5 个 struct 标签，`runtime_param.h` 不引入任何新 include，保持 DataHub 层不反向依赖 Devices/MotorControl/ParamService 层。调试器（Keil/JLink）靠 DWARF 调试信息展开字段，不依赖头文件可见性。其中 `dev_power_monitor` / `dev_commun_uart` 的 typedef 原本就是具名 struct，前向声明直接可用。

- [ ] **Step 1: 在 `runtime_param.h` 前向声明 5 个 struct**

在 [runtime_param.h:7](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/DataHub/runtime_param.h#L7) 的 `#include "state_define.h"` 之后，插入前向声明：

```c
#include <stdbool.h>
#include <stdint.h>
#include "motor_param.h"
#include "state_define.h"

/* ===== 调试映射指针的前向声明 =====
 * 仅声明 struct 标签，不引入对应头文件，避免 DataHub 层反向依赖
 * Devices/MotorControl/ParamService 层。完整类型在 runtime_param.c 中
 * include 头文件后可见；调试器靠 DWARF 信息展开字段，不受前向声明影响。
 * 前 3 个 struct 标签由 Task 1 具名化产生；后 2 个原本就是具名 struct。
 */
struct dwtTimer_s;            /* dev_dwt_counter.h:    dwtTimer_t              */
struct motor_loop_s;          /* motor_loop.h:         motor_loop_t            */
struct motor_info_storage;    /* motor_info_storage.h: motor_info_storage_t    */
struct dev_power_monitor;     /* dev_power_monitor.h:  dev_power_monitor_t（原具名）*/
struct dev_commun_uart;       /* dev_commun_uart.h:    dev_commun_uart_t（原具名）*/
```

- [ ] **Step 2: 在 `sys_data_t` 末尾增加 5 个指针字段**

将 [runtime_param.h:220-225](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/DataHub/runtime_param.h#L220-L225) 的：

```c
typedef struct sys_data_
{
	system_t sys;
	motor_state_t motor_state[MOTOR_MAX];
	motor_param_t motor_param[MOTOR_MAX];
} sys_data_t;
```

改为：

```c
typedef struct sys_data_
{
	system_t sys;
	motor_state_t motor_state[MOTOR_MAX];
	motor_param_t motor_param[MOTOR_MAX];

	/* ===== 调试映射指针 =====
	 * 指向已存在的全局变量（dwt_timer / s_motor_loop / g_motor_info_storage /
	 * dev_power_monitor / dev_commun_uart），不持有数据、不复制数据，
	 * 仅方便调试时通过 usr 一个变量统一观察。
	 * 绑定在 user_data_init() 中完成；非 const 以便调试时强制设值。
	 * 访问示例: usr.p_dwt_timer->duration_us[0], usr.p_motor_loop->vel_cnt,
	 *           usr.p_dev_power_monitor->vbus, usr.p_dev_commun_uart->tx_count
	 */
	struct dwtTimer_s           *p_dwt_timer;          /* -> dwt_timer            */
	struct motor_loop_s         *p_motor_loop;         /* -> s_motor_loop         */
	struct motor_info_storage   *p_motor_info_storage; /* -> g_motor_info_storage */
	struct dev_power_monitor    *p_dev_power_monitor;  /* -> dev_power_monitor    */
	struct dev_commun_uart      *p_dev_commun_uart;    /* -> dev_commun_uart      */
} sys_data_t;
```

- [ ] **Step 3: Keil 编译验证**

Run: Keil MDK → Build Target（`F7`）
Expected: 0 errors。此时 `usr` 实例已含 5 个新指针字段，但尚未绑定（值为 NULL）。

- [ ] **Step 4: Commit**

```bash
git add User/DataHub/runtime_param.h
git commit -m "feat: sys_data_t 增加 dwt_timer/motor_loop/motor_info_storage/dev_power_monitor/dev_commun_uart 调试映射指针字段"
```

---

### Task 3: 在 user_data_init 中绑定 5 个指针

**Files:**
- Modify: `User/DataHub/runtime_param.c:1-31`

**说明：** 在 `runtime_param.c`（实现文件，非头文件）中 include 五个头文件以访问全局变量地址。绑定用地址，与对象是否已初始化无关——指针始终有效，调试时实时反映对象最新值。`s_motor_loop` 通过 `motor_loop_get()` getter 绑定，避免直接访问带 `s_` 前缀的伪私有变量。其余 4 个都是非 static 全局单例，直接取地址。

- [ ] **Step 1: 在 `runtime_param.c` 顶部 include 五个头文件**

将 [runtime_param.c:1-3](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/DataHub/runtime_param.c#L1-L3) 的：

```c
#include "runtime_param.h"
#include "version.h"
#include <string.h>
```

改为：

```c
#include "runtime_param.h"
#include "version.h"
#include <string.h>

/* 调试映射指针绑定所需头文件（仅在 .c 中 include，不蔓延到 .h） */
#include "dev_dwt_counter.h"      /* dwt_timer              */
#include "motor_loop.h"           /* motor_loop_get()       */
#include "motor_info_storage.h"   /* g_motor_info_storage   */
#include "dev_power_monitor.h"    /* dev_power_monitor      */
#include "dev_commun_uart.h"      /* dev_commun_uart        */
```

- [ ] **Step 2: 在 `user_data_init` 末尾绑定 5 个指针**

将 [runtime_param.c:11-31](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/DataHub/runtime_param.c#L11-L31) 的 `user_data_init` 函数：

```c
void user_data_init(void)
{
	memset(&usr, 0, sizeof(sys_data_t));

	// 版本信息
	usr.sys.version.boot.byte.version_major = BOOT_VERSION_MAJOR;
	usr.sys.version.boot.byte.version_minor = BOOT_VERSION_MINOR;
	usr.sys.version.boot.byte.version_patch = BOOT_VERSION_PATCH;
	usr.sys.version.boot.byte.version_build = BOOT_VERSION_BUILD;

	usr.sys.version.app.byte.version_major = APP_VERSION_MAJOR;
	usr.sys.version.app.byte.version_minor = APP_VERSION_MINOR;
	usr.sys.version.app.byte.version_patch = APP_VERSION_PATCH;
	usr.sys.version.app.byte.version_build = APP_VERSION_BUILD;

	usr.sys.version.hardware.byte.version_major = HW_VERSION_MAJOR;
	usr.sys.version.hardware.byte.version_minor = HW_VERSION_MINOR;
	usr.sys.version.hardware.byte.version_patch = HW_VERSION_PATCH;
	usr.sys.version.hardware.byte.version_build = HW_VERSION_BUILD;
}
```

改为（在函数末尾、`}` 之前追加 5 行绑定）：

```c
void user_data_init(void)
{
	memset(&usr, 0, sizeof(sys_data_t));

	// 版本信息
	usr.sys.version.boot.byte.version_major = BOOT_VERSION_MAJOR;
	usr.sys.version.boot.byte.version_minor = BOOT_VERSION_MINOR;
	usr.sys.version.boot.byte.version_patch = BOOT_VERSION_PATCH;
	usr.sys.version.boot.byte.version_build = BOOT_VERSION_BUILD;

	usr.sys.version.app.byte.version_major = APP_VERSION_MAJOR;
	usr.sys.version.app.byte.version_minor = APP_VERSION_MINOR;
	usr.sys.version.app.byte.version_patch = APP_VERSION_PATCH;
	usr.sys.version.app.byte.version_build = APP_VERSION_BUILD;

	usr.sys.version.hardware.byte.version_major = HW_VERSION_MAJOR;
	usr.sys.version.hardware.byte.version_minor = HW_VERSION_MINOR;
	usr.sys.version.hardware.byte.version_patch = HW_VERSION_PATCH;
	usr.sys.version.hardware.byte.version_build = HW_VERSION_BUILD;

	/* 调试映射指针绑定：指向已存在的全局变量地址（不复制数据）
	 * 此时对象内容可能尚未初始化（motor_loop_init/storage_init 等在之后调用），
	 * 但地址不变，调试时通过 usr.p_xxx 实时反映对象最新值。
	 * s_motor_loop 经 getter 绑定，避免直接访问伪私有变量。 */
	usr.p_dwt_timer          = &dwt_timer;
	usr.p_motor_loop         = motor_loop_get();
	usr.p_motor_info_storage = &g_motor_info_storage;
	usr.p_dev_power_monitor  = &dev_power_monitor;
	usr.p_dev_commun_uart    = &dev_commun_uart;
}
```

- [ ] **Step 3: Keil 编译验证**

Run: Keil MDK → Build Target（`F7`）
Expected: 0 errors, 0 warnings（新增）。若出现 `warning: #223-D: function "motor_loop_get" declared implicitly`，说明 `motor_loop.h` 未正确 include——检查 Step 1。

- [ ] **Step 4: Commit**

```bash
git add User/DataHub/runtime_param.c
git commit -m "feat: user_data_init 绑定 dwt_timer/motor_loop/motor_info_storage/dev_power_monitor/dev_commun_uart 调试映射指针"
```

---

### Task 4: 硬件调试器验证

**Files:** 无代码改动，仅调试器验证。

**说明：** 嵌入式项目无主机端单测，用「编译通过 + 调试器 Watch 窗口观察」作为验收标准。验证三个指针字段能正确指向原变量、且值实时同步。

- [ ] **Step 1: 下载固件并启动调试**

Run: Keil MDK → Debug → Start/Stop Debug Session（`Ctrl+F5`）→ Run（`F5`）
Expected: 程序正常运行，无 HardFault。

- [ ] **Step 2: Watch 窗口验证 usr 指针指向正确**

在 Keil Watch 窗口添加以下表达式，观察值：

| Watch 表达式 | 预期值 | 说明 |
|--------------|--------|------|
| `usr.p_dwt_timer` | 非 0 地址 | 等于 `&dwt_timer` |
| `usr.p_motor_loop` | 非 0 地址 | 等于 `&s_motor_loop` |
| `usr.p_motor_info_storage` | 非 0 地址 | 等于 `&g_motor_info_storage` |
| `usr.p_dev_power_monitor` | 非 0 地址 | 等于 `&dev_power_monitor` |
| `usr.p_dev_commun_uart` | 非 0 地址 | 等于 `&dev_commun_uart` |
| `usr.p_dwt_timer->sys_freq_hz` | 170000000（STM32G474 主频）| 与 `dwt_timer.sys_freq_hz` 一致 |
| `usr.p_dwt_timer->duration_us[0]` | 电流环周期耗时（μs）| 与 `dwt_timer.duration_us[0]` 实时同步 |
| `usr.p_motor_loop->vel_cnt` | 递增的计数值 | 与 `s_motor_loop.vel_cnt` 实时同步 |
| `usr.p_motor_loop->motor.motor_param.ele_radian` | 实时电角度 | 与 `s_motor_loop.motor.motor_param.ele_radian` 一致 |
| `usr.p_motor_info_storage->inited` | 1（init 后）| 与 `g_motor_info_storage.inited` 一致 |
| `usr.p_motor_info_storage->motor_info.blocks.motor_calib.pole_pairs` | 7（默认）或标定值 | 与 `g_motor_info_storage.motor_info...` 一致 |
| `usr.p_dev_power_monitor->vbus` | ~24V（额定）或实测值 | 与 `dev_power_monitor.vbus` 一致 |
| `usr.p_dev_power_monitor->ibus` | 实时母线电流 | 与 `dev_power_monitor.ibus` 实时同步 |
| `usr.p_dev_commun_uart->tx_count` | 递增的发送计数 | 与 `dev_commun_uart.tx_count` 一致 |
| `usr.p_dev_commun_uart->tx_fail_count` | 0（正常）或失败数 | 与 `dev_commun_uart.tx_fail_count` 一致 |

Expected: 所有字段非 0、可展开、值与原变量一致。

- [ ] **Step 3: 实时性验证 — 确认指针值随原变量变化**

操作：让电机运行（或手动转动电机轴），在 WATCH 窗口观察：
- `usr.p_dwt_timer->duration_us[1]`（电流环耗时）应在每次中断后更新
- `usr.p_motor_loop->motor.motor_param.ele_radian` 应随电机转动变化
- `usr.p_motor_loop->motor.motion.velocity` 应反映实时速度
- `usr.p_dev_power_monitor->vbus` / `ibus` 应随负载变化
- `usr.p_dev_commun_uart->tx_count` 应随上位机轮询递增

Expected: 指针指向的字段实时更新，与直接观察原变量无差异——证明零拷贝、实时映射成功。

- [ ] **Step 4: Commit（验证记录）**

无需代码提交。在计划文件对应 checkbox 打勾，记录验收完成。

---

## Self-Review 自检

**1. Spec 覆盖：**
- 用户需求：「在 sys_data_t 中增加 dwt_timer / s_motor_loop / g_motor_info_storage 的映射，不手动复制」→ Task 2 增加指针字段，Task 3 绑定地址，零复制。✓
- 用户追加需求：「把 dev_power_monitor / dev_commun_uart 也加入」→ Task 2/3/4 同步覆盖，二者 typedef 已具名无需 Task 1 改动。✓
- 用户需求：「通过这个全局变量也能访问到这个数据结构，方便调试」→ Task 4 验证 `usr.p_xxx->field` 可展开观察。✓
- 用户需求：「不想重新完全定义一个数据手动一个一个复制」→ 指针映射方案，无任何字段复制。✓

**2. 占位符扫描：**
- 无 "TBD"、"TODO"、"add error handling" 等。每个 Step 都有完整代码。✓

**3. 类型一致性：**
- `struct dwtTimer_s` ↔ `dev_dwt_counter.h` 的 `typedef struct dwtTimer_s {...} dwtTimer_t;`（Task 1 Step 1）✓
- `struct motor_loop_s` ↔ `motor_loop.h` 的 `typedef struct motor_loop_s {...} motor_loop_t;`（Task 1 Step 2）✓
- `struct motor_info_storage` ↔ `motor_info_storage.h` 的 `typedef struct motor_info_storage {...} motor_info_storage_t;`（Task 1 Step 3，标签名与原代码 ops 方法表的 `struct motor_info_storage *` 一致）✓
- `struct dev_power_monitor` ↔ `dev_power_monitor.h:85` 的 `typedef struct dev_power_monitor {...} dev_power_monitor_t;`（原具名，Task 2 Step 1 前向声明）✓
- `struct dev_commun_uart` ↔ `dev_commun_uart.h` 的 `typedef struct dev_commun_uart {...} dev_commun_uart_t;`（原具名，Task 2 Step 1 前向声明）✓
- `usr.p_dwt_timer` / `usr.p_motor_loop` / `usr.p_motor_info_storage` / `usr.p_dev_power_monitor` / `usr.p_dev_commun_uart` 在 Task 2 定义、Task 3 绑定、Task 4 验证——名称一致。✓
- `motor_loop_get()` 在 `motor_loop.h:75` 已声明，Task 3 Step 2 调用合法。✓
- `dwt_timer` extern 声明在 Task 1 Step 1 补齐，Task 3 Step 2 可访问。✓
- `g_motor_info_storage` extern 声明在 `motor_info_storage.h:136` 已存在，Task 3 Step 2 可访问。✓
- `dev_power_monitor` extern 声明在 `dev_power_monitor.h:117` 已存在，Task 3 Step 2 可访问。✓
- `dev_commun_uart` extern 声明在 `dev_commun_uart.h:119` 已存在，Task 3 Step 2 可访问。✓

**4. 风险点复核：**
- **循环依赖**：`runtime_param.h` 仅前向声明 5 个 struct 标签，不 include 任何头文件；5 个头文件也不 include `runtime_param.h`。`runtime_param.c` include 5 个头文件，但 .c 间依赖不蔓延。✓
- **调用时机**：`user_data_init()` 在 `main` 早期调用，此时五个全局变量对象可能未 init，但**地址已确定**（全局变量），指针绑定有效；对象内容在各自 init 后填充，调试时实时反映。✓
- **const 正确性**：采用非 const 指针，保留调试时强制设值能力，符合嵌入式调试习惯。✓
- **向后兼容**：给 struct 具名（`typedef struct xxx_s {...} xxx_t;`）完全兼容原有 `xxx_t` 用法，无破坏性；`dev_power_monitor_t` / `dev_commun_uart_t` 无改动。✓
- **条件编译**：`dev_commun_uart.h` / `dev_power_monitor.h` 整体分别被 `#if defined(USE_DEV_COMMUN_UART)` / `#if defined(USE_DEV_POWER_MONITOR)` 包裹。已确认两个宏在 `Board/SFOC/Config/dev_config_board.h:8,12` 和 `Board/V1/Config/dev_config_board.h:8,11` 均已定义（经 `board_select.h` → `dev_config.h` 链 include），两个板级工程均可见，无编译风险。✓

---

## 执行交接

**计划已完成并保存到 `User/Data/plans/2026-07-07-sys-data-debug-mapping.md`。两种执行方式：**

**1. Subagent-Driven（推荐）** - 每个 Task 派发独立 subagent，任务间 review，迭代快

**2. Inline Execution** - 在当前会话内批量执行，带 checkpoint review

**选哪种？**
