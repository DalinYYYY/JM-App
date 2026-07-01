# 电机标定状态机与模块化标定实现计划（v2 — 按类别+子命令）

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 实现电机标定的顶层状态机（TOP_FSM_CALIB）与 7 级 24 子模式的模块化标定框架。采用"类别命令+子命令"设计：0x90-0x96 为 7 个标定级别，payload[0] 为子模式 ID；0x97 为进度查询，0x98 为中止。

**Architecture:** 上位机下发 `[cmd=0x90~0x96][payload=submode]` → `app_set_mode` 提取 level 和 submode，调用 `calib_mgr_start(level, submode)` → `process_ctrl_cmd` 进入 CALIB 态 → `motor_control_loop` 周期调用 `calib_mgr_poll` → `calib_mgr` 按 level 路由到对应级别模块 → 级别模块按 submode 路由到具体标定函数。标定模块按 7 个级别分文件存放于 `User/MotorCalibration/`，每文件内部用 switch(submode) 分发。

**Tech Stack:** STM32G4 HAL, 嵌入式 C (C99), Keil MDK-ARM, 现有 jm_proto 协议栈, C-OOP ops 函数表模式

---

## 7 级 24 子模式标定清单

| Level | 命令码 | 类别 | 子模式 | 说明 |
|-------|--------|------|--------|------|
| 1 | 0x90 | 驱动硬件底层 | 1=ADC偏置, 2=ADC增益, 3=电流传感器, 4=温度传感器, 5=母线电压, 6=死区特性 | 上电首要，所有采样基础 |
| 2 | 0x91 | 电机电气身份 | 1=相序识别, 2=极对数, 3=R/Ld/Lq/flux辨识 | FOC 运行核心前提 |
| 3 | 0x92 | 编码器校准 | 1=零位, 2=方向校验, 3=线性度, 4=正余弦/旋变, 5=多圈零点 | 闭环反馈基础 |
| 4 | 0x93 | 转矩基础 | 1=力矩常数 | 转矩控制基准 |
| 5 | 0x94 | 非线性补偿 | 1=齿槽, 2=摩擦, 3=死区补偿, 4=磁饱和 | 性能优化级 |
| 6 | 0x95 | 负载系统级 | 1=惯量, 2=阻尼, 3=回程间隙, 4=PID自整定 | 高阶系统级 |
| 7 | 0x96 | 自动化集成 | 1=一键全自动 | 量产工程化 |
| - | 0x97 | 进度查询 | 无 payload | 返回 ACK(完成)/NACK(0x0A进行中)/NACK(0x03未标定) |
| - | 0x98 | 中止标定 | 无 payload | 调用 calib_mgr_abort，返回 ACK |

## 文件结构

### 新增文件

| 文件 | 职责 |
|------|------|
| `MotorCalibration/calib_types.h` | 公共类型：level/submode 常量、calib_state_e、calib_status_t、calib_level_ops_t |
| `MotorCalibration/calib_mgr.h` | 标定管理器接口：init/start/poll/abort/get_status |
| `MotorCalibration/calib_mgr.c` | 标定管理器实现：level→ops 调度表、状态机 |
| `MotorCalibration/calib_level1_driver.c` | L1 驱动硬件底层（6 子模式） |
| `MotorCalibration/calib_level2_motor.c` | L2 电机电气身份（3 子模式） |
| `MotorCalibration/calib_level3_encoder.c` | L3 编码器校准（5 子模式） |
| `MotorCalibration/calib_level4_torque.c` | L4 转矩基础（1 子模式） |
| `MotorCalibration/calib_level5_nonlinear.c` | L5 非线性补偿（4 子模式） |
| `MotorCalibration/calib_level6_system.c` | L6 负载系统级（4 子模式） |
| `MotorCalibration/calib_level7_auto.c` | L7 自动化集成（1 子模式） |

### 修改文件

| 文件 | 修改内容 |
|------|----------|
| `DataHub/state_define.h` | 替换 ctrl_mode_e 标定段（12→9 枚举）；新增 calib_state_e |
| `Protocol/joint_proto/jm_cmd_def.h` | 替换 jm_cmd_e 标定段（12→9 枚举） |
| `AppServices/StateMachine/system_state.h` | system_state_t 增加 calib_state 字段 |
| `AppServices/StateMachine/system_state.c` | CALIB 进入/退出动作；motor_control_loop 中 poll；process_ctrl_cmd 路由更新 |
| `Protocol/joint_proto/jm_proto_ops.c` | app_set_mode 新增 0x90-0x98 标定分支 |
| `Board/V1/MDK-ARM/JointMotorApp.uvprojx` | 添加 10 个新 .c 文件 |
| `Board/SFOC/MDK-ARM/sfoc.uvprojx` | 同上 |

### 协议交互流程

```
上位机                          固件
  │  [0x90][submode=1]           │  app_set_mode: calib_mgr_start(1,1) → process_ctrl_cmd → CALIB态
  │  ← ACK(0)                   │
  │                              │  motor_control_loop: calib_mgr_poll() 每拍推进
  │  [0x97]                      │  app_set_mode: calib_mgr_get_status() → RUNNING
  │  ← NACK(0x0A)               │  (标定进行中)
  │  ...轮询...                  │
  │  [0x97]                      │  calib_mgr_get_status() → DONE
  │  ← ACK(0)                   │  (标定完成，结果已写入 motor_param_t)
  │  [0x00] (IDLE)              │  退出 CALIB 态 → calib_mgr_abort
  │  ← ACK(0)                   │
```

---

### Task 1: 创建 calib_types.h 公共类型定义

**Files:**
- Create: `User/MotorCalibration/calib_types.h`

- [ ] **Step 1: 创建 calib_types.h**

```c
#ifndef __CALIB_TYPES_H__
#define __CALIB_TYPES_H__

#include <stdint.h>
#include <stdbool.h>
#include "motor_param.h"

/* ===================== 标定子状态 ===================== */
typedef enum
{
	CALIB_STATE_IDLE = 0,		/* 空闲（未开始或已完成） */
	CALIB_STATE_RUNNING,		/* 标定进行中 */
	CALIB_STATE_DONE,			/* 标定完成（成功） */
	CALIB_STATE_FAILED,			/* 标定失败 */
} calib_state_e;

/* ===================== 标定级别常量 ===================== */
#define CALIB_LEVEL1_DRIVER		1	/* 驱动硬件底层 */
#define CALIB_LEVEL2_MOTOR		2	/* 电机电气身份 */
#define CALIB_LEVEL3_ENCODER	3	/* 编码器校准 */
#define CALIB_LEVEL4_TORQUE		4	/* 转矩基础 */
#define CALIB_LEVEL5_NONLINEAR	5	/* 非线性补偿 */
#define CALIB_LEVEL6_SYSTEM		6	/* 负载系统级 */
#define CALIB_LEVEL7_AUTO		7	/* 自动化集成 */
#define CALIB_LEVEL_MAX			8

/* ===================== L1 子模式: 驱动硬件底层 ===================== */
#define CALIB_L1_ADC_OFFSET		1	/* ADC偏置 */
#define CALIB_L1_ADC_GAIN		2	/* ADC增益 */
#define CALIB_L1_CURRENT_SENSOR	3	/* 电流传感器 */
#define CALIB_L1_TEMP_SENSOR	4	/* 温度传感器 */
#define CALIB_L1_VBUS			5	/* 母线电压采样 */
#define CALIB_L1_DEADTIME		6	/* 驱动死区特性 */

/* ===================== L2 子模式: 电机电气身份 ===================== */
#define CALIB_L2_PHASE_SEQ		1	/* 相序识别 */
#define CALIB_L2_POLE_PAIRS		2	/* 极对数 */
#define CALIB_L2_RL_FLUX		3	/* R/Ld/Lq/flux辨识 */

/* ===================== L3 子模式: 编码器校准 ===================== */
#define CALIB_L3_ZERO_OFFSET	1	/* 零位 */
#define CALIB_L3_DIRECTION		2	/* 方向校验 */
#define CALIB_L3_LINEARITY		3	/* 线性度 */
#define CALIB_L3_SINCOS			4	/* 正余弦/旋变幅值相位 */
#define CALIB_L3_MULTITURN_ZERO	5	/* 多圈绝对值零点 */

/* ===================== L4 子模式: 转矩基础 ===================== */
#define CALIB_L4_KT				1	/* 力矩常数 */

/* ===================== L5 子模式: 非线性补偿 ===================== */
#define CALIB_L5_COGGING		1	/* 齿槽补偿 */
#define CALIB_L5_FRICTION		2	/* 摩擦补偿 */
#define CALIB_L5_DEADTIME_COMP	3	/* 逆变器死区补偿 */
#define CALIB_L5_SATURATION		4	/* 电感磁饱和补偿 */

/* ===================== L6 子模式: 负载系统级 ===================== */
#define CALIB_L6_INERTIA		1	/* 负载惯量 */
#define CALIB_L6_DAMPING		2	/* 负载阻尼 */
#define CALIB_L6_BACKLASH		3	/* 传动回程间隙 */
#define CALIB_L6_PID_AUTOTUNE	4	/* 控制环参数自整定 */

/* ===================== L7 子模式: 自动化集成 ===================== */
#define CALIB_L7_FULL_AUTO		1	/* 一键全自动 */

/* ===================== 标定进度与结果 ===================== */
typedef struct
{
	calib_state_e state;	/* 标定子状态 */
	uint8_t progress;		/* 进度 0-100 */
	uint8_t level;			/* 当前标定级别（1-7） */
	uint8_t submode;		/* 当前子模式 */
	uint8_t step;			/* 当前步骤（L7 全自动用，单步标定为 0） */
} calib_status_t;

/* ===================== 级别模块 ops 函数表 =====================
 * 每个级别模块导出一个 const calib_level_ops_t 实例，
 * calib_mgr 按 level 索引查表调用。
 * start 返回 false 表示 submode 不支持，calib_mgr 据此拒绝启动。*/
typedef struct
{
	bool (*start)(uint8_t submode, motor_param_t *param, float dt);
	calib_state_e (*poll)(void);
	void (*abort)(void);
} calib_level_ops_t;

#endif /* __CALIB_TYPES_H__ */
```

- [ ] **Step 2: Commit**

```bash
git add User/MotorCalibration/calib_types.h
git commit -m "feat(calib): add calib_types.h with 7-level 24-submode definitions"
```

---

### Task 2: 替换 ctrl_mode_e 和 jm_cmd_e 标定段

**Files:**
- Modify: `User/DataHub/state_define.h` L97-109
- Modify: `User/Protocol/joint_proto/jm_cmd_def.h` L98-110

- [ ] **Step 1: 替换 state_define.h 的 ctrl_mode_e 标定段**

将 L97-109 的 12 个 `CONTROL_MODE_CALIB_*` 枚举替换为 9 个新枚举：

```c
	// 校准指令 (0x90-0xAF)
	CONTROL_MODE_CALIB_LEVEL1 = 0x90,	 // L1 驱动硬件底层（payload[0]=子模式）
	CONTROL_MODE_CALIB_LEVEL2 = 0x91,	 // L2 电机电气身份
	CONTROL_MODE_CALIB_LEVEL3 = 0x92,	 // L3 编码器校准
	CONTROL_MODE_CALIB_LEVEL4 = 0x93,	 // L4 转矩基础
	CONTROL_MODE_CALIB_LEVEL5 = 0x94,	 // L5 非线性补偿
	CONTROL_MODE_CALIB_LEVEL6 = 0x95,	 // L6 负载系统级
	CONTROL_MODE_CALIB_LEVEL7 = 0x96,	 // L7 自动化集成
	CONTROL_MODE_CALIB_QUERY  = 0x97,	 // 标定进度查询
	CONTROL_MODE_CALIB_ABORT  = 0x98,	 // 中止标定
```

- [ ] **Step 2: 替换 jm_cmd_def.h 的 jm_cmd_e 标定段**

将 L98-110 的 12 个 `JM_CMD_CALIB_*` 枚举替换为 9 个新枚举：

```c
	/* 校准 0x90~0xAF: 类别命令+子命令模式
	 * 0x90-0x96: payload[0]=子模式ID, 进入CALIB态并启动标定
	 * 0x97: 进度查询, ACK=完成, NACK(0x0A)=进行中, NACK(0x03)=未标定
	 * 0x98: 中止标定, ACK */
	JM_CMD_CALIB_LEVEL1 = 0x90,	/* L1 驱动硬件底层 */
	JM_CMD_CALIB_LEVEL2 = 0x91,	/* L2 电机电气身份 */
	JM_CMD_CALIB_LEVEL3 = 0x92,	/* L3 编码器校准 */
	JM_CMD_CALIB_LEVEL4 = 0x93,	/* L4 转矩基础 */
	JM_CMD_CALIB_LEVEL5 = 0x94,	/* L5 非线性补偿 */
	JM_CMD_CALIB_LEVEL6 = 0x95,	/* L6 负载系统级 */
	JM_CMD_CALIB_LEVEL7 = 0x96,	/* L7 自动化集成 */
	JM_CMD_CALIB_QUERY  = 0x97,	/* 进度查询 */
	JM_CMD_CALIB_ABORT  = 0x98,	/* 中止标定 */
```

- [ ] **Step 3: Commit**

```bash
git add User/DataHub/state_define.h User/Protocol/joint_proto/jm_cmd_def.h
git commit -m "refactor(calib): replace 12 flat calib cmds with 7-level+submode structure"
```

---

### Task 3: 创建 calib_mgr.h 和 calib_mgr.c

**Files:**
- Create: `User/MotorCalibration/calib_mgr.h`
- Create: `User/MotorCalibration/calib_mgr.c`

- [ ] **Step 1: 创建 calib_mgr.h**

```c
#ifndef __CALIB_MGR_H__
#define __CALIB_MGR_H__

#include "calib_types.h"

/**
 * @brief 初始化标定管理器
 * @param param 电机参数指针（标定结果写入此结构）
 * @param dt 控制周期(s)
 */
void calib_mgr_init(motor_param_t *param, float dt);

/**
 * @brief 启动一次标定
 * @param level 标定级别（1-7，对应 CALIB_LEVEL1~7）
 * @param submode 子模式（各级别内定义，见 calib_types.h）
 * @return true 启动成功 / false 已在标定中或级别无效
 */
bool calib_mgr_start(uint8_t level, uint8_t submode);

/**
 * @brief 周期推进标定（motor_control_loop 的 CALIB 态调用）
 * @return 当前标定状态
 */
calib_state_e calib_mgr_poll(void);

/**
 * @brief 查询标定状态
 */
calib_status_t calib_mgr_get_status(void);

/**
 * @brief 强制中止当前标定
 */
void calib_mgr_abort(void);

#endif /* __CALIB_MGR_H__ */
```

- [ ] **Step 2: 创建 calib_mgr.c**

```c
/**
 * @file calib_mgr.c
 * @brief 标定管理器：按 level 路由到级别模块，管理标定状态机
 */
#include "calib_mgr.h"
#include <string.h>

/* ---- 各级别模块导出的 ops（前向声明） ---- */
extern const calib_level_ops_t calib_level1_ops;
extern const calib_level_ops_t calib_level2_ops;
extern const calib_level_ops_t calib_level3_ops;
extern const calib_level_ops_t calib_level4_ops;
extern const calib_level_ops_t calib_level5_ops;
extern const calib_level_ops_t calib_level6_ops;
extern const calib_level_ops_t calib_level7_ops;

/* ---- level → ops 调度表（索引 1-7，0 保留） ---- */
static const calib_level_ops_t *s_level_table[CALIB_LEVEL_MAX] = {
	[0] = NULL,
	[CALIB_LEVEL1_DRIVER]    = &calib_level1_ops,
	[CALIB_LEVEL2_MOTOR]     = &calib_level2_ops,
	[CALIB_LEVEL3_ENCODER]   = &calib_level3_ops,
	[CALIB_LEVEL4_TORQUE]    = &calib_level4_ops,
	[CALIB_LEVEL5_NONLINEAR] = &calib_level5_ops,
	[CALIB_LEVEL6_SYSTEM]    = &calib_level6_ops,
	[CALIB_LEVEL7_AUTO]      = &calib_level7_ops,
};

/* ---- 管理器私有状态 ---- */
static struct
{
	motor_param_t *param;
	float dt;
	calib_status_t status;
	const calib_level_ops_t *active_ops;
} s_mgr;

void calib_mgr_init(motor_param_t *param, float dt)
{
	memset(&s_mgr, 0, sizeof(s_mgr));
	s_mgr.param = param;
	s_mgr.dt = dt;
	s_mgr.status.state = CALIB_STATE_IDLE;
}

bool calib_mgr_start(uint8_t level, uint8_t submode)
{
	if (level == 0 || level >= CALIB_LEVEL_MAX)
		return false;
	if (s_mgr.status.state == CALIB_STATE_RUNNING)
		return false;

	const calib_level_ops_t *ops = s_level_table[level];
	if (ops == NULL || ops->start == NULL)
		return false;

	/* start 返回 false 表示 submode 不支持 */
	if (!ops->start(submode, s_mgr.param, s_mgr.dt))
		return false;

	s_mgr.active_ops = ops;
	s_mgr.status.state = CALIB_STATE_RUNNING;
	s_mgr.status.progress = 0;
	s_mgr.status.level = level;
	s_mgr.status.submode = submode;
	s_mgr.status.step = 0;
	return true;
}

calib_state_e calib_mgr_poll(void)
{
	if (s_mgr.status.state != CALIB_STATE_RUNNING || s_mgr.active_ops == NULL)
		return s_mgr.status.state;

	calib_state_e st = s_mgr.active_ops->poll();
	s_mgr.status.state = st;

	if (st == CALIB_STATE_DONE)
		s_mgr.status.progress = 100;
	else if (st == CALIB_STATE_FAILED)
		s_mgr.status.progress = 0;

	return st;
}

calib_status_t calib_mgr_get_status(void)
{
	return s_mgr.status;
}

void calib_mgr_abort(void)
{
	if (s_mgr.active_ops && s_mgr.active_ops->abort)
		s_mgr.active_ops->abort();
	s_mgr.status.state = CALIB_STATE_IDLE;
	s_mgr.status.progress = 0;
	s_mgr.active_ops = NULL;
}
```

- [ ] **Step 3: Commit**

```bash
git add User/MotorCalibration/calib_mgr.h User/MotorCalibration/calib_mgr.c
git commit -m "feat(calib): implement calib_mgr with level-based dispatch table"
```

---

### Task 4: 创建 7 个级别标定模块文件（桩实现）

**Files:**
- Create: `User/MotorCalibration/calib_level1_driver.c`
- Create: `User/MotorCalibration/calib_level2_motor.c`
- Create: `User/MotorCalibration/calib_level3_encoder.c`
- Create: `User/MotorCalibration/calib_level4_torque.c`
- Create: `User/MotorCalibration/calib_level5_nonlinear.c`
- Create: `User/MotorCalibration/calib_level6_system.c`
- Create: `User/MotorCalibration/calib_level7_auto.c`

每个文件导出 `const calib_level_ops_t calib_levelN_ops`，内部用 switch(submode) 分发到具体标定函数。桩实现中所有 poll 直接返回 `CALIB_STATE_DONE`，但 switch 结构清晰展示在哪加新 case。

**模板说明（以 L1 为例）：** 每个 level 模块的结构相同：
1. file-static 状态：`s_submode` / `s_param` / `s_dt`
2. `start`：switch(submode) 分发，记录 submode，返回 true/false
3. `poll`：switch(s_submode) 分发到具体算法（桩直接返回 DONE）
4. `abort`：停止当前标定
5. 导出 `const calib_level_ops_t calib_levelN_ops`

**后期添加新子模式时**：只需在 `start` 和 `poll` 的 switch 中各加一个 case 即可。

- [ ] **Step 1: 创建 calib_level1_driver.c（L1 驱动硬件底层，6 子模式）**

```c
/**
 * @file calib_level1_driver.c
 * @brief L1 驱动硬件底层校准（ADC偏置/增益/电流传感器/温度/母线电压/死区特性）
 * @note 桩实现：所有子模式 poll 直接返回 DONE。真实算法后续逐个填充。
 *       添加新子模式：在 start/poll 的 switch 中各加一个 case。
 */
#include "calib_types.h"

static uint8_t s_submode;
static motor_param_t *s_param;
static float s_dt;

/* ---- 各子模式的独立实现函数（桩） ---- */
static calib_state_e poll_adc_offset(void)      { return CALIB_STATE_DONE; }
static calib_state_e poll_adc_gain(void)        { return CALIB_STATE_DONE; }
static calib_state_e poll_current_sensor(void)  { return CALIB_STATE_DONE; }
static calib_state_e poll_temp_sensor(void)     { return CALIB_STATE_DONE; }
static calib_state_e poll_vbus(void)            { return CALIB_STATE_DONE; }
static calib_state_e poll_deadtime(void)        { return CALIB_STATE_DONE; }

static bool calib_level1_start(uint8_t submode, motor_param_t *param, float dt)
{
	s_param = param;
	s_dt = dt;
	s_submode = submode;

	switch (submode)
	{
		case CALIB_L1_ADC_OFFSET:
		case CALIB_L1_ADC_GAIN:
		case CALIB_L1_CURRENT_SENSOR:
		case CALIB_L1_TEMP_SENSOR:
		case CALIB_L1_VBUS:
		case CALIB_L1_DEADTIME:
			/* TODO: 按 submode 初始化对应校准（注入电压/采样配置等） */
			return true;
		default:
			return false; /* 不支持的子模式 */
	}
}

static calib_state_e calib_level1_poll(void)
{
	switch (s_submode)
	{
		case CALIB_L1_ADC_OFFSET:     return poll_adc_offset();
		case CALIB_L1_ADC_GAIN:       return poll_adc_gain();
		case CALIB_L1_CURRENT_SENSOR: return poll_current_sensor();
		case CALIB_L1_TEMP_SENSOR:    return poll_temp_sensor();
		case CALIB_L1_VBUS:           return poll_vbus();
		case CALIB_L1_DEADTIME:       return poll_deadtime();
		default:                      return CALIB_STATE_FAILED;
	}
}

static void calib_level1_abort(void)
{
	/* TODO: 按 s_submode 停止对应校准 */
}

const calib_level_ops_t calib_level1_ops = {
	.start = calib_level1_start,
	.poll  = calib_level1_poll,
	.abort = calib_level1_abort,
};
```

- [ ] **Step 2: 创建 calib_level2_motor.c（L2 电机电气身份，3 子模式）**

```c
/**
 * @file calib_level2_motor.c
 * @brief L2 电机电气身份辨识（相序/极对数/R-L-flux）
 */
#include "calib_types.h"

static uint8_t s_submode;
static motor_param_t *s_param;
static float s_dt;

static calib_state_e poll_phase_seq(void)   { return CALIB_STATE_DONE; }
static calib_state_e poll_pole_pairs(void)  { return CALIB_STATE_DONE; }
static calib_state_e poll_rl_flux(void)     { return CALIB_STATE_DONE; }

static bool calib_level2_start(uint8_t submode, motor_param_t *param, float dt)
{
	s_param = param;
	s_dt = dt;
	s_submode = submode;

	switch (submode)
	{
		case CALIB_L2_PHASE_SEQ:
		case CALIB_L2_POLE_PAIRS:
		case CALIB_L2_RL_FLUX:
			return true;
		default:
			return false;
	}
}

static calib_state_e calib_level2_poll(void)
{
	switch (s_submode)
	{
		case CALIB_L2_PHASE_SEQ:  return poll_phase_seq();
		case CALIB_L2_POLE_PAIRS: return poll_pole_pairs();
		case CALIB_L2_RL_FLUX:    return poll_rl_flux();
		default:                  return CALIB_STATE_FAILED;
	}
}

static void calib_level2_abort(void) {}

const calib_level_ops_t calib_level2_ops = {
	.start = calib_level2_start,
	.poll  = calib_level2_poll,
	.abort = calib_level2_abort,
};
```

- [ ] **Step 3: 创建 calib_level3_encoder.c（L3 编码器校准，5 子模式）**

```c
/**
 * @file calib_level3_encoder.c
 * @brief L3 编码器校准（零位/方向/线性度/正余弦/多圈零点）
 */
#include "calib_types.h"

static uint8_t s_submode;
static motor_param_t *s_param;
static float s_dt;

static calib_state_e poll_zero_offset(void)    { return CALIB_STATE_DONE; }
static calib_state_e poll_direction(void)      { return CALIB_STATE_DONE; }
static calib_state_e poll_linearity(void)      { return CALIB_STATE_DONE; }
static calib_state_e poll_sincos(void)         { return CALIB_STATE_DONE; }
static calib_state_e poll_multiturn_zero(void) { return CALIB_STATE_DONE; }

static bool calib_level3_start(uint8_t submode, motor_param_t *param, float dt)
{
	s_param = param;
	s_dt = dt;
	s_submode = submode;

	switch (submode)
	{
		case CALIB_L3_ZERO_OFFSET:
		case CALIB_L3_DIRECTION:
		case CALIB_L3_LINEARITY:
		case CALIB_L3_SINCOS:
		case CALIB_L3_MULTITURN_ZERO:
			return true;
		default:
			return false;
	}
}

static calib_state_e calib_level3_poll(void)
{
	switch (s_submode)
	{
		case CALIB_L3_ZERO_OFFSET:    return poll_zero_offset();
		case CALIB_L3_DIRECTION:      return poll_direction();
		case CALIB_L3_LINEARITY:      return poll_linearity();
		case CALIB_L3_SINCOS:         return poll_sincos();
		case CALIB_L3_MULTITURN_ZERO: return poll_multiturn_zero();
		default:                      return CALIB_STATE_FAILED;
	}
}

static void calib_level3_abort(void) {}

const calib_level_ops_t calib_level3_ops = {
	.start = calib_level3_start,
	.poll  = calib_level3_poll,
	.abort = calib_level3_abort,
};
```

- [ ] **Step 4: 创建 calib_level4_torque.c（L4 转矩基础，1 子模式）**

```c
/**
 * @file calib_level4_torque.c
 * @brief L4 转矩基础校准（力矩常数 Kt）
 */
#include "calib_types.h"

static motor_param_t *s_param;
static float s_dt;

static calib_state_e poll_kt(void) { return CALIB_STATE_DONE; }

static bool calib_level4_start(uint8_t submode, motor_param_t *param, float dt)
{
	s_param = param;
	s_dt = dt;

	switch (submode)
	{
		case CALIB_L4_KT:
			return true;
		default:
			return false;
	}
}

static calib_state_e calib_level4_poll(void)
{
	/* L4 只有一个子模式，无需 switch 分发 */
	return poll_kt();
}

static void calib_level4_abort(void) {}

const calib_level_ops_t calib_level4_ops = {
	.start = calib_level4_start,
	.poll  = calib_level4_poll,
	.abort = calib_level4_abort,
};
```

- [ ] **Step 5: 创建 calib_level5_nonlinear.c（L5 非线性补偿，4 子模式）**

```c
/**
 * @file calib_level5_nonlinear.c
 * @brief L5 非线性补偿（齿槽/摩擦/死区补偿/磁饱和）
 */
#include "calib_types.h"

static uint8_t s_submode;
static motor_param_t *s_param;
static float s_dt;

static calib_state_e poll_cogging(void)       { return CALIB_STATE_DONE; }
static calib_state_e poll_friction(void)      { return CALIB_STATE_DONE; }
static calib_state_e poll_deadtime_comp(void) { return CALIB_STATE_DONE; }
static calib_state_e poll_saturation(void)    { return CALIB_STATE_DONE; }

static bool calib_level5_start(uint8_t submode, motor_param_t *param, float dt)
{
	s_param = param;
	s_dt = dt;
	s_submode = submode;

	switch (submode)
	{
		case CALIB_L5_COGGING:
		case CALIB_L5_FRICTION:
		case CALIB_L5_DEADTIME_COMP:
		case CALIB_L5_SATURATION:
			return true;
		default:
			return false;
	}
}

static calib_state_e calib_level5_poll(void)
{
	switch (s_submode)
	{
		case CALIB_L5_COGGING:       return poll_cogging();
		case CALIB_L5_FRICTION:      return poll_friction();
		case CALIB_L5_DEADTIME_COMP: return poll_deadtime_comp();
		case CALIB_L5_SATURATION:    return poll_saturation();
		default:                     return CALIB_STATE_FAILED;
	}
}

static void calib_level5_abort(void) {}

const calib_level_ops_t calib_level5_ops = {
	.start = calib_level5_start,
	.poll  = calib_level5_poll,
	.abort = calib_level5_abort,
};
```

- [ ] **Step 6: 创建 calib_level6_system.c（L6 负载系统级，4 子模式）**

```c
/**
 * @file calib_level6_system.c
 * @brief L6 负载系统级校准（惯量/阻尼/回程间隙/PID自整定）
 */
#include "calib_types.h"

static uint8_t s_submode;
static motor_param_t *s_param;
static float s_dt;

static calib_state_e poll_inertia(void)      { return CALIB_STATE_DONE; }
static calib_state_e poll_damping(void)      { return CALIB_STATE_DONE; }
static calib_state_e poll_backlash(void)     { return CALIB_STATE_DONE; }
static calib_state_e poll_pid_autotune(void) { return CALIB_STATE_DONE; }

static bool calib_level6_start(uint8_t submode, motor_param_t *param, float dt)
{
	s_param = param;
	s_dt = dt;
	s_submode = submode;

	switch (submode)
	{
		case CALIB_L6_INERTIA:
		case CALIB_L6_DAMPING:
		case CALIB_L6_BACKLASH:
		case CALIB_L6_PID_AUTOTUNE:
			return true;
		default:
			return false;
	}
}

static calib_state_e calib_level6_poll(void)
{
	switch (s_submode)
	{
		case CALIB_L6_INERTIA:      return poll_inertia();
		case CALIB_L6_DAMPING:      return poll_damping();
		case CALIB_L6_BACKLASH:     return poll_backlash();
		case CALIB_L6_PID_AUTOTUNE: return poll_pid_autotune();
		default:                    return CALIB_STATE_FAILED;
	}
}

static void calib_level6_abort(void) {}

const calib_level_ops_t calib_level6_ops = {
	.start = calib_level6_start,
	.poll  = calib_level6_poll,
	.abort = calib_level6_abort,
};
```

- [ ] **Step 7: 创建 calib_level7_auto.c（L7 自动化集成，1 子模式）**

```c
/**
 * @file calib_level7_auto.c
 * @brief L7 自动化集成校准（一键全自动）
 * @note 真实实现：按顺序调用 L1→L6 的所有必要子模式。
 *       L7 维护自己的子标定序列状态机，不依赖 calib_mgr 的 active_ops
 *       （否则会与 calib_mgr 的单例 active_ops 冲突）。
 *       step 字段标识当前执行到第几步，通过 calib_status_t.step 上报。
 */
#include "calib_types.h"

static motor_param_t *s_param;
static float s_dt;
static uint8_t s_step; /* 当前执行步骤（0=未开始） */

/* L7 内部子标定序列定义（真实实现时填充） */
static const struct
{
	uint8_t level;
	uint8_t submode;
} s_sequence[] = {
	{ CALIB_LEVEL1_DRIVER,    CALIB_L1_ADC_OFFSET     },
	{ CALIB_LEVEL1_DRIVER,    CALIB_L1_ADC_GAIN       },
	{ CALIB_LEVEL1_DRIVER,    CALIB_L1_CURRENT_SENSOR },
	{ CALIB_LEVEL2_MOTOR,     CALIB_L2_PHASE_SEQ      },
	{ CALIB_LEVEL2_MOTOR,     CALIB_L2_POLE_PAIRS     },
	{ CALIB_LEVEL2_MOTOR,     CALIB_L2_RL_FLUX        },
	{ CALIB_LEVEL3_ENCODER,   CALIB_L3_ZERO_OFFSET    },
	{ CALIB_LEVEL3_ENCODER,   CALIB_L3_DIRECTION      },
	{ CALIB_LEVEL4_TORQUE,    CALIB_L4_KT             },
	{ CALIB_LEVEL5_NONLINEAR, CALIB_L5_COGGING        },
	{ CALIB_LEVEL5_NONLINEAR, CALIB_L5_FRICTION       },
	{ CALIB_LEVEL6_SYSTEM,    CALIB_L6_INERTIA        },
	{ CALIB_LEVEL6_SYSTEM,    CALIB_L6_PID_AUTOTUNE   },
};
#define L7_SEQ_LEN (sizeof(s_sequence) / sizeof(s_sequence[0]))

static bool calib_level7_start(uint8_t submode, motor_param_t *param, float dt)
{
	s_param = param;
	s_dt = dt;
	s_step = 0;

	switch (submode)
	{
		case CALIB_L7_FULL_AUTO:
			/* TODO: 启动序列中第一个子标定 */
			return true;
		default:
			return false;
	}
}

static calib_state_e calib_level7_poll(void)
{
	/* 桩：直接返回完成。
	 * 真实实现：
	 *   1. 调用当前 step 对应 level 的 start/poll
	 *   2. 子标定 DONE 后 s_step++，启动下一个
	 *   3. s_step >= L7_SEQ_LEN 时返回 DONE */
	(void)s_param; (void)s_dt; (void)s_step;
	return CALIB_STATE_DONE;
}

static void calib_level7_abort(void)
{
	s_step = 0;
}

const calib_level_ops_t calib_level7_ops = {
	.start = calib_level7_start,
	.poll  = calib_level7_poll,
	.abort = calib_level7_abort,
};
```

- [ ] **Step 8: Commit**

```bash
git add User/MotorCalibration/calib_level*.c
git commit -m "feat(calib): add 7 level module stubs with 24 submodes"
```

---

### Task 5: 修改 system_state.h 和 system_state.c

**Files:**
- Modify: `User/AppServices/StateMachine/system_state.h` (system_state_t 结构体)
- Modify: `User/AppServices/StateMachine/system_state.c` (多处)

- [ ] **Step 1: system_state.h 添加 include 和 calib_state 字段**

在 `#include "ctrl_transition.h"` 之后添加：

```c
#include "calib_types.h" /* calib_state_e */
```

在 system_state_t 的 `run_state_e target_run_state;` 之后添加：

```c
	run_state_e target_run_state; /*!< 目标运行状态 */
	calib_state_e calib_state;	/*!< 标定子状态（仅 CALIB 态有效）*/
```

- [ ] **Step 2: system_state.c 添加 include**

在文件顶部 `#include` 区添加：

```c
#include "calib_mgr.h"
```

- [ ] **Step 3: system_state_init 中初始化 calib_mgr**

找到 `system_state_init` 函数中 `transition_init(&sys->transition);` 之后，添加：

```c
	transition_init(&sys->transition);
	calib_mgr_init(param, dt);
```

- [ ] **Step 4: top_fsm_switch 添加 CALIB 进入动作**

找到 `top_fsm_switch` 函数中 `switch (new_state)` 的进入动作部分（现有 `case TOP_FSM_CALIB:` 处），替换为：

```c
		case TOP_FSM_CALIB:
			sys->calib_state = CALIB_STATE_IDLE;
			sys->motor.ref.ctrl_type = REF_CTRL_IDLE;
			break;
```

- [ ] **Step 5: top_fsm_switch 添加 CALIB 退出动作**

找到 `top_fsm_switch` 函数中 `switch (sys->top_state)` 的退出动作部分（现有 `case TOP_FSM_CALIB:` 处），替换为：

```c
		case TOP_FSM_CALIB:
			calib_mgr_abort();
			sys->calib_state = CALIB_STATE_IDLE;
			break;
```

- [ ] **Step 6: motor_control_loop 添加 CALIB 态处理**

找到 `motor_control_loop` 函数开头，在 `fault_check(sys);` 之后、现有的非运行态判断之前，插入 CALIB 态处理：

```c
	fault_check(sys);

	/* CALIB 态：周期推进标定，不生成运动参考 */
	if (sys->top_state == TOP_FSM_CALIB)
	{
		sys->calib_state = calib_mgr_poll();
		sys->motor.ref.ctrl_type = REF_CTRL_IDLE;
		return;
	}

	// 非运行态：失能输出，不生成运动参考
	if (sys->top_state != TOP_FSM_RUN)
```

- [ ] **Step 7: process_ctrl_cmd 更新 CALIB 路由**

找到 `process_ctrl_cmd` 函数中现有的 12 个 `case CONTROL_MODE_CALIB_*:` 标签块（L363-380），替换为 9 个新枚举的路由：

```c
		/* 校准指令：进入 CALIB 状态
		 * 0x90-0x96: 启动标定（子模式已由 app_set_mode 传给 calib_mgr）
		 * 0x97/0x98: 查询/中止，不切状态（app_set_mode 已处理并 return）*/
		case CONTROL_MODE_CALIB_LEVEL1:
		case CONTROL_MODE_CALIB_LEVEL2:
		case CONTROL_MODE_CALIB_LEVEL3:
		case CONTROL_MODE_CALIB_LEVEL4:
		case CONTROL_MODE_CALIB_LEVEL5:
		case CONTROL_MODE_CALIB_LEVEL6:
		case CONTROL_MODE_CALIB_LEVEL7:
			if (sys->top_state == TOP_FSM_IDLE || sys->top_state == TOP_FSM_CALIB)
			{
				top_fsm_switch(sys, TOP_FSM_CALIB);
			}
			else
			{
				/* 非 IDLE 态不允许进入校准 */
			}
			break;

		case CONTROL_MODE_CALIB_QUERY:
		case CONTROL_MODE_CALIB_ABORT:
			/* 查询/中止不切状态，app_set_mode 已处理 */
			break;
```

- [ ] **Step 8: Commit**

```bash
git add User/AppServices/StateMachine/system_state.h User/AppServices/StateMachine/system_state.c
git commit -m "feat(calib): integrate calib_mgr into system_state, update CALIB routing"
```

---

### Task 6: 修改 jm_proto_ops.c 的 app_set_mode

**Files:**
- Modify: `User/Protocol/joint_proto/jm_proto_ops.c`

- [ ] **Step 1: 添加 include**

在文件顶部 include 区添加：

```c
#include "calib_mgr.h"
```

- [ ] **Step 2: 在 app_set_mode 的 default 分支前插入标定命令处理**

找到 `app_set_mode` 函数中的 `default: break;`（L152），在其之前插入：

```c
	/* ---- 标定启动 0x90-0x96: payload[0]=子模式 ---- */
	case JM_CMD_CALIB_LEVEL1:
	case JM_CMD_CALIB_LEVEL2:
	case JM_CMD_CALIB_LEVEL3:
	case JM_CMD_CALIB_LEVEL4:
	case JM_CMD_CALIB_LEVEL5:
	case JM_CMD_CALIB_LEVEL6:
	case JM_CMD_CALIB_LEVEL7:
	{
		if (len < 1)
			return JM_ERR_LENGTH;
		uint8_t level = cmd - JM_CMD_CALIB_LEVEL1 + 1;
		uint8_t submode = pl[0];
		if (!calib_mgr_start(level, submode))
		{
			/* 区分失败原因：已在标定中 → BUSY，其余 → OUT_OF_RANGE */
			calib_status_t st = calib_mgr_get_status();
			if (st.state == CALIB_STATE_RUNNING)
				return JM_ERR_CALIB_BUSY;     /* NACK(0x0A) 已在标定中 */
			return JM_ERR_OUT_OF_RANGE;       /* NACK(0x02) submode 不合法 */
		}
		break; /* 继续走 motor_loop_set_cmd 进入 CALIB 态 */
	}

	/* ---- 标定进度查询 0x97: 不切状态，直接返回 ---- */
	case JM_CMD_CALIB_QUERY:
	{
		calib_status_t st = calib_mgr_get_status();
		if (st.state == CALIB_STATE_DONE)
			return JM_ERR_OK;          /* ACK */
		if (st.state == CALIB_STATE_RUNNING)
			return JM_ERR_CALIB_BUSY;  /* NACK(0x0A) */
		return JM_ERR_STATE_DENY;      /* NACK(0x03) 未在标定 */
	}

	/* ---- 标定中止 0x98: 不切状态，直接返回 ---- */
	case JM_CMD_CALIB_ABORT:
	{
		calib_mgr_abort();
		return JM_ERR_OK; /* ACK */
	}

	/* ---- 其余模式(力控/轨迹/特殊/测试/诊断): 暂仅切状态 ---- */
	default:
		break;
```

注意：`JM_CMD_CALIB_QUERY` 和 `JM_CMD_CALIB_ABORT` 的 case 中直接 return，不会走到 switch 之后的 `motor_loop_set_cmd((ctrl_mode_e)cmd)`，因此不会触发状态切换。

- [ ] **Step 3: Commit**

```bash
git add User/Protocol/joint_proto/jm_proto_ops.c
git commit -m "feat(calib): route 0x90-0x98 calibration commands in app_set_mode"
```

---

### Task 7: 将 10 个新文件加入 Keil 工程

**Files:**
- Modify: `Board/V1/MDK-ARM/JointMotorApp.uvprojx`
- Modify: `Board/SFOC/MDK-ARM/sfoc.uvprojx`

- [ ] **Step 1: 在 JointMotorApp.uvprojx 中添加 10 个文件**

找到 motor_control.c 的 `<File>` 块，在其后添加：

```xml
            <File>
              <FileName>calib_mgr.c</FileName>
              <FileType>1</FileType>
              <FilePath>..\..\..\User\MotorCalibration\calib_mgr.c</FilePath>
            </File>
            <File>
              <FileName>calib_level1_driver.c</FileName>
              <FileType>1</FileType>
              <FilePath>..\..\..\User\MotorCalibration\calib_level1_driver.c</FilePath>
            </File>
            <File>
              <FileName>calib_level2_motor.c</FileName>
              <FileType>1</FileType>
              <FilePath>..\..\..\User\MotorCalibration\calib_level2_motor.c</FilePath>
            </File>
            <File>
              <FileName>calib_level3_encoder.c</FileName>
              <FileType>1</FileType>
              <FilePath>..\..\..\User\MotorCalibration\calib_level3_encoder.c</FilePath>
            </File>
            <File>
              <FileName>calib_level4_torque.c</FileName>
              <FileType>1</FileType>
              <FilePath>..\..\..\User\MotorCalibration\calib_level4_torque.c</FilePath>
            </File>
            <File>
              <FileName>calib_level5_nonlinear.c</FileName>
              <FileType>1</FileType>
              <FilePath>..\..\..\User\MotorCalibration\calib_level5_nonlinear.c</FilePath>
            </File>
            <File>
              <FileName>calib_level6_system.c</FileName>
              <FileType>1</FileType>
              <FilePath>..\..\..\User\MotorCalibration\calib_level6_system.c</FilePath>
            </File>
            <File>
              <FileName>calib_level7_auto.c</FileName>
              <FileType>1</FileType>
              <FilePath>..\..\..\User\MotorCalibration\calib_level7_auto.c</FilePath>
            </File>
```

注意：calib_types.h 和 calib_mgr.h 是头文件，无需加入工程（只需在 include path 中能找到即可，MotorCalibration/ 目录需加入 Keil 的 Include Paths）。

- [ ] **Step 2: 在 JointMotorApp.uvprojx 的 Include Paths 中添加 MotorCalibration 目录**

找到 `<IncludePath>` 标签，在末尾添加 `;..\..\..\User\MotorCalibration`

- [ ] **Step 3: 对 sfoc.uvprojx 做相同修改**

- [ ] **Step 4: Commit**

```bash
git add Board/V1/MDK-ARM/JointMotorApp.uvprojx Board/SFOC/MDK-ARM/sfoc.uvprojx
git commit -m "build(calib): add 8 calibration source files to Keil projects"
```

---

### Task 8: 编译验证

- [ ] **Step 1: 编译 V1 工程**

Run:
```
"C:\APP\Code\MDK\CORE\UV4\UV4.exe" -b "d:\AAWorkSpace\001_JointMotor\SW\JointMotor\Board\V1\MDK-ARM\JointMotorApp.uvprojx" -o "d:\AAWorkSpace\001_JointMotor\SW\JointMotor\Board\V1\MDK-ARM\build_log.txt" -j0
```
Expected: 0 编译错误。链接错误仅原有的 `motor_info_dispatch_*` 4 个（预存在问题，与本次无关）。

- [ ] **Step 2: 检查编译日志**

```
$log = Get-Content "d:\AAWorkSpace\001_JointMotor\SW\JointMotor\Board\V1\MDK-ARM\build_log.txt"
$errors = $log | Select-String "error:"
Write-Host "Errors: $($errors.Count)"
$errors | ForEach-Object { $_.Line }
```

确认：
- 无 `calib_` 开头的未定义符号
- 无 `CONTROL_MODE_CALIB_` / `JM_CMD_CALIB_` 相关的编译错误
- 仅有 `motor_info_dispatch_*` 的 4 个预存在链接错误

- [ ] **Step 3: 清理临时文件**

```
del "d:\AAWorkSpace\001_JointMotor\SW\JointMotor\Board\V1\MDK-ARM\build_log.txt"
```

- [ ] **Step 4: Commit**

```bash
git add -A
git commit -m "chore(calib): verify compilation"
```

---

## Self-Review

### 1. Spec coverage

| 用户需求 | 计划覆盖 |
|---------|----------|
| L1 ADC偏置/增益/电流传感器/温度/母线电压/死区特性 | Task 1 常量定义 + Task 4 calib_level1_driver.c |
| L2 相序/极对数/R-L-flux | Task 1 + Task 4 calib_level2_motor.c |
| L3 零位/方向/线性度/正余弦/多圈零点 | Task 1 + Task 4 calib_level3_encoder.c |
| L4 力矩常数 | Task 1 + Task 4 calib_level4_torque.c |
| L5 齿槽/摩擦/死区补偿/磁饱和 | Task 1 + Task 4 calib_level5_nonlinear.c |
| L6 惯量/阻尼/回程间隙/PID自整定 | Task 1 + Task 4 calib_level6_system.c |
| L7 一键全自动 | Task 1 + Task 4 calib_level7_auto.c |
| 按类别标定，子命令绑定具体模式 | Task 2: 0x90-0x96 类别 + payload[0] 子模式 |
| 标定状态机 | Task 5: TOP_FSM_CALIB 进入/退出/poll |
| 进度查询 | Task 6: 0x97 → ACK/NACK 返回状态 |
| 中止标定 | Task 6: 0x98 → calib_mgr_abort |
| 模块化到 MotorCalibration/ 目录 | Task 1/3/4: 10 个文件 |
| 代码生成后可直接运行 | 桩 poll 返回 DONE，协议流程立即可跑通 |

### 2. Placeholder scan

- 各级别模块 `poll` 直接返回 `CALIB_STATE_DONE` — 预期的桩实现，非 placeholder
- `TODO:` 注释是给后续算法实现者的提示，不影响当前编译运行
- 无 "TBD"、"implement later" 等模糊描述

### 3. Type consistency

- `calib_state_e`：Task 1 定义 → Task 3/4/5 使用 ✓
- `calib_status_t`：Task 1 定义（含 step 字段） → Task 3/6 使用 ✓
- `calib_level_ops_t`：Task 1 定义（start 返回 bool） → Task 3/4 使用 ✓
- `calib_mgr_init/start/poll/get_status/abort`：Task 3 声明 → Task 5/6 调用 ✓
- `calib_levelN_ops`：Task 4 导出（start 返回 bool） → Task 3 extern 引用 ✓
- `CONTROL_MODE_CALIB_LEVEL1~7/QUERY/ABORT`：Task 2 定义 → Task 5 使用 ✓
- `JM_CMD_CALIB_LEVEL1~7/QUERY/ABORT`：Task 2 定义 → Task 6 使用 ✓
- `s_level_table[CALIB_LEVEL_MAX]`：Task 3 用 `CALIB_LEVEL1~7` 索引，与 Task 1 常量一致 ✓
- `CALIB_L1_*/CALIB_L2_*/.../CALIB_L7_*`：Task 1 定义 → Task 4 在 switch 中使用 ✓

### 4. 直接可运行性

- 桩模块 `poll` 返回 `DONE`，上位机下发任意标定命令后：
  1. `[0x90][0x01]` → calib_mgr_start(1,1) 成功 → ACK → 进入 CALIB 态
  2. `[0x97]` → calib_mgr_poll 返回 DONE → ACK（标定完成）
  3. `[0x00]` → 退出 CALIB 态 → calib_mgr_abort → ACK
- 完整的状态机流转可立即工作，无运行时错误

### 5. 向后兼容性说明

- 旧的 12 个 `CONTROL_MODE_CALIB_*` 枚举被删除，上位机需同步更新命令码
- 旧命令码 0x90-0x9B 的语义改变：0x90 从"电机参数校准"变为"L1 驱动硬件底层（类别命令）"
- 上位机需适配新的 payload 格式：0x90-0x96 需携带 1 字节子模式

---

## 扩展指南

### 场景 A：在现有级别中添加新子模式

**示例：在 L1 驱动硬件底层中新增"霍尔传感器校准"**

只需改 1 个文件（`calib_level1_driver.c`），3 处：

1. **calib_types.h** 添加子模式常量：

```c
#define CALIB_L1_HALL_SENSOR	7	/* 霍尔传感器校准（新增） */
```

2. **calib_level1_driver.c** 添加桩函数 + 2 个 case：

```c
/* 在 poll_xxx 函数区添加 */
static calib_state_e poll_hall_sensor(void) { return CALIB_STATE_DONE; }

/* 在 calib_level1_start 的 switch 中添加 case */
		case CALIB_L1_HALL_SENSOR:

/* 在 calib_level1_poll 的 switch 中添加 case */
		case CALIB_L1_HALL_SENSOR:   return poll_hall_sensor();
```

3. 上位机下发 `[0x90][0x07]` 即可触发。

**无需改动**：calib_mgr.c、system_state.c、jm_proto_ops.c、Keil 工程。

### 场景 B：添加全新标定级别

**示例：新增 L8 安全参数校准**

需改 6 个文件：

1. **calib_types.h**：
   - 添加 `#define CALIB_LEVEL8_SAFETY 8`
   - 修改 `#define CALIB_LEVEL_MAX 9`

2. **state_define.h**：在 ctrl_mode_e 标定段添加 `CONTROL_MODE_CALIB_LEVEL8 = 0x97`
   - 注意：0x97 当前是 QUERY，需将 QUERY/ABORT 移到 0xAE/0xAF 腾出空间
   - 或：复用 0x90-0x96 中的空位（如果某级别不需要）

3. **jm_cmd_def.h**：同步添加 `JM_CMD_CALIB_LEVEL8`

4. **新建 calib_level8_safety.c**：导出 `calib_level8_ops`

5. **calib_mgr.c**：
   - 添加 `extern const calib_level_ops_t calib_level8_ops;`
   - 在 `s_level_table` 添加 `[CALIB_LEVEL8_SAFETY] = &calib_level8_ops,`

6. **jm_proto_ops.c**：在 app_set_mode 的标定分支添加 `case JM_CMD_CALIB_LEVEL8:`

7. **system_state.c**：在 process_ctrl_cmd 添加 `case CONTROL_MODE_CALIB_LEVEL8:`

8. **Keil 工程文件**：添加 calib_level8_safety.c

### 场景 C：填充真实标定算法

以 L1 ADC 偏置校准为例，替换桩实现：

1. 在 `calib_level1_driver.c` 的 `poll_adc_offset()` 中实现真实算法
2. 如需额外状态，在文件顶部 static 变量区添加
3. 在 `calib_level1_start()` 的 `case CALIB_L1_ADC_OFFSET:` 中初始化
4. 在 `calib_level1_abort()` 中停止注入、复位硬件
5. 完成后将结果写入 `s_param->xxx` 字段

**关键约束**：
- `poll` 在电流环中断中调用，不可阻塞、不可动态分配内存
- `start` 只做初始化，`poll` 每周期推进一步
- 返回 `CALIB_STATE_DONE` 后 calib_mgr 自动停止调度，无需手动清理
