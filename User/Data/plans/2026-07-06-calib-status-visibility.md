# 标定状态细化可观测性实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 让单步标定卡死在哪个环节、一键标定当前进行到哪一步,都能通过 0x97 查询命令清晰可见。level 模块主动上报 step 与 fail_reason,calib_mgr 统一管理状态,L7 序列进度对外可读。

**Architecture:** 三层细化:(1) level 模块在 poll 时通过 `calib_mgr_set_step()` 上报当前步骤号;(2) level 模块返回 FAILED 时通过 `calib_mgr_set_fail_reason()` 设置具体失败原因,覆盖 calib_mgr 的默认 TIMEOUT;(3) L7 在序列推进时通过 `calib_mgr_set_step_progress()` 上报当前步骤索引与总步数。协议层 0x97 改为携带详细状态 ACK,上位机一次读取全部信息。

**Tech Stack:** STM32G4, 嵌入式 C (C99), Keil MDK-ARM, 现有 jm_proto 协议栈, C-OOP ops 函数表模式

---

## 现状分析

### 问题 1: 单步标定卡死环节不可见
- `calib_status_t.step` 字段已存在([calib_types.h:99](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorCalibration/calib_types.h#L99)),但 level 模块(L1-L6)从未写入,一直是 0
- `fail_reason` 由 calib_mgr 默认填 `CALIB_FAIL_TIMEOUT`([calib_mgr.c:122-123](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorCalibration/calib_mgr.c#L122)),level 模块返回 FAILED 时未设置具体原因
- 例如 L2.3 R 标定 STEP 2 因 `id < 0.001f` 失败,上位机只能看到 NACK(0x03),无法区分是"电流采样异常"、"电机未响应"还是"结果超范围"

### 问题 2: 一键标定步骤不可见
- L7 内部 `s_step` 是 [calib_level7_auto.c:26](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorCalibration/calib_level7_auto.c#L26) 模块私有变量
- `calib_status_t.step` 在 L7 也未写入;`step_total` 字段一直为 0
- 上位机发 0x97 查询,只收到 ACK(0x00)/NACK(0x0A)/NACK(0x03),不知道当前是 L7 序列的第几步、共多少步

### 问题 3: CALIB_QUERY 应答信息不足
- [jm_proto_ops.c:180-188](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/Protocol/joint_proto/jm_proto_ops.c#L180) `JM_CMD_CALIB_QUERY` 只返回单字节 ACK(0x00) 或 NACK
- `calib_status_t` 结构体有 7 字节丰富信息(state/fail_reason/progress/level/submode/step/step_total),但协议层从未上报

## 文件结构

### 固件修改文件

| 文件 | 职责 | 修改内容 |
|------|------|----------|
| `MotorCalibration/calib_mgr.h` | 标定管理器接口 | 新增 `calib_mgr_set_step()` / `calib_mgr_set_fail_reason()` / `calib_mgr_set_l7_progress()` 接口 |
| `MotorCalibration/calib_mgr.c` | 标定管理器实现 | 实现新接口;修复 `calib_mgr_poll` 不覆盖 level 模块已设置的 fail_reason |
| `MotorCalibration/calib_level2_motor.c` | L2 电机标定 | 各 poll_* 函数在每个 step 推进时调 `calib_mgr_set_step()`,FAILED 时调 `calib_mgr_set_fail_reason()` |
| `MotorCalibration/calib_level3_encoder.c` | L3 编码器标定 | 同 L2,各 step 推进与 FAILED 时上报 |
| `MotorCalibration/calib_level7_auto.c` | L7 一键标定 | 序列推进时调 `calib_mgr_set_l7_progress()`;失败时记录失败步骤的 level/submode |
| `Protocol/joint_proto/jm_proto.c` | 协议层 dispatch | `JM_CMD_CALIB_QUERY` case 直接调 `calib_mgr_get_status()` 组织 8 字节 ACK,不经过 ops |
| `Protocol/joint_proto/jm_proto_ops.c` | 协议层 ops | 移除 `JM_CMD_CALIB_QUERY` 分支(改由 dispatch 直接处理) |
| `Protocol/joint_proto/jm_cmd_def.h` | 协议层命令注释 | 更新 0x97 注释为"返回 8 字节详细状态 ACK" |

### 上位机修改文件

| 文件 | 职责 | 修改内容 |
|------|------|----------|
| `Tools/pyqt_gui/jmproto/cmd_def.py` | 命令码定义 | 更新 0x97 注释;新增 `CalibState` / `CalibFailReason` 枚举 |
| `Tools/pyqt_gui/jmproto/__init__.py` | jmproto 包出口 | 新增 `parse_calib_status(payload)` 解析 8 字节状态 |
| `Tools/pyqt_gui/core/motor_client.py` | 通信客户端 | 新增 `calib_status_received` 信号;`_on_frame` 特判 0x97 解析后 emit |
| `Tools/pyqt_gui/ui/main_window.py` | 主窗口 | 连接 `calib_status_received` 到 `_calib_panel.on_calib_status` |
| `Tools/pyqt_gui/ui/panels/calibration_panel.py` | 标定面板 | 新增 `on_calib_status(state, fail_reason, progress, level, submode, step, step_total)` 槽;UI 增加 step/step_total/progress/fail_reason 显示;`on_ack`/`on_nack` 中 0x97 分支改为不做事(由 `on_calib_status` 处理) |
| `Tools/pyqt_gui/resources/joint_motor_command_list.csv` | 命令列表 CSV | 更新 0x97 行:ACK 载荷 8 字节,字段定义 |

### 不修改文件
- `calib_types.h`:`calib_status_t` 字段已够用,无需改结构
- `calib_step.h`:状态机骨架不变
- L1/L4/L5/L6 模块:当前为桩实现,后续填充时按 L2/L3 的模式上报即可

---

## Task 1: calib_mgr 新增状态上报接口

**Files:**
- Modify: `User/MotorCalibration/calib_mgr.h`
- Modify: `User/MotorCalibration/calib_mgr.c`

- [ ] **Step 1: 在 calib_mgr.h 添加三个上报接口声明**

在 `calib_mgr_get_status` 声明之后([calib_mgr.h:31](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorCalibration/calib_mgr.h#L31) 之后)追加:

```c
/**
 * @brief level 模块上报当前步骤号（单步标定用）
 * @note  level 模块在 calib_step_next 推进到新 step 时调用,
 *        使 0x97 查询能反映"卡在第几步"。step 从 0 开始。
 */
void calib_mgr_set_step(uint8_t step);

/**
 * @brief level 模块上报具体失败原因（覆盖 calib_mgr 默认的 TIMEOUT）
 * @note  level 模块在判定 FAILED 时调用,设置具体原因如
 *        CALIB_FAIL_OUT_OF_RANGE / CALIB_FAIL_SAMPLE_ABNORMAL 等。
 *        若不调用,calib_mgr 默认填 CALIB_FAIL_TIMEOUT。
 */
void calib_mgr_set_fail_reason(calib_fail_reason_e reason);

/**
 * @brief L7 上报序列进度（当前步骤索引 + 总步数）
 * @param step     当前步骤索引（0-based）
 * @param step_total L7 序列总步数
 * @note  同时设置 status.step 与 status.step_total, 供 0x97 查询。
 *        L7 调用此接口后, status.level/submode 也会被更新为当前子项。
 */
void calib_mgr_set_l7_progress(uint8_t step, uint8_t step_total);
```

- [ ] **Step 2: 在 calib_mgr.c 实现三个接口**

在 `calib_mgr_get_status` 函数之后([calib_mgr.c:132](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorCalibration/calib_mgr.c#L132) 之后)追加:

```c
void calib_mgr_set_step(uint8_t step)
{
	s_mgr.status.step = step;
}

void calib_mgr_set_fail_reason(calib_fail_reason_e reason)
{
	s_mgr.status.fail_reason = reason;
}

void calib_mgr_set_l7_progress(uint8_t step, uint8_t step_total)
{
	s_mgr.status.step = step;
	s_mgr.status.step_total = step_total;
	/* L7 当前子项的 level/submode 由 s_sequence[step] 决定, 此处不更新;
	 * level/submode 在 calib_mgr_start 时已设为 L7/FULL_AUTO, 保持不变即可。
	 * L7 poll 内部推进子标定时若需细化, 可由 L7 直接调 calib_mgr_set_step。*/
}
```

- [ ] **Step 3: 修复 calib_mgr_poll 不覆盖 level 模块已设置的 fail_reason**

修改 [calib_mgr.c:118-124](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorCalibration/calib_mgr.c#L118) 的 FAILED 分支:

```c
	else if (st == CALIB_STATE_FAILED)
	{
		s_mgr.status.progress = 0;
		/* level 模块可能已通过 calib_mgr_set_fail_reason 设置具体原因,
		 * 仅当未设置(仍为 NONE)时才默认记为超时 */
		if (s_mgr.status.fail_reason == CALIB_FAIL_NONE)
			s_mgr.status.fail_reason = CALIB_FAIL_TIMEOUT;
	}
```

(代码实际未变,但确认逻辑:level 模块在返回 FAILED 前调 `calib_mgr_set_fail_reason` 设置具体原因,此处 `if (== NONE)` 判断保证不覆盖)

- [ ] **Step 4: 编译验证**

Run: Keil 编译
Expected: 0 error, 0 warning(新接口有声明有实现,无未定义引用)

- [ ] **Step 5: Commit**

```bash
git add User/MotorCalibration/calib_mgr.h User/MotorCalibration/calib_mgr.c
git commit -m "feat(calib): add status reporting APIs for level modules"
```

---

## Task 2: L2 电机标定上报 step 与 fail_reason

**Files:**
- Modify: `User/MotorCalibration/calib_level2_motor.c`

- [ ] **Step 1: 在 poll_resistance 每个 step 推进与 FAILED 处上报**

修改 [calib_level2_motor.c:47-108](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorCalibration/calib_level2_motor.c#L47) `poll_resistance`:

在 `calib_step_next` 之后添加 `calib_mgr_set_step` 调用,在 FAILED 返回前添加 `calib_mgr_set_fail_reason`。完整修改后的函数:

```c
static calib_state_e poll_resistance(void)
{
	const calib_io_t *io = calib_mgr_get_io();
	dev_motor_t *m = io->motor;

	switch (s_l2.step.cur)
	{
		case 0: /* 进入标定会话，施加 DC 电压 */
			calib_hw_enter(&s_l2.session, m);
			s_l2.test_voltage = CALIB_CFG_L2_R_TEST_VOLTAGE_V;
			calib_hw_apply_voltage(&s_l2.session, s_l2.test_voltage, 0.0f, 0.0f);
			calib_mgr_set_step(0);
			calib_step_next(&s_l2.step, 1);
			calib_mgr_set_step(1);
			return CALIB_STATE_RUNNING;

		case 1: /* 等待稳态 */
			calib_hw_apply_voltage(&s_l2.session, s_l2.test_voltage, 0.0f, 0.0f);
			if (calib_step_wait(&s_l2.step, CALIB_CFG_L2_R_TEST_TICKS))
				return CALIB_STATE_RUNNING;
			calib_step_next(&s_l2.step, 2);
			calib_mgr_set_step(2);
			return CALIB_STATE_RUNNING;

		case 2: /* 多次采样 id */
		{
			calib_hw_apply_voltage(&s_l2.session, s_l2.test_voltage, 0.0f, 0.0f);
			m->foc.clarke(&m->foc);
			m->foc.park(&m->foc);
			float id = m->foc.i_dq.d;
			if (!isfinite(id) || id < 0.001f)
			{
				calib_mgr_set_fail_reason(CALIB_FAIL_SAMPLE_ABNORMAL);
				calib_hw_exit(&s_l2.session);
				return CALIB_STATE_FAILED;
			}
			calib_step_accumulate(&s_l2.step, id);
			if (s_l2.step.sample_cnt < CALIB_CFG_L2_R_SAMPLE_COUNT)
				return CALIB_STATE_RUNNING;
			calib_step_next(&s_l2.step, 3);
			calib_mgr_set_step(3);
			return CALIB_STATE_RUNNING;
		}

		case 3: /* 计算 R，校验，写入 */
		{
			float id_avg = calib_step_average(&s_l2.step);
			float R = s_l2.test_voltage / id_avg;
			if (!calib_validate_r(R))
			{
				calib_mgr_set_fail_reason(CALIB_FAIL_OUT_OF_RANGE);
				calib_hw_exit(&s_l2.session);
				return CALIB_STATE_FAILED;
			}
			motor_param_set_r(io->param, R);
			(void)motor_info_calib_submit_r(R);
			calib_hw_exit(&s_l2.session);
			calib_mgr_mark_done(CALIB_LEVEL2_MOTOR, CALIB_L2_RESISTANCE);
			calib_step_reset(&s_l2.step);
			return CALIB_STATE_DONE;
		}

		default:
			calib_mgr_set_fail_reason(CALIB_FAIL_TIMEOUT);
			calib_hw_exit(&s_l2.session);
			return CALIB_STATE_FAILED;
	}
}
```

- [ ] **Step 2: 在 poll_inductance_d 上报 step 与 fail_reason**

修改 [calib_level2_motor.c:118-183](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorCalibration/calib_level2_motor.c#L118) `poll_inductance_d`,按相同模式在每个 `calib_step_next` 后调 `calib_mgr_set_step(next)`,在 FAILED 返回前调 `calib_mgr_set_fail_reason`:

- case 0 `calib_step_next(&s_l2.step, 1)` 后加 `calib_mgr_set_step(1);`
- case 1 `calib_step_next(&s_l2.step, 2)` 后加 `calib_mgr_set_step(2);`
- case 2 `if (s_l2.step.sample_cnt == 0)` 失败处加 `calib_mgr_set_fail_reason(CALIB_FAIL_SAMPLE_ABNORMAL);`
- case 2 `if (!calib_validate_ld(Ld))` 失败处加 `calib_mgr_set_fail_reason(CALIB_FAIL_OUT_OF_RANGE);`
- default 失败处加 `calib_mgr_set_fail_reason(CALIB_FAIL_TIMEOUT);`

- [ ] **Step 3: 在 poll_inductance_q 上报 step 与 fail_reason**

修改 [calib_level2_motor.c:193-257](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorCalibration/calib_level2_motor.c#L193) `poll_inductance_q`,模式同 Step 2:

- case 0 推进到 1 后加 `calib_mgr_set_step(1);`
- case 1 推进到 2 后加 `calib_mgr_set_step(2);`
- case 2 `sample_cnt == 0` 失败加 `CALIB_FAIL_SAMPLE_ABNORMAL`
- case 2 `!calib_validate_lq` 失败加 `CALIB_FAIL_OUT_OF_RANGE`
- default 失败加 `CALIB_FAIL_TIMEOUT`

- [ ] **Step 4: 在 poll_flux_linkage 上报 step 与 fail_reason**

修改 [calib_level2_motor.c:272-340](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorCalibration/calib_level2_motor.c#L272) `poll_flux_linkage`:

- case 0 推进到 1 后加 `calib_mgr_set_step(1);`
- case 1 推进到 2 后加 `calib_mgr_set_step(2);`
- case 2 推进到 3 后加 `calib_mgr_set_step(3);`
- case 3 `sample_cnt == 0` 失败加 `CALIB_FAIL_SAMPLE_ABNORMAL`
- case 3 `!calib_validate_flux` 失败加 `CALIB_FAIL_OUT_OF_RANGE`
- default 失败加 `CALIB_FAIL_TIMEOUT`

- [ ] **Step 5: 在 poll_phase_seq 上报 step 与 fail_reason**

修改 [calib_level2_motor.c:352-409](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorCalibration/calib_level2_motor.c#L352) `poll_phase_seq`:

- case 0 推进到 1 后加 `calib_mgr_set_step(1);`
- case 1 推进到 2 后加 `calib_mgr_set_step(2);`
- case 2 推进到 3 后加 `calib_mgr_set_step(3);`
- case 3 推进到 4 后加 `calib_mgr_set_step(4);`
- case 4 `fabsf(delta) < 5.0f` 失败加 `calib_mgr_set_fail_reason(CALIB_FAIL_MOTOR_STUCK);`(电机未响应)
- default 失败加 `CALIB_FAIL_TIMEOUT`

- [ ] **Step 6: 在 poll_pole_pairs 上报 step 与 fail_reason**

修改 [calib_level2_motor.c:428-503](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorCalibration/calib_level2_motor.c#L428) `poll_pole_pairs`:

- case 0 推进到 1 后加 `calib_mgr_set_step(1);`
- case 1 推进到 2 后加 `calib_mgr_set_step(2);`
- case 2 推进到 3 后加 `calib_mgr_set_step(3);`
- case 3 推进到 4 后加 `calib_mgr_set_step(4);`
- case 4 `dmech_rad < 0.1f` 失败加 `calib_mgr_set_fail_reason(CALIB_FAIL_MOTOR_STUCK);`(转子未跟随)
- case 4 `!calib_validate_pole_pairs` 失败加 `calib_mgr_set_fail_reason(CALIB_FAIL_OUT_OF_RANGE);`
- default 失败加 `CALIB_FAIL_TIMEOUT`

- [ ] **Step 7: 编译验证**

Run: Keil 编译
Expected: 0 error, 0 warning

- [ ] **Step 8: Commit**

```bash
git add User/MotorCalibration/calib_level2_motor.c
git commit -m "feat(calib-L2): report step index and fail reason for diagnostics"
```

---

## Task 3: L3 编码器标定上报 step 与 fail_reason

**Files:**
- Modify: `User/MotorCalibration/calib_level3_encoder.c`

- [ ] **Step 1: 读取 calib_level3_encoder.c 当前实现**

Run: Read `User/MotorCalibration/calib_level3_encoder.c` 全文,确认两个 poll 函数(poll_zero_offset / poll_direction)的 step 结构与 FAILED 分支位置。

从之前 grep 结果已知:
- `poll_zero_offset`:case 0-3,case 2 多次采样,case 3 写入
- `poll_direction`:case 0-3,case 2 采样判定

- [ ] **Step 2: 在 poll_zero_offset 上报 step 与 fail_reason**

在 `poll_zero_offset` 的每个 `calib_step_next` 后调 `calib_mgr_set_step(next_step)`。FAILED 分支(若存在采样异常或校验失败)加 `calib_mgr_set_fail_reason`:

- case 0 推进到 1 后加 `calib_mgr_set_step(1);`
- case 1 推进到 2 后加 `calib_mgr_set_step(2);`
- case 2 推进到 3 后加 `calib_mgr_set_step(3);`
- 若有采样 NaN/异常分支加 `CALIB_FAIL_SAMPLE_ABNORMAL`
- 若有校验失败分支加 `CALIB_FAIL_OUT_OF_RANGE`
- default 加 `CALIB_FAIL_TIMEOUT`

- [ ] **Step 3: 在 poll_direction 上报 step 与 fail_reason**

在 `poll_direction` 的每个 `calib_step_next` 后调 `calib_mgr_set_step(next_step)`:

- case 0 推进到 1 后加 `calib_mgr_set_step(1);`
- case 1 推进到 2 后加 `calib_mgr_set_step(2);`
- case 2 推进到 3 后加 `calib_mgr_set_step(3);`
- 若有"角度变化不足/电机未转动"判定加 `CALIB_FAIL_MOTOR_STUCK`
- default 加 `CALIB_FAIL_TIMEOUT`

- [ ] **Step 4: 编译验证**

Run: Keil 编译
Expected: 0 error, 0 warning

- [ ] **Step 5: Commit**

```bash
git add User/MotorCalibration/calib_level3_encoder.c
git commit -m "feat(calib-L3): report step index and fail reason for diagnostics"
```

---

## Task 4: L7 一键标定上报序列进度

**Files:**
- Modify: `User/MotorCalibration/calib_level7_auto.c`

- [ ] **Step 1: 在 calib_level7_start 启动时上报 step_total**

修改 [calib_level7_auto.c:102-122](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorCalibration/calib_level7_auto.c#L102) `calib_level7_start`,在 `s_step = 0` 之后添加:

```c
static bool calib_level7_start(uint8_t submode, motor_param_t *param, float dt)
{
	s_param = param;
	s_dt = dt;
	s_step = 0;
	s_failed_step = 0xFF;
	s_retry_count = 0;

	/* 上报 L7 序列总步数, 使 0x97 查询能看到 step_total */
	calib_mgr_set_l7_progress(0, (uint8_t)L7_SEQ_LEN);

	switch (submode)
	{
		case CALIB_L7_FULL_AUTO:
			return l7_start_current_step();
		default:
			return false;
	}
}
```

- [ ] **Step 2: 在 calib_level7_poll 序列推进时上报当前 step**

修改 [calib_level7_auto.c:124-188](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorCalibration/calib_level7_auto.c#L124) `calib_level7_poll`,在每次 `s_step++` 之后调 `calib_mgr_set_l7_progress`:

在 `calib_state_e st = l7_poll_current_step();` 之前添加:

```c
	/* 上报当前步骤索引(L7 内部 step, 区别于子标定的内部 step) */
	calib_mgr_set_l7_progress(s_step, (uint8_t)L7_SEQ_LEN);
```

在每个 `s_step++` 之后(共 3 处:DONE 推进、SKIP 推进、RETRY_ONCE 跳过推进)添加:

```c
		calib_mgr_set_l7_progress(s_step, (uint8_t)L7_SEQ_LEN);
```

- [ ] **Step 3: 在 L7 失败时记录失败步骤的 level/submode**

修改 [calib_level7_auto.c:149-151](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorCalibration/calib_level7_auto.c#L149) FAILED 分支,在 `s_failed_step = s_step;` 之后添加:

```c
	else if (st == CALIB_STATE_FAILED)
	{
		s_failed_step = s_step;
		/* 上报失败发生在哪个子标定: 临时改 status.level/submode 为当前子项,
		 * 上位机可直接看到"L7 第 N 步的 L2.3 R 标定失败"。
		 * 注意: calib_mgr.status.level 仍为 7(L7), 此处不改 level,
		 *       仅通过 step 索引 + fail_reason 表达; 若需更细, 可扩展。*/
		calib_mgr_set_fail_reason(CALIB_FAIL_TIMEOUT); /* L7 默认超时, 子级具体原因被子级 abort 覆盖 */
		switch (CALIB_L7_STRATEGY)
		{
			...
		}
	}
```

注:L7 失败时,子级 poll 已返回 FAILED 并可能已调 `calib_mgr_set_fail_reason` 设置具体原因。L7 此处不再覆盖,让 calib_mgr 保留子级的原因。删除上面添加的 `calib_mgr_set_fail_reason(CALIB_FAIL_TIMEOUT);` 行,改为注释说明:

```c
	else if (st == CALIB_STATE_FAILED)
	{
		s_failed_step = s_step;
		/* fail_reason 已由子级 poll 设置(如 CALIB_FAIL_OUT_OF_RANGE),
		 * L7 不覆盖, 保留子级具体原因供上位机诊断 */
		switch (CALIB_L7_STRATEGY)
		{
			...
		}
	}
```

- [ ] **Step 4: 在 L7 abort 时清空 step_total**

修改 [calib_level7_auto.c:190-197](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorCalibration/calib_level7_auto.c#L190) `calib_level7_abort`,在末尾添加:

```c
static void calib_level7_abort(void)
{
	l7_abort_current_step();
	s_step = 0;
	s_failed_step = 0xFF;
	s_retry_count = 0;
	/* 清空 L7 进度, 避免 0x97 查询到残留的 step_total */
	calib_mgr_set_l7_progress(0, 0);
}
```

- [ ] **Step 5: 在 calib_level7_auto.c 添加 calib_mgr.h include**

确认 [calib_level7_auto.c:11](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorCalibration/calib_level7_auto.c#L11) 已包含 `calib_mgr.h`(已包含,无需改)。

- [ ] **Step 6: 编译验证**

Run: Keil 编译
Expected: 0 error, 0 warning

- [ ] **Step 7: Commit**

```bash
git add User/MotorCalibration/calib_level7_auto.c
git commit -m "feat(calib-L7): report sequence progress (step/step_total) for visibility"
```

---

## Task 5: 协议层 0x97 查询返回详细状态(固件)

**Files:**
- Modify: `User/Protocol/joint_proto/jm_proto.c`
- Modify: `User/Protocol/joint_proto/jm_proto_ops.c`
- Modify: `User/Protocol/joint_proto/jm_cmd_def.h`

**调研结论**(已预先调研):
- `reply_set` 是 [jm_proto.c:23](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/Protocol/joint_proto/jm_proto.c#L23) 的 static 函数,只能在 jm_proto.c 内部调用
- `_on_frame` 中 `cmd <= JmCmd.SINGLE_STEP and len(payload) >= 1` 分支([motor_client.py:217-222](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/Tools/pyqt_gui/core/motor_client.py#L217))处理 ACK:若 `payload[0]==OK` emit `ack_received(cmd)`,否则 emit `nack_received(cmd, payload[0])`
- dispatch 调用 ops 后,若返回 `JM_ERR_OK` 走 `reply_ack(p, cmd, 0)` 回单字节 ACK
- **方案**:在 dispatch 调用 ops **之前**特判 `JM_CMD_CALIB_QUERY`,直接组织 8 字节 ACK 并 return,不走 ops

- [ ] **Step 1: 在 jm_proto.c 添加 calib_mgr.h include**

在 [jm_proto.c](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/Protocol/joint_proto/jm_proto.c) 顶部 include 区添加:

```c
#include "calib_mgr.h"
```

- [ ] **Step 2: 在 jm_proto.c dispatch 中特判 CALIB_QUERY**

在 dispatch 函数开头(在通用 ops 调用之前),添加 CALIB_QUERY 特判分支:

```c
	/* ---- 标定进度查询 0x97: 返回 8 字节详细状态 ACK ----
	 * 不经过 ops, 直接读 calib_mgr 状态组织应答。
	 * ACK 载荷(8 字节):
	 *   [0] state      [1] fail_reason  [2] progress   [3] level
	 *   [4] submode    [5] step         [6] step_total [7] reserved
	 * 无论 state 为何(IDLE/RUNNING/DONE/FAILED)都返回 ACK + 详细状态。*/
	if (cmd == JM_CMD_CALIB_QUERY)
	{
		calib_status_t st = calib_mgr_get_status();
		uint8_t body[8] = {
			(uint8_t)st.state,
			(uint8_t)st.fail_reason,
			st.progress,
			st.level,
			st.submode,
			st.step,
			st.step_total,
			0u
		};
		reply_set(p, cmd, body, sizeof(body));
		return JM_ERR_OK;
	}
```

- [ ] **Step 3: 从 jm_proto_ops.c 移除 CALIB_QUERY 分支**

修改 [jm_proto_ops.c:179-188](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/Protocol/joint_proto/jm_proto_ops.c#L179),删除 `case JM_CMD_CALIB_QUERY:` 分支(改由 jm_proto.c dispatch 直接处理)。

删除整个 `case JM_CMD_CALIB_QUERY: ...` 块,保留 `case JM_CMD_CALIB_ABORT:` 分支。

- [ ] **Step 4: 更新 jm_cmd_def.h 注释**

修改 [jm_cmd_def.h:100](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/Protocol/joint_proto/jm_cmd_def.h#L100) 注释:

```c
	 * 0x97: 进度查询, 返回 8 字节详细状态 ACK (state/fail_reason/progress/level/submode/step/step_total/reserved)
```

- [ ] **Step 5: 编译验证**

Run: Keil 编译
Expected: 0 error, 0 warning

- [ ] **Step 6: 逻辑分析仪/串口抓包验证**

上位机发 `0x97`,应收到 8 字节 ACK:`[97][state][fail_reason][progress][level][submode][step][step_total][00]`

- [ ] **Step 7: Commit**

```bash
git add User/Protocol/joint_proto/jm_proto.c User/Protocol/joint_proto/jm_proto_ops.c User/Protocol/joint_proto/jm_cmd_def.h
git commit -m "feat(proto): CALIB_QUERY returns 8-byte detailed status ACK"
```

---

## Task 6: 上位机 jmproto 层解析标定状态

**Files:**
- Modify: `User/Tools/pyqt_gui/jmproto/cmd_def.py`
- Modify: `User/Tools/pyqt_gui/jmproto/__init__.py`

- [ ] **Step 1: 在 cmd_def.py 添加 CalibState / CalibFailReason 枚举**

在 [cmd_def.py](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/Tools/pyqt_gui/jmproto/cmd_def.py) `JmCmd` 类之后添加:

```python
class CalibState(IntEnum):
    """标定子状态 (0x97 ACK body[0])"""
    IDLE = 0
    RUNNING = 1
    DONE = 2
    FAILED = 3


class CalibFailReason(IntEnum):
    """标定失败原因 (0x97 ACK body[1], state=FAILED 时有效)"""
    NONE = 0
    TIMEOUT = 1              # 超时(电机卡转/无响应)
    OUT_OF_RANGE = 2         # 结果超物理范围
    DEP_NOT_MET = 3          # 前置标定未完成
    SAMPLE_ABNORMAL = 4      # 采样异常(NaN/方差过大)
    MOTOR_STUCK = 5          # 电机未转动
    OVER_CURRENT = 6         # 过流
    OVER_SPEED = 7           # 超速
    FLASH_WRITE = 8          # 持久化失败
    ABORTED = 9              # 被中止


# 失败原因中文描述(供 UI 显示)
CALIB_FAIL_REASON_CN = {
    CalibFailReason.NONE: "无失败",
    CalibFailReason.TIMEOUT: "超时",
    CalibFailReason.OUT_OF_RANGE: "结果超范围",
    CalibFailReason.DEP_NOT_MET: "前置标定未完成",
    CalibFailReason.SAMPLE_ABNORMAL: "采样异常",
    CalibFailReason.MOTOR_STUCK: "电机未转动",
    CalibFailReason.OVER_CURRENT: "过流",
    CalibFailReason.OVER_SPEED: "超速",
    CalibFailReason.FLASH_WRITE: "Flash写入失败",
    CalibFailReason.ABORTED: "被中止",
}
```

- [ ] **Step 2: 更新 cmd_def.py 中 0x97 注释**

修改 [cmd_def.py:82](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/Tools/pyqt_gui/jmproto/cmd_def.py#L82) 注释:

```python
    # 0x97: 进度查询, 返回 8 字节详细状态 ACK (state/fail_reason/progress/level/submode/step/step_total/reserved)
```

- [ ] **Step 3: 在 jmproto/__init__.py 添加 parse_calib_status**

在 [jmproto/__init__.py](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/Tools/pyqt_gui/jmproto/__init__.py) 末尾添加:

```python
def parse_calib_status(payload: bytes):
    """解析 0x97 CALIB_QUERY 的 8 字节 ACK 载荷。

    返回 dict:
      state, fail_reason, progress, level, submode, step, step_total
    payload 不足 7 字节返回 None。
    """
    if len(payload) < 7:
        return None
    return {
        'state':       payload[0],
        'fail_reason': payload[1],
        'progress':    payload[2],
        'level':       payload[3],
        'submode':     payload[4],
        'step':        payload[5],
        'step_total':  payload[6],
    }
```

- [ ] **Step 4: 验证导入**

Run: `python -c "from jmproto import parse_calib_status; print(parse_calib_status(bytes([1,0,0,2,3,2,0,0])))"`
Expected: `{'state': 1, 'fail_reason': 0, 'progress': 0, 'level': 2, 'submode': 3, 'step': 2, 'step_total': 0}`

- [ ] **Step 5: Commit**

```bash
git add User/Tools/pyqt_gui/jmproto/cmd_def.py User/Tools/pyqt_gui/jmproto/__init__.py
git commit -m "feat(gui-jmproto): add CalibState/FailReason enums and parse_calib_status"
```

---

## Task 7: 上位机 motor_client 派发 calib_status 信号

**Files:**
- Modify: `User/Tools/pyqt_gui/core/motor_client.py`

- [ ] **Step 1: 在 MotorClient 添加 calib_status_received 信号**

修改 [motor_client.py:23](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/Tools/pyqt_gui/core/motor_client.py#L23) `ack_received` 信号声明之后添加:

```python
    ack_received = pyqtSignal(int)                     # cmd
    calib_status_received = pyqtSignal(dict)           # 0x97 详细状态(dict)
```

- [ ] **Step 2: 在 _on_frame 特判 0x97 解析并 emit**

修改 [motor_client.py:207-222](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/Tools/pyqt_gui/core/motor_client.py#L207) `_on_frame`,在 NACK 处理之后、ACK 通用分支之前添加 0x97 特判:

```python
    def _on_frame(self, cmd: int, payload: bytes):
        self.raw_frame.emit(cmd, payload)

        # NACK
        if cmd == JmCmd.NACK:
            if len(payload) >= 2:
                self.nack_received.emit(payload[0], payload[1])
            return

        # CALIB_QUERY(0x97): 8 字节详细状态 ACK, 不走通用 ack_received
        if cmd == JmCmd.CALIB_QUERY:
            st = jp.parse_calib_status(payload)
            if st is not None:
                self.calib_status_received.emit(st)
            return

        # ACK 类应答 (0x00~0xB8, payload[0]==status)
        if cmd <= JmCmd.SINGLE_STEP and len(payload) >= 1:
            ...
```

- [ ] **Step 3: Commit**

```bash
git add User/Tools/pyqt_gui/core/motor_client.py
git commit -m "feat(gui-client): emit calib_status_received on 0x97 ACK"
```

---

## Task 8: 上位机标定面板显示详细状态

**Files:**
- Modify: `User/Tools/pyqt_gui/ui/main_window.py`
- Modify: `User/Tools/pyqt_gui/ui/panels/calibration_panel.py`

- [ ] **Step 1: 在 main_window.py 连接 calib_status_received 信号**

修改 [main_window.py:687-688](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/Tools/pyqt_gui/ui/main_window.py#L687),在 `nack_received` 连接之后添加:

```python
        c.ack_received.connect(self._on_ack)
        c.nack_received.connect(self._on_nack)
        c.calib_status_received.connect(self._calib_panel.on_calib_status)
```

- [ ] **Step 2: 在 calibration_panel.py 添加 on_calib_status 槽**

在 [calibration_panel.py:744](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/Tools/pyqt_gui/ui/panels/calibration_panel.py#L744) `on_nack` 方法之后添加:

```python
    def on_calib_status(self, st: dict):
        """0x97 详细状态接收(主窗口 calib_status_received 信号转发)."""
        state = st.get('state', 0)
        fail_reason = st.get('fail_reason', 0)
        progress = st.get('progress', 0)
        level = st.get('level', 0)
        submode = st.get('submode', 0)
        step = st.get('step', 0)
        step_total = st.get('step_total', 0)

        # 更新内部运行标志
        self._calib_running = (state == int(CalibState.RUNNING))

        # 构造状态文本
        state_name = {0: "IDLE", 1: "RUNNING", 2: "DONE", 3: "FAILED"}.get(state, "?")
        if state == int(CalibState.RUNNING):
            if step_total > 0:
                # L7 序列进度
                op_text = f"L7 序列 {step}/{step_total} (level={level}, sub={submode})"
            else:
                # 单步标定 step
                op_text = f"L{level}.{submode} step={step}"
            self._set_last_op(f"查询: {op_text}")
            self._add_history(
                f"[RX] CALIB_QUERY(0x97) {state_name} {op_text} progress={progress}")
            if self._link_active:
                self._poll_timer.start()
        elif state == int(CalibState.DONE):
            self._poll_timer.stop()
            if step_total > 0:
                op_text = f"L7 全流程完成 ({step_total} 步)"
            else:
                op_text = f"L{level}.{submode} 完成"
            self._set_last_op(f"查询: {op_text}")
            self._add_history(f"[RX] CALIB_QUERY(0x97) {state_name} {op_text}")
            # 标定完成 -> 主动读回结果参数(复用原 on_ack 中 0x97 分支逻辑)
            if self._config_panel is not None and self._link_active:
                param_ids = self._result_param_ids_for_current_task()
                if param_ids:
                    names = self._param_names(param_ids)
                    self._add_history(
                        f"[TX] 主动读回本次标定结果 ({len(param_ids)} 项: {names})")
                    self._config_panel.mark_fresh(param_ids)
                    self._config_panel.read_params.emit(list(param_ids))
        elif state == int(CalibState.FAILED):
            self._poll_timer.stop()
            fail_cn = CALIB_FAIL_REASON_CN.get(CalibFailReason(fail_reason), f"未知({fail_reason})")
            if step_total > 0:
                op_text = f"L7 第 {step} 步失败: {fail_cn}"
            else:
                op_text = f"L{level}.{submode} step={step} 失败: {fail_cn}"
            self._set_last_op(f"查询: {op_text}")
            self._add_history(f"[RX] CALIB_QUERY(0x97) {state_name} {op_text}")
        else:  # IDLE
            self._poll_timer.stop()
            self._set_last_op("查询: 空闲")
            self._add_history(f"[RX] CALIB_QUERY(0x97) {state_name}")

        self._refresh_status()
```

- [ ] **Step 3: 在 calibration_panel.py 导入新枚举**

修改 [calibration_panel.py:1-30](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/Tools/pyqt_gui/ui/panels/calibration_panel.py) import 区,添加:

```python
from jmproto.cmd_def import (
    JmCmd, JmErr, CalibState, CalibFailReason, CALIB_FAIL_REASON_CN,
    cmd_name, err_name_cn,
)
```

(替换原 `from jmproto.cmd_def import JmCmd, JmErr, cmd_name, err_name_cn`)

- [ ] **Step 4: 在 calibration_panel.py on_ack/on_nack 移除 0x97 分支**

修改 [calibration_panel.py:704-708](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/Tools/pyqt_gui/ui/panels/calibration_panel.py#L704) `on_ack` 中 `if cmd == JmCmd.CALIB_QUERY:` 整个分支删除(改由 `on_calib_status` 处理)。

修改 [calibration_panel.py:749-766](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/Tools/pyqt_gui/ui/panels/calibration_panel.py#L749) `on_nack` 中 `if cmd == JmCmd.CALIB_QUERY:` 整个分支删除。

注:0x97 现在永远返回 ACK(不返回 NACK),所以 on_nack 不会再收到 0x97。

- [ ] **Step 5: 验证上位机运行**

Run: `python -m pyqt_gui.main`
Expected: 启动无异常,标定面板按钮可点击

- [ ] **Step 6: Commit**

```bash
git add User/Tools/pyqt_gui/ui/main_window.py User/Tools/pyqt_gui/ui/panels/calibration_panel.py
git commit -m "feat(gui-panel): display detailed calib status (step/fail_reason/progress)"
```

---

## Task 9: 更新命令列表 CSV

**Files:**
- Modify: `User/Tools/pyqt_gui/resources/joint_motor_command_list.csv`

- [ ] **Step 1: 更新 0x97 行的 ACK 载荷定义**

修改 [joint_motor_command_list.csv:66](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/Tools/pyqt_gui/resources/joint_motor_command_list.csv#L66) 第 65 行(0x97):

原行:
```
65,标定进度查询,0x97,Host->Motor,校准,无,ACK/NACK,0,-,-,0x9700|ID,无,CONTROL_MODE_CALIB_QUERY ACK=完成 NACK(0x0A)=进行中 NACK(0x03)=未标定
```

改为:
```
65,标定进度查询,0x97,Host->Motor,校准,无,ACK,8,struct,8,0x9700|ID,state:u8|fail_reason:u8|progress:u8|level:u8|submode:u8|step:u8|step_total:u8|reserved:u8,CONTROL_MODE_CALIB_QUERY ACK=8字节详细状态(state/fail_reason/progress/level/submode/step/step_total/reserved)
```

字段说明:
- 第 7 列 `ACK`:只返回 ACK(不再有 NACK)
- 第 8 列 `8`:ACK 载荷 8 字节
- 第 9 列 `struct`:结构化数据
- 第 10 列 `8`:字节数
- 第 12 列:字段定义 `state:u8|fail_reason:u8|...`
- 第 13 列:说明

- [ ] **Step 2: Commit**

```bash
git add User/Tools/pyqt_gui/resources/joint_motor_command_list.csv
git commit -m "docs(csv): update CALIB_QUERY ACK to 8-byte detailed status"
```

---

## Task 10: 端到端验证

**Files:**
- 无修改,仅测试

- [ ] **Step 1: 单步标定 step 可见性验证**

上位机发送 `0x91 03`(L2.3 R 标定),在标定进行中周期发送 `0x97` 查询:

Expected ACK(8 字节):
- 启动初期:`01 00 00 02 03 01 00 00`(RUNNING, step=1 等待稳态)
- 采样阶段:`01 00 00 02 03 02 00 00`(RUNNING, step=2 采样 id)
- 完成后:`02 00 64 02 03 03 00 00`(DONE, step=3)

上位机标定面板"最近操作"应显示:`查询: L2.3 step=2`

- [ ] **Step 2: 单步标定失败原因可见性验证**

故意制造失败(如断开电机线),发 `0x91 03` 后查询:

Expected ACK:`03 04 00 02 03 02 00 00`(FAILED, fail_reason=SAMPLE_ABNORMAL, step=2)

上位机标定面板"最近操作"应显示:`查询: L2.3 step=2 失败: 采样异常`

- [ ] **Step 3: L7 序列进度可见性验证**

上位机发 `0x96 01`(L7 一键标定),周期查询 `0x97`:

Expected ACK 序列:
- 启动:`01 00 00 07 01 00 09 00`(RUNNING, step=0, step_total=9)
- 第 3 步(L2.3 R):`01 00 00 07 01 02 09 00`(step=2, step_total=9)
- 完成:`02 00 64 07 01 09 09 00`(DONE, step=9=step_total)

上位机标定面板"最近操作"应显示:`查询: L7 序列 2/9 (level=2, sub=3)`

- [ ] **Step 4: L7 失败步骤可见性验证**

故意在 L7 某步制造失败,查询:

Expected ACK:`03 04 00 07 01 02 09 00`(FAILED, fail_reason=子级设置的原因, step=2)

上位机标定面板"最近操作"应显示:`查询: L7 第 2 步失败: 采样异常`

- [ ] **Step 5: Commit 验证记录**

```bash
git log --oneline -9
```

Expected: 看到 9 个 commit,每个对应一个 Task

---

## 自检

### Spec 覆盖
- 问题 1(单步标定卡死环节):Task 1 接口 + Task 2/3 L2/L3 上报 + Task 5 协议层 + Task 6-8 上位机解析显示 → ✅
- 问题 2(一键标定步骤):Task 1 接口 + Task 4 L7 上报 + Task 5 协议层 + Task 6-8 上位机显示 → ✅
- CSV 更新:Task 9 → ✅
- 端到端验证:Task 10 → ✅

### Placeholder 扫描
- 无 TBD/TODO,所有 step 有具体代码
- Task 3 Step 2/3 的"若有采样 NaN/异常分支"是条件性的,需根据实际代码判断——已要求先 Read 文件确认

### 类型一致性
- 固件 `calib_mgr_set_step(uint8_t step)` ↔ 上位机 `st['step']`
- 固件 `calib_fail_reason_e` 枚举值 ↔ 上位机 `CalibFailReason` 枚举值(0-9 一一对应)
- 固件 `calib_status_t` 字段顺序 ↔ 协议层 ACK body[0-7] ↔ 上位机 `parse_calib_status` 解析
- CSV 字段定义 `state:u8|fail_reason:u8|...` ↔ 实际协议一致

### 风险点
1. **协议层特判位置**:Task 5 Step 2 的 `if (cmd == JM_CMD_CALIB_QUERY)` 必须在 dispatch 调用 ops **之前**,否则 ops 中已删除的 CALIB_QUERY 分支会导致 fall-through。需确认 dispatch 的控制流
2. **L7 失败时 fail_reason 保留子级原因**:子级 poll 返回 FAILED 前已调 `calib_mgr_set_fail_reason`,L7 不覆盖。但若子级 abort 清理了状态,可能丢失。需在 Task 4 Step 3 注意顺序
3. **step 字段语义双重含义**:单步标定时是 level 模块内部 step,L7 时是序列索引。已通过 `step_total` 区分(L7 时 step_total>0,单步时=0)
4. **上位机 on_ack/on_nack 移除 0x97 分支后**:0x97 现在永远返回 ACK,`motor_client._on_frame` 中 0x97 特判在通用 ACK 分支之前,不会 emit `ack_received`,所以 `on_ack` 不会被调用。但若固件异常返回非 8 字节(如 1 字节),`parse_calib_status` 返回 None,`calib_status_received` 不 emit,此时 0x97 既不触发 `on_ack` 也不触发 `on_calib_status`,面板无响应——可接受(固件应保证返回 8 字节)
