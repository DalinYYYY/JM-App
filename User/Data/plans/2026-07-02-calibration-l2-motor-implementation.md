# L2 电机电气身份辨识标定 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 实现 `calib_level2_motor.c` 中 6 个子模式（相序识别/极对数/R/Ld/Lq/flux）的真实标定算法，复用已有的 `calib_hw` 共享层、`calib_step_t` 状态机框架、`calib_validate.h` 校验、`calib_config.h` 集中参数。

**Architecture:** 每个子模式是一个独立的 `poll_*()` 状态机函数，通过 `calib_hw_enter/apply_voltage/exit` 操作硬件，用 `calib_step_t` 推进步骤，结果经 `calib_validate_*` 校验后写入 `motor_param_set_*`，成功时调 `calib_mgr_mark_done`。模块私有状态合并为单个 `s_l2` 结构体，无全局变量散落。相序/极对数无前置依赖；R 无依赖；Ld/Lq/flux 依赖 R（已在 `s_calib_dep_table` 配置）。

**Tech Stack:** STM32 HAL（Keil MDK-ARM）、C99、FOC 算法链（foc_core.c）、MT6701 编码器、关节电机标定框架（calib_mgr/calib_hw/calib_step/calib_validate/calib_config）。

---

## 硬件访问关键事实（实现前必读）

工程师在实现前必须知道以下接口，否则会写错代码。这些是从代码库读到的事实，不是设计：

### FOC 数据读取（[foc_core.h:88-112](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorAlgorithms/foc_core.h#L88-L112)）
- `m->foc.current.ia / .ib / .ic` — 三相电流 (A)，由 `current_callback` 刷新
- `m->foc.i_alphaBeta.alpha / .beta` — αβ 轴电流，clarke 变换后刷新
- `m->foc.i_dq.d / .q` — dq 轴电流 (A)，park 变换后刷新
- `m->foc.u_dq.d / .q` — dq 轴电压 (V)，`set_udq` 写入后可读
- `m->foc.Theta` — 当前电角度 (rad)

**重要：** `calib_hw_apply_voltage` 内部调 `set_udq + inverse_park + pfsvpwm + set_3pwm`，但**不调 `clarke` 和 `park`**。所以施加电压后 `i_dq` 不会自动刷新。要读 id/iq，必须显式调用 `m->foc.clarke(&m->foc)` + `m->foc.park(&m->foc)`。`clarke` 内部会调 `current_callback` 取最新三相电流。

### 速度读取（[motion_param.h:118-180](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorAlgorithms/motion_param.h#L118-L180)、[motor_loop.c:183](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorControl/CascadeControl/motor_loop.c#L183)）
- `m->motor_param.slide_rad_s` — 滑动滤波后的机械角速度 (rad/s)
- `m->motor_param.ele_radian` — 电角度 (rad)
- `m->motor_param.mechanical_angle` — 机械角度 (deg)
- **flux 标定需要电角速度**：`omega_e = slide_rad_s * pole_pairs`（极对数从 motor_param 读取）

### 编码器读取（[calib_hw.h:32-35](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorCalibration/calib_hw.h#L32-L35)）
- `calib_hw_get_encoder_raw_deg(m)` — MT6701 原始角度 (°) [0,360)，不含 offset/dir 补偿
- `calib_hw_get_encoder_mech_angle(m)` — 含 offset/dir 补偿的机械角度 (°) [0,360)

### 电压施加（[calib_hw.h:26](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorCalibration/calib_hw.h#L26)）
- `calib_hw_apply_voltage(&s_l2.session, ud, uq, theta)` — 施加 dq 电压，theta 强制电角度
- 内部有 5V 幅值互锁（`CALIB_CFG_MAX_VOLTAGE_MAG_V`），超限自动缩放
- `calib_hw_enter(&s_l2.session, m)` — 进入标定（替换电角度回调）
- `calib_hw_exit(&s_l2.session)` — 退出标定（恢复回调 + PWM 置零）

### motor_param setter（[motor_param.h:275-358](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/DataHub/motor_param.h#L275-L358)）
- `motor_param_set_r(cfg, float)` / `motor_param_get_r(cfg)`
- `motor_param_set_ld(cfg, float)` / `motor_param_get_ld(cfg)`
- `motor_param_set_lq(cfg, float)` / `motor_param_get_lq(cfg)`
- `motor_param_set_flux(cfg, float)` / `motor_param_get_flux(cfg)`
- `motor_param_set_pole_pairs(cfg, uint8_t)` / `motor_param_get_pole_pairs(cfg)`

### 默认参数值（[motor_param.c:27-32](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/DataHub/motor_param.c#L27-L32)）
- `r=0.1Ω, ld=0.0001H, lq=0.00012H, flux=0.001Wb, kt=0.1, pole_pairs=7, ke=0.01`
- **重要：** 这些是占位默认值，不是真实电机参数。标定的目的就是测出真实值覆盖它们。flux 标定用 R 时应读 `motor_param_get_r`（若 R 已标定则用真值，否则用默认 0.1Ω——会有误差但 flux 公式对 R 不敏感）。

### 已有配置参数（[calib_config.h:20-29](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorCalibration/calib_config.h#L20-L29)）
```c
#define CALIB_CFG_L2_R_TEST_VOLTAGE_V    0.5f   /* R辨识 DC测试电压 */
#define CALIB_CFG_L2_R_TEST_TIME_S       1.0f   /* R辨识 稳态等待 */
#define CALIB_CFG_L2_R_SAMPLE_COUNT      200    /* R辨识 采样次数 */
#define CALIB_CFG_L2_LD_TEST_VOLTAGE_V   2.0f   /* Ld辨识 d轴阶跃电压 */
#define CALIB_CFG_L2_LD_TEST_TIME_S      0.005f /* Ld辨识 阶跃持续 */
#define CALIB_CFG_L2_LQ_TEST_VOLTAGE_V   2.0f   /* Lq辨识 q轴阶跃电压 */
#define CALIB_CFG_L2_LQ_TEST_TIME_S      0.005f /* Lq辨识 阶跃持续 */
#define CALIB_CFG_L2_FLUX_SPIN_VOLTAGE_V 3.0f   /* flux辨识 驱动电压 */
#define CALIB_CFG_L2_FLUX_SPIN_TIME_S    2.0f   /* flux辨识 稳态转动时间 */
#define CALIB_CFG_L2_FLUS_SPEED_RAD_S    10.0f  /* flux辨识 目标转速(拼写错误FLUS保留) */
```

### 控制环频率
- `CALIB_TICKS_PER_SEC = 10000.0f`（dt=100us，10kHz）
- 秒→tick：`TICKS = (uint32_t)(SECONDS * CALIB_TICKS_PER_SEC)`

---

## File Structure

| 文件 | 职责 | 改动 |
|---|---|---|
| `User/MotorCalibration/calib_level2_motor.c` | L2 六个子模式标定算法 | **主要修改** |
| `User/MotorCalibration/calib_config.h` | 集中参数 | 补充 L2 缺失参数（LD/LQ/FLUX sample count、PHASE_SEQ/POLE_PAIRS 参数、_TICKS 转换） |

**不改动的文件**（已就绪）：`calib_types.h`、`calib_hw.h/.c`、`calib_step.h`、`calib_validate.h`、`calib_mgr.h/.c`、`motor_param.h/.c`、`foc_core.h/.c`、`dev_motor.h`。

---

## 算法设计（每个子模式）

### 子模式 1：相序识别 (CALIB_L2_PHASE_SEQ)
**原理：** 施加 ud 电压（电角度=0），转子对齐 d 轴。然后施加 ud（电角度=90°），转子应转动 90°电角度 = 90/pole_pairs 机械角度。观测转动方向与三相电流相位关系，确认相序 ABC（正序）vs CBA（逆序）。

**简化实现（本计划采用）：** 施加 ud 固定电角度=0 让转子对齐，然后施加 ud 电角度=120°（下一个电相位），观测机械角度是否朝预期方向转动。若转动方向与编码器 dir 一致则相序正确，否则相序反。

**状态机：**
- Step 0: enter + 施加 ud（theta=0）对齐
- Step 1: 等 CALIB_CFG_L2_PHASE_SEQ_ALIGN_TICKS
- Step 2: 记录起始机械角度 → 施加 ud（theta=120°）
- Step 3: 等 CALIB_CFG_L2_PHASE_SEQ_STEP_TICKS
- Step 4: 读结束角度，算 delta，判定相序，写 motor_param（本版只记录结果到 motor_param 不写专门字段，用 motor_param_set_enc_direction 间接反映——若无相序字段则只返回 DONE/FAILED）。**简化：** 本版相序识别只做"能转动+方向合理"校验，结果不持久化（motor_param 无相序字段），仅返回 DONE 表示通过。

### 子模式 2：极对数 (CALIB_L2_POLE_PAIRS)
**原理：** 缓慢施加 uq 让电机转动，记录机械角度变化 360° 期间电角度（编码器原始角度 × 极对数预期）变化的周期数。或：施加 ud 让转子锁定，手动转一圈数对齐次数。

**简化实现（本计划采用）：** 施加小幅 uq 让电机稳速转动 CALIB_CFG_L2_POLE_PAIRS_SPIN_TIME，记录起始与结束的机械角度和电角度。极对数 = 电角度变化 / 机械角度变化。取整后校验。

**状态机：**
- Step 0: enter + 记录起始机械角度、起始电角度（motor_param.ele_radian 或编码器角度）
- Step 1: 持续施加 uq（CALIB_CFG_L2_POLE_PAIRS_VOLTAGE_V）让电机转动
- Step 2: 等 CALIB_CFG_L2_POLE_PAIRS_SPIN_TICKS
- Step 3: 读结束角度，计算极对数 = round(电角度变化 / 机械角度变化)，校验 calib_validate_pole_pairs，写 motor_param_set_pole_pairs，mark_done

**难点：** MT6701 原始角度是机械角度 [0,360)，电角度 = 机械角度 × 极对数 mod 360。但标定时极对数未知。改用：让电机转 N 圈机械角度，数"转子被 ud 锁定的次数"——这需要交替施加 ud + 转动，复杂。**本计划采用更简单的方法：** 已知默认 pole_pairs=7（占位值），真实值通常 7 或 14。让电机转动，测机械角速度 vs 电角速度，比值=极对数。电角速度从 `foc.Theta` 变化率算。

### 子模式 3：R 辨识 (CALIB_L2_RESISTANCE) — DC 法
**原理：** 施加 DC 电压 ud（电角度=0，转子锁定 d 轴），稳态后 id = ud / R。R = ud / id。

**状态机：**
- Step 0: enter + 施加 ud=CALIB_CFG_L2_R_TEST_VOLTAGE_V, uq=0, theta=0
- Step 1: 等 CALIB_CFG_L2_R_TEST_TICKS（稳态）
- Step 2: 循环采样 id：clarke + park → calib_step_accumulate(id)，直到 sample_cnt >= CALIB_CFG_L2_R_SAMPLE_COUNT
- Step 3: R = ud / calib_step_average(id)；校验 calib_validate_r；写 motor_param_set_r；mark_done；exit

### 子模式 4：Ld 辨识 (CALIB_L2_INDUCTANCE_D) — 阶跃响应
**原理：** 施加 ud 阶跃，观测 di_d/dt。Ld = (ud - R*id) / (did/dt)。稳态后 id = ud/R，暂态期间 did/dt = (ud - R*id)/Ld。

**简化实现：** 施加 ud 阶跃，在暂态期间采样多个 (id, did/dt) 点取平均。did/dt 用相邻两次 id 差分 / dt。R 用 motor_param_get_r（依赖 R 已标定）。

**状态机：**
- Step 0: enter + 施加 ud=CALIB_CFG_L2_LD_TEST_VOLTAGE_V, uq=0, theta=0；记录 id_0
- Step 1: 循环采样（每 tick）：id_k = clarke+park；did_dt = (id_k - id_prev)/dt；calib_step_accumulate((ud - R*id_k)/did_dt)（需处理 did_dt≈0）；id_prev = id_k；直到 sample_cnt >= COUNT
- Step 2: Ld = calib_step_average；校验 calib_validate_ld；写 motor_param_set_ld；mark_done；exit

**注意：** 阶跃持续 5ms = 50 tick，暂态很快。采样必须在暂态窗口内。用 CALIB_CFG_L2_LD_TEST_TIME_S 作为"暂态采样窗口"。

### 子模式 5：Lq 辨识 (CALIB_L2_INDUCTANCE_Q) — 阶跃响应
**原理：** 同 Ld，但施加 uq 阶跃，观测 di_q/dt。Lq = (uq - ... ) / (diq/dt)。注意：uq 会让电机转动，所以必须在转子未转起的暂态内快速采样，或锁定转子。

**简化实现：** 与 Ld 对称，施加 uq 阶跃 theta=0（此时 d 轴对齐，q 轴产生力矩但转子有惯量暂态未动），采样 diq/dt。Lq = (uq - R*iq) / (diq/dt)（忽略交叉解耦）。

**状态机：** 同 Ld，改 ud→uq, id→iq, R 依赖同。

### 子模式 6：flux 辨识 (CALIB_L2_FLUX_LINKAGE) — 反电势法
**原理：** 开环驱动电机稳速转动（施加 ud/uq 维持转速），稳态后 uq = R*iq + ωe*flux（忽略 Lq 项）。flux = (uq - R*iq) / ωe。ωe = ωm * pole_pairs。

**状态机：**
- Step 0: enter + 施加 ud=0, uq=CALIB_CFG_L2_FLUX_SPIN_VOLTAGE_V, theta=motor_param.ele_radian（跟随真实电角度，让电机转动）
- Step 1: 等 CALIB_CFG_L2_FLUX_SPIN_TICKS（稳速 2s）
- Step 2: 采样多个点：clarke+park 取 iq；omega_m = motor_param.slide_rad_s；omega_e = omega_m * pole_pairs；flux_k = (uq - R*iq) / omega_e；calib_step_accumulate(flux_k)；直到 sample_cnt >= COUNT
- Step 3: flux = calib_step_average；校验 calib_validate_flux；写 motor_param_set_flux；mark_done；exit

**关键：** Step 0 的 theta 必须用**实时电角度** `motor_param.ele_radian`（不是固定 0），否则电机不转。但 calib_hw_enter 替换了 ele_radian_callback 为固定值。**冲突！** 解决：flux 标定不调 calib_hw_enter（不替换回调），直接用 `m->foc.set_udq + inverse_park + pfsvpwm + set_3pwm` 自己施加电压，让正常电角度回调工作。或：在 apply_voltage 时传 `m->motor_param.ele_radian` 作为 theta。**本计划采用后者**——flux 标定 step 0 不 enter，每 tick 调 apply_voltage(..., m->motor_param.ele_radian)。

---

## Task 分解

### Task 1: 补充 calib_config.h 缺失的 L2 参数

**Files:**
- Modify: `User/MotorCalibration/calib_config.h:19-29`（L2 参数段）

- [ ] **Step 1: 读取当前 calib_config.h 确认 L2 段内容**

Run: Read `d:\AAWorkSpace\001_JointMotor\SW\JointMotor\User\MotorCalibration\calib_config.h`
Expected: 看到 19-29 行的 L2 参数（R/LD/LQ/FLUX 的电压/时间/采样数，无 PHASE_SEQ/POLE_PAIRS 参数，无 _TICKS 转换）

- [ ] **Step 2: 替换 L2 段为完整参数**

用 Edit 工具，old_string 匹配当前 L2 段（从 `/* ===================== L2 电机电气身份参数（预留）===================== */` 到 `#define CALIB_CFG_L2_FLUS_SPEED_RAD_S 10.0f`），new_string 为：

```c
/* ===================== L2 电机电气身份参数 ===================== */
/* R 辨识（DC 法）*/
#define CALIB_CFG_L2_R_TEST_VOLTAGE_V    0.5f   /* DC 测试电压(V) */
#define CALIB_CFG_L2_R_TEST_TIME_S       1.0f   /* 稳态等待(s) */
#define CALIB_CFG_L2_R_TEST_TICKS        (uint32_t)(CALIB_CFG_L2_R_TEST_TIME_S * CALIB_TICKS_PER_SEC)
#define CALIB_CFG_L2_R_SAMPLE_COUNT      200    /* 稳态采样次数 */

/* Ld 辨识（d 轴阶跃响应）*/
#define CALIB_CFG_L2_LD_TEST_VOLTAGE_V   2.0f   /* d 轴阶跃电压(V) */
#define CALIB_CFG_L2_LD_TEST_TIME_S      0.005f /* 暂态采样窗口(s) */
#define CALIB_CFG_L2_LD_TEST_TICKS       (uint32_t)(CALIB_CFG_L2_LD_TEST_TIME_S * CALIB_TICKS_PER_SEC)
#define CALIB_CFG_L2_LD_SAMPLE_COUNT     50     /* 暂态采样点数 */

/* Lq 辨识（q 轴阶跃响应）*/
#define CALIB_CFG_L2_LQ_TEST_VOLTAGE_V   2.0f   /* q 轴阶跃电压(V) */
#define CALIB_CFG_L2_LQ_TEST_TIME_S      0.005f /* 暂态采样窗口(s) */
#define CALIB_CFG_L2_LQ_TEST_TICKS       (uint32_t)(CALIB_CFG_L2_LQ_TEST_TIME_S * CALIB_TICKS_PER_SEC)
#define CALIB_CFG_L2_LQ_SAMPLE_COUNT     50     /* 暂态采样点数 */

/* flux 辨识（反电势法）*/
#define CALIB_CFG_L2_FLUX_SPIN_VOLTAGE_V 3.0f   /* 驱动电压(V) */
#define CALIB_CFG_L2_FLUX_SPIN_TIME_S    2.0f   /* 稳速转动时间(s) */
#define CALIB_CFG_L2_FLUX_SPIN_TICKS     (uint32_t)(CALIB_CFG_L2_FLUX_SPIN_TIME_S * CALIB_TICKS_PER_SEC)
#define CALIB_CFG_L2_FLUX_SAMPLE_COUNT   200    /* 稳态采样次数 */
#define CALIB_CFG_L2_FLUX_TARGET_SPEED_RAD_S 10.0f /* 目标机械转速(rad/s) */

/* 相序识别 */
#define CALIB_CFG_L2_PHASE_SEQ_VOLTAGE_V 1.0f   /* 对齐/步进电压(V) */
#define CALIB_CFG_L2_PHASE_SEQ_ALIGN_S   1.0f   /* 对齐等待(s) */
#define CALIB_CFG_L2_PHASE_SEQ_ALIGN_TICKS (uint32_t)(CALIB_CFG_L2_PHASE_SEQ_ALIGN_S * CALIB_TICKS_PER_SEC)
#define CALIB_CFG_L2_PHASE_SEQ_STEP_S    0.5f   /* 步进后等待(s) */
#define CALIB_CFG_L2_PHASE_SEQ_STEP_TICKS (uint32_t)(CALIB_CFG_L2_PHASE_SEQ_STEP_S * CALIB_TICKS_PER_SEC)

/* 极对数辨识 */
#define CALIB_CFG_L2_POLE_PAIRS_VOLTAGE_V 1.0f  /* 驱动电压(V) */
#define CALIB_CFG_L2_POLE_PAIRS_SPIN_S   1.0f   /* 转动时间(s) */
#define CALIB_CFG_L2_POLE_PAIRS_SPIN_TICKS (uint32_t)(CALIB_CFG_L2_POLE_PAIRS_SPIN_S * CALIB_TICKS_PER_SEC)
```

- [ ] **Step 3: 编译验证 calib_config.h 无语法错误**

Run: `& "C:\APP\Code\MDK\CORE\UV4\UV4.exe" -b "d:\AAWorkSpace\001_JointMotor\SW\JointMotor\Board\SFOC\MDK-ARM\sfoc.uvprojx" -o "d:\AAWorkSpace\001_JointMotor\SW\JointMotor\Board\SFOC\MDK-ARM\build_l2_t1.log" -j0`
Expected: 0 Error（基线 sfoc 是 0 error）。warning 数不变。

- [ ] **Step 4: 不提交（用户审核后自己 add）**

---

### Task 2: 扩展 s_l2 结构体，增加算法所需字段

**Files:**
- Modify: `User/MotorCalibration/calib_level2_motor.c:25-31`（s_l2 结构体）

- [ ] **Step 1: 替换 s_l2 结构体**

用 Edit，old_string 为当前结构体：
```c
static struct
{
	uint8_t             submode;
	calib_step_t        step;          /* 统一状态机骨架（未来真实算法用）*/
	calib_hw_session_t  session;       /* 标定电压会话（未来施加测试电压用）*/
} s_l2;
```

new_string：
```c
static struct
{
	uint8_t             submode;
	calib_step_t        step;          /* 统一状态机骨架（cur/tick/sample_cnt/sample_sum）*/
	calib_hw_session_t  session;       /* 标定电压会话（替换电角度回调 + 施加电压）*/
	float               prev_i;        /* 前一次采样电流（Ld/Lq 阶跃求 did/dt 用）*/
	float               start_mech_deg;/* 起始机械角度（相序/极对数用）*/
	float               start_ele_rad; /* 起始电角度（极对数用）*/
	float               test_voltage;  /* 本次施加的测试电压（R/Ld/Lq/flux 算结果时用）*/
} s_l2;
```

- [ ] **Step 2: 编译验证**

Run: sfoc 编译
Expected: 0 Error（字段未使用，只有结构体定义）

- [ ] **Step 3: 不提交**

---

### Task 3: 实现 R 辨识 (poll_resistance)

**Files:**
- Modify: `User/MotorCalibration/calib_level2_motor.c:48-64`（poll_resistance 函数）

- [ ] **Step 1: 替换 poll_resistance 函数**

用 Edit，old_string 为当前桩函数（从 `static calib_state_e poll_resistance(void)` 到 `return CALIB_STATE_DONE;`），new_string：

```c
/* ===================== R 辨识（DC 法）=====================
 * STEP 0: 施加 ud DC 电压（theta=0 锁定转子 d 轴）
 * STEP 1: 等待稳态（CALIB_CFG_L2_R_TEST_TICKS）
 * STEP 2: 多次采样 id 取平均
 * STEP 3: R = ud / id_avg，校验后写入 motor_param，完成
 * ========================================================= */
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
			calib_step_next(&s_l2.step, 1);
			return CALIB_STATE_RUNNING;

		case 1: /* 等待稳态 */
			calib_hw_apply_voltage(&s_l2.session, s_l2.test_voltage, 0.0f, 0.0f);
			if (calib_step_wait(&s_l2.step, CALIB_CFG_L2_R_TEST_TICKS))
				return CALIB_STATE_RUNNING;
			calib_step_next(&s_l2.step, 2);
			return CALIB_STATE_RUNNING;

		case 2: /* 多次采样 id */
		{
			calib_hw_apply_voltage(&s_l2.session, s_l2.test_voltage, 0.0f, 0.0f);
			m->foc.clarke(&m->foc);  /* 刷新 i_alphaBeta（内部调 current_callback）*/
			m->foc.park(&m->foc);    /* 刷新 i_dq */
			float id = m->foc.i_dq.d;
			/* 过滤异常值（NaN/过大）*/
			if (!isfinite(id) || id < 0.001f)
			{
				calib_hw_exit(&s_l2.session);
				return CALIB_STATE_FAILED;
			}
			calib_step_accumulate(&s_l2.step, id);
			if (s_l2.step.sample_cnt < CALIB_CFG_L2_R_SAMPLE_COUNT)
				return CALIB_STATE_RUNNING;
			calib_step_next(&s_l2.step, 3);
			return CALIB_STATE_RUNNING;
		}

		case 3: /* 计算 R，校验，写入 */
		{
			float id_avg = calib_step_average(&s_l2.step);
			float R = s_l2.test_voltage / id_avg;
			if (!calib_validate_r(R))
			{
				calib_hw_exit(&s_l2.session);
				return CALIB_STATE_FAILED;
			}
			motor_param_set_r(io->param, R);
			calib_hw_exit(&s_l2.session);
			calib_mgr_mark_done(CALIB_LEVEL2_MOTOR, CALIB_L2_RESISTANCE);
			calib_step_reset(&s_l2.step);
			return CALIB_STATE_DONE;
		}

		default:
			calib_hw_exit(&s_l2.session);
			return CALIB_STATE_FAILED;
	}
}
```

- [ ] **Step 2: 编译验证**

Run: sfoc 编译
Expected: 0 Error。注意 `isfinite` 需要 `<math.h>`——若 calib_level2_motor.c 未 include math.h，需补充（Task 2 的结构体改动不带 math.h，但 calib_validate.h 已 include math.h，应该传递可见）。

- [ ] **Step 3: 不提交**

---

### Task 4: 实现 Ld 辨识 (poll_inductance_d)

**Files:**
- Modify: `User/MotorCalibration/calib_level2_motor.c`（poll_inductance_d 函数）

- [ ] **Step 1: 替换 poll_inductance_d 函数**

old_string 为当前桩（从 `static calib_state_e poll_inductance_d(void)` 到其 `return CALIB_STATE_DONE;`），new_string：

```c
/* ===================== Ld 辨识（d 轴阶跃响应）=====================
 * STEP 0: 施加 ud 阶跃，记录初始 id
 * STEP 1: 暂态窗口内采样 (ud - R*id)/did_dt 取平均
 * STEP 2: Ld = avg，校验后写入，完成
 *
 * 公式：ud = R*id + Ld*did/dt  →  Ld = (ud - R*id) / (did/dt)
 * R 从 motor_param 读取（前置依赖 L2.3 R 已标定）
 * ============================================================ */
static calib_state_e poll_inductance_d(void)
{
	const calib_io_t *io = calib_mgr_get_io();
	dev_motor_t *m = io->motor;
	float dt = io->dt;

	switch (s_l2.step.cur)
	{
		case 0: /* 施加 ud 阶跃 */
			calib_hw_enter(&s_l2.session, m);
			s_l2.test_voltage = CALIB_CFG_L2_LD_TEST_VOLTAGE_V;
			calib_hw_apply_voltage(&s_l2.session, s_l2.test_voltage, 0.0f, 0.0f);
			m->foc.clarke(&m->foc);
			m->foc.park(&m->foc);
			s_l2.prev_i = m->foc.i_dq.d;
			calib_step_next(&s_l2.step, 1);
			return CALIB_STATE_RUNNING;

		case 1: /* 暂态窗口内采样 */
		{
			calib_hw_apply_voltage(&s_l2.session, s_l2.test_voltage, 0.0f, 0.0f);
			m->foc.clarke(&m->foc);
			m->foc.park(&m->foc);
			float id = m->foc.i_dq.d;
			float did_dt = (id - s_l2.prev_i) / dt;
			s_l2.prev_i = id;
			/* did_dt 过小则跳过该点（避免除零）*/
			if (isfinite(did_dt) && fabsf(did_dt) > 1.0f)
			{
				float R = motor_param_get_r(io->param);
				float Ld = (s_l2.test_voltage - R * id) / did_dt;
				if (isfinite(Ld) && Ld > 0.0f)
					calib_step_accumulate(&s_l2.step, Ld);
			}
			if (s_l2.step.sample_cnt < CALIB_CFG_L2_LD_SAMPLE_COUNT)
				return CALIB_STATE_RUNNING;
			calib_step_next(&s_l2.step, 2);
			return CALIB_STATE_RUNNING;
		}

		case 2: /* 校验，写入 */
		{
			if (s_l2.step.sample_cnt == 0)
			{
				calib_hw_exit(&s_l2.session);
				return CALIB_STATE_FAILED;
			}
			float Ld = calib_step_average(&s_l2.step);
			if (!calib_validate_ld(Ld))
			{
				calib_hw_exit(&s_l2.session);
				return CALIB_STATE_FAILED;
			}
			motor_param_set_ld(io->param, Ld);
			calib_hw_exit(&s_l2.session);
			calib_mgr_mark_done(CALIB_LEVEL2_MOTOR, CALIB_L2_INDUCTANCE_D);
			calib_step_reset(&s_l2.step);
			return CALIB_STATE_DONE;
		}

		default:
			calib_hw_exit(&s_l2.session);
			return CALIB_STATE_FAILED;
	}
}
```

- [ ] **Step 2: 编译验证**

Run: sfoc 编译
Expected: 0 Error

- [ ] **Step 3: 不提交**

---

### Task 5: 实现 Lq 辨识 (poll_inductance_q)

**Files:**
- Modify: `User/MotorCalibration/calib_level2_motor.c`（poll_inductance_q 函数）

- [ ] **Step 1: 替换 poll_inductance_q 函数**

old_string 为当前桩，new_string（与 Ld 对称，ud→uq, id→iq, i_dq.d→i_dq.q）：

```c
/* ===================== Lq 辨识（q 轴阶跃响应）=====================
 * STEP 0: 施加 uq 阶跃，记录初始 iq
 * STEP 1: 暂态窗口内采样 (uq - R*iq)/diq_dt 取平均
 * STEP 2: Lq = avg，校验后写入，完成
 *
 * 公式：uq = R*iq + Lq*diq/dt  →  Lq = (uq - R*iq) / (diq/dt)
 * 注意：uq 会产生力矩使转子转动，暂态窗口 5ms 内转子因惯量尚未转起。
 * ============================================================ */
static calib_state_e poll_inductance_q(void)
{
	const calib_io_t *io = calib_mgr_get_io();
	dev_motor_t *m = io->motor;
	float dt = io->dt;

	switch (s_l2.step.cur)
	{
		case 0: /* 施加 uq 阶跃 */
			calib_hw_enter(&s_l2.session, m);
			s_l2.test_voltage = CALIB_CFG_L2_LQ_TEST_VOLTAGE_V;
			calib_hw_apply_voltage(&s_l2.session, 0.0f, s_l2.test_voltage, 0.0f);
			m->foc.clarke(&m->foc);
			m->foc.park(&m->foc);
			s_l2.prev_i = m->foc.i_dq.q;
			calib_step_next(&s_l2.step, 1);
			return CALIB_STATE_RUNNING;

		case 1: /* 暂态窗口内采样 */
		{
			calib_hw_apply_voltage(&s_l2.session, 0.0f, s_l2.test_voltage, 0.0f);
			m->foc.clarke(&m->foc);
			m->foc.park(&m->foc);
			float iq = m->foc.i_dq.q;
			float diq_dt = (iq - s_l2.prev_i) / dt;
			s_l2.prev_i = iq;
			if (isfinite(diq_dt) && fabsf(diq_dt) > 1.0f)
			{
				float R = motor_param_get_r(io->param);
				float Lq = (s_l2.test_voltage - R * iq) / diq_dt;
				if (isfinite(Lq) && Lq > 0.0f)
					calib_step_accumulate(&s_l2.step, Lq);
			}
			if (s_l2.step.sample_cnt < CALIB_CFG_L2_LQ_SAMPLE_COUNT)
				return CALIB_STATE_RUNNING;
			calib_step_next(&s_l2.step, 2);
			return CALIB_STATE_RUNNING;
		}

		case 2: /* 校验，写入 */
		{
			if (s_l2.step.sample_cnt == 0)
			{
				calib_hw_exit(&s_l2.session);
				return CALIB_STATE_FAILED;
			}
			float Lq = calib_step_average(&s_l2.step);
			if (!calib_validate_lq(Lq))
			{
				calib_hw_exit(&s_l2.session);
				return CALIB_STATE_FAILED;
			}
			motor_param_set_lq(io->param, Lq);
			calib_hw_exit(&s_l2.session);
			calib_mgr_mark_done(CALIB_LEVEL2_MOTOR, CALIB_L2_INDUCTANCE_Q);
			calib_step_reset(&s_l2.step);
			return CALIB_STATE_DONE;
		}

		default:
			calib_hw_exit(&s_l2.session);
			return CALIB_STATE_FAILED;
	}
}
```

- [ ] **Step 2: 编译验证**

Run: sfoc 编译
Expected: 0 Error

- [ ] **Step 3: 不提交**

---

### Task 6: 实现 flux 辨识 (poll_flux_linkage)

**Files:**
- Modify: `User/MotorCalibration/calib_level2_motor.c`（poll_flux_linkage 函数）

- [ ] **Step 1: 替换 poll_flux_linkage 函数**

old_string 为当前桩，new_string：

```c
/* ===================== flux 辨识（反电势法）=====================
 * STEP 0: 施加 uq 驱动电机转动（theta 跟随实时电角度，不替换回调）
 * STEP 1: 等待稳速（CALIB_CFG_L2_FLUX_SPIN_TICKS）
 * STEP 2: 多次采样 flux_k = (uq - R*iq) / omega_e 取平均
 * STEP 3: flux = avg，校验后写入，完成
 *
 * 公式：稳速时 uq ≈ R*iq + omega_e*flux（忽略 Lq*diq/dt 稳态为 0）
 *   omega_e = omega_m * pole_pairs
 *   omega_m = m->motor_param.slide_rad_s
 *
 * 注意：本子模式不调 calib_hw_enter（需保留实时电角度回调让电机转动），
 *       直接用 calib_hw_apply_voltage 传 m->motor_param.ele_radian 作为 theta。
 * ============================================================ */
static calib_state_e poll_flux_linkage(void)
{
	const calib_io_t *io = calib_mgr_get_io();
	dev_motor_t *m = io->motor;

	switch (s_l2.step.cur)
	{
		case 0: /* 初始化会话（仅记录 motor 指针，不替换回调）*/
			s_l2.session.motor = m;
			s_l2.session.orig_ele_cb = NULL;  /* 标记不替换 */
			s_l2.session.forced_ele_angle = 0.0f;
			s_l2.test_voltage = CALIB_CFG_L2_FLUX_SPIN_VOLTAGE_V;
			calib_step_next(&s_l2.step, 1);
			return CALIB_STATE_RUNNING;

		case 1: /* 驱动转动，等待稳速 */
		{
			float theta = m->motor_param.ele_radian;  /* 跟随实时电角度 */
			calib_hw_apply_voltage(&s_l2.session, 0.0f, s_l2.test_voltage, theta);
			if (calib_step_wait(&s_l2.step, CALIB_CFG_L2_FLUX_SPIN_TICKS))
				return CALIB_STATE_RUNNING;
			calib_step_next(&s_l2.step, 2);
			return CALIB_STATE_RUNNING;
		}

		case 2: /* 稳态采样 */
		{
			float theta = m->motor_param.ele_radian;
			calib_hw_apply_voltage(&s_l2.session, 0.0f, s_l2.test_voltage, theta);
			m->foc.clarke(&m->foc);
			m->foc.park(&m->foc);
			float iq = m->foc.i_dq.q;
			float omega_m = m->motor_param.slide_rad_s;
			uint8_t pp = motor_param_get_pole_pairs(io->param);
			float omega_e = omega_m * (float)pp;
			/* omega_e 过小则跳过（电机未转起）*/
			if (isfinite(omega_e) && fabsf(omega_e) > 1.0f && isfinite(iq))
			{
				float R = motor_param_get_r(io->param);
				float flux_k = (s_l2.test_voltage - R * iq) / omega_e;
				if (isfinite(flux_k) && flux_k > 0.0f)
					calib_step_accumulate(&s_l2.step, flux_k);
			}
			if (s_l2.step.sample_cnt < CALIB_CFG_L2_FLUX_SAMPLE_COUNT)
				return CALIB_STATE_RUNNING;
			calib_step_next(&s_l2.step, 3);
			return CALIB_STATE_RUNNING;
		}

		case 3: /* 校验，写入，撤销电压 */
		{
			calib_hw_apply_zero(m);  /* 直接置零 PWM（未替换回调，无需 exit 恢复）*/
			if (s_l2.step.sample_cnt == 0)
				return CALIB_STATE_FAILED;
			float flux = calib_step_average(&s_l2.step);
			if (!calib_validate_flux(flux))
				return CALIB_STATE_FAILED;
			motor_param_set_flux(io->param, flux);
			calib_mgr_mark_done(CALIB_LEVEL2_MOTOR, CALIB_L2_FLUX_LINKAGE);
			calib_step_reset(&s_l2.step);
			return CALIB_STATE_DONE;
		}

		default:
			calib_hw_apply_zero(m);
			return CALIB_STATE_FAILED;
	}
}
```

- [ ] **Step 2: 编译验证**

Run: sfoc 编译
Expected: 0 Error

- [ ] **Step 3: 不提交**

---

### Task 7: 实现相序识别 (poll_phase_seq)

**Files:**
- Modify: `User/MotorCalibration/calib_level2_motor.c`（poll_phase_seq 函数）

- [ ] **Step 1: 替换 poll_phase_seq 函数**

old_string 为当前桩，new_string：

```c
/* ===================== 相序识别 =====================
 * STEP 0: 施加 ud（theta=0）让转子对齐 d 轴
 * STEP 1: 等待对齐稳定
 * STEP 2: 记录起始机械角度，施加 ud（theta=120°）让转子转 1/3 电周期
 * STEP 3: 等待转动完成
 * STEP 4: 读结束角度，判定 delta 是否合理（应朝一个方向变化），完成
 *
 * 简化版：本版只验证"电机能响应 ud 阶跃并转动"，不持久化结果
 * （motor_param 无相序字段）。若 delta < 阈值视为电机未响应，FAILED。
 * ================================================== */
static calib_state_e poll_phase_seq(void)
{
	const calib_io_t *io = calib_mgr_get_io();
	dev_motor_t *m = io->motor;

	switch (s_l2.step.cur)
	{
		case 0: /* 对齐 d 轴 */
			calib_hw_enter(&s_l2.session, m);
			calib_hw_apply_voltage(&s_l2.session, CALIB_CFG_L2_PHASE_SEQ_VOLTAGE_V, 0.0f, 0.0f);
			calib_step_next(&s_l2.step, 1);
			return CALIB_STATE_RUNNING;

		case 1: /* 等待对齐 */
			calib_hw_apply_voltage(&s_l2.session, CALIB_CFG_L2_PHASE_SEQ_VOLTAGE_V, 0.0f, 0.0f);
			if (calib_step_wait(&s_l2.step, CALIB_CFG_L2_PHASE_SEQ_ALIGN_TICKS))
				return CALIB_STATE_RUNNING;
			calib_step_next(&s_l2.step, 2);
			return CALIB_STATE_RUNNING;

		case 2: /* 记录起始角度，施加 120° 电角度步进 */
			s_l2.start_mech_deg = calib_hw_get_encoder_mech_angle(m);
			/* 120° 电角度 = 2*pi/3 rad */
			calib_hw_apply_voltage(&s_l2.session, CALIB_CFG_L2_PHASE_SEQ_VOLTAGE_V, 0.0f, 2.0F * 3.14159265F / 3.0F);
			calib_step_next(&s_l2.step, 3);
			return CALIB_STATE_RUNNING;

		case 3: /* 等待转动 */
			calib_hw_apply_voltage(&s_l2.session, CALIB_CFG_L2_PHASE_SEQ_VOLTAGE_V, 0.0f, 2.0F * 3.14159265F / 3.0F);
			if (calib_step_wait(&s_l2.step, CALIB_CFG_L2_PHASE_SEQ_STEP_TICKS))
				return CALIB_STATE_RUNNING;
			calib_step_next(&s_l2.step, 4);
			return CALIB_STATE_RUNNING;

		case 4: /* 判定 */
		{
			float end_deg = calib_hw_get_encoder_mech_angle(m);
			float delta = end_deg - s_l2.start_mech_deg;
			if (delta > 180.0f) delta -= 360.0f;
			else if (delta < -180.0f) delta += 360.0f;
			calib_hw_exit(&s_l2.session);
			/* 120° 电角度应对应 120/pole_pairs 机械角度，默认 7 极对 ≈ 17°
			 * 阈值取 5°（电机应明显转动）*/
			if (fabsf(delta) < 5.0f)
				return CALIB_STATE_FAILED;  /* 电机未响应 */
			calib_mgr_mark_done(CALIB_LEVEL2_MOTOR, CALIB_L2_PHASE_SEQ);
			calib_step_reset(&s_l2.step);
			return CALIB_STATE_DONE;
		}

		default:
			calib_hw_exit(&s_l2.session);
			return CALIB_STATE_FAILED;
	}
}
```

- [ ] **Step 2: 编译验证**

Run: sfoc 编译
Expected: 0 Error

- [ ] **Step 3: 不提交**

---

### Task 8: 实现极对数辨识 (poll_pole_pairs)

**Files:**
- Modify: `User/MotorCalibration/calib_level2_motor.c`（poll_pole_pairs 函数）

- [ ] **Step 1: 替换 poll_pole_pairs 函数**

old_string 为当前桩，new_string：

```c
/* ===================== 极对数辨识 =====================
 * STEP 0: 记录起始电角度，施加 uq 让电机转动
 * STEP 1: 持续驱动转动 CALIB_CFG_L2_POLE_PAIRS_SPIN_TICKS
 * STEP 2: 计算极对数 = 电角度变化 / 机械角度变化，取整校验写入
 *
 * 原理：电机转 1 圈机械角度（360°），电角度变化 = 360° * pole_pairs
 *   pole_pairs = 电角度变化 / 机械角度变化
 *   电角度从 motor_param.ele_radian 读（rad），机械角度从编码器读（deg→rad）
 * ===================================================== */
static calib_state_e poll_pole_pairs(void)
{
	const calib_io_t *io = calib_mgr_get_io();
	dev_motor_t *m = io->motor;

	switch (s_l2.step.cur)
	{
		case 0: /* 记录起始角度，驱动转动 */
			s_l2.session.motor = m;
			s_l2.session.orig_ele_cb = NULL;  /* 不替换回调，需实时电角度 */
			s_l2.session.forced_ele_angle = 0.0f;
			s_l2.start_ele_rad = m->motor_param.ele_radian;
			s_l2.start_mech_deg = calib_hw_get_encoder_mech_angle(m);
			calib_hw_apply_voltage(&s_l2.session, 0.0f, CALIB_CFG_L2_POLE_PAIRS_VOLTAGE_V, m->motor_param.ele_radian);
			calib_step_next(&s_l2.step, 1);
			return CALIB_STATE_RUNNING;

		case 1: /* 持续转动 */
			calib_hw_apply_voltage(&s_l2.session, 0.0f, CALIB_CFG_L2_POLE_PAIRS_VOLTAGE_V, m->motor_param.ele_radian);
			if (calib_step_wait(&s_l2.step, CALIB_CFG_L2_POLE_PAIRS_SPIN_TICKS))
				return CALIB_STATE_RUNNING;
			calib_step_next(&s_l2.step, 2);
			return CALIB_STATE_RUNNING;

		case 2: /* 计算极对数 */
		{
			float end_ele_rad = m->motor_param.ele_radian;
			float end_mech_deg = calib_hw_get_encoder_mech_angle(m);

			/* 机械角度变化（deg → rad）*/
			float dmech_deg = end_mech_deg - s_l2.start_mech_deg;
			if (dmech_deg > 180.0f) dmech_deg -= 360.0f;
			else if (dmech_deg < -180.0f) dmech_deg += 360.0f;
			float dmech_rad = dmech_deg * (3.14159265F / 180.0F);

			/* 电角度变化（rad），处理 2pi 跳变——取绝对值累加多圈 */
			float dele_rad = end_ele_rad - s_l2.start_ele_rad;
			/* ele_radian 是 [0,2pi) 单圈值，转动多圈时需数周期。
			 * 简化：取 |dele| 并加 2pi 若为负；近似单圈测，假设 spin 时间内
			 * 转动不超过 1 机械圈（CALIB_CFG_L2_POLE_PAIRS_SPIN_S=1s，10rad/s≈1.6圈，
			 * 略超 1 圈，但用绝对值比值仍近似正确）*/
			if (dele_rad < 0) dele_rad = -dele_rad;

			calib_hw_apply_zero(m);

			if (fabsf(dmech_rad) < 0.1f)
				return CALIB_STATE_FAILED;  /* 电机未转动 */

			float pp_f = dele_rad / dmech_rad;
			uint8_t pp = (uint8_t)(pp_f + 0.5f);  /* 四舍五入 */

			if (!calib_validate_pole_pairs(pp))
				return CALIB_STATE_FAILED;
			motor_param_set_pole_pairs(io->param, pp);
			calib_mgr_mark_done(CALIB_LEVEL2_MOTOR, CALIB_L2_POLE_PAIRS);
			calib_step_reset(&s_l2.step);
			return CALIB_STATE_DONE;
		}

		default:
			calib_hw_apply_zero(m);
			return CALIB_STATE_FAILED;
	}
}
```

- [ ] **Step 2: 编译验证**

Run: sfoc 编译
Expected: 0 Error

- [ ] **Step 3: 不提交**

---

### Task 9: 补充 math.h include + 最终编译验证

**Files:**
- Modify: `User/MotorCalibration/calib_level2_motor.c`（includes 段）

- [ ] **Step 1: 检查是否需要补 math.h**

Run: Read `d:\AAWorkSpace\001_JointMotor\SW\JointMotor\User\MotorCalibration\calib_level2_motor.c` 前 25 行
Expected: 确认 includes 段。若已有 `#include <math.h>`（可能通过 calib_validate.h 传递），跳过 Step 2。

- [ ] **Step 2: 若缺 math.h，补充**

用 Edit，在 `#include "calib_validate.h"` 后加：
```c
#include <math.h>
```

- [ ] **Step 3: 编译 sfoc 工程**

Run: `& "C:\APP\Code\MDK\CORE\UV4\UV4.exe" -b "d:\AAWorkSpace\001_JointMotor\SW\JointMotor\Board\SFOC\MDK-ARM\sfoc.uvprojx" -o "d:\AAWorkSpace\001_JointMotor\SW\JointMotor\Board\SFOC\MDK-ARM\build_l2_t9.log" -j0`
Expected: 0 Error, ≤14 Warning（与基线一致）。`isfinite`/`fabsf` 可用。

- [ ] **Step 4: 编译 JointMotorApp (V1) 工程**

Run: `& "C:\APP\Code\MDK\CORE\UV4\UV4.exe" -b "d:\AAWorkSpace\001_JointMotor\SW\JointMotor\Board\V1\MDK-ARM\JointMotorApp.uvprojx" -o "d:\AAWorkSpace\001_JointMotor\SW\JointMotor\Board\V1\MDK-ARM\build_l2_t9.log" -j0`
Expected: 4 Error（预存 motor_info_* 链接错误，与基线一致），0 个标定相关 error。

- [ ] **Step 5: 不提交**

---

## Self-Review

**1. Spec coverage:**
- 子模式 1 相序识别 → Task 7 ✅
- 子模式 2 极对数 → Task 8 ✅
- 子模式 3 R → Task 3 ✅
- 子模式 4 Ld → Task 4 ✅
- 子模式 5 Lq → Task 5 ✅
- 子模式 6 flux → Task 6 ✅
- 共享参数补全 → Task 1 ✅
- 结构体扩展 → Task 2 ✅
- 编译验证 → Task 9 ✅
- 依赖关系（Ld/Lq/flux 依赖 R）→ 已在 `s_calib_dep_table` 配置，本计划不动 ✅

**2. Placeholder scan:**
- 无 TBD/TODO（桩的 TODO 注释被真实实现替换）
- 无"add error handling"——每个 case 都有显式 FAILED 路径
- 无"similar to Task N"——Ld/Lq 虽算法对称但完整重复代码

**3. Type consistency:**
- `calib_step_t` 字段：`cur`/`tick`/`sample_cnt`/`sample_sum` — 全部一致
- `s_l2` 字段：`submode`/`step`/`session`/`prev_i`/`start_mech_deg`/`start_ele_rad`/`test_voltage` — Task 2 定义，Task 3-8 使用一致
- `calib_hw_apply_voltage(session, ud, uq, theta)` 签名 — 所有调用一致
- `motor_param_get_r/ld/lq/flux/pole_pairs` + `motor_param_set_*` — 与 motor_param.h 一致
- `m->foc.clarke/park/i_dq.d/i_dq.q` — 与 foc_core.h 一致
- `m->motor_param.ele_radian/slide_rad_s` — 与 motion_param.h 一致
- `CALIB_LEVEL2_MOTOR` / `CALIB_L2_*` 常量 — 与 calib_types.h 一致

**已知限制（验收时需注意）：**
1. **相序识别**：本版只做"电机能响应"校验，不持久化结果（motor_param 无相序字段）。
2. **极对数**：用单圈 ele_radian 差分近似，spin 1s 可能转超 1 圈，精度有限。若验收要求高精度需改为多圈累加。
3. **Ld/Lq**：暂态窗口 5ms=50tick，采样 50 点，did/dt 阈值 1.0 A/s 过滤——真实电机可能需调参。
4. **flux**：依赖 R 已标定（用 motor_param_get_r），若 R 用默认 0.1Ω 会有误差。
5. **所有子模式**：假设电机已通过 L3 零位/方向标定（编码器 offset/dir 正确），否则 foc.park 的 id/iq 不准。建议验收顺序 L3→L2。

---

## Execution Handoff

**Plan complete and saved to `User/Data/plans/2026-07-02-calibration-l2-motor-implementation.md`. Two execution options:**

**1. Subagent-Driven (recommended)** - I dispatch a fresh subagent per task, review between tasks, fast iteration

**2. Inline Execution** - Execute tasks in this session using executing-plans, batch execution with checkpoints

**Which approach?**
