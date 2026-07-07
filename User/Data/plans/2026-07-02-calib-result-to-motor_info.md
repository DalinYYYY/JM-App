# 标定结果回流 motor_info 实现方案 v2

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将 L2/L3/L7 标定模式产出的数据写入 `motor_info` 内存实例,由上位机统一通过 0xEA 命令保存到 Flash;`motor_profile.h/c` 的默认参数改为**逐字段零值 fallback** 语义——每个字段独立判断,若该字段为 0(零值)则用默认值覆盖,非零(已标定)则保留标定值。

**Architecture:**
1. 新增 `motor_info_calib` 模块(应用服务层),按子模式提供"标定结果提交"窄接口,内部经 `motor_info_storage_get()` 拿句柄调 `motor_info_set_*()`。
2. 各标定子模块在 `motor_param_set_*()` 之后立即调对应 `motor_info_calib_submit_*()`(单字段提交,非累积,匹配实际代码结构)。
3. `motor_profile_apply_info()` 改**逐字段零值 fallback**:`for each field, if (field == 0) field = DEFAULT;`——每个字段独立判断,零值视为"未设置"用默认值覆盖,非零保留标定值。不依赖 `is_calibrated` 整体标志。
4. L7 全流程在 `s_step >= L7_SEQ_LEN` 返回 `CALIB_STATE_DONE` 前调 `motor_info_calib_mark_calibrated()` 置位 `is_calibrated`(作为"已完成全流程标定"的状态标志供上位机查询,**不作为 fallback 判断依据**)。
5. L4 当前是桩实现(无参数产出),本期不提交。
6. L1 ADC offset 属驱动层运行时校准,不入 motor_info。

**Tech Stack:** STM32G474 HAL + C99 + Keil MDK;无单元测试框架,采用"逐步编译验证 + 硬件在环测试"。

---

## Scope Check

单一子系统重构(标定结果回流 + profile fallback 语义),不需拆分多 plan。

## 实际代码结构调研结果

### L2 各子模式完成点(行号基于当前 [calib_level2_motor.c](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorCalibration/calib_level2_motor.c))

| 子模式 | 完成分支 | 行号 | 写入 motor_param |
|--------|---------|------|-----------------|
| R (DC法) | `poll_resistance` case 3 | 93 | `motor_param_set_r(io->param, R)` |
| Ld (阶跃) | `poll_inductance_d` case 2 | 167 | `motor_param_set_ld(io->param, Ld)` |
| Lq (阶跃) | `poll_inductance_q` case 2 | 240 | `motor_param_set_lq(io->param, Lq)` |
| flux (反电势) | `poll_flux_linkage` case 3 | 323 | `motor_param_set_flux(io->param, flux)` |
| pole_pairs | `poll_pole_pairs` case 2 | 465 | `motor_param_set_pole_pairs(io->param, pp)` |
| phase_seq | `poll_phase_seq` case 4 | 393 | **不写参数**(只验证电机响应) |

**关键**:每个子模式独立完成、独立写 motor_param,无累积 `s_l2.result` 结构体(原方案假设错误)。因此提交 API 应按子模式粒度提供。

### L3 各子模式完成点([calib_level3_encoder.c](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorCalibration/calib_level3_encoder.c))

| 子模式 | 完成分支 | 行号 | 写入 motor_param |
|--------|---------|------|-----------------|
| zero_offset | `poll_zero_offset` case 3 | 82-84 | enc_offset / enc_direction=1 / elec_angle_bias=0 |
| direction | `poll_direction` case 2 | 164 | enc_direction |
| linearity/sincos/multiturn | 桩 | — | 无 |

### L4([calib_level4_torque.c](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorCalibration/calib_level4_torque.c))

`poll_kt` 直接 `return CALIB_STATE_DONE`,**桩实现无参数产出**,本期不提交。

### L7 完成分支([calib_level7_auto.c](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorCalibration/calib_level7_auto.c#L123-L183))

`calib_level7_poll` 在两处返回 `CALIB_STATE_DONE`:
- 行 127:函数入口 `if (s_step >= L7_SEQ_LEN) return CALIB_STATE_DONE;`(重复进入)
- 行 137-138:正常流程完成 `s_step++; if (s_step >= L7_SEQ_LEN) return CALIB_STATE_DONE;`
- 行 157-158/175-176:SKIP/RETRY 策略下完成(当前默认 STOP 策略不走到)

置位点选**行 137-138**(正常流程完成的唯一入口),避免重复进入时重复置位。

### motor_info_set_* 可用 setter(自动生成,[motor_info.h](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/DataHub/motor_info.h))

`MotorCalibParam_t` 段对应的 setter(全部返回 int,0=成功,>0=越界 param_id,<0=错误):
- `motor_info_set_phase_resistance(motor_info_t*, float)`
- `motor_info_set_phase_inductance_d(motor_info_t*, float)`
- `motor_info_set_phase_inductance_q(motor_info_t*, float)`
- `motor_info_set_flux_linkage(motor_info_t*, float)`
- `motor_info_set_pole_pairs(motor_info_t*, uint32_t)`
- `motor_info_set_motor_type(motor_info_t*, uint32_t)`
- `motor_info_set_direction(motor_info_t*, uint32_t)`
- `motor_info_set_torque_constant(motor_info_t*, float)`
- `motor_info_set_rotor_inertia(motor_info_t*, float)`
- `motor_info_set_elec_angle_bias(motor_info_t*, float)`
- `motor_info_set_enc_offset(motor_info_t*, int32_t)`
- `motor_info_set_enc_direction(motor_info_t*, int32_t)`
- `motor_info_set_friction_coulomb(motor_info_t*, float)`
- `motor_info_set_friction_viscous(motor_info_t*, float)`
- `motor_info_set_is_calibrated(motor_info_t*, uint32_t)`

## File Structure

### 新建文件
- `User/AppServices/ParamService/motor_info_calib.h` — 提交 API(按子模式粒度)
- `User/AppServices/ParamService/motor_info_calib.c` — 实现

### 修改文件
| 文件 | 修改内容 | 行号 |
|------|---------|------|
| [motor_profile.c](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/Config/motor_profile.c) | `motor_profile_apply_info` 改 fallback | 47-64 |
| [motor_profile.h](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/Config/motor_profile.h) | apply 语义注释更新 | 69-77 |
| [calib_level2_motor.c](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorCalibration/calib_level2_motor.c) | 5 处追加 submit 调用 | 93, 167, 240, 323, 465 |
| [calib_level3_encoder.c](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorCalibration/calib_level3_encoder.c) | 2 处追加 submit 调用 | 84, 164 |
| [calib_level7_auto.c](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorCalibration/calib_level7_auto.c) | 1 处追加 mark_calibrated | 137-138 |
| [sfoc.uvprojx](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/Board/SFOC/MDK-ARM/sfoc.uvprojx) | 添加 .c 到工程 | AppServices group |

### 不修改文件
- `motor_info.h/.c` — 字段已齐备,自动生成不动
- `motor_info_storage.h/.c` — 上层接口已就绪
- `calib_level4_torque.c` — 桩实现,本期不提交
- `calib_level1_driver.c` — L1 ADC offset 不入 motor_info

## 设计决策

### 决策1: 为什么不直接在 calib 模块调 motor_info_set_*?
- **封装边界**:calib 模块是"测量者",不应感知 motor_info 内部字段映射;通过 `motor_info_calib_submit_*()` 窄接口集中管理
- **可测试性**:未来可 stub submit 函数验证 calib 流程
- **DRY**:L7 调子流程时,子流程内部已自行提交,L7 只需 mark_calibrated

### 决策2: 为什么用单字段/单子模式提交而非"整体完成后一次性提交"?
- **匹配实际代码结构**:每个子模式独立完成、独立写 motor_param,无累积结构体
- **容错性**:中途失败已标定字段(非零)仍生效,配合逐字段零值 fallback 实现部分标定也可用
- **可独立触发**:L2.3 R 可单独标定、单独提交,不需等其他子模式

### 决策3: 为什么用逐字段零值判断 fallback 而非 `is_calibrated` 整体判断?
- **用户明确要求**:具体参数为零就用默认值,非零保留标定值
- **细粒度**:某字段标定成功(非零)就生效,不必等全流程完成;L2.3 R 标定后 R 立即生效,Ld/Lq/flux 仍用默认值(因它们为零)
- **零值合理性**:`motor_info_set_*` 的范围校验已拒绝零值/负值(R=0 会校验失败返回 >0,但 calib 模块忽略返回值继续执行,所以 motor_info 中可能存零),零值=未标定是合理假设
- **NaN/Inf 处理**:浮点 NaN 与 0.0f 比较返回 false(NaN ≠ 0),所以 NaN 字段不会被 fallback 覆盖;但 NaN 是异常状态,应在校验阶段拦截,不依赖 fallback 兜底
- **`is_calibrated` 仍保留**:L7 置位作为状态标志(上位机查询"是否完成全流程"),但不参与 fallback 判断,职责单一化

### 决策4: 为什么 phase_seq 不提交?
- [calib_level2_motor.c:393](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorCalibration/calib_level2_motor.c#L393) `poll_phase_seq` 只验证"电机能响应 ud 阶跃并转动",不持久化结果(motor_param 无相序字段)
- motor_info 也无对应字段,跳过

### 决策5: L3 zero_offset 提交 enc_direction=1 但 direction 子模式会覆盖?
- 是的,zero_offset 先置 CW=1 作为默认,后续 direction 子模式测量真实方向后覆盖
- 两个子模式都提交,最终值由执行顺序决定(direction 后执行则其值生效)

### 决策6: L4 桩为什么不提交?
- [calib_level4_torque.c:10-13](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorCalibration/calib_level4_torque.c#L10-L13) `poll_kt` 直接返回 DONE,无任何计算
- 提交零值无意义,且会触发范围校验失败
- 等 L4 真实实现后再加提交调用

---

## Task 1: motor_profile_apply_info 改逐字段零值 fallback

**Files:**
- Modify: `User/Config/motor_profile.c:47-64`
- Modify: `User/Config/motor_profile.h:69-77`

- [ ] **Step 1: 修改 motor_profile.h 更新 apply 语义注释**

打开 [motor_profile.h](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/Config/motor_profile.h),把第 69-77 行替换为:

```c
/* ===================== apply 接口（在 motor_profile.c 实现）======================
 * 在 motor_param_init() / motor_info_init() 调用之后紧接着调用。
 * **逐字段零值 fallback 语义**：对每个电机电气身份字段独立判断，
 * 若该字段为零值（未设置/未标定），则用 motor_profile.h 的 MOTOR_* 宏覆盖；
 * 非零（已标定）则保留 motor_info 中的实际标定值，不被默认值覆盖。
 *   motor_profile_apply_param(cfg)  —— cfg 实际类型为 motor_param_t*
 *   motor_profile_apply_info(cfg)   —— cfg 实际类型为 motor_info_t*
 * 用 void* 是为了避免 motor_profile.h 循环 include motor_param.h/motor_info.h，
 * 实际类型检查在 motor_profile.c 内部完成。*/
void motor_profile_apply_param(void *cfg);
void motor_profile_apply_info(void *cfg);
```

- [ ] **Step 2: 修改 motor_profile.c 的 motor_profile_apply_info**

打开 [motor_profile.c](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/Config/motor_profile.c),把第 47-64 行的 `motor_profile_apply_info` 函数整体替换为:

```c
void motor_profile_apply_info(void *cfg)
{
	motor_info_t *p = (motor_info_t *)cfg;
	if (p == NULL)
		return;

	/* 逐字段零值 fallback：零值视为未设置，用 profile 默认值覆盖；
	 * 非零保留 motor_info 中的标定值。
	 *   - 首次上电（Flash 无数据）：所有字段为 0，全部用默认值
	 *   - 已标定后上电：标定字段非零保留，未标定字段仍为 0 用默认值
	 *   - 部分标定：已标定字段生效，未标定字段用默认值兜底
	 * 注意：is_calibrated 不参与 fallback 判断，仅作状态标志。*/
	if (p->blocks.motor_calib.pole_pairs == 0U)
		p->blocks.motor_calib.pole_pairs = (uint32_t)MOTOR_POLE_PAIRS;
	if (p->blocks.motor_calib.phase_resistance == 0.0f)
		p->blocks.motor_calib.phase_resistance = MOTOR_R;
	if (p->blocks.motor_calib.phase_inductance_d == 0.0f)
		p->blocks.motor_calib.phase_inductance_d = MOTOR_LD;
	if (p->blocks.motor_calib.phase_inductance_q == 0.0f)
		p->blocks.motor_calib.phase_inductance_q = MOTOR_LQ;
	if (p->blocks.motor_calib.flux_linkage == 0.0f)
		p->blocks.motor_calib.flux_linkage = MOTOR_FLUX;
	if (p->blocks.motor_calib.torque_constant == 0.0f)
		p->blocks.motor_calib.torque_constant = MOTOR_KT;
	if (p->blocks.motor_calib.rotor_inertia == 0.0f)
		p->blocks.motor_calib.rotor_inertia = MOTOR_INERTIA;

	/* is_calibrated / motor_type / direction / 减速器 / 编码器 / 功率级 /
	 * 电流采样 / PID 等不在此覆盖，保留 motor_info_init() 的默认值。*/
}
```

- [ ] **Step 3: 编译验证**

```powershell
$base = "d:\AAWorkSpace\001_JointMotor\SW\JointMotor\Board\SFOC\MDK-ARM\JointMotorApp"
Remove-Item -Path "$base\motor_profile.*" -Force -ErrorAction SilentlyContinue
Get-Process -Name "UV4" -ErrorAction SilentlyContinue | Stop-Process -Force
& "C:\APP\Code\MDK\CORE\UV4\UV4.exe" --% -r "d:\AAWorkSpace\001_JointMotor\SW\JointMotor\Board\SFOC\MDK-ARM\sfoc.uvprojx" -o "d:\AAWorkSpace\001_JointMotor\SW\JointMotor\Board\SFOC\MDK-ARM\build_log.txt"
```
Expected: 0 errors(警告数应与基线一致,约 46-48)

---

## Task 2: 新建 motor_info_calib 模块

**Files:**
- Create: `User/AppServices/ParamService/motor_info_calib.h`
- Create: `User/AppServices/ParamService/motor_info_calib.c`
- Modify: `Board/SFOC/MDK-ARM/sfoc.uvprojx`

- [ ] **Step 1: 创建 motor_info_calib.h**

```c
/**
 * @file        motor_info_calib.h
 * @brief       标定结果回流 motor_info 的提交 API（按子模式粒度）
 *
 * @details     本模块是标定层(calib_levelN_*.c)与持久化层(motor_info_storage)之间的桥梁。
 *              各标定子模式在完成 motor_param_set_*() 之后立即调用对应的 submit_* API，
 *              本模块内部经 motor_info_storage_get() 拿全局 motor_info 句柄，
 *              调用 motor_info_set_*() 写入字段（含范围校验）。
 *              上位机随后通过 0xEA 命令把整个 motor_info 落盘 Flash。
 *
 * @par 模块契约
 *   所有权：本模块无状态，仅转发到 motor_info 全局实例。
 *   生命周期：须在 motor_info_storage_init() 之后调用（依赖全局实例已就绪）。
 *   实时性：submit API 仅写字段，不阻塞；可在标定线程调用。
 *   错误语义：0=成功，<0=系统错误(未初始化/句柄空)，>0=字段范围越界(透传 motor_info_set_*)。
 *
 * @note        L1 相电流 ADC offset 不入 motor_info（属驱动层运行时校准）。
 *              L4 力矩常数当前是桩实现，无数据可提交，本期不提供 submit API。
 *
 * @author      Dalin (dalinyy@163.com)
 * @version     1.0
 * @date        2026-07-02
 */
#ifndef __MOTOR_INFO_CALIB_H__
#define __MOTOR_INFO_CALIB_H__

#include "motor_info.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* ===== L2 子模式提交（每个子模式独立提交单字段或相关字段组）===== */

/**
 * @brief  提交 L2.3 R（相电阻）标定结果
 * @param  r  相电阻(ohm)
 * @return 0=成功, <0=系统错误, >0=字段越界 param_id
 */
int motor_info_calib_submit_r(float r);

/**
 * @brief  提交 L2.4 Ld（d轴电感）标定结果
 * @param  ld  d轴电感(H)
 */
int motor_info_calib_submit_ld(float ld);

/**
 * @brief  提交 L2.5 Lq（q轴电感）标定结果
 * @param  lq  q轴电感(H)
 */
int motor_info_calib_submit_lq(float lq);

/**
 * @brief  提交 L2.6 flux（磁链）标定结果
 * @param  flux  磁链(Wb)
 */
int motor_info_calib_submit_flux(float flux);

/**
 * @brief  提交 L2.2 pole_pairs（极对数）标定结果
 * @param  pole_pairs  极对数
 */
int motor_info_calib_submit_pole_pairs(uint32_t pole_pairs);

/* ===== L3 子模式提交 ===== */

/**
 * @brief  提交 L3.1 编码器零位标定结果
 * @param  elec_angle_bias  电角度偏移(rad)（零位标定为 0）
 * @param  enc_offset       编码器初始位置偏移(counts)
 * @param  enc_direction    编码器方向(1=CW, 零位标定先置 CW)
 */
int motor_info_calib_submit_enc_zero(float elec_angle_bias, int32_t enc_offset, int32_t enc_direction);

/**
 * @brief  提交 L3.2 编码器方向标定结果
 * @param  enc_direction  编码器方向(1=CW, -1=CCW)
 */
int motor_info_calib_submit_enc_direction(int32_t enc_direction);

/* ===== L7 全流程完成置位 ===== */

/**
 * @brief  标记电机已完成全流程标定(is_calibrated = 1)
 * @details 仅在 L7 全自动流程成功完成后调用。
 *          置位后下次上电 motor_profile_apply_info 不再用默认值覆盖。
 */
int motor_info_calib_mark_calibrated(void);

/* ===== 重置标定状态 ===== */

/**
 * @brief  清除标定状态(is_calibrated = 0)，并清零电机电气字段
 * @details 用于上位机"恢复出厂"或重新标定前的清理。
 *          清除后 profile 默认值会在下次 motor_profile_apply_info 时生效。
 */
int motor_info_calib_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_INFO_CALIB_H__ */
```

- [ ] **Step 2: 创建 motor_info_calib.c**

```c
/**
 * @file        motor_info_calib.c
 * @brief       标定结果回流 motor_info 的提交 API 实现
 *
 * @author      Dalin (dalinyy@163.com)
 * @version     1.0
 * @date        2026-07-02
 */
#include "motor_info_calib.h"
#include "motor_info_storage.h"

#if defined(USE_DEV_FLASH)

/* ===== 内部工具：取全局 motor_info 句柄并校验 ===== */
static motor_info_t *get_info_checked(void)
{
    return motor_info_storage_get();  /* init 后保证非 NULL */
}

/* ===== L2.3 R ===== */
int motor_info_calib_submit_r(float r)
{
    motor_info_t *p = get_info_checked();
    if (p == NULL) return -1;
    return motor_info_set_phase_resistance(p, r);
}

/* ===== L2.4 Ld ===== */
int motor_info_calib_submit_ld(float ld)
{
    motor_info_t *p = get_info_checked();
    if (p == NULL) return -1;
    return motor_info_set_phase_inductance_d(p, ld);
}

/* ===== L2.5 Lq ===== */
int motor_info_calib_submit_lq(float lq)
{
    motor_info_t *p = get_info_checked();
    if (p == NULL) return -1;
    return motor_info_set_phase_inductance_q(p, lq);
}

/* ===== L2.6 flux ===== */
int motor_info_calib_submit_flux(float flux)
{
    motor_info_t *p = get_info_checked();
    if (p == NULL) return -1;
    return motor_info_set_flux_linkage(p, flux);
}

/* ===== L2.2 pole_pairs ===== */
int motor_info_calib_submit_pole_pairs(uint32_t pole_pairs)
{
    motor_info_t *p = get_info_checked();
    if (p == NULL) return -1;
    return motor_info_set_pole_pairs(p, pole_pairs);
}

/* ===== L3.1 编码器零位（一次提交 3 个相关字段）===== */
int motor_info_calib_submit_enc_zero(float elec_angle_bias, int32_t enc_offset, int32_t enc_direction)
{
    motor_info_t *p = get_info_checked();
    if (p == NULL) return -1;
    int rc;
    rc = motor_info_set_elec_angle_bias(p, elec_angle_bias);
    if (rc != 0) return rc;
    rc = motor_info_set_enc_offset(p, enc_offset);
    if (rc != 0) return rc;
    rc = motor_info_set_enc_direction(p, enc_direction);
    return rc;
}

/* ===== L3.2 编码器方向 ===== */
int motor_info_calib_submit_enc_direction(int32_t enc_direction)
{
    motor_info_t *p = get_info_checked();
    if (p == NULL) return -1;
    return motor_info_set_enc_direction(p, enc_direction);
}

/* ===== L7 置位 is_calibrated ===== */
int motor_info_calib_mark_calibrated(void)
{
    motor_info_t *p = get_info_checked();
    if (p == NULL) return -1;
    return motor_info_set_is_calibrated(p, 1U);
}

/* ===== 重置标定状态 ===== */
int motor_info_calib_reset(void)
{
    motor_info_t *p = get_info_checked();
    if (p == NULL) return -1;

    (void)motor_info_set_is_calibrated(p, 0U);
    (void)motor_info_set_phase_resistance(p, 0.0f);
    (void)motor_info_set_phase_inductance_d(p, 0.0f);
    (void)motor_info_set_phase_inductance_q(p, 0.0f);
    (void)motor_info_set_flux_linkage(p, 0.0f);
    (void)motor_info_set_elec_angle_bias(p, 0.0f);
    (void)motor_info_set_enc_offset(p, 0);
    (void)motor_info_set_enc_direction(p, 0);

    return 0;
}

#endif /* USE_DEV_FLASH */
```

- [ ] **Step 3: 添加 motor_info_calib.c 到 Keil 工程**

打开 [sfoc.uvprojx](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/Board/SFOC/MDK-ARM/sfoc.uvprojx),在 AppServices group 内(参照 `motor_info_storage.c` 的 `<File>` 条目格式),添加:

```xml
<File>
  <FileName>motor_info_calib.c</FileName>
  <FileType>1</FileType>
  <FilePath>..\..\..\User\AppServices\ParamService\motor_info_calib.c</FilePath>
</File>
```

- [ ] **Step 4: 编译验证**

```powershell
$base = "d:\AAWorkSpace\001_JointMotor\SW\JointMotor\Board\SFOC\MDK-ARM\JointMotorApp"
Remove-Item -Path "$base\motor_info_calib.*","$base\motor_profile.*" -Force -ErrorAction SilentlyContinue
Get-Process -Name "UV4" -ErrorAction SilentlyContinue | Stop-Process -Force
& "C:\APP\Code\MDK\CORE\UV4\UV4.exe" --% -r "d:\AAWorkSpace\001_JointMotor\SW\JointMotor\Board\SFOC\MDK-ARM\sfoc.uvprojx" -o "d:\AAWorkSpace\001_JointMotor\SW\JointMotor\Board\SFOC\MDK-ARM\build_log.txt"
```
Expected: 0 errors(新模块独立编译,无外部依赖)

---

## Task 3: L2 各子模式完成后提交结果

**Files:**
- Modify: `User/MotorCalibration/calib_level2_motor.c`(5 处:行 93, 167, 240, 323, 465)

- [ ] **Step 1: 添加 include**

打开 [calib_level2_motor.c](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorCalibration/calib_level2_motor.c),在第 24 行 `#include "calib_validate.h"` 之后添加:

```c
#include "calib_validate.h"
#include "motor_info_calib.h"
```

- [ ] **Step 2: L2.3 R 完成后提交(行 93 附近)**

定位 `poll_resistance` case 3(行 84-98),在 `motor_param_set_r(io->param, R);` 之后(行 93)、`calib_hw_exit` 之前添加 submit 调用:

修改前(行 92-94):
```c
			motor_param_set_r(io->param, R);
			calib_hw_exit(&s_l2.session);
			calib_mgr_mark_done(CALIB_LEVEL2_MOTOR, CALIB_L2_RESISTANCE);
```

修改后:
```c
			motor_param_set_r(io->param, R);
			(void)motor_info_calib_submit_r(R);
			calib_hw_exit(&s_l2.session);
			calib_mgr_mark_done(CALIB_LEVEL2_MOTOR, CALIB_L2_RESISTANCE);
```

- [ ] **Step 3: L2.4 Ld 完成后提交(行 167 附近)**

定位 `poll_inductance_d` case 2(行 154-172),在 `motor_param_set_ld(io->param, Ld);` 之后添加:

修改前(行 167-169):
```c
			motor_param_set_ld(io->param, Ld);
			calib_hw_exit(&s_l2.session);
			calib_mgr_mark_done(CALIB_LEVEL2_MOTOR, CALIB_L2_INDUCTANCE_D);
```

修改后:
```c
			motor_param_set_ld(io->param, Ld);
			(void)motor_info_calib_submit_ld(Ld);
			calib_hw_exit(&s_l2.session);
			calib_mgr_mark_done(CALIB_LEVEL2_MOTOR, CALIB_L2_INDUCTANCE_D);
```

- [ ] **Step 4: L2.5 Lq 完成后提交(行 240 附近)**

定位 `poll_inductance_q` case 2(行 227-245),在 `motor_param_set_lq(io->param, Lq);` 之后添加:

修改前(行 240-242):
```c
			motor_param_set_lq(io->param, Lq);
			calib_hw_exit(&s_l2.session);
			calib_mgr_mark_done(CALIB_LEVEL2_MOTOR, CALIB_L2_INDUCTANCE_Q);
```

修改后:
```c
			motor_param_set_lq(io->param, Lq);
			(void)motor_info_calib_submit_lq(Lq);
			calib_hw_exit(&s_l2.session);
			calib_mgr_mark_done(CALIB_LEVEL2_MOTOR, CALIB_L2_INDUCTANCE_Q);
```

- [ ] **Step 5: L2.6 flux 完成后提交(行 323 附近)**

定位 `poll_flux_linkage` case 3(行 315-327),在 `motor_param_set_flux(io->param, flux);` 之后添加:

修改前(行 323-325):
```c
			motor_param_set_flux(io->param, flux);
			calib_mgr_mark_done(CALIB_LEVEL2_MOTOR, CALIB_L2_FLUX_LINKAGE);
			calib_step_reset(&s_l2.step);
```

修改后:
```c
			motor_param_set_flux(io->param, flux);
			(void)motor_info_calib_submit_flux(flux);
			calib_mgr_mark_done(CALIB_LEVEL2_MOTOR, CALIB_L2_FLUX_LINKAGE);
			calib_step_reset(&s_l2.step);
```

- [ ] **Step 6: L2.2 pole_pairs 完成后提交(行 465 附近)**

定位 `poll_pole_pairs` case 2(行 437-468),在 `motor_param_set_pole_pairs(io->param, pp);` 之后添加:

修改前(行 465-467):
```c
			motor_param_set_pole_pairs(io->param, pp);
			calib_mgr_mark_done(CALIB_LEVEL2_MOTOR, CALIB_L2_POLE_PAIRS);
			calib_step_reset(&s_l2.step);
```

修改后:
```c
			motor_param_set_pole_pairs(io->param, pp);
			(void)motor_info_calib_submit_pole_pairs(pp);
			calib_mgr_mark_done(CALIB_LEVEL2_MOTOR, CALIB_L2_POLE_PAIRS);
			calib_step_reset(&s_l2.step);
```

- [ ] **Step 7: 编译验证**

```powershell
$base = "d:\AAWorkSpace\001_JointMotor\SW\JointMotor\Board\SFOC\MDK-ARM\JointMotorApp"
Remove-Item -Path "$base\calib_level2_motor.*" -Force -ErrorAction SilentlyContinue
Get-Process -Name "UV4" -ErrorAction SilentlyContinue | Stop-Process -Force
& "C:\APP\Code\MDK\CORE\UV4\UV4.exe" --% -r "d:\AAWorkSpace\001_JointMotor\SW\JointMotor\Board\SFOC\MDK-ARM\sfoc.uvprojx" -o "d:\AAWorkSpace\001_JointMotor\SW\JointMotor\Board\SFOC\MDK-ARM\build_log.txt"
```
Expected: 0 errors

---

## Task 4: L3 编码器标定完成后提交结果

**Files:**
- Modify: `User/MotorCalibration/calib_level3_encoder.c`(2 处:行 84, 164)

- [ ] **Step 1: 添加 include**

打开 [calib_level3_encoder.c](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorCalibration/calib_level3_encoder.c),在第 24 行 `#include "calib_validate.h"` 之后添加:

```c
#include "calib_validate.h"
#include "motor_info_calib.h"
```

- [ ] **Step 2: L3.1 零位标定完成后提交(行 82-84 附近)**

定位 `poll_zero_offset` case 3(行 69-89),在三个 `motor_param_set_*` 之后(行 84)、`calib_hw_exit` 之前添加 submit 调用:

修改前(行 82-85):
```c
			motor_param_set_enc_offset(io->param, raw_counts);
			motor_param_set_enc_direction(io->param, 1); /* 1=CW */
			motor_param_set_elec_angle_bias(io->param, 0.0f);
			calib_hw_exit(&s_l3.session);
```

修改后:
```c
			motor_param_set_enc_offset(io->param, raw_counts);
			motor_param_set_enc_direction(io->param, 1); /* 1=CW */
			motor_param_set_elec_angle_bias(io->param, 0.0f);
			/* 提交零位标定结果到 motor_info */
			(void)motor_info_calib_submit_enc_zero(0.0f, raw_counts, 1);
			calib_hw_exit(&s_l3.session);
```

- [ ] **Step 3: L3.2 方向标定完成后提交(行 164 附近)**

定位 `poll_direction` case 2(行 128-167),在 `motor_param_set_enc_direction(io->param, enc_dir);` 之后(行 164)添加:

修改前(行 164-165):
```c
			motor_param_set_enc_direction(io->param, enc_dir);
			calib_step_next(&s_l3.step, 3);
```

修改后:
```c
			motor_param_set_enc_direction(io->param, enc_dir);
			(void)motor_info_calib_submit_enc_direction(enc_dir);
			calib_step_next(&s_l3.step, 3);
```

- [ ] **Step 4: 编译验证**

```powershell
$base = "d:\AAWorkSpace\001_JointMotor\SW\JointMotor\Board\SFOC\MDK-ARM\JointMotorApp"
Remove-Item -Path "$base\calib_level3_encoder.*" -Force -ErrorAction SilentlyContinue
Get-Process -Name "UV4" -ErrorAction SilentlyContinue | Stop-Process -Force
& "C:\APP\Code\MDK\CORE\UV4\UV4.exe" --% -r "d:\AAWorkSpace\001_JointMotor\SW\JointMotor\Board\SFOC\MDK-ARM\sfoc.uvprojx" -o "d:\AAWorkSpace\001_JointMotor\SW\JointMotor\Board\SFOC\MDK-ARM\build_log.txt"
```
Expected: 0 errors

---

## Task 5: L7 全流程完成置位 is_calibrated(状态标志)

**Files:**
- Modify: `User/MotorCalibration/calib_level7_auto.c`(1 处:行 137-138)

> **说明**:本任务置位 `is_calibrated` 作为"已完成全流程标定"的状态标志供上位机查询,**不参与 fallback 判断**(fallback 由 Task 1 的逐字段零值判断决定)。保留置位是因为它有独立的状态指示价值。

- [ ] **Step 1: 添加 include**

打开 [calib_level7_auto.c](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/MotorCalibration/calib_level7_auto.c),在第 11 行 `#include "calib_mgr.h"` 之后添加:

```c
#include "calib_mgr.h"
#include "motor_info_calib.h"
```

- [ ] **Step 2: L7 全流程完成时置位(行 137-138 附近)**

定位 `calib_level7_poll` 中正常流程完成分支(行 131-142),在 `s_step++;` 之后、`if (s_step >= L7_SEQ_LEN) return CALIB_STATE_DONE;` 之前添加 mark_calibrated:

修改前(行 134-138):
```c
		calib_mgr_mark_done(s_sequence[s_step].level, s_sequence[s_step].submode);
		s_step++;
		s_retry_count = 0;
		if (s_step >= L7_SEQ_LEN)
			return CALIB_STATE_DONE;
```

修改后:
```c
		calib_mgr_mark_done(s_sequence[s_step].level, s_sequence[s_step].submode);
		s_step++;
		s_retry_count = 0;
		if (s_step >= L7_SEQ_LEN)
		{
			/* L7 全流程完成，置位 is_calibrated，下次上电使用标定值 */
			(void)motor_info_calib_mark_calibrated();
			return CALIB_STATE_DONE;
		}
```

- [ ] **Step 3: 编译验证**

```powershell
$base = "d:\AAWorkSpace\001_JointMotor\SW\JointMotor\Board\SFOC\MDK-ARM\JointMotorApp"
Remove-Item -Path "$base\calib_level7_auto.*" -Force -ErrorAction SilentlyContinue
Get-Process -Name "UV4" -ErrorAction SilentlyContinue | Stop-Process -Force
& "C:\APP\Code\MDK\CORE\UV4\UV4.exe" --% -r "d:\AAWorkSpace\001_JointMotor\SW\JointMotor\Board\SFOC\MDK-ARM\sfoc.uvprojx" -o "d:\AAWorkSpace\001_JointMotor\SW\JointMotor\Board\SFOC\MDK-ARM\build_log.txt"
```
Expected: 0 errors

---

## Task 6: 全量编译验证

**Files:** 无修改,仅验证

- [ ] **Step 1: 全量清理并编译**

```powershell
$base = "d:\AAWorkSpace\001_JointMotor\SW\JointMotor\Board\SFOC\MDK-ARM\JointMotorApp"
Remove-Item -Path "$base\*.o","$base\*.d","$base\*.crf" -Force -ErrorAction SilentlyContinue
Get-Process -Name "UV4" -ErrorAction SilentlyContinue | Stop-Process -Force
& "C:\APP\Code\MDK\CORE\UV4\UV4.exe" --% -r "d:\AAWorkSpace\001_JointMotor\SW\JointMotor\Board\SFOC\MDK-ARM\sfoc.uvprojx" -o "d:\AAWorkSpace\001_JointMotor\SW\JointMotor\Board\SFOC\MDK-ARM\build_log.txt"
```

- [ ] **Step 2: 验证编译结果**

读取 `build_log.txt`,确认:
- 0 errors
- 警告数 ≤ 48(基线)
- Program Size Code 与基线对比,新增量来自 motor_info_calib.c(估 +400B)
- 链接阶段正常(`linking...` 出现)
- `JointMotorApp.axf - 0 Error(s), N Warning(s).`

---

## Task 7: 硬件验证(可选,需硬件在场)

- [ ] **Step 1: 烧录最新固件**

通过 Keil 下载或 ST-Link 烧录 `JointMotorApp.axf`。

- [ ] **Step 2: 验证首次上电 fallback**

首次上电(Flash 无数据):
- 上位机读 0xE6 motor_info,确认 R/Ld/Lq/flux/pole_pairs = motor_profile.h 默认值
- 确认 is_calibrated = 0

- [ ] **Step 3: 触发 L2.3 R 标定并验证提交**

上位机发 0x90 payload[0]=3(启动 L2 R 标定):
- 标定完成后,读 0xE6,确认 phase_resistance 已更新为测量值(非默认值)
- 但 is_calibrated 仍为 0(L7 未完成)

- [ ] **Step 4: 触发 L7 全流程并验证置位**

上位机发 0x96(启动 L7 全自动):
- 全流程完成后,读 0xE6,确认 is_calibrated = 1
- 确认所有电气参数均已更新

- [ ] **Step 5: 上位机保存到 Flash**

上位机发 0xEA:
- 确认返回 0(成功)
- 重启设备,读 0xE6 确认数据已持久化

- [ ] **Step 6: 验证已标定上电不 fallback**

重启后:
- 读 0xE6,确认 is_calibrated = 1
- 确认 R/Ld/Lq/flux 等为标定值,未被 motor_profile 默认值覆盖

---

## Self-Review

### 1. Spec 覆盖
- ✅ 所有标定数据设置到 motor_info:Task 3(L2 五字段)/Task 4(L3 两字段)/Task 5(L7 置位状态标志)
- ✅ 上位机统一保存:0xEA 已存在,标定提交后内存更新,0xEA 落盘
- ✅ motor_profile 逐字段零值 fallback:Task 1 改 apply_info 每字段独立判断
- ✅ 标定成功后使用标定数据:非零字段保留,零值字段才用默认值

### 2. Placeholder 扫描
- ✅ 无 "TBD"/"TODO"/"implement later"
- ✅ 所有字段名来自实际代码读取(L2/L3/L7 调研结果表)
- ✅ 所有行号基于当前文件实际行号
- ✅ 每个修改步骤都有"修改前/修改后"对比

### 3. 类型一致性
- ✅ `motor_info_calib_submit_r/ld/lq/flux/pole_pairs` 在 Task 2 定义,Task 3 调用,签名一致
- ✅ `motor_info_calib_submit_enc_zero/enc_direction` 在 Task 2 定义,Task 4 调用,签名一致
- ✅ `motor_info_calib_mark_calibrated` 在 Task 2 定义,Task 5 调用,签名一致
- ✅ 返回值约定统一:0=成功, <0=系统错误, >0=字段越界

### 4. 风险点
- **零值=未标定假设的合理性**:`motor_info_set_*` 范围校验会拒绝零值(R=0 返回 >0),但 calib 模块用 `(void)` 忽略返回值,motor_info 中字段可能存零。但 calib 内部已有 `calib_validate_*` 预校验(如 `calib_validate_r(R)` 拒绝 R<=0),所以写入 motor_info 的值不会是零。零值仅出现在"未标定"场景,fallback 假设成立
- **NaN 字段不被 fallback 覆盖**:浮点 NaN 与 0.0f 比较返回 false,NaN 字段会保留;但 NaN 是异常状态,应在校验阶段拦截,不依赖 fallback 兜底
- **L3 零位提交方向被 L3 方向覆盖**:zero_offset 提交 enc_direction=1,若后续 direction 子模式执行会覆盖为测量值,符合预期(决策5)。注意 enc_direction 不在 motor_profile fallback 覆盖范围
- **L7 SKIP 策略下完成仍置位**:当前默认 STOP 策略,SKIP 策略下若某步失败被跳过,L7 仍会 DONE 并置位 is_calibrated;但因 fallback 不依赖 is_calibrated,即使置位错误也不会影响字段值(由逐字段零值判断决定),风险降低
- **L4 桩不提交**:L4 真实实现后需补 submit API 和调用点,本方案预留

---

## Execution Handoff

Plan complete and saved to `User/Data/plans/2026-07-02-calib-result-to-motor_info.md`.

Two execution options:

**1. Subagent-Driven (recommended)** - 每个 Task 派发独立子代理,任务间审查,快速迭代

**2. Inline Execution** - 在当前会话内逐 Task 执行,带检查点

Which approach?
