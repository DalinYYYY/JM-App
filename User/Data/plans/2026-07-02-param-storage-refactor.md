# 参数存储功能重构 Implementation Plan (v3)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 重构 motor_info 参数存储链路：保留 dev_flash 调用链但配置成单扇区固定地址，明确"首次上电初始化"与"非首次上电直接加载"两条路径，消除 motor_info_init 非零默认值与 apply_info 零值判断的语义冲突。

**Architecture:** motor_info_storage 通过 dev_flash（单扇区，不实际轮转）调用 drv_flash_g4。dev_flash 的 last_sequence 判断是否首次上电。motor_profile 提供两种覆盖语义（apply_info_default 无条件覆盖 / apply_info 零值fallback）。用 magic + config_version + CRC 判断 Flash 数据有效性。

**Tech Stack:** STM32G474 HAL Flash (drv_flash_g4) / dev_flash 通用页存储 / C-OOP 设备对象模式 / CRC32 校验

---

## 核对结论（已验证）

| 核对项 | 实际情况 | 方案处理 |
|--------|---------|---------|
| dev_flash_read/write 签名 | `int dev_flash_read(struct dev_flash *pobj, u32 offset, u64 *data, u16 size)` | 保留调用 |
| dev_flash_init 签名 | `void dev_flash_init(struct dev_flash *pobj, u32 start_addr, u32 total_size, u32 page_size)` | 单扇区: total_size=page_size |
| dev_flash last_sequence 字段 | `dev_flash.h:58` 定义，init 扫描后赋值 | 用于判断首次上电 |
| jm_proto_ops.c 弱符号 | `jm_proto_ops.c:664-668` 有 `__weak` 修饰 | 强符号覆盖正常 |
| user_interface.c 条件编译 | `user_interface.c:23` 无条件 include，`35` 行无条件调用 | 不需改条件编译 |
| motor_info_calib.c 条件编译 | `motor_info_calib.c:12` 用 `#if defined(USE_DEV_FLASH)` | motor_info_storage.h 保持 USE_DEV_FLASH |
| motor_info_init 非零默认值 | `motor_info.c:50-59` pole_pairs=7, R=0.1 等 | apply_info_default 无条件覆盖 |
| motor_info_validate 签名 | `int motor_info_validate(const motor_info_t *cfg)` | 保留调用 |
| config_version 字段 | `motor_info.c:43` init 设为 65536 | apply_info_default 覆盖为 MOTOR_PROFILE_CONFIG_VERSION |

---

## 当前问题分析

### 问题1：motor_info_init 非零默认值与 apply_info 零值判断冲突

`motor_info.c:50-59`（自动生成文件，不可手动改）设置了非零默认值（pole_pairs=7, R=0.1 等）。而 `motor_profile.c:59-72` 的 `apply_info` 用零值判断（`if (field == 0) field = DEFAULT`），init 的非零默认值让零值判断失效。当前补丁在 storage 层手动清零 7 字段——硬编码，DRY 违反。

**解决**：新增 `apply_info_default` 无条件覆盖，首次上电时无视 init 默认值强制用 profile 覆盖。

### 问题2：非首次上电仍走 motor_info_init

当前 motor_info_storage.c:168 无条件调用 motor_info_init，即使 Flash 有有效数据。

**解决**：用 dev_flash 的 last_sequence 判断，>0 时直接用 Flash 数据，不调 motor_info_init。

### 问题3：profile 升级后旧 Flash 数据不更新

**解决**：config_version 不匹配时走首次上电路径重新初始化。

---

## 重构方案设计

### 核心设计：保留 dev_flash + 单扇区 + 两条路径

```
存储区域：0x0804F000，单扇区 2KB（dev_flash 配置 total_size=page_size=2048）
数据布局：扇区首 8 字节 = dev_flash flag(magic+sequence)，数据从 offset 8 开始
查看方式：调试器看 0x0804F000 是 flag，0x0804F008 是 motor_info 数据

motor_info_storage_init()
    │
    ├─ dev_flash_init(0x0804F000, 2048, 2048) 单扇区，扫描 flag
    │
    ├─ dev_flash.last_sequence > 0 ?（有有效 flag）
    │   ├─ YES（非首次上电）
    │   │   └─ dev_flash_read → 四重校验(magic+config_version+CRC+range)
    │   │       ├─ 通过 → 直接用 Flash 数据 → apply_info(零值fallback) → 完成
    │   │       └─ 失败 → 走首次上电路径
    │   │
    │   └─ NO（首次上电 / Flash 全空）
    │       └─ motor_info_init(默认值) → apply_info_default(无条件覆盖)
    │           → dev_flash_write(回写)
    │
    └─ 标定完成 → motor_info_calib_submit_*（写内存）→ 0xEA 保存 Flash
```

### 关键设计决策

**决策1：保留 dev_flash 调用链，配置单扇区**

dev_flash_init 传入 total_size = page_size = 2048，sector_count = 1。
- 写入时 next = (0+1)%1 = 0，始终写扇区0，固定地址
- last_sequence 每次写入递增，用作"是否首次上电"判断
- 数据有 8 字节 flag 偏移（dev_flash 机制），调试器看 0x0804F008 是 motor_info

**决策2：apply_info 拆成两个函数**

- `motor_profile_apply_info(cfg)` — 零值 fallback（非首次上电用）：零值字段用 profile 覆盖，非零保留标定值
- `motor_profile_apply_info_default(cfg)` — 无条件覆盖（首次上电用）：无视 init 的非零默认值，强制用 profile 覆盖 + 设 config_version

**决策3：用 dev_flash.last_sequence 判断首次上电**

- last_sequence == 0 → 首次上电（dev_flash_init 扫描无有效 flag）
- last_sequence > 0 → 非首次上电（有写入过的扇区）

**决策4：非首次上电不调用 motor_info_init**

Flash 数据有效时，直接用 Flash 数据，不经过 motor_info_init。仅在零值字段用 profile 补缺。

**决策5：首次上电自动回写 Flash**

首次上电 init + apply_info_default 后，立即 dev_flash_write。下次上电直接走非首次上电路径。

**决策6：标定完成后不自动保存 Flash**

标定完成后只更新 motor_info 内存。保存到 Flash 由上位机发 0xEA 触发。

**决策7：config_version 检测 profile 变更**

固件升级改 profile 后，Flash 中 config_version 不匹配 → 走首次上电路径。

---

## 文件结构

| 文件 | 职责 | 改动 |
|------|------|------|
| `User/Config/motor_profile.h` | profile 公共接口 | 新增 apply_info_default 声明 + MOTOR_PROFILE_CONFIG_VERSION 宏 |
| `User/Config/motor_profile.c` | profile apply 实现 | 新增 apply_info_default（无条件覆盖） |
| `User/AppServices/ParamService/motor_info_storage.c` | storage 实现 | 重构 init 两条路径，删除清零7字段硬编码 |

不改动（已核对无需修改）：
- `motor_info_storage.h` — 保持 USE_DEV_FLASH + dev_flash 依赖
- `dev_flash.c/h` — 通用页存储，单扇区配置即可
- `drv_flash_g4.c/h` — HAL 驱动
- `motor_info.c/h` — 自动生成文件
- `motor_info_calib.c/h` — 用 USE_DEV_FLASH，与 storage.h 一致
- `user_interface.c` — 无条件编译包裹
- `jm_proto_ops.c` — 弱符号机制正常
- `sfoc.uvprojx` — 无新增文件

---

## Task 1: motor_profile 新增 apply_info_default + config_version

**Files:**
- Modify: `User/Config/motor_profile.h:76-77`
- Modify: `User/Config/motor_profile.c:76`

- [ ] **Step 1: motor_profile.h 新增声明和 config_version 宏**

当前 `motor_profile.h:76-77`：
```c
void motor_profile_apply_param(void *cfg);
void motor_profile_apply_info(void *cfg);
```

替换为：
```c
/* profile 配置版本号：修改 MOTOR_PROFILE 宏切换电机型号时递增此版本号，
 * 用于上电时检测 Flash 中存储的参数是否对应当前固件的 profile。
 * 版本不匹配时触发重新初始化（init + apply_default + save）。*/
#define MOTOR_PROFILE_CONFIG_VERSION 1U

void motor_profile_apply_param(void *cfg);
void motor_profile_apply_info(void *cfg);          /* 零值 fallback：非首次上电用 */
void motor_profile_apply_info_default(void *cfg);  /* 无条件覆盖：首次上电用 */
```

- [ ] **Step 2: motor_profile.c 新增 apply_info_default 实现**

当前 `motor_profile.c:76` 是 apply_info 函数末尾 `}`。在其后新增：

```c
void motor_profile_apply_info_default(void *cfg)
{
	motor_info_t *p = (motor_info_t *)cfg;
	if (p == NULL)
		return;

	/* 无条件覆盖：无视 motor_info_init 的非零通用默认值，
	 * 强制用 profile 的电机型号特定默认值覆盖。
	 * 仅在首次上电（Flash 无数据 或 profile 版本不匹配）时调用。*/
	p->blocks.motor_calib.pole_pairs         = (uint32_t)MOTOR_POLE_PAIRS;
	p->blocks.motor_calib.phase_resistance   = MOTOR_R;
	p->blocks.motor_calib.phase_inductance_d = MOTOR_LD;
	p->blocks.motor_calib.phase_inductance_q = MOTOR_LQ;
	p->blocks.motor_calib.flux_linkage       = MOTOR_FLUX;
	p->blocks.motor_calib.torque_constant    = MOTOR_KT;
	p->blocks.motor_calib.rotor_inertia      = MOTOR_INERTIA;

	/* 同时设置 config_version，标记当前 profile 版本 */
	p->blocks.system.config_version = MOTOR_PROFILE_CONFIG_VERSION;
}
```

- [ ] **Step 3: 验证编译**

```powershell
& "C:\APP\Code\MDK\CORE\UV4\UV4.exe" --% -b "d:\AAWorkSpace\001_JointMotor\SW\JointMotor\Board\SFOC\MDK-ARM\sfoc.uvprojx" -o "d:\AAWorkSpace\001_JointMotor\SW\JointMotor\Board\SFOC\MDK-ARM\build_log.txt"
```
Expected: 0 Error（新增函数未被调用，仅声明+定义）

---

## Task 2: motor_info_storage.c 重构 init 两条路径

**Files:**
- Modify: `User/AppServices/ParamService/motor_info_storage.c:150-214`

### 当前代码问题（motor_info_storage.c:150-214）

1. 行168 无条件调 motor_info_init
2. 行185-193 load 失败后手动清零 7 字段（硬编码）
3. 非首次上电也走 init → load 覆盖

### 修改后

- [ ] **Step 1: 在 motor_info_storage.c 头部确认 include**

当前 `motor_info_storage.c:25` 已有 `#include "motor_profile.h"`。
需新增 `#include "motor_info.h"`（用于 motor_info_init 调用，确认是否已有）。

检查：如果 motor_info_storage.h 已 include motor_info.h（通过 dev_flash.h 链），则 motor_info_storage.c 中 motor_info_init 可见。如不可见则添加。

- [ ] **Step 2: 替换 motor_info_storage_init 函数**

将 `motor_info_storage.c:150-214` 的 `motor_info_storage_init` 函数替换为：

```c
motor_info_storage_status_t motor_info_storage_init(void)
{
	struct motor_info_storage *pobj = &g_motor_info_storage;

	/* 1. 装配 ops 方法表指针 */
	pobj->ops = &s_motorinfo_ops;

	/* 2. 初始化 dev_flash 子设备（单扇区配置：total_size=page_size=2048）
	 *    sector_count=1，写入时 next=(0+1)%1=0 始终写扇区0，固定地址
	 *    dev_flash_init 扫描扇区 flag，last_sequence>0 表示有写入过的数据 */
	dev_flash_init(&pobj->flash_dev,
	               MOTORINFO_FLASH_START_ADDR,
	               MOTORINFO_FLASH_PAGE_SIZE,    /* total_size = page_size，单扇区 */
	               MOTORINFO_FLASH_PAGE_SIZE);
	pobj->inited = pobj->flash_dev.inited ? true : false;

	/* 3. 加载全局 motor_info（上电自动加载，仅一次） */
	if (!pobj->motor_info_loaded)
	{
		bool need_init_default = true;  /* 是否走首次上电默认路径 */

		/* 路径A：非首次上电——dev_flash 扫描到有效 flag（last_sequence > 0） */
		if (pobj->inited && pobj->flash_dev.last_sequence > 0U)
		{
			motor_info_storage_status_t lr = motorinfo_ops_load(pobj, &pobj->motor_info);
			if (lr == MOTOR_INFO_STORAGE_OK)
			{
				/* Flash 数据有效（magic + config_version + CRC + range 全通过）
				 * 直接用 Flash 数据，不调用 motor_info_init
				 * apply_info(零值fallback)：未标定字段（零值）用 profile 补缺，
				 * 已标定字段（非零）保留 Flash 中的标定值 */
				motor_profile_apply_info(&pobj->motor_info);
				need_init_default = false;
			}
			/* lr != OK：Flash 数据损坏 → 走路径B */
		}
		/* last_sequence == 0：首次上电（Flash 无有效 flag）→ 走路径B */

		/* 路径B：首次上电——init + apply_default + save */
		if (need_init_default)
		{
			/* B1. 填默认值（header 元数据 magic/version/blocks 索引 + 通用默认值） */
			(void)motor_info_init(&pobj->motor_info);

			/* B2. profile 无条件覆盖（含 config_version 设置）
			 *     无视 init 的非零默认值，强制用 profile 覆盖 7 个字段 */
			motor_profile_apply_info_default(&pobj->motor_info);

			/* B3. 回写 Flash（下次上电走路径A，直接 load）
			 *     保存失败不阻断启动（内存数据已正确） */
			if (pobj->inited)
			{
				(void)motorinfo_ops_save(pobj, &pobj->motor_info);
			}
		}

		pobj->motor_info_loaded = true;
	}

	return pobj->inited ? MOTOR_INFO_STORAGE_OK : MOTOR_INFO_STORAGE_ERR_FLASH;
}
```

- [ ] **Step 3: 更新 motorinfo_verify 增加 config_version 校验**

当前 `motor_info_storage.c:54-66` 的 motorinfo_verify 只校验 magic + CRC + range。
在 magic 校验后、CRC 校验前新增 config_version 校验：

当前代码（motor_info_storage.c:54-66）：
```c
static motor_info_storage_status_t motorinfo_verify(const motor_info_t *cfg)
{
	if (cfg->blocks.header.magic != PARAM_MAGIC)
		return MOTOR_INFO_STORAGE_NO_DATA;

	if (cfg->blocks.header.crc32 != motorinfo_crc32_compute(cfg))
		return MOTOR_INFO_STORAGE_CRC_FAIL;

	if (motor_info_validate(cfg) != 0)
		return MOTOR_INFO_STORAGE_RANGE_FAIL;

	return MOTOR_INFO_STORAGE_OK;
}
```

替换为：
```c
static motor_info_storage_status_t motorinfo_verify(const motor_info_t *cfg)
{
	if (cfg->blocks.header.magic != PARAM_MAGIC)
		return MOTOR_INFO_STORAGE_NO_DATA;

	/* config_version 校验：profile 版本不匹配视为无有效数据（触发重新初始化） */
	if (cfg->blocks.system.config_version != MOTOR_PROFILE_CONFIG_VERSION)
		return MOTOR_INFO_STORAGE_NO_DATA;

	if (cfg->blocks.header.crc32 != motorinfo_crc32_compute(cfg))
		return MOTOR_INFO_STORAGE_CRC_FAIL;

	if (motor_info_validate(cfg) != 0)
		return MOTOR_INFO_STORAGE_RANGE_FAIL;

	return MOTOR_INFO_STORAGE_OK;
}
```

- [ ] **Step 4: 验证编译**

```powershell
& "C:\APP\Code\MDK\CORE\UV4\UV4.exe" --% -b "d:\AAWorkSpace\001_JointMotor\SW\JointMotor\Board\SFOC\MDK-ARM\sfoc.uvprojx" -o "d:\AAWorkSpace\001_JointMotor\SW\JointMotor\Board\SFOC\MDK-ARM\build_log.txt"
```
Expected: 0 Error

---

## Task 3: 全量编译验证

**Files:** 无（仅验证）

- [ ] **Step 1: 清理中间文件避免缓存**

```powershell
$base = "d:\AAWorkSpace\001_JointMotor\SW\JointMotor\Board\SFOC\MDK-ARM\JointMotorApp"
Remove-Item -Path "$base\motor_info_storage.*","$base\motor_profile.*" -Force -ErrorAction SilentlyContinue
Get-Process -Name "UV4" -ErrorAction SilentlyContinue | Stop-Process -Force
```

- [ ] **Step 2: 全量编译**

```powershell
& "C:\APP\Code\MDK\CORE\UV4\UV4.exe" --% -b "d:\AAWorkSpace\001_JointMotor\SW\JointMotor\Board\SFOC\MDK-ARM\sfoc.uvprojx" -o "d:\AAWorkSpace\001_JointMotor\SW\JointMotor\Board\SFOC\MDK-ARM\build_log.txt"
```

- [ ] **Step 3: 检查编译结果**

Expected: `0 Error(s)`，警告数与基线（48）持平。

---

## 重构后行为对照表

### 四种上电场景

| 场景 | dev_flash last_sequence | load 校验 | 走哪条路径 | motor_info_init | apply_info_default | flash_save |
|------|------------------------|-----------|------------|-----------------|-------------------|-----------|
| 首次上电（Flash 全 0xFF） | 0 | — | 路径B | ✅ | ✅ | ✅ |
| 非首次上电（Flash 有效） | >0 | OK | 路径A | ❌ | ❌ | ❌ |
| 非首次上电（Flash 损坏） | >0 | CRC/RANGE FAIL | 路径B | ✅ | ✅ | ✅ |
| 固件升级（profile 变更） | >0 | NO_DATA(version不匹配) | 路径B | ✅ | ✅ | ✅ |

### 存储布局（dev_flash flag 偏移）

```
0x0804F000: dev_flash flag (magic=0x4D4F544F + sequence)     ← 8B
0x0804F008: motor_info_t.blocks.header (magic, version, crc32) ← 64B
0x0804F048: motor_info_t.blocks.system (config_version, ...)   ← 64B
0x0804F088: motor_info_t.blocks.motor_calib (pole_pairs, R...) ← 128B
0x0804F108: motor_info_t 其余字段                              ← ...
0x0804F408: (扇区内剩余空间，未使用)
```

调试器查看 0x0804F000 可见 flag，0x0804F008 开始是 motor_info_t 原始数据。

### 标定流程

| 步骤 | 操作 | motor_info 内存 | Flash |
|------|------|----------------|-------|
| 标定中 | motor_info_calib_submit_r(...) | ✅ 更新 | 不变 |
| 标定完成 | motor_info_calib_mark_calibrated() | ✅ is_calibrated=1 | 不变 |
| 上位机保存 | 0xEA → jm_app_motor_info_storage_save() | 不变 | ✅ dev_flash_write |
| 下次上电 | motor_info_storage_init() 路径A | ✅ 从 Flash 加载 | 不变 |

### 消除的问题

| 原问题 | 解决方式 |
|--------|---------|
| motor_info_init 非零默认值与零值判断冲突 | apply_info_default 无条件覆盖 |
| 非首次上电仍走 motor_info_init | last_sequence > 0 时直接用 Flash 数据 |
| 清零 7 字段硬编码（DRY 违反） | 删除硬编码，apply_info_default 负责覆盖 |
| profile 升级后旧数据不更新 | config_version 不匹配时走路径B |

---

## Self-Review

**1. Spec coverage:**
- 通过 dev_flash 调用 flash 接口：✅ 决策1（保留 dev_flash，单扇区配置）
- 首次上电 flash 检测和初始化：✅ 路径B（last_sequence==0 → init + apply_default + save）
- motor_profile 默认参数加载条件判断：✅ apply_info_default（无条件）+ apply_info（零值fallback）
- 标定完成后参数同步和保存：✅ 决策6（内存更新 + 0xEA 保存）
- 非首次上电 Flash 有效不调 motor_info_init：✅ 路径A（last_sequence>0 + 校验通过 → 直接用）

**2. Placeholder scan:** 无占位符。所有代码完整。

**3. Type consistency:**
- `motor_profile_apply_info_default(void *cfg)` Task 1 声明和定义一致
- `MOTOR_PROFILE_CONFIG_VERSION` Task 1 定义，Task 2 使用，名称一致
- `pobj->flash_dev.last_sequence` dev_flash.h:58 定义，Task 2 使用，字段名一致
- `cfg->blocks.system.config_version` motor_info.c:43 初始化，Task 2 使用，路径一致
- `MOTORINFO_FLASH_PAGE_SIZE` motor_info_storage.h:68 定义，Task 2 用作 total_size 参数
- `motorinfo_ops_load` / `motorinfo_ops_save` 已在 motor_info_storage.c 中定义，Task 2 调用，名称一致

**4. 调用链完整性核对：**
- user_interface.c:35 `motor_info_storage_init()` → 无条件调用 ✅
- jm_proto_ops.c:711 `jm_app_motor_info_storage_save(motor_info_storage_get())` → 强符号覆盖 ✅
- motor_info_calib.c:17 `motor_info_storage_get()` → USE_DEV_FLASH 条件编译 ✅
- motor_info_storage.h 条件编译保持 USE_DEV_FLASH → 与 motor_info_calib.c 一致 ✅
