# motor_info Flash 持久化（通用 dev_flash + 应用层存储服务）Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 重新实现 `dev_flash.c/h` 为通用 Flash 页存储设备（init 只传起始地址+总大小+页大小，不耦合 boot/app 分区），在应用层 `User/AppServices/ParamService/motor_info_storage.c/h` 定义 motor_info 存储地址并实现 CRC32 校验 + 上电加载 + 0xEA 保存。

**Architecture:** 三层分离：
- **Driver 层** `drv_flash_g4.c` — 底层 Flash 操作（不动）
- **Device 层** `dev_flash.c/h` — 通用页存储设备，init 传入 `(start_addr, total_size, page_size)`，内部用 Sequence Number 实现磨损均衡，不感知任何业务分区
- **App 层** `motor_info_storage.c/h`（新建）— 定义存储地址宏 + CRC32/magic/字段校验 + 强符号覆盖

Flash 分区（链接脚本预留数据区）由用户在 Keil 中自行配置，本计划不涉及链接脚本。

**Tech Stack:** STM32G474 HAL Flash、C99、Keil MDK-ARM、dev_flash_t OOP 设备接口。

---

## 关键事实（实现前必读）

### drv_flash_g4 API（已实现，底层不动）

| 函数 | 签名 | 说明 |
|---|---|---|
| `drv_flash_read` | `u8 (u32 addr, u64 *pdata64, u32 len_64)` | 读，`len_64` 是 u64 个数 |
| `drv_flash_write` | `u8 (u32 addr, u64 *pdata64, u32 len_64)` | 写（自动读改擦写，关中断） |
| `drv_flash_clear` | `u8 (u32 addr, u8 len, u8 bank)` | 擦 `len` 页 |
| `drv_g4_flash_page_size` | `u32 (void)` | 返回页大小（双Bank 2KB / 单Bank 4KB） |
| `drv_g4_flash_get_bank` | `u8 (u32 addr)` | 返回 Bank 编号 1/2，越界 0xFF |
| `FLASH_ERR_OK` | `0U` | 成功码 |

### motor_info_t 结构（1024B）

| 偏移 | 字段 | 说明 |
|---|---|---|
| 0x0000 | `blocks.header.magic` (4B) | `PARAM_MAGIC = 0x53455256` |
| 0x0004 | `blocks.header.version_major/minor` (4B) | 版本 |
| 0x0008 | `blocks.header.crc32` (4B) | CRC32（校验时跳过本字段） |
| ... | 其余子块 | 见 motor_info.h |
| **合计** | **1024B** | `PARAM_AREA_SIZE = 1024`，访问 `cfg->raw[1024]` 可整块 memcpy |

### CRC32 计算策略

`utils_crc32c(uint8_t *data, uint32_t len)` 接受连续缓冲区。crc32 字段在 header 偏移 8（中间）。临时缓冲区法：memcpy 1024B → crc32 置零 → 对整块求值。save/verify 用同一函数保证一致。

### 协议层弱符号（待覆盖）

[jm_proto_ops.c:679](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/Protocol/joint_proto/jm_proto_ops.c#L679) 定义弱符号 `jm_app_motor_info_storage_save(const motor_info_t *cfg)`，仅校验不落盘。本计划在 `motor_info_storage.c` 提供强符号覆盖。

### dev_flash 旧版 Bug 清单（本次重写全部修复）

| # | 问题 |
|---|---|
| C1 | `data_sector_start_address = ... + 1`，应为绝对地址 |
| C2 | 地址计算依赖 boot/app/backup 配置表，不通用 |
| C3/C4 | drv_flash_read/write 第3参数传字节数而非 u64 个数，读 16KB 越界 |
| C5 | `change_sector_buffer` 为 `uint32_t*` 接 `u64` 数据，截断 |
| C6 | malloc 后无 NULL 检查 |
| C7/C8/C10 | 磨损均衡失效：写入未设有效标志、init 找"第一个空扇区"读到空数据 |
| H1 | 头文件守卫 `__DRV_FLASH_H__` 命名误导 |
| H3 | `FLASH_SECTOR0 = -1` 负值作索引 |

---

## Sequence Number 磨损均衡方案

### Flag 布局（每个扇区第一个 u64）

```c
#define FLASH_FLAG_MAGIC  0x4D4F544Fu  /* "MOTO" */
#define FLASH_FLAG_MAKE(seq)    (((uint64_t)FLASH_FLAG_MAGIC << 32) | ((seq) & 0xFFFFFFFFu))
#define FLASH_FLAG_IS_VALID(f)  (((uint32_t)((f) >> 32)) == FLASH_FLAG_MAGIC)
#define FLASH_FLAG_SEQ(f)       ((uint32_t)((f) & 0xFFFFFFFFu))
```
- 擦除态 `0xFFFF...F` → magic 不匹配 → 无效
- 有效态 magic 匹配 + seq > 0

### Init 逻辑

```
init(pobj, start_addr, total_size, page_size):
  扇区数 = total_size / page_size
  扫描所有扇区 flag → 找 seq 最大的 → last_sector
  全无效 → last_sector=0, seq=0 (首次上电)
```

### Write 逻辑

```
读当前扇区整页 → 修改 offset 区域 → flag = MAKE(seq+1) → 轮转下一扇区 → drv_flash_write
```

---

## 文件结构

| 文件 | 职责 | 动作 |
|---|---|---|
| `User/Devices/dev_flash.h` | 通用设备接口（init 传地址+大小+页大小） | 重写 |
| `User/Devices/dev_flash.c` | 设备实现（磨损均衡） | 重写 |
| `User/AppServices/ParamService/motor_info_storage.h` | 应用层接口 + 存储地址宏 | 创建 |
| `User/AppServices/ParamService/motor_info_storage.c` | CRC + load/save + 强符号 | 创建 |
| `User/AppEntry/user_interface.c` | `hardware_init` 增加 init 调用 | 修改 |
| `User/Protocol/joint_proto/jm_proto_ops.c` | 懒加载增加 Flash 加载 | 修改 |

Flash 分区配置（链接脚本、Keil Target Options）由用户自行完成，不在本计划内。

---

## Task 1: 重写 dev_flash.h（通用化接口）

**Files:**
- Modify: `User/Devices/dev_flash.h`
- Modify: `Board/SFOC/Config/dev_config_board.h`（启用宏）
- Modify: `User/Driver/drv_config.h:33`（启用宏）

- [ ] **Step 1: 启用 dev_flash 与 flash_g4 驱动宏**

修改 [dev_config_board.h](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/Board/SFOC/Config/dev_config_board.h)，在 `#endif /* __DEV_CONFIG_BOARD_SFOC_H__ */` 之前添加 `#define USE_DEV_FLASH`：

将
```c
#define USE_DEV_COMMUN_UART

#endif /* __DEV_CONFIG_BOARD_SFOC_H__ */
```
改为
```c
#define USE_DEV_COMMUN_UART
#define USE_DEV_FLASH

#endif /* __DEV_CONFIG_BOARD_SFOC_H__ */
```

修改 [drv_config.h:33](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/Driver/drv_config.h#L33)：

将
```c
//#define USE_FLASH_G4_DRIVER
```
改为
```c
#define USE_FLASH_G4_DRIVER
```

- [ ] **Step 2: 重写 dev_flash.h 全文**

将 `User/Devices/dev_flash.h` 全文替换为：

```c
/**
 * @file        dev_flash.h
 * @brief       通用片内 Flash 页存储设备（磨损均衡）
 *
 * @author      Dalin (dalinyy@163.com)
 * @version     2.0
 * @date        2026-07-02
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @par 修改日志:
 * | 日期       | 版本 | 作者  | 修改内容                                   |
 * |------------|------|-------|--------------------------------------------|
 * | 2026-06-11 | 1.0  | --    | 初始创建（依赖 boot/app 分区表，不通用）   |
 * | 2026-07-02 | 2.0  | Dalin | 重写：init 传地址+大小，通用化；修复10处bug|
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 * @note        每个扇区第一个 u64 为标志位(magic+sequence)，数据从第 2 个 u64 起。
 *              写入时轮转扇区实现磨损均衡；init 扫描所有扇区找 sequence 最大的有效扇区。
 *              设备不感知业务分区（boot/app），调用方传入起始地址+总大小+页大小。
 */
#ifndef __DEV_FLASH_H__
#define __DEV_FLASH_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "dev_config.h"

#if defined(USE_DEV_FLASH)

#include "drv_flash_g4.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#define WORD_SIZE 8U

/* 扇区标志位：magic(高32) + sequence(低32)
 * 擦除态 0xFFFF...F → magic 不匹配 → 无效
 * 有效态 magic 匹配 + seq > 0 */
#define FLASH_FLAG_MAGIC  0x4D4F544Fu  /* "MOTO" */
#define FLASH_FLAG_MAKE(seq)    (((uint64_t)FLASH_FLAG_MAGIC << 32) | ((seq) & 0xFFFFFFFFu))
#define FLASH_FLAG_IS_VALID(f)  (((uint32_t)((f) >> 32)) == FLASH_FLAG_MAGIC)
#define FLASH_FLAG_SEQ(f)       ((uint32_t)((f) & 0xFFFFFFFFu))

/**
 * @brief  flash 设备句柄结构体
 */
typedef struct dev_flash
{
    uint32_t start_addr;     /* 数据区起始绝对地址(init 时传入) */
    uint32_t total_size;     /* 数据区总大小(字节) */
    uint32_t page_size;      /* 单页大小(字节) */
    uint32_t sector_count;   /* 扇区数 = total_size / page_size */
    uint32_t last_sector;    /* 当前有效扇区索引 */
    uint32_t last_sequence;  /* 当前有效扇区的 sequence 号 */
    bool inited;             /* 初始化完成标志 */

    int (*flash_write)(struct dev_flash *pobj, u32 offset, u64 *data, u16 size);
    int (*flash_read)(struct dev_flash *pobj, u32 offset, u64 *data, u16 size);
} dev_flash_t;

/**
 * @brief  初始化 flash 设备(传入地址+大小+页大小，扫描扇区定位最新有效数据)
 * @param  pobj       flash 设备句柄
 * @param  start_addr 数据区起始绝对地址(须页对齐)
 * @param  total_size 数据区总大小(字节，须为 page_size 的整数倍)
 * @param  page_size  单页大小(字节，须等于 drv_g4_flash_page_size() 返回值)
 */
void dev_flash_init(struct dev_flash *pobj, uint32_t start_addr, uint32_t total_size, uint32_t page_size);

#ifdef __cplusplus
}
#endif

#endif /* USE_DEV_FLASH */
#endif /* __DEV_FLASH_H__ */
```

- [ ] **Step 3: 编译验证**

编译 sfoc 工程。
Expected: dev_flash.c 尚未重写，可能因接口变化报错（dev_flash_read/write 签名变化），先继续 Task 2。

- [ ] **Step 4: 提交**

```bash
git add User/Devices/dev_flash.h Board/SFOC/Config/dev_config_board.h User/Driver/drv_config.h
git commit -m "refactor(dev_flash): 重写头文件为通用接口(init 传地址+大小+页大小)"
```

---

## Task 2: 重写 dev_flash.c

**Files:**
- Modify: `User/Devices/dev_flash.c`

- [ ] **Step 1: 重写 dev_flash.c 全文（修复 C1-C10）**

将 `User/Devices/dev_flash.c` 全文替换为：

```c
/**
 * @file        dev_flash.c
 * @brief       通用片内 Flash 页存储设备（磨损均衡）
 *
 * @author      Dalin (dalinyy@163.com)
 * @version     2.0
 * @date        2026-07-02
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @par 修改日志:
 * | 日期       | 版本 | 作者  | 修改内容                                   |
 * |------------|------|-------|--------------------------------------------|
 * | 2026-06-11 | 1.0  | --    | 初始创建                                   |
 * | 2026-07-02 | 2.0  | Dalin | 重写：通用化接口+修复地址/u64计数/磨损均衡 |
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 * @note        每扇区首 u64 = flag(magic+seq)，数据从第 2 个 u64 起。
 *              写入轮转扇区实现磨损均衡；init 扫描找 seq 最大的有效扇区。
 */
#include "dev_flash.h"
#include "assert_report.h"

#if defined(USE_DEV_FLASH)

/*
 * @brief  读取 flash 指定偏移地址长度的值
 * @param  pobj   : FLASH 设备句柄
 * @param  offset : 扇区内字节偏移(从数据区起算，不含 flag)
 * @param  data   : 数据 buffer
 * @param  size   : 读取长度(u64 个数)
 * @return DEV_EOK 成功, DEV_ERROR 失败
 */
int dev_flash_read(struct dev_flash *pobj, u32 offset, u64 *data, u16 size)
{
    if (pobj == NULL || data == NULL || !pobj->inited)
        return DEV_ERROR;

    /* 边界：offset(bytes) + size(u64) 不超过 (页大小 - flag 的 1 个 u64) */
    uint32_t usable = pobj->page_size - WORD_SIZE;
    if ((uint64_t)offset + (uint64_t)size * WORD_SIZE > usable)
        return DEV_ERROR;

    /* 绝对地址 = 起址 + 当前扇区 * 页大小 + flag(WORD_SIZE) + offset */
    uint32_t addr = pobj->start_addr
                   + pobj->last_sector * pobj->page_size
                   + WORD_SIZE  /* 跳过 flag u64 */
                   + offset;

    return (drv_flash_read(addr, data, size) == FLASH_ERR_OK) ? DEV_EOK : DEV_ERROR;
}

/*
 * @brief  写入 flash 设备指定偏移地址和长度的数据
 * @param  pobj   : FLASH 设备句柄
 * @param  offset : 扇区内字节偏移(从数据区起算，不含 flag)
 * @param  data   : 待写入 buffer
 * @param  size   : 写入长度(u64 个数)
 * @return DEV_EOK 成功, DEV_ERROR 失败
 * @note   读当前扇区整页 → 修改指定区域 → 设 flag(new_seq) → 轮转下一扇区 → 擦写
 */
int dev_flash_write(struct dev_flash *pobj, u32 offset, u64 *data, u16 size)
{
    if (pobj == NULL || data == NULL || !pobj->inited)
        return DEV_ERROR;

    uint32_t page_u64 = pobj->page_size / WORD_SIZE;  /* 页内 u64 个数 */
    uint32_t usable   = pobj->page_size - WORD_SIZE;   /* 扣除 flag 后可用字节 */

    if ((uint64_t)offset + (uint64_t)size * WORD_SIZE > usable)
        return DEV_ERROR;

    /* 1. 分配页缓冲 */
    u64 *page_buf = (u64 *)malloc(pobj->page_size);
    if (page_buf == NULL)
        return DEV_ERROR;

    /* 2. 读当前扇区整页(首次上电 last_sector 无效时读到 0xFF) */
    uint32_t cur_addr = pobj->start_addr + pobj->last_sector * pobj->page_size;
    if (drv_flash_read(cur_addr, page_buf, page_u64) != FLASH_ERR_OK)
    {
        free(page_buf);
        return DEV_ERROR;
    }

    /* 3. 修改缓冲中的目标数据(跳过 flag u64) */
    uint32_t data_idx = (WORD_SIZE + offset) / WORD_SIZE;
    for (uint16_t i = 0; i < size; i++)
    {
        page_buf[data_idx + i] = data[i];
    }

    /* 4. 设置 flag = magic + new_sequence */
    uint32_t new_seq = pobj->last_sequence + 1U;
    page_buf[0] = FLASH_FLAG_MAKE(new_seq);

    /* 5. 轮转到下一扇区(磨损均衡) */
    uint32_t next = (pobj->last_sector + 1U) % pobj->sector_count;
    uint32_t write_addr = pobj->start_addr + next * pobj->page_size;

    /* 6. 写入新扇区(drv_flash_write 内部读改擦写，关中断) */
    u8 ret = drv_flash_write(write_addr, page_buf, page_u64);
    free(page_buf);

    if (ret != FLASH_ERR_OK)
        return DEV_ERROR;

    /* 7. 更新状态 */
    pobj->last_sector   = next;
    pobj->last_sequence = new_seq;

    return DEV_EOK;
}

/*
 * @brief  初始化 flash 设备
 * @param  pobj       : flash 设备句柄
 * @param  start_addr : 数据区起始绝对地址(须页对齐)
 * @param  total_size : 数据区总大小(字节)
 * @param  page_size  : 单页大小(字节)
 * @details 计算扇区数 → 扫描所有扇区找 seq 最大的有效扇区 → 设 last_sector
 */
void dev_flash_init(struct dev_flash *pobj, uint32_t start_addr, uint32_t total_size, uint32_t page_size)
{
    assert_report(pobj);
    pobj->inited = false;

    /* 1. 参数校验 */
    if (page_size == 0U || total_size < page_size)
        return;
    if ((start_addr % page_size) != 0U)
        return;
    if ((total_size % page_size) != 0U)
        return;

    pobj->start_addr   = start_addr;
    pobj->total_size   = total_size;
    pobj->page_size    = page_size;
    pobj->sector_count = total_size / page_size;

    /* 2. 扫描所有扇区，找 sequence 最大的有效扇区 */
    uint32_t best_seq = 0;
    int best_sector = -1;

    for (uint32_t i = 0; i < pobj->sector_count; i++)
    {
        uint32_t addr = pobj->start_addr + i * pobj->page_size;
        u64 flag;
        if (drv_flash_read(addr, &flag, 1) != FLASH_ERR_OK)
            return;  /* Flash 读取失败，保持 inited=false */

        if (FLASH_FLAG_IS_VALID(flag))
        {
            uint32_t seq = FLASH_FLAG_SEQ(flag);
            if (seq >= best_seq)  /* >= 处理回绕：后者覆盖前者 */
            {
                best_seq = seq;
                best_sector = (int)i;
            }
        }
    }

    if (best_sector >= 0)
    {
        pobj->last_sector   = (uint32_t)best_sector;
        pobj->last_sequence = best_seq;
    }
    else
    {
        /* 首次上电：无有效数据，下次写入从 sector 0 开始 */
        pobj->last_sector   = 0U;
        pobj->last_sequence = 0U;
    }

    pobj->inited     = true;
    pobj->flash_read  = dev_flash_read;
    pobj->flash_write = dev_flash_write;
}

#endif /* USE_DEV_FLASH */
```

- [ ] **Step 2: 将 dev_flash.c 添加到 Keil 工程（若未添加）**

在 Keil IDE 中打开 `Board/SFOC/MDK-ARM/sfoc.uvprojx`：
1. Project 面板 → 右键 `User/Devices` 分组 → Add Existing Files
2. 选择 `User/Devices/dev_flash.c`
3. 保存工程

- [ ] **Step 3: 编译验证**

编译 sfoc 工程。
Expected: 0 errors。dev_flash.c 自包含，无外部调用者，编译应通过。

- [ ] **Step 4: 提交**

```bash
git add User/Devices/dev_flash.c Board/SFOC/MDK-ARM/sfoc.uvprojx
git commit -m "refactor(dev_flash): 重写实现，通用化接口+修复地址/u64计数/磨损均衡bug"
```

---

## Task 3: 创建 motor_info_storage 应用层服务（.h + .c 合并）

**Files:**
- Create: `User/AppServices/ParamService/motor_info_storage.h`
- Create: `User/AppServices/ParamService/motor_info_storage.c`

- [ ] **Step 1: 创建 motor_info_storage.h（含存储地址宏定义）**

创建 `User/AppServices/ParamService/motor_info_storage.h`：

```c
/**
 * @file        motor_info_storage.h
 * @brief       motor_info Flash 持久化应用服务（基于通用 dev_flash 设备）
 *
 * @details     应用层服务，封装 motor_info_t 的 Flash 持久化语义：
 *              - 定义 motor_info 在 Flash 中的存储地址（末 4KB，2 扇区磨损均衡）
 *              - 上电时 motor_info_storage_init() 初始化 dev_flash 设备
 *              - motor_info_storage_load() 从 Flash 加载并三重校验(magic+CRC+字段)
 *              - motor_info_storage_save() 校验后计算 CRC 写入 Flash
 *              - 强符号 jm_app_motor_info_storage_save 覆盖协议层弱符号，接入 0xEA
 *
 * @note        Flash 分区由 Keil 链接脚本预留（用户自行配置）。
 *              存储地址定义在本头文件的宏中，与应用逻辑同模块。
 *
 * @author      Dalin (dalinyy@163.com)
 * @version     1.0
 * @date        2026-07-02
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @par 修改日志:
 * | 日期       | 版本 | 作者  | 修改内容   |
 * |------------|------|-------|------------|
 * | 2026-07-02 | 1.0  | Dalin | 初始创建   |
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 */
#ifndef __MOTOR_INFO_STORAGE_H__
#define __MOTOR_INFO_STORAGE_H__

#include "dev_config.h"

#if defined(USE_DEV_FLASH)

#include "motor_info.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===== motor_info Flash 存储地址定义 =====
 * G474 512KB Flash 末尾 4KB 保留为 motor_info 数据区：
 *   起始地址 0x0807F000，总大小 4KB，页大小 2KB（双Bank）
 *   2 个扇区 A/B 轮转磨损均衡
 * 注意：需在 Keil 链接脚本中将代码区大小设为 0x7F000，
 *       避免代码段溢出到数据区。*/
#define MOTORINFO_FLASH_START_ADDR  0x0807F000U
#define MOTORINFO_FLASH_TOTAL_SIZE  0x00001000U  /* 4KB */
#define MOTORINFO_FLASH_PAGE_SIZE   2048U        /* 2KB，须等于 drv_g4_flash_page_size() */

/**
 * @brief  初始化 motor_info Flash 存储服务
 * @details 内部调用 dev_flash_init(地址, 大小, 页大小) 扫描扇区定位最新有效数据。
 *          须在 motor_loop_init 之前、hardware_init 中调用。
 * @return 0=成功, -1=dev_flash 初始化失败
 */
int motor_info_storage_init(void);

/**
 * @brief  从 Flash 加载 motor_info 配置
 * @param  cfg  目标参数区指针（成功时整块覆盖）
 * @return 0=加载成功且校验通过,
 *         1=Flash 无有效数据(首次上电或全擦除)，调用方回退默认值,
 *         2=CRC 校验失败,
 *         3=字段范围校验失败,
 *        -1=参数空指针,
 *        -2=Flash 读取失败,
 *        -3=服务未初始化
 */
int motor_info_storage_load(motor_info_t *cfg);

/**
 * @brief  将 motor_info 配置保存到 Flash
 * @param  cfg  源参数区指针
 * @return 0=保存成功,
 *         >0=motor_info_validate 返回的首个越界 param_id,
 *        -1=参数空指针,
 *        -2=CRC 计算失败,
 *        -3=Flash 写入失败,
 *        -4=服务未初始化
 * @note   dev_flash_write 内部关中断约 10-30ms，仅在 0xEA 命令时调用。
 */
int motor_info_storage_save(const motor_info_t *cfg);

#ifdef __cplusplus
}
#endif

#endif /* USE_DEV_FLASH */
#endif /* __MOTOR_INFO_STORAGE_H__ */
```

- [ ] **Step 2: 创建 motor_info_storage.c**

创建 `User/AppServices/ParamService/motor_info_storage.c`：

```c
/**
 * @file        motor_info_storage.c
 * @brief       motor_info Flash 持久化应用服务实现
 *
 * @author      Dalin (dalinyy@163.com)
 * @version     1.0
 * @date        2026-07-02
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @par 修改日志:
 * | 日期       | 版本 | 作者  | 修改内容   |
 * |------------|------|-------|------------|
 * | 2026-07-02 | 1.0  | Dalin | 初始创建   |
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 */
#include "motor_info_storage.h"

#if defined(USE_DEV_FLASH)

#include "dev_flash.h"
#include "utils.h"
#include <string.h>

/* motor_info_t = 1024B = 128 个 u64 */
#define MOTORINFO_LEN_U64  (PARAM_AREA_SIZE / 8U)

/* 模块静态状态 */
static dev_flash_t s_flash_dev;
static uint8_t s_inited = 0U;

/* ===== 内部 CRC32 工具 ===== */

/**
 * @brief  计算 motor_info_t 的 CRC32（跳过 header.crc32 字段）
 * @details 拷贝到临时缓冲 → crc32 置零 → 对 1024B 用 utils_crc32c 求值
 */
static uint32_t motorinfo_crc32_compute(const motor_info_t *cfg)
{
    motor_info_t tmp;
    memcpy(&tmp, cfg, PARAM_AREA_SIZE);
    tmp.blocks.header.crc32 = 0U;
    return utils_crc32c(tmp.raw, PARAM_AREA_SIZE);
}

/**
 * @brief  三重校验：magic → CRC32 → 字段范围
 * @return 0=通过, 1=无数据(magic不符), 2=CRC失败, 3=范围越界
 */
static int motorinfo_verify(const motor_info_t *cfg)
{
    if (cfg->blocks.header.magic != PARAM_MAGIC)
        return 1;

    if (cfg->blocks.header.crc32 != motorinfo_crc32_compute(cfg))
        return 2;

    if (motor_info_validate(cfg) != 0)
        return 3;

    return 0;
}

/* ===== 公开 API ===== */

int motor_info_storage_init(void)
{
    dev_flash_init(&s_flash_dev,
                   MOTORINFO_FLASH_START_ADDR,
                   MOTORINFO_FLASH_TOTAL_SIZE,
                   MOTORINFO_FLASH_PAGE_SIZE);
    s_inited = s_flash_dev.inited ? 1U : 0U;
    return s_inited ? 0 : -1;
}

int motor_info_storage_load(motor_info_t *cfg)
{
    if (cfg == NULL)
        return -1;
    if (s_inited == 0U)
        return -3;

    /* 从 dev_flash 读 1024B = 128 u64，offset=0 */
    int rc = s_flash_dev.flash_read(&s_flash_dev, 0, (u64 *)cfg, MOTORINFO_LEN_U64);
    if (rc != DEV_EOK)
        return -2;

    /* 三重校验 */
    return motorinfo_verify(cfg);
}

int motor_info_storage_save(const motor_info_t *cfg)
{
    if (cfg == NULL)
        return -1;
    if (s_inited == 0U)
        return -4;

    /* 1. 保存前范围校验 */
    int vrc = motor_info_validate(cfg);
    if (vrc != 0)
        return vrc;  /* >0: 越界 param_id */

    /* 2. 计算 CRC32 写入临时副本 */
    motor_info_t tmp;
    memcpy(&tmp, cfg, PARAM_AREA_SIZE);
    tmp.blocks.header.crc32 = motorinfo_crc32_compute(&tmp);

    /* 3. 通过 dev_flash 写入(内部轮转扇区+磨损均衡) */
    int rc = s_flash_dev.flash_write(&s_flash_dev, 0, (u64 *)&tmp, MOTORINFO_LEN_U64);
    return (rc == DEV_EOK) ? 0 : -3;
}

/* ===== 强符号覆盖: jm_app_motor_info_storage_save =====
 * 覆盖 jm_proto_ops.c 中的 __weak jm_app_motor_info_storage_save()。
 * 协议层收到 0xEA 时调用本强符号，经 dev_flash 落盘。
 * 返回值约定：0=成功, >0=越界 param_id, <0=错误。*/
int jm_app_motor_info_storage_save(const motor_info_t *cfg)
{
    return motor_info_storage_save(cfg);
}

#endif /* USE_DEV_FLASH */
```

- [ ] **Step 3: 将 motor_info_storage.c 添加到 Keil 工程**

在 Keil IDE 中：
1. Project 面板 → 右键 `User/AppServices/ParamService` 分组（或合适分组）→ Add Existing Files
2. 选择 `User/AppServices/ParamService/motor_info_storage.c`
3. 保存工程

- [ ] **Step 4: 编译验证**

编译 sfoc 工程。
Expected: 0 errors。`jm_app_motor_info_storage_save` 强符号已定义，将覆盖 jm_proto_ops.c 中的弱符号。

- [ ] **Step 5: 提交**

```bash
git add User/AppServices/ParamService/motor_info_storage.h User/AppServices/ParamService/motor_info_storage.c Board/SFOC/MDK-ARM/sfoc.uvprojx
git commit -m "feat(storage): 创建 motor_info_storage 应用服务(含存储地址定义+CRC+强符号)"
```

---

## Task 4: 接入上电初始化与协议层懒加载

**Files:**
- Modify: `User/AppEntry/user_interface.c`
- Modify: `User/Protocol/joint_proto/jm_proto_ops.c`

- [ ] **Step 1: 在 user_interface.c 添加头文件包含**

修改 [user_interface.c](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/AppEntry/user_interface.c) 的 include 区块（第 20-26 行）：

将
```c
#include "user_interface.h"
#include "thread_management.h"
#include "dev_dwt_counter.h"
#include "motor_loop.h"
#include "motor_loop_config.h"
#include "runtime_param.h"
#include "tim.h"
```

改为
```c
#include "user_interface.h"
#include "thread_management.h"
#include "dev_dwt_counter.h"
#include "motor_info_storage.h"
#include "motor_loop.h"
#include "motor_loop_config.h"
#include "runtime_param.h"
#include "tim.h"
```

- [ ] **Step 2: 在 hardware_init 中添加 motor_info_storage_init**

修改 `hardware_init()` 函数（第 28-36 行）：

将
```c
static void hardware_init(void)
{
	/* 初始化DWT定时器 */
	dev_dwt_counter_init();

	/* 初始化电机三环控制（dev_motor + 状态机 + 级联控制） */
	motor_loop_init(10000.0f);
}
```

改为
```c
static void hardware_init(void)
{
	/* 初始化DWT定时器 */
	dev_dwt_counter_init();

	/* 初始化 motor_info Flash 存储服务（须在 motor_loop_init 之前） */
	motor_info_storage_init();

	/* 初始化电机三环控制（dev_motor + 状态机 + 级联控制） */
	motor_loop_init(10000.0f);
}
```

- [ ] **Step 3: 在 jm_proto_ops.c 添加头文件包含**

修改 [jm_proto_ops.c:36](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/Protocol/joint_proto/jm_proto_ops.c#L36)：

将
```c
#include "motor_info.h"		   /* motor_info_t / motor_info_init / motor_info_dispatch_read/write */
```

改为
```c
#include "motor_info.h"		   /* motor_info_t / motor_info_init / motor_info_dispatch_read/write */
#include "motor_info_storage.h"  /* motor_info_storage_load: 上电从 Flash 加载 */
```

- [ ] **Step 4: 修改 app_motor_info 懒加载函数**

修改 [jm_proto_ops.c:661-670](file:///d:/AAWorkSpace/001_JointMotor/SW/JointMotor/User/Protocol/joint_proto/jm_proto_ops.c#L661-L670)：

将
```c
static motor_info_t *app_motor_info(void)
{
	if (g_motor_info_inited == 0u)
	{
		(void)motor_info_init(&g_motor_info);
		motor_profile_apply_info(&g_motor_info);
		g_motor_info_inited = 1u;
	}
	return &g_motor_info;
}
```

改为
```c
static motor_info_t *app_motor_info(void)
{
	if (g_motor_info_inited == 0u)
	{
		/* 1. 先填默认值（含 header.magic/version/block索引表） */
		(void)motor_info_init(&g_motor_info);

		/* 2. 尝试从 Flash 加载覆盖默认值
		 *    返回 0=加载成功；返回 1=无数据(首次上电)；返回 2/3=数据损坏
		 *    任何非 0 都保留默认值，系统仍可正常启动 */
		(void)motor_info_storage_load(&g_motor_info);

		/* 3. 施加编译期 motor_profile 覆盖（硬件级电气身份参数，最终决定权） */
		motor_profile_apply_info(&g_motor_info);

		g_motor_info_inited = 1u;
	}
	return &g_motor_info;
}
```

- [ ] **Step 5: 编译验证**

编译 sfoc 工程。
Expected: 0 errors。链接器选用 `motor_info_storage.o` 中的强符号 `jm_app_motor_info_storage_save`，覆盖 `jm_proto_ops.o` 中的弱符号。

- [ ] **Step 6: 提交**

```bash
git add User/AppEntry/user_interface.c User/Protocol/joint_proto/jm_proto_ops.c
git commit -m "feat(app): 接入上电初始化与协议层懒加载 Flash 加载"
```

---

## Task 5: 全量编译与符号检查

**Files:**
- 无修改，仅验证

- [ ] **Step 1: 全量 Rebuild sfoc 工程**

在 Keil 中执行 Rebuild All。
Expected: 0 errors, 0 warnings（新增）。

- [ ] **Step 2: 检查 map 文件符号归属**

打开 `Board/SFOC/MDK-ARM/sfoc.map`，搜索 `jm_app_motor_info_storage_save`：
Expected: 符号定义在 `motor_info_storage.o`，`jm_proto_ops.o` 中的弱符号被覆盖。

- [ ] **Step 3: 检查 Flash 数据区未被代码段占用**

在 `sfoc.map` 中检查 `0x0807F000`-`0x08080000`（4KB 数据区）范围内是否有 `.text`/`.data` 段：
Expected: 该范围内无代码/数据段，专供 dev_flash 磨损均衡使用。
若代码段溢出到该区域，需在 Keil 中调整链接脚本缩小代码区。

---

## Task 6: 硬件验证

**Files:**
- 无修改，仅硬件测试

- [ ] **Step 1: 烧录固件，首次上电行为**

用上位机发 0xE6 读 pole_pairs。
Expected: Flash 为空（flag=0xFF），`motorinfo_verify` 返回 1，系统 fallback 默认值。读到 pole_pairs=7。

- [ ] **Step 2: 上位机修改并保存**

1. 0xE7 写 pole_pairs=14
2. 0xEA 保存到 Flash
Expected: 0xEA 返回 ACK。dev_flash 轮转到扇区 0，写入 flag+data。

- [ ] **Step 3: 断电重启验证加载**

断电后上电，0xE6 读 pole_pairs。
Expected: 读到 14（从 Flash 加载，init 扫描找到扇区 0 的有效 flag）。

- [ ] **Step 4: 验证磨损均衡轮转**

重复 Step 2 多次保存（改不同值），断电重启验证每次都读到最新值。
Expected: 每次保存轮转扇区（0→1→0→1...），数据始终可正确加载。

- [ ] **Step 5: 验证 CRC 防护**

用 ST-Link Utility 修改 Flash 数据区某字节（破坏 CRC），重新上电。
Expected: `motorinfo_verify` 返回 2，系统 fallback 默认值，不加载损坏数据。

---

## 设计决策说明

### 为什么 dev_flash 通用化（传地址+大小）？

原 dev_flash 依赖 `flash_list` 配置表（boot/app/backup/page_size），导致：
- 不通用：换项目要改配置表
- 职责不清：设备层感知业务分区

通用化后：dev_flash 只需 `(start_addr, total_size, page_size)`，业务地址由应用层定义。dev_flash 可被任何模块复用。

### 为什么存储地址定义在 motor_info_storage.h？

应用层最清楚自己需要多大空间、存在哪里。dev_flash 作为通用设备不关心。motor_info_storage.h 定义 `MOTORINFO_FLASH_START_ADDR/TOTAL_SIZE/PAGE_SIZE` 三个宏，与 motor_info 业务逻辑同模块，符合"谁用谁定义"原则。

### 为什么用 Sequence Number 磨损均衡？

- 每个 flag 含递增 sequence，init 找 seq 最大的有效扇区 = 最新数据
- 无需擦除旧扇区（旧扇区 seq 较低，自然被忽略）
- 处理回绕：`>=` 比较使后写的覆盖先写的

### 为什么 CRC32 用临时缓冲区法？

`utils_crc32c(data, len)` 接受连续缓冲区。crc32 字段在 header 偏移 8（中间）。临时缓冲区法：memcpy 1024B → crc32 置零 → 对整块求值。不改公共 API，1024B 栈开销可接受。

### 为什么 motor_profile_apply_info 在 Flash 加载之后？

加载顺序：默认值 → Flash 用户配置 → 编译期 profile 覆盖。profile（R/Ld/Lq/flux/pole_pairs）是板级硬件固有参数，不应被 Flash 中的用户修改覆盖。profile 有最终决定权，防止用户保存错误电气参数导致 FOC 失效。

---

## 验收清单

- [ ] dev_flash.h 守卫改为 `__DEV_FLASH_H__`，init 接口为 `(pobj, start_addr, total_size, page_size)`
- [ ] dev_flash.c 修复 10 处 bug（地址/u64 计数/类型/磨损均衡）
- [ ] `USE_DEV_FLASH` + `USE_FLASH_G4_DRIVER` 已启用
- [ ] `motor_info_storage.h` 定义存储地址宏（0x0807F000, 4KB, 2KB 页）
- [ ] `motor_info_storage.c` 实现 init/load/save + 强符号覆盖
- [ ] `motor_info_storage_init()` 在 `hardware_init()` 中、`motor_loop_init()` 之前调用
- [ ] `app_motor_info()` 懒加载调用 `motor_info_storage_load()`
- [ ] `jm_app_motor_info_storage_save` 强符号在 map 中指向 `motor_info_storage.o`
- [ ] Keil 链接脚本已预留 4KB 数据区（用户自行配置）
- [ ] 上位机 0xEA 可保存，断电重启后 0xE6 读到保存值
- [ ] 多次保存验证磨损均衡轮转正确
- [ ] 破坏 Flash 数据后上电，系统 fallback 默认值不崩溃
