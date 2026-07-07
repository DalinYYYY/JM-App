/**
 * @file        drv_flash_g4.h
 * @brief       STM32G4内部FLASH驱动接口，双Bank页擦除/读/读改写
 *
 * @author      Dalin (dalinyy@163.com)
 * @version     1.0
 * @date        2026-06-16
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @par 修改日志:
 * | 日期       | 版本 | 作者   | 修改内容   |
 * |------------|------|--------|------------|
 * | 2026-06-16 | 1.0  | Dalin  | 初始创建   |
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 * @note        FLASH几何(容量/单双Bank/页大小)由HAL运行期值自适应，不按型号宏硬编码，
 *              自动兼容各容量后缀(CB/CC/CE)与单/双Bank配置
 */
#ifndef __FLASH_G4_H
#define __FLASH_G4_H

#include "drv_config.h"

#ifdef USE_FLASH_G4_DRIVER

#include "main.h"

/* 简写整型别名（本工程无独立 types.h，内联定义供 drv_flash_g4/dev_flash 共用） */
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

/*
 * FLASH几何参数全部来自HAL运行期值，不按型号宏硬编码：
 *   FLASH_SIZE       - 由芯片FLASHSIZE寄存器读出的实际容量(自动适配CB/CC/CE等容量后缀)
 *   FLASH_BANK_SIZE  - 单Bank=FLASH_SIZE，双Bank=FLASH_SIZE/2
 *   FLASH_PAGE_SIZE  - 双Bank 2KB/页，单Bank 4KB/页(由HAL按DBANK配置给出)
 *   FLASH_PAGE_NB    - 总页数
 * 单/双Bank由选项位FLASH_OPTR_DBANK运行期判断，详见drv_flash_g4.c
 */
#define STM32_FLASH_BASE FLASH_BASE /* STM32 FLASH 起始地址(0x08000000) */

#define FLASH_WAITETIME 50000U      /* FLASH操作等待超时(FLASH_WaitForLastOperation单位) */

/* FLASH空值定义 */
#define FLASH_NULL_U8  0xff
#define FLASH_NULL_U16 0xffff
#define FLASH_NULL_U32 0xffffffff
#define FLASH_NULL_U64 0xffffffffffffffff

/* 统一错误码定义 */
#define FLASH_ERR_OK             0U /* 成功 */
#define FLASH_ERR_ADDR_OUT_RANGE 1U /* 地址越界/无效地址/未对齐 */
#define FLASH_ERR_BUSY           2U /* FLASH忙 */
#define FLASH_ERR_ERASE_FAILED   3U /* 擦除失败 */
#define FLASH_ERR_BANK_INVALID   4U /* Bank编号非法（仅支持1/2） */
#define FLASH_ERR_WRITE_VERIFY   5U /* 写入校验失败 */
#define FLASH_ERR_PARAM          6U /* 参数非法 */

/**
 * @brief       判断当前FLASH是否为双Bank模式(运行期读DBANK选项位)
 * @return       : 1双Bank，0单Bank
 */
u8 drv_g4_flash_is_dualbank(void);

/**
 * @brief       获取FLASH总容量(字节)
 * @return       : 实际容量，来自芯片FLASHSIZE寄存器
 */
u32 drv_g4_flash_total_size(void);

/**
 * @brief       获取单页大小(字节)
 * @return       : 双Bank 2KB，单Bank 4KB
 */
u32 drv_g4_flash_page_size(void);

/**
 * @brief       根据地址获取Bank编号
 * @param        addr              : FLASH地址
 * @return       : 1=Bank1，2=Bank2，0xFF=越界。单Bank时有效地址恒返回1
 */
u8 drv_g4_flash_get_bank(u32 addr);

/* 底层FLASH操作函数 */
u8 drv_g4_flash_erase_page(const u32 addr, u8 len, u8 bank);                     // 按页擦除FLASH
u8 drv_g4_flash_read(const u32 addr, u64 *pdata64, u32 len_64);                  // 读取FLASH
u8 drv_g4_flash_write(const u32 addr, u64 *pdata64, u32 len_64, u8 bank);        // 写入FLASH
u8 drv_g4_flash_write_buffer(const u32 addr, u64 *pdata64, u32 len_64, u8 bank); // 批量写入FLASH

/* 上层通用FLASH操作函数 */
u8 drv_flash_read(const u32 addr, u64 *pdata64, u32 len_64);
u8 drv_flash_write(const u32 addr, u64 *pdata64, u32 len_64);
u8 drv_flash_clear(const u32 addr, u8 len, u8 bank);

#endif // USE_FLASH_G4_DRIVER
#endif // __FLASH_G4_H