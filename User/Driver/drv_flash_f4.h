/**
 * @file        drv_flash_f4.h
 * @brief       STM32F4内部FLASH驱动接口（单Bank扇区擦除/读/写）
 *
 * @note        接口签名与 drv_flash_g4.h 完全对齐，dev_flash.c / motor_info_storage.c
 *              业务层无需改动。F405RG(1MB) 扇区划分：
 *                Sector 0~3: 16KB  (0x08000000~0x0800FFFF)
 *                Sector 4:   64KB  (0x08010000~0x0801FFFF)
 *                Sector 5~11: 128KB (0x08020000~0x080FFFFF)
 *              编程粒度 32-bit word (FLASH_TYPEPROGRAM_WORD)。
 *              本驱动无内部页缓冲：write 直接擦除目标扇区 + 字编程，
 *              上层 dev_flash.c 在 F4 下走单扇区无磨损均衡路径(跳过整页读回)。
 */
#ifndef __FLASH_F4_H
#define __FLASH_F4_H

#include "drv_config.h"

#ifdef USE_FLASH_F4_DRIVER

#include "main.h"

/* 简写整型别名（与 drv_flash_g4.h 一致，供 drv_flash_f4/dev_flash 共用） */
typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

#define STM32_FLASH_BASE FLASH_BASE /* 0x08000000 */
#define FLASH_WAITETIME 50000U

#define FLASH_NULL_U8  0xff
#define FLASH_NULL_U16 0xffff
#define FLASH_NULL_U32 0xffffffff
#define FLASH_NULL_U64 0xffffffffffffffff

/* F4 HAL 没有 FLASH_FLAG_ALL_ERRORS 宏（G4 专有），这里用 F4 的错误标志组合补全 */
#ifndef FLASH_FLAG_ALL_ERRORS
#define FLASH_FLAG_ALL_ERRORS (FLASH_FLAG_EOP    | FLASH_FLAG_OPERR  | \
                               FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | \
                               FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR)
#endif

#define FLASH_ERR_OK             0U
#define FLASH_ERR_ADDR_OUT_RANGE 1U
#define FLASH_ERR_BUSY           2U
#define FLASH_ERR_ERASE_FAILED   3U
#define FLASH_ERR_BANK_INVALID   4U
#define FLASH_ERR_WRITE_VERIFY   5U
#define FLASH_ERR_PARAM          6U

u8  drv_f4_flash_is_dualbank(void);
u32 drv_f4_flash_total_size(void);
u32 drv_f4_flash_page_size(void);
u8  drv_f4_flash_get_bank(u32 addr);

u8 drv_f4_flash_erase_sector(const u32 addr, u8 len, u8 bank);
u8 drv_f4_flash_read(const u32 addr, u64 *pdata64, u32 len_64);
u8 drv_f4_flash_write(const u32 addr, u64 *pdata64, u32 len_64, u8 bank);
u8 drv_f4_flash_write_buffer(const u32 addr, u64 *pdata64, u32 len_64, u8 bank);

u8 drv_flash_read(const u32 addr, u64 *pdata64, u32 len_64);
u8 drv_flash_write(const u32 addr, u64 *pdata64, u32 len_64);
u8 drv_flash_clear(const u32 addr, u8 len, u8 bank);

#endif /* USE_FLASH_F4_DRIVER */
#endif /* __FLASH_F4_H */
