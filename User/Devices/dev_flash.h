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
extern "C"
{
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
#define FLASH_FLAG_MAGIC       0x4D4F544Fu /* "MOTO" */
#define FLASH_FLAG_MAKE(seq)   (((uint64_t)FLASH_FLAG_MAGIC << 32) | ((seq) & 0xFFFFFFFFu))
#define FLASH_FLAG_IS_VALID(f) (((uint32_t)((f) >> 32)) == FLASH_FLAG_MAGIC)
#define FLASH_FLAG_SEQ(f)      ((uint32_t)((f) & 0xFFFFFFFFu))

	/**
 * @brief  flash 设备句柄结构体
 */
	typedef struct dev_flash
	{
		uint32_t start_addr;    /* 数据区起始绝对地址(init 时传入) */
		uint32_t total_size;    /* 数据区总大小(字节) */
		uint32_t page_size;     /* 单页大小(字节) */
		uint32_t sector_count;  /* 扇区数 = total_size / page_size */
		uint32_t last_sector;   /* 当前有效扇区索引 */
		uint32_t last_sequence; /* 当前有效扇区的 sequence 号 */
		bool inited;            /* 初始化完成标志 */

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
