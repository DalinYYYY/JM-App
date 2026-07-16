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
	                + WORD_SIZE /* 跳过 flag u64 */
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

	/* offset 必须 8 字节(u64)对齐，否则写入位置错位 */
	if ((offset % WORD_SIZE) != 0U)
		return DEV_ERROR;

	uint32_t usable = pobj->page_size - WORD_SIZE; /* 扣除 flag 后可用字节 */
	if ((uint64_t)offset + (uint64_t)size * WORD_SIZE > usable)
		return DEV_ERROR;

#if defined(USE_FLASH_F4_DRIVER)
	/* F4: 单扇区无磨损均衡。直接擦除扇区 + 写 flag+data, 无需整页读回。
	 * motor_info 每次写全量(~1KB=128 u64), page_buf 只需容纳 flag+data。
	 * F4 末段扇区 128KB, G4 路径的整页 read-modify-write 需 128KB 缓冲 → RAM 溢出,
	 * 故 F4 下跳过整页读回, 每次擦扇区 + 全量写。 */
	static u64 f4_buf[257U]; /* flag(1) + 最多 256 个 u64 数据 = 2056 字节 */
	if (size > 256U)
		return DEV_ERROR;

	uint32_t new_seq = pobj->last_sequence + 1U;
	f4_buf[0] = FLASH_FLAG_MAKE(new_seq);
	for (uint16_t i = 0; i < size; i++)
		f4_buf[1U + i] = data[i];

	/* 写入起始地址 = start_addr(扇区首), drv_flash_f4_write 会先擦除该扇区 */
	u8 ret = drv_flash_write(pobj->start_addr, f4_buf, (u16)(1U + size));
	if (ret != FLASH_ERR_OK)
		return DEV_ERROR;

	pobj->last_sector = 0U;
	pobj->last_sequence = new_seq;
	return DEV_EOK;
#else
	/* G4: 读改写 + 磨损均衡 */
	uint32_t page_u64 = pobj->page_size / WORD_SIZE; /* 页内 u64 个数 */

	/* 1. 静态页缓冲（按最大单 Bank 页 4KB 预分配，避免堆不足导致 save 静默失败） */
	static u64 page_buf[0x1000U / sizeof(u64)]; /* 4KB / 8 = 512 个 u64，兼容单 Bank 4KB 页 */
	if (pobj->page_size > sizeof(page_buf))
		return DEV_ERROR;

	/* 2. 读当前扇区整页(首次上电 last_sector 无效时读到 0xFF) */
	uint32_t cur_addr = pobj->start_addr + pobj->last_sector * pobj->page_size;
	if (drv_flash_read(cur_addr, page_buf, page_u64) != FLASH_ERR_OK)
	{
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

	if (ret != FLASH_ERR_OK)
		return DEV_ERROR;

	/* 7. 更新状态 */
	pobj->last_sector = next;
	pobj->last_sequence = new_seq;

	return DEV_EOK;
#endif
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

	pobj->start_addr = start_addr;
	pobj->total_size = total_size;
	pobj->page_size = page_size;
	pobj->sector_count = total_size / page_size;

	/* 2. 扫描所有扇区，找 sequence 最大的有效扇区 */
	uint32_t best_seq = 0;
	int best_sector = -1;

	for (uint32_t i = 0; i < pobj->sector_count; i++)
	{
		uint32_t addr = pobj->start_addr + i * pobj->page_size;
		u64 flag;
		if (drv_flash_read(addr, &flag, 1) != FLASH_ERR_OK)
			return; /* Flash 读取失败，保持 inited=false */

		if (FLASH_FLAG_IS_VALID(flag))
		{
			uint32_t seq = FLASH_FLAG_SEQ(flag);
			if (seq >= best_seq) /* >= 处理回绕：后者覆盖前者 */
			{
				best_seq = seq;
				best_sector = (int)i;
			}
		}
	}

	if (best_sector >= 0)
	{
		pobj->last_sector = (uint32_t)best_sector;
		pobj->last_sequence = best_seq;
	}
	else
	{
		/* 首次上电：无有效数据，下次写入从 sector 0 开始 */
		pobj->last_sector = 0U;
		pobj->last_sequence = 0U;
	}

	pobj->inited = true;
	pobj->flash_read = dev_flash_read;
	pobj->flash_write = dev_flash_write;
}

#endif /* USE_DEV_FLASH */
