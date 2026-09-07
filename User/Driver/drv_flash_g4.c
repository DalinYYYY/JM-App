/**
 * @file        drv_flash_g4.c
 * @brief       STM32G4内部FLASH驱动实现，双Bank页擦除/读/读改写
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
 */
#include "drv_flash_g4.h"

#ifdef USE_FLASH_G4_DRIVER
#include "stm32g4xx_hal_flash.h"
#include "stm32g4xx_hal_flash_ex.h"
#include "stm32g4xx_hal_flash_ramfunc.h"

/* 页缓冲区：读-改-擦-写回时暂存整页数据。
 * 按G4最大页(单Bank模式4KB=512个u64)分配，双Bank模式(2KB)仅用前一半 */
static u64 page_buf[0x1000U / 8U];

/* G4(RM0440)要求: Flash擦写期间ICache/DCache必须禁用, 否则行为未定义
 * 擦写段前保存缓存状态并禁用, 段后复位缓存行并按原状态恢复
 * (未使能缓存的板子恢复后保持禁用, 不改变板级默认行为) */
static u32 flash_cache_disable(void)
{
	u32 cache = FLASH->ACR & (FLASH_ACR_ICEN | FLASH_ACR_DCEN);
	__HAL_FLASH_INSTRUCTION_CACHE_DISABLE();
	__HAL_FLASH_DATA_CACHE_DISABLE();
	return cache;
}

static void flash_cache_restore(u32 cache)
{
	/* 复位缓存行(清残留总线状态)后按原状态恢复 */
	__HAL_FLASH_INSTRUCTION_CACHE_RESET();
	__HAL_FLASH_DATA_CACHE_RESET();
	if (cache & FLASH_ACR_ICEN)
		__HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
	if (cache & FLASH_ACR_DCEN)
		__HAL_FLASH_DATA_CACHE_ENABLE();
}

/* STM32G4 双Bank模式下 Bank2 基地址固定为 0x08040000（硬件地址译码固定，
 * 与芯片容量无关）。Bank1 起始于 FLASH_BASE(0x08000000)，两Bank之间
 * 是地址空洞（小容量芯片尤为明显，如128KB芯片: Bank1=64KB@0x08000000，
 * Bank2=64KB@0x08040000，中间0x08010000-0x0803FFFF无物理Flash）。 */
#define STM32G4_FLASH_BANK2_BASE 0x08040000U

/**
 * @brief 判断当前FLASH是否为双Bank模式(运行期读DBANK选项位)
 */
u8 drv_g4_flash_is_dualbank(void)
{
	return (FLASH->OPTR & FLASH_OPTR_DBANK) ? 1U : 0U;
}

/**
 * @brief 获取FLASH总容量(字节)
 */
u32 drv_g4_flash_total_size(void)
{
	return (u32)FLASH_SIZE;
}

/**
 * @brief 获取单页大小(字节)：双Bank 2KB，单Bank 4KB
 */
u32 drv_g4_flash_page_size(void)
{
	return drv_g4_flash_is_dualbank() ? 0x800U : 0x1000U;
}

/**
 * @brief 获取单Bank大小(字节)：双Bank为总容量一半，单Bank为全容量
 */
static u32 flash_bank_size(void)
{
	u32 total = (u32)FLASH_SIZE;
	return drv_g4_flash_is_dualbank() ? (total >> 1) : total;
}

/**
 * @brief 根据地址获取Bank编号(1/2，越界0xFF；单Bank有效地址恒为1)
 * @note  双Bank模式下两Bank地址不连续：Bank1=[FLASH_BASE, FLASH_BASE+bank_sz)，
 *        Bank2=[0x08040000, 0x08040000+bank_sz)，中间为地址空洞。
 */
u8 drv_g4_flash_get_bank(u32 addr)
{
	u32 total = (u32)FLASH_SIZE;

	if (!drv_g4_flash_is_dualbank())
	{
		/* 单Bank：连续地址 [FLASH_BASE, FLASH_BASE + total) */
		if (addr < FLASH_BASE || addr >= FLASH_BASE + total)
			return 0xFFU;
		return 1U;
	}

	/* 双Bank：Bank1、Bank2 地址不连续，分别校验 */
	u32 bank_sz = total >> 1; /* 每 Bank 大小 */
	if (addr >= FLASH_BASE && addr < FLASH_BASE + bank_sz)
		return 1U;
	if (addr >= STM32G4_FLASH_BANK2_BASE && addr < STM32G4_FLASH_BANK2_BASE + bank_sz)
		return 2U;
	return 0xFFU;
}

/**
 * @brief 获取指定Bank的起始地址
 */
static u32 flash_bank_start(u8 bank)
{
	return (bank == 2U) ? STM32G4_FLASH_BANK2_BASE : FLASH_BASE;
}

/**
 * @brief 按页擦除FLASH（运行期自适应单/双Bank与页大小）
 * @param addr 擦除起始地址（必须页对齐）
 * @param len  擦除页数
 * @param bank Bank编号（1=Bank1，2=Bank2；单Bank模式仅支持1）
 * @return 错误码（FLASH_ERR_OK=成功）
 */
u8 drv_g4_flash_erase_page(const u32 addr, u8 len, u8 bank)
{
	u32 err;
	u8 ret = HAL_ERROR;
	FLASH_EraseInitTypeDef EraseInitStruct;
	u32 page_size = drv_g4_flash_page_size();
	u32 pages_per_bank = flash_bank_size() / page_size;
	u8 dual = drv_g4_flash_is_dualbank();

	/* 1. Bank合法性校验 */
	if (dual)
	{
		if (bank != 1U && bank != 2U)
			return FLASH_ERR_BANK_INVALID;
	}
	else
	{
		if (bank != 1U)
			return FLASH_ERR_BANK_INVALID;
	}

	/* 2. 地址校验：有效范围+页对齐+Bank匹配 */
	u8 addr_bank = drv_g4_flash_get_bank(addr);
	if (addr_bank == 0xFFU || addr_bank != bank)
	{
		return FLASH_ERR_ADDR_OUT_RANGE;
	}
	if ((addr % page_size) != 0U)
	{
		return FLASH_ERR_ADDR_OUT_RANGE;
	}

	/* 3. Bank内页号计算 */
	u32 start_page_sn = (addr - flash_bank_start(bank)) / page_size;
	if ((start_page_sn + len) > pages_per_bank)
	{
		return FLASH_ERR_ADDR_OUT_RANGE;
	}

	/* 4. 解锁+清错误标志 */
	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
	u32 cache = flash_cache_disable(); /* 擦除期间禁用缓存(RM0440硬性要求) */
	if (FLASH_WaitForLastOperation(FLASH_WAITETIME) != HAL_OK)
	{
		__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
		flash_cache_restore(cache);
		HAL_FLASH_Lock();
		return FLASH_ERR_BUSY;
	}

	/* 5. 配置擦除参数。HAL的Banks字段：单Bank或Bank1用FLASH_BANK_1，Bank2用FLASH_BANK_2 */
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Page = start_page_sn;
	EraseInitStruct.NbPages = len;
	EraseInitStruct.Banks = (bank == 2U) ? FLASH_BANK_2 : FLASH_BANK_1;

	/* 6. 擦除+重试 */
	for (u8 i = 0; i < 10; i++)
	{
		ret = HAL_FLASHEx_Erase(&EraseInitStruct, &err);
		if (ret == HAL_OK)
			break;
		__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
		FLASH_WaitForLastOperation(FLASH_WAITETIME);
	}

	flash_cache_restore(cache); /* 恢复原缓存状态 */
	HAL_FLASH_Lock();
	return (ret == HAL_OK) ? FLASH_ERR_OK : FLASH_ERR_ERASE_FAILED;
}

/**
 * @brief 读取FLASH数据（仅支持Bank1、Bank2有效地址）
 * @param addr     读取起始地址（8字节对齐）
 * @param pdata64  数据缓冲区
 * @param len_64   读取长度（u64数量）
 * @return 错误码
 */
u8 drv_g4_flash_read(const u32 addr, u64 *pdata64, u32 len_64)
{
	// 校验参数+地址有效性
	if (pdata64 == NULL || len_64 == 0 || (addr % 8) != 0)
	{
		return FLASH_ERR_PARAM;
	}
	u8 addr_bank = drv_g4_flash_get_bank(addr);
	if (addr_bank == 0xFFU)
	{
		return FLASH_ERR_ADDR_OUT_RANGE;
	}
	// 校验读取范围不越界(末字节须仍属同一Bank，双Bank下地址不连续)
	u32 end_addr = addr + len_64 * 8U;
	if (drv_g4_flash_get_bank(end_addr - 1U) != addr_bank)
	{
		return FLASH_ERR_ADDR_OUT_RANGE;
	}

	for (u32 i = 0; i < len_64; i++)
	{
		pdata64[i] = *(__IO uint64_t *)(addr + i * 8);
	}
	return FLASH_ERR_OK;
}

/**
 * @brief 写入FLASH数据（仅支持Bank1、Bank2，按页读-改-擦-写回，不丢页内其余数据）
 * @param addr     写入起始地址（8字节对齐）
 * @param pdata64  待写入数据
 * @param len_64   写入长度（u64数量）
 * @param bank     Bank编号（1=Bank1，2=Bank2）
 * @return 错误码
 */
u8 drv_g4_flash_write(const u32 addr, u64 *pdata64, u32 len_64, u8 bank)
{
	// 校验参数+地址有效性
	if (pdata64 == NULL || len_64 == 0 || (addr % 8) != 0)
	{
		return FLASH_ERR_PARAM;
	}
	u8 addr_bank = drv_g4_flash_get_bank(addr);
	if (addr_bank == 0xFFU || addr_bank != bank)
	{
		return FLASH_ERR_ADDR_OUT_RANGE;
	}
	u32 end_addr = addr + len_64 * 8U;
	/* 不允许跨Bank写入(末字节须仍属同一Bank)，双Bank下地址不连续 */
	if (drv_g4_flash_get_bank(end_addr - 1U) != bank)
	{
		return FLASH_ERR_ADDR_OUT_RANGE;
	}

	const u32 page_size = drv_g4_flash_page_size();
	u32 bank_start = flash_bank_start(bank);
	const u32 words_per_page = page_size / 8U;
	u8 ret;

	/* 按页处理：读整页->叠加用户数据->擦除->整页写回，避免丢失页内其余数据 */
	u32 curr_addr = addr;
	while (curr_addr < end_addr)
	{
		/* 当前地址所在页的页首地址 */
		u32 page_addr = curr_addr - ((curr_addr - bank_start) % page_size);

		/* 1. 读整页到缓冲 */
		ret = drv_g4_flash_read(page_addr, page_buf, words_per_page);
		if (ret != FLASH_ERR_OK)
			return ret;

		/* 2. 将本页范围内的用户数据叠加到页缓冲 */
		for (u32 w = 0; w < words_per_page; w++)
		{
			u32 word_addr = page_addr + w * 8U;
			if (word_addr >= addr && word_addr < end_addr)
			{
				page_buf[w] = pdata64[(word_addr - addr) / 8U];
			}
		}

		/* 3. 擦除该页 */
		ret = drv_g4_flash_erase_page(page_addr, 1, bank);
		if (ret != FLASH_ERR_OK)
			return ret;

		/* 4. 整页写回(解锁->逐双字编程校验->加锁) */
		HAL_FLASH_Unlock();
		__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
		u32 cache = flash_cache_disable(); /* 编程期间禁用缓存(RM0440硬性要求) */
		if (FLASH_WaitForLastOperation(FLASH_WAITETIME) != HAL_OK)
		{
			__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
			flash_cache_restore(cache);
			HAL_FLASH_Lock();
			return FLASH_ERR_BUSY;
		}

		for (u32 w = 0; w < words_per_page; w++)
		{
			u32 write_addr = page_addr + w * 8U;
			u8 ok = 0;
			for (u8 i = 0; i < 10; i++)
			{
				if (FLASH_WaitForLastOperation(FLASH_WAITETIME) != HAL_OK)
				{
					flash_cache_restore(cache);
					HAL_FLASH_Lock();
					return FLASH_ERR_BUSY;
				}
				if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, write_addr, page_buf[w]) != HAL_OK)
				{
					flash_cache_restore(cache);
					HAL_FLASH_Lock();
					return FLASH_ERR_WRITE_VERIFY;
				}
				/* 校验写入结果 */
				if (*(__IO uint64_t *)(write_addr) == page_buf[w])
				{
					ok = 1;
					break;
				}
			}
			if (!ok)
			{
				flash_cache_restore(cache);
				HAL_FLASH_Lock();
				return FLASH_ERR_WRITE_VERIFY;
			}
		}
		flash_cache_restore(cache);
		HAL_FLASH_Lock();

		curr_addr = page_addr + page_size;
	}

	return FLASH_ERR_OK;
}

/**
 * @brief 批量写入FLASH（仅支持Bank1、Bank2）
 * @param addr     起始地址
 * @param pdata64  数据缓冲区
 * @param len_64   长度（u64）
 * @param bank     Bank编号（1=Bank1，2=Bank2）
 * @return 错误码
 */
u8 drv_g4_flash_write_buffer(const u32 addr, u64 *pdata64, u32 len_64, u8 bank)
{
	if (pdata64 == NULL || len_64 == 0)
		return FLASH_ERR_PARAM;

	u32 write_addr = addr;
	for (u32 i = 0; i < len_64; i++)
	{
		u8 ret = drv_g4_flash_write(write_addr, &pdata64[i], 1, bank);
		if (ret != FLASH_ERR_OK)
			return ret;
		write_addr += 8;
	}
	return FLASH_ERR_OK;
}

/**
 * @brief 上层通用读取函数（带中断保护，仅支持有效Bank）
 */
u8 drv_flash_read(const u32 addr, u64 *pdata64, u32 len_64)
{
	u8 ret = FLASH_ERR_OK;
	__disable_irq();
	ret = drv_g4_flash_read(addr, pdata64, len_64);
	__enable_irq();
	return ret;
}

/**
 * @brief 上层通用写入函数（自动识别Bank1、Bank2）
 * @warning 全程关中断保证擦写原子性，单页擦写耗时可达数十ms，会阻塞所有中断
 *          (含电机控制等实时中断)，请在非实时阶段调用
 */
u8 drv_flash_write(const u32 addr, u64 *pdata64, u32 len_64)
{
	u8 ret = FLASH_ERR_OK;
	u8 bank = drv_g4_flash_get_bank(addr);
	if (bank == 0xFF)
		return FLASH_ERR_ADDR_OUT_RANGE;

	__disable_irq();
	ret = drv_g4_flash_write(addr, pdata64, len_64, bank);
	__enable_irq();
	return ret;
}

/**
 * @brief 上层通用擦除函数（带中断保护，仅支持Bank1、Bank2）
 * @warning 全程关中断，擦除耗时可达数十ms，会阻塞所有中断，请在非实时阶段调用
 */
u8 drv_flash_clear(const u32 addr, u8 len, u8 bank)
{
	u8 ret = FLASH_ERR_OK;
	__disable_irq();
	ret = drv_g4_flash_erase_page(addr, len, bank);
	__enable_irq();
	return ret;
}

#endif /* USE_FLASH_G4_DRIVER */