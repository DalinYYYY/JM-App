/**
 * @file        drv_flash_f4.c
 * @brief       STM32F4 内部 FLASH 驱动实现（单 Bank 扇区擦除 + 32-bit 字编程）
 *
 * @note        F405RG(1MB) 共 12 个扇区：
 *                Sector 0~3: 16KB, Sector 4: 64KB, Sector 5~11: 128KB
 *              write 假设上层传入整段数据(从扇区首地址开始),
 *              内部擦除该扇区 + 按 32-bit 字编程, 无需页缓冲。
 */
#include "drv_flash_f4.h"

#ifdef USE_FLASH_F4_DRIVER
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_flash_ex.h"

/* F405RG(1MB) 扇区表：起始地址 + 大小 */
typedef struct {
	u32 base;
	u32 size;
} f4_sector_t;

static const f4_sector_t s_sectors[12] = {
	{0x08000000U, 0x4000U},   /* Sector 0:  16KB */
	{0x08004000U, 0x4000U},   /* Sector 1:  16KB */
	{0x08008000U, 0x4000U},   /* Sector 2:  16KB */
	{0x0800C000U, 0x4000U},   /* Sector 3:  16KB */
	{0x08010000U, 0x10000U},  /* Sector 4:  64KB */
	{0x08020000U, 0x20000U},  /* Sector 5:  128KB */
	{0x08040000U, 0x20000U},  /* Sector 6:  128KB */
	{0x08060000U, 0x20000U},  /* Sector 7:  128KB */
	{0x08080000U, 0x20000U},  /* Sector 8:  128KB */
	{0x080A0000U, 0x20000U},  /* Sector 9:  128KB */
	{0x080C0000U, 0x20000U},  /* Sector 10: 128KB */
	{0x080E0000U, 0x20000U},  /* Sector 11: 128KB (Flash 尾部, 存储区) */
};

/* 根据地址找扇区号 0~11, 越界返回 0xFF */
static u8 addr_to_sector(u32 addr)
{
	for (u8 i = 0; i < 12; i++)
	{
		if (addr >= s_sectors[i].base && addr < s_sectors[i].base + s_sectors[i].size)
			return i;
	}
	return 0xFFU;
}

u8 drv_f4_flash_is_dualbank(void) { return 0U; }
u32 drv_f4_flash_total_size(void) { return 0x100000U; } /* F405RG 1MB */
u32 drv_f4_flash_page_size(void)  { return 0x20000U; }  /* 末段扇区 128KB */
u8 drv_f4_flash_get_bank(u32 addr)
{
	if (addr < FLASH_BASE || addr >= FLASH_BASE + drv_f4_flash_total_size())
		return 0xFFU;
	return 1U; /* F4 单 Bank */
}

/**
 * @brief 擦除一个或多个扇区（addr 需扇区对齐）
 */
u8 drv_f4_flash_erase_sector(const u32 addr, u8 len, u8 bank)
{
	(void)bank; /* F4 单 Bank */
	if (len == 0U)
		return FLASH_ERR_PARAM;

	u8 start_sec = addr_to_sector(addr);
	if (start_sec == 0xFFU || (u16)start_sec + len > 12U)
		return FLASH_ERR_ADDR_OUT_RANGE;
	/* 要求 addr 扇区对齐 */
	if (addr != s_sectors[start_sec].base)
		return FLASH_ERR_ADDR_OUT_RANGE;

	FLASH_EraseInitTypeDef er;
	u32 err;
	er.TypeErase = FLASH_TYPEERASE_SECTORS;
	er.VoltageRange = FLASH_VOLTAGE_RANGE_3; /* 2.7~3.6V */
	er.Sector = start_sec;
	er.NbSectors = len;

	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
	HAL_StatusTypeDef hal_ret = HAL_FLASHEx_Erase(&er, &err);
	HAL_FLASH_Lock();
	return (hal_ret == HAL_OK) ? FLASH_ERR_OK : FLASH_ERR_ERASE_FAILED;
}

/**
 * @brief 从 FLASH 读取数据（addr 需 8 字节对齐）
 */
u8 drv_f4_flash_read(const u32 addr, u64 *pdata64, u32 len_64)
{
	if (pdata64 == NULL || len_64 == 0U || (addr % 8U) != 0U)
		return FLASH_ERR_PARAM;
	if (drv_f4_flash_get_bank(addr) == 0xFFU)
		return FLASH_ERR_ADDR_OUT_RANGE;
	u32 end_addr = addr + len_64 * 8U;
	if (drv_f4_flash_get_bank(end_addr - 1U) == 0xFFU)
		return FLASH_ERR_ADDR_OUT_RANGE;
	for (u32 i = 0; i < len_64; i++)
		pdata64[i] = *(__IO uint64_t *)(addr + i * 8U);
	return FLASH_ERR_OK;
}

/* write: 擦除包含 addr 的扇区(若 addr 为扇区首地址), 再按 32-bit 字编程。
 * 假设上层 dev_flash.c 在 F4 下传 flag+data(非整扇区), 直接擦除+编程。 */
u8 drv_f4_flash_write(const u32 addr, u64 *pdata64, u32 len_64, u8 bank)
{
	(void)bank;
	if (pdata64 == NULL || len_64 == 0U || (addr % 8U) != 0U)
		return FLASH_ERR_PARAM;
	if (drv_f4_flash_get_bank(addr) == 0xFFU)
		return FLASH_ERR_ADDR_OUT_RANGE;

	u8 sec = addr_to_sector(addr);
	if (sec == 0xFFU)
		return FLASH_ERR_ADDR_OUT_RANGE;
	u32 sec_end = s_sectors[sec].base + s_sectors[sec].size;
	u32 end_addr = addr + len_64 * 8U;
	if (end_addr > sec_end)
		return FLASH_ERR_ADDR_OUT_RANGE;

	/* 1. 若 addr 为扇区首地址, 先擦除该扇区 */
	if (addr == s_sectors[sec].base)
	{
		u8 r = drv_f4_flash_erase_sector(addr, 1, 1);
		if (r != FLASH_ERR_OK)
			return r;
	}

	/* 2. 解锁 + 32-bit 字编程(u64 拆成 2 个 u32) */
	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
	if (FLASH_WaitForLastOperation(FLASH_WAITETIME) != HAL_OK)
	{
		__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
		HAL_FLASH_Lock();
		return FLASH_ERR_BUSY;
	}

	for (u32 i = 0; i < len_64; i++)
	{
		u64 v = pdata64[i];
		u32 w0 = (u32)(v & 0xFFFFFFFFU);
		u32 w1 = (u32)(v >> 32);
		u32 a0 = addr + i * 8U;
		u32 a1 = a0 + 4U;

		for (u8 try = 0; try < 10; try++)
		{
			if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, a0, w0) == HAL_OK)
				if (*(__IO uint32_t *)a0 == w0)
					break;
			__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
		}
		if (*(__IO uint32_t *)a0 != w0)
		{
			HAL_FLASH_Lock();
			return FLASH_ERR_WRITE_VERIFY;
		}
		for (u8 try = 0; try < 10; try++)
		{
			if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, a1, w1) == HAL_OK)
				if (*(__IO uint32_t *)a1 == w1)
					break;
			__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
		}
		if (*(__IO uint32_t *)a1 != w1)
		{
			HAL_FLASH_Lock();
			return FLASH_ERR_WRITE_VERIFY;
		}
	}
	HAL_FLASH_Lock();
	return FLASH_ERR_OK;
}

u8 drv_f4_flash_write_buffer(const u32 addr, u64 *pdata64, u32 len_64, u8 bank)
{
	return drv_f4_flash_write(addr, pdata64, len_64, bank);
}

u8 drv_flash_read(const u32 addr, u64 *pdata64, u32 len_64)
{
	/* 内存映射读Flash无需关全局中断临界区 */
	return drv_f4_flash_read(addr, pdata64, len_64);
}

u8 drv_flash_write(const u32 addr, u64 *pdata64, u32 len_64)
{
	if (drv_f4_flash_get_bank(addr) == 0xFFU)
		return FLASH_ERR_ADDR_OUT_RANGE;
	u8 ret;
	u32 primask = __get_PRIMASK();
	__disable_irq();
	ret = drv_f4_flash_write(addr, pdata64, len_64, 1U);
	__set_PRIMASK(primask);
	return ret;
}

/**
 * @brief 上层通用扇区擦除（关中断保护）
 */
u8 drv_flash_clear(const u32 addr, u8 len, u8 bank)
{
	u8 ret;
	u32 primask = __get_PRIMASK();
	__disable_irq();
	ret = drv_f4_flash_erase_sector(addr, len, bank);
	__set_PRIMASK(primask);
	return ret;
}

#endif /* USE_FLASH_F4_DRIVER */
