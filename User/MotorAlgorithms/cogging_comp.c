/**
 * @file    cogging_comp.c
 * @brief   齿槽转矩补偿实现：查表插值 + dev_flash 表存储
 * @note    表 RAM 副本 2016B 静态分配；存储复用通用 dev_flash 设备
 *          （页擦写+扇区轮转磨损均衡），页参数来自板级宏（见 cogging_comp.h）。
 *          本模块不依赖 motor_param/DataHub，使能位与增益由调用方（级联控制层）
 *          每拍直传，运行期参数热更新即时生效。
 */

#include "cogging_comp.h"

#if defined(USE_DEV_FLASH) && COGGING_COMP_EN

#include "dev_flash.h"
#include "utils.h" /* utils_crc32c */
#include <stddef.h>

/* ===== RAM 表副本与有效标志（ZI 静态分配 ~2KB）===== */
static int16_t s_table[COGGING_TABLE_N];
static uint8_t s_valid;

/* ===== 待写表（ISR 登记 → 线程落盘）===== */
static int16_t s_pending[COGGING_TABLE_N];
static volatile uint8_t s_write_state; /* 0=idle 1=pending 2=写成功 3=写失败 */

/* ===== 独立 dev_flash 实例（页参数板级宏注入）===== */
static dev_flash_t s_flash;

/* 存储记录布局（254 u64 = 2032B，页数据区上限 2040B 内）*/
#define COGGING_REC_U64 254u
#define COGGING_HDR_U64 2u /* [0]=magic/n/ver  [1]=crc32 */

/* 拼装 u64 头: 低32=magic, [47:32]=table_n(16bit), [63:48]=ver(16bit) */
#define COGGING_HDR_U64_VAL \
	((uint64_t)COGGING_MAGIC | ((uint64_t)COGGING_TABLE_N << 32) | ((uint64_t)COGGING_VER << 48))

static u64 s_rec[COGGING_REC_U64]; /* 擦写静态缓冲(2KB): 避免栈上大块分配 */

uint8_t cogging_comp_is_valid(void)
{
	return s_valid;
}

uint8_t cogging_comp_reload(void)
{
	dev_flash_init(&s_flash, COGGING_FLASH_START_ADDR, COGGING_FLASH_TOTAL_SIZE,
				   COGGING_FLASH_PAGE_SIZE);

	s_valid = 0u;

	/* 读当前轮转扇区记录（空片/未写过时 magic 不符, init 定位失败读回全 FF）*/
	if (s_flash.flash_read(&s_flash, 0, s_rec, COGGING_REC_U64) != DEV_EOK)
		return 0u;

	/* 头校验: magic + 点数 + 版本（同时排除擦除态全 FF）
	 * 点数取 [47:32] 并掩掉高 16 位版本号: 不掩时会得到 (ver<<16)|n,
	 * 恒 != COGGING_TABLE_N, 所有有效表都会被拒绝 */
	if ((uint32_t)s_rec[0] != COGGING_MAGIC ||
		(uint32_t)((s_rec[0] >> 32) & 0xFFFFu) != COGGING_TABLE_N ||
		(uint32_t)((s_rec[0] >> 48) & 0xFFFFu) != COGGING_VER)
		return 0u;

	/* CRC 校验: crc 字段置零后对全记录求值 */
	{
		uint64_t hdr_crc = s_rec[1];
		uint32_t crc;
		s_rec[1] = 0u;
		crc = utils_crc32c((uint8_t *)s_rec, COGGING_REC_U64 * 8u);
		s_rec[1] = hdr_crc;
		if (crc != (uint32_t)hdr_crc)
			return 0u;
	}

	/* 拷贝表数据（记录 u64 区起第 16B）
	 * 关中断原子换表: flush 落盘后 drv_flash_write 已重新开中断, reload 期间
	 * 电机可能重新使能, 防止控制 ISR 在拷贝中途读到半新半旧的表
	 * (s_valid 与表内容同临界区更新, ISR 要么全旧、要么全新) */
	{
		const uint8_t *src = (const uint8_t *)&s_rec[COGGING_HDR_U64];
		uint32_t primask = __get_PRIMASK();
		__disable_irq();
		for (uint16_t i = 0u; i < COGGING_TABLE_N; i++)
		{
			s_table[i] = (int16_t)(src[2u * i] | ((uint16_t)src[2u * i + 1u] << 8));
		}
		s_valid = 1u;
		__set_PRIMASK(primask);
	}
	return 1u;
}

uint8_t cogging_comp_flash_write(const int16_t *table_ma)
{
	if (table_ma == NULL)
		return 0u;

	/* 确保 dev_flash 已初始化（幂等, 扫描定位轮转扇区）*/
	dev_flash_init(&s_flash, COGGING_FLASH_START_ADDR, COGGING_FLASH_TOTAL_SIZE,
				   COGGING_FLASH_PAGE_SIZE);

	/* 构造记录: 头 + crc 置零 + 表数据（int16 小端逐字节）*/
	s_rec[0] = COGGING_HDR_U64_VAL;
	s_rec[1] = 0u;
	{
		uint8_t *dst = (uint8_t *)&s_rec[COGGING_HDR_U64];
		for (uint16_t i = 0u; i < COGGING_TABLE_N; i++)
		{
			uint16_t v = (uint16_t)table_ma[i];
			dst[2u * i] = (uint8_t)(v & 0xFFu);
			dst[2u * i + 1u] = (uint8_t)(v >> 8);
		}
	}
	s_rec[1] = (uint64_t)utils_crc32c((uint8_t *)s_rec, COGGING_REC_U64 * 8u);

	/* dev_flash 写入: 自动轮转扇区+先擦后写(内部关中断约 10~40ms) */
	if (s_flash.flash_write(&s_flash, 0, s_rec, COGGING_REC_U64) != DEV_EOK)
		return 0u;

	/* 回读走加载校验链, 失败保持 RAM 旧状态 */
	return cogging_comp_reload();
}

void cogging_comp_request_write(const int16_t *table_ma)
{
	if (table_ma == NULL)
		return;
	/* 拷贝完成后置标志(volatile): 线程读到 pending 时缓冲必然完整 */
	for (uint16_t i = 0u; i < COGGING_TABLE_N; i++)
		s_pending[i] = table_ma[i];
	s_write_state = 1u;
}

uint8_t cogging_comp_flush(void)
{
	uint8_t ok;
	if (s_write_state != 1u)
		return 0u;
	ok = cogging_comp_flash_write(s_pending);
	s_write_state = ok ? 2u : 3u; /* 诊断状态: 线程读后回写使能参数 */
	return ok;
}

uint8_t cogging_comp_get_write_state(void)
{
	return s_write_state;
}

void cogging_comp_clear_write_state(void)
{
	s_write_state = 0u;
}

float cogging_comp_current(float theta_mech, uint8_t enable, float gain)
{
	float u;
	float frac;
	int i;
	int i2;

	/* 快速路径: 单一可预测分支(表无效/未使能/零增益已在参数热应用时收敛至此) */
	if (s_valid == 0u || enable == 0u || gain == 0.0f)
		return 0.0f;

	/* 机械角 → 表索引(浮点), floor 分解出 [0,1) 插值系数(负角修正保证环形插值正确) */
	u = theta_mech * ((float)COGGING_TABLE_N * 0.15915494309189535f); /* /(2π) */
	i = (int)u;
	if (u < 0.0f)
		i -= 1; /* floor: 负角向 -∞ 取整 */
	frac = u - (float)i;

	i %= (int)COGGING_TABLE_N;
	if (i < 0)
		i += (int)COGGING_TABLE_N;
	i2 = i + 1;
	if (i2 >= (int)COGGING_TABLE_N)
		i2 = 0;

	return (s_table[i] + (s_table[i2] - s_table[i]) * frac) * (gain * 0.001f);
}

#endif /* USE_DEV_FLASH && COGGING_COMP_EN */
