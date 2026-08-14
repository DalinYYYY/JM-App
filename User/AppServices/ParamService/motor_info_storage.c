/**
 * @file        motor_info_storage.c
 * @brief       motor_info Flash 持久化设备对象实现
 *
 * @author      Dalin (dalinyy@163.com)
 * @version     3.0
 * @date        2026-07-02
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @par 修改日志:
 * | 日期       | 版本 | 作者  | 修改内容                                                      |
 * |------------|------|-------|-------------------------------------------------------------|
 * | 2026-07-02 | 1.0  | Dalin | 初始创建                                                      |
 * | 2026-07-02 | 2.0  | Dalin | 按 C-OOP 规范重构:加 status_t 枚举/deinit/契约文档         |
 * | 2026-07-02 | 3.0  | Dalin | 改设备对象模式:ops 方法表+extern 单例+组合 dev_flash 子设备  |
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》及 C-OOP 设备对象开发规范
 */
#include "motor_info_storage.h"

#if defined(USE_DEV_FLASH)

#include "utils.h"
#include "motor_profile.h" /* motor_profile_apply_info: 编译期电机电气身份覆盖 */
#include "assert_report.h"
#include <string.h>
#if defined(USE_DEV_EEPROM)
#include "dev_eeprom.h" /* 可选 EEPROM 双备份 */
#endif

/* motor_info_t = 1024B = 128 个 u64 */
#define MOTORINFO_LEN_U64 (PARAM_AREA_SIZE / 8U)

/* ===== 全局单例定义（外部经 extern 引用） ===== */
motor_info_storage_t g_motor_info_storage;

/* ===== 内部 CRC32 工具（私有 static 函数，不通过 ops） ===== */

/* 静态 CRC 计算缓冲: 避免在 motorinfo_ops_save 嵌套调用时栈上分配 1024B
 * 导致主栈(2KB)溢出触发 MemManage_Handler。
 * 单线程使用(初始化阶段 + 命令处理), 无需重入保护。*/
static motor_info_t s_crc_tmp;

/**
 * @brief  计算 motor_info_t 的 CRC32（跳过 header.crc32 字段）
 * @details 拷贝到静态缓冲 → crc32 置零 → 对 1024B 用 utils_crc32c 求值
 * @note   使用静态缓冲替代栈分配, 避免 save 路径 1024B 嵌套栈占用。
 */
static uint32_t motorinfo_crc32_compute(const motor_info_t *cfg)
{
	memcpy(&s_crc_tmp, cfg, PARAM_AREA_SIZE);
	s_crc_tmp.blocks.header.crc32 = 0U;
	return utils_crc32c(s_crc_tmp.raw, PARAM_AREA_SIZE);
}

/**
 * @brief  三重校验：config_version → CRC32 → 字段范围
 * @return MOTOR_INFO_STORAGE_OK / NO_DATA / CRC_FAIL / RANGE_FAIL
 */
static motor_info_storage_status_t motorinfo_verify(const motor_info_t *cfg)
{
	/* config_version 校验：profile 版本不匹配视为无有效数据（触发重新初始化） */
	if (cfg->blocks.system.config_version != MOTOR_PROFILE_CONFIG_VERSION)
		return MOTOR_INFO_STORAGE_NO_DATA;

	/* 先保存存储的 crc32 再计算：motorinfo_crc32_compute 复用 s_crc_tmp 作为工作区，
	 * 当 cfg==&s_crc_tmp（save 回读校验路径）时会先清零 cfg->crc32，
	 * 若直接比较会因左侧被清零而恒失败。 */
	uint32_t stored_crc = cfg->blocks.header.crc32;
	if (stored_crc != motorinfo_crc32_compute(cfg))
		return MOTOR_INFO_STORAGE_CRC_FAIL;

	if (motor_info_validate(cfg) != 0)
		return MOTOR_INFO_STORAGE_RANGE_FAIL;

	return MOTOR_INFO_STORAGE_OK;
}

#if defined(USE_DEV_EEPROM)
/**
 * @brief  从 EEPROM 加载 motor_info 配置（含 magic + 三重校验）
 * @details 布局: [0:4)=MOTC magic, [4:1028)=motor_info 整块(含 CRC)。
 *          magic 单独存放，不污染 motor_info 的 CRC 计算。
 * @return OK(数据有效) / NO_DATA(magic 不符或读失败) / CRC_FAIL / RANGE_FAIL
 */
static motor_info_storage_status_t motorinfo_eeprom_load(dev_eeprom_t *dev, motor_info_t *cfg)
{
	if (dev == NULL || cfg == NULL)
		return MOTOR_INFO_STORAGE_ERR_ARG;

	uint32_t magic = 0U;
	if (dev->read(dev, MOTORINFO_EEPROM_START_ADDR, (uint8_t *)&magic, sizeof(magic)) != DEV_EOK)
		return MOTOR_INFO_STORAGE_NO_DATA; /* 读失败视为无有效数据 */

	if (magic != MOTORINFO_EEPROM_MAGIC)
		return MOTOR_INFO_STORAGE_NO_DATA; /* 未写入过 EEPROM */

	if (dev->read(dev, MOTORINFO_EEPROM_START_ADDR + MOTORINFO_EEPROM_CFG_OFFSET,
	              (uint8_t *)cfg, PARAM_AREA_SIZE) != DEV_EOK)
		return MOTOR_INFO_STORAGE_NO_DATA;

	return motorinfo_verify(cfg);
}

/**
 * @brief  将 motor_info 配置写入 EEPROM（magic + 数据 + 回读校验）
 * @return OK / ERR_EEPROM_WRITE / ERR_EEPROM_VERIFY
 */
static motor_info_storage_status_t motorinfo_eeprom_save(dev_eeprom_t *dev, const motor_info_t *cfg)
{
	if (dev == NULL || cfg == NULL)
		return MOTOR_INFO_STORAGE_ERR_ARG;

	uint32_t magic = MOTORINFO_EEPROM_MAGIC;
	if (dev->write(dev, MOTORINFO_EEPROM_START_ADDR, (uint8_t *)&magic, sizeof(magic)) != DEV_EOK)
		return MOTOR_INFO_STORAGE_ERR_EEPROM_WRITE;
	if (dev->write(dev, MOTORINFO_EEPROM_START_ADDR + MOTORINFO_EEPROM_CFG_OFFSET,
	               (uint8_t *)cfg, PARAM_AREA_SIZE) != DEV_EOK)
		return MOTOR_INFO_STORAGE_ERR_EEPROM_WRITE;

	/* 回读校验：复用静态缓冲避免 1024B 栈分配，校验链与 Flash 一致 */
	if (dev->read(dev, MOTORINFO_EEPROM_START_ADDR + MOTORINFO_EEPROM_CFG_OFFSET,
	              s_crc_tmp.raw, PARAM_AREA_SIZE) != DEV_EOK)
		return MOTOR_INFO_STORAGE_ERR_EEPROM_VERIFY;
	if (motorinfo_verify(&s_crc_tmp) != MOTOR_INFO_STORAGE_OK)
		return MOTOR_INFO_STORAGE_ERR_EEPROM_VERIFY;

	return MOTOR_INFO_STORAGE_OK;
}
#endif /* USE_DEV_EEPROM */

/* ===== ops 方法实现（static，只通过 ops 表对外暴露） ===== */

/**
 * @brief  取全局 motor_info 句柄
 * @note   包装函数已做 pobj/ops 检查，此处直接访问字段
 */
static motor_info_t *motorinfo_ops_get(struct motor_info_storage *pobj)
{
	return &pobj->motor_info;
}

/**
 * @brief  从 Flash 加载 motor_info 配置（覆盖传入 cfg）
 * @details 调用 dev_flash->flash_read 读 1024B → 三重校验(config_version+CRC+范围)
 */
static motor_info_storage_status_t motorinfo_ops_load(struct motor_info_storage *pobj, motor_info_t *cfg)
{
	if (cfg == NULL)
		return MOTOR_INFO_STORAGE_ERR_ARG;
	if (!pobj->inited)
		return MOTOR_INFO_STORAGE_ERR_INIT;

#if defined(USE_DEV_EEPROM)
	/* 优先从 EEPROM 加载（上电默认使用 EEPROM 数据）：
	 * 设备就绪且数据校验通过则直接采用；否则回退 Flash。 */
	if (pobj->eeprom_dev != NULL && pobj->eeprom_dev->is_ready(pobj->eeprom_dev))
	{
		motor_info_storage_status_t er = motorinfo_eeprom_load(pobj->eeprom_dev, cfg);
		if (er == MOTOR_INFO_STORAGE_OK)
			return MOTOR_INFO_STORAGE_OK;
		/* EEPROM 无有效数据/损坏 → 回退 Flash */
	}
#endif

	/* 从 dev_flash 读 1024B = 128 u64，offset=0 */
	int rc = pobj->flash_dev.flash_read(&pobj->flash_dev, 0, (u64 *)cfg, MOTORINFO_LEN_U64);
	if (rc != DEV_EOK)
		return MOTOR_INFO_STORAGE_ERR_FLASH;

	/* 三重校验：config_version → CRC32 → 字段范围 */
	return motorinfo_verify(cfg);
}

/**
 * @brief  将 motor_info 配置保存到 EEPROM(默认) + 可选 Flash 备份
 *
 * @param  flags 存储目标标志(MOTORINFO_SAVE_FLAG_* 位或):
 *               0 = 仅写 EEPROM(上电优先加载, 默认);
 *               MOTORINFO_SAVE_FLAG_FLASH = 追加写 Flash 备份。
 *               EEPROM 设备不可用时自动回退写 Flash, 保证无 EEPROM 的板子仍能持久化。
 *
 * @details 固化次数 save_count 每次固化(0xEA 保存)都递增, 记录固化总次数;
 *          无论是否写 Flash 备份都计数(默认 EEPROM 保存也算一次固化)。
 *          EEPROM 与 Flash 落盘数据保持一致(共用 s_crc_tmp 构造的递增计数+CRC)。
 *          EEPROM 成功即视为固化成功(Flash 失败不阻断); EEPROM 未启用时回退 Flash 结果。
 *          全程零栈上大块分配(避免 1024B motor_info_t 嵌套栈占用触发 MemManage)。
 *
 * @note   Flash 写入失败重试: 最多 3 次重试, 每次重试前重新计算 CRC32。
 *         重试全部失败后记 ERR_FLASH_WRITE; 回读校验失败记 ERR_FLASH_VERIFY。
 *         EEPROM 写/回读校验失败返回 ERR_EEPROM_WRITE / ERR_EEPROM_VERIFY。
 *         重试间隔约 1ms(给 Flash 控制器恢复时间), 总最坏耗时约 90ms(3*30ms)。
 */
static motor_info_storage_status_t motorinfo_ops_save(struct motor_info_storage *pobj,
                                                      const motor_info_t *cfg, uint32_t flags)
{
	/* 最大重试次数 */
	#define MOTOR_INFO_SAVE_MAX_RETRY  3u

	if (cfg == NULL)
		return MOTOR_INFO_STORAGE_ERR_ARG;
	if (!pobj->inited)
		return MOTOR_INFO_STORAGE_ERR_INIT;

	/* 1. 保存前必须通过范围校验。若允许无效数据落盘，本次 save
	 *    虽会成功，但下次上电必然在 motorinfo_verify() 中失败并重走默认路径。 */
	int vrc = motor_info_validate(cfg);
	if (vrc != 0)
		return (motor_info_storage_status_t)vrc;

	/* 1.5 确定 EEPROM 可用性, 并据此决定是否写 Flash:
	 *     - 显式带 FLASH 标志: 写 EEPROM + Flash 备份;
	 *     - EEPROM 设备不可用: 回退写 Flash(保证无 EEPROM 板子数据不丢);
	 *     - 默认(无标志且 EEPROM 可用): 仅写 EEPROM, 不碰 Flash。 */
#if defined(USE_DEV_EEPROM)
	bool eeprom_ready = (pobj->eeprom_dev != NULL) && pobj->eeprom_dev->is_ready(pobj->eeprom_dev);
#else
	bool eeprom_ready = false;
#endif
	bool write_flash = ((flags & MOTORINFO_SAVE_FLAG_FLASH) != 0u) || !eeprom_ready;

	/* 2. 固化次数自增并超限保护: 每次固化(0xEA 保存)都递增, 记录固化总次数。
	 *    (无论是否写 Flash 备份, 默认 EEPROM 保存也算一次固化) */
	memcpy(&s_crc_tmp, cfg, PARAM_AREA_SIZE);
	uint32_t new_save_count = s_crc_tmp.blocks.system.save_count;
	if (new_save_count >= MOTORINFO_SAVE_LIMIT)
		return MOTOR_INFO_STORAGE_ERR_SAVE_LIMIT;
	new_save_count += 1u;

	/* 3. 构造待落盘数据（单次构造，EEPROM/Flash 共用）：
	 *    固化次数自增 + 原地重算 CRC32
	 *    (复用 s_crc_tmp 避免栈上 1024B 分配) */
	memcpy(&s_crc_tmp, cfg, PARAM_AREA_SIZE);
	s_crc_tmp.blocks.system.save_count = new_save_count;
	s_crc_tmp.blocks.header.crc32 = 0U;
	s_crc_tmp.blocks.header.crc32 = utils_crc32c(s_crc_tmp.raw, PARAM_AREA_SIZE);

	/* 4. 写 EEPROM（主存储，独立于 Flash）：即使 Flash 失败也能固化到 EEPROM，
	 *    满足"上电默认加载 EEPROM"。源数据用 s_crc_tmp（含递增计数+CRC）。 */
	motor_info_storage_status_t eeprom_ret = MOTOR_INFO_STORAGE_OK;
#if defined(USE_DEV_EEPROM)
	if (eeprom_ready)
	{
		eeprom_ret = motorinfo_eeprom_save(pobj->eeprom_dev, &s_crc_tmp);
	}
#endif

	/* 5. 写 Flash（备份，3 次重试；结果记录到 flash_ret，不阻断 EEPROM 结果） */
	motor_info_storage_status_t flash_ret = MOTOR_INFO_STORAGE_OK;
	if (write_flash)
	{
		for (uint8_t attempt = 0u; attempt < MOTOR_INFO_SAVE_MAX_RETRY; attempt++)
		{
			/* 重新构造 s_crc_tmp（固化计数 + 重算 CRC），EEPROM 已完成的写入不受影响 */
			memcpy(&s_crc_tmp, cfg, PARAM_AREA_SIZE);
			s_crc_tmp.blocks.system.save_count = new_save_count;
			s_crc_tmp.blocks.header.crc32 = 0U;
			s_crc_tmp.blocks.header.crc32 = utils_crc32c(s_crc_tmp.raw, PARAM_AREA_SIZE);

			/* 通过 dev_flash 写入(内部轮转扇区+磨损均衡，关中断约 10-30ms) */
			int rc = pobj->flash_dev.flash_write(&pobj->flash_dev, 0, (u64 *)&s_crc_tmp, MOTORINFO_LEN_U64);
			if (rc != DEV_EOK)
			{
				/* 擦写失败: 重试(最后一次失败则记 ERR_FLASH_WRITE) */
				if (attempt + 1u < MOTOR_INFO_SAVE_MAX_RETRY)
				{
					/* 重试间隔: 给 Flash 控制器恢复时间, 避免连续失败 */
					for (volatile uint32_t d = 0u; d < 1000u * SystemCoreClock / 1000000u; d++) { }
					continue;
				}
				flash_ret = MOTOR_INFO_STORAGE_ERR_FLASH_WRITE;
				break;
			}

			/* 回读并走与上电相同的校验链（复用 s_crc_tmp, 内部拷贝隔离原始 cfg） */
			rc = pobj->flash_dev.flash_read(&pobj->flash_dev, 0, (u64 *)&s_crc_tmp, MOTORINFO_LEN_U64);
			if (rc != DEV_EOK)
			{
				flash_ret = MOTOR_INFO_STORAGE_ERR_FLASH_VERIFY;
				break;
			}
			flash_ret = (motorinfo_verify(&s_crc_tmp) == MOTOR_INFO_STORAGE_OK)
				? MOTOR_INFO_STORAGE_OK : MOTOR_INFO_STORAGE_ERR_FLASH_VERIFY;
			break;
		}
	}

	/* 6. 把本次固化次数回写 RAM，保证上位机 0xE6 读到的计数与存储一致。 */
	pobj->motor_info.blocks.system.save_count = new_save_count;

	/* 7. 返回：EEPROM(主)已启用则以 EEPROM 结果为准（成功即固化成功）；
	 *    未启用时回退 Flash 结果。 */
	if (eeprom_ready)
		return eeprom_ret;
	return flash_ret;
}

/**
 * @brief  反初始化：清零状态标志，下次 init 可重新加载
 * @note   不释放资源（无动态内存），不擦除 Flash。
 *        dev_flash 子设备也置 inited=false。
 */
static void motorinfo_ops_deinit(struct motor_info_storage *pobj)
{
	pobj->inited = false;
	pobj->motor_info_loaded = false;
	pobj->flash_dev.inited = false;
}

/* ===== 共享 static const ops 方法表（进 Flash 只读，所有实例共享） ===== */
static const motor_info_storage_ops_t s_motorinfo_ops = {
	.get = motorinfo_ops_get,
	.load = motorinfo_ops_load,
	.save = motorinfo_ops_save,
	.deinit = motorinfo_ops_deinit,
};

/* ===== 构造函数 + 生命周期 ===== */

motor_info_storage_status_t motor_info_storage_init(void)
{
	struct motor_info_storage *pobj = &g_motor_info_storage;

	/* 1. 装配 ops 方法表指针（一行装配所有方法） */
	pobj->ops = &s_motorinfo_ops;

	/* 2. 初始化 dev_flash 子设备（单扇区配置：total_size=page_size=2048）
	 *    sector_count=1，写入时 next=(0+1)%1=0 始终写扇区0，固定地址
	 *    dev_flash_init 扫描扇区 flag，last_sequence>0 表示有写入过的数据 */
	dev_flash_init(&pobj->flash_dev,
	               MOTORINFO_FLASH_START_ADDR,
	               MOTORINFO_FLASH_TOTAL_SIZE,
	               MOTORINFO_FLASH_PAGE_SIZE);
	pobj->inited = pobj->flash_dev.inited ? true : false;

	/* 3. 加载全局 motor_info（上电自动加载，仅一次） */
	if (!pobj->motor_info_loaded)
	{
		bool need_init_default = true; /* 是否走首次上电默认路径 */

		/* 路径A：从持久化介质加载（EEPROM 优先，其次 Flash）。
		 * 不再用 Flash last_sequence 门控：首次上电 Flash 为空时，
		 * 只要 EEPROM 有有效数据也应加载（满足"上电默认使用 EEPROM"）。
		 * ops_load 内部已做 EEPROM→Flash 优先级回退。 */
		if (pobj->inited)
		{
			motor_info_storage_status_t lr = motorinfo_ops_load(pobj, &pobj->motor_info);
			if (lr == MOTOR_INFO_STORAGE_OK)
			{
				/* 存储数据有效（config_version + CRC + range 全通过）
				 * 直接用存储数据，不调用 motor_info_init
				 * apply_info(零值fallback)：未标定字段（零值）用 profile 补缺，
				 * 已标定字段（非零）保留存储中的标定值 */
				motor_profile_apply_info(&pobj->motor_info);
				need_init_default = false;
			}
			/* lr != OK：介质无有效数据/损坏 → 走路径B */
		}

		/* 路径B：首次上电——init + apply_default + save */
		if (need_init_default)
		{
			/* B1. 填默认值（header 元数据 magic/version/blocks 索引 + 通用默认值） */
			(void)motor_info_init(&pobj->motor_info);

			/* B2. profile 无条件覆盖（含 config_version 设置）
			 *     无视 init 的非零默认值，强制用 profile 覆盖 7 个字段 */
			motor_profile_apply_info_default(&pobj->motor_info);

			/* B3. 回写存储(默认仅 EEPROM, 不写 Flash; EEPROM 不可用时自动回退 Flash)
			 *     保存失败不阻断启动（内存数据已正确），但输出错误便于排查 */
			if (pobj->inited)
			{
				motor_info_storage_status_t sr = motorinfo_ops_save(pobj, &pobj->motor_info, 0u);
				if (sr != MOTOR_INFO_STORAGE_OK)
				{
					/* save 失败: Flash 擦写异常, 下次上电仍走路径B */
				}
			}
		}

		pobj->motor_info_loaded = true;
	}

	return pobj->inited ? MOTOR_INFO_STORAGE_OK : MOTOR_INFO_STORAGE_ERR_FLASH;
}

void motor_info_storage_deinit(void)
{
	struct motor_info_storage *pobj = &g_motor_info_storage;
	if (pobj->ops && pobj->ops->deinit)
	{
		pobj->ops->deinit(pobj);
	}
}

/* ===== 对外包装函数（包装 ops 方法，集中处理 pobj/ops 检查） ===== */

motor_info_t *motor_info_storage_get(void)
{
	struct motor_info_storage *pobj = &g_motor_info_storage;
	if (pobj == NULL || pobj->ops == NULL || pobj->ops->get == NULL)
		return NULL;
	return pobj->ops->get(pobj);
}

motor_info_storage_status_t motor_info_storage_load(motor_info_t *cfg)
{
	struct motor_info_storage *pobj = &g_motor_info_storage;
	if (pobj == NULL || pobj->ops == NULL || pobj->ops->load == NULL)
		return MOTOR_INFO_STORAGE_ERR_INIT;
	return pobj->ops->load(pobj, cfg);
}

motor_info_storage_status_t motor_info_storage_save(const motor_info_t *cfg, uint32_t flags)
{
	struct motor_info_storage *pobj = &g_motor_info_storage;
	if (pobj == NULL || pobj->ops == NULL || pobj->ops->save == NULL)
		return MOTOR_INFO_STORAGE_ERR_INIT;
	return pobj->ops->save(pobj, cfg, flags);
}

/* ===== 强符号覆盖: jm_app_motor_info_storage_save =====
 * 覆盖 jm_proto_ops.c 中的 __weak jm_app_motor_info_storage_save()。
 * 协议层收到 0xEA 时调用本强符号，接收到 flags 后向下传递。
 * 返回值约定（与弱符号一致）：0=成功, >0=越界 param_id, <0=系统错误。*/
motor_info_storage_status_t jm_app_motor_info_storage_save(const motor_info_t *cfg, uint32_t flags)
{
	return motor_info_storage_save(cfg, flags);
}


#endif /* USE_DEV_FLASH */
