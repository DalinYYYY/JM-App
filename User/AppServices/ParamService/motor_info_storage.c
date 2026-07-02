/**
 * @file        motor_info_storage.c
 * @brief       motor_info Flash 持久化设备对象实现
 *
 * @author      Dalin (dalin@robot.com)
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

/* motor_info_t = 1024B = 128 个 u64 */
#define MOTORINFO_LEN_U64 (PARAM_AREA_SIZE / 8U)

/* ===== 全局单例定义（外部经 extern 引用） ===== */
motor_info_storage_t g_motor_info_storage;

/* ===== 内部 CRC32 工具（私有 static 函数，不通过 ops） ===== */

/**
 * @brief  计算 motor_info_t 的 CRC32（跳过 header.crc32 字段）
 * @details 拷贝到临时缓冲 → crc32 置零 → 对 1024B 用 utils_crc32c 求值
 * @note   栈分配 1024B，仅 load 路径调用；save 路径已改为原地计算避免嵌套栈占用。
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
 * @return MOTOR_INFO_STORAGE_OK / NO_DATA / CRC_FAIL / RANGE_FAIL
 */
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
 * @details 调用 dev_flash->flash_read 读 1024B → 三重校验(magic+CRC+范围)
 */
static motor_info_storage_status_t motorinfo_ops_load(struct motor_info_storage *pobj, motor_info_t *cfg)
{
	if (cfg == NULL)
		return MOTOR_INFO_STORAGE_ERR_ARG;
	if (!pobj->inited)
		return MOTOR_INFO_STORAGE_ERR_INIT;

	/* 从 dev_flash 读 1024B = 128 u64，offset=0 */
	int rc = pobj->flash_dev.flash_read(&pobj->flash_dev, 0, (u64 *)cfg, MOTORINFO_LEN_U64);
	if (rc != DEV_EOK)
		return MOTOR_INFO_STORAGE_ERR_FLASH;

	/* 三重校验：magic → CRC32 → 字段范围 */
	return motorinfo_verify(cfg);
}

/**
 * @brief  将 motor_info 配置保存到 Flash
 * @details 原地计算 CRC32（避免嵌套调用 motorinfo_crc32_compute 再开 1024B 栈）
 *          → dev_flash->flash_write 轮转扇区+磨损均衡写入
 */
static motor_info_storage_status_t motorinfo_ops_save(struct motor_info_storage *pobj, const motor_info_t *cfg)
{
	if (cfg == NULL)
		return MOTOR_INFO_STORAGE_ERR_ARG;
	if (!pobj->inited)
		return MOTOR_INFO_STORAGE_ERR_INIT;

	/* 1. 保存前范围校验（透传 motor_info_validate 返回的越界 param_id，>0）
     *    当前业务允许越界数据落盘（host 可写任意值），故不阻断；
     *    保留校验仅用于未来策略切换（如需阻断则 if (vrc > 0) return vrc;） */
	(void)motor_info_validate(cfg);

	/* 2. 拷贝到栈上临时副本,原地计算 CRC32
     *    （避免嵌套调用 motorinfo_crc32_compute 再开 1024B 栈导致栈溢出） */
	motor_info_t tmp;
	memcpy(&tmp, cfg, PARAM_AREA_SIZE);
	tmp.blocks.header.crc32 = 0U;
	tmp.blocks.header.crc32 = utils_crc32c(tmp.raw, PARAM_AREA_SIZE);

	/* 3. 通过 dev_flash 写入(内部轮转扇区+磨损均衡，关中断约 10-30ms) */
	int rc = pobj->flash_dev.flash_write(&pobj->flash_dev, 0, (u64 *)&tmp, MOTORINFO_LEN_U64);
	return (rc == DEV_EOK) ? MOTOR_INFO_STORAGE_OK : MOTOR_INFO_STORAGE_ERR_FLASH;
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

	/* 2. 初始化 dev_flash 子设备（扫描扇区定位最新有效数据） */
	dev_flash_init(&pobj->flash_dev,
	               MOTORINFO_FLASH_START_ADDR,
	               MOTORINFO_FLASH_TOTAL_SIZE,
	               MOTORINFO_FLASH_PAGE_SIZE);
	pobj->inited = pobj->flash_dev.inited ? true : false;

	/* 3. 加载全局 motor_info（上电自动加载，仅一次） */
	if (!pobj->motor_info_loaded)
	{
		/* 3.1 填默认值（含 header.magic/version/block 索引表） */
		(void)motor_info_init(&pobj->motor_info);

		/* 3.2 尝试从 Flash 加载覆盖默认值
         *     返回 OK=加载成功；NO_DATA=Flash 无数据(首次上电)；CRC_FAIL/RANGE_FAIL=数据损坏
         *     任何非 OK 都保留默认值，系统仍可正常启动 */
		if (pobj->inited)
		{
			(void)motorinfo_ops_load(pobj, &pobj->motor_info);
		}

		/* 3.3 施加编译期 motor_profile 覆盖（硬件级电气身份参数，最终决定权） */
		motor_profile_apply_info(&pobj->motor_info);

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

motor_info_storage_status_t motor_info_storage_save(const motor_info_t *cfg)
{
	struct motor_info_storage *pobj = &g_motor_info_storage;
	if (pobj == NULL || pobj->ops == NULL || pobj->ops->save == NULL)
		return MOTOR_INFO_STORAGE_ERR_INIT;
	return pobj->ops->save(pobj, cfg);
}

/* ===== 强符号覆盖: jm_app_motor_info_storage_save =====
 * 覆盖 jm_proto_ops.c 中的 __weak jm_app_motor_info_storage_save()。
 * 协议层收到 0xEA 时调用本强符号，经 dev_flash 落盘。
 * 返回值约定（与弱符号一致）：0=成功, >0=越界 param_id, <0=系统错误。*/
motor_info_storage_status_t jm_app_motor_info_storage_save(const motor_info_t *cfg)
{
	return motor_info_storage_save(cfg);
}

#endif /* USE_DEV_FLASH */
