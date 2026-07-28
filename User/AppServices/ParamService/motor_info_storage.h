/**
 * @file        motor_info_storage.h
 * @brief       motor_info Flash 持久化设备对象（基于通用 dev_flash 设备）
 *
 * @details     应用层设备对象，封装 motor_info_t 的 Flash 持久化语义：
 *              - 持有全局唯一实例 g_motor_info_storage（extern 单例）
 *              - 组合子设备 dev_flash_t（提供页擦写+磨损均衡）
 *              - 持有全局 g_motor_info 实例字段
 *              - 共享 static const ops 方法表（进 Flash 只读，省 RAM）
 *              - 上电时 motor_info_storage_init() 一次性完成：
 *                  dev_flash_init → motor_info_init(默认) → Flash加载 → profile覆盖
 *              - motor_info_storage_get() 返回已初始化的 g_motor_info 句柄
 *              - motor_info_storage_save() 校验后计算 CRC 写入 Flash
 *              - 强符号 jm_app_motor_info_storage_save 覆盖协议层弱符号，接入 0xEA
 *
 * @par 模块契约（C-OOP 设备对象规范）
 *   所有权：模块自持 g_motor_info_storage 单例 + dev_flash 子设备（静态分配，无 malloc）。
 *           外部经 motor_info_storage_get() 取 motor_info 句柄，不提供存储。
 *   生命周期：motor_info_storage_init() 构造 + 自动加载（init→get/load/save→deinit）。
 *             init 可重复调用（幂等，二次调用仅重建 dev_flash 不重载 motor_info）。
 *             deinit 后不得再调用 get/load/save（行为未定义）。
 *   实时性：init/get/load 不阻塞；save 内部关中断擦写 Flash 约 10-30ms，
 *           严禁在 ISR / 电流环 / 控制环调用，仅可在通信线程（0xEA 命令处理）调用。
 *   错误语义：见 motor_info_storage_status_t 枚举定义。
 *
 * @note        Flash 分区由 Keil 链接脚本预留（用户自行配置）。
 *              存储地址定义在本头文件的宏中，与应用逻辑同模块。
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
#ifndef __MOTOR_INFO_STORAGE_H__
#define __MOTOR_INFO_STORAGE_H__

#include "dev_config.h"

#if defined(USE_DEV_FLASH)

#include "motor_info.h"
#include "dev_flash.h" /* 组合子设备:通用 Flash 设备 */

#ifdef __cplusplus
extern "C"
{
#endif

/* ===== motor_info Flash 存储地址定义 =====
 * STM32G474CB 128KB 双Bank Flash，Bank2 末尾 4KB 保留为 motor_info 数据区：
 *   Bank1: 0x08000000-0x0800FFFF (64KB)
 *   Bank2: 0x08040000-0x0804FFFF (64KB)   ← 双Bank地址不连续，中间为空洞
 *   存储区起始 0x0804F000（=Bank2基址 + 0xF000 = Bank2末尾4KB），总大小 4KB，
 *   页大小 2KB（双Bank），2 个扇区 A/B 轮转磨损均衡。
 * ODrive(F405RG 1MB) 板在 dev_config_board.h 中覆盖为 Sector 11(0x080E0000, 128KB),
 * 单扇区无磨损均衡。
 * @note 须在 Keil 链接脚本中将代码区限制在存储区起始地址之前。*/
#ifndef MOTORINFO_FLASH_START_ADDR
#define MOTORINFO_FLASH_START_ADDR 0x0804F000U
#endif
#ifndef MOTORINFO_FLASH_TOTAL_SIZE
#define MOTORINFO_FLASH_TOTAL_SIZE 0x00001000U /* 4KB */
#endif
#ifndef MOTORINFO_FLASH_PAGE_SIZE
#define MOTORINFO_FLASH_PAGE_SIZE  2048U       /* 2KB，双Bank页大小 */
#endif

	/**
 * @brief  模块状态码枚举
 * @note   值域约定：
 *         - 0            = 成功
 *         - < 0          = 系统错误（参数/初始化/Flash 硬件/CRC 计算失败）
 *         - > 0          = 数据状态（load 路径: 1=无数据/2=CRC失败/3=范围越界；
 *                                     save 路径: 透传 motor_info_validate 的越界 param_id）
 *         此约定与 jm_proto_ops.c 的 `rc>0 ? OUT_OF_RANGE : FLASH` 判断兼容。
 */
	typedef enum
	{
		/* ===== 成功 ===== */
		MOTOR_INFO_STORAGE_OK = 0, /* 操作成功 */

		/* ===== 数据状态 (>0)：调用方可回退默认值，非致命 ===== */
		MOTOR_INFO_STORAGE_NO_DATA = 1,    /* load: Flash 无有效数据(首次上电/全擦除) */
		MOTOR_INFO_STORAGE_CRC_FAIL = 2,   /* load: CRC 校验失败，数据损坏 */
		MOTOR_INFO_STORAGE_RANGE_FAIL = 3, /* load: 字段范围校验失败 */
		/* 注: >3 区间预留给 motor_info_validate 透传的 param_id (save 路径) */

		/* ===== 系统错误 (<0)：致命，调用方不应继续使用 ===== */
		MOTOR_INFO_STORAGE_ERR_ARG = -1,    /* 空指针/非法参数 */
		MOTOR_INFO_STORAGE_ERR_FLASH = -2,  /* Flash 读/写失败(通用, 兼容旧代码) */
		MOTOR_INFO_STORAGE_ERR_INIT = -3,   /* 服务未初始化 */
		/* 详细 Flash 错误码(供 0xEA 应答区分擦写/校验失败) */
		MOTOR_INFO_STORAGE_ERR_FLASH_WRITE = -4,  /* Flash 擦写失败(3 次重试后仍失败) */
		MOTOR_INFO_STORAGE_ERR_FLASH_VERIFY = -5, /* Flash 回读校验失败(CRC/范围不匹配) */
	} motor_info_storage_status_t;

	/* ===== 设备对象前置声明（供 ops 函数指针类型引用） ===== */
	struct motor_info_storage;

	/**
 * @brief  设备对象方法表（ops 虚函数表）
 * @details 同一类型所有实例共享同一张 static const ops 表（进 Flash 只读），
 *          对象只持有 `const ops *` 指针指向它，构造时一行装配所有方法。
 *          对外提供包装函数，由包装函数检查 pobj、ops、必要函数指针。
 */
	typedef struct
	{
		motor_info_t *(*get)(struct motor_info_storage *pobj);                                         /* 取全局 motor_info 句柄 */
		motor_info_storage_status_t (*load)(struct motor_info_storage *pobj, motor_info_t *cfg);       /* 从 Flash 加载 */
		motor_info_storage_status_t (*save)(struct motor_info_storage *pobj, const motor_info_t *cfg); /* 保存到 Flash */
		void (*deinit)(struct motor_info_storage *pobj);                                               /* 反初始化 */
	} motor_info_storage_ops_t;

	/**
 * @brief  motor_info 存储设备对象结构体
 * @details 组合 dev_flash 子设备 + motor_info 实例 + 状态标志 + ops 指针。
 *          - 静态单例分配（g_motor_info_storage），无 malloc。
 *          - 共享 ops 表（static const，进 Flash）。
 */
	typedef struct motor_info_storage
	{
		/* 组合子设备：通用 Flash 设备（提供页擦写+磨损均衡） */
		dev_flash_t flash_dev;

		/* 全局唯一 motor_info 实例（外部经 get 方法取句柄） */
		motor_info_t motor_info;

		/* 状态标志 */
		bool inited;            /* Flash 存储服务初始化完成 */
		bool motor_info_loaded; /* motor_info 已加载默认+Flash+profile */

		/* 方法表指针（指向共享 static const ops，构造时装配） */
		const motor_info_storage_ops_t *ops;
	} motor_info_storage_t;

	/* ===== 全局单例（extern，调用方不需持有对象） ===== */
	extern motor_info_storage_t g_motor_info_storage;

	/* ===== 构造与生命周期 ===== */

	/**
 * @brief  构造 motor_info 存储设备对象并上电自动加载默认配置
 * @details 一次性完成全流程（上电自动加载）：
 *          1. 装配 ops 方法表指针
 *          2. dev_flash_init(地址, 大小, 页大小) 扫描扇区定位最新有效数据
 *          3. motor_info_init(&motor_info) 填默认值
 *          4. ops->load() 从 Flash 加载覆盖默认值
 *             （Flash 无数据/校验失败时保留默认值，系统仍可正常启动）
 *          5. motor_profile_apply_info(&motor_info) 编译期 profile 覆盖
 *             （硬件级电气身份参数，最终决定权）
 * @note   须在 motor_loop_init 之前、hardware_init 中调用。可重复调用（幂等）。
 *         不阻塞；可在主线程启动阶段调用。
 * @return MOTOR_INFO_STORAGE_OK 成功;
 *         MOTOR_INFO_STORAGE_ERR_FLASH dev_flash 初始化失败（Flash 读取异常）。
 */
	motor_info_storage_status_t motor_info_storage_init(void);

	/**
 * @brief  反初始化 motor_info 存储设备对象（包装 ops->deinit）
 * @details 清零内部状态标志，下一次 init 可重新加载。
 *          不释放资源（无动态内存），不擦除 Flash。
 * @note   调用后 get/load/save 不得再调用（行为未定义）。
 */
	void motor_info_storage_deinit(void);

	/* ===== 对外包装函数（包装 ops 方法，集中处理 pobj/ops 检查） ===== */

	/**
 * @brief  获取全局 motor_info 实例句柄（包装 ops->get）
 * @return 已初始化的 g_motor_info_storage.motor_info 指针。
 * @note   须先调用 motor_info_storage_init。返回指针指向模块内部存储，
 *         外部可读可写；写操作即修改全局 motor_info 实例。
 *         不阻塞；可在任意线程/上下文调用。
 */
	motor_info_t *motor_info_storage_get(void);

	/**
 * @brief  从 Flash 加载 motor_info 配置（包装 ops->load，覆盖传入 cfg）
 * @param  cfg  目标参数区指针（成功时整块覆盖 1024B）
 * @return MOTOR_INFO_STORAGE_OK            加载成功且校验通过;
 *         MOTOR_INFO_STORAGE_NO_DATA       Flash 无有效数据(首次上电);
 *         MOTOR_INFO_STORAGE_CRC_FAIL      CRC 校验失败;
 *         MOTOR_INFO_STORAGE_RANGE_FAIL    字段范围校验失败;
 *         MOTOR_INFO_STORAGE_ERR_ARG       cfg 为空;
 *         MOTOR_INFO_STORAGE_ERR_FLASH     Flash 读取失败;
 *         MOTOR_INFO_STORAGE_ERR_INIT      服务未初始化.
 * @note   不阻塞（Flash 读取无擦写）。可在主线程/通信线程调用。
 */
	motor_info_storage_status_t motor_info_storage_load(motor_info_t *cfg);

	/**
 * @brief  将 motor_info 配置保存到 Flash（包装 ops->save）
 * @param  cfg  源参数区指针
 * @return MOTOR_INFO_STORAGE_OK            保存成功;
 *         > 0                                motor_info_validate 返回的首个越界 param_id;
 *         MOTOR_INFO_STORAGE_ERR_ARG       cfg 为空;
 *         MOTOR_INFO_STORAGE_ERR_FLASH     Flash 写入失败;
 *         MOTOR_INFO_STORAGE_ERR_INIT      服务未初始化.
 * @note   **阻塞**:dev_flash_write 内部关中断约 10-30ms，会阻塞所有中断
 *         (含电机控制等实时中断)。仅在 0xEA 命令处理线程调用，
 *         严禁在 ISR / 电流环 / 控制环调用。
 */
	motor_info_storage_status_t motor_info_storage_save(const motor_info_t *cfg);

#ifdef __cplusplus
}
#endif

#endif /* USE_DEV_FLASH */
#endif /* __MOTOR_INFO_STORAGE_H__ */
