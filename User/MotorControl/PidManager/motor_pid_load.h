/**
 * @file    motor_pid_load.h
 * @brief   PID 三环独立加载模块
 * @date    2026-07-08
 *
 * @details 本模块实现三环（电流/速度/位置）PID 参数来源的独立选择。
 *          每个环可独立选择三种来源：
 *          - PID_SOURCE_DEFAULT:  用 motor_param.c 默认值（不覆盖 motor_param_t）
 *          - PID_SOURCE_FLASH:    用 Flash 中 ControlParam_t 工程调试值
 *          - PID_SOURCE_AUTOTUNE: 用理论估计值（已由 autotune 写入 ControlParam_t）
 *
 *          source 标志存于模块内部 static 数组，不持久化到 Flash。
 *          每次上电默认 source=0（DEFAULT），行为向后兼容。
 *
 *          三环互不干扰：切换电流环 source 不影响速度/位置环。
 */
#ifndef __MOTOR_PID_LOAD_H__
#define __MOTOR_PID_LOAD_H__

#include <stdint.h>
#include "motor_info.h"
#include "motor_param.h"

/**
 * @brief  PID 参数来源
 * @note   三环各自独立选择，互不干扰
 */
typedef enum
{
	PID_SOURCE_DEFAULT = 0,  /* 用 motor_param.c 默认值（不覆盖 motor_param_t） */
	PID_SOURCE_FLASH = 1,    /* 用 Flash 中 ControlParam_t 工程调试值 */
	PID_SOURCE_AUTOTUNE = 2, /* 用理论估计值（零极点对消法，已写入 ControlParam_t） */
} pid_source_e;

/**
 * @brief  PID 控制环选择
 */
typedef enum
{
	PID_RING_CURRENT = 0,  /* 电流环（d/q 双轴） */
	PID_RING_VELOCITY = 1, /* 速度环 */
	PID_RING_POSITION = 2, /* 位置环 */
	PID_RING_MAX
} pid_ring_e;

/**
 * @brief  从 motor_info 按 pid_source 独立加载三环 PID 到 motor_param_t
 * @param  param  目标 motor_param_t（运行期参数）
 * @param  info   源 motor_info_t（Flash 加载的 ControlParam_t）
 * @note   source=DEFAULT 时不覆盖 param，保留 motor_param_init 的默认值。
 *         source=FLASH/AUTOTUNE 时从 info->blocks.control 读取对应字段。
 *         三环各自独立判断 source，互不影响。
 */
void motor_pid_load(motor_param_t *param, const motor_info_t *info);

/**
 * @brief  上电启动加载：Flash → autotune → default 三级回退
 * @param  param  目标 motor_param_t（已由 motor_param_init 填入默认值）
 * @param  info   源 motor_info_t（Flash 加载的 ControlParam_t + 辨识参数）
 * @note   三环各自独立判断，互不影响：
 *         1. Flash ControlParam 对应字段范围检查通过 → source=FLASH
 *         2. Flash 无效 → 尝试 autotune 计算（纯数学，不施加电压）
 *            辨识参数就绪 → source=AUTOTUNE，写入 param
 *         3. autotune 失败（未标定）→ source=DEFAULT，保留 motor_param.c 默认值
 *         此函数在 motor_loop_init 中替代 motor_pid_load 调用一次。
 */
void motor_pid_load_boot(motor_param_t *param, const motor_info_t *info);

/**
 * @brief  运行时 reload（协议修改 source 或 autotune 写入后调用）
 * @note   重新从 motor_info 按 source 加载到 motor_param_t，
 *         并同步到 motor_pid_profile 管理器。仅在 IDLE 态调用（并发安全）。
 */
void motor_pid_reload(void);

/**
 * @brief  设置指定环的参数来源
 * @param  ring  PID 控制环
 * @param  src   参数来源
 * @note   设置后需调用 motor_pid_reload() 生效。
 *         source 不持久化，重启回 DEFAULT。
 */
void motor_pid_set_source(pid_ring_e ring, pid_source_e src);

/**
 * @brief  获取指定环的参数来源
 * @param  ring  PID 控制环
 * @return 当前 source 值
 */
pid_source_e motor_pid_get_source(pid_ring_e ring);

#endif /* __MOTOR_PID_LOAD_H__ */
