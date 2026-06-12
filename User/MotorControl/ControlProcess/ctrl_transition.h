#ifndef __TRANSITION_H__
#define __TRANSITION_H__

#include <stdint.h>
#include <stdbool.h>
#include "motor_control.h"

/**
 * @brief 过渡引擎状态
 */
typedef enum
{
	TRANSITION_IDLE = 0,
	TRANSITION_IN_PROGRESS,
	TRANSITION_COMPLETED
} transition_state_e;

/**
 * @brief 过渡引擎结构体
 * @details 工作在参考层：对本模块输出的 motor_ref_t 做平滑过渡。
 *          - 同 ctrl_type（量纲一致）：对目标值做线性混合，输出连续无跳变；
 *          - 异 ctrl_type（量纲不同）：不混合，直接切到新参考，由下游三环
 *            检测 ctrl_type 变化后预装载积分实现无扰切换。
 */
typedef struct
{
	transition_state_e state;
	uint32_t elapsed;	 // 已调用次数（每次 transition_update 自增）
	uint32_t duration;	 // 过渡总时长，以调用次数计
	motor_ref_t old_ref; // 过渡起点参考（源模式）
	float ratio;
} transition_t;

/**
 * @brief 初始化过渡引擎
 * @param trans 过渡引擎指针
 */
void transition_init(transition_t *trans);

/**
 * @brief 启动模式切换过渡
 * @param trans 过渡引擎指针
 * @param duration 过渡时长(以 transition_update 调用次数计)
 * @param old_ref 源模式当前参考输出
 */
void transition_start(transition_t *trans, uint32_t duration, const motor_ref_t *old_ref);

/**
 * @brief 更新过渡引擎状态
 * @param trans 过渡引擎指针
 * @param new_ref 目标模式当前参考输出
 * @param out_ref 混合后的参考输出
 * @return true: 过渡完成; false: 过渡进行中
 */
bool transition_update(transition_t *trans, const motor_ref_t *new_ref, motor_ref_t *out_ref);

/**
 * @brief 强制完成过渡
 * @param trans 过渡引擎指针
 */
void transition_force_complete(transition_t *trans);

#endif /* __TRANSITION_H__ */
