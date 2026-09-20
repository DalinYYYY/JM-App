#ifndef CTRL_TRANSITION_MGR_H
#define CTRL_TRANSITION_MGR_H

#include "ctrl_transition.h"
#include "motor_control.h"
#include "motor_info.h" /* motor_info_t: softstart 持久化镜像 */

/* 前向声明打破与 system_state.h 的循环依赖。
 * 前提：system_state.h 需把匿名 typedef 改为带 tag 的
 *      typedef struct system_state_s { ... } system_state_t;（见 Task 3 Step 1）。*/
typedef struct system_state_s system_state_t;

/**
 * @brief 过渡管理器：收拢所有过渡策略（模式切换过渡 + 同模式渐变 + 应急终止）
 * @details transition_t 实例与 ref_smooth_cfg_t 配置均内聚于此结构，
 *          system_state.c 不再直接操作 transition_*，改为调用 transition_mgr_*。
 *          不再保留全局 g_ref_smooth_cfg，配置完全由 mgr->smooth_cfg 持有。
 */
typedef struct
{
	transition_t trans;		    /* 过渡引擎实例（从 system_state_t 迁入） */
	ref_smooth_cfg_t smooth_cfg; /* 同模式渐变配置（完全内聚） */
	run_state_e target_run_state; /* 模式切换过渡的目标 run_state */
	float dt;					/* 控制周期(s)，默认 100µs，供 rate 模式算 duration，运行期可改 */
	bool stop_pending;			/* STOP 延迟过渡标志：过渡完成后才真正切 READY */
} transition_mgr_t;

/**
 * @brief 初始化过渡管理器
 * @param mgr 管理器指针
 * @param dt 控制周期(s)，默认 100µs(1e-4)，存入 mgr->dt
 */
void transition_mgr_init(transition_mgr_t *mgr, float dt);

/**
 * @brief 顶层状态变化通知（替代 top_fsm_switch 内的 transition_force_complete）
 * @details 退出 RUN 或进入 FAULT/SAFETY 时调用，立即结束任何进行中的过渡
 * @param mgr 管理器指针
 * @param new_state 即将进入的顶层状态
 */
void transition_mgr_on_top_fsm_change(transition_mgr_t *mgr, top_fsm_e new_state);

/**
 * @brief 模式切换请求（替代 run_state_switch 内的 transition_start）
 * @param mgr 管理器指针
 * @param new_state 目标运行子状态
 * @param trans_count 过渡时长(调用次数)
 * @param cur_ref 当前参考（作为过渡起点 old_ref）
 */
void transition_mgr_on_mode_switch(transition_mgr_t *mgr,
								   run_state_e new_state,
								   uint32_t trans_count,
								   const motor_ref_t *cur_ref);

/**
 * @brief STOP 命令延迟过渡：保持当前模式，目标渐变到零，完成后自动切 READY
 * @details 不立即切 top_fsm，而是把 cmd 目标清零（POSITION 类停在当前 ref 位置），
 *          设 stop_pending，让 transition_mgr_step 的 ref_smooth 引擎渐变到零。
 *          渐变完成后由 step 自动调 top_fsm_switch(READY)。
 *          进行中的过渡（模式切换/同模式渐变）被终止，停机渐变从当前混合
 *          参考起步；模式切换过渡中 STOP 时先提交目标 run_state
 *          （过渡期间 run_state 仍为旧值，判据按目标模式）。
 *          ESTOP/FAULT 触发 on_top_fsm_change 时立即中止 stop_pending。
 * @param mgr 管理器指针
 * @param sys 系统状态（读 run_state/ref，写 cmd）
 * @return true 已启动停机过渡（延迟切 READY）；false 模式不支持（调用方应直接切 READY）
 */
bool transition_mgr_on_stop(transition_mgr_t *mgr, system_state_t *sys);

/**
 * @brief 取消停机渐变（STOP 后、渐变完成前收到新运动指令时调用）
 * @details 仅清除 stop_pending：停机目标不再被强制清零，残留的 stop_pending
 *          也不会在切换过渡完成后误触发立即停机。进行中的减速渐变保持，
 *          新目标经"目标再变化检测"重启渐变平滑接管（同模式），或走
 *          模式切换过渡（不同模式）。
 */
void transition_mgr_cancel_stop(transition_mgr_t *mgr);

/**
 * @brief motor_info → 运行时 smooth_cfg 全量应用
 * @details 启动加载 / 0xA3 镜像写 / 0xE7 写 softstart PID 后的同步 hook 三方共用。
 *          softstart_valid=0（老配置迁移或未配置）时跳过，保持编译期默认。
 *          含 g_run_state_trans_count（跨模式兜底时长）。
 * @param mgr 管理器指针
 * @param info motor_info 持久化镜像（读 advanced 块 softstart 字段）
 */
void transition_mgr_apply_softstart(transition_mgr_t *mgr, const motor_info_t *info);

/**
 * @brief 每拍步进（替代 motor_control_loop 的 if/else 分支 + 同模式渐变逻辑）
 * @details 统一入口：模式切换过渡中则 blend；否则 dispatch + ref_smooth 检测 + 渐变。
 *          结果写入 sys->motor.ref。
 * @param mgr 管理器指针
 * @param sys 系统状态（读 run_state/dt，写 motor.ref）
 */
void transition_mgr_step(transition_mgr_t *mgr, system_state_t *sys);

#endif /* CTRL_TRANSITION_MGR_H */
