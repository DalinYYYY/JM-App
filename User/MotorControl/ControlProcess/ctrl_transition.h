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
 * @brief 参考渐变形状
 * @details LINEAR: 斜率恒定的线性斜坡；
 *          SCURVE: smoothstep(r)=r²(3-2r)，加速度连续无拐点，
 *          峰值斜率为平均值的 1.5 倍（rate 模式已按 1.5 倍自动补偿时长）。
 */
typedef enum
{
	TRANSITION_SHAPE_LINEAR = 0,
	TRANSITION_SHAPE_SCURVE
} transition_shape_e;

/**
 * @brief 过渡引擎结构体
 * @details 工作在参考层：对本模块输出的 motor_ref_t 做平滑过渡。
 *          - 同 ctrl_type（量纲一致）：对目标值做形状渐变，输出连续无跳变；
 *          - 异 ctrl_type（量纲不同）：ctrl_type 立即切换（环路结构即时生效，
 *            下游三环预装载积分保证输出无扰），数值字段仍按形状渐变，
 *            消除使能/切模式瞬间的给定阶跃。
 */
typedef struct
{
	transition_state_e state;
	uint32_t elapsed;	 // 已调用次数（每次 transition_update 自增）
	uint32_t duration;	 // 过渡总时长，以调用次数计
	transition_shape_e shape; // 渐变形状（启动时由配置指定）
	motor_ref_t old_ref; // 过渡起点参考（源模式）
	motor_ref_t target_ref; // 渐变朝向的目标（首拍登记，目标再变化检测用）
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
 * @param shape 渐变形状
 */
void transition_start(transition_t *trans, uint32_t duration, const motor_ref_t *old_ref, transition_shape_e shape);

/**
 * @brief 更新过渡引擎状态
 * @param trans 过渡引擎指针
 * @param new_ref 目标模式当前参考输出
 * @param out_ref 混合后的参考输出（含解析加速度 accel）
 * @param dt 控制周期(s)，用于把每拍混合斜率换算为加速度(rad/s²)
 * @return true: 过渡完成; false: 过渡进行中
 * @note 过渡进行中 out_ref->accel 为混合速度的解析导数：
 *       LINEAR 恒斜率 Δv/T；SCURVE 按 smoothstep 导数 6r(1-r)·Δv/T。
 *       过渡完成/空闲时 accel 透传目标参考值（由模式生成）。
 */
bool transition_update(transition_t *trans, const motor_ref_t *new_ref, motor_ref_t *out_ref, float dt);

/**
 * @brief 强制完成过渡
 * @param trans 过渡引擎指针
 */
void transition_force_complete(transition_t *trans);

/* ===== 同模式目标值渐变 ===== */

/**
 * @brief 同模式目标值渐变配置
 * @details 当同一运行模式内目标值突变超过阈值时，启动参考层线性渐变，
 *          避免目标阶跃导致 PID 大误差冲击。
 *
 * 配置语义：
 *  - enable: 总开关，false 时完全跳过 ref_smooth 逻辑
 *  - *_thresh: 触发阈值，突变绝对值超过此值才启动渐变；置 0 表示该字段不检测
 *  - 过渡时长二选一：
 *    * 速率模式（优先）：对应 rate 字段 > 0 时，duration = ceil(|delta| / (rate × dt))
 *    * 固定时长模式：所有 rate == 0 时，duration = smooth_duration
 *
 * rate 单位为"每秒最大变化量"：pos_rate=rad/s, vel_rate=rad/s²,
 * torque_rate=N·m/s, current_rate=A/s。
 */
typedef struct
{
	bool enable;			 /* 总开关，false 完全禁用同模式渐变 */
	uint32_t smooth_duration; /* 固定时长模式：渐变时长(transition_update 调用次数) */

	/* 触发阈值 */
	float pos_thresh;		 /* 位置目标突变阈值(rad) */
	float vel_thresh;		 /* 速度目标突变阈值(rad/s) */
	float torque_thresh;	 /* 力矩目标突变阈值(N·m) */
	float current_thresh;	 /* 电流目标突变阈值(A)，用于 id/iq */
	float voltage_thresh;	 /* 电压目标突变阈值(V)，用于 ud/voltage */
	float duty_thresh;		 /* 占空比突变阈值 */

	/* 速率模式：> 0 时按速率自动算 duration，覆盖 smooth_duration */
	float pos_rate;		 /* 位置变化速率上限(rad/s) */
	float vel_rate;		 /* 速度变化速率上限(rad/s²) */
	float torque_rate;	 /* 力矩变化速率上限(N·m/s) */
	float current_rate;	 /* 电流变化速率上限(A/s) */

	transition_shape_e shape; /* 渐变形状：LINEAR 线性斜坡 / SCURVE S曲线 */
} ref_smooth_cfg_t;

/* 配置完全内聚于 transition_mgr_t.smooth_cfg，不提供全局实例。
 * 运行期调参通过 sys->trans_mgr.smooth_cfg 字段访问（见 Task 8 / Task 9）。*/

/**
 * @brief 用默认值初始化渐变配置
 * @param cfg 配置指针
 * @note 默认: enable=true, smooth_duration=500,
 *             pos_thresh=0.1rad, vel_thresh=1rad/s, torque_thresh=0.1Nm,
 *             current_thresh=0.5A, voltage_thresh=1V, duty_thresh=0.1,
 *             pos_rate=100rad/s, vel_rate=500rad/s²,
 *             torque_rate=20Nm/s, current_rate=100A/s, shape=LINEAR
 *       （rate 模式默认启用，覆盖 smooth_duration）
 */
void ref_smooth_cfg_init_defaults(ref_smooth_cfg_t *cfg);

/**
 * @brief 按速率配置计算过渡时长(transition_update 调用次数)
 * @param raw 目标参考
 * @param prev 起点参考
 * @param cfg 渐变配置
 * @param dt 控制周期(s)
 * @return 过渡时长；0 表示速率模式不适用（rate 全<=0 或无差异）
 * @note duration_i = ceil(|delta_i| / (rate_i × dt))，取各字段最大值，
 *       SCURVE 自动 ×1.5 补偿峰值斜率。
 *       同模式渐变与模式切换过渡共用：后者在首拍调用，
 *       使使能/切模式的目标阶跃也遵循 rate 斜坡。
 */
uint32_t transition_calc_duration_by_rate(const motor_ref_t *raw,
                                          const motor_ref_t *prev,
                                          const ref_smooth_cfg_t *cfg,
                                          float dt);

/**
 * @brief 检测同模式内目标值突变，超阈值时启动参考层渐变
 * @param trans 过渡引擎
 * @param run_state 当前运行子状态（白名单判据，用于排除 MIT/HOLD/直控模式）
 * @param raw_ref 本拍 dispatch 输出的原始参考（目标）
 * @param prev_ref 上一拍实际输出的参考（渐变起点）
 * @param cfg 渐变配置
 * @param dt 控制周期(s)，用于 rate 模式算 duration
 * @return true 已启动渐变; false 未启动（差异在阈值内或不适用）
 * @note - cfg->enable=false 直接返回 false
 *       - trans->state == TRANSITION_IN_PROGRESS：过渡中不重启，返回 false
 *       - trans->state == TRANSITION_COMPLETED：复位为 IDLE 并返回 false（本拍不检测，
 *         避免模式切换过渡刚完成那拍被误判为同模式突变）
 *       - 判据用 run_state 白名单：仅 POSITION/POSITION_VELOCITY/POSITION_TORQUE/
 *         VELOCITY/TORQUE/CURRENT 启用；MIT/HOLD/OPEN_LOOP/DUTY/VOLTAGE 等返回 false。
 *         （MIT 与 TORQUE 的 ctrl_type 同为 REF_CTRL_TORQUE，必须用 run_state 区分）
 *       - ctrl_type 与 prev_ref 不一致时返回 false（交由模式切换过渡处理）
 *       - ctrl_type 仅用于选择比较字段（pos/vel/torque/id-iq）
 *       - rate 模式：duration = max(ceil(|delta_i| / (rate_i × dt)))，至少为 1
 *       - 固定时长模式（所有 rate==0）：duration = cfg->smooth_duration
 */
bool transition_ref_smooth_check(transition_t *trans,
								 run_state_e run_state,
								 const motor_ref_t *raw_ref,
								 const motor_ref_t *prev_ref,
								 const ref_smooth_cfg_t *cfg,
								 float dt);

#endif /* __TRANSITION_H__ */
