/**
 * @file        motor_info_calib.h
 * @brief       标定结果回流 motor_info 的提交 API（按子模式粒度）
 *
 * @details     本模块是标定层(calib_levelN_*.c)与持久化层(motor_info_storage)之间的桥梁。
 *              各标定子模式在完成 motor_param_set_*() 之后立即调用对应的 submit_* API，
 *              本模块内部经 motor_info_storage_get() 拿全局 motor_info 句柄，
 *              调用 motor_info_set_*() 写入字段（含范围校验）。
 *              上位机随后通过 0xEA 命令把整个 motor_info 落盘 Flash。
 *
 * @par 模块契约
 *   所有权：本模块无状态，仅转发到 motor_info 全局实例。
 *   生命周期：须在 motor_info_storage_init() 之后调用（依赖全局实例已就绪）。
 *   实时性：submit API 仅写字段，不阻塞；可在标定线程调用。
 *   错误语义：0=成功，<0=系统错误(未初始化/句柄空)，>0=字段范围越界(透传 motor_info_set_*)。
 *
 * @note        L1 相电流 ADC offset 不入 motor_info（属驱动层运行时校准）。
 *              L4 力矩常数当前是桩实现，无数据可提交，本期不提供 submit API。
 *
 * @par 三大场景操作手册（启用 USE_DEV_FLASH 的板）
 *
 *   场景 A：新板子首次上电（无任何标定数据）
 *     1. 上电后 motor_info 全 0，motor_profile_apply_info() 使用 motor_profile.h 默认值
 *     2. 通过上位机选择电机型号预设，点击"应用到 motor_info (0xE7)"批量写入电气字段
 *        - phase_resistance / phase_inductance_d / phase_inductance_q
 *        - flux_linkage / pole_pairs / torque_constant / rotor_inertia
 *        - peak_current / max_speed  (Index 43/44, 运行期限幅参数)
 *     3. 点击"参数固化 (0xEA)"持久化到 Flash
 *     4. 重启或重新上电，标定算法（calib_config_runtime.h）与控制环
 *        （motor_profile_sync_to_param）从 motor_info 读取新参数生效
 *     5. 执行 L1~L3 标定（编码器零位/方向、R/Ld/Lq/flux 等）
 *     6. 标定完成后再次 0xEA 固化，is_calibrated 自动置 1
 *
 *   场景 B：换用新电机型号（已有板子，换电机本体）
 *     1. 上位机通过"管理预设"新建电机配置（或选择已有用户预设）
 *     2. 切换 ComboBox 到新电机型号，点击"应用到 motor_info (0xE7)"
 *     3. 点击"参数固化 (0xEA)"持久化电气字段与限幅参数
 *     4. 重启，标定算法自适应切换（CALIB_IS_LOW_R 基于 R 自动判断）
 *     5. 执行 L3 编码器标定（必须，新电机编码器零位/方向不同）
 *     6. 可选：执行 L2 电气参数辨识（如厂家手册参数不准）
 *     7. 标定完成后 0xEA 固化
 *
 *   场景 C：同一电机重新标定（编码器零位漂移、机械重构后）
 *     1. 上位机点击"重新标定 (0xEC)"，调用 motor_info_calib_reset_for_recalibration()
 *        - 清除：is_calibrated + enc_offset + elec_angle_bias + enc_direction
 *        - 保留：R/Ld/Lq/flux/pole_pairs/Kt/inertia 等电气字段
 *        - 保留：peak_current/max_speed 限幅字段
 *     2. 点击"参数固化 (0xEA)"把清除后的状态写入 Flash
 *     3. 重启，电机本体参数保留生效，仅编码器字段需重做
 *     4. 执行 L3 编码器零位/方向标定
 *     5. 标定完成后 0xEA 固化，is_calibrated 自动置 1
 *
 * @note        场景 B/C 中，0xEC 仅清 RAM，必须随后的 0xEA 固化才能持久化清除效果。
 *              场景 B 切换电机型号时不应使用 0xEC（会保留旧电机电气参数），
 *              应直接用 0xE7 覆盖写入新电机电气字段。
 *
 * @author      Dalin (dalinyy@163.com)
 * @version     1.2
 * @date        2026-07-20
 */
#ifndef __MOTOR_INFO_CALIB_H__
#define __MOTOR_INFO_CALIB_H__

#include "motor_info.h"

#ifdef __cplusplus
extern "C"
{
#endif

	/* ===== L2 子模式提交（每个子模式独立提交单字段或相关字段组）===== */

	/**
 * @brief  提交 L2.3 R（相电阻）标定结果
 * @param  r  相电阻(ohm)
 * @return 0=成功, <0=系统错误, >0=字段越界 param_id
 */
	int motor_info_calib_submit_r(float r);

	/**
 * @brief  提交 L2.4 Ld（d轴电感）标定结果
 * @param  ld  d轴电感(H)
 */
	int motor_info_calib_submit_ld(float ld);

	/**
 * @brief  提交 L2.5 Lq（q轴电感）标定结果
 * @param  lq  q轴电感(H)
 */
	int motor_info_calib_submit_lq(float lq);

	/**
 * @brief  提交 L2.6 flux（磁链）标定结果
 * @param  flux  磁链(Wb)
 */
	int motor_info_calib_submit_flux(float flux);

	/**
	 * @brief  提交 L2.1 direction（相序方向）标定结果
	 * @param  direction  电机方向(0=正向 1=反向)，UVW 任意两相互换时为 1
	 */
	int motor_info_calib_submit_direction(uint32_t direction);

	/**
	 * @brief  提交 L2.2 pole_pairs（极对数）标定结果
	 * @param  pole_pairs  极对数
	 */
	int motor_info_calib_submit_pole_pairs(uint32_t pole_pairs);

	/* ===== L3 子模式提交 ===== */

	/**
 * @brief  提交 L3.1 编码器零位标定结果
 * @param  elec_angle_bias  电角度偏移(rad)（零位标定为 0）
 * @param  enc_offset       编码器初始位置偏移(deg)
 * @param  enc_direction    编码器方向(1=CW, 零位标定先置 CW)
 */
	int motor_info_calib_submit_enc_zero(float elec_angle_bias, float enc_offset, int32_t enc_direction);

	/**
 * @brief  提交 L3.2 编码器方向标定结果
 * @param  enc_direction  编码器方向(1=CW, -1=CCW)
 */
	int motor_info_calib_submit_enc_direction(int32_t enc_direction);

	/* ===== L7 全流程完成置位 ===== */

	/**
 * @brief  标记电机已完成全流程标定(is_calibrated = 1)
 * @details 仅在 L7 全自动流程成功完成后调用。
 *          置位后下次上电 motor_profile_apply_info 不再用默认值覆盖。
 */
	int motor_info_calib_mark_calibrated(void);

	/* ===== 重置标定状态 ===== */

	/**
 * @brief  清除标定状态(is_calibrated = 0)，并清零电机电气字段
 * @details 用于上位机"恢复出厂"或重新标定前的清理。
 *          清除后 profile 默认值会在下次 motor_profile_apply_info 时生效。
 */
	int motor_info_calib_reset(void);

	/**
 * @brief  重置标定状态以便重新标定（保留电机本体参数）
 * @details 清除 is_calibrated + 编码器标定字段（enc_offset/elec_angle_bias/enc_direction），
 *          保留电气字段（R/Ld/Lq/flux/pole_pairs/kt/inertia）和限幅字段
 *          （peak_current/max_speed）。
 *          用于场景：同一电机重新做编码器标定（如机械重构后零位漂移），
 *          电机本体参数不变，仅需重做 L3 编码器零位/方向标定。
 * @return 0=成功, <0=系统错误
 */
	int motor_info_calib_reset_for_recalibration(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_INFO_CALIB_H__ */
