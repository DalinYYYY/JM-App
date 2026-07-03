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
 * @author      Dalin (dalin@robot.com)
 * @version     1.0
 * @date        2026-07-02
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

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_INFO_CALIB_H__ */
