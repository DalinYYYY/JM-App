#ifndef __CALIB_VALIDATE_H__
#define __CALIB_VALIDATE_H__

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "calib_config.h"
#include "calib_config_runtime.h" /* 运行期派生参数 */

/* ===================== 标定结果合理性校验 =====================
 * 各 level 模块在写 motor_param_set_* 前调用，拒绝超物理范围的值。
 * 校验失败时 level 模块应返回 CALIB_STATE_FAILED + fail_reason=CALIB_FAIL_OUT_OF_RANGE。
 * 范围参数通过 calib_config_runtime.h 运行期派生，从 motor_info 读取电机身份。*/

/* R 相电阻(Ω) */
static inline bool calib_validate_r(float r)
{
	return isfinite(r) && r >= calib_cfg_r_min_ohm() && r <= calib_cfg_r_max_ohm();
}

/* Ld d轴电感(H) */
static inline bool calib_validate_ld(float ld)
{
	return isfinite(ld) && ld >= calib_cfg_ld_min_h() && ld <= calib_cfg_ld_max_h();
}

/* Lq q轴电感(H) */
static inline bool calib_validate_lq(float lq)
{
	return isfinite(lq) && lq >= calib_cfg_lq_min_h() && lq <= calib_cfg_lq_max_h();
}

/* flux 磁链(Wb) */
static inline bool calib_validate_flux(float flux)
{
	return isfinite(flux) && flux >= calib_cfg_flux_min_wb() && flux <= calib_cfg_flux_max_wb();
}

/* 极对数（通用范围，与电机型号无关）*/
static inline bool calib_validate_pole_pairs(uint8_t pp)
{
	return pp >= CALIB_CFG_POLE_PAIRS_MIN && pp <= CALIB_CFG_POLE_PAIRS_MAX;
}

/* 编码器 offset（deg 单位，范围 -360.0 ~ 360.0）*/
static inline bool calib_validate_enc_offset(float offset)
{
	return (offset >= -360.0f) && (offset <= 360.0f);
}

/* 编码器方向（+1=CW / -1=CCW）*/
static inline bool calib_validate_enc_direction(int8_t dir)
{
	return dir == 1 || dir == -1;
}

/* J 轴系转动惯量(kg·m²)
 * 用绝对物理范围而非电机相对值：L6.1 测的是带载轴系总惯量，
 * 可远大于电机转子惯量（外接飞轮/磁滞测功机转子），相对范围会误杀。
 * 下限与 motor_pid_autotune 的就绪阈值一致(1e-7)，上限 1kg·m² 覆盖
 * 各类关节电机+台架负载的物理量级。*/
static inline bool calib_validate_inertia(float j)
{
	return isfinite(j) && j >= 1.0e-7f && j <= 1.0f;
}

#endif /* __CALIB_VALIDATE_H__ */
