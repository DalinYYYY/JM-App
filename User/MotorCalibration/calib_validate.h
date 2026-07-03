#ifndef __CALIB_VALIDATE_H__
#define __CALIB_VALIDATE_H__

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "calib_config.h"

/* ===================== 标定结果合理性校验 =====================
 * 各 level 模块在写 motor_param_set_* 前调用，拒绝超物理范围的值。
 * 校验失败时 level 模块应返回 CALIB_STATE_FAILED + fail_reason=CALIB_FAIL_OUT_OF_RANGE。
 * 范围常量定义在 calib_config.h，可统一调参。*/

/* R 相电阻(Ω) */
static inline bool calib_validate_r(float r)
{
	return isfinite(r) && r >= CALIB_CFG_R_MIN_OHM && r <= CALIB_CFG_R_MAX_OHM;
}

/* Ld d轴电感(H) */
static inline bool calib_validate_ld(float ld)
{
	return isfinite(ld) && ld >= CALIB_CFG_LD_MIN_H && ld <= CALIB_CFG_LD_MAX_H;
}

/* Lq q轴电感(H) */
static inline bool calib_validate_lq(float lq)
{
	return isfinite(lq) && lq >= CALIB_CFG_LQ_MIN_H && lq <= CALIB_CFG_LQ_MAX_H;
}

/* flux 磁链(Wb) */
static inline bool calib_validate_flux(float flux)
{
	return isfinite(flux) && flux >= CALIB_CFG_FLUX_MIN_WB && flux <= CALIB_CFG_FLUX_MAX_WB;
}

/* 极对数 */
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

#endif /* __CALIB_VALIDATE_H__ */
