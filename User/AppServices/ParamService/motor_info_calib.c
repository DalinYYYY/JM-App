/**
 * @file        motor_info_calib.c
 * @brief       标定结果回流 motor_info 的提交 API 实现
 *
 * @author      Dalin (dalinyy@163.com)
 * @version     1.0
 * @date        2026-07-02
 */
#include "motor_info_calib.h"
#include "motor_info_storage.h"

#if defined(USE_DEV_FLASH)

/* ===== 内部工具：取全局 motor_info 句柄并校验 ===== */
static motor_info_t *get_info_checked(void)
{
	return motor_info_storage_get(); /* init 后保证非 NULL */
}

/* ===== L2.3 R ===== */
int motor_info_calib_submit_r(float r)
{
	motor_info_t *p = get_info_checked();
	if (p == NULL)
		return -1;
	return motor_info_write_f32(p, MOTOR_INFO_PID_PHASE_RESISTANCE, r);
}

/* ===== L2.4 Ld ===== */
int motor_info_calib_submit_ld(float ld)
{
	motor_info_t *p = get_info_checked();
	if (p == NULL)
		return -1;
	return motor_info_write_f32(p, MOTOR_INFO_PID_PHASE_INDUCTANCE_D, ld);
}

/* ===== L2.5 Lq ===== */
int motor_info_calib_submit_lq(float lq)
{
	motor_info_t *p = get_info_checked();
	if (p == NULL)
		return -1;
	return motor_info_write_f32(p, MOTOR_INFO_PID_PHASE_INDUCTANCE_Q, lq);
}

/* ===== L2.6 flux ===== */
int motor_info_calib_submit_flux(float flux)
{
	motor_info_t *p = get_info_checked();
	if (p == NULL)
		return -1;
	return motor_info_write_f32(p, MOTOR_INFO_PID_FLUX_LINKAGE, flux);
}

/* ===== L2.2 pole_pairs ===== */
int motor_info_calib_submit_pole_pairs(uint32_t pole_pairs)
{
	motor_info_t *p = get_info_checked();
	if (p == NULL)
		return -1;
	return motor_info_write_u32(p, MOTOR_INFO_PID_POLE_PAIRS, pole_pairs);
}

/* ===== L2.1 direction（相序辨识产出，0=正向 1=反向）===== */
int motor_info_calib_submit_direction(uint32_t direction)
{
	motor_info_t *p = get_info_checked();
	if (p == NULL)
		return -1;
	return motor_info_write_u32(p, MOTOR_INFO_PID_DIRECTION, direction);
}

/* ===== L3.1 编码器零位（一次提交 3 个相关字段）===== */
int motor_info_calib_submit_enc_zero(float elec_angle_bias, float enc_offset, int32_t enc_direction)
{
	motor_info_t *p = get_info_checked();
	if (p == NULL)
		return -1;
	int rc;
	rc = motor_info_write_f32(p, MOTOR_INFO_PID_ELEC_ANGLE_BIAS, elec_angle_bias);
	if (rc != 0)
		return rc;
	rc = motor_info_write_f32(p, MOTOR_INFO_PID_ENC_OFFSET, enc_offset);
	if (rc != 0)
		return rc;
	rc = motor_info_write_i32(p, MOTOR_INFO_PID_ENC_DIRECTION, enc_direction);
	return rc;
}

/* ===== L3.2 编码器方向 ===== */
int motor_info_calib_submit_enc_direction(int32_t enc_direction)
{
	motor_info_t *p = get_info_checked();
	if (p == NULL)
		return -1;
	return motor_info_write_i32(p, MOTOR_INFO_PID_ENC_DIRECTION, enc_direction);
}

/* ===== L7 置位 is_calibrated ===== */
int motor_info_calib_mark_calibrated(void)
{
	motor_info_t *p = get_info_checked();
	if (p == NULL)
		return -1;
	return motor_info_write_u32(p, MOTOR_INFO_PID_IS_CALIBRATED, 1U);
}

/* ===== 重置标定状态 ===== */
int motor_info_calib_reset(void)
{
	motor_info_t *p = get_info_checked();
	if (p == NULL)
		return -1;

	(void)motor_info_write_u32(p, MOTOR_INFO_PID_IS_CALIBRATED, 0U);
	(void)motor_info_write_f32(p, MOTOR_INFO_PID_PHASE_RESISTANCE, 0.0f);
	(void)motor_info_write_f32(p, MOTOR_INFO_PID_PHASE_INDUCTANCE_D, 0.0f);
	(void)motor_info_write_f32(p, MOTOR_INFO_PID_PHASE_INDUCTANCE_Q, 0.0f);
	(void)motor_info_write_f32(p, MOTOR_INFO_PID_FLUX_LINKAGE, 0.0f);
	(void)motor_info_write_f32(p, MOTOR_INFO_PID_ELEC_ANGLE_BIAS, 0.0f);
	(void)motor_info_write_f32(p, MOTOR_INFO_PID_ENC_OFFSET, 0.0f);
	(void)motor_info_write_i32(p, MOTOR_INFO_PID_ENC_DIRECTION, 0);

	return 0;
}

/* ===== 重置标定状态（保留电机本体参数，仅清编码器+is_calibrated）=====
 * 用于场景 C: 同一电机重做编码器标定（如零位漂移），保留电气字段和限幅字段。*/
int motor_info_calib_reset_for_recalibration(void)
{
	motor_info_t *p = get_info_checked();
	if (p == NULL)
		return -1;

	/* 清除标定状态标志 */
	(void)motor_info_write_u32(p, MOTOR_INFO_PID_IS_CALIBRATED, 0U);
	/* 清除编码器标定字段（L3 标定结果）*/
	(void)motor_info_write_f32(p, MOTOR_INFO_PID_ELEC_ANGLE_BIAS, 0.0f);
	(void)motor_info_write_f32(p, MOTOR_INFO_PID_ENC_OFFSET, 0.0f);
	(void)motor_info_write_i32(p, MOTOR_INFO_PID_ENC_DIRECTION, 0);  /* 0 视为未标定，apply_info 会 fallback=1 */

	/* 保留：R/Ld/Lq/flux/pole_pairs/kt/inertia（电机本体参数）
	 *      peak_current/max_speed（运行时限幅参数）
	 *      protect_over_current（保护阈值）
	 *      decouple_algo/bemf_ff_enable/deadtime_comp_enable（解耦配置）*/
	return 0;
}

#else /* !USE_DEV_FLASH */

/* ===== 未启用 Flash 存储的 stub 实现 =====
 * V1 等未启用 USE_DEV_FLASH 的板级配置: 无 motor_info_t 全局实例, 标定结果
 * 仅写入运行期 motor_param_t(由 calib_level*.c 直接调用 motor_param_set_*),
 * 不持久化到 Flash。本组 stub 返回 0(成功)使标定流程继续, 实际不保存任何数据。
 * 上位机 0xEA 命令在 jm_proto_ops.c 中已被 #if 屏蔽, 不会触发保存。*/
int motor_info_calib_submit_r(float r)                 { (void)r; return 0; }
int motor_info_calib_submit_ld(float ld)               { (void)ld; return 0; }
int motor_info_calib_submit_lq(float lq)               { (void)lq; return 0; }
int motor_info_calib_submit_flux(float flux)           { (void)flux; return 0; }
int motor_info_calib_submit_pole_pairs(uint32_t pp)    { (void)pp; return 0; }
int motor_info_calib_submit_direction(uint32_t dir)   { (void)dir; return 0; }
int motor_info_calib_submit_enc_zero(float bias, float off, int32_t dir)
{ (void)bias; (void)off; (void)dir; return 0; }
int motor_info_calib_submit_enc_direction(int32_t dir) { (void)dir; return 0; }
int motor_info_calib_mark_calibrated(void)             { return 0; }
int motor_info_calib_reset(void)                       { return 0; }
int motor_info_calib_reset_for_recalibration(void)     { return 0; }

#endif /* USE_DEV_FLASH */
