/**
 * @file        motor_info_calib.c
 * @brief       标定结果回流 motor_info 的提交 API 实现
 *
 * @author      Dalin (dalin@robot.com)
 * @version     1.0
 * @date        2026-07-02
 */
#include "motor_info_calib.h"
#include "motor_info_storage.h"

#if defined(USE_DEV_FLASH)

/* ===== 内部工具：取全局 motor_info 句柄并校验 ===== */
static motor_info_t *get_info_checked(void)
{
    return motor_info_storage_get();  /* init 后保证非 NULL */
}

/* ===== L2.3 R ===== */
int motor_info_calib_submit_r(float r)
{
    motor_info_t *p = get_info_checked();
    if (p == NULL) return -1;
    return motor_info_set_phase_resistance(p, r);
}

/* ===== L2.4 Ld ===== */
int motor_info_calib_submit_ld(float ld)
{
    motor_info_t *p = get_info_checked();
    if (p == NULL) return -1;
    return motor_info_set_phase_inductance_d(p, ld);
}

/* ===== L2.5 Lq ===== */
int motor_info_calib_submit_lq(float lq)
{
    motor_info_t *p = get_info_checked();
    if (p == NULL) return -1;
    return motor_info_set_phase_inductance_q(p, lq);
}

/* ===== L2.6 flux ===== */
int motor_info_calib_submit_flux(float flux)
{
    motor_info_t *p = get_info_checked();
    if (p == NULL) return -1;
    return motor_info_set_flux_linkage(p, flux);
}

/* ===== L2.2 pole_pairs ===== */
int motor_info_calib_submit_pole_pairs(uint32_t pole_pairs)
{
    motor_info_t *p = get_info_checked();
    if (p == NULL) return -1;
    return motor_info_set_pole_pairs(p, pole_pairs);
}

/* ===== L3.1 编码器零位（一次提交 3 个相关字段）===== */
int motor_info_calib_submit_enc_zero(float elec_angle_bias, float enc_offset, int32_t enc_direction)
{
    motor_info_t *p = get_info_checked();
    if (p == NULL) return -1;
    int rc;
    rc = motor_info_set_elec_angle_bias(p, elec_angle_bias);
    if (rc != 0) return rc;
    rc = motor_info_set_enc_offset(p, enc_offset);
    if (rc != 0) return rc;
    rc = motor_info_set_enc_direction(p, enc_direction);
    return rc;
}

/* ===== L3.2 编码器方向 ===== */
int motor_info_calib_submit_enc_direction(int32_t enc_direction)
{
    motor_info_t *p = get_info_checked();
    if (p == NULL) return -1;
    return motor_info_set_enc_direction(p, enc_direction);
}

/* ===== L7 置位 is_calibrated ===== */
int motor_info_calib_mark_calibrated(void)
{
    motor_info_t *p = get_info_checked();
    if (p == NULL) return -1;
    return motor_info_set_is_calibrated(p, 1U);
}

/* ===== 重置标定状态 ===== */
int motor_info_calib_reset(void)
{
    motor_info_t *p = get_info_checked();
    if (p == NULL) return -1;

    (void)motor_info_set_is_calibrated(p, 0U);
    (void)motor_info_set_phase_resistance(p, 0.0f);
    (void)motor_info_set_phase_inductance_d(p, 0.0f);
    (void)motor_info_set_phase_inductance_q(p, 0.0f);
    (void)motor_info_set_flux_linkage(p, 0.0f);
    (void)motor_info_set_elec_angle_bias(p, 0.0f);
    (void)motor_info_set_enc_offset(p, 0.0f);
    (void)motor_info_set_enc_direction(p, 0);

    return 0;
}

#endif /* USE_DEV_FLASH */
