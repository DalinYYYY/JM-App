/**
 * @file        dev_encoder_mt6835.c
 * @brief       MT6835 编码器适配层实现
 */
#include "dev_encoder_mt6835.h"

#if defined(USE_DEV_MT6835)

#include <string.h>
#include "assert_report.h"

dev_mt6835_t s_mt6835[MT6835_ID_MAX];

/* ---- MT6835 → 抽象编码器 适配函数 ---- */

static void encoder_mt6835_update(struct dev_encoder *enc)
{
    dev_mt6835_t *chip = (dev_mt6835_t *)enc->ctx;
    chip->update(chip);
    enc->mechanical_angle = chip->get_mechanical_angle(chip);
}

static float encoder_mt6835_get_mechanical_angle(struct dev_encoder *enc)
{
    return enc->mechanical_angle;
}

static void encoder_mt6835_set_offset(struct dev_encoder *enc, float offset_deg)
{
    dev_mt6835_t *chip = (dev_mt6835_t *)enc->ctx;
    chip->set_offset(chip, offset_deg);
}

static float encoder_mt6835_get_offset(struct dev_encoder *enc)
{
    dev_mt6835_t *chip = (dev_mt6835_t *)enc->ctx;
    return chip->offset;
}

static void encoder_mt6835_set_dir(struct dev_encoder *enc, int8_t dir)
{
    dev_mt6835_t *chip = (dev_mt6835_t *)enc->ctx;
    /* MT6835 约定: 1=CW(正向), -1=CCW(反向), 与项目其他编码器一致 */
    chip->set_dir(chip, (dir < 0) ? -1 : 1);
}

static int8_t encoder_mt6835_get_dir(struct dev_encoder *enc)
{
    dev_mt6835_t *chip = (dev_mt6835_t *)enc->ctx;
    return (chip->running_dir < 0) ? -1 : 1;
}

static float encoder_mt6835_get_raw_deg(struct dev_encoder *enc)
{
    dev_mt6835_t *chip = (dev_mt6835_t *)enc->ctx;
    /* MT6835 21bit 原始角度转 deg */
    return (float)chip->get_mechanical_angle_raw(chip) / MT6835_ANGLE_RESOLUTION * 360.0F;
}

static uint16_t encoder_mt6835_get_err_cnt(struct dev_encoder *enc)
{
    /* MT6835 驱动维护累计 SPI 失败计数(不清零), 适配层差分为"连续失败拍数":
     * 每拍查询一次, read_error_count 增长=失败拍+1, 不变=连续计数清零。
     * 语义对齐 MT6701 的 err_cnt(连续坏帧), 供故障检测 0x5105 判定。 */
    static uint32_t s_last_err_cnt = 0;
    static uint16_t s_consec_fail = 0;
    dev_mt6835_t *chip = (dev_mt6835_t *)enc->ctx;

    if (chip->read_error_count != s_last_err_cnt)
    {
        s_last_err_cnt = chip->read_error_count;
        if (s_consec_fail < 0xFFFFu)
            s_consec_fail++;
    }
    else
    {
        s_consec_fail = 0u;
    }
    return s_consec_fail;
}

static uint8_t encoder_mt6835_get_health(struct dev_encoder *enc)
{
    (void)enc; /* 磁场强度状态位图待驱动扩展, 预留 */
    return 0u;
}

/* ---- 工厂函数 ---- */

void dev_encoder_mt6835_create(dev_encoder_t *enc, mt6835_id_e id)
{
    assert_report(enc != NULL);
    assert_report(id < MT6835_ID_MAX);

    dev_mt6835_init(&s_mt6835[id], id);

    enc->ctx = &s_mt6835[id];
    enc->mechanical_angle = 0.0F;
    enc->update = encoder_mt6835_update;
    enc->get_mechanical_angle = encoder_mt6835_get_mechanical_angle;
    enc->set_offset = encoder_mt6835_set_offset;
    enc->get_offset = encoder_mt6835_get_offset;
    enc->set_dir = encoder_mt6835_set_dir;
    enc->get_dir = encoder_mt6835_get_dir;
    enc->get_raw_deg = encoder_mt6835_get_raw_deg;
    enc->get_err_cnt = encoder_mt6835_get_err_cnt;
    enc->get_health = encoder_mt6835_get_health;
}

#endif /* USE_DEV_MT6835 */
