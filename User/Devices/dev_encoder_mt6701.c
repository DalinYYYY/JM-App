/**
 * @file        dev_encoder_mt6701.c
 * @brief       MT6701 编码器适配层实现
 */
#include "dev_encoder_mt6701.h"

#if defined(USE_DEV_MT6701)

#include <string.h>
#include "assert_report.h"

/*============================================================================
 * 静态实体存储
 *   单例数组，按 id 索引。与 dev_power_monitor 全局单例风格一致。
 *   dev_encoder_t.ctx 指向 &s_mt6701[id]。
 *==========================================================================*/
static dev_mt6701_t s_mt6701[MT6701_ID_MAX];

/*============================================================================
 * MT6701 → 抽象编码器 适配函数
 *   把 dev_mt6701_t 的具体接口适配到 dev_encoder_t 抽象接口。
 *   对外只通过 dev_encoder_t 方法指针暴露，不直接调用。
 *==========================================================================*/

static void encoder_mt6701_update(struct dev_encoder *enc)
{
    dev_mt6701_t *chip = (dev_mt6701_t *)enc->ctx;
    chip->update(chip);
    enc->mechanical_angle = chip->mechanical_angle; /* mt6701 无 getter，直接读字段 */
}

static float encoder_mt6701_get_mechanical_angle(struct dev_encoder *enc)
{
    return enc->mechanical_angle;
}

static void encoder_mt6701_set_offset(struct dev_encoder *enc, float offset_deg)
{
    dev_mt6701_t *chip = (dev_mt6701_t *)enc->ctx;
    chip->offset = offset_deg;
}

static float encoder_mt6701_get_offset(struct dev_encoder *enc)
{
    dev_mt6701_t *chip = (dev_mt6701_t *)enc->ctx;
    return chip->offset;
}

static void encoder_mt6701_set_dir(struct dev_encoder *enc, int8_t dir)
{
    dev_mt6701_t *chip = (dev_mt6701_t *)enc->ctx;
    /* MT6701 枚举已统一为 -1/1，与抽象层约定一致，直接赋值 */
    chip->set_dir(chip, (mt6701_dir_e)dir);
}

static int8_t encoder_mt6701_get_dir(struct dev_encoder *enc)
{
    dev_mt6701_t *chip = (dev_mt6701_t *)enc->ctx;
    return (int8_t)chip->get_dir(chip);
}

static float encoder_mt6701_get_raw_deg(struct dev_encoder *enc)
{
    dev_mt6701_t *chip = (dev_mt6701_t *)enc->ctx;
    chip->update(chip); /* 确保 raw 字段已刷新 */
    return (float)chip->raw / MT6701_ANGLE_RESOLUTION * 360.0F;
}

/*============================================================================
 * 工厂函数：初始化实体 + 装配抽象接口
 *==========================================================================*/

void dev_encoder_mt6701_create(dev_encoder_t *enc, mt6701_id_e id)
{
    assert_report(enc != NULL);
    assert_report(id < MT6701_ID_MAX);

    /* 初始化静态实体 */
    dev_mt6701_init(&s_mt6701[id], id);

    /* 装配抽象接口 */
    enc->ctx = &s_mt6701[id];
    enc->mechanical_angle = 0.0F;
    enc->update = encoder_mt6701_update;
    enc->get_mechanical_angle = encoder_mt6701_get_mechanical_angle;
    enc->set_offset = encoder_mt6701_set_offset;
    enc->get_offset = encoder_mt6701_get_offset;
    enc->set_dir = encoder_mt6701_set_dir;
    enc->get_dir = encoder_mt6701_get_dir;
    enc->get_raw_deg = encoder_mt6701_get_raw_deg;
}

#endif /* USE_DEV_MT6701 */
