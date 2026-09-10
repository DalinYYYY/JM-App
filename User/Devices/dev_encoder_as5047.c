/**
 * @file        dev_encoder_as5047.c
 * @brief       AS5047P 编码器适配层实现
 */
#include "dev_encoder_as5047.h"

#if defined(USE_DEV_AS5047)

#include <string.h>
#include "assert_report.h"

static dev_as5047_t s_as5047[AS5047_ID_MAX];

static void encoder_as5047_update(struct dev_encoder *enc)
{
	dev_as5047_t *chip = (dev_as5047_t *)enc->ctx;
	chip->update(chip);
	enc->mechanical_angle = chip->mechanical_angle;
}

static float encoder_as5047_get_mechanical_angle(struct dev_encoder *enc)
{
	return enc->mechanical_angle;
}

static void encoder_as5047_set_offset(struct dev_encoder *enc, float offset_deg)
{
	dev_as5047_t *chip = (dev_as5047_t *)enc->ctx;
	chip->offset = offset_deg;
}

static float encoder_as5047_get_offset(struct dev_encoder *enc)
{
	dev_as5047_t *chip = (dev_as5047_t *)enc->ctx;
	return chip->offset;
}

static void encoder_as5047_set_dir(struct dev_encoder *enc, int8_t dir)
{
	dev_as5047_t *chip = (dev_as5047_t *)enc->ctx;
	chip->set_dir(chip, (as5047_dir_e)dir);
}

static int8_t encoder_as5047_get_dir(struct dev_encoder *enc)
{
	dev_as5047_t *chip = (dev_as5047_t *)enc->ctx;
	return (int8_t)chip->get_dir(chip);
}

static float encoder_as5047_get_raw_deg(struct dev_encoder *enc)
{
	dev_as5047_t *chip = (dev_as5047_t *)enc->ctx;
	chip->update(chip);
	return (float)chip->raw * 360.0F / (float)AS5047_ANGLE_RESOLUTION;
}

static uint16_t encoder_as5047_get_err_cnt(struct dev_encoder *enc)
{
	dev_as5047_t *chip = (dev_as5047_t *)enc->ctx;
	return (uint16_t)(chip->err_cnt + chip->spi_err_cnt);
}

static uint8_t encoder_as5047_get_health(struct dev_encoder *enc)
{
	(void)enc; /* 磁编码器专属位图, 光学编码器无磁场状态, 恒健康 */
	return 0u;
}

void dev_encoder_as5047_create(dev_encoder_t *enc, as5047_id_e id)
{
	assert_report(enc != NULL);
	assert_report(id < AS5047_ID_MAX);

	dev_as5047_init(&s_as5047[id], id);

	enc->ctx = &s_as5047[id];
	enc->mechanical_angle = 0.0F;
	enc->update = encoder_as5047_update;
	enc->get_mechanical_angle = encoder_as5047_get_mechanical_angle;
	enc->set_offset = encoder_as5047_set_offset;
	enc->get_offset = encoder_as5047_get_offset;
	enc->set_dir = encoder_as5047_set_dir;
	enc->get_dir = encoder_as5047_get_dir;
	enc->get_raw_deg = encoder_as5047_get_raw_deg;
	enc->get_err_cnt = encoder_as5047_get_err_cnt;
	enc->get_health = encoder_as5047_get_health;
}

#endif /* USE_DEV_AS5047 */
