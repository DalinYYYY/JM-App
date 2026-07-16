/**
 * @file        dev_encoder_as5047.h
 * @brief       AS5047P → dev_encoder_t 抽象适配层
 */
#ifndef __DEV_ENCODER_AS5047_H__
#define __DEV_ENCODER_AS5047_H__

#include "dev_config.h"
#if defined(USE_DEV_AS5047)

#include "dev_encoder.h"
#include "dev_as5047.h"

#ifdef __cplusplus
extern "C" {
#endif

void dev_encoder_as5047_create(dev_encoder_t *enc, as5047_id_e id);

#ifdef __cplusplus
}
#endif
#endif /* USE_DEV_AS5047 */
#endif /* __DEV_ENCODER_AS5047_H__ */
