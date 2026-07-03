/**
 * @file        dev_encoder_mt6835.h
 * @brief       MT6835 编码器适配层 — 把 dev_mt6835_t 适配到 dev_encoder_t 抽象接口
 */
#ifndef __DEV_ENCODER_MT6835_H__
#define __DEV_ENCODER_MT6835_H__

#include "dev_config.h"
#if defined(USE_DEV_MT6835)

#include "dev_encoder.h"
#include "dev_mt6835.h"

#ifdef __cplusplus
extern "C" {
#endif

void dev_encoder_mt6835_create(dev_encoder_t *enc, mt6835_id_e id);

#ifdef __cplusplus
}
#endif
#endif /* USE_DEV_MT6835 */
#endif /* __DEV_ENCODER_MT6835_H__ */
