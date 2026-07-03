/**
 * @file        dev_encoder_mt6701.h
 * @brief       MT6701 编码器适配层 — 把 dev_mt6701_t 适配到 dev_encoder_t 抽象接口
 *
 * @details     本文件持有 MT6701 的静态实体存储，通过 create() 工厂函数
 *              初始化实体并装配抽象接口的全部方法指针。
 *              调用后即可通过 enc->update(enc) 等抽象接口访问 MT6701。
 *
 *              新增编码器型号时，参照本文件创建 dev_encoder_<new>.h/c。
 */
#ifndef __DEV_ENCODER_MT6701_H__
#define __DEV_ENCODER_MT6701_H__

#include "dev_config.h"
#if defined(USE_DEV_MT6701)

#include "dev_encoder.h"
#include "dev_mt6701.h"  /* mt6701_id_e, dev_mt6701_t */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 创建 MT6701 编码器实例并装配抽象接口
 * @param enc  抽象编码器对象（由调用者提供存储，通常为 &dev_motor_t.encoder）
 * @param id   MT6701 设备编号
 * @details 内部初始化静态 MT6701 实体，绑定到 enc->ctx，
 *          并装配全部 7 个方法指针。
 */
void dev_encoder_mt6701_create(dev_encoder_t *enc, mt6701_id_e id);

#ifdef __cplusplus
}
#endif
#endif /* USE_DEV_MT6701 */
#endif /* __DEV_ENCODER_MT6701_H__ */
