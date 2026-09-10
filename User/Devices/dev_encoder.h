/**
 * @file        dev_encoder.h
 * @brief       抽象编码器接口（与具体芯片型号无关）
 *
 * @details     控制层只面向本接口，不感知背后是 MT6701 / MT6835 / AS5047 等。
 *              适配层（dev_encoder_xxx.c）负责把具体芯片绑定到 ctx，
 *              并装配全部方法指针。
 *
 *              调用约定：先 update(self) 刷新，再读 self->mechanical_angle，
 *              或调 get_mechanical_angle(self)。
 *
 * @par 标定扩展接口（set_offset / set_dir / get_raw_deg）
 *              供 calib_hw / calib_level3 等标定模块使用，统一通过抽象层访问编码器。
 *              方向统一为 -1/1 约定（1=CW 正向, -1=CCW 反向），角度统一为 deg 单位。
 *              虚拟模式下这些方法可为 NULL（标定不在虚拟模式运行）。
 *
 * @note 本结构被 dev_motor.h 和 dev_motor_virtual.h 共同 include，
 *       保证真实/虚拟两种编译路径下布局一致。
 */
#ifndef __DEV_ENCODER_H__
#define __DEV_ENCODER_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 抽象编码器接口
 */
typedef struct dev_encoder
{
    void *ctx;                                                       /* 指向具体编码器对象（适配层静态实体）*/
    float mechanical_angle;                                          /* 最新机械角度(deg)，update 后刷新 */
    void (*update)(struct dev_encoder *pobj);                        /* 刷新角度 */
    float (*get_mechanical_angle)(struct dev_encoder *pobj);         /* 读取机械角度(deg) */
    /* 标定扩展接口：统一 -1/1 方向约定，统一 deg 单位 */
    void (*set_offset)(struct dev_encoder *pobj, float offset_deg);  /* 设置零点偏移(deg) */
    float (*get_offset)(struct dev_encoder *pobj);                  /* 读取零点偏移(deg) */
    void (*set_dir)(struct dev_encoder *pobj, int8_t dir);           /* 设置方向: 1=CW, -1=CCW */
    int8_t (*get_dir)(struct dev_encoder *pobj);                    /* 读取方向: 1/-1 */
    float (*get_raw_deg)(struct dev_encoder *pobj);                 /* 原始角度(deg)，未补偿 */
    /* 健康查询接口(供故障检测; 无对应能力的芯片/虚拟模式为 NULL, 调用方判空) */
    uint16_t (*get_err_cnt)(struct dev_encoder *pobj);              /* 连续坏帧计数(有效帧清零) */
    uint8_t (*get_health)(struct dev_encoder *pobj);                /* 健康位图: bit0=位置无效 bit1=磁场过弱 bit2=磁场过强 */
} dev_encoder_t;

#ifdef __cplusplus
}
#endif

#endif /* __DEV_ENCODER_H__ */
