/**
 * @file        foc_decoupling.h
 * @brief       FOC 电流环 DQ 解耦模块接口
 * @details     封装 PMSM 电流环三项补偿功能：
 *                1. DQ 交叉解耦（抵消 we*Lq*iq / we*Ld*id 耦合项）
 *                2. 反电势前馈（抵消 we*flux 反电势扰动）
 *                3. 死区补偿（按电流极性叠加标量电压）
 *
 *              采用 ops 表模式支持多种解耦算法切换：
 *                NONE       — 无交叉解耦
 *                FEEDFORWARD— 前馈解耦（用实测电流，精度高但受噪声影响）
 *                FEEDBACK   — 反馈解耦（用参考电流，抗噪声但动态精度低）
 *
 *              三项功能通过 foc_decoupling_config_t 独立使能，互不影响。
 *
 * @author      yangsl
 * @version     1.0
 * @date        2026-07-17
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 *              算法公式与 dev_motor_virtual.c 的 PMSM 物理模型对齐。
 */

#ifndef __FOC_DECOUPLING_H__
#define __FOC_DECOUPLING_H__

#include <stdint.h>
#include <stdbool.h>
#include "motor_param.h"

/**
 * @brief 交叉解耦算法枚举
 */
typedef enum
{
    FOC_DECOUPLE_NONE = 0,       /* 无交叉解耦 */
    FOC_DECOUPLE_FEEDFORWARD,    /* 前馈解耦：用实测电流 */
    FOC_DECOUPLE_FEEDBACK,       /* 反馈解耦：用参考电流 */
} foc_decouple_algo_e;

/**
 * @brief 模块输入：PI 输出 + 实测/参考电流 + 机械转速 + 母线电压
 */
typedef struct
{
    float ud_pi;        /* d轴PI输出 (V) */
    float uq_pi;        /* q轴PI输出 (V) */
    float id;           /* 实测d轴电流 (A) */
    float iq;           /* 实测q轴电流 (A) */
    float id_ref;       /* d轴电流参考 (A) */
    float iq_ref;       /* q轴电流参考 (A) */
    float omega_mech;   /* 机械角速度 (rad/s) */
    float vbus;         /* 母线电压 (V) — 死区补偿自动计算用 */
} foc_decoupling_in_t;

/**
 * @brief 模块输出：补偿后 dq 电压 (V)
 */
typedef struct
{
    float ud;
    float uq;
    /* 诊断用的独立补偿项（V），不参与控制计算。 */
    float ud_cross;
    float uq_cross;
    float uq_bemf;
} foc_decoupling_out_t;

/**
 * @brief 模块配置：算法选择 + 三项独立使能
 */
typedef struct
{
    foc_decouple_algo_e algo;   /* 交叉解耦算法 */
    bool bemf_ff_enable;        /* 反电势前馈使能 */
    bool deadtime_comp_enable;  /* 死区补偿使能 */
} foc_decoupling_config_t;

/**
 * @brief 算法 ops 表：支持多算法切换
 * @note  新增算法只需实现 cross_decouple 并注册到 foc_decouple_get_ops
 */
typedef struct
{
    void (*cross_decouple)(const foc_decoupling_in_t *in,
                           foc_decoupling_out_t *out,
                           const motor_param_t *param);
} foc_decouple_ops_t;

/**
 * @brief 按 algo 获取 ops 表
 * @param algo 解耦算法枚举
 * @return ops 表指针，NONE 或非法值返回 NULL
 */
const foc_decouple_ops_t *foc_decouple_get_ops(foc_decouple_algo_e algo);

/**
 * @brief 模块主入口：依次执行 交叉解耦 → 反电势前馈 → 死区补偿
 * @param in 输入（PI 输出 + 实测/参考电流 + 机械转速）
 * @param out 输出（补偿后 dq 电压）
 * @param config 配置（算法选择 + 三项使能）
 * @param param 电机参数（R/L/flux/pole_pairs 及各增益）
 * @note  当 algo=NONE 且两个 enable 均为 false 时，输出 = 输入（零补偿）
 */
void foc_decoupling_run(const foc_decoupling_in_t *in,
                        foc_decoupling_out_t *out,
                        const foc_decoupling_config_t *config,
                        const motor_param_t *param);

#endif /* __FOC_DECOUPLING_H__ */
