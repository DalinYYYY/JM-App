/**
 * @file        foc_decoupling.c
 * @brief       FOC 电流环 DQ 解耦模块实现
 * @details     实现 PMSM 电流环三项补偿：
 *                1. DQ 交叉解耦（前馈/反馈两种算法，ops 表切换）
 *                2. 反电势前馈（uq += we*flux*k_qff）
 *                3. 死区补偿（按电流极性叠加标量电压）
 *
 *              算法公式依据（与 dev_motor_virtual.c L84-93 物理模型对齐）：
 *                PMSM: did/dt = (ud - Rs*id + we*Lq*iq) / Ld
 *                      diq/dt = (uq - Rs*iq - we*Ld*id - we*flux) / Lq
 *                解耦补偿：ud_decouple = -we*Lq*iq  (抵消 q→d 耦合)
 *                          uq_decouple = +we*Ld*id  (抵消 d→q 耦合)
 *                反电势前馈：uq_bemf = +we*flux     (抵消反电势扰动)
 *
 * @author      yangsl
 * @version     1.0
 * @date        2026-07-17
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 */

#include "foc_decoupling.h"
#include <stddef.h> /* NULL */

/**
 * @brief 前馈解耦：用实测电流
 * @note  精度高（直接抵消实际耦合项），但受电流采样噪声影响
 */
static void decouple_feedforward(const foc_decoupling_in_t *in,
                                 foc_decoupling_out_t *out,
                                 const motor_param_t *param)
{
    float k_decoup = (param)->current_loop.decoupling_gain;
    float k_dff    = (param)->current_loop.d_feedforward_gain;
    float Lq       = (param)->motor_base.lq;
    float Ld       = (param)->motor_base.ld;
    float we       = (float)(param)->motor_base.pole_pairs * in->omega_mech;
    /* PMSM 方程：ud += -we*Lq*iq (q→d 耦合)，uq += +we*Ld*id (d→q 耦合) */
    out->ud += -we * Lq * in->iq * k_decoup * k_dff;
    out->uq +=  we * Ld * in->id * k_decoup;
}

/**
 * @brief 反馈解耦：用参考电流
 * @note  抗噪声（参考电流无采样噪声），但动态精度低（参考与实际电流有相位差）
 */
static void decouple_feedback(const foc_decoupling_in_t *in,
                              foc_decoupling_out_t *out,
                              const motor_param_t *param)
{
    float k_decoup = (param)->current_loop.decoupling_gain;
    float k_dff    = (param)->current_loop.d_feedforward_gain;
    float Lq       = (param)->motor_base.lq;
    float Ld       = (param)->motor_base.ld;
    float we       = (float)(param)->motor_base.pole_pairs * in->omega_mech;
    out->ud += -we * Lq * in->iq_ref * k_decoup * k_dff;
    out->uq +=  we * Ld * in->id_ref * k_decoup;
}

/* ops 表实例：静态常量，运行期不可变 */
static const foc_decouple_ops_t s_ops_feedforward = { decouple_feedforward };
static const foc_decouple_ops_t s_ops_feedback    = { decouple_feedback };

const foc_decouple_ops_t *foc_decouple_get_ops(foc_decouple_algo_e algo)
{
    switch (algo)
    {
    case FOC_DECOUPLE_FEEDFORWARD:
        return &s_ops_feedforward;
    case FOC_DECOUPLE_FEEDBACK:
        return &s_ops_feedback;
    default:
        return NULL;
    }
}

/**
 * @brief 死区补偿：配置值优先 + 自动计算回退 + 带死区线性过渡
 * @details 逆变器死区导致电压输出非线性。补偿电压按电流方向叠加，
 *          电流过零附近用线性平滑过渡，避免 sign() 突变引入电压阶跃扰动。
 *
 *          补偿电压 V_dt 来源（配置值优先）：
 *            deadtime_comp_v > 0 : 使用配置值（标定/调试场景手动设定）
 *            deadtime_comp_v <= 0: 自动计算 = (dead_time_ns×1e-9) × pwm_freq_hz × vbus
 *                                  物理意义：死区时间占 PWM 周期的比例 × 母线电压
 *
 *          过零平滑（带死区线性过渡）：
 *            i_th = rated_current × 0.1  (过零平滑区阈值, 默认额定电流10%)
 *            |i| >= i_th : V_comp = sign(i) × V_dt        (饱和区, 全补偿)
 *            |i| <  i_th : V_comp = (i / i_th) × V_dt     (线性过渡区, 平滑)
 *
 * @param in  输入（含实测电流 id/iq + vbus）
 * @param out 输出（累加补偿电压到 ud/uq）
 * @param param 电机参数（deadtime_comp_v / dead_time_ns / pwm_freq_hz / rated_current）
 */
static void deadtime_compensate(const foc_decoupling_in_t *in,
                                foc_decoupling_out_t *out,
                                const motor_param_t *param)
{
    /* V_dt 来源：配置值优先，否则自动计算 */
    float v_dt = (param)->current_loop.deadtime_comp_v;
    if (v_dt <= 0.0f)
    {
        /* 自动计算：V_dt = t_dt/T_pwm × Vbus */
        float t_dt  = (param)->motor_base.dead_time_ns * 1e-9f;
        float f_pwm = (float)(param)->motor_base.pwm_freq_hz;
        float vbus  = in->vbus;
        if (vbus < 1.0f)
            return; /* Vbus 未就绪, 跳过补偿 */
        v_dt = t_dt * f_pwm * vbus;
    }
    if (v_dt <= 0.0f)
        return;

    /* 过零平滑阈值：额定电流的 10% */
    float i_th = (param)->motor_base.rated_current * 0.1f;
    if (i_th < 1e-3f)
        i_th = 1e-3f; /* 保护：避免除零 */

    /* d 轴死区补偿（带过零平滑）*/
    float id = in->id;
    float ud_comp;
    if (id >= i_th)
        ud_comp = v_dt;
    else if (id <= -i_th)
        ud_comp = -v_dt;
    else
        ud_comp = (id / i_th) * v_dt; /* 线性过渡区 */
    out->ud += ud_comp;

    /* q 轴死区补偿（带过零平滑）*/
    float iq = in->iq;
    float uq_comp;
    if (iq >= i_th)
        uq_comp = v_dt;
    else if (iq <= -i_th)
        uq_comp = -v_dt;
    else
        uq_comp = (iq / i_th) * v_dt;
    out->uq += uq_comp;
}

void foc_decoupling_run(const foc_decoupling_in_t *in,
                        foc_decoupling_out_t *out,
                        const foc_decoupling_config_t *config,
                        const motor_param_t *param)
{
    /* step1: 初始输出 = PI 输出（V 域） */
    out->ud = in->ud_pi;
    out->uq = in->uq_pi;

    /* step2: 交叉解耦（按 algo 选择 ops，NONE 时跳过） */
    const foc_decouple_ops_t *ops = foc_decouple_get_ops(config->algo);
    if (ops != NULL && ops->cross_decouple != NULL)
        ops->cross_decouple(in, out, param);

    /* step3: 反电势前馈补偿 uq += we*flux*k_qff
     * 抵消 PMSM 方程中的 -we*flux 反电势扰动项 */
    if (config->bemf_ff_enable)
    {
        float k_qff = (param)->current_loop.q_feedforward_gain;
        float flux  = (param)->motor_base.flux;
        float we    = (float)(param)->motor_base.pole_pairs * in->omega_mech;
        out->uq += we * flux * k_qff;
    }

    /* step4: 死区补偿（配置值优先 + 自动计算回退 + 过零平滑）*/
    if (config->deadtime_comp_enable)
        deadtime_compensate(in, out, param);
}
