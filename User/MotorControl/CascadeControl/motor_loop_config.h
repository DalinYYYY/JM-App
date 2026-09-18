/**
 * @file        motor_loop_config.h
 * @brief       电机三环控制全局配置
 * @details     电流环运行在中断基频（由触发电流环中断的硬件定时器/ADC 决定，
 *              本层不配置）。速度环、位置环通过分频在同一中断内执行——此处仅
 *              配置两者相对电流环的分频系数。
 *
 *              分频判断在中断中执行，为兼顾执行效率，采用自增计数器与阈值比较，
 *              避免在中断里做取模/除法运算。
 *
 *              示例（电流环 10kHz 时）：
 *                VEL_DIV = 5  → 速度环 2kHz
 *                POS_DIV = 10 → 位置环 1kHz
 *
 * @author      yangsl
 * @version     1.0
 * @date        2026-06-11
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 */

#ifndef __MOTOR_LOOP_CONFIG_H__
#define __MOTOR_LOOP_CONFIG_H__

/*============================================================================
 * 设备层来源开关（虚拟电机 / 真实驱动）
 *   - 0：使用虚拟电机（dev_motor_virtual）。一个完整的 dq PMSM 物理模型伪装成
 *        dev_motor_t，让 FOC 三环在无硬件时即可闭环运行、调参。编码器角度、
 *        三相电流采样均由模型实时生成，Clarke/Park/SVPWM/PI 全链路真实参与。
 *   - 1：使用真实 dev_motor 驱动（编码器/三相采样/半桥PWM 等硬件）。
 *
 *        两种模式下三环控制层（current_loop / motor_loop）代码完全一致，
 *        API（dev_motor_t / dev_motor_init）签名不变。dev_motor_select.h 按本宏
 *        二选一包含 dev_motor.h 或 dev_motor_virtual.h；dev_motor_virtual.c 实现体
 *        由本宏控制（=1 时编译为空），与真实 dev_motor.c 零符号冲突。
 *
 *        硬件就绪后将本宏置 1 即切换到真实电机，无需改动三环控制层。
 *==========================================================================*/
#ifndef MOTOR_LOOP_ENABLE_DEV_DRIVER
#define MOTOR_LOOP_ENABLE_DEV_DRIVER 1u
#endif

/*============================================================================
 * 分频系数配置（用户可修改）
 *   - 电流环频率由中断决定，不在此配置
 *   - 速度/位置环 = 电流环频率 / 对应分频系数
 *   - 约束：POS_DIV 必须为 VEL_DIV 的整数倍，保证两环节拍对齐
 *==========================================================================*/

/** @brief 速度环分频：每 N 个电流环中断执行一次速度环 */
#ifndef MOTOR_LOOP_VEL_DIV
#define MOTOR_LOOP_VEL_DIV 5u
#endif

/** @brief 位置环分频：每 N 个电流环中断执行一次位置环 */
#ifndef MOTOR_LOOP_POS_DIV
#define MOTOR_LOOP_POS_DIV 10u
#endif

#if (MOTOR_LOOP_POS_DIV % MOTOR_LOOP_VEL_DIV) != 0
#error "MOTOR_LOOP_POS_DIV 必须为 MOTOR_LOOP_VEL_DIV 的整数倍"
#endif

/*============================================================================
 * 超周期错峰调度（削峰: 最小化单次最长中断执行时间）
 *   motor_loop_isr 内 sched_cnt 于 0..POS_DIV-1 循环(超周期=POS_DIV 拍,
 *   10kHz 时 1ms), 各特殊活动在指定拍执行, 特殊增量互不叠加:
 *     速度环      : 拍 0 与 VEL_DIV(=5)  → 2kHz, 节拍与原始实现一致
 *     位置环      : 拍 POS_BEAT(=4)      → 1kHz, 与速度拍错开
 *     故障全量检测: 拍 FAULT_BEAT(=2)    → 1kHz, 须 FAULT_DET_FAST_DIV==POS_DIV
 *     遥测同步    : 拍 SYNC_BEAT(=7)     → 1kHz, sync_state 从每拍降频
 *   约束: ①PLL 速度解算严格只发生在速度拍(0/VEL_DIV, 间隔均匀 500µs,
 *   vel_calc_pll 按固定 dt 递推); 位置拍仅做多圈累计, fb->vel 用上一速度拍
 *   结果(≤500µs 旧)。②位置拍分离曾在坏参数(kp_s 超标10倍)下观测到失稳,
 *   参数修正后重新启用本调度——测试前务必确认速度环参数已修复且平稳。
 *   反馈解算随所属环同拍执行; NaN 发散检测/状态机框架每拍。
 *   仅自增与比较, 无取模/除法。
 *==========================================================================*/
#ifndef MOTOR_SCHED_FAULT_BEAT
#define MOTOR_SCHED_FAULT_BEAT 2u
#endif

#ifndef MOTOR_SCHED_POS_BEAT
#define MOTOR_SCHED_POS_BEAT 4u
#endif

#ifndef MOTOR_SCHED_SYNC_BEAT
#define MOTOR_SCHED_SYNC_BEAT 7u
#endif

/* 相位合法性编译期校验: 不落速度拍(0/VEL_DIV)、不越界、互不相同 */
#if (MOTOR_SCHED_POS_BEAT >= MOTOR_LOOP_POS_DIV) || \
	(MOTOR_SCHED_POS_BEAT == 0u) || (MOTOR_SCHED_POS_BEAT == MOTOR_LOOP_VEL_DIV)
#error "MOTOR_SCHED_POS_BEAT 须位于速度拍之间(0<拍号<POS_DIV 且非速度拍)"
#endif
#if (MOTOR_SCHED_FAULT_BEAT >= MOTOR_LOOP_POS_DIV) || \
	(MOTOR_SCHED_FAULT_BEAT == 0u) || (MOTOR_SCHED_FAULT_BEAT == MOTOR_LOOP_VEL_DIV) || \
	(MOTOR_SCHED_FAULT_BEAT == MOTOR_SCHED_POS_BEAT)
#error "MOTOR_SCHED_FAULT_BEAT 须位于速度拍之间且与其他特殊拍错开"
#endif
#if (MOTOR_SCHED_SYNC_BEAT >= MOTOR_LOOP_POS_DIV) || \
	(MOTOR_SCHED_SYNC_BEAT == 0u) || (MOTOR_SCHED_SYNC_BEAT == MOTOR_LOOP_VEL_DIV) || \
	(MOTOR_SCHED_SYNC_BEAT == MOTOR_SCHED_POS_BEAT) || \
	(MOTOR_SCHED_SYNC_BEAT == MOTOR_SCHED_FAULT_BEAT)
#error "MOTOR_SCHED_SYNC_BEAT 须位于速度拍之间且与其他特殊拍错开"
#endif

/*============================================================================
 * 速度环惯量加速度前馈开关（iq_ff += accel_ff_gain * ref->accel）
 *   - 0：关闭（默认）。实测空载速度闭环受前馈扰动失稳，待排查
 *        （疑点：Flash aff 字段无有效性校验，旧数据/自整定值可直接生效；
 *         过渡期 blend_vel_accel 生成的解析加速度量级偏大）。
 *        关闭后 Flash 中 aff 参数照常读写，仅不参与前馈计算。
 *   - 1：启用。前馈增益来源 motor_param position_loop.accel_ff_gain
 *        （Flash ctl->aff / L6.4 自整定写入）。
 *==========================================================================*/
#ifndef MOTOR_LOOP_VEL_ACCEL_FF_ENABLE
#define MOTOR_LOOP_VEL_ACCEL_FF_ENABLE 0u
#endif

/*============================================================================
 * 位置误差死区（零速 stick-slip 抑制）
 *   - 进入死区: |pos_err| < DEADBAND 时位置环输出置 0, 速度环以 0 为目标
 *     主动刹停, 并按时间常数 VEL_INT_LEAK_TAU 泄漏速度环积分, 消除积分蓄能
 *     松闸。
 *   - 迟滞退出: 进带后需 |pos_err| > HYS(须大于 DEADBAND)才重新出力。
 *     无迟滞时噪声/齿槽使误差在边界来回穿越, 每次出界触发一次全增益
 *     位置环打击, 形成边界极限环（周期性"嗒"声）。
 *   - 摩擦前馈(friction_comp 模块)从源头补偿摩擦拖尾, 本死区切断
 *     极限环能量来源, 两者配合使用。
 *   - 注意: 开关必须用整型宏（#if 中浮点常量被截断为整数, 0.005f 会变 0）。
 *     VEL_INT_LEAK_TAU <= 0 时死区内不泄漏积分。
 *==========================================================================*/
#ifndef MOTOR_LOOP_POS_DEADBAND_EN
#define MOTOR_LOOP_POS_DEADBAND_EN 1u
#endif

#ifndef MOTOR_LOOP_POS_DEADBAND_RAD
#define MOTOR_LOOP_POS_DEADBAND_RAD 0.005f
#endif

#ifndef MOTOR_LOOP_POS_DEADBAND_HYS_RAD
#define MOTOR_LOOP_POS_DEADBAND_HYS_RAD 0.020f
#endif

#ifndef MOTOR_LOOP_VEL_INT_LEAK_TAU
#define MOTOR_LOOP_VEL_INT_LEAK_TAU 0.1f
#endif

/*============================================================================
 * 电角度超前补偿（采样→PWM 生效管线延迟补偿）
 *   电流采样到新 PWM 电压生效存在约 1~1.5 个 PWM 周期延迟（ISR 计算 +
 *   CCR 预装载时机），期间转子继续转过 we×Td 电角度。未补偿时高速下
 *   电压矢量错相（816Hz 电频率@3500rpm 时滞后 30~44°），uq 泄漏进 d 轴
 *   激励 Id 振荡发散。
 *   补偿在 motor_loop_ele_radian_cb 内实施：theta += we × 系数 × T_pwm。
 *   静止/低速 we≈0 补偿量≈0，对标定路径无影响。
 *   精调方法：固定转速看 ud_PI——补偿不足 ud_PI 偏负，过补偏正，居中即准。
 *==========================================================================*/
#ifndef FOC_ELE_ANGLE_LEAD_CYCLES
#define FOC_ELE_ANGLE_LEAD_CYCLES (1.5f)
#endif

#endif /* __MOTOR_LOOP_CONFIG_H__ */
