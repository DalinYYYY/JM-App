#ifndef __MOTOR_PROFILE_H__
#define __MOTOR_PROFILE_H__

/* ===================== 电机参数配置文件（唯一真相源）=====================
 * 所有电机电气身份参数集中于此。三个消费方通过 #include 引用：
 *   - User/DataHub/motor_param.c         （控制环运行时默认值）
 *   - User/DataHub/motor_info.c          （协议持久化参数默认值）
 *   - User/MotorCalibration/calib_config.h（标定结果合理性范围）
 *
 * 切换电机型号：修改下面的 MOTOR_PROFILE 宏定义为对应的型号编号。
 * 新增电机型号：在下面追加 #define MOTOR_PROFILE_XXX N，并补一个
 *               #elif 分支填写该型号的全部 MOTOR_* 参数。
 *
 * 说明：本文件只含电机电气身份参数（换电机时变的量）。
 *       板级参数（pwm_freq/enc_lines/dead_time）、减速器、PID 增益
 *       不在此文件，留在各自原文件。
 */

/* ===================== 电机型号选择（修改此行切换）===================== */
#define MOTOR_PROFILE_GM4820H 1
#define MOTOR_PROFILE_DEMO    2 /* 示例占位，演示多型号切换 */
#define MOTOR_PROFILE         MOTOR_PROFILE_GM4820H

/* ===================== 各型号参数 ===================== */
#if MOTOR_PROFILE == MOTOR_PROFILE_GM4820H
/* GM4820H 无刷云台电机（参数来源：GM4820H参数_2024.pdf）
 * 结构 12N14P / WYE / SPMSM（表贴式，Ld≈Lq）
 * 时间常数 τ = L/R = 4.8mH/3.6Ω = 1.33ms */
#define MOTOR_NAME            "GM4820H"
#define MOTOR_R               3.6f      /* 相电阻(Ω) PDF: Ri */
#define MOTOR_LD              4.8e-3f   /* d轴电感(H) PDF: 4.8mH */
#define MOTOR_LQ              4.8e-3f   /* q轴电感(H) SPMSM: Ld≈Lq */
#define MOTOR_FLUX            0.02f     /* 磁链(Wb) KV=66反算: 60/(2π·66·7) */
#define MOTOR_KT              0.21f     /* 转矩常数(Nm/A) = 1.5·pp·flux */
#define MOTOR_KE              0.14f     /* 反电动势常数(V/(rad/s)) = flux·pp */
#define MOTOR_POLE_PAIRS      7         /* 极对数 PDF: 12N14P */
#define MOTOR_RATED_CURRENT   0.6f      /* 额定电流(A) 保守 */
#define MOTOR_PEAK_CURRENT    3.7f      /* 峰值电流(A) PDF: 堵转 */
#define MOTOR_MAX_SPEED       200.0f    /* 最大转速(rad/s) ~1900RPM */
#define MOTOR_RATED_VOLTAGE   24.0f     /* 额定电压(V) PDF: 推荐 */
#define MOTOR_RATED_SPEED_RPM 1550.0f   /* 额定转速(rpm) PDF: 24V */
#define MOTOR_RATED_TORQUE    0.2f      /* 额定转矩(Nm) PDF */
#define MOTOR_PEAK_TORQUE     0.5f      /* 峰值转矩(Nm) */
#define MOTOR_INERTIA         1e-5f     /* 转子惯量(kg·m²) 估算 */

#elif MOTOR_PROFILE == MOTOR_PROFILE_DEMO
/* 示例：演示如何添加第二个电机型号（占位，非真实参数）*/
#define MOTOR_NAME            "DEMO"
#define MOTOR_R               1.0f
#define MOTOR_LD              1e-3f
#define MOTOR_LQ              1e-3f
#define MOTOR_FLUX            0.01f
#define MOTOR_KT              0.1f
#define MOTOR_KE              0.07f
#define MOTOR_POLE_PAIRS      7
#define MOTOR_RATED_CURRENT   1.0f
#define MOTOR_PEAK_CURRENT    5.0f
#define MOTOR_MAX_SPEED       300.0f
#define MOTOR_RATED_VOLTAGE   24.0f
#define MOTOR_RATED_SPEED_RPM 2000.0f
#define MOTOR_RATED_TORQUE    1.0f
#define MOTOR_PEAK_TORQUE     3.0f
#define MOTOR_INERTIA         1e-5f

#else
#error "未知 MOTOR_PROFILE，请在 motor_profile.h 中定义有效的电机型号编号"
#endif

/* ===================== apply 接口（在 motor_profile.c 实现）======================
 * 在 motor_param_init() / motor_info_init() 调用之后紧接着调用，
 * 用 motor_profile.h 的 MOTOR_* 宏覆盖电机电气身份字段。
 *   motor_profile_apply_param(cfg)  —— cfg 实际类型为 motor_param_t*
 *   motor_profile_apply_info(cfg)   —— cfg 实际类型为 motor_info_t*
 * 用 void* 是为了避免 motor_profile.h 循环 include motor_param.h/motor_info.h，
 * 实际类型检查在 motor_profile.c 内部完成。*/
void motor_profile_apply_param(void *cfg);
void motor_profile_apply_info(void *cfg);

#endif /* __MOTOR_PROFILE_H__ */
