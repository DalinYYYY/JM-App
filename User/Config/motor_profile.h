#ifndef __MOTOR_PROFILE_H__
#define __MOTOR_PROFILE_H__

#include "motor_param.h" /* motor_param_t：sync_to_param 参数类型 */
#include "motor_info.h"  /* motor_info_t：sync_to_param 参数类型 */

/* ===================== 电机参数配置文件（首次上电 fallback 默认值）=====================
 * 本文件提供电机电气身份参数的编译期默认值，仅在以下场景使用：
 *   1. 首次上电（Flash 无有效数据）：apply_info_default() 用 MOTOR_* 宏初始化 motor_info
 *   2. 未启用 USE_DEV_FLASH 的板（V1 等）：calib_config_runtime.c 回退到 MOTOR_* 编译期值
 *   3. motor_param_init 后的 apply_param()：写入运行期 motor_param_t 默认值
 *
 * 切换电机型号（启用 Flash 的板）：
 *   上位机通过 0xE7 命令批量写入 motor_info.blocks.motor_calib 字段，0xEA 固化到 Flash。
 *   标定算法通过 calib_config_runtime.h 读取 motor_info 派生标定参数，无需重编译固件。
 *   修改本文件的 MOTOR_PROFILE 宏仅影响首次上电的 fallback 默认值。
 *
 * 三个消费方：
 *   - User/DataHub/motor_param.c           （控制环运行时默认值）
 *   - User/DataHub/motor_info.c            （协议持久化参数默认值）
 *   - User/MotorCalibration/calib_config_runtime.c（运行期派生标定参数的 fallback）
 *
 * 说明：本文件只含电机电气身份参数（换电机时变的量）。
 *       板级参数（pwm_freq/enc_lines/dead_time）、减速器、PID 增益
 *       不在此文件，留在各自原文件。
 */

/* ===================== 电机型号选择（修改此行切换）===================== */
#define MOTOR_PROFILE_GM4820H  1
#define MOTOR_PROFILE_5010_360 2 /* 5010 360KV 云台电机 */
#define MOTOR_PROFILE_DEMO     3 /* 示例占位，演示多型号切换 */
#define MOTOR_PROFILE          MOTOR_PROFILE_GM4820H

/* ===================== 各型号参数 ===================== */
#if MOTOR_PROFILE == MOTOR_PROFILE_GM4820H
/* GM4820H 无刷云台电机（参数来源：GM4820H参数_2024.pdf）
 * 结构 12N14P / WYE / SPMSM（表贴式，Ld≈Lq）
 * 时间常数 τ = L/R = 4.8mH/3.6Ω = 1.33ms */
#define MOTOR_NAME            "GM4820H"
#define MOTOR_R               3.6f    /* 相电阻(Ω) PDF: Ri */
#define MOTOR_LD              4.8e-3f /* d轴电感(H) PDF: 4.8mH */
#define MOTOR_LQ              4.8e-3f /* q轴电感(H) SPMSM: Ld≈Lq */
#define MOTOR_FLUX            0.02f   /* 磁链(Wb) KV=66反算: 60/(2π·66·7) */
#define MOTOR_KT              0.21f   /* 转矩常数(Nm/A) = 1.5·pp·flux */
#define MOTOR_KE              0.14f   /* 反电动势常数(V/(rad/s)) = flux·pp */
#define MOTOR_POLE_PAIRS      7       /* 极对数 PDF: 12N14P */
#define MOTOR_RATED_CURRENT   0.6f    /* 额定电流(A) 保守 */
#define MOTOR_PEAK_CURRENT    3.7f    /* 峰值电流(A) PDF: 堵转 */
#define MOTOR_MAX_SPEED       200.0f  /* 最大转速(rad/s) ~1900RPM */
#define MOTOR_RATED_VOLTAGE   24.0f   /* 额定电压(V) PDF: 推荐 */
#define MOTOR_RATED_SPEED_RPM 1550.0f /* 额定转速(rpm) PDF: 24V */
#define MOTOR_RATED_TORQUE    0.2f    /* 额定转矩(Nm) PDF */
#define MOTOR_PEAK_TORQUE     0.5f    /* 峰值转矩(Nm) */
#define MOTOR_INERTIA         1e-5f   /* 转子惯量(kg·m²) 估算 */

#elif MOTOR_PROFILE == MOTOR_PROFILE_5010_360
/* MKS 5010 360KV 无刷云台电机（参数来源：厂家规格书）
 * 结构 12N14P / WYE / SPMSM（表贴式，Ld≈Lq）
 * 厂家参数: R=120mΩ, L=50μH, Imax=20A, Pmax=300W, 极对数=7, DC12~24V
 * 派生计算:
 *   磁链 flux = 60/(2π·KV·pp) = 60/(2π·360·7) ≈ 3.79e-5 Wb
 *   转矩常数 KT = 1.5·pp·flux ≈ 0.398e-3 Nm/A（注: KV法反算磁链对低KV大电机偏小，
 *                实际应以堵转转矩实测为准；此值仅用于首次上电）
 *   反电动势常数 KE = flux·pp ≈ 2.65e-4 V/(rad/s)
 *   时间常数 τ = L/R = 50μH/0.12Ω ≈ 0.42ms */
#define MOTOR_NAME            "MKS5010_360KV"
#define MOTOR_R               0.12f     /* 相电阻(Ω) 厂家: 120mΩ */
#define MOTOR_LD              50e-6f    /* d轴电感(H) 厂家: 50μH */
#define MOTOR_LQ              50e-6f    /* q轴电感(H) SPMSM: Ld≈Lq */
#define MOTOR_FLUX            3.79e-5f  /* 磁链(Wb) KV=360反算: 60/(2π·360·7) */
#define MOTOR_KT              0.398e-3f /* 转矩常数(Nm/A) = 1.5·pp·flux */
#define MOTOR_KE              2.65e-4f  /* 反电动势常数(V/(rad/s)) = flux·pp */
#define MOTOR_POLE_PAIRS      7         /* 极对数 厂家: 7 (12N14P) */
#define MOTOR_RATED_CURRENT   2.0f      /* 额定电流(A) 保守取 Imax/2 */
#define MOTOR_PEAK_CURRENT    6.0f      /* 峰值电流(A) 厂家: 20A */
#define MOTOR_MAX_SPEED       150.0f    /* 最大转速(rad/s) ~1432RPM(24V/360KV) */
#define MOTOR_RATED_VOLTAGE   12.0f     /* 额定电压(V) 厂家: DC12~24V, 取上限 */
#define MOTOR_RATED_SPEED_RPM 1432.0f   /* 额定转速(rpm) ≈ 24V×360KV */
#define MOTOR_RATED_TORQUE    0.08f     /* 额定转矩(Nm) 估算: KT×Irated ≈ 4mNm(偏小,待实测) */
#define MOTOR_PEAK_TORQUE     0.2f      /* 峰值转矩(Nm) 估算 */
#define MOTOR_INERTIA         5e-6f     /* 转子惯量(kg·m²) 估算: 5010尺寸 */

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
 * 在 motor_param_init() / motor_info_init() 调用之后紧接着调用。
 * **逐字段零值 fallback 语义**：对每个电机电气身份字段独立判断，
 * 若该字段为零值（未设置/未标定），则用 motor_profile.h 的 MOTOR_* 宏覆盖；
 * 非零（已标定）则保留 motor_info 中的实际标定值，不被默认值覆盖。
 *   motor_profile_apply_param(cfg)  —— cfg 实际类型为 motor_param_t*
 *   motor_profile_apply_info(cfg)   —— cfg 实际类型为 motor_info_t*
 * 用 void* 是为了避免 motor_profile.h 循环 include motor_param.h/motor_info.h，
 * 实际类型检查在 motor_profile.c 内部完成。*/
void motor_profile_apply_param(void *cfg);
void motor_profile_apply_info(void *cfg);

/* profile 配置版本号：仅当 motor_info_t 结构体字段增删时递增此版本号，
 * 用于上电时检测 Flash 中存储的参数布局是否对应当前固件。
 * 版本不匹配时触发重新初始化（init + apply_default + save）。
 * 切换电机型号不再递增此版本号（型号切换通过 0xE7 写入 motor_info 实现）。
 *
 * 版本历史：
 *   v1: 初始版本
 *   v2: 增加 pid_flash_valid_magic 字段
 *   v3: MotorCalibParam 段增加 peak_current/max_speed 字段（Index 43/44）*/
#define MOTOR_PROFILE_CONFIG_VERSION 3U

void motor_profile_apply_info_default(void *cfg); /* 无条件覆盖：首次上电用 */

/* ===================== Flash → 运行期参数同步 =====================
 * 把 motor_info_storage_get() 返回的 Flash 加载数据同步到控制环实际使用的
 * motor_param_t。补上 motor_info_storage_init 之后断裂的桥接链路。
 *
 * 同步策略：
 *   - is_calibrated == 1：电气参数(R/Ld/Lq/flux/kt/inertia/pole_pairs)用Flash标定值
 *                        覆盖 motor_profile_apply_param 已写入的 profile 默认值
 *   - is_calibrated == 0：电气参数保留 profile 默认值（apply_param 已写入）
 *   - 编码器参数(enc_direction/enc_offset/elec_angle_bias)：始终同步
 *     （apply_info 已对 enc_direction 做零值 fallback=1；
 *      enc_offset/elec_angle_bias 保留 0 表示需标定）
 *
 * 调用时机：motor_loop_init 中 motor_profile_apply_param(param) 之后立即调用，
 *           须保证 motor_info_storage_init() 已先执行（在 user_interface.c 中）。
 *
 * @param param  运行期参数（&usr.motor_param[M1]）
 * @param info   Flash 加载的 motor_info 句柄（motor_info_storage_get() 返回值）
 */
void motor_profile_sync_to_param(motor_param_t *param, const motor_info_t *info);

#endif /* __MOTOR_PROFILE_H__ */
