/**
 * @file calib_config_runtime.h
 * @brief 标定参数运行期派生接口（替代 calib_config.h 中编译期派生宏）
 *
 * @details 从 motor_info_storage_get() 读取电机电气身份参数（R/Ld/Lq/flux/pole_pairs/
 *          peak_current/max_speed），运行期派生标定电压/时间/范围等参数。
 *          替代 calib_config.h 中从 MOTOR_* 宏编译期派生的 34 处常量。
 *
 *          未启用 USE_DEV_FLASH 的板：回退到 motor_profile.h 的 MOTOR_* 编译期值，
 *          行为与改造前完全一致（不支持运行期切换电机型号）。
 *
 * @note 线程模型：所有函数可安全在 ISR 上下文调用（仅读取全局 motor_info 单例）。
 *       电机参数在标定 start 时由上位机写入，标定过程中保持不变。
 */
#ifndef __CALIB_CONFIG_RUNTIME_H__
#define __CALIB_CONFIG_RUNTIME_H__

#include <stdint.h>
#include <stdbool.h>

/* ===================== 电机身份快照 =====================
 * 从 motor_info_storage_get()->blocks.motor_calib 读取的电机电气身份参数。
 * 未启用 USE_DEV_FLASH 时从 MOTOR_* 编译期宏构造（V1 等板回退行为）。*/
typedef struct
{
	float r;              /* 相电阻(Ω) */
	float ld;             /* d轴电感(H) */
	float lq;             /* q轴电感(H) */
	float flux;           /* 磁链(Wb) */
	uint32_t pole_pairs;  /* 极对数 */
	float peak_current;   /* 峰值电流(A) */
	float max_speed;      /* 最大转速(rad/s) */
} calib_motor_ident_t;

/* 获取当前电机身份快照（const 指针，全局单例，无需释放）。
 * 启用 USE_DEV_FLASH: 从 motor_info_storage_get() 读取
 * 未启用: 从 MOTOR_* 宏构造的静态实例返回 */
const calib_motor_ident_t *calib_motor_ident_get(void);

/* ===================== 基础派生参数 ===================== */
float calib_tau_s(void);                /* 时间常数 τ = Ld/R (秒) */
bool  calib_is_low_r(void);             /* 低阻电机判断 R < 0.5Ω */
float calib_test_current_a(void);       /* 标准测试电流 = min(peak×0.3, 1.5A) */
float calib_test_current_actual_a(void); /* 实际测试电流(低阻电机提高至 peak×0.5) */

/* ===================== L2 R 辨识参数 ===================== */
float   calib_cfg_l2_r_test_voltage_v(void);      /* 高档电压 V2 = I×R */
float   calib_cfg_l2_r_test_voltage_lo_v(void);   /* 低档电压 V1 = max(V2×0.4, V_dt×2) */
uint32_t calib_cfg_l2_r_test_ticks(void);         /* R 测试时间 tick = τ×750×10kHz */
uint32_t calib_cfg_l2_r_sample_count(void);       /* 每档采样次数 = 500 */

/* ===================== L2 Ld 辨识参数 ===================== */
float   calib_cfg_l2_ld_test_voltage_v(void);     /* Ld 阶跃电压 = V_R×1.3 */
uint32_t calib_cfg_l2_ld_test_ticks(void);        /* Ld 暂态窗口 = τ×3×10kHz */
uint32_t calib_cfg_l2_ld_sample_count(void);      /* Ld 采样点数 = 30 */
uint32_t calib_cfg_l2_ld_skip_ticks(void);        /* Ld 跳拍 = 2 */

/* ===================== L2 Lq 辨识参数 ===================== */
float   calib_cfg_l2_lq_test_voltage_v(void);     /* Lq 阶跃电压 = V_R×1.3 */
uint32_t calib_cfg_l2_lq_test_ticks(void);        /* Lq 暂态窗口 = τ×3×10kHz */
uint32_t calib_cfg_l2_lq_sample_count(void);      /* Lq 采样点数 = 30 */
uint32_t calib_cfg_l2_lq_skip_ticks(void);        /* Lq 跳拍 = 2 */

/* ===================== L2 flux 辨识参数 ===================== */
float   calib_cfg_l2_flux_target_speed_clamped_rad_s(void); /* 目标转速(限幅) */
float   calib_cfg_l2_flux_spin_voltage_v(void);             /* 稳速旋转电压 */
uint32_t calib_cfg_l2_flux_spin_ticks(void);                /* 稳速时间 = 2s×10kHz */
uint32_t calib_cfg_l2_flux_sample_count(void);              /* 稳态采样 = 200 */

/* ===================== 交流注入法参数 ===================== */
float   calib_cfg_ac_inject_freq_hz(void);        /* 注入频率 = 0.3/(2π×τ) */
float   calib_cfg_ac_inject_amp_v(void);          /* 交流幅值 = I×R×0.5 */
float   calib_cfg_ac_inject_dc_v(void);           /* 直流偏置 = I×R */
uint32_t calib_cfg_ac_inject_warmup_ticks(void);  /* 预热 = τ×10×10kHz */
uint32_t calib_cfg_ac_inject_total_ticks(void);   /* 采样总 tick = 5/freq×10kHz */
uint32_t calib_cfg_ac_inject_sample_count(void);  /* 采样次数 = 200 */

/* ===================== L2 相序识别参数 ===================== */
float   calib_cfg_l2_phase_seq_voltage_v(void);            /* 对齐电压 = I×R */
uint32_t calib_cfg_l2_phase_seq_align_ticks(void);         /* 对齐等待 = 1s×10kHz */
uint32_t calib_cfg_l2_phase_seq_step_ticks(void);          /* 步进等待 = 0.5s×10kHz */

/* ===================== L2 极对数辨识参数 ===================== */
float   calib_cfg_l2_pole_pairs_voltage_v(void);           /* 锁定电压 = I×R */
uint32_t calib_cfg_l2_pole_pairs_align_ticks(void);        /* 对齐等待 = 1s×10kHz */
float   calib_cfg_l2_pole_pairs_dtheta_rad(void);          /* 每 tick 电角度增量 */
float   calib_cfg_l2_pole_pairs_target_rad(void);          /* 扫描目标 = 8×2π */

/* ===================== L3 编码器校准参数 ===================== */
float   calib_cfg_l3_align_voltage_v(void);       /* 对齐电压 = I×R */
uint32_t calib_cfg_l3_align_ticks(void);          /* 对齐时间 = 2s×10kHz */
uint32_t calib_cfg_l3_sample_count(void);         /* 零位采样次数 = 100 */
float   calib_cfg_l3_dir_voltage_v(void);         /* 方向电压 = I×R×0.7 */
uint32_t calib_cfg_l3_dir_ticks(void);            /* 方向测试时间 = 1s×10kHz */

/* ===================== 标定结果合理性范围 ===================== */
float calib_cfg_r_min_ohm(void);     /* R 下限 = max(R×0.2, 0.01) */
float calib_cfg_r_max_ohm(void);     /* R 上限 = R×5 */
float calib_cfg_ld_min_h(void);      /* Ld 下限 = Ld×0.2 */
float calib_cfg_ld_max_h(void);      /* Ld 上限 = Ld×3 */
float calib_cfg_lq_min_h(void);      /* Lq 下限 = Lq×0.2 */
float calib_cfg_lq_max_h(void);      /* Lq 上限 = Lq×3 */
float calib_cfg_flux_min_wb(void);   /* flux 下限 = flux×0.2 */
float calib_cfg_flux_max_wb(void);   /* flux 上限 = flux×3 */

#endif /* __CALIB_CONFIG_RUNTIME_H__ */
