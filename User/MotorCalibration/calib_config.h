#ifndef __CALIB_CONFIG_H__
#define __CALIB_CONFIG_H__

/* ===================== 标定集中配置（与电机型号无关的常量）=====================
 * 所有与电机型号无关的标定可调参数集中于此。
 * 时间相关 TICK 数约定：控制环频率 10kHz（dt=100us）。
 *
 * 与电机型号相关的派生参数（基于 R/Ld/Lq/flux/pole_pairs/peak_current/max_speed）
 * 已迁移至 calib_config_runtime.h/c，运行期从 motor_info 读取派生。
 * 切换电机型号无需重编译固件，通过上位机 0xE7/0xEA 命令修改 motor_info 即可。
 */

#define CALIB_TICKS_PER_SEC 10000.0f

/* 电机分档识别阈值（与电机型号无关的通用阈值）*/
#define CALIB_LOW_R_THRESHOLD 0.5f

/* 测试电流安全上限（电源限流保护，与电机型号无关）*/
#define CALIB_CFG_MAX_TEST_CURRENT_A 1.5f

/* 标定施加电压幅值上限(√(ud²+uq²))，与电机型号无关 */
#define CALIB_CFG_MAX_VOLTAGE_MAG_V 7.0f

/* 全局超时(s)，与电机型号无关。
 * 注: L5.1 齿槽扫描(正反各~3.2圈)全程约 40s, 兜底上限放宽至 120s;
 *     其余标定级均在 30s 内完成, 卡死场景仅延迟报错不影响功能 */
#define CALIB_CFG_GLOBAL_TIMEOUT_S     120.0f
#define CALIB_CFG_GLOBAL_TIMEOUT_TICKS (uint32_t)(CALIB_CFG_GLOBAL_TIMEOUT_S * CALIB_TICKS_PER_SEC)

/* ===================== L1 驱动硬件底层参数（与电机无关）===================== */
#define CALIB_CFG_L1_ADC_OFFSET_SAMPLES 1000
#define CALIB_CFG_L1_VBUS_SAMPLE_COUNT  500

/* ===================== L2 采样次数（与电机无关）===================== */
#define CALIB_CFG_L2_R_SAMPLE_COUNT    500
#define CALIB_CFG_L2_LD_SAMPLE_COUNT   30
#define CALIB_CFG_L2_LD_SKIP_TICKS     2u
#define CALIB_CFG_L2_LQ_SAMPLE_COUNT   30
#define CALIB_CFG_L2_LQ_SKIP_TICKS     2u
#define CALIB_CFG_L2_FLUX_SAMPLE_COUNT 200

/* ===================== L2 死区压降估计（与电机无关的保守值）===================== */
#define CALIB_CFG_L2_R_V_DT_ESTIMATE_V 0.5f

/* ===================== L2 flux 稳速判断（与电机无关）===================== */
#define CALIB_CFG_L2_FLUX_SPEED_STABLE_WINDOW 50u
#define CALIB_CFG_L2_FLUX_SPEED_STABLE_RATIO  0.05f
#define CALIB_CFG_L2_FLUX_OMEGA_E_MIN_RAD_S   5.0f
#define CALIB_CFG_L2_FLUX_SPIN_TIME_S         2.0f

/* ===================== 交流注入采样次数（与电机无关）===================== */
/* 平均周期数: 噪声按 √N 压缩。5 周期(旧值)对 Ld 小信号方差抑制不足,
 * 提升到 20 周期(约 0.13s @153Hz), 方差再降 2 倍 */
#define CALIB_CFG_AC_INJECT_CYCLES       20u
#define CALIB_CFG_AC_INJECT_SAMPLE_COUNT 200

/* ===================== L2 相序/极对数时间（与电机无关）===================== */
#define CALIB_CFG_L2_PHASE_SEQ_ALIGN_S      1.0f
#define CALIB_CFG_L2_PHASE_SEQ_SCAN_ELE_CYCLES 2u /* 相序开环扫描电周期数 */
#define CALIB_CFG_L2_POLE_PAIRS_ALIGN_S     1.0f
#define CALIB_CFG_L2_POLE_PAIRS_ELE_CYCLES  21u
#define CALIB_CFG_L2_POLE_PAIRS_ELE_FREQ_HZ 2.0f

/* ===================== L5.1 齿槽转矩扫描（与电机无关）===================== */
/* 原理: 强制电角度匀速递增开环同步拖动(calib_hw_enter 会话), 恒 uq 电压下
 * iq(θm) 即齿槽+摩擦力矩曲线; 正反双程 bin 累加取平均自动抵消库仑摩擦。 */
#define CALIB_CFG_L5_COG_SPEED_RAD_S  1.0f    /* 拖动机械角速度(恒速) */
#define CALIB_CFG_L5_COG_HOLD_CURRENT 0.15f   /* 拖动保持电流估计(A, 克服摩擦+齿槽峰值) */
#define CALIB_CFG_L5_COG_VOLTAGE_MUL  2.0f    /* 拖动电压裕度系数(uq=R·I·本系数+ωe·ψ) */
#define CALIB_CFG_L5_COG_ROUND_SCAN   2.0f    /* 每方向采集圈数 */
#define CALIB_CFG_L5_COG_ROUND_SKIP   1.2f    /* 起步/换向过渡圈数(不累计) */
#define CALIB_CFG_L5_COG_SLIP_RATIO   0.25f   /* 失步判定: |ω实际|<命令×本系数
                                                 * (开环拖动在齿槽峰处速度自然跌落,
                                                 *  阈值过严会误判, 0.5 实测误报) */
#define CALIB_CFG_L5_COG_SLIP_TICKS   6000u   /* 失步持续时间(6000 tick=0.6s) */
#define CALIB_CFG_L5_COG_TABLE_MA_MAX 2000.0f /* 表幅值上限(mA), 超出判标定异常 */

/* ===================== L3 编码器校准时间（与电机无关）===================== */
#define CALIB_CFG_L3_ALIGN_TIME_S 2.0f
#define CALIB_CFG_L3_SAMPLE_COUNT 100
#define CALIB_CFG_L3_DIR_TIME_S   1.0f

/* ===================== 极对数通用范围（与电机型号无关）===================== */
#define CALIB_CFG_R_MIN_OHM_SAFE 0.01f
#define CALIB_CFG_POLE_PAIRS_MIN 1
#define CALIB_CFG_POLE_PAIRS_MAX 24 /* 24 覆盖多极对电机(RS03=21, 轮毂电机可达23) */

/* 标定对齐电流安全上限(低阻电机突破 1.5A 默认上限, 保证足够对齐力矩)
 * RS03(R=0.1Ω, 额定13A) 用 6A 时电压=0.6V, 占空比 1.25%, 力矩 1.2Nm(电机端)
 * 6A 为 46% 额定, 短时标定(扫描10s)安全, 避免大电流持续导致绕组发热
 * 3A 时力矩不足导致极对数辨识偏低(实测21→20), 提升至 6A 修复 */
#define CALIB_CFG_ALIGN_CURRENT_MAX_A 6.0f

/* ===================== L6 惯量辨识参数（与电机无关）===================== */
/* 双向恒流加速法：恒流 ±I 往返加速，在速度窗 [v_a,v_b] 内 LSQ 回归斜率，
 * 往返取和抵消摩擦/磁滞负载转矩。开窗由运动触发（|v| 进入窗口），
 * 测试电流按飞行时长自适应缩放（小惯量自动减小电流拉长斜坡保证采样窗）。
 * 电流/转速上限派生见 calib_config_runtime.c。*/
#define CALIB_CFG_L6_INERTIA_SPEED_FRAC      0.3f    /* v_b = max_speed×0.3 再 clamp */
#define CALIB_CFG_L6_INERTIA_SPEED_MIN_RAD_S 5.0f    /* v_b 下限(rad/s) */
#define CALIB_CFG_L6_INERTIA_SPEED_MAX_RAD_S 50.0f   /* v_b 上限(rad/s 台架机械约束) */
#define CALIB_CFG_L6_INERTIA_VSTART_RAD_S    1.0f    /* v_a 开窗速度(rad/s) */
#define CALIB_CFG_L6_INERTIA_VSTOP_RAD_S     0.5f    /* 尝试间停机判定速度(rad/s) */
#define CALIB_CFG_L6_INERTIA_VHARD_FRAC      0.5f    /* 硬超速中止 = max_speed×0.5 */
#define CALIB_CFG_L6_INERTIA_CURRENT_FRAC    0.3f    /* 初始测试电流 = peak×0.3 */
#define CALIB_CFG_L6_INERTIA_CURRENT_MIN_A   0.3f    /* 初始电流下限(A) */
#define CALIB_CFG_L6_INERTIA_CURRENT_FLOOR_A 0.05f   /* 自适应电流下限(A) */
#define CALIB_CFG_L6_INERTIA_MAX_ATTEMPTS    4u      /* 最大测量尝试次数 */
#define CALIB_CFG_L6_INERTIA_LEAD_TICKS      500u    /* 相1开窗前导 50ms(PLL/电流环整定) */
#define CALIB_CFG_L6_INERTIA_PRE_TIMEOUT_TICKS  15000u /* 起动段超时 1.5s */
#define CALIB_CFG_L6_INERTIA_WIN_TIMEOUT_TICKS  15000u /* 采样窗超时 1.5s */
#define CALIB_CFG_L6_INERTIA_SETTLE_TICKS    15000u /* 尝试间停机等待上限 1.5s */
#define CALIB_CFG_L6_INERTIA_FLIGHT_TARGET_TICKS 3000u /* 目标飞行时长 300ms */
#define CALIB_CFG_L6_INERTIA_MIN_SAMPLES     200u   /* 每窗最小样本数(10kHz→20ms) */
#define CALIB_CFG_L6_INERTIA_MIN_ACCEL_RAD_S2 2.0f  /* 最小可辨识加速度(rad/s²) */
#define CALIB_CFG_L6_INERTIA_IQ_TRACK_RATIO  0.5f   /* iq̄/I 最低跟踪比(电压饱和检测) */
#define CALIB_CFG_L6_INERTIA_CURLOOP_BW_HZ   500.0f /* 标定用本地电流环带宽(Hz) */
#define CALIB_CFG_L6_INERTIA_PI_INTEG_MAX_V  5.0f   /* 电流 PI 积分限幅(V 抗饱和) */

#endif /* __CALIB_CONFIG_H__ */
