#ifndef __CALIB_HW_H__
#define __CALIB_HW_H__

#include <stdbool.h>
#include "calib_types.h" /* 引入 dev_motor 前向声明与 motor_param_t */

/* ===================== 标定硬件访问会话 =====================
 * 封装"标定期间替换电角度回调 + 直接施加 dq 电压"的上下文。
 * 每个 level 模块在 start() 时持有一个本地 session，poll() 中传给 calib_hw_* 函数。
 * 系统单电机 + calib_mgr 保证同时只有一个标定活动，s_active 单例约束可接受。*/
typedef struct
{
	struct dev_motor *motor;    /* 目标电机设备 */
	float (*orig_ele_cb)(void); /* 保存的原始电角度回调，exit 时恢复 */
	float forced_ele_angle;     /* 标定期间强制使用的电角度(rad) */
} calib_hw_session_t;

/* 进入标定电压会话：保存并替换电角度回调，强制电角度=0 */
void calib_hw_enter(calib_hw_session_t *s, struct dev_motor *m);

/* 退出标定电压会话：撤销 PWM 输出，恢复原始电角度回调 */
void calib_hw_exit(calib_hw_session_t *s);

/* 施加 dq 电压（绕过电流环 PI，直接操作 FOC 链路 + SVPWM + 半桥）
 * @param theta 强制使用的电角度(rad)，用于标定期间固定电角度 */
void calib_hw_apply_voltage(calib_hw_session_t *s, float ud, float uq, float theta);

/* 撤销电压输出（PWM 三相置零）*/
void calib_hw_apply_zero(struct dev_motor *m);

/* 获取编码器原始角度(°) [0,360)，不含 offset/dir 补偿（通过抽象层访问）*/
float calib_hw_get_encoder_raw_deg(struct dev_motor *m);

/* 获取编码器当前机械角度(°) [0,360)，含 offset/dir 补偿 */
float calib_hw_get_encoder_mech_angle(struct dev_motor *m);

/* 施加 dq 电压并回读限幅后的实际电压，用于 R 标定两点差分法
 * @param ud_cmd/uq_cmd 命令电压(V)
 * @param theta 强制电角度(rad)
 * @param ud_act/uq_act 输出：限幅后的实际施加电压(V)，传 NULL 可不回读
 * @note 内部调 calib_hw_apply_voltage 完成限幅+归一化+SVPWM */
void calib_hw_apply_voltage_comp(calib_hw_session_t *s, float ud_cmd, float uq_cmd,
                                 float theta, float *ud_act, float *uq_act);

/* 施加交流注入电压（d 轴 DC + AC），用于低阻电机 R/Ld 辨识
 * @param ud_dc 直流偏置电压(V)
 * @param ud_ac 交流幅值(V)
 * @param freq_hz 交流频率(Hz)
 * @param tick 当前 tick 计数（用于计算相位）
 * @param theta 强制电角度(rad) */
void calib_hw_apply_ac_injection(calib_hw_session_t *s, float ud_dc, float ud_ac,
                                 float freq_hz, uint32_t tick, float theta);

#endif /* __CALIB_HW_H__ */
