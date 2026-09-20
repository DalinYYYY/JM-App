#ifndef __CALIB_MGR_H__
#define __CALIB_MGR_H__

#include "calib_types.h"

/**
 * @brief 初始化标定管理器
 * @param motor 底层电机设备指针（FOC/编码器/半桥，供标定模块直接操作硬件）
 * @param param 电机参数指针（标定结果写入此结构）
 * @param dt    控制周期(s)
 */
void calib_mgr_init(struct dev_motor *motor, motor_param_t *param, float dt);

/**
 * @brief 启动一次标定
 * @param level 标定级别（1-7，对应 CALIB_LEVEL1~7）
 * @param submode 子模式（各级别内定义，见 calib_types.h）
 * @return true 启动成功 / false 已在标定中或级别无效
 */
bool calib_mgr_start(uint8_t level, uint8_t submode);

/**
 * @brief 周期推进标定（motor_control_loop 的 CALIB 态调用）
 * @return 当前标定状态
 */
calib_state_e calib_mgr_poll(void);

/**
 * @brief 查询标定状态
 */
calib_status_t calib_mgr_get_status(void);

/**
 * @brief level 模块上报当前步骤号（单步标定用）
 * @note  level 模块在 calib_step_next 推进到新 step 时调用,
 *        使 0x97 查询能反映"卡在第几步"。step 从 0 开始。
 */
void calib_mgr_set_step(uint8_t step);

/**
 * @brief level 模块上报具体失败原因（覆盖 calib_mgr 默认的 TIMEOUT）
 * @note  level 模块在判定 FAILED 时调用,设置具体原因如
 *        CALIB_FAIL_OUT_OF_RANGE / CALIB_FAIL_SAMPLE_ABNORMAL 等。
 *        若不调用,calib_mgr 默认填 CALIB_FAIL_TIMEOUT。
 */
void calib_mgr_set_fail_reason(calib_fail_reason_e reason);

/**
 * @brief L7 上报序列进度（当前步骤索引 + 总步数）
 * @param step     当前步骤索引（0-based）
 * @param step_total L7 序列总步数
 * @note  同时设置 status.step 与 status.step_total, 供 0x97 查询。
 *        level/submode 不入此接口更新, 保持 calib_mgr_start 时写入的 L7/FULL_AUTO。
 */
void calib_mgr_set_l7_progress(uint8_t step, uint8_t step_total);

/**
 * @brief 强制中止当前标定
 */
void calib_mgr_abort(void);

/**
 * @brief 获取标定硬件访问接口（供各 level 模块访问 dev_motor_t/param/dt）
 * @note  在 level 模块的 start/poll/abort 中调用，获取注入的硬件资源
 */
const calib_io_t *calib_mgr_get_io(void);

/* 标记某子模式已完成（标定 DONE 时由 level 模块或 calib_mgr_poll 调用）*/
void calib_mgr_mark_done(uint8_t level, uint8_t submode);

/* 查询某子模式是否已完成（前置依赖检查用）*/
bool calib_mgr_is_done(uint8_t level, uint8_t submode);

/* 清除所有标定完成标志（重新标定前调用）*/
void calib_mgr_clear_done(void);

#endif /* __CALIB_MGR_H__ */
