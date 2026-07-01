#ifndef __CALIB_MGR_H__
#define __CALIB_MGR_H__

#include "calib_types.h"

/**
 * @brief 初始化标定管理器
 * @param param 电机参数指针（标定结果写入此结构）
 * @param dt 控制周期(s)
 */
void calib_mgr_init(motor_param_t *param, float dt);

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
 * @brief 强制中止当前标定
 */
void calib_mgr_abort(void);

#endif /* __CALIB_MGR_H__ */
