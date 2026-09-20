#ifndef __MOTOR_CONFIG_BOARD_V1_H__
#define __MOTOR_CONFIG_BOARD_V1_H__

/* ======================================================================
 * Board: V1 电机选型（本板默认搭配的电机型号，首次上电 fallback 值）
 *   - 型号参数库见 User/Config/motor_profile.h（R/L/flux/KT 等）
 *   - 启用 Flash 的板运行期可通过 0xE7/0xEA 在线换电机，无需改此处
 *   - 未定义本宏的板由 motor_profile.h 兜底为 GM4820H
 * ====================================================================== */
#define MOTOR_PROFILE_BOARD MOTOR_PROFILE_QH8919 /* V1 台架现配强和 QH8919 关节电机 */

#endif /* __MOTOR_CONFIG_BOARD_V1_H__ */
