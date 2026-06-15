/**
 * @file multiturn_counter.h
 * @brief 电机绝对多圈计数模块（从 motion_param 拆分独立）
 *
 * 输入一路或多路角度，输出电机的绝对整圈数与多圈位置(rad)。支持三种机制：
 *   - 软件累圈 (SOFT)：单编码器靠过零跳变累加圈数（掉电丢失）
 *   - 双齿轮游标 (NONIUS_2GEAR)：主齿轮 + 1 副齿轮，齿数互质用游标原理重建绝对圈数
 *   - 三齿轮游标 (NONIUS_3GEAR)：主齿轮 + 2 副齿轮，扩展量程并交叉校验抗噪
 *
 * 本模块与 motion_param 完全独立：motion_param 吃机械角度出电角度/速度，
 * multiturn 吃齿轮角度出绝对圈数/位置，由调用方各自驱动后组合反馈。
 *
 * @author Dalin
 * @version 1.0
 * @date 2026-06-15
 */

#ifndef __MOTOR_MULTITURN_COUNTER_H_
#define __MOTOR_MULTITURN_COUNTER_H_

#include <stdint.h>
#include <stdbool.h>

/* 齿轮总数最大值（三齿轮游标：主齿轮 + 2 副齿轮） */
#define MULTITURN_GEAR_MAX 3u

/**
 * @brief 多圈计数模式
 */
typedef enum
{
	MULTITURN_MODE_NONE = 0,	 // 不计多圈，position 即单圈累计
	MULTITURN_MODE_SOFT,		 // 单编码器软件累圈（掉电丢失）
	MULTITURN_MODE_NONIUS_2GEAR, // 双齿轮游标绝对多圈
	MULTITURN_MODE_NONIUS_3GEAR, // 三齿轮游标绝对多圈（扩展量程 + 容错）
} multiturn_mode_e;

/**
 * @brief 多圈计数初始化配置
 *
 * 齿轮链以主齿轮（与测量轴同轴）为基准，gear_teeth[0]/gear_dir[0] 为主齿轮，
 * 其余为副齿轮。副齿轮齿数与主齿轮齿数互质时，游标周期内相位差单调变化。
 */
typedef struct
{
	multiturn_mode_e mode;					   // 多圈模式
	uint16_t gear_teeth[MULTITURN_GEAR_MAX];   // 各齿轮齿数（[0]=主齿轮）
	int8_t gear_dir[MULTITURN_GEAR_MAX];	   // 各齿轮计数方向 (+1/-1)
	uint16_t settle_ticks;					   // SOFT 模式上电稳定丢弃拍数（0 用默认 1000）
	float (*device_compensation_callback)(void); // 设备角度补偿 (rad)，可为 NULL（仅 SOFT 用）
} multiturn_config_t;

/**
 * @brief 多圈计数对象
 */
typedef struct multiturn
{
	/* ---- 配置 ---- */
	multiturn_config_t cfg;

	/* ---- 输出 ---- */
	volatile float position;		   // 累计位置 (rad，按模式：软件累圈或绝对多圈)
	int32_t turns;					   // 绝对整圈数
	volatile float multiturn_position; // 绝对多圈位置 (rad，与 position 对齐)

	/* ---- SOFT 模式状态 ---- */
	float last_single_rad;	 // 上一次单圈弧度
	uint16_t pos_settle_ticks; // 上电稳定计数

	/* ---- 接口 ---- */
	// 多齿轮入口：gear_angles[0]=主齿轮角(deg)，其余为副齿轮角；count 为齿轮数量
	float (*update)(struct multiturn *pobj, const float *gear_angles, uint8_t count);
	// 单角度便捷入口（SOFT 模式）：仅主轴角度
	float (*update_single)(struct multiturn *pobj, float mechanical_angle);
	float (*get_position)(struct multiturn *pobj);			 // 多圈位置 (rad)
	int32_t (*get_turns)(struct multiturn *pobj);			 // 绝对整圈数
} multiturn_t;

/**
 * @brief 初始化多圈计数模块
 * @param pobj 多圈对象
 * @param cfg  初始化配置
 */
void multiturn_init(multiturn_t *pobj, const multiturn_config_t *cfg);

#endif /* __MOTOR_MULTITURN_COUNTER_H_ */
