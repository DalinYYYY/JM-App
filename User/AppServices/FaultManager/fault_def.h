/**
 * @file        fault_def.h
 * @brief 		故障码表定义(由 fault_param.csv 生成, 勿手改)
 *
 * @author      fault_param_generate.py
 * @version     1.0
 * @date        2026-09-09
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @par 修改日志:
 * | 日期       | 版本 | 作者  | 修改内容   |
 * |------------|------|-------|------------|
 * | 2026-09-09 | 1.0  | auto  | 初始生成   |
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 *
 * @details 故障码编码 0xSLNN: S=故障来源(高4位, 数值越小优先级越高),
 *          L=故障级别(中4位), NN=编号(低8位)。
 *          优先级 = 码值数值比较(先来源后级别), 码值越小优先级越高。
 *          共 109 项: 一期 9 + 二期 0 + 预留 100(硬件缺失/未实现, 永不触发)。
 *          阈值参数经 motor_info FaultParam 块配置, 字段名见 CSV 参数Key列。
 */

#ifndef __FAULT_DEF_H__
#define __FAULT_DEF_H__

#include <stdint.h>
#include <stddef.h> /* NULL */

#ifdef __cplusplus
extern "C"
{
#endif

/* ===================== 故障来源 (0xSLNN 高4位, 编码顺序=优先级顺序) ===================== */
typedef enum
{
	FAULT_SOURCE_SAFETY = 0x1,        /* 安全系统(优先级最高) */
	FAULT_SOURCE_POWER = 0x2,         /* 电源系统 */
	FAULT_SOURCE_DRIVER = 0x3,        /* 驱动器 */
	FAULT_SOURCE_MOTOR = 0x4,         /* 电机本体 */
	FAULT_SOURCE_ENCODER = 0x5,       /* 编码器/传感器 */
	FAULT_SOURCE_MECHANICAL = 0x6,    /* 机械传动系统 */
	FAULT_SOURCE_BRAKE = 0x7,         /* 抱闸/刹车系统 */
	FAULT_SOURCE_SOFTWARE = 0x8,      /* 软件与算法 */
	FAULT_SOURCE_COMMUNICATION = 0x9, /* 通信系统 */
	FAULT_SOURCE_ENVIRONMENT = 0xA,   /* 环境因素(优先级最低) */
} fault_source_e;

/* ===================== 故障级别 (中4位, 恢复策略) =====================
 * 故障级: 立即停机, 需断电排查/人工清障
 * 异常级: 按处理动作降功率或停机, 故障消除后手动清除
 * 警告级: 正常运行仅记录, 自动清除或手动清除
 * 注: 即时响应对应"处理动作"字段(独立于级别), 级别决定恢复与展示语义 */
typedef enum
{
	FAULT_LEVEL_CRITICAL = 0x1,  /* 故障级 */
	FAULT_LEVEL_EXCEPTION = 0x2, /* 异常级 */
	FAULT_LEVEL_WARNING = 0x3,   /* 警告级 */
} fault_level_e;

/* ===================== 处理动作 (触发时的即时响应) ===================== */
typedef enum
{
	FAULT_ACTION_STOP_POWER = 0, /* 立即停机, 切断电源语义 */
	FAULT_ACTION_STOP_BRAKE = 1, /* 立即停机, 动态刹车 */
	FAULT_ACTION_STOP_IDLE = 2,  /* 立即停机, 可软件清障恢复 */
	FAULT_ACTION_DERATE = 3,     /* 降功率运行(系数经降功率档查 FaultParam) */
	FAULT_ACTION_CLAMP_POS = 4,  /* 软限位钳制(停止运动允许反向) */
	FAULT_ACTION_DENY = 5,       /* 拒绝操作(禁止运行/使能) */
	FAULT_ACTION_RECAL = 6,      /* 提示重新标定 */
	FAULT_ACTION_LOG = 7,        /* 仅记录 */
	FAULT_ACTION_RESET = 8,      /* 系统复位 */
} fault_action_e;

/* ===================== 故障记录状态 ===================== */
typedef enum
{
	FAULT_STATUS_INACTIVE = 0, /* 未激活 */
	FAULT_STATUS_ACTIVE = 1,   /* 活动状态 */
	FAULT_STATUS_CLEARED = 2,  /* 已清除(历史保留) */
} fault_status_e;

/* ===================== 实现期 =====================
 * P0=预留(硬件缺失/未实现, 检测永不触发, 码值保留供协议兼容)
 * P1=一期实现, P2=二期实现 */
typedef enum
{
	FAULT_PHASE_RESERVED = 0,
	FAULT_PHASE_P1 = 1,
	FAULT_PHASE_P2 = 2,
} fault_phase_e;

/* ===================== 故障码 (全量 109 项, 升序) ===================== */
typedef enum
{
	FAULT_ESTOP = 0x1101, /* 急停触发 */
	FAULT_HARD_LIMIT = 0x1102, /* 硬限位触发 */
	FAULT_SOFT_LIMIT = 0x1201, /* 软限位触发 */
	FAULT_COLLISION = 0x1202, /* 碰撞检测触发 */
	FAULT_NEAR_SOFT_LIMIT = 0x1301, /* 接近软限位 */
	FAULT_REVERSE_PWR = 0x2101, /* 电源反接 */
	FAULT_VBUS_OVER = 0x2102, /* 母线过压 P1 */
	FAULT_VBUS_UNDER = 0x2103, /* 母线欠压 P1 */
	FAULT_IBUS_OVER = 0x2104, /* 母线过流 P1 */
	FAULT_VBUS_SAMPLE = 0x2105, /* 母线电压采样异常 */
	FAULT_VBUS_OVER_MID = 0x2201, /* 母线过压(中度) */
	FAULT_VBUS_UNDER_MID = 0x2202, /* 母线欠压(中度) */
	FAULT_IBUS_CONT = 0x2203, /* 长时间过流 P1 */
	FAULT_BATT_LOW = 0x2204, /* 电池低电量 */
	FAULT_PWR_EFF_LOW = 0x2301, /* 电源效率下降 */
	FAULT_BATT_CYCLE = 0x2302, /* 电池循环次数预警 */
	FAULT_VBUS_RIPPLE = 0x2303, /* 母线电压波动 */
	FAULT_GATE_NFAULT = 0x3101, /* 硬件短路保护 P1 */
	FAULT_I_OVER_PEAK = 0x3102, /* 峰值电流过载 P1 */
	FAULT_THERMAL_INT = 0x3103, /* 累计热量过载 */
	FAULT_DRV_INIT = 0x3104, /* DRV芯片初始化错误 */
	FAULT_IA_RANGE = 0x3105, /* A相电流采样错误 */
	FAULT_IB_RANGE = 0x3106, /* B相电流采样错误 */
	FAULT_IC_RANGE = 0x3107, /* C相电流采样错误 */
	FAULT_MCU_SELFTEST = 0x3108, /* MCU/FPGA故障 */
	FAULT_FET_OVER_TEMP = 0x3201, /* 驱动器过温 */
	FAULT_FET_UNDER_TEMP = 0x3202, /* 驱动器低温 */
	FAULT_DUTY_SATUR = 0x3203, /* PWM占空比饱和 */
	FAULT_REGEN_OVER = 0x3204, /* 再生能量过载 */
	FAULT_FET_TEMP_SAMPLE = 0x3205, /* 驱动器温度采样异常 */
	FAULT_I_RIPPLE = 0x3301, /* 驱动波形畸变 */
	FAULT_FAN_FAULT = 0x3302, /* 风扇故障 */
	FAULT_FET_TEMP_WARN = 0x3303, /* 驱动器温度预警 */
	FAULT_MOTOR_OVER_TEMP = 0x4101, /* 电机过温熔断 */
	FAULT_PHASE_SHORT = 0x4102, /* 相间短路 */
	FAULT_GND_SHORT = 0x4103, /* 电机对地短路 */
	FAULT_PHASE_OPEN = 0x4104, /* 电机断路 */
	FAULT_MOTOR_OVERLOAD = 0x4105, /* 电机过载 */
	FAULT_MOTOR_OVER_SPEED = 0x4106, /* 电机超速 P1 */
	FAULT_I_UNBALANCE = 0x4201, /* 三相不平衡 */
	FAULT_MOTOR_STALL = 0x4202, /* 电机堵转 P1 */
	FAULT_DEMAG = 0x4203, /* 磁钢退磁 */
	FAULT_WINDING_HOT = 0x4204, /* 绕组过热 */
	FAULT_MOTOR_TEMP_SAMPLE = 0x4205, /* 电机温度采样异常 */
	FAULT_MOTOR_EFF_LOW = 0x4301, /* 电机效率下降 */
	FAULT_MOTOR_TEMP_WARN = 0x4302, /* 电机温度预警 */
	FAULT_ENC_POS_LOST = 0x5101, /* 绝对位置丢失 */
	FAULT_ENC_DEMAG = 0x5102, /* 磁编码器消磁 */
	FAULT_HALL_ERR = 0x5103, /* 霍尔传感器错误 */
	FAULT_ENC_JUMP = 0x5104, /* 位置数据跳变 */
	FAULT_ENC_COMM_LOST = 0x5105, /* 编码器通讯中断 */
	FAULT_FORCE_SENSOR = 0x5106, /* 力传感器故障 */
	FAULT_MULTITURN_OVF = 0x5201, /* 多圈计数溢出 */
	FAULT_AB_PHASE = 0x5202, /* 信号相位偏移 */
	FAULT_ZERO_OFFSET = 0x5203, /* 零位偏移 */
	FAULT_ENC_HOT = 0x5204, /* 编码器过热 */
	FAULT_FORCE_OVER = 0x5205, /* 力传感器过载 */
	FAULT_ENC_SNR = 0x5301, /* 分辨率降级 */
	FAULT_ZERO_OFFSET_MIN = 0x5302, /* 轻微零位偏移 */
	FAULT_ENC_TEMP_WARN = 0x5303, /* 编码器温度预警 */
	FAULT_GEAR_STUCK = 0x6101, /* 减速器卡死 */
	FAULT_BEARING = 0x6102, /* 轴承损坏 */
	FAULT_GEAR_TOOTH = 0x6103, /* 齿轮断齿 */
	FAULT_BACKLASH = 0x6201, /* 减速器回程间隙过大 */
	FAULT_LUBE_AGE = 0x6202, /* 润滑脂老化 */
	FAULT_FASTENER_LOOSE = 0x6203, /* 紧固件松动 */
	FAULT_GEAR_WEAR = 0x6301, /* 减速器磨损预警 */
	FAULT_VIBRATION = 0x6302, /* 振动异常 */
	FAULT_BRAKE_OPEN_FAIL = 0x7101, /* 抱闸打开失败 */
	FAULT_BRAKE_CLOSE_FAIL = 0x7102, /* 抱闸闭合失败 */
	FAULT_BRAKE_HOT = 0x7103, /* 抱闸过热 */
	FAULT_BRAKE_WEAR = 0x7201, /* 抱闸磨损 */
	FAULT_BRAKE_I_AB = 0x7202, /* 抱闸电流异常 */
	FAULT_BRAKE_LIFE = 0x7301, /* 抱闸寿命预警 */
	FAULT_WDT_RESET = 0x8101, /* 看门狗超时 */
	FAULT_TASK_DEADLOCK = 0x8102, /* 关键任务死锁 */
	FAULT_MEM_ERR = 0x8103, /* 内存错误 */
	FAULT_FLASH_ERR = 0x8104, /* FLASH错误 */
	FAULT_PARAM_RANGE = 0x8105, /* 参数越界 */
	FAULT_HOMING_ERR = 0x8106, /* 回零错误 */
	FAULT_FW_CRC = 0x8107, /* 固件校验错误 */
	FAULT_FOLLOW_ERR = 0x8108, /* 跟随误差过大 */
	FAULT_NOT_CALIB = 0x8109, /* 未标定运行 P1 */
	FAULT_ALGO_DIVERGE = 0x810A, /* 算法发散 */
	FAULT_TRAJ_CLAMP = 0x8201, /* 轨迹规划超限 */
	FAULT_MODE_INVALID = 0x8202, /* 运行模式错误 */
	FAULT_UNIT_ERR = 0x8203, /* 单位转换错误 */
	FAULT_TASK_TIMEOUT_SLOW = 0x8204, /* 非关键任务超时 */
	FAULT_EEPROM_ERR = 0x8205, /* 参数存储器读写错误 */
	FAULT_CONV_SLOW = 0x8301, /* 算法收敛延迟 */
	FAULT_LOG_FULL = 0x8302, /* 日志存储满 */
	FAULT_PARAM_DRIFT = 0x8303, /* 参数漂移 */
	FAULT_BUS_DEAD = 0x9101, /* 总线死锁 */
	FAULT_HOST_LOST = 0x9102, /* 主节点失联 */
	FAULT_BUS_OFF = 0x9103, /* 物理层损坏 */
	FAULT_BUS_LOAD_HI = 0x9201, /* 总线过载 */
	FAULT_FRAME_ERR = 0x9202, /* 帧错误 */
	FAULT_FRAME_LOSS = 0x9203, /* 帧丢失 */
	FAULT_CRC_RATE = 0x9301, /* CRC错误率上升 */
	FAULT_ARB_LOSS = 0x9302, /* 仲裁冲突频发 */
	FAULT_BUS_LOAD_WARN = 0x9303, /* 总线负载预警 */
	FAULT_HUMIDITY = 0xA101, /* 过湿短路 */
	FAULT_EMC = 0xA102, /* 强电磁干扰 */
	FAULT_PRESSURE = 0xA201, /* 气压异常 */
	FAULT_ENV_VIB = 0xA202, /* 环境过振 */
	FAULT_ENV_HOT = 0xA203, /* 环境温度过高 */
	FAULT_ENV_COLD = 0xA204, /* 环境温度过低 */
	FAULT_PRESSURE_WARN = 0xA301, /* 轻度气压异常 */
	FAULT_ENV_TEMP_WARN = 0xA302, /* 环境温度预警 */
} fault_code_e;

#define FAULT_CODE_COUNT 109

/* ===================== 三级使能掩码 bit 位定义 (级别内序号, 生成期绑定) =====================
 * 每个故障码在所属级别 64bit 使能掩码中的 bit 位, 与 fault_meta_table 的
 * FAULT_META_LEVEL_BIT 编译期同源(同一 _level_bit 生成)。
 * 掩码参数: 故障级=mask_critical1/2(PID154/155) 异常级=mask_exception1/2(PID156/157)
 *           警告级=mask_warning1/2(PID158/159), 位=1 使能该故障检测, 位=0 禁用 */
typedef enum
{
	/* ---- 故障级 -> mask_critical1/2 (PID154/155),  bit0~31 在低位字 ---- */
	FAULT_BIT_ESTOP = 0, /* 0x1101 急停触发 */
	FAULT_BIT_HARD_LIMIT = 1, /* 0x1102 硬限位触发 */
	FAULT_BIT_REVERSE_PWR = 2, /* 0x2101 电源反接 */
	FAULT_BIT_VBUS_OVER = 3, /* 0x2102 母线过压 */
	FAULT_BIT_VBUS_UNDER = 4, /* 0x2103 母线欠压 */
	FAULT_BIT_IBUS_OVER = 5, /* 0x2104 母线过流 */
	FAULT_BIT_VBUS_SAMPLE = 6, /* 0x2105 母线电压采样异常 */
	FAULT_BIT_GATE_NFAULT = 7, /* 0x3101 硬件短路保护 */
	FAULT_BIT_I_OVER_PEAK = 8, /* 0x3102 峰值电流过载 */
	FAULT_BIT_THERMAL_INT = 9, /* 0x3103 累计热量过载 */
	FAULT_BIT_DRV_INIT = 10, /* 0x3104 DRV芯片初始化错误 */
	FAULT_BIT_IA_RANGE = 11, /* 0x3105 A相电流采样错误 */
	FAULT_BIT_IB_RANGE = 12, /* 0x3106 B相电流采样错误 */
	FAULT_BIT_IC_RANGE = 13, /* 0x3107 C相电流采样错误 */
	FAULT_BIT_MCU_SELFTEST = 14, /* 0x3108 MCU/FPGA故障 */
	FAULT_BIT_MOTOR_OVER_TEMP = 15, /* 0x4101 电机过温熔断 */
	FAULT_BIT_PHASE_SHORT = 16, /* 0x4102 相间短路 */
	FAULT_BIT_GND_SHORT = 17, /* 0x4103 电机对地短路 */
	FAULT_BIT_PHASE_OPEN = 18, /* 0x4104 电机断路 */
	FAULT_BIT_MOTOR_OVERLOAD = 19, /* 0x4105 电机过载 */
	FAULT_BIT_MOTOR_OVER_SPEED = 20, /* 0x4106 电机超速 */
	FAULT_BIT_ENC_POS_LOST = 21, /* 0x5101 绝对位置丢失 */
	FAULT_BIT_ENC_DEMAG = 22, /* 0x5102 磁编码器消磁 */
	FAULT_BIT_HALL_ERR = 23, /* 0x5103 霍尔传感器错误 */
	FAULT_BIT_ENC_JUMP = 24, /* 0x5104 位置数据跳变 */
	FAULT_BIT_ENC_COMM_LOST = 25, /* 0x5105 编码器通讯中断 */
	FAULT_BIT_FORCE_SENSOR = 26, /* 0x5106 力传感器故障 */
	FAULT_BIT_GEAR_STUCK = 27, /* 0x6101 减速器卡死 */
	FAULT_BIT_BEARING = 28, /* 0x6102 轴承损坏 */
	FAULT_BIT_GEAR_TOOTH = 29, /* 0x6103 齿轮断齿 */
	FAULT_BIT_BRAKE_OPEN_FAIL = 30, /* 0x7101 抱闸打开失败 */
	FAULT_BIT_BRAKE_CLOSE_FAIL = 31, /* 0x7102 抱闸闭合失败 */
	FAULT_BIT_BRAKE_HOT = 32, /* 0x7103 抱闸过热 */
	FAULT_BIT_WDT_RESET = 33, /* 0x8101 看门狗超时 */
	FAULT_BIT_TASK_DEADLOCK = 34, /* 0x8102 关键任务死锁 */
	FAULT_BIT_MEM_ERR = 35, /* 0x8103 内存错误 */
	FAULT_BIT_FLASH_ERR = 36, /* 0x8104 FLASH错误 */
	FAULT_BIT_PARAM_RANGE = 37, /* 0x8105 参数越界 */
	FAULT_BIT_HOMING_ERR = 38, /* 0x8106 回零错误 */
	FAULT_BIT_FW_CRC = 39, /* 0x8107 固件校验错误 */
	FAULT_BIT_FOLLOW_ERR = 40, /* 0x8108 跟随误差过大 */
	FAULT_BIT_NOT_CALIB = 41, /* 0x8109 未标定运行 */
	FAULT_BIT_ALGO_DIVERGE = 42, /* 0x810A 算法发散 */
	FAULT_BIT_BUS_DEAD = 43, /* 0x9101 总线死锁 */
	FAULT_BIT_HOST_LOST = 44, /* 0x9102 主节点失联 */
	FAULT_BIT_BUS_OFF = 45, /* 0x9103 物理层损坏 */
	FAULT_BIT_HUMIDITY = 46, /* 0xA101 过湿短路 */
	FAULT_BIT_EMC = 47, /* 0xA102 强电磁干扰 */
	/* ---- 异常级 -> mask_exception1/2 (PID156/157), bit0~31 在低位字 ---- */
	FAULT_BIT_SOFT_LIMIT = 0, /* 0x1201 软限位触发 */
	FAULT_BIT_COLLISION = 1, /* 0x1202 碰撞检测触发 */
	FAULT_BIT_VBUS_OVER_MID = 2, /* 0x2201 母线过压(中度) */
	FAULT_BIT_VBUS_UNDER_MID = 3, /* 0x2202 母线欠压(中度) */
	FAULT_BIT_IBUS_CONT = 4, /* 0x2203 长时间过流 */
	FAULT_BIT_BATT_LOW = 5, /* 0x2204 电池低电量 */
	FAULT_BIT_FET_OVER_TEMP = 6, /* 0x3201 驱动器过温 */
	FAULT_BIT_FET_UNDER_TEMP = 7, /* 0x3202 驱动器低温 */
	FAULT_BIT_DUTY_SATUR = 8, /* 0x3203 PWM占空比饱和 */
	FAULT_BIT_REGEN_OVER = 9, /* 0x3204 再生能量过载 */
	FAULT_BIT_FET_TEMP_SAMPLE = 10, /* 0x3205 驱动器温度采样异常 */
	FAULT_BIT_I_UNBALANCE = 11, /* 0x4201 三相不平衡 */
	FAULT_BIT_MOTOR_STALL = 12, /* 0x4202 电机堵转 */
	FAULT_BIT_DEMAG = 13, /* 0x4203 磁钢退磁 */
	FAULT_BIT_WINDING_HOT = 14, /* 0x4204 绕组过热 */
	FAULT_BIT_MOTOR_TEMP_SAMPLE = 15, /* 0x4205 电机温度采样异常 */
	FAULT_BIT_MULTITURN_OVF = 16, /* 0x5201 多圈计数溢出 */
	FAULT_BIT_AB_PHASE = 17, /* 0x5202 信号相位偏移 */
	FAULT_BIT_ZERO_OFFSET = 18, /* 0x5203 零位偏移 */
	FAULT_BIT_ENC_HOT = 19, /* 0x5204 编码器过热 */
	FAULT_BIT_FORCE_OVER = 20, /* 0x5205 力传感器过载 */
	FAULT_BIT_BACKLASH = 21, /* 0x6201 减速器回程间隙过大 */
	FAULT_BIT_LUBE_AGE = 22, /* 0x6202 润滑脂老化 */
	FAULT_BIT_FASTENER_LOOSE = 23, /* 0x6203 紧固件松动 */
	FAULT_BIT_BRAKE_WEAR = 24, /* 0x7201 抱闸磨损 */
	FAULT_BIT_BRAKE_I_AB = 25, /* 0x7202 抱闸电流异常 */
	FAULT_BIT_TRAJ_CLAMP = 26, /* 0x8201 轨迹规划超限 */
	FAULT_BIT_MODE_INVALID = 27, /* 0x8202 运行模式错误 */
	FAULT_BIT_UNIT_ERR = 28, /* 0x8203 单位转换错误 */
	FAULT_BIT_TASK_TIMEOUT_SLOW = 29, /* 0x8204 非关键任务超时 */
	FAULT_BIT_EEPROM_ERR = 30, /* 0x8205 参数存储器读写错误 */
	FAULT_BIT_BUS_LOAD_HI = 31, /* 0x9201 总线过载 */
	FAULT_BIT_FRAME_ERR = 32, /* 0x9202 帧错误 */
	FAULT_BIT_FRAME_LOSS = 33, /* 0x9203 帧丢失 */
	FAULT_BIT_PRESSURE = 34, /* 0xA201 气压异常 */
	FAULT_BIT_ENV_VIB = 35, /* 0xA202 环境过振 */
	FAULT_BIT_ENV_HOT = 36, /* 0xA203 环境温度过高 */
	FAULT_BIT_ENV_COLD = 37, /* 0xA204 环境温度过低 */
	/* ---- 警告级 -> mask_warning1/2 (PID158/159),   bit0~31 在低位字 ---- */
	FAULT_BIT_NEAR_SOFT_LIMIT = 0, /* 0x1301 接近软限位 */
	FAULT_BIT_PWR_EFF_LOW = 1, /* 0x2301 电源效率下降 */
	FAULT_BIT_BATT_CYCLE = 2, /* 0x2302 电池循环次数预警 */
	FAULT_BIT_VBUS_RIPPLE = 3, /* 0x2303 母线电压波动 */
	FAULT_BIT_I_RIPPLE = 4, /* 0x3301 驱动波形畸变 */
	FAULT_BIT_FAN_FAULT = 5, /* 0x3302 风扇故障 */
	FAULT_BIT_FET_TEMP_WARN = 6, /* 0x3303 驱动器温度预警 */
	FAULT_BIT_MOTOR_EFF_LOW = 7, /* 0x4301 电机效率下降 */
	FAULT_BIT_MOTOR_TEMP_WARN = 8, /* 0x4302 电机温度预警 */
	FAULT_BIT_ENC_SNR = 9, /* 0x5301 分辨率降级 */
	FAULT_BIT_ZERO_OFFSET_MIN = 10, /* 0x5302 轻微零位偏移 */
	FAULT_BIT_ENC_TEMP_WARN = 11, /* 0x5303 编码器温度预警 */
	FAULT_BIT_GEAR_WEAR = 12, /* 0x6301 减速器磨损预警 */
	FAULT_BIT_VIBRATION = 13, /* 0x6302 振动异常 */
	FAULT_BIT_BRAKE_LIFE = 14, /* 0x7301 抱闸寿命预警 */
	FAULT_BIT_CONV_SLOW = 15, /* 0x8301 算法收敛延迟 */
	FAULT_BIT_LOG_FULL = 16, /* 0x8302 日志存储满 */
	FAULT_BIT_PARAM_DRIFT = 17, /* 0x8303 参数漂移 */
	FAULT_BIT_CRC_RATE = 18, /* 0x9301 CRC错误率上升 */
	FAULT_BIT_ARB_LOSS = 19, /* 0x9302 仲裁冲突频发 */
	FAULT_BIT_BUS_LOAD_WARN = 20, /* 0x9303 总线负载预警 */
	FAULT_BIT_PRESSURE_WARN = 21, /* 0xA301 轻度气压异常 */
	FAULT_BIT_ENV_TEMP_WARN = 22, /* 0xA302 环境温度预警 */
} fault_en_bit_e;

/* 故障码字段提取 */
#define FAULT_GET_SOURCE(code) ((uint8_t)(((uint16_t)(code) >> 12) & 0x0Fu))
#define FAULT_GET_LEVEL(code)  ((uint8_t)(((uint16_t)(code) >> 8) & 0x0Fu))

/* ===================== 故障元数据 (const 表, flash 存储) =====================
 * 由 CSV 生成, 字段打包为 4B/项:
 *   pack1(u8): [7:4]=fault_action_e [3:0]=level_bit 低4位
 *   pack2(u8): [7:6]=level_bit 高2位 [5:4]=fault_phase_e [3:2]=降功率档 [1:0]=fault_level_e
 * level_bit = 三级 64bit 使能掩码(mask_critical/exception/warning)的 bit 位,
 * 即该码在码表(升序)中同级别前驱的数量, 生成时直接绑定, 1=使能 0=禁用。
 * 经访问宏读取, 大小 109*4B */
typedef struct
{
	uint16_t code;   /* 故障码 0xSLNN */
	uint8_t pack1;   /* [7:4]=fault_action_e [3:0]=level_bit[3:0] */
	uint8_t pack2;   /* [7:6]=level_bit[5:4] [5:4]=fault_phase_e [3:2]=降功率档 [1:0]=fault_level_e */
} fault_meta_t;

#define FAULT_META_LEVEL(m)     ((fault_level_e)(((m)->pack2 >> 0) & 0x03u))
#define FAULT_META_ACTION(m)    ((fault_action_e)(((m)->pack1 >> 4) & 0x0Fu))
#define FAULT_META_LEVEL_BIT(m) ((uint8_t)(((((m)->pack2 >> 6) & 0x03u) << 4) | (((m)->pack1 >> 0) & 0x0Fu)))
#define FAULT_META_PHASE(m)     ((fault_phase_e)(((m)->pack2 >> 4) & 0x03u))
#define FAULT_META_DERATE(m)    ((uint8_t)(((m)->pack2 >> 2) & 0x03u))

extern const fault_meta_t fault_meta_table[FAULT_CODE_COUNT];

/* 故障码 -> 元数据表索引, 未定义码返回 -1 (表升序, 二分查找) */
int8_t fault_code_to_index(uint16_t code);

/* 取故障码元数据, 未定义码返回 NULL */
const fault_meta_t *fault_meta_get(uint16_t code);

/* 优先级比较: 返回 1 表示 f1 优先级高于 f2 (码值数值小者优先) */
uint8_t fault_is_higher_priority(uint16_t f1, uint16_t f2);

#ifdef __cplusplus
}
#endif
#endif /* __FAULT_DEF_H__ */
