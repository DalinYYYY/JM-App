/**
 * @file        fault_def.c
 * @brief 		故障码表数据(由 fault_param.csv 生成, 勿手改)
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
 */

#include "fault_def.h"

/* 故障元数据表: 按故障码升序排列(二分查找依赖), 字段含义见 fault_def.h */
const fault_meta_t fault_meta_table[FAULT_CODE_COUNT] = {
	{FAULT_ESTOP, 0x00u, 0x01u}, /* 急停触发 L1A0D0P0 BIT0 */
	{FAULT_HARD_LIMIT, 0x11u, 0x01u}, /* 硬限位触发 L1A1D0P0 BIT1 */
	{FAULT_SOFT_LIMIT, 0x40u, 0x02u}, /* 软限位触发 L2A4D0P0 BIT0 */
	{FAULT_COLLISION, 0x11u, 0x02u}, /* 碰撞检测触发 L2A1D0P0 BIT1 */
	{FAULT_NEAR_SOFT_LIMIT, 0x30u, 0x0Bu}, /* 接近软限位 L3A3D2P0 BIT0 */
	{FAULT_REVERSE_PWR, 0x02u, 0x01u}, /* 电源反接 L1A0D0P0 BIT2 */
	{FAULT_VBUS_OVER, 0x13u, 0x11u}, /* 母线过压 L1A1D0P1 BIT3 */
	{FAULT_VBUS_UNDER, 0x14u, 0x11u}, /* 母线欠压 L1A1D0P1 BIT4 */
	{FAULT_IBUS_OVER, 0x05u, 0x11u}, /* 母线过流 L1A0D0P1 BIT5 */
	{FAULT_VBUS_SAMPLE, 0x16u, 0x01u}, /* 母线电压采样异常 L1A1D0P0 BIT6 */
	{FAULT_VBUS_OVER_MID, 0x32u, 0x06u}, /* 母线过压(中度) L2A3D1P0 BIT2 */
	{FAULT_VBUS_UNDER_MID, 0x33u, 0x06u}, /* 母线欠压(中度) L2A3D1P0 BIT3 */
	{FAULT_IBUS_CONT, 0x34u, 0x1Au}, /* 长时间过流 L2A3D2P1 BIT4 */
	{FAULT_BATT_LOW, 0x35u, 0x0Au}, /* 电池低电量 L2A3D2P0 BIT5 */
	{FAULT_PWR_EFF_LOW, 0x71u, 0x03u}, /* 电源效率下降 L3A7D0P0 BIT1 */
	{FAULT_BATT_CYCLE, 0x72u, 0x03u}, /* 电池循环次数预警 L3A7D0P0 BIT2 */
	{FAULT_VBUS_RIPPLE, 0x73u, 0x03u}, /* 母线电压波动 L3A7D0P0 BIT3 */
	{FAULT_GATE_NFAULT, 0x07u, 0x11u}, /* 硬件短路保护 L1A0D0P1 BIT7 */
	{FAULT_I_OVER_PEAK, 0x18u, 0x11u}, /* 峰值电流过载 L1A1D0P1 BIT8 */
	{FAULT_THERMAL_INT, 0x19u, 0x01u}, /* 累计热量过载 L1A1D0P0 BIT9 */
	{FAULT_DRV_INIT, 0x0Au, 0x01u}, /* DRV芯片初始化错误 L1A0D0P0 BIT10 */
	{FAULT_IA_RANGE, 0x0Bu, 0x01u}, /* A相电流采样错误 L1A0D0P0 BIT11 */
	{FAULT_IB_RANGE, 0x0Cu, 0x01u}, /* B相电流采样错误 L1A0D0P0 BIT12 */
	{FAULT_IC_RANGE, 0x0Du, 0x01u}, /* C相电流采样错误 L1A0D0P0 BIT13 */
	{FAULT_MCU_SELFTEST, 0x0Eu, 0x01u}, /* MCU/FPGA故障 L1A0D0P0 BIT14 */
	{FAULT_FET_OVER_TEMP, 0x36u, 0x0Au}, /* 驱动器过温 L2A3D2P0 BIT6 */
	{FAULT_FET_UNDER_TEMP, 0x57u, 0x02u}, /* 驱动器低温 L2A5D0P0 BIT7 */
	{FAULT_DUTY_SATUR, 0x38u, 0x06u}, /* PWM占空比饱和 L2A3D1P0 BIT8 */
	{FAULT_REGEN_OVER, 0x39u, 0x06u}, /* 再生能量过载 L2A3D1P0 BIT9 */
	{FAULT_FET_TEMP_SAMPLE, 0x2Au, 0x02u}, /* 驱动器温度采样异常 L2A2D0P0 BIT10 */
	{FAULT_I_RIPPLE, 0x74u, 0x03u}, /* 驱动波形畸变 L3A7D0P0 BIT4 */
	{FAULT_FAN_FAULT, 0x75u, 0x03u}, /* 风扇故障 L3A7D0P0 BIT5 */
	{FAULT_FET_TEMP_WARN, 0x76u, 0x03u}, /* 驱动器温度预警 L3A7D0P0 BIT6 */
	{FAULT_MOTOR_OVER_TEMP, 0x1Fu, 0x01u}, /* 电机过温熔断 L1A1D0P0 BIT15 */
	{FAULT_PHASE_SHORT, 0x00u, 0x41u}, /* 相间短路 L1A0D0P0 BIT16 */
	{FAULT_GND_SHORT, 0x01u, 0x41u}, /* 电机对地短路 L1A0D0P0 BIT17 */
	{FAULT_PHASE_OPEN, 0x02u, 0x41u}, /* 电机断路 L1A0D0P0 BIT18 */
	{FAULT_MOTOR_OVERLOAD, 0x13u, 0x41u}, /* 电机过载 L1A1D0P0 BIT19 */
	{FAULT_MOTOR_OVER_SPEED, 0x14u, 0x51u}, /* 电机超速 L1A1D0P1 BIT20 */
	{FAULT_I_UNBALANCE, 0x3Bu, 0x0Au}, /* 三相不平衡 L2A3D2P0 BIT11 */
	{FAULT_MOTOR_STALL, 0x2Cu, 0x12u}, /* 电机堵转 L2A2D0P1 BIT12 */
	{FAULT_DEMAG, 0x3Du, 0x06u}, /* 磁钢退磁 L2A3D1P0 BIT13 */
	{FAULT_WINDING_HOT, 0x3Eu, 0x0Au}, /* 绕组过热 L2A3D2P0 BIT14 */
	{FAULT_MOTOR_TEMP_SAMPLE, 0x2Fu, 0x02u}, /* 电机温度采样异常 L2A2D0P0 BIT15 */
	{FAULT_MOTOR_EFF_LOW, 0x77u, 0x03u}, /* 电机效率下降 L3A7D0P0 BIT7 */
	{FAULT_MOTOR_TEMP_WARN, 0x78u, 0x03u}, /* 电机温度预警 L3A7D0P0 BIT8 */
	{FAULT_ENC_POS_LOST, 0x15u, 0x41u}, /* 绝对位置丢失 L1A1D0P0 BIT21 */
	{FAULT_ENC_DEMAG, 0x06u, 0x41u}, /* 磁编码器消磁 L1A0D0P0 BIT22 */
	{FAULT_HALL_ERR, 0x17u, 0x41u}, /* 霍尔传感器错误 L1A1D0P0 BIT23 */
	{FAULT_ENC_JUMP, 0x18u, 0x41u}, /* 位置数据跳变 L1A1D0P0 BIT24 */
	{FAULT_ENC_COMM_LOST, 0x19u, 0x41u}, /* 编码器通讯中断 L1A1D0P0 BIT25 */
	{FAULT_FORCE_SENSOR, 0x1Au, 0x41u}, /* 力传感器故障 L1A1D0P0 BIT26 */
	{FAULT_MULTITURN_OVF, 0x60u, 0x42u}, /* 多圈计数溢出 L2A6D0P0 BIT16 */
	{FAULT_AB_PHASE, 0x71u, 0x42u}, /* 信号相位偏移 L2A7D0P0 BIT17 */
	{FAULT_ZERO_OFFSET, 0x62u, 0x42u}, /* 零位偏移 L2A6D0P0 BIT18 */
	{FAULT_ENC_HOT, 0x33u, 0x46u}, /* 编码器过热 L2A3D1P0 BIT19 */
	{FAULT_FORCE_OVER, 0x34u, 0x4Au}, /* 力传感器过载 L2A3D2P0 BIT20 */
	{FAULT_ENC_SNR, 0x79u, 0x03u}, /* 分辨率降级 L3A7D0P0 BIT9 */
	{FAULT_ZERO_OFFSET_MIN, 0x7Au, 0x03u}, /* 轻微零位偏移 L3A7D0P0 BIT10 */
	{FAULT_ENC_TEMP_WARN, 0x7Bu, 0x03u}, /* 编码器温度预警 L3A7D0P0 BIT11 */
	{FAULT_GEAR_STUCK, 0x0Bu, 0x41u}, /* 减速器卡死 L1A0D0P0 BIT27 */
	{FAULT_BEARING, 0x0Cu, 0x41u}, /* 轴承损坏 L1A0D0P0 BIT28 */
	{FAULT_GEAR_TOOTH, 0x0Du, 0x41u}, /* 齿轮断齿 L1A0D0P0 BIT29 */
	{FAULT_BACKLASH, 0x75u, 0x42u}, /* 减速器回程间隙过大 L2A7D0P0 BIT21 */
	{FAULT_LUBE_AGE, 0x76u, 0x42u}, /* 润滑脂老化 L2A7D0P0 BIT22 */
	{FAULT_FASTENER_LOOSE, 0x37u, 0x46u}, /* 紧固件松动 L2A3D1P0 BIT23 */
	{FAULT_GEAR_WEAR, 0x7Cu, 0x03u}, /* 减速器磨损预警 L3A7D0P0 BIT12 */
	{FAULT_VIBRATION, 0x7Du, 0x03u}, /* 振动异常 L3A7D0P0 BIT13 */
	{FAULT_BRAKE_OPEN_FAIL, 0x0Eu, 0x41u}, /* 抱闸打开失败 L1A0D0P0 BIT30 */
	{FAULT_BRAKE_CLOSE_FAIL, 0x0Fu, 0x41u}, /* 抱闸闭合失败 L1A0D0P0 BIT31 */
	{FAULT_BRAKE_HOT, 0x00u, 0x81u}, /* 抱闸过热 L1A0D0P0 BIT32 */
	{FAULT_BRAKE_WEAR, 0x78u, 0x42u}, /* 抱闸磨损 L2A7D0P0 BIT24 */
	{FAULT_BRAKE_I_AB, 0x39u, 0x46u}, /* 抱闸电流异常 L2A3D1P0 BIT25 */
	{FAULT_BRAKE_LIFE, 0x7Eu, 0x03u}, /* 抱闸寿命预警 L3A7D0P0 BIT14 */
	{FAULT_WDT_RESET, 0x81u, 0x81u}, /* 看门狗超时 L1A8D0P0 BIT33 */
	{FAULT_TASK_DEADLOCK, 0x82u, 0x81u}, /* 关键任务死锁 L1A8D0P0 BIT34 */
	{FAULT_MEM_ERR, 0x83u, 0x81u}, /* 内存错误 L1A8D0P0 BIT35 */
	{FAULT_FLASH_ERR, 0x84u, 0x81u}, /* FLASH错误 L1A8D0P0 BIT36 */
	{FAULT_PARAM_RANGE, 0x55u, 0x81u}, /* 参数越界 L1A5D0P0 BIT37 */
	{FAULT_HOMING_ERR, 0x16u, 0x81u}, /* 回零错误 L1A1D0P0 BIT38 */
	{FAULT_FW_CRC, 0x87u, 0x81u}, /* 固件校验错误 L1A8D0P0 BIT39 */
	{FAULT_FOLLOW_ERR, 0x18u, 0x81u}, /* 跟随误差过大 L1A1D0P0 BIT40 */
	{FAULT_NOT_CALIB, 0x59u, 0x91u}, /* 未标定运行 L1A5D0P1 BIT41 */
	{FAULT_ALGO_DIVERGE, 0x1Au, 0x81u}, /* 算法发散 L1A1D0P0 BIT42 */
	{FAULT_TRAJ_CLAMP, 0x4Au, 0x42u}, /* 轨迹规划超限 L2A4D0P0 BIT26 */
	{FAULT_MODE_INVALID, 0x7Bu, 0x42u}, /* 运行模式错误 L2A7D0P0 BIT27 */
	{FAULT_UNIT_ERR, 0x7Cu, 0x42u}, /* 单位转换错误 L2A7D0P0 BIT28 */
	{FAULT_TASK_TIMEOUT_SLOW, 0x7Du, 0x42u}, /* 非关键任务超时 L2A7D0P0 BIT29 */
	{FAULT_EEPROM_ERR, 0x7Eu, 0x42u}, /* 参数存储器读写错误 L2A7D0P0 BIT30 */
	{FAULT_CONV_SLOW, 0x7Fu, 0x03u}, /* 算法收敛延迟 L3A7D0P0 BIT15 */
	{FAULT_LOG_FULL, 0x70u, 0x43u}, /* 日志存储满 L3A7D0P0 BIT16 */
	{FAULT_PARAM_DRIFT, 0x71u, 0x43u}, /* 参数漂移 L3A7D0P0 BIT17 */
	{FAULT_BUS_DEAD, 0x2Bu, 0x81u}, /* 总线死锁 L1A2D0P0 BIT43 */
	{FAULT_HOST_LOST, 0x2Cu, 0x81u}, /* 主节点失联 L1A2D0P0 BIT44 */
	{FAULT_BUS_OFF, 0x0Du, 0x81u}, /* 物理层损坏 L1A0D0P0 BIT45 */
	{FAULT_BUS_LOAD_HI, 0x7Fu, 0x42u}, /* 总线过载 L2A7D0P0 BIT31 */
	{FAULT_FRAME_ERR, 0x70u, 0x82u}, /* 帧错误 L2A7D0P0 BIT32 */
	{FAULT_FRAME_LOSS, 0x71u, 0x82u}, /* 帧丢失 L2A7D0P0 BIT33 */
	{FAULT_CRC_RATE, 0x72u, 0x43u}, /* CRC错误率上升 L3A7D0P0 BIT18 */
	{FAULT_ARB_LOSS, 0x73u, 0x43u}, /* 仲裁冲突频发 L3A7D0P0 BIT19 */
	{FAULT_BUS_LOAD_WARN, 0x74u, 0x43u}, /* 总线负载预警 L3A7D0P0 BIT20 */
	{FAULT_HUMIDITY, 0x0Eu, 0x81u}, /* 过湿短路 L1A0D0P0 BIT46 */
	{FAULT_EMC, 0x0Fu, 0x81u}, /* 强电磁干扰 L1A0D0P0 BIT47 */
	{FAULT_PRESSURE, 0x32u, 0x86u}, /* 气压异常 L2A3D1P0 BIT34 */
	{FAULT_ENV_VIB, 0x33u, 0x8Au}, /* 环境过振 L2A3D2P0 BIT35 */
	{FAULT_ENV_HOT, 0x34u, 0x86u}, /* 环境温度过高 L2A3D1P0 BIT36 */
	{FAULT_ENV_COLD, 0x55u, 0x82u}, /* 环境温度过低 L2A5D0P0 BIT37 */
	{FAULT_PRESSURE_WARN, 0x75u, 0x43u}, /* 轻度气压异常 L3A7D0P0 BIT21 */
	{FAULT_ENV_TEMP_WARN, 0x76u, 0x43u}, /* 环境温度预警 L3A7D0P0 BIT22 */
};

/**
 * @brief 故障码 -> 元数据表索引
 * @param code 故障码 0xSLNN
 * @retval 表索引(0~FAULT_CODE_COUNT-1), 未定义码返回 -1
 * @note  表按码值升序, 采用二分查找(109项约7次比较)
 */
int8_t fault_code_to_index(uint16_t code)
{
	int8_t lo = 0;
	int8_t hi = (int8_t)FAULT_CODE_COUNT - 1;

	while (lo <= hi)
	{
		int8_t mid = (int8_t)((lo + hi) / 2);
		uint16_t v = fault_meta_table[mid].code;
		if (v == code)
			return mid;
		if (v < code)
			lo = (int8_t)(mid + 1);
		else
			hi = (int8_t)(mid - 1);
	}
	return -1;
}

/**
 * @brief 取故障码元数据
 * @param code 故障码 0xSLNN
 * @retval 元数据指针, 未定义码返回 NULL
 */
const fault_meta_t *fault_meta_get(uint16_t code)
{
	int8_t idx = fault_code_to_index(code);
	return (idx < 0) ? NULL : &fault_meta_table[idx];
}

/**
 * @brief 故障优先级比较
 * @param f1/f2 故障码
 * @retval 1=f1优先级高, 0=不高于(含相等)
 * @note  先比故障来源(高4位小者优先), 同来源比级别(中4位小者优先)
 */
uint8_t fault_is_higher_priority(uint16_t f1, uint16_t f2)
{
	uint8_t s1 = FAULT_GET_SOURCE(f1);
	uint8_t s2 = FAULT_GET_SOURCE(f2);

	if (s1 != s2)
		return (s1 < s2) ? 1u : 0u;
	return (FAULT_GET_LEVEL(f1) < FAULT_GET_LEVEL(f2)) ? 1u : 0u;
}
