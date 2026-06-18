/**
 * @file        jm_cmd_def.h
 * @brief       关节电机通信命令码定义(忠实转写 joint_motor_command_list.csv)
 *
 * @author      Dalin (dalin@robot.com)
 * @version     1.0
 * @date        2026-06-18
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @par 修改日志:
 * | 日期       | 版本 | 作者   | 修改内容   |
 * |------------|------|--------|------------|
 * | 2026-06-18 | 1.0  | Dalin  | 初始创建   |
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 * @note        CMD 0x00~0xB8 段数值与 state_define.h 的 ctrl_mode_e 一致,
 *              固件可直接把 CMD 当控制模式分发。串口/CAN 共用同一套 CMD。
 */
#ifndef __JM_CMD_DEF_H__
#define __JM_CMD_DEF_H__

#ifdef __cplusplus
extern "C"
{
#endif

	/* ===================== 命令码 CMD ===================== */
	typedef enum
	{
		/* 系统控制 0x00~0x0F */
		JM_CMD_IDLE = 0x00,	   /* 进入待机 */
		JM_CMD_HOLD = 0x01,	   /* 位置保持 */
		JM_CMD_BRAKE = 0x02,   /* 机械刹车 */
		JM_CMD_ESTOP = 0x03,   /* 紧急停止 */
		JM_CMD_ENABLE = 0x04,  /* 上使能 */
		JM_CMD_DISABLE = 0x05, /* 下使能 */
		JM_CMD_STOP = 0x06,	   /* 停止运行 */

		/* 运动控制 0x10~0x2F */
		JM_CMD_OPEN_LOOP = 0x10,		 /* 开环电压 ud,uq */
		JM_CMD_CURRENT = 0x11,			 /* 电流环 id,iq */
		JM_CMD_TORQUE = 0x12,			 /* 力矩环 torque */
		JM_CMD_MIT = 0x13,				 /* MIT pos,vel,kp,kd,tff */
		JM_CMD_VELOCITY = 0x14,			 /* 速度环 vel */
		JM_CMD_POSITION = 0x15,			 /* 位置环 pos */
		JM_CMD_POSITION_VELOCITY = 0x16, /* 位置+速度前馈 */
		JM_CMD_POSITION_TORQUE = 0x17,	 /* 位置+力矩限幅 */
		JM_CMD_VELOCITY_TORQUE = 0x18,	 /* 速度+力矩限幅 */
		JM_CMD_DUTY_CYCLE = 0x19,		 /* 占空比 */
		JM_CMD_VOLTAGE_VECTOR = 0x1A,	 /* 电压矢量 */
		JM_CMD_FIELD_WEAKENING = 0x1B,	 /* 弱磁 */
		JM_CMD_SENSORLESS = 0x1C,		 /* 无感FOC */

		/* 高级力控 0x30~0x4F */
		JM_CMD_IMPEDANCE = 0x30,
		JM_CMD_ADMITTANCE = 0x31,
		JM_CMD_FORCE_CONTROL = 0x32,
		JM_CMD_FORCE_POSITION_HYBRID = 0x33,
		JM_CMD_GRAVITY_COMPENSATION = 0x34,
		JM_CMD_COLLISION_DETECTION = 0x35,
		JM_CMD_ZERO_FORCE = 0x36,
		JM_CMD_CONSTANT_FORCE = 0x37,
		JM_CMD_VARIABLE_IMPEDANCE = 0x38,
		JM_CMD_ADAPTIVE_GRAVITY_COMP = 0x39,
		JM_CMD_LANDING_BUFFER = 0x3A,

		/* 轨迹同步 0x50~0x6F */
		JM_CMD_PVT = 0x50,
		JM_CMD_CUBIC_SPLINE = 0x51,
		JM_CMD_TRAPEZOIDAL_TRAJ = 0x52,
		JM_CMD_S_CURVE_TRAJ = 0x53,
		JM_CMD_HOMING = 0x54,
		JM_CMD_CANOPEN_SYNC = 0x55,
		JM_CMD_ETHERCAT_CSP = 0x56,
		JM_CMD_ETHERCAT_CSV = 0x57,
		JM_CMD_ETHERCAT_CST = 0x58,
		JM_CMD_PP = 0x59,
		JM_CMD_PV = 0x5A,
		JM_CMD_PT = 0x5B,
		JM_CMD_ELECTRONIC_GEAR = 0x5C,
		JM_CMD_ELECTRONIC_CAM = 0x5D,

		/* 特殊应用与测试 0x70~0x8F */
		JM_CMD_STEP_DIR = 0x70,
		JM_CMD_ANALOG_INPUT = 0x71,
		JM_CMD_PWM_INPUT = 0x72,
		JM_CMD_JOG = 0x73,
		JM_CMD_SAFE_TEACH = 0x74,
		JM_CMD_TEST_AGING = 0x75,
		JM_CMD_TEST_SWEEP_FREQ = 0x76,
		JM_CMD_TEST_COGGING = 0x77,
		JM_CMD_TEST_FRICTION = 0x78,
		JM_CMD_TEST_INERTIA = 0x79,
		JM_CMD_TEST_CURRENT_LOOP = 0x7A,
		JM_CMD_TEST_VELOCITY_LOOP = 0x7B,

		/* 校准 0x90~0xAF */
		JM_CMD_CALIB_MOTOR_PARAM = 0x90,
		JM_CMD_CALIB_ENCODER_OFFSET = 0x91,
		JM_CMD_CALIB_ENCODER_LINEARITY = 0x92,
		JM_CMD_CALIB_TORQUE_CONST = 0x93,
		JM_CMD_CALIB_COGGING_COMP = 0x94,
		JM_CMD_CALIB_FRICTION_COMP = 0x95,
		JM_CMD_CALIB_INERTIA = 0x96,
		JM_CMD_CALIB_ADC_OFFSET = 0x97,
		JM_CMD_CALIB_ADC_GAIN = 0x98,
		JM_CMD_CALIB_CURRENT_SENSOR = 0x99,
		JM_CMD_CALIB_TEMPERATURE = 0x9A,
		JM_CMD_CALIB_FULL_AUTO = 0x9B,

		/* 系统诊断 0xB0~0xBF */
		JM_CMD_CLEAR_FAULT = 0xB0,
		JM_CMD_DIAGNOSTIC = 0xB1,
		JM_CMD_ENTER_BOOTLOADER = 0xB2,
		JM_CMD_SAVE_CONFIG = 0xB3,
		JM_CMD_FACTORY_RESET = 0xB4,
		JM_CMD_START_LOG = 0xB5,
		JM_CMD_STOP_LOG = 0xB6,
		JM_CMD_HIGH_SPEED_DAQ = 0xB7,
		JM_CMD_SINGLE_STEP = 0xB8,

		/* 反馈查询 0xC0~0xCF */
		JM_CMD_READ_FEEDBACK = 0xC0, /* 主实时反馈 */
		JM_CMD_READ_STATE = 0xC1,	 /* 电机状态机 */
		JM_CMD_READ_PHASE_CURRENT = 0xC2,
		JM_CMD_READ_DQ_CURRENT = 0xC3,
		JM_CMD_READ_BUS = 0xC4,
		JM_CMD_READ_TEMPERATURE = 0xC5,
		JM_CMD_READ_POS_VEL = 0xC6,
		JM_CMD_READ_MULTITURN = 0xC7,
		JM_CMD_READ_FAULT = 0xC8,

		/* 设备信息 0xD0~0xDF */
		JM_CMD_READ_DEV_INFO = 0xD0,
		JM_CMD_READ_DEV_NAME = 0xD1,
		JM_CMD_HEARTBEAT = 0xD2,

		/* 参数读写 0xE0~0xEF */
		JM_CMD_PARAM_READ = 0xE0,  /* 读单个参数 */
		JM_CMD_PARAM_WRITE = 0xE1, /* 写单个参数 */
		JM_CMD_PARAM_READ_BULK = 0xE2,
		JM_CMD_PARAM_WRITE_BULK = 0xE3,
		JM_CMD_PARAM_SAVE = 0xE4,
		JM_CMD_PARAM_RESET = 0xE5,

		/* CAN管理与通用 0xF0~0xFF */
		JM_CMD_SET_CAN_ID = 0xF0,
		JM_CMD_SET_BAUDRATE = 0xF1,
		JM_CMD_BROADCAST_SYNC = 0xF2,
		JM_CMD_NACK = 0xFE, /* 错误应答 */
	} jm_cmd_e;

	/* ===================== 错误码(NACK 的 err_code) ===================== */
	typedef enum
	{
		JM_ERR_OK = 0x00,			/* 成功(用ACK,不发NACK) */
		JM_ERR_UNSUPPORTED = 0x01,	/* CMD不支持 */
		JM_ERR_OUT_OF_RANGE = 0x02, /* 参数越界 */
		JM_ERR_STATE_DENY = 0x03,	/* 状态不允许 */
		JM_ERR_BAD_PARAM_ID = 0x04, /* param_id无效 */
		JM_ERR_CRC = 0x05,			/* 校验错误 */
		JM_ERR_LENGTH = 0x06,		/* 长度错误 */
		JM_ERR_READ_ONLY = 0x07,	/* 只读参数不可写 */
		JM_ERR_FLASH = 0x08,		/* Flash读写失败 */
		JM_ERR_FAULT_STATE = 0x09,	/* 故障态需先清障 */
		JM_ERR_CALIB_BUSY = 0x0A,	/* 校准未完成/校准中 */
	} jm_err_e;

	/* ===================== 参数类型码(0xE0读应答的 type 字段) ===================== */
	typedef enum
	{
		JM_PT_U8 = 0,
		JM_PT_I8 = 1,
		JM_PT_U16 = 2,
		JM_PT_I16 = 3,
		JM_PT_U32 = 4,
		JM_PT_I32 = 5,
		JM_PT_F32 = 6,
		JM_PT_STR = 7, /* char[] */
	} jm_param_type_e;

	/* CAN 仲裁ID编解码: ID = (CMD<<8) | 电机ID */
#define JM_CAN_MAKE_ID(cmd, motor_id) (((uint32_t)(cmd) << 8) | ((motor_id) & 0xFF))
#define JM_CAN_GET_CMD(id) ((uint8_t)(((id) >> 8) & 0xFF))
#define JM_CAN_GET_MOTOR_ID(id) ((uint8_t)((id) & 0xFF))
#define JM_CAN_BROADCAST_ID 0x00 /* 电机ID=0 为广播地址 */

	/* Bootloader/恢复出厂魔数(防误触) */
#define JM_MAGIC_BOOTLOADER 0xB00710ADu
#define JM_MAGIC_FACTORY_RESET 0xFAC70F5Fu

#ifdef __cplusplus
}
#endif
#endif /* __JM_CMD_DEF_H__ */
