/**
 * @file        jm_cmd_def.h
 * @brief       关节电机通信命令码定义(忠实转写 joint_motor_command_list.csv)
 *
 * @author      Dalin (dalinyy@163.com)
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
		JM_CMD_IDLE = 0x00,    /* 进入待机 */
		JM_CMD_HOLD = 0x01,    /* 位置保持 */
		JM_CMD_BRAKE = 0x02,   /* 机械刹车 */
		JM_CMD_ESTOP = 0x03,   /* 紧急停止 */
		JM_CMD_ENABLE = 0x04,  /* 上使能 */
		JM_CMD_DISABLE = 0x05, /* 下使能 */
		JM_CMD_STOP = 0x06,    /* 停止运行 */

		/* 运动控制 0x10~0x2F */
		JM_CMD_OPEN_LOOP = 0x10,         /* 开环电压 ud,uq */
		JM_CMD_CURRENT = 0x11,           /* 电流环 id,iq */
		JM_CMD_TORQUE = 0x12,            /* 力矩环 torque */
		JM_CMD_MIT = 0x13,               /* MIT pos,vel,kp,kd,tff */
		JM_CMD_VELOCITY = 0x14,          /* 速度环 vel */
		JM_CMD_POSITION = 0x15,          /* 位置环 pos */
		JM_CMD_POSITION_VELOCITY = 0x16, /* 位置+速度前馈 */
		JM_CMD_POSITION_TORQUE = 0x17,   /* 位置+力矩限幅 */
		JM_CMD_VELOCITY_TORQUE = 0x18,   /* 速度+力矩限幅 */
		JM_CMD_DUTY_CYCLE = 0x19,        /* 占空比 */
		JM_CMD_VOLTAGE_VECTOR = 0x1A,    /* 电压矢量 */
		JM_CMD_FIELD_WEAKENING = 0x1B,   /* 弱磁 */
		JM_CMD_SENSORLESS = 0x1C,        /* 无感FOC */

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

		/* 校准 0x90~0xAF: 类别命令+子命令模式
	 * 0x90-0x96: payload[0]=子模式ID, 进入CALIB态并启动标定
	 * 0x97: 进度查询, 返回 8 字节详细状态 ACK (state/fail_reason/progress/level/submode/step/step_total/reserved)
	 * 0x98: 中止标定, ACK */
		JM_CMD_CALIB_LEVEL1 = 0x90, /* L1 驱动硬件底层 */
		JM_CMD_CALIB_LEVEL2 = 0x91, /* L2 电机电气身份 */
		JM_CMD_CALIB_LEVEL3 = 0x92, /* L3 编码器校准 */
		JM_CMD_CALIB_LEVEL4 = 0x93, /* L4 转矩基础 */
		JM_CMD_CALIB_LEVEL5 = 0x94, /* L5 非线性补偿 */
		JM_CMD_CALIB_LEVEL6 = 0x95, /* L6 负载系统级 */
		JM_CMD_CALIB_LEVEL7 = 0x96, /* L7 自动化集成 */
		JM_CMD_CALIB_QUERY = 0x97,  /* 进度查询 */
		JM_CMD_CALIB_ABORT = 0x98,  /* 中止标定 */

		/* PID 管理 0x9A~0x9B: 三环独立参数来源管理
		 * 0x9A: 触发理论估计(零极点对消法)并自动设 source=2, 仅IDLE态
		 * 0x9B: 独立切换某环 source, 仅IDLE态 */
		JM_CMD_PID_AUTOTUNE   = 0x9A, /* PID 理论估计 */
		JM_CMD_PID_SOURCE_SET = 0x9B, /* PID 来源切换 */

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
		JM_CMD_READ_STATE = 0xC1,    /* 电机状态机 */
		JM_CMD_READ_PHASE_CURRENT = 0xC2,
		JM_CMD_READ_DQ_CURRENT = 0xC3,
		JM_CMD_READ_BUS = 0xC4,
		JM_CMD_READ_TEMPERATURE = 0xC5,
		JM_CMD_READ_POS_VEL = 0xC6,
		JM_CMD_READ_MULTITURN = 0xC7,
		JM_CMD_READ_FAULT = 0xC8,
		JM_CMD_READ_DEBUG = 0xC9,    /* 通用调试通道: float[] 任意挂载量, 免改协议加观测点 */
		JM_CMD_TELEMETRY = 0xCA,     /* 周期遥测帧(下位机->上位机, 无应答): mask(u16) + 按位序拼接所选数据组 */
		JM_CMD_SET_TELEMETRY = 0xCB, /* 遥控开关(上位机->下位机): enable(u8)+mask(u16)[+period_ms(u16)], 回单次ACK */

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
		/* 电机配置(motor_info)读写 0xE6~0xE8: 独立于0xE0-0xE5的运行时参数,
		 * 面向Flash/EEPROM持久化的硬件配置/校准数据。固定4字节值传输,固件按字段类型转换。*/
		JM_CMD_MOTOR_INFO_READ = 0xE6,       /* 读单个电机配置 */
		JM_CMD_MOTOR_INFO_WRITE = 0xE7,      /* 写单个电机配置(RAM, 需0xEA固化) */
		JM_CMD_MOTOR_INFO_READ_BULK = 0xE8,  /* 批量读(块内连续ID, 固定4B/值) */
		JM_CMD_MOTOR_INFO_WRITE_BULK = 0xE9, /* 批量写(块内连续ID, 固定4B/值) */
		JM_CMD_MOTOR_INFO_SAVE = 0xEA,       /* 把motor_info整块写入Flash */
		JM_CMD_MOTOR_INFO_RESET = 0xEB,      /* 恢复默认(param_id=0xFFFF全部) */

		/* CAN管理与通用 0xF0~0xFF */
		JM_CMD_SET_CAN_ID = 0xF0,
		JM_CMD_SET_BAUDRATE = 0xF1,
		JM_CMD_BROADCAST_SYNC = 0xF2,
		JM_CMD_NACK = 0xFE, /* 错误应答 */
	} jm_cmd_e;

	/* ===================== 同步遥测分组位掩码(0xCA/0xCB 共用) =====================
	 * 上位机用 SET_TELEMETRY(0xCB) 选择订阅哪些组; 下位机把所选组在同一拍打包成
	 * 单帧 TELEMETRY(0xCA) 上传。固件打包与上位机解析须按【位序由低到高】拼接,
	 * 帧内自带 mask, 故增删订阅项时解析器无需改动。
	 * 新增一组: 在此追加一个 bit, 固件 pack 端按位序补一段, 上位机解析端按位序补一段。*/
	typedef enum
	{
		JM_TLM_POS_VEL = (1u << 0),   /* pos(f32),vel(f32)            8B */
		JM_TLM_DQ = (1u << 1),        /* id(f32),iq(f32)              8B */
		JM_TLM_PHASE = (1u << 2),     /* ia,ib,ic(f32)               12B */
		JM_TLM_BUS = (1u << 3),       /* vbus,ibus,power(f32)        12B */
		JM_TLM_TEMP = (1u << 4),      /* tempFet,tempMotor(f32)       8B */
		JM_TLM_MULTITURN = (1u << 5), /* multiturn(u32),single(f32)   8B */
		JM_TLM_TORQUE = (1u << 6),    /* torque(f32)                  4B */
		JM_TLM_FAULT = (1u << 7),     /* fault(u32),warn(u32)         8B */
		JM_TLM_STATE = (1u << 8),     /* topFsm,runState,ctrlMode,enable(u8) 4B */
		JM_TLM_DEBUG = (1u << 9),     /* jm_dbg[JM_DBG_CH](f32)   N*4B */
	} jm_telemetry_bit_e;

	/* ===================== 错误码(NACK 的 err_code) ===================== */
	typedef enum
	{
		JM_ERR_OK = 0x00,           /* 成功(用ACK,不发NACK) */
		JM_ERR_UNSUPPORTED = 0x01,  /* CMD不支持 */
		JM_ERR_OUT_OF_RANGE = 0x02, /* 参数越界 */
		JM_ERR_STATE_DENY = 0x03,   /* 状态不允许 */
		JM_ERR_BAD_PARAM_ID = 0x04, /* param_id无效 */
		JM_ERR_CRC = 0x05,          /* 校验错误 */
		JM_ERR_LENGTH = 0x06,       /* 长度错误 */
		JM_ERR_READ_ONLY = 0x07,    /* 只读参数不可写 */
		JM_ERR_FLASH = 0x08,        /* Flash读写失败 */
		JM_ERR_FAULT_STATE = 0x09,  /* 故障态需先清障 */
		JM_ERR_CALIB_BUSY = 0x0A,   /* 校准未完成/校准中 */
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
#define JM_CAN_GET_CMD(id)            ((uint8_t)(((id) >> 8) & 0xFF))
#define JM_CAN_GET_MOTOR_ID(id)       ((uint8_t)((id) & 0xFF))
#define JM_CAN_BROADCAST_ID           0x00 /* 电机ID=0 为广播地址 */

										   /* Bootloader/恢复出厂魔数(防误触) */
#define JM_MAGIC_BOOTLOADER    0xB00710ADu
#define JM_MAGIC_FACTORY_RESET 0xFAC70F5Fu

#ifdef __cplusplus
}
#endif
#endif /* __JM_CMD_DEF_H__ */
