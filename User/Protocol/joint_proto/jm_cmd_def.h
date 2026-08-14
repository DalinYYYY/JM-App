/**
 * @file        jm_cmd_def.h
 * @brief       关节电机通信命令码定义(忠实转写 joint_motor_command_list.csv)
 *
 * @author      Dalin (dalinyy@163.com)
 * @version     1.1
 * @date        2026-06-18
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @par 修改日志:
 * | 日期       | 版本 | 作者   | 修改内容   |
 * |------------|------|--------|------------|
 * | 2026-06-18 | 1.0  | Dalin  | 初始创建 |
 * | 2026-07-27 | 1.1  | Dalin  | 协议版本号 + 命令码保留区 + 预留 0x80/0x81/0x82 同步触发, 0xCC~0xCF OTA |
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 * @note        CMD 0x00~0xB8 段数值与 state_define.h 的 ctrl_mode_e 一致,
 *              固件可直接把 CMD 当控制模式分发。串口/CAN 共用同一套 CMD。
 * @note        命令码保留区分组（编码前固化）:
 *              0x00~0x0F 系统控制 | 0x10~0x2F 运动控制 | 0x30~0x4F 高级力控
 *              0x50~0x6F 轨迹同步 | 0x70~0x7F 特殊应用与测试 | 0x80~0x8F 多电机同步(预留)
 *              0x90~0xAF 校准+PID | 0xB0~0xBF 系统诊断 | 0xC0~0xCF 反馈查询+OTA预留
 *              0xD0~0xDF 设备信息 | 0xE0~0xEF 参数读写 | 0xF0~0xFE CAN管理 | 0xFF 保留
 */
#ifndef __JM_CMD_DEF_H__
#define __JM_CMD_DEF_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

	/* ===================== 协议版本号 ===================== */
	/* 主版本: 不兼容变更(命令码重排/载荷语义改); 次版本: 兼容追加(新命令/新字段);
	 * 补丁: bug 修复。0xD0 READ_DEV_INFO 应答(FD 模式)携带此版本号。 */
#define JM_PROTO_VERSION_MAJOR 1
#define JM_PROTO_VERSION_MINOR 2 /* 1.2: LIVE/TRACE 双数据流, TRACE 主动批量上报 */
#define JM_PROTO_VERSION_PATCH 0
#define JM_PROTO_VERSION       ((uint16_t)(((JM_PROTO_VERSION_MAJOR) << 8) | (JM_PROTO_VERSION_MINOR)))

	/* ===================== feature_flags 位定义 =====================
	 * 0xD0 READ_DEV_INFO 应答(28B 扩展格式)携带的特性位图(u16), 上位机据此自适应。
	 * bit0=CAN_FD 支持 | bit1=AUTH 鉴权启用 | bit2=AUTOTUNE 自整定 | bit3=BODE_SWEEP 扫频
	 * bit4=DUAL_CHANNEL_ARB 双通道仲裁 | bit5=CAN_LOSS_TIMER 通信中断降级 | bit6-7=reserved */
#define JM_FEAT_CAN_FD         (1u << 0)
#define JM_FEAT_AUTH           (1u << 1)
#define JM_FEAT_AUTOTUNE       (1u << 2)
#define JM_FEAT_BODE_SWEEP     (1u << 3)
#define JM_FEAT_DUAL_ARB       (1u << 4)
#define JM_FEAT_CAN_LOSS_TIMER (1u << 5)

#if defined(USE_CAN_FD_MODE) && (USE_CAN_FD_MODE == 1)
#define JM_FEATURE_FLAGS_LO (JM_FEAT_CAN_FD | JM_FEAT_AUTOTUNE | JM_FEAT_BODE_SWEEP | JM_FEAT_CAN_LOSS_TIMER)
#else
#define JM_FEATURE_FLAGS_LO (JM_FEAT_AUTOTUNE | JM_FEAT_BODE_SWEEP | JM_FEAT_CAN_LOSS_TIMER)
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

		/* 特殊应用与测试 0x70~0x7F */
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

		/* 多电机同步触发预留 0x80~0x8F (仅定义命令码, 不实现 handler, 回 NACK(NOT_SUPPORTED))
		 *   0x80 SYNC: CANopen-style 周期同步帧
		 *   0x81 PRESET_AND_TRIGGER: 预存指令不执行
		 *   0x82 TRIGGER: 广播触发同步执行
		 * 注: 0x12/0x13 已被 TORQUE/MIT 占用, 故同步触发改到 0x80 段预留 */
		JM_CMD_SYNC = 0x80,               /* 预留: 周期同步帧 */
		JM_CMD_PRESET_AND_TRIGGER = 0x81, /* 预留: 预存指令不执行 */
		JM_CMD_TRIGGER = 0x82,            /* 预留: 广播触发同步执行 */

		/* 校准 0x90~0xAF: 类别命令+子命令模式*/
		JM_CMD_CALIB_LEVEL1 = 0x90, /* L1 驱动硬件底层 */
		JM_CMD_CALIB_LEVEL2 = 0x91, /* L2 电机电气身份 */
		JM_CMD_CALIB_LEVEL3 = 0x92, /* L3 编码器校准 */
		JM_CMD_CALIB_LEVEL4 = 0x93, /* L4 转矩基础 */
		JM_CMD_CALIB_LEVEL5 = 0x94, /* L5 非线性补偿 */
		JM_CMD_CALIB_LEVEL6 = 0x95, /* L6 负载系统级 */
		JM_CMD_CALIB_LEVEL7 = 0x96, /* L7 自动化集成 */
		JM_CMD_CALIB_QUERY = 0x97,  /* 进度查询 */
		JM_CMD_CALIB_ABORT = 0x98,  /* 中止标定 */

		/* PID 管理 0xA0~0xAF: 三环独立参数来源管理 + 实时调试*/
		JM_CMD_PID_AUTOTUNE = 0xA0,   /* PID 理论估计 */
		JM_CMD_PID_SOURCE_SET = 0xA1, /* PID 来源切换 */
		JM_CMD_PID_SOURCE_GET = 0xA2, /* 读 PID 来源状态 */
		JM_CMD_PID_PARAM_SET = 0xA5,  /* 实时写PID参数 */
		JM_CMD_PID_PARAM_GET = 0xA6,  /* 实时读PID参数 */

		/* 系统诊断 0xB0~0xBF */
		JM_CMD_CLEAR_FAULT = 0xB0,
		JM_CMD_DIAGNOSTIC = 0xB1,
		JM_CMD_ENTER_BOOTLOADER = 0xB2,
		JM_CMD_SAVE_CONFIG = 0xB3,
		JM_CMD_FACTORY_RESET = 0xB4,
		/* B5~B7 为旧版一次性采样命令, 已废弃且不可复用 */
		JM_CMD_START_LOG = 0xB5,
		JM_CMD_STOP_LOG = 0xB6,
		JM_CMD_HIGH_SPEED_DAQ = 0xB7,
		JM_CMD_SINGLE_STEP = 0xB8,
		JM_CMD_TRACE_CONFIG = 0xB9, /* Host->Motor: TRACE配置/停止 */
		JM_CMD_TRACE_DATA = 0xBA,   /* Motor->Host: TRACE批量数据, 无应答 */

		/* 反馈查询 0xC0~0xCB (已用), 0xCC~0xCF OTA 预留 (仅定义不实现) */
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
		/* OTA 预留: 仅定义命令码, 不实现 handler, 回 NACK(NOT_SUPPORTED)
		 * 注: 0xB0~0xB3 已被系统诊断占用, 故 OTA 改到 0xCC~0xCF 预留 */
		JM_CMD_OTA_START = 0xCC,  /* 预留: 固件升级启动 */
		JM_CMD_OTA_DATA = 0xCD,   /* 预留: 固件数据分块 */
		JM_CMD_OTA_END = 0xCE,    /* 预留: 固件升级结束 */
		JM_CMD_OTA_RESUME = 0xCF, /* 预留: 断点续传查询 */

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
		/* 电机配置(motor_info)读写 0xE6~0xE8: 独立于0xE0-0xE5的运行时参数*/
		JM_CMD_MOTOR_INFO_READ = 0xE6,          /* 读单个电机配置 */
		JM_CMD_MOTOR_INFO_WRITE = 0xE7,         /* 写单个电机配置(RAM, 需0xEA固化) */
		JM_CMD_MOTOR_INFO_READ_BULK = 0xE8,     /* 批量读(块内连续ID, 固定4B/值) */
		JM_CMD_MOTOR_INFO_WRITE_BULK = 0xE9,    /* 批量写(块内连续ID, 固定4B/值) */
		JM_CMD_MOTOR_INFO_SAVE = 0xEA,          /* 把motor_info整块写入Flash */
		JM_CMD_MOTOR_INFO_RESET = 0xEB,         /* 恢复默认(param_id=0xFFFF全部) */
		JM_CMD_MOTOR_INFO_RECALIB_RESET = 0xEC, /* 清除标定状态以便重新标定(保留电机本体参数) */

		/* CAN管理与通用 0xF0~0xFF */
		JM_CMD_SET_CAN_ID = 0xF0,
		JM_CMD_SET_BAUDRATE = 0xF1,
		JM_CMD_BROADCAST_SYNC = 0xF2,
		JM_CMD_SET_FD_MODE = 0xF3,     /* CAN FD模式切换: enable(u8) → ACK(enable+cap) */
		JM_CMD_CAN_DI_DISCOVER = 0xF4, /* CAN-DI广播发现: 单帧时隙响应 */
		JM_CMD_CAN_DI_SET_ID = 0xF5,   /* 按CAN-DI广播定向设置节点ID */
		JM_CMD_CAN_DI_IDENTIFY = 0xF6, /* 按CAN-DI触发物理设备指示 */
		JM_CMD_NACK = 0xFE,            /* 错误应答 */
	} jm_cmd_e;

	/* ===================== 同步遥测分组位掩码(0xCA/0xCB 共用) =====================
	 * 上位机用 SET_TELEMETRY(0xCB) 选择订阅哪些组; 下位机把所选组在同一拍打包成
	 * 单帧 TELEMETRY(0xCA) 上传。固件打包与上位机解析须按【位序由低到高】拼接,
	 * 帧内自带 mask, 故增删订阅项时解析器无需改动。
	 * 新增一组: 在此追加一个 bit, 固件 pack 端按位序补一段, 上位机解析端按位序补一段。*/
	typedef enum
	{
		JM_TLM_POS_VEL = (1u << 0),         /* pos(f32),vel(f32)            8B */
		JM_TLM_DQ = (1u << 1),              /* id(f32),iq(f32)              8B */
		JM_TLM_PHASE = (1u << 2),           /* ia,ib,ic(f32)               12B */
		JM_TLM_BUS = (1u << 3),             /* vbus,ibus,power(f32)        12B */
		JM_TLM_TEMP = (1u << 4),            /* tempFet,tempMotor(f32)       8B */
		JM_TLM_MULTITURN = (1u << 5),       /* multiturn(u32),single(f32)   8B */
		JM_TLM_TORQUE = (1u << 6),          /* torque(f32)                  4B */
		JM_TLM_FAULT = (1u << 7),           /* fault(u32),warn(u32)         8B */
		JM_TLM_STATE = (1u << 8),           /* topFsm,runState,ctrlMode,enable(u8) 4B */
		JM_TLM_DEBUG = (1u << 9),           /* jm_dbg[JM_DBG_CH](f32), 变长组始终最后 */
		JM_TLM_CURRENT_TARGET = (1u << 10), /* idRef,iqRef(f32)        8B */
		JM_TLM_MOTION_TARGET = (1u << 11),  /* velRef,posRef(f32)      8B */
	} jm_telemetry_bit_e;

	/* ===================== 错误码(NACK 的 err_code) =====================
	 * NACK 载荷格式: [0xFE][orig_cmd][err_code][seq] (4B)
	 *   - orig_cmd: 触发 NACK 的原命令码
	 *   - err_code: 见下表
	 *   - seq: 异步命令的序列号, 同步命令填 0
	 * 异步命令双时序协议:
	 *   - 即时 NACK: 收到命令立即校验失败时返回 (err_code + seq=0)
	 *   - 最终 ACK/NACK: 异步任务完成后返回 (err_code=OK/具体错误 + seq=递增)
	 * 0x01 UNSUPPORTED 与 0x13 NOT_SUPPORTED 的区别:
	 *   - 0x01: 命令码区间不识别(老固件收到新命令, 整个 CMD 段未实现)
	 *   - 0x13: 命令码已定义但当前固件未实现(预留命令如 OTA/SYNC, 有意留空) */
	typedef enum
	{
		JM_ERR_OK = 0x00,           /* 成功(用ACK,不发NACK) */
		JM_ERR_UNSUPPORTED = 0x01,  /* CMD不支持(命令码区间不识别) */
		JM_ERR_OUT_OF_RANGE = 0x02, /* 参数越界 */
		JM_ERR_STATE_DENY = 0x03,   /* 状态不允许 */
		JM_ERR_BAD_PARAM_ID = 0x04, /* param_id无效 */
		JM_ERR_CRC = 0x05,          /* 校验错误 */
		JM_ERR_LENGTH = 0x06,       /* 长度错误 */
		JM_ERR_READ_ONLY = 0x07,    /* 只读参数不可写 */
		JM_ERR_FLASH = 0x08,        /* Flash读写失败(通用) */
		JM_ERR_FAULT_STATE = 0x09,  /* 故障态需先清障 */
		JM_ERR_CALIB_BUSY = 0x0A,   /* 校准未完成/校准中 */
		/* 新增 0x0B~0x0F */
		JM_ERR_PENDING = 0x0B,      /* 异步已排队, 即时 NACK 携带 seq */
		JM_ERR_BUSY = 0x0C,         /* 忙(双通道主控被占用) */
		JM_ERR_UNAUTHORIZED = 0x0D, /* 鉴权失败(令牌不匹配) */
		JM_ERR_RATE_LIMIT = 0x0E,   /* 速率限制(命令频率超限) */
		JM_ERR_NOT_FOUND = 0x0F,    /* 未找到(资源/文件/记录不存在) */
		/* 新增 0x10~0x12 (Flash 详细错误码) */
		JM_ERR_FLASH_ERASE = 0x10,  /* Flash 擦除失败 */
		JM_ERR_FLASH_WRITE = 0x11,  /* Flash 写入失败 */
		JM_ERR_FLASH_VERIFY = 0x12, /* Flash 校验失败(读回不匹配) */
		/* 新增 0x13 (预留命令专用) */
		JM_ERR_NOT_SUPPORTED = 0x13, /* 命令码已定义但当前固件未实现(预留命令如 OTA/SYNC) */
		JM_ERR_FLASH_LIMIT = 0x14,   /* Flash 写入次数超限(磨损均衡寿命保护) */
		JM_ERR_EEPROM_WRITE = 0x15,  /* EEPROM 写入失败 */
		JM_ERR_EEPROM_VERIFY = 0x16, /* EEPROM 回读校验失败 */
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
#define JM_CAN_MAKE_ID(cmd, motor_id)       (((uint32_t)(cmd) << 8) | ((motor_id) & 0xFF))
#define JM_CAN_MULTI_FLAG                   (1u << 16)
#define JM_CAN_MAKE_MULTI_ID(cmd, motor_id) (JM_CAN_MAKE_ID((cmd), (motor_id)) | JM_CAN_MULTI_FLAG)
#define JM_CAN_IS_MULTI_ID(id)              (((id) & JM_CAN_MULTI_FLAG) != 0u)
#define JM_CAN_GET_CMD(id)                  ((uint8_t)(((id) >> 8) & 0xFF))
#define JM_CAN_GET_MOTOR_ID(id)             ((uint8_t)((id) & 0xFF))
#define JM_CAN_BROADCAST_ID                 0x00 /* 电机ID=0 为广播地址 */

												 /* Bootloader/恢复出厂魔数(防误触) */
#define JM_MAGIC_BOOTLOADER    0xB00710ADu
#define JM_MAGIC_FACTORY_RESET 0xFAC70F5Fu

#ifdef __cplusplus
}
#endif
#endif /* __JM_CMD_DEF_H__ */
