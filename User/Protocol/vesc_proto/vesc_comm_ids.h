/**
 * @file    vesc_comm_ids.h
 * @brief   完整 VESC 通信命令 ID 枚举（COMM_PACKET_ID，0~159）。
 * @details 与 VESC 固件 datatypes.h 的 COMM_PACKET_ID 完全对齐。
 *          单独成文件，便于上位机/从机共享同一份命令定义。
 */
#ifndef VESC_COMM_IDS_H_
#define VESC_COMM_IDS_H_

#ifdef __cplusplus
extern "C"
{
#endif

	/** @brief VESC 通信命令 ID。 */
	typedef enum
	{
		/* --- 固件 / 引导加载 --- */
		COMM_FW_VERSION = 0,		 /**< 查询固件版本与硬件信息 */
		COMM_JUMP_TO_BOOTLOADER = 1, /**< 跳转到引导加载程序 */
		COMM_ERASE_NEW_APP = 2,		 /**< 擦除新固件存储区 */
		COMM_WRITE_NEW_APP_DATA = 3, /**< 写入新固件数据 */

		/* --- 实时值与电机控制 --- */
		COMM_GET_VALUES = 4,		/**< 读取实时运行数据 */
		COMM_SET_DUTY = 5,			/**< 设置占空比 */
		COMM_SET_CURRENT = 6,		/**< 设置电机电流 */
		COMM_SET_CURRENT_BRAKE = 7, /**< 设置制动电流 */
		COMM_SET_RPM = 8,			/**< 设置目标转速（ERPM） */
		COMM_SET_POS = 9,			/**< 设置目标位置 */
		COMM_SET_HANDBRAKE = 10,	/**< 设置手刹电流 */
		COMM_SET_DETECT = 11,		/**< 设置转子位置检测模式 */
		COMM_SET_SERVO_POS = 12,	/**< 设置舵机输出位置 */

		/* --- 电机 / 应用配置 --- */
		COMM_SET_MCCONF = 13,		   /**< 写入电机配置 */
		COMM_GET_MCCONF = 14,		   /**< 读取电机配置 */
		COMM_GET_MCCONF_DEFAULT = 15,  /**< 读取电机配置默认值 */
		COMM_SET_APPCONF = 16,		   /**< 写入应用配置 */
		COMM_GET_APPCONF = 17,		   /**< 读取应用配置 */
		COMM_GET_APPCONF_DEFAULT = 18, /**< 读取应用配置默认值 */

		/* --- 采样 / 终端 / 调试 --- */
		COMM_SAMPLE_PRINT = 19,		 /**< 采样并回传波形数据 */
		COMM_TERMINAL_CMD = 20,		 /**< 发送终端命令（异步输出） */
		COMM_PRINT = 21,			 /**< 设备文本输出（设备→上位机） */
		COMM_ROTOR_POSITION = 22,	 /**< 回传转子位置 */
		COMM_EXPERIMENT_SAMPLE = 23, /**< 实验采样数据 */

		/* --- 参数自动检测 --- */
		COMM_DETECT_MOTOR_PARAM = 24,		 /**< 检测电机参数（BLDC） */
		COMM_DETECT_MOTOR_R_L = 25,			 /**< 检测电机电阻/电感 */
		COMM_DETECT_MOTOR_FLUX_LINKAGE = 26, /**< 检测电机磁链 */
		COMM_DETECT_ENCODER = 27,			 /**< 检测编码器 */
		COMM_DETECT_HALL_FOC = 28,			 /**< 检测霍尔传感器（FOC） */

		/* --- 系统 / 心跳 --- */
		COMM_REBOOT = 29, /**< 重启设备 */
		COMM_ALIVE = 30,  /**< 心跳，维持控制不超时 */

		/* --- 遥控输入解码 --- */
		COMM_GET_DECODED_PPM = 31,	/**< 读取解码后的 PPM 输入 */
		COMM_GET_DECODED_ADC = 32,	/**< 读取解码后的 ADC 输入 */
		COMM_GET_DECODED_CHUK = 33, /**< 读取解码后的 Nunchuk 输入 */

		/* --- CAN 转发 / 自定义 --- */
		COMM_FORWARD_CAN = 34,		 /**< 经本机 CAN 转发命令给从机 */
		COMM_SET_CHUCK_DATA = 35,	 /**< 写入 Nunchuk 控制数据 */
		COMM_CUSTOM_APP_DATA = 36,	 /**< 自定义应用透传数据 */
		COMM_NRF_START_PAIRING = 37, /**< 启动 NRF 配对 */

		/* --- GPD 通用脉冲驱动 --- */
		COMM_GPD_SET_FSW = 38,				/**< 设置 GPD 开关频率 */
		COMM_GPD_BUFFER_NOTIFY = 39,		/**< GPD 缓冲就绪通知 */
		COMM_GPD_BUFFER_SIZE_LEFT = 40,		/**< 查询 GPD 缓冲剩余空间 */
		COMM_GPD_FILL_BUFFER = 41,			/**< 填充 GPD 缓冲（float） */
		COMM_GPD_OUTPUT_SAMPLE = 42,		/**< 输出单个 GPD 采样 */
		COMM_GPD_SET_MODE = 43,				/**< 设置 GPD 工作模式 */
		COMM_GPD_FILL_BUFFER_INT8 = 44,		/**< 填充 GPD 缓冲（int8） */
		COMM_GPD_FILL_BUFFER_INT16 = 45,	/**< 填充 GPD 缓冲（int16） */
		COMM_GPD_SET_BUFFER_INT_SCALE = 46, /**< 设置 GPD 整数缓冲缩放 */

		/* --- 整车实时值 / 临时配置 / 选择性读取 --- */
		COMM_GET_VALUES_SETUP = 47,			  /**< 读取整车实时值 */
		COMM_SET_MCCONF_TEMP = 48,			  /**< 设置临时电机配置 */
		COMM_SET_MCCONF_TEMP_SETUP = 49,	  /**< 设置整车临时电机配置 */
		COMM_GET_VALUES_SELECTIVE = 50,		  /**< 按掩码选择性读取实时值 */
		COMM_GET_VALUES_SETUP_SELECTIVE = 51, /**< 按掩码选择性读取整车实时值 */

		/* --- 外部 NRF ESB 无线 --- */
		COMM_EXT_NRF_PRESENT = 52,		   /**< 外部 NRF 存在通知 */
		COMM_EXT_NRF_ESB_SET_CH_ADDR = 53, /**< 设置 ESB 信道/地址 */
		COMM_EXT_NRF_ESB_SEND_DATA = 54,   /**< ESB 发送数据 */
		COMM_EXT_NRF_ESB_RX_DATA = 55,	   /**< ESB 接收数据 */
		COMM_EXT_NRF_SET_ENABLED = 56,	   /**< 使能/禁用外部 NRF */

		/* --- 磁链开环检测 / FOC 全自动 --- */
		COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP = 57, /**< 开环检测电机磁链 */
		COMM_DETECT_APPLY_ALL_FOC = 58,				  /**< 全自动检测并应用 FOC 参数 */

		/* --- 全 CAN 广播操作 --- */
		COMM_JUMP_TO_BOOTLOADER_ALL_CAN = 59, /**< 全 CAN 跳转引导加载 */
		COMM_ERASE_NEW_APP_ALL_CAN = 60,	  /**< 全 CAN 擦除新固件区 */
		COMM_WRITE_NEW_APP_DATA_ALL_CAN = 61, /**< 全 CAN 写入新固件数据 */
		COMM_PING_CAN = 62,					  /**< 扫描 CAN 总线上的设备 */
		COMM_APP_DISABLE_OUTPUT = 63,		  /**< 临时禁用电机输出 */

		COMM_TERMINAL_CMD_SYNC = 64, /**< 同步终端命令（等待结果） */
		COMM_GET_IMU_DATA = 65,		 /**< 读取 IMU 姿态数据 */

		/* --- BM：外部芯片烧录（如 NRF5x） --- */
		COMM_BM_CONNECT = 66,		   /**< 连接目标烧录芯片 */
		COMM_BM_ERASE_FLASH_ALL = 67,  /**< 擦除目标全部 Flash */
		COMM_BM_WRITE_FLASH = 68,	   /**< 写入目标 Flash */
		COMM_BM_REBOOT = 69,		   /**< 重启目标芯片 */
		COMM_BM_DISCONNECT = 70,	   /**< 断开目标芯片 */
		COMM_BM_MAP_PINS_DEFAULT = 71, /**< 映射默认烧录引脚 */
		COMM_BM_MAP_PINS_NRF5X = 72,   /**< 映射 NRF5x 烧录引脚 */

		/* --- 引导加载擦除 --- */
		COMM_ERASE_BOOTLOADER = 73,			/**< 擦除本机引导加载 */
		COMM_ERASE_BOOTLOADER_ALL_CAN = 74, /**< 全 CAN 擦除引导加载 */

		/* --- 绘图调试 --- */
		COMM_PLOT_INIT = 75,		   /**< 初始化绘图 */
		COMM_PLOT_DATA = 76,		   /**< 发送绘图数据点 */
		COMM_PLOT_ADD_GRAPH = 77,	   /**< 新增绘图曲线 */
		COMM_PLOT_SET_GRAPH = 78,	   /**< 选择当前绘图曲线 */
		COMM_GET_DECODED_BALANCE = 79, /**< 读取平衡控制器解码数据 */

		/* --- BM 内存读 / LZO 压缩固件写入 --- */
		COMM_BM_MEM_READ = 80,					  /**< 读取目标内存 */
		COMM_WRITE_NEW_APP_DATA_LZO = 81,		  /**< 写入新固件数据（LZO 压缩） */
		COMM_WRITE_NEW_APP_DATA_ALL_CAN_LZO = 82, /**< 全 CAN 写入新固件（LZO） */
		COMM_BM_WRITE_FLASH_LZO = 83,			  /**< 写入目标 Flash（LZO） */

		COMM_SET_CURRENT_REL = 84, /**< 设置相对电流（-1..1） */
		COMM_CAN_FWD_FRAME = 85,   /**< 转发原始 CAN 帧 */
		COMM_SET_BATTERY_CUT = 86, /**< 设置电池截止电压 */

		/* --- 蓝牙 / CAN 模式 / IMU 标定 --- */
		COMM_SET_BLE_NAME = 87,		   /**< 设置蓝牙名称 */
		COMM_SET_BLE_PIN = 88,		   /**< 设置蓝牙配对码 */
		COMM_SET_CAN_MODE = 89,		   /**< 设置 CAN 工作模式 */
		COMM_GET_IMU_CALIBRATION = 90, /**< 读取 IMU 标定数据 */
		COMM_GET_MCCONF_TEMP = 91,	   /**< 读取临时电机配置 */

		/* --- 自定义配置 --- */
		COMM_GET_CUSTOM_CONFIG_XML = 92,	 /**< 读取自定义配置 XML 描述 */
		COMM_GET_CUSTOM_CONFIG = 93,		 /**< 读取自定义配置 */
		COMM_GET_CUSTOM_CONFIG_DEFAULT = 94, /**< 读取自定义配置默认值 */
		COMM_SET_CUSTOM_CONFIG = 95,		 /**< 写入自定义配置 */

		/* --- BMS 电池管理系统 --- */
		COMM_BMS_GET_VALUES = 96,			/**< 读取 BMS 实时值 */
		COMM_BMS_SET_CHARGE_ALLOWED = 97,	/**< 设置是否允许充电 */
		COMM_BMS_SET_BALANCE_OVERRIDE = 98, /**< 强制设置均衡 */
		COMM_BMS_RESET_COUNTERS = 99,		/**< 复位 BMS 计数器 */
		COMM_BMS_FORCE_BALANCE = 100,		/**< 强制启动均衡 */
		COMM_BMS_ZERO_CURRENT_OFFSET = 101, /**< 校零电流偏置 */

		/* --- 指定硬件类型的引导/固件操作 --- */
		COMM_JUMP_TO_BOOTLOADER_HW = 102,		  /**< 跳转引导加载（指定硬件） */
		COMM_ERASE_NEW_APP_HW = 103,			  /**< 擦除新固件区（指定硬件） */
		COMM_WRITE_NEW_APP_DATA_HW = 104,		  /**< 写入新固件（指定硬件） */
		COMM_ERASE_BOOTLOADER_HW = 105,			  /**< 擦除引导加载（指定硬件） */
		COMM_JUMP_TO_BOOTLOADER_ALL_CAN_HW = 106, /**< 全 CAN 跳转引导（指定硬件） */
		COMM_ERASE_NEW_APP_ALL_CAN_HW = 107,	  /**< 全 CAN 擦除固件区（指定硬件） */
		COMM_WRITE_NEW_APP_DATA_ALL_CAN_HW = 108, /**< 全 CAN 写入固件（指定硬件） */
		COMM_ERASE_BOOTLOADER_ALL_CAN_HW = 109,	  /**< 全 CAN 擦除引导（指定硬件） */

		COMM_SET_ODOMETER = 110, /**< 设置里程计数值 */

		/* --- 电源开关板（PSW） --- */
		COMM_PSW_GET_STATUS = 111, /**< 读取电源开关板状态 */
		COMM_PSW_SWITCH = 112,	   /**< 控制电源开关板通断 */

		/* --- BMS 扩展 --- */
		COMM_BMS_FWD_CAN_RX = 113,	/**< 转发收到的 BMS CAN 帧 */
		COMM_BMS_HW_DATA = 114,		/**< BMS 硬件数据 */
		COMM_GET_BATTERY_CUT = 115, /**< 读取电池截止电压 */
		COMM_BM_HALT_REQ = 116,		/**< 请求暂停目标芯片 */

		/* --- QML 界面资源 --- */
		COMM_GET_QML_UI_HW = 117,  /**< 读取硬件 QML 界面 */
		COMM_GET_QML_UI_APP = 118, /**< 读取应用 QML 界面 */
		COMM_CUSTOM_HW_DATA = 119, /**< 自定义硬件透传数据 */
		COMM_QMLUI_ERASE = 120,	   /**< 擦除 QML 界面存储 */
		COMM_QMLUI_WRITE = 121,	   /**< 写入 QML 界面存储 */

		/* --- IO 扩展板 --- */
		COMM_IO_BOARD_GET_ALL = 122,	 /**< 读取 IO 板全部状态 */
		COMM_IO_BOARD_SET_PWM = 123,	 /**< 设置 IO 板 PWM 输出 */
		COMM_IO_BOARD_SET_DIGITAL = 124, /**< 设置 IO 板数字输出 */

		COMM_BM_MEM_WRITE = 125,	  /**< 写入目标内存 */
		COMM_BMS_BLNC_SELFTEST = 126, /**< BMS 均衡自检 */
		COMM_GET_EXT_HUM_TMP = 127,	  /**< 读取外部温湿度 */
		COMM_GET_STATS = 128,		  /**< 读取统计信息 */
		COMM_RESET_STATS = 129,		  /**< 复位统计信息 */

		/* --- LispBM 脚本 --- */
		COMM_LISP_READ_CODE = 130,	 /**< 读取 Lisp 脚本 */
		COMM_LISP_WRITE_CODE = 131,	 /**< 写入 Lisp 脚本 */
		COMM_LISP_ERASE_CODE = 132,	 /**< 擦除 Lisp 脚本 */
		COMM_LISP_SET_RUNNING = 133, /**< 启动/停止 Lisp 运行 */
		COMM_LISP_GET_STATS = 134,	 /**< 读取 Lisp 运行统计 */
		COMM_LISP_PRINT = 135,		 /**< Lisp 文本输出 */

		/* --- BMS 电池类型 --- */
		COMM_BMS_SET_BATT_TYPE = 136, /**< 设置电池类型 */
		COMM_BMS_GET_BATT_TYPE = 137, /**< 读取电池类型 */

		/* --- LispBM REPL / 流式 --- */
		COMM_LISP_REPL_CMD = 138,	 /**< Lisp REPL 命令 */
		COMM_LISP_STREAM_CODE = 139, /**< 流式写入 Lisp 脚本 */

		/* --- 文件系统 --- */
		COMM_FILE_LIST = 140,	/**< 列出目录 */
		COMM_FILE_READ = 141,	/**< 读取文件 */
		COMM_FILE_WRITE = 142,	/**< 写入文件 */
		COMM_FILE_MKDIR = 143,	/**< 新建目录 */
		COMM_FILE_REMOVE = 144, /**< 删除文件/目录 */

		/* --- 数据记录 --- */
		COMM_LOG_START = 145,		 /**< 开始数据记录 */
		COMM_LOG_STOP = 146,		 /**< 停止数据记录 */
		COMM_LOG_CONFIG_FIELD = 147, /**< 配置记录字段 */
		COMM_LOG_DATA_F32 = 148,	 /**< 记录数据（float32） */

		COMM_SET_APPCONF_NO_STORE = 149, /**< 设置应用配置（不写入 Flash） */
		COMM_GET_GNSS = 150,			 /**< 读取 GNSS 定位数据 */
		COMM_LOG_DATA_F64 = 151,		 /**< 记录数据（float64） */
		COMM_LISP_RMSG = 152,			 /**< Lisp 远程消息 */
		/* 153~155 预留给 pinlock，未启用 */
		COMM_SHUTDOWN = 156,			/**< 关机 */
		COMM_FW_INFO = 157,				/**< 读取固件信息 */
		COMM_CAN_UPDATE_BAUD_ALL = 158, /**< 全 CAN 更新波特率 */
		COMM_MOTOR_ESTOP = 159,			/**< 电机紧急停止 */
	} COMM_PACKET_ID;

#ifdef __cplusplus
}
#endif

#endif /* VESC_COMM_IDS_H_ */
