#!/usr/bin/env python3
"""
关节电机 CAN DBC 文件生成器
从 joint_motor_command_list.csv 的命令定义生成 Vector CANdb++ 兼容的 DBC 文件。

用法: python gen_can_dbc.py [--motor-id 1]
输出: <repo>/User/Protocol/docs/joint_motor_can.dbc
"""
import argparse
import os

# ============================== 信号辅助函数 ==============================
# (name, start_bit, bits, signed, is_float, factor, offset, min, max, unit)
def F(n, s):   return (n, s, 32, False, True,  1, 0, -3.4e38, 3.4e38, '')
def U8(n, s):  return (n, s, 8,  False, False, 1, 0, 0, 255, '')
def U12(n, s): return (n, s, 12, False, False, 1, 0, 0, 4095, '')
def U16(n, s): return (n, s, 16, False, False, 1, 0, 0, 65535, '')
def U32(n, s): return (n, s, 32, False, False, 1, 0, 0, 4294967295, '')
def I8(n, s):  return (n, s, 8,  True,  False, 1, 0, -128, 127, '')
def I32(n, s): return (n, s, 32, True,  False, 1, 0, -2147483648, 2147483647, '')

# ============================== 命令定义 ==============================
# (cmd, name, sender, dlc, signals, comment, multi)
#   sender: 'Host' 或 'Motor'
#   dlc:    消息数据长度 (多帧命令固定 8)
#   multi:  True=多帧命令 (载荷>8B)
# 策略: 请求有载荷→定义请求(Host); 请求无载荷但应答有载荷→定义应答(Motor)

CMDS = [
    # ---- 系统控制 0x00~0x07 ----
    (0x00, "IDLE",            "Host", 0, [], "进入待机 IDLE停止输出", False),
    (0x01, "HOLD",            "Host", 0, [], "位置保持 锁定当前位置", False),
    (0x02, "BRAKE",           "Host", 0, [], "机械刹车 短接相线抱闸", False),
    (0x03, "ESTOP",           "Host", 0, [], "紧急停止 最高优先级断输出", False),
    (0x04, "ENABLE",          "Host", 0, [], "上使能 IDLE->READY", False),
    (0x05, "DISABLE",         "Host", 0, [], "下使能 READY/RUN->IDLE", False),
    (0x06, "STOP",            "Host", 0, [], "停止运行 RUN->READY", False),
    (0x07, "SOFT_RESET",      "Host", 4, [U32("magic",0)], "软件复位 magic=0x5E7E7E5E ACK后延迟约200ms复位 广播不开放", False),

    # ---- 运动控制 0x10~0x1C ----
    (0x10, "OPEN_LOOP",       "Host", 8, [F("ud",0), F("uq",32)], "开环电压", False),
    (0x11, "CURRENT",         "Host", 8, [F("id_ref",0), F("iq_ref",32)], "电流环", False),
    (0x12, "TORQUE",          "Host", 4, [F("torque",0)], "力矩环 Nm", False),
    (0x13, "MIT",             "Host", 8, [U16("pos",0), U12("vel",16), U12("kp",28), U12("kd",40), U12("tff",52)],
     "MIT控制 CAN定点压缩: pos16+vel12+kp12+kd12+tff12", False),
    (0x14, "VELOCITY",        "Host", 4, [F("vel_ref",0)], "速度环 rad/s", False),
    (0x15, "POSITION",        "Host", 4, [F("pos_ref",0)], "位置环 rad", False),
    (0x16, "POS_VEL_FF",      "Host", 8, [F("pos",0), F("vel_ff",32)], "位置+速度前馈", False),
    (0x17, "POS_TORQUE_LIM",  "Host", 8, [F("pos",0), F("tq_lim",32)], "位置+力矩限幅", False),
    (0x18, "VEL_TORQUE_LIM",  "Host", 8, [F("vel",0), F("tq_lim",32)], "速度+力矩限幅", False),
    (0x19, "DUTY_CYCLE",      "Host", 4, [F("duty",0)], "占空比控制 -1~1", False),
    (0x1A, "VOLTAGE_VECTOR",  "Host", 8, [F("u_alpha",0), F("u_beta",32)], "电压矢量", False),
    (0x1B, "FIELD_WEAKENING", "Host", 8, [F("id_weak",0), F("iq_ref",32)], "弱磁控制", False),
    (0x1C, "SENSORLESS",      "Host", 4, [F("vel_ref",0)], "无感FOC", False),

    # ---- 高级力控 0x30~0x3A ----
    (0x30, "IMPEDANCE",       "Host", 8, [U16("pos",0), U12("vel",16), U12("kp",28), U12("kd",40), U12("tff",52)],
     "阻抗控制 CAN同MIT压缩", False),
    (0x31, "ADMITTANCE",      "Host", 8, [], "导纳控制 多帧(16B): force/mass/damp/stiff", True),
    (0x32, "FORCE_CONTROL",   "Host", 4, [F("force",0)], "纯力控制 N或Nm", False),
    (0x33, "FORCE_POS_HYBRID","Host", 8, [], "力位混合 多帧(12B): pos/force/sel_mask", True),
    (0x34, "GRAVITY_COMP",    "Host", 0, [], "重力补偿", False),
    (0x35, "COLLISION_DET",   "Host", 5, [F("threshold",0), U8("enable",32)], "碰撞检测", False),
    (0x36, "ZERO_FORCE",      "Host", 0, [], "零力模式 拖动示教", False),
    (0x37, "CONSTANT_FORCE",  "Host", 4, [F("force",0)], "恒力控制 N或Nm", False),
    (0x38, "VAR_IMPEDANCE",   "Host", 8, [], "变阻抗 多帧(12B): kp/kd/rate", True),
    (0x39, "ADAPT_GRAVITY",   "Host", 4, [F("gain",0)], "自适应重力补偿", False),
    (0x3A, "LANDING_BUFFER",  "Host", 8, [F("stiffness",0), F("damp",32)], "落地缓冲", False),

    # ---- 轨迹同步 0x50~0x5D ----
    (0x50, "PVT",             "Host", 8, [], "PVT插补 多帧(12B): pos/vel/time_ms", True),
    (0x51, "CUBIC_SPLINE",    "Host", 8, [], "三次样条 多帧(18B): seg_idx/coef[4]", True),
    (0x52, "TRAPEZOIDAL",     "Host", 8, [], "梯形轨迹 多帧(12B): target/vmax/acc", True),
    (0x53, "S_CURVE",         "Host", 8, [], "S型轨迹 多帧(16B): target/vmax/acc/jerk", True),
    (0x54, "HOMING",          "Host", 1, [U8("method",0)], "回零", False),
    (0x55, "CANOPEN_SYNC",    "Host", 0, [], "CANopen同步 SYNC对象", False),
    (0x56, "ETHERCAT_CSP",    "Host", 4, [F("pos",0)], "EtherCAT CSP 周期同步位置", False),
    (0x57, "ETHERCAT_CSV",    "Host", 4, [F("vel",0)], "EtherCAT CSV", False),
    (0x58, "ETHERCAT_CST",    "Host", 4, [F("torque",0)], "EtherCAT CST", False),
    (0x59, "PP",              "Host", 8, [F("pos",0), F("vel",32)], "轮廓位置", False),
    (0x5A, "PV",              "Host", 8, [F("vel",0), F("acc",32)], "轮廓速度", False),
    (0x5B, "PT",              "Host", 8, [F("torque",0), F("slope",32)], "轮廓力矩", False),
    (0x5C, "ELECTRONIC_GEAR", "Host", 8, [I32("ratio_num",0), I32("ratio_den",32)], "电子齿轮", False),
    (0x5D, "ELECTRONIC_CAM",  "Host", 2, [U16("cam_table_id",0)], "电子凸轮", False),

    # ---- 特殊应用与测试 0x70~0x7B ----
    (0x70, "STEP_DIR",        "Host", 4, [U32("pulse_per_rev",0)], "脉冲方向", False),
    (0x71, "ANALOG_INPUT",    "Host", 5, [U8("ch",0), F("scale",8)], "模拟量输入", False),
    (0x72, "PWM_INPUT",       "Host", 4, [U16("min_us",0), U16("max_us",16)], "PWM输入", False),
    (0x73, "JOG",             "Host", 5, [I8("dir",0), F("speed",8)], "点动", False),
    (0x74, "SAFE_TEACH",      "Host", 0, [], "安全示教", False),
    (0x75, "TEST_AGING",      "Host", 4, [U32("cycles",0)], "老化测试", False),
    (0x76, "TEST_SWEEP_FREQ", "Host", 8, [], "扫频测试 多帧(12B): f_start/f_end/amp", True),
    (0x77, "TEST_COGGING",    "Host", 0, [], "齿槽测试", False),
    (0x78, "TEST_FRICTION",   "Host", 0, [], "摩擦测试", False),
    (0x79, "TEST_INERTIA",    "Host", 0, [], "惯量测试", False),
    (0x7A, "TEST_CURRENT_LOOP","Host",8, [F("amp",0), F("freq",32)], "电流环测试", False),
    (0x7B, "TEST_VEL_LOOP",   "Host", 8, [F("amp",0), F("freq",32)], "速度环测试", False),

    # ---- 多电机同步(预留) 0x80~0x82 ----
    (0x80, "SYNC_FRAME",      "Host", 0, [], "周期同步帧 预留 回NACK(NOT_SUPPORTED)", False),
    (0x81, "PRESET_CMD",      "Host", 0, [], "预存指令 预留 回NACK(NOT_SUPPORTED)", False),
    (0x82, "TRIGGER",         "Host", 0, [], "广播触发 预留 ID=0广播 回NACK(NOT_SUPPORTED)", False),

    # ---- 校准 0x90~0x98 ----
    (0x90, "CALIB_L1",        "Host", 1, [U8("submode",0)], "L1驱动硬件底层校准 submode:1=ADC偏置 2=ADC增益 3=电流传感器 4=温度 5=母线电压 6=死区", False),
    (0x91, "CALIB_L2",        "Host", 1, [U8("submode",0)], "L2电机电气身份校准 submode:1=相序 2=极对数 3=R/Ld/Lq/flux", False),
    (0x92, "CALIB_L3",        "Host", 1, [U8("submode",0)], "L3编码器校准 submode:1=零位 2=方向 3=线性度 4=正余弦/旋变 5=多圈零点", False),
    (0x93, "CALIB_L4",        "Host", 1, [U8("submode",0)], "L4转矩基础校准 submode:1=力矩常数Kt", False),
    (0x94, "CALIB_L5",        "Host", 1, [U8("submode",0)], "L5非线性补偿校准 submode:1=齿槽 2=摩擦 3=死区补偿 4=磁饱和", False),
    (0x95, "CALIB_L6",        "Host", 1, [U8("submode",0)], "L6负载系统级校准 submode:1=惯量 2=阻尼 3=回程间隙 4=PID自整定", False),
    (0x96, "CALIB_L7",        "Host", 1, [U8("submode",0)], "L7自动化集成校准 submode:1=一键全自动", False),
    (0x97, "CALIB_QUERY_ACK", "Motor",8,
     [U8("state",0), U8("fail_reason",8), U8("progress",16), U8("level",24),
      U8("submode",32), U8("step",40), U8("step_total",48), U8("reserved",56)],
     "标定进度查询应答 state:0=空闲 1=进行中 2=完成 3=失败 fail_reason:0~9", False),
    (0x98, "CALIB_ABORT",     "Host", 0, [], "标定中止", False),

    # ---- PID管理 0xA0~0xA6 ----
    (0xA0, "PID_AUTOTUNE",    "Host", 8, [], "PID理论估计 多帧(13B): ring_mask/cur_bw/vel_bw/pos_bw", True),
    (0xA1, "PID_SOURCE_SET",  "Host", 2, [U8("ring_select",0), U8("source",8)],
     "PID来源切换 ring:0=电流 1=速度 2=位置 source:0=默认 1=Flash 2=理论估计 3=调试", False),
    (0xA2, "PID_SOURCE_ACK",  "Motor",3, [U8("cur_src",0), U8("vel_src",8), U8("pos_src",16)],
     "读PID来源应答 0=默认 1=Flash 2=理论估计 3=调试", False),
    (0xA3, "SMOOTH_CFG_SET", "Host", 5, [U8("param_id",0), U32("value",8)],
     "缓启动渐变配置写 id:0=enable 1=shape 2=duration 3~6=rate 7~12=thresh 13=模式切换兜底", False),
    (0xA4, "SMOOTH_CFG_GET", "Host", 1, [U8("param_id",0)],
     "缓启动渐变配置读 应答4B单字段/0xFF多帧(52B整块)", False),
    (0xA5, "PID_PARAM_WRITE", "Host", 6, [U8("ring",0), U8("param_type",8), U32("value",16)],
     "PID参数实时写 ring:0=D轴 1=Q轴 2=速度 3=位置 type:1=kp 2=ki 3=kd 4=output_limit 5=integral_limit 6=alpha 7=flags", False),
    (0xA6, "PID_PARAM_READ",  "Host", 2, [U8("ring",0), U8("param_type",8)],
     "PID参数实时读 应答6B: ring/param_type/value(4B)", False),

    # ---- 系统诊断 0xB0~0xB8 ----
    (0xB0, "CLEAR_FAULT",     "Host", 0, [], "清除故障 FAULT->IDLE", False),
    (0xB1, "DIAGNOSTIC",      "Host", 0, [], "诊断模式", False),
    (0xB2, "ENTER_BOOTLOADER","Host", 4, [U32("magic",0)], "进入Bootloader magic=0xB00710AD 不匹配回NACK(UNAUTHORIZED)", False),
    (0xB3, "SAVE_CONFIG",     "Host", 0, [], "保存配置 参数写入Flash(与0xE4等价)", False),
    (0xB4, "FACTORY_RESET",   "Host", 4, [U32("magic",0)], "恢复出厂 magic=0xFAC70F5F", False),
    (0xB5, "START_LOG",       "Host", 6, [U16("rate_hz",0), U32("mask",16)], "开始日志", False),
    (0xB6, "STOP_LOG",        "Host", 0, [], "停止日志", False),
    (0xB7, "HIGH_SPEED_DAQ",  "Host", 8, [U32("ch_mask",0), U32("rate_hz",32)], "高速采集", False),
    (0xB8, "SINGLE_STEP",     "Host", 0, [], "单步调试", False),

    # ---- 反馈查询 0xC0~0xCB ----
    (0xC0, "FEEDBACK_ACK",    "Motor",8,
     [U16("pos",0), U16("vel",16), U16("torque",32), U8("temp",48), U8("err",56)],
     "读实时反馈应答 CAN压缩: pos16+vel16+tq16+tmp8+err8", False),
    (0xC1, "STATE_ACK",       "Motor",4,
     [U8("top_fsm",0), U8("run_state",8), U8("ctrl_mode",16), U8("enable",24)],
     "读电机状态应答", False),
    (0xC2, "PHASE_CURRENT_ACK","Motor",8, [], "读相电流应答 多帧(12B): ia/ib/ic f32", True),
    (0xC3, "DQ_CURRENT_ACK",  "Motor",8, [F("id",0), F("iq",32)], "读DQ电流应答", False),
    (0xC4, "BUS_INFO_ACK",    "Motor",8, [], "读母线信息应答 多帧(12B): vbus/ibus/power", True),
    (0xC5, "TEMP_ACK",        "Motor",8, [F("temp_fet",0), F("temp_motor",32)], "读温度应答", False),
    (0xC6, "POS_VEL_ACK",     "Motor",8, [F("pos",0), F("vel",32)], "读位置速度应答", False),
    (0xC7, "MULTITURN_ACK",   "Motor",8, [I32("multiturn",0), F("single",32)], "读多圈位置应答", False),
    (0xC8, "FAULT_ACK",       "Motor",8, [U32("fault_mask",0), U32("warn_mask",32)], "读故障码应答", False),
    (0xC9, "DEBUG_ACK",       "Motor",8, [], "读调试数据应答 多帧(变长): dbg:f32[N] N<=16", True),
    (0xCA, "TELEMETRY",       "Motor",8, [U16("mask",0)], "同步遥测 多帧(变长): mask后按位拼接数据", True),
    (0xCB, "SET_TELEMETRY",   "Host", 5, [U8("enable",0), U16("mask",8), U16("period",24)], "设置遥测", False),

    # ---- OTA预留 0xCC~0xCF ----
    (0xCC, "OTA_START",       "Host", 0, [], "OTA启动 预留 回NACK(NOT_SUPPORTED)", False),
    (0xCD, "OTA_DATA",        "Host", 0, [], "OTA数据 预留 回NACK(NOT_SUPPORTED)", False),
    (0xCE, "OTA_END",         "Host", 0, [], "OTA结束 预留 回NACK(NOT_SUPPORTED)", False),
    (0xCF, "OTA_RESUME",      "Host", 0, [], "OTA续传 预留 回NACK(NOT_SUPPORTED)", False),

    # ---- 设备信息 0xD0~0xD2 ----
    (0xD0, "DEV_INFO_ACK",    "Motor",8, [], "读设备信息应答 多帧(28B): hw_ver/fw_ver/uid+扩展8B", True),
    (0xD1, "DEV_NAME_ACK",    "Motor",8, [], "读设备名称应答 多帧(16B): char[16]", True),
    (0xD2, "HEARTBEAT",       "Motor",7, [U8("state",0), U16("err",8), U32("ts",24)], "心跳/在线", False),

    # ---- 参数读写 0xE0~0xE5 ----
    (0xE0, "PARAM_READ",      "Host", 2, [U16("param_id",0)], "读单个参数", False),
    (0xE1, "PARAM_WRITE",     "Host", 8, [], "写单个参数 多帧(变长): param_id+value", True),
    (0xE2, "PARAM_READ_BULK", "Host", 8, [], "批量读参数 多帧: start_id+count", True),
    (0xE3, "PARAM_WRITE_BULK","Host", 8, [], "批量写参数 多帧(变长): start_id+count+values", True),
    (0xE4, "PARAM_SAVE",      "Host", 0, [], "参数存Flash(同0xB3)", False),
    (0xE5, "PARAM_DEFAULT",   "Host", 2, [U16("param_id",0)], "参数恢复默认 0xFFFF=全部", False),

    # ---- 电机配置 0xE6~0xEC ----
    (0xE6, "MOTOR_INFO_READ", "Host", 2, [U16("param_id",0)], "读电机配置 依赖USE_DEV_FLASH", False),
    (0xE7, "MOTOR_INFO_WRITE","Host", 6, [U16("param_id",0), U32("value",16)], "写电机配置 RAM生效 需0xEA固化", False),
    (0xE8, "MOTOR_INFO_READ_BULK","Host",8, [], "批量读电机配置 多帧: start_id+count", True),
    (0xE9, "MOTOR_INFO_WRITE_BULK","Host",8, [], "批量写电机配置 多帧(变长)", True),
    (0xEA, "MOTOR_INFO_SAVE", "Host", 0, [], "电机配置存Flash 依赖USE_DEV_FLASH", False),
    (0xEB, "MOTOR_INFO_DEFAULT","Host",2, [U16("param_id",0)], "电机配置恢复默认 0xFFFF=全部", False),
    (0xEC, "MOTOR_INFO_RECALIB_RESET","Host",0, [], "电机配置重新标定复位 清除编码器字段", False),

    # ---- CAN管理 0xF0~0xF6 ----
    (0xF0, "SET_CAN_ID",      "Host", 1, [U8("new_id",0)], "设置CAN_ID 1~127 需保存重启生效", False),
    (0xF1, "SET_BAUDRATE",    "Host", 1, [U8("baud_code",0)], "设置波特率 0=1M 1=500K 2=250K 3=125K", False),
    (0xF2, "BROADCAST_SYNC",  "Host", 0, [], "广播同步 ID=0广播", False),
    (0xF3, "SET_FD_MODE",     "Host", 1, [U8("enable",0)], "设置FD模式 运行期切换", False),
    (0xF4, "CAN_DI_DISCOVER", "Both", 8, [], "CAN-DI广播发现/单帧时隙响应", False),
    (0xF5, "CAN_DI_SET_ID",   "Both", 8, [], "按CAN-DI定向设置节点ID", False),
    (0xF6, "CAN_DI_IDENTIFY", "Both", 8, [], "按CAN-DI触发物理设备指示", False),

    # ---- 通用 0xFE ----
    (0xFE, "NACK",            "Motor",2, [U8("cmd",0), U8("err_code",8)], "错误应答 err_code见协议文档", False),
]

# ============================== 值表定义 ==============================
# (cmd, signal_name, [(value, label), ...])
VAL_TABLES = [
    (0xA1, "ring_select", [(0,"current"), (1,"velocity"), (2,"position")]),
    (0xA1, "source",      [(0,"default"), (1,"flash"), (2,"autotune"), (3,"debug")]),
    (0xA5, "ring",        [(0,"d_axis"), (1,"q_axis"), (2,"velocity"), (3,"position")]),
    (0xA5, "param_type",  [(1,"kp"), (2,"ki"), (3,"kd"), (4,"output_limit"),
                           (5,"integral_limit"), (6,"output_filter_alpha"), (7,"flags")]),
    (0xF1, "baud_code",   [(0,"1M"), (1,"500K"), (2,"250K"), (3,"125K")]),
    (0xFE, "err_code",    [(0x01,"CMD_UNSUPPORTED"), (0x02,"OUT_OF_RANGE"), (0x03,"STATE_DENY"),
                           (0x04,"BAD_PARAM_ID"), (0x05,"CRC_ERROR"), (0x06,"LENGTH"),
                           (0x07,"READ_ONLY"), (0x08,"FLASH_FAIL"), (0x09,"FAULT_STATE"),
                           (0x0A,"CALIB_BUSY"), (0x0B,"ASYNC_QUEUED"), (0x0C,"BUSY"),
                           (0x0D,"UNAUTHORIZED"), (0x0E,"RATE_LIMIT"), (0x0F,"NOT_FOUND"),
                           (0x10,"FLASH_ERASE_FAIL"), (0x11,"FLASH_WRITE_FAIL"),
                           (0x12,"FLASH_VERIFY_FAIL"), (0x13,"NOT_IMPLEMENTED")]),
    (0x97, "state",       [(0,"idle"), (1,"running"), (2,"done"), (3,"failed")]),
    (0x97, "fail_reason", [(0,"none"), (1,"timeout"), (2,"out_of_range"), (3,"dep_not_met"),
                           (4,"sample_error"), (5,"motor_not_turn"), (6,"over_current"),
                           (7,"over_speed"), (8,"flash_write_fail"), (9,"aborted")]),
]

# ============================== DBC 生成 ==============================

def gen_dbc(motor_id: int) -> str:
    lines = []
    EXT = 0x80000000  # 扩展帧标志位

    # ---- 文件头 ----
    lines.append('VERSION "JointMotor CAN Database v1.1"')
    lines.append('')
    lines.append('NS_ :')
    lines.append('    NS_DESC_')
    lines.append('    CM_')
    lines.append('    BA_DEF_')
    lines.append('    BA_')
    lines.append('    VAL_')
    lines.append('    BA_DEF_DEF_')
    lines.append('    SIG_VALTYPE_')
    lines.append('')
    lines.append('BS_:')
    lines.append('')
    lines.append('BU_: Host Motor')
    lines.append('')

    # ---- 消息+信号定义 ----
    # 扩展帧: BO_行ID用 can_id | 0x80000000 标记
    for cmd, name, sender, dlc, sigs, cmt, multi in CMDS:
        can_id = (cmd << 8) | motor_id
        ext_id = can_id | EXT
        lines.append(f'BO_ {ext_id} {name}: {dlc} {sender}')
        receiver = 'Motor' if sender == 'Host' else 'Host'
        for sig in sigs:
            sname, sbit, sbits, ssigned, sfloat, fac, off, smin, smax, unit = sig
            sign_char = '-' if ssigned else '+'
            byte_order = 1  # Intel little-endian
            unit_str = unit if unit else ''
            min_str = f'{smin}'
            max_str = f'{smax}'
            lines.append(
                f' SG_ {sname} : {sbit}|{sbits}@{byte_order}{sign_char} '
                f'({fac},{off}) [{min_str}|{max_str}] "{unit_str}" {receiver}'
            )
        lines.append('')

    # ---- 消息注释 ----
    for cmd, name, sender, dlc, sigs, cmt, multi in CMDS:
        can_id = (cmd << 8) | motor_id
        ext_id = can_id | EXT
        prefix = "[Multi] " if multi else ""
        lines.append(f'CM_ BO_ {ext_id} "{prefix}{cmt}  CAN_ID=0x{cmd:02X}<<8|MotorID";')

    lines.append('')

    # ---- float 信号类型标记 (带冒号格式) ----
    for cmd, name, sender, dlc, sigs, cmt, multi in CMDS:
        can_id = (cmd << 8) | motor_id
        ext_id = can_id | EXT
        for sig in sigs:
            sname = sig[0]
            is_float = sig[4]
            if is_float:
                lines.append(f'SIG_VALTYPE_ {ext_id} {sname} : 1;')

    lines.append('')

    # ---- 值表 ----
    for cmd, sig_name, vals in VAL_TABLES:
        can_id = (cmd << 8) | motor_id
        ext_id = can_id | EXT
        val_str = ' '.join(f'{v} "{lbl}"' for v, lbl in vals)
        lines.append(f'VAL_ {ext_id} {sig_name} {val_str} ;')

    lines.append('')

    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description='生成关节电机CAN DBC文件')
    parser.add_argument('--motor-id', type=int, default=1, help='电机ID (1~127, 默认1)')
    args = parser.parse_args()

    if not 1 <= args.motor_id <= 127:
        raise ValueError(f"MotorID必须在1~127范围, 当前={args.motor_id}")

    dbc = gen_dbc(args.motor_id)

    # 输出到 Protocol/docs 目录
    script_dir = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.abspath(os.path.join(script_dir, '..', '..'))
    out_path = os.path.join(repo_root, 'Protocol', 'docs', 'joint_motor_can.dbc')

    with open(out_path, 'w', encoding='utf-8') as f:
        f.write(dbc)

    # 统计
    total = len(CMDS)
    multi_cnt = sum(1 for c in CMDS if c[6])
    sig_cnt = sum(len(c[4]) for c in CMDS)
    float_cnt = sum(1 for c in CMDS for s in c[4] if s[4])

    print(f"DBC文件已生成: {out_path}")
    print(f"  消息总数: {total}")
    print(f"  多帧命令: {multi_cnt}")
    print(f"  信号总数: {sig_cnt} (其中float信号: {float_cnt})")
    print(f"  值表总数: {len(VAL_TABLES)}")
    print(f"  MotorID: {args.motor_id}")


if __name__ == '__main__':
    main()
