#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
生成 Serial Studio 工程 jointmotor_uart.ssproj
- 解析器按 CMD 路由解析 0xC0~0xC8 全部反馈帧 -> 定长 21 槽 Frame Index
- 为 0xC0~0xC8 各加一条连接后自动轮询动作(CRC16/XMODEM 自动计算)
- 数据集按功能分组铺开全部遥测量
"""
import json

parser = r'''/**
 * 关节电机串口多命令反馈解析器 (Serial Studio - JavaScript)
 *
 * 解码器: Binary (decoder=3)  帧检测: No Delimiters (frameDetection=2)
 * 帧格式 (与固件 packer_parser.c 一致):
 *   A5 5A | LEN_H LEN_L | HDR_CHK | [CMD + DATA] | CRC_L CRC_H
 *   LEN     = CMD(1)+DATA(n) 总字节数, 大端
 *   HDR_CHK = (0xA5 + 0x5A + LEN_H + LEN_L) & 0xFF
 *   CRC16   = CRC-16/XMODEM(poly 0x1021, init 0x0000) 覆盖数据区(CMD+DATA), 小端落帧
 *
 * 按 CMD 分别解析全部反馈帧, 映射到定长 21 槽 Frame Index:
 *   1 pos  2 vel  3 torque  4 tempFet  5 tempMotor  6 vbus  7 ibus  8 power
 *   9 id  10 iq  11 ia  12 ib  13 ic  14 multiturn  15 single
 *   16 fault  17 warn  18 topFsm  19 runState  20 ctrlMode  21 enable
 * 未刷新的槽保留上一次值(脚本内静态保持)。
 *
 *   0xC0 实时反馈(22B): pos,vel,torque,tempMotor,vbus,fault(u16)
 *   0xC1 状态(4B)     : topFsm,runState,ctrlMode,enable (u8)
 *   0xC2 三相电流(12B): ia,ib,ic
 *   0xC3 dq电流(8B)   : id,iq
 *   0xC4 母线(12B)    : vbus,ibus,power
 *   0xC5 温度(8B)     : tempFet,tempMotor
 *   0xC6 位置速度(8B) : pos,vel
 *   0xC7 多圈(8B)     : multiturn(u32),single(f32)
 *   0xC8 故障(8B)     : fault(u32),warn(u32)
 */

var __buf = [];
var __crcTab = null;
var __slots = new Array(21);
for (var __i = 0; __i < 21; __i++) __slots[__i] = 0;

var MAX_DLEN = 512;
var BUF_CAP  = 4096;

function crcInit() {
    __crcTab = new Array(256);
    for (var i = 0; i < 256; i++) {
        var c = (i << 8) & 0xFFFF;
        for (var k = 0; k < 8; k++)
            c = (c & 0x8000) ? (((c << 1) ^ 0x1021) & 0xFFFF) : ((c << 1) & 0xFFFF);
        __crcTab[i] = c;
    }
}
function crc16(buf, start, len) {
    if (!__crcTab) crcInit();
    var crc = 0;
    for (var i = 0; i < len; i++)
        crc = ((crc << 8) ^ __crcTab[((crc >> 8) ^ buf[start + i]) & 0xFF]) & 0xFFFF;
    return crc;
}
function rdF32(b, o) {
    var dv = new DataView(new ArrayBuffer(4));
    dv.setUint8(0, b[o]); dv.setUint8(1, b[o + 1]);
    dv.setUint8(2, b[o + 2]); dv.setUint8(3, b[o + 3]);
    return dv.getFloat32(0, true);
}
function rdU16(b, o) { return (b[o] | (b[o + 1] << 8)) & 0xFFFF; }
function rdU32(b, o) {
    return (b[o] | (b[o+1]<<8) | (b[o+2]<<16) | (b[o+3]<<24)) >>> 0;
}

function applyFrame(data) {
    var cmd = data[0];
    if (cmd === 0xC0 && data.length >= 23) {
        __slots[0]=rdF32(data,1); __slots[1]=rdF32(data,5);
        __slots[2]=rdF32(data,9); __slots[4]=rdF32(data,13);
        __slots[5]=rdF32(data,17); __slots[15]=rdU16(data,21);
    } else if (cmd === 0xC1 && data.length >= 5) {
        __slots[17]=data[1]; __slots[18]=data[2];
        __slots[19]=data[3]; __slots[20]=data[4];
    } else if (cmd === 0xC2 && data.length >= 13) {
        __slots[10]=rdF32(data,1); __slots[11]=rdF32(data,5); __slots[12]=rdF32(data,9);
    } else if (cmd === 0xC3 && data.length >= 9) {
        __slots[8]=rdF32(data,1); __slots[9]=rdF32(data,5);
    } else if (cmd === 0xC4 && data.length >= 13) {
        __slots[5]=rdF32(data,1); __slots[6]=rdF32(data,5); __slots[7]=rdF32(data,9);
    } else if (cmd === 0xC5 && data.length >= 9) {
        __slots[3]=rdF32(data,1); __slots[4]=rdF32(data,5);
    } else if (cmd === 0xC6 && data.length >= 9) {
        __slots[0]=rdF32(data,1); __slots[1]=rdF32(data,5);
    } else if (cmd === 0xC7 && data.length >= 9) {
        __slots[13]=rdU32(data,1); __slots[14]=rdF32(data,5);
    } else if (cmd === 0xC8 && data.length >= 9) {
        __slots[15]=rdU32(data,1); __slots[16]=rdU32(data,5);
    }
}

function parse(frame) {
    var i;
    for (i = 0; i < frame.length; i++) __buf.push(frame[i] & 0xFF);
    if (__buf.length > BUF_CAP) __buf = __buf.slice(__buf.length - BUF_CAP);

    var got = false;
    while (true) {
        while (__buf.length >= 2 && !(__buf[0] === 0xA5 && __buf[1] === 0x5A))
            __buf.shift();
        if (__buf.length < 5) break;
        var lenH = __buf[2], lenL = __buf[3];
        var dlen = (lenH << 8) | lenL;
        if (dlen > MAX_DLEN) { __buf.shift(); continue; }
        var total = 5 + dlen + 2;
        if (__buf.length < total) break;
        var hdrChk = (0xA5 + 0x5A + lenH + lenL) & 0xFF;
        if (__buf[4] !== hdrChk) { __buf.shift(); continue; }
        var rxCrc = __buf[5 + dlen] | (__buf[6 + dlen] << 8);
        var ok = (crc16(__buf, 5, dlen) === rxCrc);
        var data = __buf.slice(5, 5 + dlen);
        __buf = __buf.slice(total);
        if (!ok) continue;
        applyFrame(data);
        got = true;
    }
    if (!got) return [];
    return __slots.slice(0);
}
'''


def crc16_xmodem(data):
    crc = 0
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


def frame_cmd_only(cmd):
    crc = crc16_xmodem([cmd])
    return 'A55A0001%02X%02X%02X' % (cmd, crc & 0xFF, (crc >> 8) & 0xFF)


def act(title, tx, auto=False, ms=100, mode=0):
    return {"autoExecuteOnConnect": auto, "binary": True, "eol": "", "icon": "Play Property",
            "repeatCount": 3, "sourceId": 0, "timerIntervalMs": ms, "timerMode": mode,
            "title": title, "txData": tx, "txEncoding": 0}


actions = [
    # ── 系统控制 ──
    act("【系统】上使能 Enable", "A55A000100048440"),
    act("【系统】下使能 Disable", "A55A00010005A550"),
    act("【系统】停止运行 Stop", "A55A00010006C660"),
    act("【系统】位置保持 Hold", "A55A000100012110"),
    act("【系统】紧急停止 E-Stop", "A55A000100036330"),
    act("【系统】清除故障 ClearFault", "A55A000100B0DBA7"),
    act("【系统】保存配置 SaveCfg", "A55A000100B3B897"),
    # ── 运动控制 ──
    act("【运动】回零 Homing", "A55A00020154007BC2"),
    act("【运动】位置环 -> 0 rad", "A55A00050415000000000D27"),
    act("【运动】位置环 -> 1.0 rad", "A55A000504150000803F29FB"),
    act("【运动】位置环 -> -1.0 rad", "A55A00050415000080BFA16A"),
    act("【运动】速度环 -> 5 rad/s", "A55A000504140000A040E6D8"),
    act("【运动】力矩环 -> 0.5 Nm", "A55A000504120000003F6587"),
    # ── 自动轮询(连接后自动 20Hz/分频) ──
    act("【轮询】实时反馈 0xC0", frame_cmd_only(0xC0), auto=True, ms=50, mode=1),
    act("【轮询】电机状态 0xC1", frame_cmd_only(0xC1), auto=True, ms=200, mode=1),
    act("【轮询】三相电流 0xC2", frame_cmd_only(0xC2), auto=True, ms=50, mode=1),
    act("【轮询】dq电流 0xC3", frame_cmd_only(0xC3), auto=True, ms=50, mode=1),
    act("【轮询】母线 0xC4", frame_cmd_only(0xC4), auto=True, ms=100, mode=1),
    act("【轮询】温度 0xC5", frame_cmd_only(0xC5), auto=True, ms=500, mode=1),
    act("【轮询】位置速度 0xC6", frame_cmd_only(0xC6), auto=True, ms=50, mode=1),
    act("【轮询】多圈 0xC7", frame_cmd_only(0xC7), auto=True, ms=200, mode=1),
    act("【轮询】故障码 0xC8", frame_cmd_only(0xC8), auto=True, ms=200, mode=1),
]

# (title, unit, min, max, widget, graph)
fields = [
    ('位置 Pos', 'rad', -12.5, 12.5, 'gauge', True),
    ('速度 Vel', 'rad/s', -65, 65, 'gauge', True),
    ('力矩 Torque', 'Nm', -50, 50, 'gauge', True),
    ('温度FET', 'C', 0, 120, 'bar', False),
    ('电机温度', 'C', 0, 120, 'bar', False),
    ('母线电压 Vbus', 'V', 0, 60, 'bar', False),
    ('母线电流 Ibus', 'A', -30, 30, 'gauge', True),
    ('功率 Power', 'W', -500, 500, 'gauge', True),
    ('Id 电流', 'A', -30, 30, 'gauge', True),
    ('Iq 电流', 'A', -30, 30, 'gauge', True),
    ('A相 Ia', 'A', -30, 30, '', True),
    ('B相 Ib', 'A', -30, 30, '', True),
    ('C相 Ic', 'A', -30, 30, '', True),
    ('多圈 Multiturn', 'turn', -100, 100, '', False),
    ('单圈 Single', 'rad', -6.3, 6.3, 'gauge', True),
    ('故障码 Fault', '', 0, 0, '', False),
    ('警告码 Warn', '', 0, 0, '', False),
    ('顶层状态 TopFSM', '', 0, 0, '', False),
    ('运行状态 RunState', '', 0, 0, '', False),
    ('控制模式 CtrlMode', '', 0, 0, '', False),
    ('使能 Enable', '', 0, 1, '', False),
]

groups_def = [
    ("运动 (0xC0/C6/C7)", [0, 1, 2, 14, 13]),
    ("电气电流 (0xC2/C3)", [8, 9, 10, 11, 12]),
    ("母线功率 (0xC4)", [5, 6, 7]),
    ("温度 (0xC5)", [3, 4]),
    ("状态故障 (0xC1/C8)", [17, 18, 19, 20, 15, 16]),
]

uid = 10000


def dataset(idx, gid, f):
    global uid
    title, unit, mn, mx, widget, graph = f
    d = {"datasetId": 0, "displayFormat": "0d", "displayTickCount": 5, "fft": False,
         "fftMax": 0, "fftMin": 0, "fftSamples": 1024, "fftSamplingRate": -1,
         "graph": graph, "groupId": gid, "index": idx + 1, "led": False, "ledHigh": 1,
         "log": False, "numericValue": 0, "plotMax": mx, "plotMin": mn, "title": title,
         "uniqueId": uid, "units": unit, "value": "--.--", "widget": widget,
         "widgetMax": mx, "widgetMin": mn, "xAxis": -1}
    uid += 1
    return d


groups = []
for gid, (gtitle, idxs) in enumerate(groups_def):
    ds = [dataset(i, gid, fields[i]) for i in idxs]
    for j, d in enumerate(ds):
        d["datasetId"] = j
    groups.append({"datasets": ds, "title": gtitle, "uniqueId": 10003 + gid, "widget": ""})

proj = {
    "actions": actions,
    "groups": groups,
    "hexadecimalDelimiters": False,
    "mqttPublisher": {"cleanSession": True, "customClientId": False, "enabled": False,
                      "hostname": "127.0.0.1", "keepAlive": 60, "mode": 0, "mqttVersion": 2,
                      "notificationTopic": "", "peerVerifyDepth": 10, "peerVerifyMode": 3,
                      "port": 1883, "publishFrequency": 10, "publishNotifications": False,
                      "scriptCode": "", "scriptLanguage": 0, "scriptTopic": "",
                      "sslEnabled": False, "sslProtocol": 5, "topicBase": ""},
    "nextUniqueId": uid + 1,
    "plotTimeRange": 10, "pointCount": 100, "schemaVersion": 3,
    "sources": [{"busType": 0, "checksum": "", "checksumAlgorithm": "",
                 "connection": {"autoReconnect": False, "baudRate": 115200, "dataBitsIndex": 3,
                                "deviceId": {"description": "USB 串行设备", "pid": "0204",
                                             "portName": "COM3", "serial": "EF8B8A0E0949B1EF01",
                                             "vid": "0D28"},
                                "dtr": True, "flowControlIndex": 0, "parityIndex": 0,
                                "portIndex": 1, "stopBitsIndex": 0},
                 "decoder": 3, "decoderMethod": 3, "frameDetection": 2, "frameEnd": "",
                 "frameParserCode": parser, "frameStart": "", "hexadecimalDelimiters": False,
                 "sourceId": 0, "title": "设备 A"}],
    "title": "关节电机 JointMotor UART",
    "writerVersion": "4.0.1", "writerVersionAtCreation": "4.0.1"
}

if __name__ == "__main__":
    import sys
    path = sys.argv[1] if len(sys.argv) > 1 else "jointmotor_uart.ssproj"
    with open(path, "w", encoding="utf-8") as f:
        json.dump(proj, f, ensure_ascii=False, indent=4)
    print("written", path)
    print("actions:", len(actions), "groups:", len(groups),
          "datasets:", sum(len(g["datasets"]) for g in groups))
