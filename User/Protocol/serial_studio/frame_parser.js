/**
 * 关节电机串口反馈解析器 (Serial Studio - JavaScript)
 *
 * 解码器: Binary (decoder=3)  帧检测: No Delimiters (frameDetection=2)
 * parse(frame) 收到的是"本次到达的字节数组"(0~255)，不是字符串。
 * No-Delimiters 模式下 Serial Studio 不做分帧，必须在脚本里自行缓存+拆帧。
 *
 * 帧格式 (与固件 packer_parser.c 一致):
 *   A5 5A | LEN_H LEN_L | HDR_CHK | [CMD + DATA] | CRC_L CRC_H
 *   LEN     = CMD(1)+DATA(n) 总字节数, 大端
 *   HDR_CHK = (0xA5 + 0x5A + LEN_H + LEN_L) & 0xFF
 *   CRC16   = CRC-16/XMODEM(poly 0x1021, init 0x0000) 覆盖数据区(CMD+DATA), 小端落帧
 *
 * 仅解析实时反馈帧 0xC0 (串口应答, 22字节载荷):
 *   {pos:f32, vel:f32, torque:f32, temp:f32, volt:f32, err:u16}
 * 返回值映射到数据集 Frame Index 1..6。
 */

var __buf = [];          // 跨帧字节缓存
var __crcTab = null;     // CRC 查表(首次惰性构建)

var FB_CMD   = 0xC0;     // 实时反馈命令码
var FB_DLEN  = 1 + 22;   // CMD + 22字节载荷
var MAX_DLEN = 512;      // 数据区长度上限, 超出视为错位
var BUF_CAP  = 4096;     // 缓存上限, 防异常增长

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

function rdF32(buf, o) {
    var dv = new DataView(new ArrayBuffer(4));
    dv.setUint8(0, buf[o]); dv.setUint8(1, buf[o + 1]);
    dv.setUint8(2, buf[o + 2]); dv.setUint8(3, buf[o + 3]);
    return dv.getFloat32(0, true);   // 小端
}

function rdU16(buf, o) {
    return (buf[o] | (buf[o + 1] << 8)) & 0xFFFF;
}

function parse(frame) {
    var i;
    for (i = 0; i < frame.length; i++) __buf.push(frame[i] & 0xFF);
    if (__buf.length > BUF_CAP) __buf = __buf.slice(__buf.length - BUF_CAP);

    var rows = [];
    while (true) {
        // 1) 对齐帧头 A5 5A
        while (__buf.length >= 2 && !(__buf[0] === 0xA5 && __buf[1] === 0x5A))
            __buf.shift();
        if (__buf.length < 5) break;                       // 头部未齐

        var lenH = __buf[2], lenL = __buf[3];
        var dlen = (lenH << 8) | lenL;                     // 数据区长度
        if (dlen > MAX_DLEN) { __buf.shift(); continue; }  // 长度异常, 错位

        var total = 5 + dlen + 2;                          // 头(5)+数据+CRC(2)
        if (__buf.length < total) break;                   // 整帧未到齐

        var hdrChk = (0xA5 + 0x5A + lenH + lenL) & 0xFF;
        if (__buf[4] !== hdrChk) { __buf.shift(); continue; }

        var rxCrc = __buf[5 + dlen] | (__buf[6 + dlen] << 8);
        var ok = (crc16(__buf, 5, dlen) === rxCrc);
        var data = __buf.slice(5, 5 + dlen);               // data[0]=CMD
        __buf = __buf.slice(total);                        // 消费整帧
        if (!ok) continue;

        if (data.length >= FB_DLEN && data[0] === FB_CMD) {
            rows.push([
                rdF32(data, 1),    // pos    rad
                rdF32(data, 5),    // vel    rad/s
                rdF32(data, 9),    // torque Nm
                rdF32(data, 13),   // temp   °C
                rdF32(data, 17),   // volt   V
                rdU16(data, 21)    // err    bitmask
            ]);
        }
    }

    if (rows.length === 0) return [];
    if (rows.length === 1) return rows[0];
    return rows;   // 多帧一次返回 (2D)
}
