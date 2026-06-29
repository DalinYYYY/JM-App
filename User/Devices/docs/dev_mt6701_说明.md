# MT6701 磁编码器驱动说明

> 适用文件：[dev_mt6701.c](../dev_mt6701.c) / [dev_mt6701.h](../dev_mt6701.h)
> 适用板卡：SFOC（MT6701 挂 SPI1）；V1 板的 SPI1 为另一套配置，需单独核对
> 版本：1.0　日期：2026-06-29

---

## 1. 芯片概述

MT6701 是 MagnTek 的差分霍尔磁旋转编码器，14 位绝对角度（2^14 = 16384 计数/圈，分辨率约 0.022°）。本工程使用其 **SSI（同步串行）模式**，由 MCU 作 SPI 主机读取。

| 项目 | 值 |
|---|---|
| 角度分辨率 | 14 bit / 16384 计数每圈 |
| 接口模式 | SSI（SPI 主机读，只读 MISO） |
| 单帧长度 | 24 bit（角度 14 + 磁场 4 + CRC 6） |
| CRC 多项式 | X⁶ + X + 1 |
| CRC 覆盖范围 | 前 18 位（角度 14 + 磁场 4） |

---

## 2. SSI 帧格式（24 bit，MSB 先出）

```
bit:  23 ............ 10 | 9 .. 6 | 5 ...... 0
内容:  [   角度 14bit   ] | [Mg 4 ] | [ CRC 6  ]
```

- **角度 [23:10]**：14 位绝对角度，`0 ~ 16383` 对应 `0 ~ 360°`。
- **磁场 Mg [9:6]**：磁场状态，见下表。
- **CRC [5:0]**：对前 18 位的 CRC6 校验。

### 磁场状态 Mg[3:0] 解析

| 位 | 含义 |
|---|---|
| Mg[1:0] | 0=正常　1=磁场过强　2=磁场过弱　3=无效 |
| Mg[2] | 按键按下（叠加位） |
| Mg[3] | 超速（叠加位） |

> 解析见 [mt6701_parse_mg_state()](../dev_mt6701.c)。磁铁安装正常且静止时，`mg_state` 应为 0（NORMAL）。

---

## 3. ⚠️ 关键陷阱：帧前导位导致整帧右移

这是本驱动调试中遇到的核心问题，**务必牢记**。

### 现象

- 角度恒落在 **180°~360°**，进不去 0~180°（即 14 位角度最高位 D13 恒为 1）。
- 或：角度能扫满 0~360，但 **CRC 校验全部失败**，`err_cnt` 快速饱和。

### 根因

CSN 拉低后、真正的角度 MSB 出现之前，DO 线处于**空闲高电平**。若按固定 3 字节（24 clk）读取，SPI 会把这个空闲位当成第一个有效 bit，导致**整条 24 位数据流右移一位**：

- D13 永远采到前导高电平 → 角度恒 ≥180°；
- 18 位数据与 6 位 CRC 全部错位 → CRC 必然失败。

### ❌ 走过的弯路（请勿重复）

| 错误猜测 | 反证 |
|---|---|
| 改 SPI 采样边沿 1EDGE→2EDGE | 2EDGE 数据全乱，反证 **1EDGE 本就是正确边沿** |
| 关闭 NSSP | 与帧偏移无关，已撤回 |

**结论：问题不在 SPI 边沿，不在位拼接公式，不在分辨率常数。**

### ✅ 正确解法：CRC 自校验帧对齐

见 [dev_mt6701_get_raw()](../dev_mt6701.c)：

1. **多读 1 字节**（读 4 字节 / 32 clk），把可能的前导位也读进来。
2. 4 字节拼成 32 位流，在 **0~8 bit 偏移**范围内逐个试解码。
3. **用 CRC 通过与否作为帧边界判据**——哪个偏移让 CRC 通过就用哪个（CRC 在此同时充当“帧对齐校验器”）。
4. 命中的偏移锁存到 `bit_offset`，下次优先尝试，稳态下一次命中、不浪费 CPU。

```c
/* 偏移 offset 时，24 位帧 = word32 右移 (8 - offset) 后的低 24 位 */
frame = (word32 >> (8 - offset)) & 0xFFFFFF;
```

实测正常工作时：`bit_offset` 稳定（通常为 1），`err_cnt = 0`，角度跑满 0~360°。

---

## 4. CRC6 校验

见 [mt6701_crc6_check()](../dev_mt6701.c)。

- 使用 MagnTek **官方 6-bit 分组查表法**，配套 `tableCRC6[64]`。
- 把 18 位输入拆成 3 个 6-bit 组依次过表：

```c
idx = (data >> 12) & 0x3F;                 // 高 6 位
idx = ((data >> 6) & 0x3F) ^ tableCRC6[idx];   // 中 6 位
idx = (data & 0x3F)        ^ tableCRC6[idx];   // 低 6 位
crc = tableCRC6[idx];                      // 与帧内 CRC 比较
```

> ⚠️ **不能用逐 bit 移位查表**（`(crc<<1)|bit`）：那是另一套表的算法，与本 `tableCRC6` 不匹配，会导致每帧 CRC 失败。这是本驱动早期 `crc_check` 恒为 1 的原因。

---

## 5. 角度处理链路

```
14bit raw → raw_deg(°) → 去零点offset → 方向(CW/CCW) → mechanical_angle [0,360)°
                                                              │
                              motor_loop.c publish_motion ────┘ → ×DEG2RAD → 运动层(rad)
```

- **驱动层** [dev_mt6701_get_machAngle()](../dev_mt6701.c) 输出**度** `[0, 360)`。
- **运动层** [motor_loop.c](../../MotorControl/CascadeControl/motor_loop.c) `publish_motion()` 统一转 **rad** 后写入 `motor_motion_t`。
- 协议上报的 `single`/`pos`（0xC6/0xC7/0xCA）已是 **rad**。

### CRC 门控

[dev_mt6701_get_machAngle()](../dev_mt6701.c) 在 `crc_check != 0` 时**丢弃该帧**：不更新 `mechanical_angle`，保持上一帧有效值，并累计 `err_cnt`。这样单帧 SPI 误码不会污染角度（旋转中表现为尖峰/跳变）。

---

## 6. 对象字段（dev_mt6701_t）

| 字段 | 含义 |
|---|---|
| `raw_buf[4]` | SPI 读回的 4 字节原始帧（多读 1B 容纳前导位） |
| `raw` | 14bit 原始角度值 |
| `mech_angle_org` | 去偏移/方向前的原始角度（°） |
| `mechanical_angle` | 最终机械角度（°，`[0,360)`） |
| `mg_state` | 磁场状态（Mg[3:0] 解析，可叠加） |
| `crc_code` | 帧内 6bit CRC |
| `crc_check` | CRC 结果（0=通过，1=失败） |
| `err_cnt` | 连续 CRC 坏帧计数（收到有效帧清零） |
| `bit_offset` | SSI 帧前导 bit 偏移（0~8，由 CRC 自校验锁定） |
| `offset` | 零点偏移量（°） |
| `dir` | 旋转方向（CW/CCW） |

---

## 7. API

| 函数 | 说明 |
|---|---|
| `dev_mt6701_init(pobj, id)` | 初始化对象，绑定方法表 |
| `update(pobj)` | 读原始帧并刷新机械角度（主循环周期调用） |
| `set_zero_angle(pobj, deg)` | 设零点：令当前位置显示为 `deg`，成功返回 true |
| `get_zero_angle(pobj)` | 刷新并返回当前机械角度（°） |
| `calibrate_zero(pobj)` | 将当前位置标定为 0° |
| `set_dir/get_dir` | 设置/读取旋转方向 |

资源（SPI 句柄、CSN 引脚）在 `Board/<板名>/Config/dev_config_board.inc` 的 `mt6701_list[]` 配置。

---

## 8. 调试排查顺序（角度异常时）

按此顺序查，**不要先动 SPI 边沿**：

1. 看 `err_cnt`：持续增长 → CRC 失败，进入第 2 步；恒为 0 → CRC 正常，查机械/装配。
2. 看 `bit_offset`：能稳定锁定（如 1）→ 帧对齐 OK；在 0~8 间乱跳或恒为失败标记 → 见第 3 步。
3. 抓一帧 `raw_buf[0..3]` 十六进制 + 已知真实角度，反推 CRC 参数是否匹配本芯片批次。
4. 确认 SPI1 为 **8bit / Mode0(CPOL=0,CPHA=1EDGE) / MSB**（SFOC 板基线配置）。

---

## 参考

- [MT6701 数据手册 Rev.1.8](http://blog.dzl.dk/wp-content/uploads/2024/01/MT6701_Rev.1.8.pdf)
- [smartknob MT6701 驱动](https://github.com/scottbez1/smartknob/blob/master/firmware/src/mt6701_sensor.cpp)
- [I-AM-ENGINEER/MT6701-driver](https://github.com/I-AM-ENGINEER/MT6701-driver)
- [SimpleFOC Arduino-FOC-drivers MT6701](https://github.com/simplefoc/Arduino-FOC-drivers/blob/master/src/encoders/mt6701/README.md)
