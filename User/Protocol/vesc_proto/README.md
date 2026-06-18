# vesc_proto —— 可移植 VESC 串行协议模块（从机）

让 MCU 被 VESC Tool 上位机识别为一个 VESC 设备。平台无关，通过回调注入对接你的 USART。

## 文件清单

| 文件 | 必须 | 说明 |
|------|:----:|------|
| `vesc_proto.h` / `vesc_proto.c` | ✅ | 帧层引擎：组帧/拆帧、CRC16、收发状态机 |
| `vesc_comm_ids.h` | ✅ | 完整命令 ID 枚举（被 vesc_proto.h 引用） |
| `vesc_slave.h` / `vesc_slave.c` | ✅ | 从机层：FW_VERSION 握手 + GET_VALUES 实时值回复 |
| `example_slave.c` | ❌ | 参考示例，不参与编译 |
| `README.md` / `VESC串行通信协议.md` | ❌ | 文档 |

移植进工程时编译 **4 个文件**：`vesc_proto.c`、`vesc_slave.c` 及两个对应头文件 + `vesc_comm_ids.h`。

## 三步对接 USART

### 1. 实现回调并初始化

```c
#include "vesc_slave.h"

static vesc_slave_t s_slave;

/* 把字节写到你的 USART */
static void cb_send(const uint8_t *data, uint16_t len, void *ctx) {
    (void)ctx;
    HAL_UART_Transmit(&huart1, (uint8_t *)data, len, 100);  // 换成你的发送函数
}

/* 填充仪表盘实时值（用关节电机的真实数据替换）*/
static void cb_get_values(vesc_values_t *v, void *ctx) {
    (void)ctx;
    v->temp_fet      = get_driver_temp();
    v->temp_motor    = get_motor_temp();
    v->current_motor = get_motor_current();
    v->v_in          = get_bus_voltage();
    v->rpm           = get_erpm();
    v->fault_code    = 0;
}

void slave_setup(void) {
    vesc_slave_cfg_t cfg = {0};
    cfg.send_bytes = cb_send;        // 必填
    cfg.get_values = cb_get_values;  // 必填
    cfg.hw_name    = "MyJoint";      // VESC Tool 上显示的硬件名
    cfg.fw_major   = 6;
    cfg.fw_minor   = 0;
    // cfg.uuid 可填芯片唯一 ID
    vesc_slave_init(&s_slave, &cfg);
}
```

### 2. 在串口接收处喂数据

中断/DMA 收到一批字节后调用，回复自动完成：

```c
void on_uart_rx_batch(uint8_t *buf, uint16_t len) {
    vesc_slave_recv(&s_slave, buf, len);
}
```

### 3. 连接 VESC Tool

USART TX/RX 交叉接到上位机串口（共地），波特率默认 115200。VESC Tool 打开端口后：

```
VESC Tool ── COMM_FW_VERSION ──▶  本模块回设备身份 → 被识别
          ── COMM_GET_VALUES ──▶  调 get_values 回调 → 仪表盘实时显示
```

## 扩展其他命令

VESC Tool 的配置页会发 `COMM_GET_MCCONF` 等命令。这些请求会落到可选的 `user_packet` 回调：

```c
static void cb_user(vesc_proto_t *vp, uint8_t cmd_id,
                    const uint8_t *body, uint16_t len, void *ctx) {
    switch (cmd_id) {
    case COMM_GET_MCCONF:
        // 按 confgenerator 格式组装 mcconf 回复...
        break;
    }
}
// cfg.user_packet = cb_user;
```

> 配置页要完整工作需移植固件 confgenerator 的序列化逻辑（几百个字段）。
> 仅"被识别 + 看实时数据"则只需 FW_VERSION + GET_VALUES，本模块已内置。

## 编译期配置

| 宏 | 默认 | 说明 |
|----|------|------|
| `VESC_PROTO_MAX_PL` | `512` | 最大 payload 长度，需与上位机一致。减小可省 RAM |

每个实例约占 `2 * (VESC_PROTO_MAX_PL + 8)` 字节 RAM（收发缓冲），默认约 1KB。

## 线程安全

- 回调在 `vesc_slave_recv()` 调用栈内同步触发，避免在回调里做耗时操作。
- 模块不内置锁；若收发在不同中断/线程，需自行加临界区。

## 协议细节

完整帧格式、CRC、命令列表见 [VESC串行通信协议.md](VESC串行通信协议.md)。
