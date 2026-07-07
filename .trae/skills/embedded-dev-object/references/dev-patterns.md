# dev 设备对象层代码模板

> 本文件是 `SKILL.md` 的参考资料，提供新设备骨架、配置表填充、抽象接口适配、组合装配、通信设备四段式、CRC/滤波/标定算法等代码模板。所有模板从 `User/Devices/` 既有设备提炼。

## 1. 新设备骨架模板

以一个假想的 `dev_xxx` 设备为例，展示完整的最小骨架。新增设备时按此结构填充。

### 1.1 板级使能

```c
/* Board/SFOC/Config/dev_config_board.h */
#define USE_DEV_XXX
```

```c
/* dev_config.h —— 设备相关常量 */
#if defined(USE_DEV_XXX)
#define XXX_RESOLUTION  (4096.0f)
#define XXX_VREF        (3.3f)
#endif
```

### 1.2 dev_xxx.h 公共头

```c
/**
 * @file        dev_xxx.h
 * @brief       XXX设备(简述功能)
 *
 * @author      Dalin (dalinyy@163.com)
 * @version     1.0
 * @date        2026-06-XX
 *
 * @copyright   Copyright (c) 2026 RuidiculousTech.co, Ltd. All rights reserved.
 *
 * @par 修改日志:
 * | 日期       | 版本 | 作者   | 修改内容   |
 * |------------|------|--------|------------|
 * | 2026-06-XX | 1.0  | Dalin  | 初始创建   |
 *
 * @note        本文件遵循《嵌入式C代码规范V1.0》开发
 * @note        资源配置在 dev_config_board.inc 的 xxx_list 填表
 * @note        init只装配接口; 硬件启动在 start 中完成
 */
#ifndef __DEV_XXX_H_
#define __DEV_XXX_H_

#include "dev_config.h"
#if defined(USE_DEV_XXX)

#include "drv_spi.h"   /* 按需 include drv 层头 */
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 设备编号枚举 */
typedef enum {
    XXX_ID_1 = 0,
    XXX_ID_MAX,
} xxx_id_e;

/* 状态枚举 */
typedef enum {
    XXX_STATE_IDLE = 0,
    XXX_STATE_RUNNING,
} xxx_state_e;

/**
 * @brief 资源配置(在 dev_config_board.inc 的 xxx_list 填表)
 * @param  name    : 设备名(调试用)
 * @param  spi_num : SPI外设编号
 * @param  csn     : 片选引脚
 */
typedef struct {
    char name[20];
    spiDrv_t spi_num;
    gpioDrv_t csn;
} xxx_config_t;

/* 配置表定义在板级 .inc */
extern const xxx_config_t xxx_list[XXX_ID_MAX];

/**
 * @brief XXX设备对象
 * @param  id              : 设备编号
 * @param  state           : 设备状态
 * @param  raw             : 原始数据
 * @param  value           : 物理量(已转换)
 * @param  offset          : 零点偏移
 * @param  start           : 启动设备, 返回 DEV_EOK/DEV_ERROR
 * @param  stop            : 停止设备
 * @param  update          : 刷新数据
 * @param  get_value       : 取物理量
 * @param  set_zero        : 设置零点
 */
typedef struct dev_xxx {
    xxx_id_e id;
    xxx_state_e state;
    uint32_t raw;
    float value;
    float offset;

    /* public 方法指针 */
    int  (*start)(struct dev_xxx *pobj);
    int  (*stop)(struct dev_xxx *pobj);
    void (*update)(struct dev_xxx *pobj);
    float (*get_value)(struct dev_xxx *pobj);
    void (*set_zero)(struct dev_xxx *pobj, float zero);
} dev_xxx_t;

/**
 * @brief 初始化 XXX 设备对象
 * @note  init只装配接口; 硬件启动在 start 中完成
 */
void dev_xxx_init(dev_xxx_t *pobj, xxx_id_e id);

#ifdef __cplusplus
}
#endif
#endif /* USE_DEV_XXX */
#endif /* __DEV_XXX_H_ */
```

### 1.3 dev_xxx.c 实现

```c
/**
 * @file        dev_xxx.c
 * @brief       XXX设备实现
 * ...（头注释与 .h 一致）
 */
#include "dev_xxx.h"

#if defined(USE_DEV_XXX)
#include "drv_spi.h"
#include "drv_gpio.h"
#include "assert_report.h"
#include <math.h>

/* ===== 内部工具函数 ===== */

static inline float xxx_norm360(float deg)
{
    deg = fmodf(deg, 360.0F);
    return (deg < 0.0F) ? (deg + 360.0F) : deg;
}

/* ===== 方法实现 ===== */

static int dev_xxx_start(struct dev_xxx *pobj)
{
    assert_report(pobj != NULL);
    const xxx_config_t *cfg = &xxx_list[pobj->id];
    /* 启动硬件... */
    pobj->state = XXX_STATE_RUNNING;
    return DEV_EOK;
}

static int dev_xxx_stop(struct dev_xxx *pobj)
{
    assert_report(pobj != NULL);
    pobj->state = XXX_STATE_IDLE;
    return DEV_EOK;
}

static void dev_xxx_update(struct dev_xxx *pobj)
{
    assert_report(pobj != NULL);
    const xxx_config_t *cfg = &xxx_list[pobj->id];
    /* 读硬件 → raw → value */
    pobj->value = xxx_norm360((float)pobj->raw / XXX_RESOLUTION * 360.0F - pobj->offset);
}

static float dev_xxx_get_value(struct dev_xxx *pobj)
{
    assert_report(pobj != NULL);
    return pobj->value;
}

static void dev_xxx_set_zero(struct dev_xxx *pobj, float zero)
{
    assert_report(pobj != NULL);
    pobj->offset = xxx_norm360(pobj->value - zero);
}

/* ===== 构造函数 ===== */

void dev_xxx_init(dev_xxx_t *pobj, xxx_id_e id)
{
    assert_report(pobj != NULL);
    assert_report(id < XXX_ID_MAX);
    memset(pobj, 0, sizeof(dev_xxx_t));

    pobj->id = id;
    pobj->state = XXX_STATE_IDLE;
    pobj->offset = 0.0F;

    /* 装配方法指针 */
    pobj->start = dev_xxx_start;
    pobj->stop = dev_xxx_stop;
    pobj->update = dev_xxx_update;
    pobj->get_value = dev_xxx_get_value;
    pobj->set_zero = dev_xxx_set_zero;
}

#endif /* USE_DEV_XXX */
```

### 1.4 板级配置表

```c
/* Board/SFOC/Config/dev_config_board.inc */
#ifdef JM_BOARD_CONFIG_DEFINE_TABLES
#if defined(USE_DEV_XXX)
const xxx_config_t xxx_list[XXX_ID_MAX] = {
    [XXX_ID_1] = {
        .name = "XXX_1",
        .spi_num = {.hspi = DRV_SPI1},
        .csn = {.gpiox = DRV_GPIOA, .pin = DRV_PIN_4, .ste = DRV_PIN_LOW},
    },
};
#endif
#endif
```

### 1.5 dev_config.c 注册

```c
/* dev_config.c */
#define JM_BOARD_CONFIG_DEFINE_TABLES
#include "dev_config.h"
#include "dev_xxx.h"   /* 新增 */
/* ... 其他设备头 ... */
#include JM_BOARD_DEV_CONFIG_INC
```

## 2. 配置表填充模板

### 2.1 简单设备（单通道）

```c
const dev_power_monitor_config_t power_monitor_list[PM_CH_MAX] = {
    [PM_IBUS] = {.name = "IBUS", .id = DRV_ADC_1, .channel = DRV_ADC_CH12},
    [PM_VBUS] = {.name = "VBUS", .id = DRV_ADC_3, .channel = DRV_ADC_CH12},
};
```

### 2.2 多通道设备（数组字段）

```c
const dev_half_bridge_config_t half_bridge_list[BRIDGE_ID_MAX] = {
    [BRIDGE_DEV1] = {
        .name = "HALF_BRIDGE_1",
        .tim = DRV_TIM1,
        .channel = {TIM_CH1, TIM_CH2, TIM_CH3, TIM_CH4},
    },
};
```

### 2.3 带字符串字段

```c
const dev_commun_vesc_config_t commun_vesc_list[VESC_COMM_ID_MAX] = {
    [VESC_COMM_ID_1] = {
        .name = "VESC_UART_1",
        .uart = DRV_UART2,
        .hw_name = "JointMotor V1",
        .fw_name = "JM-FW",
        .fw_major = 1,
        .fw_minor = 0,
    },
};
```

### 2.4 RGB LED（多引脚）

```c
const dev_rgb_led_config_t rgb_led_list[RGB_LED_ID_MAX] = {
    [RGB_LED_ID_1] = {
        .name = "RGB_LED_1",
        .r = {.gpiox = DRV_GPIOC, .pin = DRV_PIN_13, .ste = DRV_PIN_LOW},
        .g = {.gpiox = DRV_GPIOC, .pin = DRV_PIN_14, .ste = DRV_PIN_LOW},
        .b = {.gpiox = DRV_GPIOC, .pin = DRV_PIN_15, .ste = DRV_PIN_LOW},
        .polarity = LED_ACTIVE_HIGH,
    },
};
```

## 3. 抽象接口适配模板

### 3.1 抽象接口定义

```c
/* dev_motor.h —— 抽象编码器接口 */
typedef struct dev_encoder {
    void *ctx;                // 指向具体编码器对象
    float mechanical_angle;   // 最新机械角度
    void (*update)(struct dev_encoder *pobj);
    float (*get_mechanical_angle)(struct dev_encoder *pobj);
} dev_encoder_t;
```

### 3.2 适配函数（每种芯片一套）

```c
/* dev_motor.c —— MT6701 适配 */
#if (DEV_MOTOR_ENCODER_TYPE == DEV_MOTOR_ENCODER_MT6701)
static void encoder_mt6701_update(struct dev_encoder *enc)
{
    dev_mt6701_t *chip = (dev_mt6701_t *)enc->ctx;
    chip->update(chip);
    enc->mechanical_angle = chip->mechanical_angle;
}
static float encoder_mt6701_get_mechanical_angle(struct dev_encoder *enc)
{
    return enc->mechanical_angle;
}

/* MT6835 适配 */
#elif (DEV_MOTOR_ENCODER_TYPE == DEV_MOTOR_ENCODER_MT6835)
static void encoder_mt6835_update(struct dev_encoder *enc)
{
    dev_mt6835_t *chip = (dev_mt6835_t *)enc->ctx;
    chip->update(chip);
    enc->mechanical_angle = chip->get_mechanical_angle(chip);
}
static float encoder_mt6835_get_mechanical_angle(struct dev_encoder *enc)
{
    return enc->mechanical_angle;
}
#else
#error "未知的 DEV_MOTOR_ENCODER_TYPE"
#endif
```

### 3.3 装配（在组合设备 init 中）

```c
#if (DEV_MOTOR_ENCODER_TYPE == DEV_MOTOR_ENCODER_MT6701)
    dev_mt6701_init(&pobj->mt6701, (mt6701_id_e)id);
    pobj->mt6701.set_zero_angle(&pobj->mt6701, 0.0F);
    pobj->encoder.ctx = &pobj->mt6701;
    pobj->encoder.update = encoder_mt6701_update;
    pobj->encoder.get_mechanical_angle = encoder_mt6701_get_mechanical_angle;

#elif (DEV_MOTOR_ENCODER_TYPE == DEV_MOTOR_ENCODER_MT6835)
    dev_mt6835_init(&pobj->mt6835, (mt6835_id_e)id);
    pobj->mt6835.set_zero_angle(&pobj->mt6835, 0.0F);
    pobj->encoder.ctx = &pobj->mt6835;
    pobj->encoder.update = encoder_mt6835_update;
    pobj->encoder.get_mechanical_angle = encoder_mt6835_get_mechanical_angle;
#endif
```

## 4. 组合装配模板

```c
/* dev_motor.c —— 组合多子设备 */
void dev_motor_init(dev_motor_t *pobj, motor_id_e id,
                    focCurrent_t (*current_callback)(void),
                    float (*ele_radian_callback)(void))
{
    assert_report(pobj != NULL);
    assert_report(current_callback != NULL);
    assert_report(ele_radian_callback != NULL);
    assert_report(id < DEV_MOTOR_MAX);
    memset(pobj, 0, sizeof(dev_motor_t));

    dev_motor_disable();   // 默认上电禁能

    pobj->id = id;
    pobj->poles = dev_motor_get_poles(id);
    pobj->fsm_tim = DRV_TIM2;

    /* 1. 编码器（型号由宏选择，控制层不感知） */
#if (DEV_MOTOR_ENCODER_TYPE == DEV_MOTOR_ENCODER_MT6701)
    dev_mt6701_init(&pobj->mt6701, (mt6701_id_e)id);
    pobj->mt6701.set_zero_angle(&pobj->mt6701, 0.0F);
    pobj->encoder.ctx = &pobj->mt6701;
    pobj->encoder.update = encoder_mt6701_update;
    pobj->encoder.get_mechanical_angle = encoder_mt6701_get_mechanical_angle;
#endif

    /* 2. 角度转化器 */
    motion_param_init(&pobj->motor_param, pobj->poles, 10, NULL);

    /* 3. 多圈计数（设备补偿回调注入） */
    multiturn_config_t mt_cfg;
    memset(&mt_cfg, 0, sizeof(mt_cfg));
    mt_cfg.mode = MULTITURN_MODE_SOFT;
    mt_cfg.device_compensation_callback = device_compensation;
    multiturn_init(&pobj->multiturn, &mt_cfg);

    /* 4. 半桥 */
    dev_half_bridge_init(&pobj->half_bridge, (half_bridge_id_e)id);

    /* 5. 相电流 */
    dev_phase_current_init(&pobj->phase_current, PHASE_CURRENT_GAIN, PHASE_CURRENT_SHUNT);
    pobj->phase_current.set_offset(&pobj->phase_current,
        (dev_current_i3axis_t){PHASE_CURRENT_ZERO_ADC, PHASE_CURRENT_ZERO_ADC, PHASE_CURRENT_ZERO_ADC});

    /* 6. FOC（注入外部回调） */
    pobj->current_callback = current_callback;
    pobj->ele_radian_callback = ele_radian_callback;
    foc_init(&pobj->foc, pobj->current_callback, pobj->ele_radian_callback);

    dev_motor_enable();
}
```

## 5. 通信设备四段式模板

### 5.1 设备对象

```c
typedef struct dev_commun_uart {
    jm_proto_uart_t jm;          // 协议栈实例（私有，首成员）
    usartNumber_e uart;
    uint8_t started;
    uint8_t rx_tmp[DEV_JM_UART_RX_BUF_SIZE];
    int last_error;
    uint32_t poll_count;         // 调试计数

    const jm_proto_ops_t *ops;   // 业务回调（应用注入）

    void (*set_ops)(struct dev_commun_uart *pobj, const jm_proto_ops_t *ops);
    int  (*start)(struct dev_commun_uart *pobj);
    void (*on_rx_idle)(struct dev_commun_uart *pobj);
    void (*poll)(struct dev_commun_uart *pobj);
    void (*report)(struct dev_commun_uart *pobj, uint8_t cmd, const uint8_t *body, uint16_t len);
} dev_commun_uart_t;
```

### 5.2 四段式实现

```c
/* set_ops: 注入业务回调 */
static void dev_commun_uart_set_ops(struct dev_commun_uart *pobj, const jm_proto_ops_t *ops)
{
    assert_report(pobj != NULL);
    pobj->ops = ops;
}

/* start: 启动空闲中断+DMA接收 */
static int dev_commun_uart_start(struct dev_commun_uart *pobj)
{
    assert_report(pobj != NULL);
    if (pobj->started) return DEV_EOK;
    if (usart_idle_init(pobj->uart, DEV_JM_UART_RX_BUF_SIZE) != DEV_EOK) {
        pobj->last_error = DEV_ERROR;
        return DEV_ERROR;
    }
    pobj->started = 1;
    pobj->start_count++;
    return DEV_EOK;
}

/* on_rx_idle: 串口 ISR 中调用 */
static void dev_commun_uart_on_rx_idle(struct dev_commun_uart *pobj)
{
    assert_report(pobj != NULL);
    if (!pobj->started) return;
    drv_uart_idle(pobj->uart);   // 检测 IDLE 并落数据
    pobj->idle_irq_count++;
}

/* poll: 主循环调用，取数据喂协议栈 */
static void dev_commun_uart_poll(struct dev_commun_uart *pobj)
{
    assert_report(pobj != NULL);
    pobj->poll_count++;
    if (!pobj->started) { pobj->poll_not_started_count++; return; }

    uint16_t len = 0;
    if (usart_idle_get_data(pobj->uart, pobj->rx_tmp, &len) != DEV_EOK || len == 0)
        return;

    pobj->poll_rx_count++;
    pobj->poll_rx_bytes += len;
    /* 喂协议栈（协议栈实例为首成员，可转型） */
    jm_proto_uart_feed(&pobj->jm, pobj->rx_tmp, len);
    jm_proto_uart_process(&pobj->jm);   // 自动回复
}

/* report: 主动上报 */
static void dev_commun_uart_report(struct dev_commun_uart *pobj, uint8_t cmd,
                                    const uint8_t *body, uint16_t len)
{
    assert_report(pobj != NULL);
    uint8_t tx_buf[DEV_JM_UART_RX_BUF_SIZE];
    uint16_t tx_len = jm_proto_uart_build(&pobj->jm, cmd, body, len, tx_buf, sizeof(tx_buf));
    if (tx_len > 0) {
        drv_uart_send(pobj->uart, tx_buf, tx_len, 100);
        pobj->tx_count++;
    }
}

void dev_commun_uart_init(dev_commun_uart_t *pobj, jm_uart_comm_id_e id)
{
    assert_report(pobj != NULL);
    memset(pobj, 0, sizeof(dev_commun_uart_t));
    const dev_commun_uart_config_t *cfg = &commun_uart_list[id];
    pobj->uart = cfg->uart;
    pobj->motor_id = cfg->motor_id;
    jm_proto_uart_init(&pobj->jm, cfg->motor_id);   // 协议栈实例初始化

    pobj->set_ops = dev_commun_uart_set_ops;
    pobj->start = dev_commun_uart_start;
    pobj->on_rx_idle = dev_commun_uart_on_rx_idle;
    pobj->poll = dev_commun_uart_poll;
    pobj->report = dev_commun_uart_report;
}
```

## 6. CRC 校验与坏帧保护模板

以 MT6701 的 CRC6 校验为例：

```c
/* CRC6 查表(多项式 X^6+X+1) */
static const uint8_t tableCRC6[64] = {
    0x00, 0x03, 0x06, 0x05, 0x0C, 0x0F, 0x0A, 0x09,
    /* ... 64 项 ... */
};

/*
 * @brief CRC6校验
 * @param data_word : 24位字 [23:10]角度 [9:6]Mg [5:0]CRC
 * @return 0=通过, 1=失败
 * @note  须把18位输入拆成3个6-bit组依次过表
 */
static uint8_t mt6701_crc6_check(uint32_t data_word)
{
    uint32_t crc_data = (data_word >> 6) & 0x3FFFFu;   /* 参与校验的18位 */
    uint8_t idx, crc_calc;

    idx = (uint8_t)((crc_data >> 12) & 0x3Fu);          /* [17:12] 高6位 */
    crc_calc = (uint8_t)((crc_data >> 6) & 0x3Fu);      /* [11:6]  中6位 */
    idx = (uint8_t)(crc_calc ^ tableCRC6[idx]);
    crc_calc = (uint8_t)(crc_data & 0x3Fu);             /* [5:0]   低6位 */
    idx = (uint8_t)(crc_calc ^ tableCRC6[idx]);
    crc_calc = tableCRC6[idx];

    return (crc_calc == (uint8_t)(data_word & 0x3Fu)) ? 0 : 1;
}

/* 坏帧保护：CRC 失败时不更新角度，保持上一帧有效值 */
static void dev_mt6701_get_machAngle(struct dev_mt6701 *pobj)
{
    if (pobj->crc_check != 0) {
        if (pobj->err_cnt < 0xFFFFu) pobj->err_cnt++;
        return;   /* 坏帧：不更新 mechanical_angle */
    }
    pobj->err_cnt = 0;
    /* ... 算角度 ... */
}
```

## 7. SSI 帧偏移自锁定模板

MT6701 SSI 帧可能有前导空闲 bit，用 CRC 自校验锁定偏移：

```c
static void dev_mt6701_get_raw(struct dev_mt6701 *pobj)
{
    /* CSN 拉低 → SPI 读 4 字节 → CSN 拉高 */
    dev_mt6701_csn_ctrl(pobj, MT6701_LOW);
    mt6701_spi_read(pobj, pobj->raw_buf, 4);
    dev_mt6701_csn_ctrl(pobj, MT6701_HIGH);

    uint32_t word32 = ((uint32_t)pobj->raw_buf[0] << 24) |
                      ((uint32_t)pobj->raw_buf[1] << 16) |
                      ((uint32_t)pobj->raw_buf[2] << 8) |
                       (uint32_t)pobj->raw_buf[3];

    /* 先用上次锁定的偏移尝试（稳态下一次命中） */
    if (mt6701_try_decode(pobj, word32, pobj->bit_offset) == 0)
        return;

    /* 失配则在 0~8 内重新扫描，命中即锁存新偏移 */
    for (uint8_t k = 0; k <= 8u; k++) {
        if (mt6701_try_decode(pobj, word32, k) == 0) {
            pobj->bit_offset = k;
            return;
        }
    }
    pobj->crc_check = 1;   /* 全部偏移均失败：标记坏帧 */
}
```

## 8. 热路径状态缓存模板

```c
/* start 时缓存 ARR，热路径直接用缓存值 */
static int dev_half_bridge_start(struct dev_half_bridge *pobj)
{
    const dev_half_bridge_config_t *cfg = &half_bridge_list[pobj->id];
    /* 启动 PWM ... */
    drv_tim_get_autoreload(cfg->tim, &pobj->autoreload);  // 缓存 ARR
    pobj->tim = cfg->tim;
    return DEV_EOK;
}

/* set_3pwm 是热路径，用缓存 ARR 限幅，不读 HAL */
static int dev_half_bridge_set_3pwm(struct dev_half_bridge *pobj,
                                    uint32_t ccr1, uint32_t ccr2, uint32_t ccr3)
{
    uint16_t arr = pobj->autoreload;   // 缓存值

    ccr1 = (ccr1 > arr) ? arr : ccr1;
    ccr2 = (ccr2 > arr) ? arr : ccr2;
    ccr3 = (ccr3 > arr) ? arr : ccr3;

    if (!pobj->output_enable) {        // 软急停
        ccr1 = ccr2 = ccr3 = 0;
    }

    drv_pwm_set_dutycycle(cfg->tim, cfg->channel[PHASE_U], ccr1);
    /* ... */
    return DEV_EOK;
}
```

## 9. 滤波状态对象化模板

```c
typedef struct dev_adc_injected {
    /* ... */
    dev_current_f3axis_t current;       // 滤波后电流
    dev_current_f3axis_t prev_current;  // 上一拍（对象内状态，保证多实例可重入）
    float lpf_alpha;                    // 滤波系数
} dev_phase_current_t;

#define _lpfilter(alpha, cur_val, prev_val) ((alpha)*(cur_val) + (1.0f-(alpha))*(prev_val))

static void dev_phase_current_update(struct dev_adc_injected *pobj)
{
    /* 读 ADC → voltage → current */
    dev_current_f3axis_t raw_current = dev_adc_get_current(pobj);

    /* 一阶低通（状态在对象里，多实例不串扰） */
    pobj->current.a = _lpfilter(pobj->lpf_alpha, raw_current.a, pobj->prev_current.a);
    pobj->current.b = _lpfilter(pobj->lpf_alpha, raw_current.b, pobj->prev_current.b);
    pobj->current.c = _lpfilter(pobj->lpf_alpha, raw_current.c, pobj->prev_current.c);

    pobj->prev_current = pobj->current;
}
```

## 10. 自动校准模板

```c
/* 多次采样取均值标定零电流偏置（须在电机不通电时调用） */
static void dev_phase_current_calibrate_offset(struct dev_adc_injected *pobj, uint16_t samples)
{
    assert_report(pobj != NULL);
    if (samples == 0) samples = 256;

    int32_t sum_a = 0, sum_b = 0, sum_c = 0;
    for (uint16_t i = 0; i < samples; i++) {
        pobj->update(pobj);
        sum_a += pobj->adc.a;
        sum_b += pobj->adc.b;
        sum_c += pobj->adc.c;
        drv_delay_us(100);   // 采样间隔
    }
    pobj->offset.a = sum_a / samples;
    pobj->offset.b = sum_b / samples;
    pobj->offset.c = sum_c / samples;
}
```

## 11. 单例 extern 模板

全局唯一设备用 `extern` 单例：

```c
/* dev_power_monitor.h */
extern dev_power_monitor_t dev_power_monitor;

/* dev_power_monitor.c */
dev_power_monitor_t dev_power_monitor;

/* 使用方 */
dev_power_monitor_init(&dev_power_monitor);
dev_power_monitor.start(&dev_power_monitor);
dev_power_monitor.update(&dev_power_monitor);
float vbus = dev_power_monitor.get_vbus(&dev_power_monitor);
```

## 12. 三相轴类型模板

```c
/* 三相整型量(ADC原始值/偏置) */
typedef struct {
    int32_t a, b, c;
} dev_current_i3axis_t;

/* 三相浮点量(电压/电流) */
typedef struct {
    float a, b, c;
} dev_current_f3axis_t;

/* 复合字面量初始化 */
pobj->set_offset(pobj, (dev_current_i3axis_t){PHASE_CURRENT_ZERO_ADC,
                                               PHASE_CURRENT_ZERO_ADC,
                                               PHASE_CURRENT_ZERO_ADC});
```

## 13. LED 闪烁状态机模板

```c
typedef struct dev_led {
    led_id_e id;
    led_state_e state;
    uint32_t blink_period_ms;   // 闪烁周期, 0 表示常亮/常灭
    uint32_t blink_last_tick;   // 上次翻转时刻

    void (*on)(struct dev_led *pobj);
    void (*off)(struct dev_led *pobj);
    void (*toggle)(struct dev_led *pobj);
    void (*set)(struct dev_led *pobj, led_state_e state);
    void (*set_blink)(struct dev_led *pobj, uint32_t period_ms);
    void (*update)(struct dev_led *pobj);   // 周期任务调用
} dev_led_t;

static void dev_led_update(struct dev_led *pobj)
{
    if (pobj->blink_period_ms == 0) return;   // 非闪烁模式
    uint32_t now = drv_dwt_counter_get_ticks();
    if ((now - pobj->blink_last_tick) >= pobj->blink_period_ms * 1000) {
        pobj->toggle(pobj);
        pobj->blink_last_tick = now;
    }
}
```

## 14. 共享 ops 表模板（推荐）

当同一类型所有实例共享相同方法时，用共享 `static const ops` 表替代内联方法指针，省 RAM 且表进 Flash 只读。

### 14.1 ops 表类型与对象

```c
/* dev_xxx.h */
typedef struct dev_xxx dev_xxx_t;   // 前向声明

/* 方法表类型 */
typedef struct {
    void  (*update)(dev_xxx_t *pobj);
    float (*get_value)(dev_xxx_t *pobj);
    void  (*set_zero)(dev_xxx_t *pobj, float zero);
    int   (*start)(dev_xxx_t *pobj);
    int   (*stop)(dev_xxx_t *pobj);
} xxx_ops_t;

/* 设备对象只持有一个 ops 指针 */
struct dev_xxx {
    xxx_id_e id;
    xxx_state_e state;
    uint32_t raw;
    float value;
    float offset;
    const xxx_ops_t *ops;   // 指向共享方法表
};
```

### 14.2 共享方法表与构造

```c
/* dev_xxx.c */
static const xxx_ops_t s_xxx_ops = {
    .update    = dev_xxx_update_impl,
    .get_value = dev_xxx_get_value_impl,
    .set_zero  = dev_xxx_set_zero_impl,
    .start     = dev_xxx_start_impl,
    .stop      = dev_xxx_stop_impl,
};

int dev_xxx_init(dev_xxx_t *pobj, xxx_id_e id)
{
    if (pobj == NULL || id >= XXX_ID_MAX)
        return DEV_ERROR;
    memset(pobj, 0, sizeof(*pobj));
    pobj->id = id;
    pobj->ops = &s_xxx_ops;   // 一行装配所有方法
    return DEV_EOK;
}
```

### 14.3 包装函数（推荐）

对外提供包装函数，集中处理参数检查，不让外部直接调 `self->ops->fn`：

```c
/* dev_xxx.h —— 包装函数声明 */
void  dev_xxx_update(dev_xxx_t *pobj);
float dev_xxx_get_value(dev_xxx_t *pobj);
int   dev_xxx_start(dev_xxx_t *pobj);

/* dev_xxx.c —— 包装函数实现 */
void dev_xxx_update(dev_xxx_t *pobj)
{
    if (pobj == NULL || pobj->ops == NULL || pobj->ops->update == NULL)
        return;
    pobj->ops->update(pobj);
}

int dev_xxx_start(dev_xxx_t *pobj)
{
    if (pobj == NULL || pobj->ops == NULL || pobj->ops->start == NULL)
        return DEV_ERROR;
    return pobj->ops->start(pobj);
}
```

### 14.4 内联方法指针 vs 共享 ops 表 对照

```c
/* 内联方法指针（项目既有风格，每实例存 N 个指针） */
typedef struct dev_mt6701 {
    /* 状态字段... */
    void (*update)(struct dev_mt6701 *pobj);        // 每实例 1 个指针
    bool (*set_zero_angle)(struct dev_mt6701 *pobj, float deg);  // 每实例 1 个指针
    /* 共 N 个方法指针 × 每实例 */
} dev_mt6701_t;

/* 共享 ops 表（推荐，每实例只 1 个 ops 指针） */
typedef struct dev_mt6701 {
    /* 状态字段... */
    const mt6701_ops_t *ops;   // 每实例只 1 个指针，指向 Flash 中的共享表
} dev_mt6701_t;
```

## 15. 继承与 container_of 模板

当需要表达"is-a"关系（派生设备继承基类接口），用结构体嵌套 + `container_of`。

### 15.1 基类作为首字段命名为 base

```c
/* encoder_if.h —— 基类接口 */
typedef struct encoder_base {
    const encoder_ops_t *ops;
    float angle;
} encoder_base_t;

/* encoder_mt6701.h —— 派生类，base 为首字段 */
typedef struct encoder_mt6701 {
    encoder_base_t base;    // 必须为首字段，命名为 base
    mt6701_id_e id;
    uint32_t raw;
    float offset;
    /* MT6701 特有字段... */
} encoder_mt6701_t;
```

### 15.2 container_of 下转型宏

```c
/* container_of：从基类指针反推派生类指针 */
#ifndef container_of
#define container_of(ptr, type, member) \
    ((type *)((char *)(ptr) - offsetof(type, member)))
#endif

/* 上转型：encoder_mt6701_t * → encoder_base_t *（隐式，因 base 是首字段） */
encoder_mt6701_t mt6701_dev;
encoder_base_t *base = &mt6701_dev.base;   // 上转型，合法且安全

/* 下转型：encoder_base_t * → encoder_mt6701_t *（只在派生实现内部做） */
static void encoder_mt6701_update_impl(encoder_base_t *base)
{
    encoder_mt6701_t *self = container_of(base, encoder_mt6701_t, base);
    /* 访问 self->raw, self->offset 等 MT6701 特有字段 */
}
```

### 15.3 多态调用

```c
/* 对外包装函数，通过 ops 表分派 */
void encoder_update(encoder_base_t *enc)
{
    if (enc == NULL || enc->ops == NULL || enc->ops->update == NULL)
        return;
    enc->ops->update(enc);   // 实际调 encoder_mt6701_update_impl / encoder_mt6835_update_impl
}
```

规则：只公开从派生到基类的上转型；下转型只在派生实现内部用 `container_of`；不建立深继承层级，超过一层改组合。

