#ifdef JM_BOARD_CONFIG_DEFINE_TABLES
#if defined(USE_DEV_MT6701)
const mt6701_config_t mt6701_list[MT6701_ID_MAX] = {
    [MT6701_ID_1] = {
        .name = "MT6701",
        .spi_num = {.hspi = DRV_SPI3},
        .csn = {.gpiox = DRV_GPIOB, .pin = DRV_PIN_5, .ste = DRV_PIN_LOW},
    },
};
#endif

#if defined(USE_DEV_MT6835)
const mt6835_config_t mt6835_list[MT6835_ID_MAX] = {
    [MT6805_ID_1] = {
        .name = "MT6835",
        .spi_num = {.hspi = DRV_SPI1},                                         
        .csn = {.gpiox = DRV_GPIOA, .pin = DRV_PIN_4, .ste = DRV_PIN_LOW},     
        .cal_en = {.gpiox = DRV_GPIOB, .pin = DRV_PIN_2, .ste = DRV_PIN_LOW},   
    },
};
#endif

#if defined(USE_DEV_HALF_BRIDGE)
const dev_half_bridge_config_t half_bridge_list[BRIDGE_ID_MAX] = {
    [BRIDGE_DEV1] = {
        .name = "HALF_BRIDGE_1",
        .tim = DRV_TIM1,
        .channel = {TIM_CH1, TIM_CH2, TIM_CH3, TIM_CH4},
    },
};
#endif

#if defined(USE_DEV_POWER_MONITOR)
const dev_power_monitor_config_t power_monitor_list[PM_CH_MAX] = {
    [0] = {.name = "VBUS",    .id = DRV_ADC_1,    .channel = DRV_ADC_CH1,  .type = PM_CH_VBUS,        .scale = PM_VBUS_RATIO, .offset = 0.0f},
    [1] = {.name = "IBUS",    .id = DRV_ADC_INIT, .channel = DRV_ADC_CH0,  .type = PM_CH_IBUS_SYNTH,  .scale = 0.0f,          .offset = 0.0f},
    [2] = {.name = "NTC_FET", .id = DRV_ADC_1,    .channel = DRV_ADC_CH15, .type = PM_CH_TEMP_DRIVER, .scale = 0.0f,          .offset = 0.0f},
    [3] = {.name = "NTC_MOT", .id = DRV_ADC_1,    .channel = DRV_ADC_CH12, .type = PM_CH_TEMP_MOTOR,  .scale = 0.0f,          .offset = 0.0f},
};
#endif

#if defined(USE_DEV_PHASE_CURRENT)
const dev_phase_current_config_t phase_current_list[ADCX_INX_INJECTED_MAX] = {
    [ADCX_IA] = {.name = "IA", .id = DRV_ADC_1, .channel = DRV_ADC_CH2},
    [ADCX_IB] = {.name = "IB", .id = DRV_ADC_1, .channel = DRV_ADC_CH3},
    [ADCX_IC] = {.name = "IC", .id = DRV_ADC_1, .channel = DRV_ADC_CH4},
};
#endif

#if defined(USE_DEV_COMMUN_UART)
const dev_commun_uart_config_t commun_uart_list[JM_UART_COMM_ID_MAX] = {
    [JM_UART_COMM_ID_1] = {
        .name = "JM_UART_1",
        .uart = DRV_UART1,
        .motor_id = 1,
    },
};
#endif

#if defined(USE_DEV_COMMUN_CAN)
/* §V3 CAN 配置表: V1 板使用 FDCAN1 + CA-IS2062A 收发器
 * - ide=1 扩展帧: 因 CAN ID = (CMD<<8)|motor_id, 最大 0xFFFF, 超出 11 位标准帧(0x7FF)
 * - use_fd=1: 启用 FD 模式(收发仍走 8 字节单帧/多帧, FD 长帧暂未启用)
 * - motor_id=1: 本机默认地址(可由 0xD0/0xD1 命令在线修改) */
const dev_commun_can_config_t commun_can_list[JM_CAN_COMM_ID_MAX] = {
    [JM_CAN_COMM_ID_1] = {
        .name = "JM_CAN_1",
        .can = DRV_CAN1,
        .motor_id = MOTOR_ID_DEFAULT,
        .ide = 1,
        .use_fd = USE_CAN_FD_MODE,
    },
};
#endif

#if defined(USE_DEV_LED)
/* LED 配置表: V1 板 LED1(红)=PC14 故障指示, LED2(绿)=PC15 运行状态指示
 * 极性 LED_ACTIVE_HIGH: 阳极经 1kΩ 限流电阻接 MCU, 阴极接 GND, 高电平点亮 */
const dev_led_config_t led_list[LED_ID_MAX] = {
    [LED_ID_1] = {
        .name = "LED1_RED_FAULT",
        .gpio = {.gpiox = DRV_GPIOC, .pin = DRV_PIN_14, .ste = DRV_PIN_LOW},
        .polarity = LED_ACTIVE_HIGH,
    },
    [LED_ID_2] = {
        .name = "LED2_GREEN_STATUS",
        .gpio = {.gpiox = DRV_GPIOC, .pin = DRV_PIN_15, .ste = DRV_PIN_LOW},
        .polarity = LED_ACTIVE_HIGH,
    },
};
#endif

#endif /* JM_BOARD_CONFIG_DEFINE_TABLES */
