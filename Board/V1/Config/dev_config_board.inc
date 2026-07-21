#ifdef JM_BOARD_CONFIG_DEFINE_TABLES
#if defined(USE_DEV_MT6701)
const mt6701_config_t mt6701_list[MT6701_ID_MAX] = {
    [MT6701_ID_1] = {
        .name = "MT6701_1",
        .spi_num = {.hspi = DRV_SPI1},
        .csn = {.gpiox = DRV_GPIOA, .pin = DRV_PIN_4, .ste = DRV_PIN_LOW},
    },
};
#endif

#if defined(USE_DEV_MT6835)
const mt6835_config_t mt6835_list[MT6835_ID_MAX] = {
    [MT6805_ID_1] = {
        .name = "MT6835_1",
        .spi_num = {.hspi = DRV_SPI3},                                    /* 原理图: MT6835(U18) 接 SPI3 (PB3=SCK, PB4=MISO) */
        .csn = {.gpiox = DRV_GPIOB, .pin = DRV_PIN_5, .ste = DRV_PIN_LOW}, /* 原理图: MT6835 CSN 接 PB5 */
        .cal_en = {.gpiox = DRV_GPIOB, .pin = DRV_PIN_2, .ste = DRV_PIN_LOW}, /* CAL 引脚, 驱动暂未使用, 保留占位 */
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
    /* [0] VBUS: PA0=ADC1_IN1=CH1, 分压比 11:1 */
    [0] = {.name = "VBUS",    .id = DRV_ADC_1,    .channel = DRV_ADC_CH1,  .type = PM_CH_VBUS,        .scale = PM_VBUS_RATIO, .offset = 0.0f},
    /* [1] IBUS 合成: 不读 ADC, id=DRV_ADC_INIT 跳过 DMA, 由 motor_loop_isr 写入 */
    [1] = {.name = "IBUS",    .id = DRV_ADC_INIT, .channel = DRV_ADC_CH0,  .type = PM_CH_IBUS_SYNTH,  .scale = 0.0f,          .offset = 0.0f},
    /* [2] NTC1: PB0=ADC1_IN15, 驱动器温度(占位, 计算逻辑待实现) */
    [2] = {.name = "NTC_FET", .id = DRV_ADC_1,    .channel = DRV_ADC_CH15, .type = PM_CH_TEMP_DRIVER, .scale = 0.0f,          .offset = 0.0f},
    /* [3] NTC2: PB1=ADC1_IN12, 电机温度(占位, 计算逻辑待实现) */
    [3] = {.name = "NTC_MOT", .id = DRV_ADC_1,    .channel = DRV_ADC_CH12, .type = PM_CH_TEMP_MOTOR,  .scale = 0.0f,          .offset = 0.0f},
};
#endif

#if defined(USE_DEV_PHASE_CURRENT)
const dev_phase_current_config_t phase_current_list[ADCX_INX_INJECTED_MAX] = {
    /* 注: channel 字段为元信息(驱动未读取), 实际 rank 由 start 按 list 出现次序分配
     * 原理图: IA=MOTOR_U(PA1=ADC1_IN2=CH2), IB=MOTOR_V(PA2=ADC1_IN3=CH3), IC=MOTOR_W(PA3=ADC1_IN4=CH4)
     * 注入组 rank1/rank2/rank3 必须与下方次序一致 (IA→IB→IC) */
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

#endif /* JM_BOARD_CONFIG_DEFINE_TABLES */
