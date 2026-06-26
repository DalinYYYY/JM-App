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
    [PM_IBUS] = {.name = "IBUS", .id = DRV_ADC_1, .channel = DRV_ADC_CH12},
    [PM_VBUS] = {.name = "VBUS", .id = DRV_ADC_1, .channel = DRV_ADC_CH15},
};
#endif

#if defined(USE_DEV_PHASE_CURRENT)
const dev_phase_current_config_t phase_current_list[ADCX_INX_INJECTED_MAX] = {
    [ADCX_IA] = {.name = "IA", .id = DRV_ADC_1, .channel = DRV_ADC_CH1},
    [ADCX_IB] = {.name = "IB", .id = DRV_ADC_1, .channel = DRV_ADC_CH2},
    [ADCX_IC] = {.name = "IC", .id = DRV_ADC_1, .channel = DRV_ADC_CH3},
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
