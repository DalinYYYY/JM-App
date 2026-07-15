#ifdef JM_BOARD_CONFIG_DEFINE_TABLES

#if defined(USE_DEV_AS5047)
const as5047_config_t as5047_list[AS5047_ID_MAX] = {
    [AS5047_ID_1] = {
        .name = "AS5047_1",
        .spi_num = {.hspi = DRV_SPI3},
        .csn = {.gpiox = DRV_GPIOB, .pin = DRV_PIN_2, .ste = DRV_PIN_LOW},  /* PB2 = GPIO_6 */
    },
};
#endif

#if defined(USE_DEV_DRV8301)
const drv8301_config_t drv8301_list[DRV8301_ID_MAX] = {
    [DRV8301_ID_1] = {
        .name = "DRV8301_1",
        .spi_num = {.hspi = DRV_SPI3},  /* 共享 SPI3 */
        .nscs = {.gpiox = DRV_GPIOC, .pin = DRV_PIN_13, .ste = DRV_PIN_LOW},  /* PC13 */
        .nfault = {.gpiox = DRV_GPIOD, .pin = DRV_PIN_2, .ste = DRV_PIN_LOW},  /* PD2 */
        /* CTRL1: D0=DC_CAL(0=正常), D2:D1=GAIN(10=40V/V), D5:D3=OCTW(111=默认上报OT+OC)
         *        → GAIN=40V/V: D2:D1=10b=0x0004, OCTW=111b=0x0038, 合并=0x003C
         * CTRL2: D7=PWM_MODE(0=6PWM), D5:D4=OCP(00=current limit), D2:D1=GATE_CURRENT(11=3.0A) */
        .ctrl1_value = 0x003C,  /* GAIN=40V/V, DC_CAL=0, OCTW=111(默认保护) */
        .ctrl2_value = 0x0006,  /* GATE_CURRENT=3.0A, 6PWM, OCP=current limit */
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
    [PM_VBUS] = {.name = "VBUS", .id = DRV_ADC_1, .channel = DRV_ADC_CH6},   /* PA6 */
    /* PM_IBUS 不配置硬件通道: PM_IBUS_SOURCE=1 时由三相电流合成, 不读 ADC */
};
#endif

#if defined(USE_DEV_PHASE_CURRENT)
const dev_phase_current_config_t phase_current_list[ADCX_INX_INJECTED_MAX] = {
    [ADCX_IA] = {.name = "M0_IA", .id = DRV_ADC_1, .channel = DRV_ADC_CH10},  /* PC0 SO1 */
    [ADCX_IB] = {.name = "M0_IB", .id = DRV_ADC_1, .channel = DRV_ADC_CH11},  /* PC1 SO2 */
    [ADCX_IC] = {.name = "M0_IC_PHANTOM", .id = DRV_ADC_1, .channel = DRV_ADC_CH10},  /* 占位, IC 由合成 */
};
#endif

#if defined(USE_DEV_COMMUN_UART)
const dev_commun_uart_config_t commun_uart_list[JM_UART_COMM_ID_MAX] = {
    [JM_UART_COMM_ID_1] = {
        .name = "JM_UART_1",
        .uart = DRV_UART4,
        .motor_id = 1,
    },
};
#endif

#endif /* JM_BOARD_CONFIG_DEFINE_TABLES */
