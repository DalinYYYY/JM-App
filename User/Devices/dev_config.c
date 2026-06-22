/**
 * @file device_config.c
 * @brief 设备层硬件资源映射表（集中管理）
 * @note  所有设备实例的引脚/总线/通道配置都在此处，使用 C99 指定初始化器。
 *        使能开关与换算常量见 device_config.h。
 */
#include "dev_config.h"

#include "dev_led.h"
#include "dev_mt6701.h"
#include "dev_mt6835.h"
#include "dev_half_bridge.h"
#include "dev_eeprom.h"
#include "dev_power_monitor.h"
#include "dev_motor_phase_current.h"
// #include "dev_commun_vesc.h"
#include "dev_commun_uart.h"

/* ---------------- MT6701 磁编码器 ---------------- */
#if defined(USE_DEV_MT6701)
const mt6701_config_t mt6701_list[MT6701_ID_MAX] = {
	[MT6701_ID_1] = {
					 .name = "MT6701_1",
					 .csn = {.gpiox = DRV_GPIOA, .pin = DRV_PIN_4, .ste = DRV_PIN_LOW},
					 },
};
#endif

/* ---------------- MT6835 磁编码器 ---------------- */
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

/* ---------------- 三相半桥 PWM ---------------- */
#if defined(USE_DEV_HALF_BRIDGE)
const dev_half_bridge_config_t half_bridge_list[BRIDGE_ID_MAX] = {
	[BRIDGE_DEV1] = {
					 .name = "HALF_BRIDGE_1",
					 .tim = DRV_TIM1,
					 /* U / V / W / 触发ADC注入组 */
		.channel = {TIM_CH1, TIM_CH2, TIM_CH3, TIM_CH4},
					 },
};
#endif

/* ---------------- 电源监控 ADC 规则组 (母线电压/电流/温度) ---------------- */
#if defined(USE_DEV_POWER_MONITOR)
const dev_power_monitor_config_t power_monitor_list[PM_CH_MAX] = {
	[PM_IBUS] = {.name = "IBUS", .id = DRV_ADC_1, .channel = DRV_ADC_CH12},
	[PM_VBUS] = {.name = "VBUS", .id = DRV_ADC_1, .channel = DRV_ADC_CH15},
};
#endif

/* ---------------- ADC 注入组 (三相相电流) ---------------- */
#if defined(USE_DEV_PHASE_CURRENT)
const dev_phase_current_config_t phase_current_list[ADCX_INX_INJECTED_MAX] = {
	[ADCX_IA] = {.name = "IA", .id = DRV_ADC_1, .channel = DRV_ADC_CH2},
	[ADCX_IB] = {.name = "IB", .id = DRV_ADC_2, .channel = DRV_ADC_CH3},
	[ADCX_IC] = {.name = "IC", .id = DRV_ADC_2, .channel = DRV_ADC_CH4},
};
#endif

/* ---------------- VESC Tool 串口通信 (USART1 + DMA空闲中断) ---------------- */
#if defined(USE_DEV_COMMUN_VESC)
const dev_commun_vesc_config_t commun_vesc_list[VESC_COMM_ID_MAX] = {
	[VESC_COMM_ID_1] = {
						.name = "VESC_COMM_1",
						.uart = DRV_UART1, /* 须为已配置 DMA 收发的串口 */
		.hw_name = "JointMotor",
						.fw_name = "JM_FW",
						.fw_major = 6,
						.fw_minor = 1,
						},
};
#endif

/* ---------------- 关节电机串口通信 (joint_proto, USART+DMA空闲中断) ---------------- */
#if defined(USE_DEV_COMMUN_UART)
const dev_commun_uart_config_t commun_uart_list[JM_UART_COMM_ID_MAX] = {
	[JM_UART_COMM_ID_1] = {
						   .name = "JM_UART_1",
						   .uart = DRV_UART1, /* 须为已配置 DMA 收发的串口 */
		.motor_id = 1,	   /* 本机地址(串口可忽略, 与CAN保持一致) */
	},
};
#endif

/* ---------------- 单色 LED ---------------- */
#if defined(USE_DEV_LED)
const dev_led_config_t led_list[LED_ID_MAX] = {
	[LED_ID_1] = {
				  .name = "LED_1",
				  .gpio = {.gpiox = DRV_GPIOC, .pin = DRV_PIN_13, .ste = DRV_PIN_LOW},
				  .polarity = LED_ACTIVE_LOW,
				  },
};
#endif

/* ---------------- RGB LED ---------------- */
#if defined(USE_DEV_RGB_LED)
const dev_rgb_led_config_t rgb_led_list[RGB_LED_ID_MAX] = {
	[RGB_LED_ID_1] = {
					  .name = "RGB_LED_1",
					  .r = {.gpiox = DRV_GPIOA, .pin = DRV_PIN_0, .ste = DRV_PIN_LOW},
					  .g = {.gpiox = DRV_GPIOA, .pin = DRV_PIN_1, .ste = DRV_PIN_LOW},
					  .b = {.gpiox = DRV_GPIOA, .pin = DRV_PIN_2, .ste = DRV_PIN_LOW},
					  .polarity = LED_ACTIVE_HIGH,
					  },
};
#endif

/* ---------------- EEPROM (I2C / AT24Cxx) ---------------- */
#if defined(USE_DEV_EEPROM)
const dev_eeprom_config_t eeprom_list[EEPROM_ID_MAX] = {
	[EEPROM_ID_1] = {
					 .name = "EEPROM_1",
					 .i2c = {.hi2c = DRV_I2C1},
					 .i2c_addr = 0x50,	/* 7bit 器件地址 */
		.page_size = 32,	/* 页大小, 字节 */
		.total_size = 8192, /* 总容量, 字节 (AT24C64=8KB) */
		.addr_width = 2,	/* 内部地址宽度: 1=8bit 2=16bit */
	},
};
#endif
