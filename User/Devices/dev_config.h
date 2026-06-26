/**
 * @file dev_config.h
 * @brief Shared device configuration interface.
 *
 * Board-specific enable macros and pin/channel tables are selected through
 * `User/Config/board_select.h`.
 */
#ifndef __DEV_CONFIG_H__
#define __DEV_CONFIG_H__

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "board_select.h"

#define DEV_EOK 0
#define DEV_ERROR 1
#define DEV_ENABLE 1
#define DEV_DISABLE 0

/* ===================================================================== */
/*                          Device constants                             */
/* ===================================================================== */

#if defined(USE_DEV_MT6701)
/* MT6701 SPI mode
 *   0 => Hardware SPI
 *   1 => Software SPI
 */
#define DEV_MT6701_SPI_DRIVER 0
#endif

#if defined(USE_DEV_MT6835)
/* MT6835 SPI mode
 *   0 => Hardware SPI
 *   1 => Software SPI
 */
#define DEV_MT6835_SPI_DRIVER 0
#endif

#if defined(USE_DEV_POWER_MONITOR)
#define PM_VREF (3.3f)
#define PM_RESOLUTION (4096.0f)
#define PM_VBUS_RATIO (24.0f)
#define PM_IBUS_RATIO (10.0f)
#define PM_TEMP_RATIO (10.0f)
#endif

#if defined(USE_DEV_HALF_BRIDGE)
#define HALF_BRIDGE_ADC_TRIG_CCR (8380u)
#endif

#if defined(USE_DEV_PHASE_CURRENT)
#define PHASE_CURRENT_GAIN (50.0f)
#define PHASE_CURRENT_SHUNT (0.01f)
#define PHASE_CURRENT_VREF (3.3f)
#define PHASE_CURRENT_RESOLUTION (4096.0f)
#define PHASE_CURRENT_LPF_ALPHA (0.9f)
#define PHASE_CURRENT_ZERO_ADC (2024u)
#endif

#if defined(USE_DEV_COMMUN_VESC)
#define DEV_VESC_RX_BUF_SIZE (256u)
#endif

#if defined(USE_DEV_COMMUN_UART)
#define DEV_JM_UART_RX_BUF_SIZE (256u)
#endif

#endif /* __DEV_CONFIG_H__ */
