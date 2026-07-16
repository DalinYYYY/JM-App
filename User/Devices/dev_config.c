/**
 * @file dev_config.c
 * @brief Board-specific device mapping tables.
 */
#define JM_BOARD_CONFIG_DEFINE_TABLES
#include "dev_config.h"

#include "dev_led.h"
#include "dev_mt6701.h"
#include "dev_mt6835.h"
#include "dev_as5047.h"
#include "dev_drv8301.h"
#include "dev_half_bridge.h"
#include "dev_eeprom.h"
#include "dev_power_monitor.h"
#include "dev_motor_phase_current.h"
#include "dev_commun_vesc.h"
#include "dev_commun_uart.h"

#include JM_BOARD_DEV_CONFIG_INC
