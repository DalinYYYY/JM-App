/**
 * @file dev_config.c
 * @brief 板级设备映射配置表（由 dev_config.h 根据板型展开定义）
 */
#define JM_BOARD_CONFIG_DEFINE_TABLES
#include "dev_config.h"

#include "dev_led.h"
#include "dev_mt6701.h"
#include "dev_mt6835.h"
#include "dev_as5047.h"
#include "dev_drv8301.h"
#include "dev_half_bridge.h"
#include "dev_motor.h"
#include "dev_eeprom.h"
#include "dev_power_monitor.h"
#include "dev_motor_phase_current.h"
#include "dev_commun_uart.h"
#include "dev_commun_can.h"

#include JM_BOARD_DEV_CONFIG_INC
