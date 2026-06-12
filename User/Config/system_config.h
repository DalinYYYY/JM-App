#ifndef __SYSTEM_CONFIG_H__
#define __SYSTEM_CONFIG_H__

#include "main.h"

#define UVW_CURRENT_U_HANDLE (hadc1)
#define UVW_CURRENT_V_HANDLE (hadc2)
#define UVW_CURRENT_W_HANDLE (hadc3)
#define UVW_CURRENT_U_CHANNEL (UVW_CURRENT_U_HANDLE.Instance->JDR1)
#define UVW_CURRENT_V_CHANNEL (UVW_CURRENT_V_HANDLE.Instance->JDR1)
#define UVW_CURRENT_W_CHANNEL (UVW_CURRENT_W_HANDLE.Instance->JDR1)
#define CURRENT_LOOP_IRQ_TASK HAL_ADCEx_InjectedConvCpltCallback

#endif /* __SYSTEM_CONFIG_H__ */
