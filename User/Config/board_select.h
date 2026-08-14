#ifndef __BOARD_SELECT_H__
#define __BOARD_SELECT_H__

/*
 * Select exactly one board macro in the MDK project:
 *   JM_BOARD_V1
 *   JM_BOARD_SFOC_V2
 *   JM_BOARD_SFOC
 *   JM_BOARD_ODRIVE
 */
#if defined(JM_BOARD_V1) && defined(JM_BOARD_SFOC_V2)
#error "Define only one board macro."
#endif
#if defined(JM_BOARD_V1) && defined(JM_BOARD_SFOC)
#error "Define only one board macro."
#endif
#if defined(JM_BOARD_V1) && defined(JM_BOARD_ODRIVE)
#error "Define only one board macro."
#endif
#if defined(JM_BOARD_SFOC_V2) && defined(JM_BOARD_SFOC)
#error "Define only one board macro."
#endif
#if defined(JM_BOARD_SFOC_V2) && defined(JM_BOARD_ODRIVE)
#error "Define only one board macro."
#endif
#if defined(JM_BOARD_SFOC) && defined(JM_BOARD_ODRIVE)
#error "Define only one board macro."
#endif

#ifndef JM_BOARD_SFOC
#ifndef JM_BOARD_ODRIVE
#define JM_BOARD_SFOC
#endif
#endif

#if defined(JM_BOARD_V1)
#define JM_BOARD_NAME "V1"
#include "../../Board/V1/Config/dev_config_board.h"
#define JM_BOARD_DEV_CONFIG_INC "../../Board/V1/Config/dev_config_board.inc"
#elif defined(JM_BOARD_SFOC_V2)
#define JM_BOARD_NAME "SFOC_V2"
#include "../../Board/SFOC_V2/Config/dev_config_board.h"
#define JM_BOARD_DEV_CONFIG_INC "../../Board/SFOC_V2/Config/dev_config_board.inc"
#elif defined(JM_BOARD_SFOC)
#define JM_BOARD_NAME "SFOC"
#include "../../Board/SFOC/Config/dev_config_board.h"
#define JM_BOARD_DEV_CONFIG_INC "../../Board/SFOC/Config/dev_config_board.inc"
#elif defined(JM_BOARD_ODRIVE)
#define JM_BOARD_NAME "ODRIVE"
#include "../../Board/ODriveMKS/Config/dev_config_board.h"
#define JM_BOARD_DEV_CONFIG_INC "../../Board/ODriveMKS/Config/dev_config_board.inc"
#else
#error "Define JM_BOARD_V1 / JM_BOARD_SFOC_V2 / JM_BOARD_SFOC / JM_BOARD_ODRIVE in the target options."
#endif

#endif /* __BOARD_SELECT_H__ */
