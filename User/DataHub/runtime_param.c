#include "runtime_param.h"
#include "main.h"
#include "version.h"
#include <string.h>

sys_data_t usr;

/* 通用调试观测通道(0xC9): 任意处写 jm_dbg[i]=变量; 即可在上位机查看 */
float jm_dbg[JM_DBG_CH];

static void write_u32_le(uint8_t dst[4], uint32_t value)
{
	dst[0] = (uint8_t)(value & 0xFFu);
	dst[1] = (uint8_t)((value >> 8) & 0xFFu);
	dst[2] = (uint8_t)((value >> 16) & 0xFFu);
	dst[3] = (uint8_t)((value >> 24) & 0xFFu);
}

void user_data_init(void)
{
	memset(&usr, 0, sizeof(sys_data_t));

	/* STM32 UID 固定为 96 bit，按协议小端序保存为连续 12 字节。 */
	write_u32_le(&usr.sys.info.device_uid[0], HAL_GetUIDw0());
	write_u32_le(&usr.sys.info.device_uid[4], HAL_GetUIDw1());
	write_u32_le(&usr.sys.info.device_uid[8], HAL_GetUIDw2());

	// 版本信息
	usr.sys.version.boot.byte.version_major = BOOT_VERSION_MAJOR;
	usr.sys.version.boot.byte.version_minor = BOOT_VERSION_MINOR;
	usr.sys.version.boot.byte.version_patch = BOOT_VERSION_PATCH;
	usr.sys.version.boot.byte.version_build = BOOT_VERSION_BUILD;

	usr.sys.version.app.byte.version_major = APP_VERSION_MAJOR;
	usr.sys.version.app.byte.version_minor = APP_VERSION_MINOR;
	usr.sys.version.app.byte.version_patch = APP_VERSION_PATCH;
	usr.sys.version.app.byte.version_build = APP_VERSION_BUILD;

	usr.sys.version.hardware.byte.version_major = HW_VERSION_MAJOR;
	usr.sys.version.hardware.byte.version_minor = HW_VERSION_MINOR;
	usr.sys.version.hardware.byte.version_patch = HW_VERSION_PATCH;
	usr.sys.version.hardware.byte.version_build = HW_VERSION_BUILD;
}

/**************************************** 协议解析 ****************************************/
