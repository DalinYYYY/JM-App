#include "runtime_param.h"
#include "version.h"
#include <string.h>

sys_data_t usr;

void user_data_init(void)
{
	memset(&usr, 0, sizeof(sys_data_t));

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
