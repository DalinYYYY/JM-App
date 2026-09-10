"""Keil 编译前自动递增 APP_VERSION_BUILD。

由 uvprojx 的 Before Build/Rebuild 用户命令调用, 每次 Build/Rebuild 前
将 User/DataHub/version.h 的 APP_VERSION_BUILD +1, 使每次编译产物携带
唯一递增的版本号(BOOT/HW 版本不动)。

按字节读写与正则替换, 不改变文件编码与其余内容。
用法: python incr_build.py [version.h路径]  (缺省按脚本位置推导)
"""
import re
import sys
from pathlib import Path

# User\Tools\scripts\incr_build.py -> parents[2] = User -> User\DataHub\version.h
DEFAULT_PATH = Path(__file__).resolve().parents[2] / "DataHub" / "version.h"


def main() -> int:
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_PATH
    try:
        data = path.read_bytes()
    except OSError as e:
        print(f"incr_build: 读取失败 {path}: {e}")
        return 1
    m = re.search(rb"(#define\s+APP_VERSION_BUILD\s+)(\d+)", data)
    if m is None:
        print(f"incr_build: 未找到 APP_VERSION_BUILD ({path})")
        return 1
    old_build = int(m.group(2))
    new_build = old_build + 1
    data = data[:m.start(2)] + str(new_build).encode() + data[m.end(2):]
    path.write_bytes(data)
    print(f"incr_build: APP_VERSION_BUILD {old_build} -> {new_build}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
