# 板级目录结构

每块硬件板在 `Board/<名称>/` 下保存各自 CubeMX 生成的工程。

现有规则：
- `Board/<名称>/Core`、`Drivers`、`Middlewares`、`MDK-ARM` 及 `.ioc` 文件保持板内私有。
- 共享设备 API 统一放在 `User/Devices/`。
- 板级设备映射（引脚/外设配置表）放在 `Board/<名称>/Config/`。

## 新增一块板

为新板创建以下文件：

- `Board/<名称>/Config/dev_config_board.h`
- `Board/<名称>/Config/dev_config_board.inc`

然后按步骤操作：

1. 在 `dev_config_board.h` 中定义板级设备使能宏（`USE_DEV_XXX`）。
2. 在 `dev_config_board.inc` 中填充 `*_list` 配置表（硬件资源映射）。
3. 在 MDK 目标选项中添加板宏，例如 `JM_BOARD_<名称>`。
4. 在 `User/Config/board_select.h` 中为新板添加分支。

## 现有板

- `Board/V1/`
- `Board/SFOC/`（作为 git 子模块管理）
