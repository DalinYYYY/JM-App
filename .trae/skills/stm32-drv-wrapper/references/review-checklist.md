# drv 驱动封装层评审清单

> 评审 `User/Driver/` 下的 drv 驱动代码时逐项检查。每项给出"通过/违规/不适用"判定，违规项必须给出修正建议。

## A. HAL 类型屏蔽（核心）

| # | 检查项 | 判定方法 |
|---|--------|----------|
| A1 | 公共头 `drv_xxx.h` 不 `#include "xxx.h"`（HAL 头） | 读 `.h`，确认只 include `drv_config.h` 和 `<stdint.h>` 等标准头 |
| A2 | 公共头不出现任何 HAL 类型（`SPI_HandleTypeDef*`、`GPIO_TypeDef*`、`CAN_RxHeaderTypeDef` 等） | 全文搜索 `HandleTypeDef`、`TypeDef`（HAL 特征后缀） |
| A3 | 设备描述符 `xxxDrv_t` 用编号枚举，不持有 HAL 句柄指针 | 检查 struct 字段类型 |
| A4 | HAL 头只在 `.c` 内 `#include` | 读 `.c` 顶部 include 块 |
| A5 | 跨系列外设有统一结构体屏蔽 HAL 报文头差异（如 `drvCanMsg_t`） | 检查公共类型定义 |

## B. 句柄查找表

| # | 检查项 | 判定方法 |
|---|--------|----------|
| B1 | HAL 句柄用 `__weak` 声明，允许 drv 层独立编译 | 检查 `.c` 顶部 `__weak` 声明 |
| B2 | 句柄映射用 `static const` 指针数组 + designated initializer | 搜索 `s_xxx_map`，确认是 `static const` 且用 `[DRV_XXX1] = &hxxx1` |
| B3 | getter 是 `static inline`，做边界检查返回 NULL | 检查 `get_xxx_handle` 函数 |
| B4 | 没有 `switch(peripheral){case DRV_XXX1: return &hxxx1;}` 链 | 搜索 switch 语句定位句柄 |
| B5 | 未编译进的外设对应表项为 NULL 或 `#ifdef` 条件编译 | 检查 GPIOE/F/G 等可选外设 |
| B6 | 调用方统一检查 getter 返回 NULL | 检查每个公共函数开头 |

## C. 枚举映射

| # | 检查项 | 判定方法 |
|---|--------|----------|
| C1 | HAL 位域宏（`TIM_CHANNEL_x`、`ADC_CHANNEL_x`）用查找表映射，不当连续值移位 | 检查 `s_xxx_channel_map` |
| C2 | 简单枚举（连续值）可用 `switch`，但需有 `default` 分支 | 检查 `get_gpio_mode` 等 |
| C3 | 通道/模式枚举有 `XXX_MAX` 边界值 | 检查枚举定义 |
| C4 | 外设编号枚举有 `DRV_XXX_INIT` 占位和 `DRV_XXX_NUMBER_MAX` 边界 | 检查枚举定义 |

## D. API 设计

| # | 检查项 | 判定方法 |
|---|--------|----------|
| D1 | 阻塞/IT/DMA 三态接口命名统一后缀（`_it`/`_dma`） | 检查函数名 |
| D2 | 阻塞接口有 `timeout` 参数，IT/DMA 无 | 检查函数签名 |
| D3 | 返回 `int`，成功 `DRV_EOK`、失败 `DRV_ERROR` | 检查返回值 |
| D4 | 公共函数命名 `drv_<peripheral>_<action>` | 检查函数名前缀 |
| D5 | 类型命名 `_t` 后缀（struct）、`_e` 后缀（enum） | 检查 typedef |
| D6 | 非阻塞传输启动失败立即释放 CS 等资源 | 检查 `HAL_XXX_Transmit_IT/DMA` 失败分支 |
| D7 | 非阻塞传输成功时 CS 释放交由用户完成回调 | 检查注释和文档 |

## E. 跨系列抽象

| # | 检查项 | 判定方法 |
|---|--------|----------|
| E1 | 系列差异用 `#if defined(STM32F4)` / `#elif defined(STM32G4)` 在 `.c` 内分支 | 检查 `.c` 内条件编译 |
| E2 | 跨系列分支不泄漏到公共 API | 公共 API 签名与系列无关 |
| E3 | HAL 中断回调按系列分别弱覆盖 | 检查 `HAL_XXX_RxCallback` 的条件编译 |
| E4 | MCU 型号由 `drv_config.h` 的 `FLASH_MCU` 选择，展开为系列宏 | 检查 `drv_config.h` |
| E5 | 运行期可获取的参数（FLASH 几何等）不按型号宏硬编码 | 检查是否用 HAL 运行期值 |

## F. 回调注册

| # | 检查项 | 判定方法 |
|---|--------|----------|
| F1 | 中断接收类外设提供 `drv_xxx_register_rx_callback` | 检查公共 API |
| F2 | 回调表是 `static` 数组，按外设编号索引 | 检查 `user_xxx_callback[]` |
| F3 | HAL 中断回调只做：识别外设 → 调 recv → 转发给用户回调 | 检查弱覆盖的 HAL 回调函数 |
| F4 | 回调指针调用前判空 | 检查 `if (user_xxx_callback[id])` |
| F5 | 回调类型定义在公共头，参数用 drv 类型不用 HAL 类型 | 检查 typedef |

## G. 软驱动

| # | 检查项 | 判定方法 |
|---|--------|----------|
| G1 | 软驱动不直接调用 `HAL_GPIO_WritePin` 等 HAL 函数 | 搜索 HAL 调用 |
| G2 | GPIO 操作通过回调注入 | 检查设备描述符的回调指针字段 |
| G3 | 构造函数 `memset(pobj, 0, sizeof(*pobj))` 清零后赋值 | 检查 `drv_xxx_init` |
| G4 | 构造函数检查 `pobj != NULL`（assert 或返回错误） | 检查参数校验 |
| G5 | 时序延时用 `volatile` 计数防优化删除 | 检查 `_xxx_delay` |
| G6 | 方法指针在构造时挂载，不每实例重复存 | 检查 `pobj->write_nbytes = drv_xxx_write_nbytes` |

## H. 实时约束

| # | 检查项 | 判定方法 |
|---|--------|----------|
| H1 | 热路径（PWM 占空比、ADC 读值）直接操作寄存器宏，不触发 HAL 重配置 | 检查 `__HAL_TIM_SET_COMPARE` 等 |
| H2 | 高频 getter 是 `static inline` | 检查 `get_xxx_handle` |
| H3 | ISR 与主循环共享标志位考虑中断安全 | 检查 `flag` 等共享量的访问 |
| H4 | DWT/延时用裸寄存器，不依赖 HAL | 检查 `drv_dwt_timer` |
| H5 | DRV 层不在 ISR 上下文调用阻塞 HAL 接口 | 检查回调函数内的 HAL 调用 |

## I. 文件与配置

| # | 检查项 | 判定方法 |
|---|--------|----------|
| I1 | 文件头注释完整（@file/@brief/@author/@date/修改日志表） | 读文件顶部 |
| I2 | 注明"对外接口不暴露HAL类型"和"初始化由CubeMX完成" | 检查 `@note` |
| I3 | `drv_config.h` 有对应 `USE_XXX_DRIVER` 使能宏 | 检查 Configuration Wizard |
| I4 | `.h` 用 `#ifdef USE_XXX_DRIVER` 包裹整个内容 | 检查包含卫式结构 |
| I5 | 子功能（如 PWM）头单独，实现并入宿主 `.c` | 检查 `drv_tim_pwm.h` 与 `drv_tim.c` 关系 |
| I6 | include guard 命名一致（`_DRV_XXX_H_`） | 检查 `#ifndef` |

## 评审流程

1. 先读 `drv_config.h` 确认该驱动已使能、MCU 型号正确。
2. 读 `.h` 检查 A（HAL 屏蔽）、C（枚举）、D（API）、I（文件）。
3. 读 `.c` 检查 B（句柄表）、E（跨系列）、F（回调）、H（实时）。
4. 若是软驱动，重点检查 G。
5. 汇总违规项，按 A/B 类（核心）优先级最高，I 类（文件）最低给出修正建议。
6. 给出整体结论：通过 / 需修改后通过 / 需重做。
