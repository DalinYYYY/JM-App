# dev 设备对象层评审清单

> 评审 `User/Devices/` 下的 dev 设备代码时逐项检查。每项给出"通过/违规/不适用"判定，违规项必须给出修正建议。

## A. 配置表（核心）

| # | 检查项 | 判定方法 |
|---|--------|----------|
| A1 | 设备有 `xxx_config_t` 配置表结构，含 `name[20]` + 硬件资源字段 | 检查 `.h` 的 typedef |
| A2 | `.h` 声明 `extern const xxx_config_t xxx_list[XXX_ID_MAX];` | 检查 extern 声明 |
| A3 | 配置表在板级 `.inc` 用 designated initializer 填充 | 检查 `Board/XXX/Config/dev_config_board.inc` |
| A4 | 配置表用 `#if defined(USE_DEV_XXX)` 包裹 | 检查 `.inc` 中的条件编译 |
| A5 | `dev_config.c` include 了该设备的 `.h` | 检查 `dev_config.c` 的 include 块 |
| A6 | 方法实现通过 `xxx_list[pobj->id]` 取硬件资源，不硬编码引脚/通道 | 搜索 `GPIOA`/`DRV_PIN_`/`DRV_SPI1` 等硬编码 |
| A7 | 没有用 `switch(id){case XXX_ID_1: ...}` 选资源 | 检查 switch 语句 |
| A8 | 板级 `.h` 加了对应的 `USE_DEV_XXX` 使能宏 | 检查 `dev_config_board.h` |

## B. OOP 设备对象（核心）

| # | 检查项 | 判定方法 |
|---|--------|----------|
| B1 | 设备对象 `dev_xxx_t` 含状态字段 + 方法指针 | 检查 struct 定义 |
| B2 | 方法指针第一个参数是 `struct dev_xxx *pobj` | 检查方法指针签名 |
| B3 | 构造函数 `dev_xxx_init(pobj, id)` 做 `memset` 清零 | 检查 init 函数 |
| B4 | 构造函数装配所有方法指针，无遗漏 | 对比 struct 方法字段与 init 赋值 |
| B5 | 构造函数不启动硬件（不调 `HAL_xxx_Start`/`drv_xxx_start`） | 检查 init 中是否有启动调用 |
| B6 | 方法实现是 `static` 函数，只通过方法指针对外暴露 | 检查函数修饰符 |
| B7 | 入口 `assert_report(pobj != NULL)` 校验 | 检查每个方法开头 |
| B8 | 设备对象不用 `malloc` 分配，调用者提供存储 | 检查是否有 malloc/calloc |
| B9 | 方法指针命名用动词（`update`/`start`/`set_xxx`），不带 `dev_` 前缀 | 检查方法名 |
| B10 | 新设备 init 返回状态码（`int`），而非 `void` | 检查 init 返回类型（既有 void 保持兼容，新设备优先返回状态码） |
| B11 | 所有实例共享相同方法时，用共享 `static const ops` 表而非内联方法指针 | 检查是否有 `s_xxx_ops` 共享表 + `const xxx_ops_t *ops` 字段 |
| B12 | 共享 ops 表是 `static const`（进 Flash 只读） | 检查 ops 表的修饰符 |
| B13 | 若用共享 ops 表，对外有包装函数检查 `ops`/`fn` 空指针 | 检查包装函数实现 |

## C. 板级隔离

| # | 检查项 | 判定方法 |
|---|--------|----------|
| C1 | 设备代码不含板级特定路径（`Board/V1/`/`Board/SFOC/`） | 搜索 `Board/` 字符串 |
| C2 | 设备代码不直接 include 板级头（`dev_config_board.h`） | 检查 include 块 |
| C3 | 板级差异（外设编号、引脚）全在 `.inc` 配置表 | 对比 V1 与 SFOC 的 `.inc` |
| C4 | `board_select.h` 的 `JM_BOARD_DEV_CONFIG_INC` 宏指向正确 `.inc` | 检查宏定义 |
| C5 | 新增板子只需加 `Board/XXX/Config/` 两个文件，不改设备代码 | 验证添加新板的改动范围 |

## D. 抽象接口适配

| # | 检查项 | 判定方法 |
|---|--------|----------|
| D1 | 同类设备（编码器）有抽象接口（如 `dev_encoder_t`） | 检查抽象接口定义 |
| D2 | 抽象接口用 `void *ctx` 指向具体对象 | 检查接口字段 |
| D3 | 每种具体芯片有适配函数 | 检查 `encoder_xxx_update` 等 |
| D4 | 适配函数用 `#if (XXX_TYPE == ...)` 条件编译 | 检查条件编译 |
| D5 | 控制层只调抽象接口，不直接调具体芯片方法 | 搜索控制层是否出现 `mt6701.`/`mt6835.` |
| D6 | 切换型号只改宏值，不改控制层代码 | 验证切换流程 |
| D7 | 有 `#else #error` 捕获未知型号宏值 | 检查条件编译末尾 |
| D8 | 若用继承（base 嵌套），基类作为派生结构体首字段且命名为 `base` | 检查 struct 字段顺序与命名 |
| D9 | 下转型用 `container_of` 宏，不在派生实现外做 | 搜索 `container_of` 使用位置 |

## D2. 设备抽象领域边界

| # | 检查项 | 判定方法 |
|---|--------|----------|
| DB1 | 驱动/传感器只持有自身固有参数，不塞上层领域参数 | 检查 struct 字段：编码器无 `pole_pairs`/`electrical_angle` |
| DB2 | 编码器只输出机械角，电角度换算在电机/控制层 | 检查编码器是否有 `electrical_angle` 字段或输出 |
| DB3 | 相电流采样只出电流/电压，不含 PID 参数 | 检查 `dev_phase_current_t` 字段 |
| DB4 | 半桥驱动只出 PWM/ARR，不含电流环输出 | 检查 `dev_half_bridge_t` 字段 |
| DB5 | 同一参数不在两个模块各存一份 | 检查是否有参数重复（如 `poles` 同时在编码器和电机层） |

## D3. 封装

| # | 检查项 | 判定方法 |
|---|--------|----------|
| EP1 | 外部模块不直接写另一个设备的内部字段 | 搜索 `deviceA.xxx =` 形式的跨模块写 |
| EP2 | 需要修改外部对象状态时，通过窄接口（`set_xxx` 方法） | 检查是否有 setter 方法 |
| EP3 | 只读查询用 `const dev_xxx_t *` 参数或 getter 方法 | 检查 getter 方法 |
| EP4 | 配置参数用 `const xxx_config_t *` 传递 | 检查配置传递方式 |

## E. 组合装配

| # | 检查项 | 判定方法 |
|---|--------|----------|
| E1 | 组合设备（如 `dev_motor_t`）把子设备作为字段嵌入 | 检查 struct 字段 |
| E2 | 组合设备 init 逐一调子设备 init | 检查 init 函数调用链 |
| E3 | 抽象接口在组合 init 中装配（`ctx` + 方法指针） | 检查装配代码 |
| E4 | 外部回调通过组合设备注入到子模块 | 检查回调注入 |
| E5 | 组合设备 init 顺序合理（先底层后上层） | 检查 init 调用顺序 |
| E6 | 组合设备 init 有 `assert_report` 校验所有回调非空 | 检查参数校验 |

## F. 通信设备四段式

| # | 检查项 | 判定方法 |
|---|--------|----------|
| F1 | 有 `set_ops`/`start`/`on_rx_idle`/`poll`/`report` 四段式 API | 检查方法指针 |
| F2 | 协议栈实例作为设备对象首成员 | 检查 struct 字段顺序 |
| F3 | `start` 启动 DMA+空闲中断，不阻塞 | 检查 start 实现 |
| F4 | `on_rx_idle` 在 ISR 调用，只落数据不做协议解析 | 检查 on_rx_idle 实现 |
| F5 | `poll` 在主循环调用，喂协议栈并自动回复 | 检查 poll 实现 |
| F6 | 有 `started` 标志防止重复启动 | 检查 start 逻辑 |
| F7 | 有 `last_error` 和调试计数字段 | 检查调试字段 |
| F8 | `set_ops` 注入的业务回调在 `poll` 中被协议栈调用 | 检查回调链路 |

## G. 实时与安全

| # | 检查项 | 判定方法 |
|---|--------|----------|
| G1 | 热路径方法（如 `set_3pwm`）用缓存值，不读 HAL | 检查 `pobj->autoreload` 等缓存 |
| G2 | `start` 时缓存热路径所需参数 | 检查 start 中的缓存赋值 |
| G3 | 有软急停开关（`output_enable`），强制 0 占空比 | 检查 set_3pwm 的 output_enable 分支 |
| G4 | CRC/校验失败的帧丢弃，保持上一帧有效值 | 检查坏帧处理逻辑 |
| G5 | 有连续坏帧计数 `err_cnt` | 检查错误计数字段 |
| G6 | 滤波状态存在对象里（`prev_current`），不用 static 全局 | 检查滤波状态字段位置 |
| G7 | 多实例可重入（无 static 可变状态） | 搜索 `static` 变量（除 const 表和查找表） |
| G8 | ISR 中调用的方法不做阻塞操作 | 检查 on_rx_idle 中的调用 |
| G9 | ISR 与主循环共享的标志位加 `volatile` | 检查 `started`/`flag` 等共享字段的修饰符 |
| G10 | 多字节共享量配合关中断访问（volatile 不保证原子性） | 检查多字节共享量的访问方式 |

## H. 校准与标定

| # | 检查项 | 判定方法 |
|---|--------|----------|
| H1 | 有 `set_zero`/`set_offset` 手动设置零点 | 检查零点设置方法 |
| H2 | 有 `calibrate_zero`/`calibrate_offset` 自动标定 | 检查标定方法 |
| H3 | 自动标定用多次采样取均值 | 检查标定实现 |
| H4 | 标定方法注释说明"须在电机不通电时调用"等前提 | 检查注释 |
| H5 | 零点/偏移存在对象里（`pobj->offset`），不用全局 | 检查偏移字段位置 |

## I. 文件与配置

| # | 检查项 | 判定方法 |
|---|--------|----------|
| I1 | 文件头注释完整（@file/@brief/@author/@date/修改日志表） | 读文件顶部 |
| I2 | 注明"资源配置在 dev_config_board.inc"和"init只装配接口" | 检查 `@note` |
| I3 | `.h` 用 `#ifdef USE_DEV_XXX` 包裹整个内容 | 检查包含卫式结构 |
| I4 | `.c` 用 `#if defined(USE_DEV_XXX)` 包裹整个实现 | 检查条件编译 |
| I5 | include guard 命名一致（`__DEV_XXX_H_`） | 检查 `#ifndef` |
| I6 | `dev_config.h` 有该设备的常量宏（如分辨率、增益） | 检查 `#if defined(USE_DEV_XXX)` 区块 |
| I7 | 设备常量宏用 `#if defined(USE_DEV_XXX)` 包裹 | 检查常量宏的条件编译 |
| I8 | 有 `extern "C"` 包裹（兼容 C++） | 检查 `#ifdef __cplusplus` |

## J. 单例与命名

| # | 检查项 | 判定方法 |
|---|--------|----------|
| J1 | 全局唯一设备用 `extern` 单例声明 | 检查 `extern dev_xxx_t dev_xxx;` |
| J2 | 单例仍需调 `init` 构造 | 检查使用方是否调 init |
| J3 | 文件名 `dev_xxx.c/.h`，带 `dev_` 前缀 | 检查文件名 |
| J4 | 函数名 `dev_xxx_init`/`dev_xxx_<action>`，带 `dev_` 前缀 | 检查公共函数名 |
| J5 | 类型名 `dev_xxx_t`/`xxx_config_t`/`xxx_id_e`，后缀规范 | 检查 typedef |
| J6 | 枚举常量 `XXX_ID_1`/`XXX_ID_MAX`，带 `XXX_` 前缀 | 检查枚举值 |
| J7 | 错误码用 `DEV_EOK`/`DEV_ERROR`，不用 `0`/`1`/`HAL_OK` | 检查返回值 |

## 评审流程

1. 先读 `dev_config.h` 确认该设备已使能、常量宏正确。
2. 读板级 `dev_config_board.h` 确认 `USE_DEV_XXX` 已定义。
3. 读板级 `dev_config_board.inc` 确认配置表已填充。
4. 读 `.h` 检查 A（配置表）、B（OOP 对象）、D（抽象接口）、J（命名）。
5. 读 `.c` 检查 B（构造函数/方法）、E（组合装配）、F（通信四段式）、G（实时安全）、H（校准）。
6. 若是组合设备，重点检查 E。
7. 若是通信设备，重点检查 F。
8. 汇总违规项，按 A/B/D 类（核心）优先级最高，I/J 类（文件/命名）最低给出修正建议。
9. 给出整体结论：通过 / 需修改后通过 / 需重做。
