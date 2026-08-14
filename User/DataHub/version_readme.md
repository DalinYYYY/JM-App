# 版本记录

> 版本号定义见 `User/DataHub/version.h`。每次主仓库提交递增 `APP_VERSION_PATCH`，并在下表追加对应条目。

| 版本 | 日期 | 类型 | 备注 |
| --- | --- | --- | --- |
| 1.0.1 | 2026-08-07 | baseline | 现有基线版本（DataHub 重构后） |
| 1.0.2 | 2026-08-07 | feat | 新增 motor_observer 实时快照与高速采样（B7/B6/C9） |
| 1.0.3 | 2026-08-07 | feat | 增强 PID 整定/校验与调试会话（autotune/validate/A1/A2/A5，含 B7/B6/C9 协议接口）；同步 pyqt_gui PID 调试 UI |
| 1.0.4 | 2026-08-07 | fix | system_state 调整：注释 fault_check 调用、CALIB 态切换逻辑收敛与缩进整理 |
| 1.0.5 | 2026-08-07 | docs | 更新协议文档与命令清单（高速采样 B7/B6/C9、PID 调试 A1/A2/A5） |
| 1.0.6 | 2026-08-07 | build | 更新 Keil 编译分析结果（新增 motor_observer 后的 Flash/RAM 占用） |
| 1.1.0 | 2026-08-07 | feat | 高速波形重构为连续 TRACE 流：环形缓冲+设备主动批量上报（0xB9/0xBA），废弃旧版 B7/B6/C9 采样（不兼容，次版本升级），协议升至 1.2 |
| 1.1.1 | 2026-08-07 | docs | 更新协议文档与命令清单（TRACE 0xB9/0xBA、废弃 B5~B7/C9 波形） |
| 1.1.2 | 2026-08-07 | build | 更新 Keil 编译分析结果（TRACE 重构后 Flash/RAM 占用） |
| 1.1.3 | 2026-08-07 | feat | 删除 VESC/SerialStudio 通信模块（dev_commun_vesc/vesc_proto/serial_studio）并清理冗余注释与死代码 |
| 1.1.4 | 2026-08-07 | docs+build | 更新 readme 与协议文档（移除 VESC/SerialStudio 引用）、Keil 工程移除 vesc_proto 头文件路径 |
| 1.1.5 | 2026-08-07 | chore | 同步 pyqt_gui 子模块至 TRACE 主动波形适配版本；清理本地计划/HTML 模式文档并加入 .gitignore，整理 jm_cmd_def.h 注释对齐 |
| 1.2.0 | 2026-08-14 | feat | motor_info 增加片外 EEPROM 双备份存储（AT24C16，上电优先加载）与 Flash 磨损均衡写入次数保护；新增 SFOC_V2 板级支持；协议新增 EEPROM/写限错误码；同步 pyqt_gui 固化通知与 SFOC_V2 驱动（次版本升级） |