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
| 1.3.0 | 2026-08-14 | feat | 新增 Bode 扫频测试：0x76 改造为偏置扫频 12B 整数载荷（control/point_cfg/起止频率/幅值/偏置/格式标签），支持力矩→速度/电流→速度/电流→电流/速度→位置/速度→速度/位置→位置 6 种闭环模式；多圈零点复位接口；TRACE 扩展 512 样本缓冲与 SWEEP/POINT_START/POINT_END 标志，UART 链路单周期 4 包排空支撑 2k~10kHz 上传；协议升至 1.4（次版本升级） |
| 1.3.1 | 2026-08-14 | fix | 修复 MKS5010 电机电气参数数量级错误（flux/KT/KE 由 e-5/e-3 级别修正 100 倍），力矩模式 iq=τ/KT 放大导致母线被拉垮 |
| 1.4.0 | 2026-08-21 | feat | 新增对拖台负载模拟：协议 0x60~0x67 命令段（被动恒转矩/动态转矩/平方负载/恒功率/摩擦/惯量/工况谱/冲击过载）、零力模式独立实现；TRACE 链路重构（UART 主走 CAN 兜底）、调试通道增转矩参考；电机选型改板级 motor_config_board.h；协议升至 1.5（次版本升级，HW 版本归位 1.0） |
| 1.4.1 | 2026-08-24 | feat | L2.1 相序识别重构为开环旋转平均法：累计编码器原始角整电周期平均判定 enc_direction 并持久化至 motor_info.direction（index 19），修复原 120° 步进法不能区分相序/计数反向且未写回方向的问题；同步标定配置与参数表注释 |