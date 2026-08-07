# 版本记录

> 版本号定义见 `User/DataHub/version.h`。每次主仓库提交递增 `APP_VERSION_PATCH`，并在下表追加对应条目。

| 版本 | 日期 | 类型 | 备注 |
| --- | --- | --- | --- |
| 1.0.1 | 2026-08-07 | baseline | 现有基线版本（DataHub 重构后） |
| 1.0.2 | 2026-08-07 | feat | 新增 motor_observer 实时快照与高速采样（B7/B6/C9） |
| 1.0.3 | 2026-08-07 | feat | 增强 PID 整定/校验与调试会话（autotune/validate/A1/A2/A5，含 B7/B6/C9 协议接口）；同步 pyqt_gui PID 调试 UI |
| 1.0.4 | 2026-08-07 | fix | system_state 调整：注释 fault_check 调用、CALIB 态切换逻辑收敛与缩进整理 |