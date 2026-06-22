"""关节电机上位机主窗口: 组装各面板 + 连接信号

主窗口只负责: 创建 JmClient(注入 SerialTransport)、组装面板、连接信号槽、
管理连接状态与轮询兜底定时器。具体 UI 细节都在各 panel 内。
"""

import time

from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QTabWidget,
    QGroupBox, QGridLayout, QPushButton, QMessageBox,
)
from PyQt6.QtCore import QTimer

import jmproto as jp
from jmproto import JmCmd, cmd_name, err_name
from transport.serial_transport import SerialTransport
from core.motor_client import JmClient

from ui.panels.connection_panel import ConnectionPanel
from ui.panels.control_panel import ControlPanel
from ui.panels.motion_panel import MotionPanel
from ui.panels.param_panel import ParamPanel
from ui.panels.feedback_panel import FeedbackPanel
from ui.panels.telemetry_panel import TelemetryPanel
from ui.panels.plot_panel import PlotPanel
from ui.panels.log_panel import LogPanel


class MainWindow(QMainWindow):
    """关节电机上位机主窗口"""

    def __init__(self):
        super().__init__()

        # 通信客户端(串口传输)
        self._client = JmClient(SerialTransport())
        self._registry = self._client.registry

        # 轮询兜底定时器
        self._poll_timer = QTimer()
        self._poll_timer.timeout.connect(self._on_poll_tick)
        self._poll_period = 100

        self.setWindowTitle("Joint Motor Controller - 关节电机控制面板")
        self.setMinimumSize(1200, 800)

        self._build_ui()
        self._connect_signals()

        self._client.start()

        # 启动告警(CSV 加载情况)
        for w in self._registry.warnings:
            self._log_panel.log(f"[WARN] {w}")
        self._log_panel.log(
            f"协议表加载: 命令 {len(self._registry.commands)} 条, 参数 {len(self._registry.params)} 个")

    # ==================== UI 组装 ====================
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)

        # 左侧: 控制类面板
        left = QWidget()
        left.setMaximumWidth(420)
        left_layout = QVBoxLayout(left)

        self._conn_panel = ConnectionPanel()
        self._control_panel = ControlPanel()
        self._motion_panel = MotionPanel(self._registry)
        self._telemetry_panel = TelemetryPanel()

        left_layout.addWidget(self._conn_panel)
        left_layout.addWidget(self._control_panel)
        left_layout.addWidget(self._motion_panel)
        left_layout.addWidget(self._telemetry_panel)
        left_layout.addWidget(self._create_dev_info_group())
        left_layout.addStretch()

        # 右侧: 反馈/参数/绘图 选项卡 + 日志
        right = QWidget()
        right_layout = QVBoxLayout(right)

        self._feedback_panel = FeedbackPanel()
        self._param_panel = ParamPanel(self._registry)
        self._plot_panel = PlotPanel()
        self._log_panel = LogPanel()

        tabs = QTabWidget()
        tabs.addTab(self._feedback_panel, "实时反馈")
        tabs.addTab(self._param_panel, "参数读写")
        tabs.addTab(self._plot_panel, "实时曲线")

        right_layout.addWidget(tabs, 3)
        right_layout.addWidget(self._log_panel, 2)

        root.addWidget(left)
        root.addWidget(right, 1)

        self.statusBar().showMessage("未连接")

    def _create_dev_info_group(self) -> QGroupBox:
        grp = QGroupBox("设备信息")
        layout = QGridLayout(grp)
        btn_info = QPushButton("读设备信息")
        btn_info.clicked.connect(lambda: self._client.query_dev_info())
        btn_name = QPushButton("读设备名称")
        btn_name.clicked.connect(lambda: self._client.query_dev_name())
        btn_hb = QPushButton("心跳")
        btn_hb.clicked.connect(lambda: self._client.heartbeat())
        layout.addWidget(btn_info, 0, 0)
        layout.addWidget(btn_name, 0, 1)
        layout.addWidget(btn_hb, 0, 2)
        return grp

    # ==================== 信号连接 ====================
    def _connect_signals(self):
        c = self._client
        c.connected.connect(self._on_connected)
        c.error_occurred.connect(self._on_error)
        c.tx_log.connect(self._log_panel.log)
        c.feedback_updated.connect(self._on_feedback)
        c.state_updated.connect(self._feedback_panel.update_state)
        c.ack_received.connect(self._on_ack)
        c.nack_received.connect(self._on_nack)
        c.dev_info_received.connect(self._on_dev_info)
        c.dev_name_received.connect(self._on_dev_name)
        c.param_read_result.connect(self._on_param_result)
        c.raw_frame.connect(self._log_panel.log_raw_rx)

        # 面板 -> 客户端
        self._conn_panel.connect_requested.connect(self._on_connect)
        self._conn_panel.disconnect_requested.connect(self._on_disconnect)
        self._control_panel.command.connect(self._on_control_command)
        self._motion_panel.send_command.connect(self._on_motion_command)
        self._telemetry_panel.apply_telemetry.connect(self._on_apply_telemetry)
        self._telemetry_panel.poll_toggled.connect(self._on_poll_toggled)
        self._param_panel.read_param.connect(self._on_param_read)
        self._param_panel.write_param.connect(self._on_param_write)
        self._param_panel.save_all.connect(lambda: self._client.param_save())

    # ==================== 连接管理 ====================
    def _on_connect(self, port: str, baud: int):
        if self._client.open(port=port, baudrate=baud):
            self.statusBar().showMessage(f"已连接 {port} @{baud}")

    def _on_disconnect(self):
        self._poll_timer.stop()
        self._client.close()
        self.statusBar().showMessage("已断开")

    def _on_connected(self, connected: bool):
        self._conn_panel.set_connected(connected)
        if not connected:
            self._poll_timer.stop()

    def _on_error(self, msg: str):
        self._log_panel.log(f"[ERROR] {msg}")
        self.statusBar().showMessage(msg)

    # ==================== 命令下发 ====================
    def _ensure_open(self) -> bool:
        if not self._client.is_open():
            QMessageBox.warning(self, "提示", "请先连接串口")
            return False
        return True

    def _on_control_command(self, cmd: int):
        if self._ensure_open():
            self._client.send_command(cmd)

    def _on_motion_command(self, cmd: int, values: dict):
        if not self._ensure_open():
            return
        try:
            self._client.send_command(cmd, values)
        except Exception as e:
            QMessageBox.critical(self, "错误", f"发送异常: {e}")

    def _on_apply_telemetry(self, mask: int, period_ms: int):
        if self._ensure_open():
            self._client.set_telemetry(mask, period_ms)

    def _on_poll_toggled(self, enabled: bool, period: int):
        self._poll_period = period
        if enabled and self._client.is_open():
            self._poll_timer.start(period)
        else:
            self._poll_timer.stop()

    def _on_poll_tick(self):
        if self._client.is_open():
            self._client.query_feedback()
            self._client.query_state()

    # ==================== 参数读写 ====================
    def _on_param_read(self, param_id: int):
        if self._ensure_open():
            self._client.param_read(param_id)

    def _on_param_write(self, param_id: int, text: str):
        if not self._ensure_open():
            return
        try:
            value = self._registry.pack_param_value(param_id, text)
        except ValueError:
            QMessageBox.warning(self, "错误", f"参数值无效: {text}")
            return
        self._client.param_write(param_id, value)

    def _on_param_result(self, param_id: int, ptype: int, value_bytes: bytes):
        val = self._registry.unpack_param_value(param_id, value_bytes)
        disp = str(val) if val is not None else value_bytes.hex(' ')
        self._param_panel.set_value(param_id, disp)
        spec = self._registry.get_param(param_id)
        name = spec.code_name if spec else f"id={param_id}"
        self._log_panel.log(f"[RX] PARAM {name}(id={param_id}) = {disp}")

    # ==================== 数据接收 ====================
    def _on_feedback(self, fb):
        self._feedback_panel.update_feedback(fb)
        self._plot_panel.feed_feedback(time.monotonic(), fb)

    def _on_ack(self, cmd: int):
        self._log_panel.log(f"[RX] ACK {cmd_name(cmd)}(0x{cmd:02X})")

    def _on_nack(self, cmd: int, err: int):
        self._log_panel.log(f"[RX] NACK {cmd_name(cmd)}(0x{cmd:02X}) err={err_name(err)}(0x{err:02X})")

    def _on_dev_info(self, hw: int, fw: int, uid: bytes):
        uid_hex = uid.hex(':').upper()
        self._log_panel.log(f"[RX] DEV_INFO HW=0x{hw:08X} FW=0x{fw:08X} UID={uid_hex}")
        QMessageBox.information(
            self, "设备信息",
            f"硬件版本: 0x{hw:08X}\n固件版本: 0x{fw:08X}\nUID: {uid_hex}")

    def _on_dev_name(self, name: str):
        self._log_panel.log(f'[RX] DEV_NAME="{name}"')
        QMessageBox.information(self, "设备名称", f"设备名称: {name}")

    # ==================== 退出 ====================
    def closeEvent(self, event):
        self._poll_timer.stop()
        self._client.stop()
        event.accept()
