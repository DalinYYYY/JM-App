"""通信日志面板"""

import datetime

from PyQt6.QtWidgets import (
    QGroupBox, QVBoxLayout, QHBoxLayout, QWidget, QPushButton,
    QCheckBox, QPlainTextEdit,
)
from PyQt6.QtGui import QFont

from jmproto import cmd_name


class LogPanel(QGroupBox):
    """通信日志(TX/RX/原始帧)"""

    def __init__(self, parent=None):
        super().__init__("通信日志", parent)
        layout = QVBoxLayout(self)

        self._log = QPlainTextEdit()
        self._log.setReadOnly(True)
        self._log.setMaximumBlockCount(1000)
        self._log.setFont(QFont("Consolas", 9))
        layout.addWidget(self._log)

        btn_row = QWidget()
        btn_layout = QHBoxLayout(btn_row)
        btn_layout.setContentsMargins(0, 0, 0, 0)

        self._btn_clear = QPushButton("清空日志")
        self._btn_clear.clicked.connect(self._log.clear)
        btn_layout.addWidget(self._btn_clear)

        self.chk_raw = QCheckBox("原始帧日志")
        self.chk_raw.setChecked(False)
        btn_layout.addWidget(self.chk_raw)
        btn_layout.addStretch()

        layout.addWidget(btn_row)

    def log(self, msg: str):
        ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self._log.appendPlainText(f"[{ts}] {msg}")

    def log_raw_rx(self, cmd: int, payload: bytes):
        if self.chk_raw.isChecked():
            hex_data = payload.hex(' ') if payload else "(empty)"
            self.log(f"[RAW] RX {cmd_name(cmd)}(0x{cmd:02X}) [{len(payload)}B] {hex_data}")
