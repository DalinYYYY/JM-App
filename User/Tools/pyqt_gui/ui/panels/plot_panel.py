"""绘图面板(占位)

预留实时曲线接口, 本次不集成 pyqtgraph。主窗口可直接调用 append(),
后续把内部实现换成 pyqtgraph.PlotWidget 即可, 不影响其他层。
"""

from collections import deque, defaultdict

from PyQt6.QtWidgets import QGroupBox, QVBoxLayout, QLabel
from PyQt6.QtCore import Qt


class PlotPanel(QGroupBox):
    """实时曲线占位。接口与未来 pyqtgraph 实现保持一致。"""

    MAX_POINTS = 2000

    def __init__(self, parent=None):
        super().__init__("实时曲线 (预留)", parent)
        self._series = defaultdict(lambda: deque(maxlen=self.MAX_POINTS))
        self._build()

    def _build(self):
        layout = QVBoxLayout(self)
        self._hint = QLabel(
            "绘图功能已预留接口, 暂未集成。\n"
            "后续安装 pyqtgraph 后, 在此面板内替换为 PlotWidget 即可,\n"
            "主窗口已通过 append(name, ts, value) 持续喂入数据, 无需改其他层。")
        self._hint.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._hint.setStyleSheet("color: #888; padding: 20px;")
        self._hint.setWordWrap(True)
        layout.addWidget(self._hint)

    # ---- 预留接口(签名稳定) ----
    def add_series(self, name: str):
        """注册一条曲线(占位实现只建缓冲)"""
        _ = self._series[name]

    def append(self, name: str, ts: float, value: float):
        """追加一个数据点(占位实现只入缓冲, 不绘制)"""
        self._series[name].append((ts, value))

    def clear(self):
        self._series.clear()

    def feed_feedback(self, ts: float, fb):
        """便捷入口: 把一帧反馈的常用量喂入对应曲线"""
        for name in ("pos", "vel", "torque", "iq", "vbus"):
            self.append(name, ts, getattr(fb, name, 0.0))
