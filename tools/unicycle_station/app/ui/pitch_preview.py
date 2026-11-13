from PySide6 import QtWidgets as W
from app.plotting.scope import PlotPanel


class PitchPreviewPanel(W.QGroupBox):
    def __init__(self, engine, plots):
        super().__init__("主轮 Pitch · 零输出预览")
        self.engine, self.plots = engine, plots
        self.setObjectName("pitch_preview_panel")
        layout = W.QVBoxLayout(self)
        layout.setContentsMargins(8, 5, 8, 5)
        self.note = W.QLabel("使用 MCU 已应用的 Pitch PID 参数运行正式控制器；最长 10 秒，计算输出不施加到电机。")
        self.note.setWordWrap(True)
        layout.addWidget(self.note)
        row = W.QHBoxLayout()
        self.read = W.QPushButton("读取预览状态")
        self.start = W.QPushButton("开始预览")
        self.stop = W.QPushButton("停止预览")
        self.read.clicked.connect(lambda: engine.submit("pitch_preview", "read"))
        self.start.clicked.connect(self.start_preview)
        # Same priority STOP path as the existing red button and Space shortcut.
        self.stop.clicked.connect(lambda: engine.emergency_stop())
        for button in (self.read, self.start, self.stop):
            row.addWidget(button)
        row.addStretch()
        layout.addLayout(row)
        self.state = W.QLabel()
        self.state.setWordWrap(True)
        layout.addWidget(self.state)
        self.graphs = {}
        for ring, title, channel, tab in (
                ("angle", "Pitch 预览角度", "preview_pitch", "预览角度 · °"),
                ("rate", "Pitch 预览角速度", "preview_pitch_rate", "预览角速度 · °/s"),
                ("output", "控制器计算输出（未施加）", "preview_output", "预览计算输出")):
            graph = PlotPanel(engine.pitch_preview.store, title, [channel])
            plots.addTab(graph, tab)
            plots.panels.append(graph)
            self.graphs[ring] = graph
        index = plots.ring.findData("p_rate_")
        if index >= 0:
            plots.ring.setCurrentIndex(index)
        self.update_data()

    def selected_ring(self):
        return {"p_rate_": "rate", "p_angle_": "angle"}.get(self.plots.ring.currentData())

    def start_preview(self):
        ring = self.selected_ring()
        if ring:
            self.plots.setCurrentWidget(self.graphs[ring])
            self.engine.submit("pitch_preview", "start", ring)

    def update_data(self):
        service = self.engine.pitch_preview
        connected = self.engine.connected and self.engine.protocol_ready and not self.engine.replay
        pending = bool(self.engine.requests.pending or self.engine.requests.queue)
        self.read.setEnabled(connected and not pending)
        self.start.setEnabled(bool(connected and not pending and not service.busy
                                   and service.supported is not False and self.selected_ring()
                                   and self.engine.stopped()))
        self.stop.setEnabled(bool(self.engine.connected and not self.engine.replay))
        self.plots.ring.setEnabled(not service.busy)
        lines = [service.message]
        status = service.latest
        if status and (status["axis"] == 1 or status["active"]):
            ring = "角速度环" if status["ring"] == 0 else "角度环"
            lines.append(f"{'Pitch' if status['axis']==1 else 'Roll'} · {ring} · {status['elapsed_ms']} ms · "
                         f"角度 {status['angle']:.3f}° · 角速度 {status['rate']:.3f}°/s · "
                         f"计算输出 {status['output']:.3f}（未施加）")
        if not self.selected_ring():
            lines.append("请选择角速度环或角度环；当前固件未提供速度环预览。")
        lines.append("预览状态约 10 Hz 采样；带动力闭环整定尚未启用。" if self.engine.mode != "mock"
                     else "Mock 示例数据，仅演示界面，不是固件 PID 仿真或实车验证。")
        self.state.setText("\n".join(lines))
