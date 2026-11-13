"""Pitch 角速度仅计算预览界面。导入模块不会创建应用或网络连接。"""
import argparse
import csv
import math
import sys
import threading
import time
from pathlib import Path

from PySide6 import QtCore, QtWidgets
import pyqtgraph as pg


class Bridge(QtCore.QObject):
    event = QtCore.Signal(str, object)
    done = QtCore.Signal()


class Window(QtWidgets.QMainWindow):
    def __init__(self, demo=False):
        super().__init__()
        self.setWindowTitle('Pitch 角速度：仅计算预览' + ('【离线模拟演示】' if demo else ''))
        self.resize(1100, 950)
        self.demo = demo
        self.rows = []
        self.thread = None
        self.cancel = threading.Event()
        self.last_received = None
        self.last_finish = None
        self.start_attempted = None
        self.closing = False
        self.bridge = Bridge()
        self.bridge.event.connect(self.on_event)
        self.bridge.done.connect(self.on_done)
        root = QtWidgets.QWidget()
        self.setCentralWidget(root)
        layout = QtWidgets.QVBoxLayout(root)
        note = QtWidgets.QLabel('仅计算预览 • 不代表物理输出为零，也不提供物理安全认证。\n'
            '必须物理断开电机电源；不得同时运行其他上位机。\n'
            '控制器预览使用当前固件 PID，不是 C 台架独立的 kp/ki。\n'
            '显示的计算输出不是实际施加输出的测量值。')
        note.setWordWrap(True)
        note.setStyleSheet('background:#552000;color:#fff2c7;padding:12px;font-weight:bold;')
        layout.addWidget(note)
        if demo:
            banner = QtWidgets.QLabel('离线模拟演示：不连接控制器，数据不是物理测量结果')
            banner.setStyleSheet('background:#493475;color:white;padding:8px;font-weight:bold;')
            layout.addWidget(banner)
        form = QtWidgets.QHBoxLayout()
        self.fields = {}
        for name, value in [('Host','192.168.0.111'),('Remote port','5001'),('Bind','0.0.0.0'),('Local port','8081')]:
            form.addWidget(QtWidgets.QLabel({'Host':'目标地址','Remote port':'目标端口','Bind':'本地绑定地址','Local port':'本地端口'}[name]))
            field = QtWidgets.QLineEdit(value)
            form.addWidget(field)
            self.fields[name] = field
        layout.addLayout(form)
        self.affirm = QtWidgets.QCheckBox('我确认已物理断开电机电源，且没有其他上位机正在运行。')
        layout.addWidget(self.affirm)
        controls = QtWidgets.QHBoxLayout()
        self.start_button = QtWidgets.QPushButton('开始一次预览（最长 10 秒）')
        self.stop_button = QtWidgets.QPushButton('停止预览')
        self.export_button = QtWidgets.QPushButton('导出 CSV…')
        self.start_button.clicked.connect(self.start)
        self.stop_button.clicked.connect(self.stop)
        self.export_button.clicked.connect(self.export)
        self.stop_button.setEnabled(False)
        self.start_button.setEnabled(False)
        self.affirm.toggled.connect(lambda checked: self.start_button.setEnabled(checked and self.thread is None))
        for button in (self.start_button,self.stop_button,self.export_button): controls.addWidget(button)
        layout.addLayout(controls)
        self.identity = QtWidgets.QLabel('控制器身份：未核验')
        self.state = QtWidgets.QLabel('未连接。不自动连接、重连或启动。')
        self.age = QtWidgets.QLabel('主机样本年龄：未知（尚无样本）')
        self.details = QtWidgets.QLabel('活动状态 / 轴 / 控制环 / 原因 / 链路新鲜度：未知')
        self.details.setWordWrap(True)
        for label in (self.identity,self.state,self.age,self.details): layout.addWidget(label)
        self.curves = []
        for title, units in [('角度','度'),('角速度','度/秒'),('计算输出（不是实际施加输出）','固件单位')]:
            plot = pg.PlotWidget(title=title)
            plot.setLabel('left',units)
            plot.setLabel('bottom','控制器预览经过时间','ms')
            plot.showGrid(x=True,y=True,alpha=0.25)
            self.curves.append(plot.plot(pen=pg.mkPen('#57c7ff',width=2),connect='finite'))
            layout.addWidget(plot,1)
        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumHeight(110)
        layout.addWidget(self.log)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_age)
        self.timer.start(100)

    def start(self):
        if self.thread is not None or not self.affirm.isChecked(): return
        try:
            host = self.fields['Host'].text().strip()
            bind = self.fields['Bind'].text().strip()
            port = int(self.fields['Remote port'].text())
            local = int(self.fields['Local port'].text())
            if not host or not bind or not (1 <= port <= 65535 and 1 <= local <= 65535):
                raise ValueError('请填写目标地址、本地绑定地址和有效端口（1–65535）。')
        except ValueError as exc:
            QtWidgets.QMessageBox.warning(self,'连接参数无效',str(exc)); return
        self.rows.clear()
        self.last_received = self.last_finish = None
        self.start_attempted = None
        self.identity.setText('控制器身份：未核验')
        self.details.setText('活动状态 / 轴 / 控制环 / 原因 / 链路新鲜度：未知')
        self.update_age()
        for curve in self.curves: curve.setData([],[])
        self.cancel = threading.Event()
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.affirm.setEnabled(False)
        for field in self.fields.values(): field.setEnabled(False)
        self.state.setText('正在启动离线模拟演示…' if self.demo else '正在启动本次手动请求的预览…')
        def work():
            session = None
            try:
                if self.demo:
                    self.bridge.event.emit('identity',{'firmware':'SYNTHETIC','profile':'OFFLINE DEMO'})
                    for i in range(100):
                        if self.cancel.wait(0.1): break
                        self.bridge.event.emit('sample',self.synthetic(i))
                    self.bridge.event.emit('finish',{'confirmed':True,'start_attempted':False,'message':'离线模拟演示结束，未使用控制器。'})
                else:
                    from preview_core import PreviewSession
                    session = PreviewSession(host,port,bind,local,emit=self.bridge.event.emit)
                    session.run(self.cancel)
            except Exception as exc:
                self.bridge.event.emit('log',{'message':f'错误：{type(exc).__name__}: {exc}'})
                self.bridge.event.emit('finish',{'confirmed':False,'start_attempted':getattr(session,'start_attempted',None) if session is not None else False,'message':f'工作线程异常：{type(exc).__name__}: {exc}。请保持电机电源断开。'})
            finally:
                self.bridge.done.emit()
        self.thread = threading.Thread(target=work,name='preview-session',daemon=False)
        self.thread.start()

    @staticmethod
    def synthetic(i):
        return dict(host_monotonic_s=time.monotonic(),elapsed_ms=i*100,angle_deg=5*math.sin(i/10),rate_dps=5*math.cos(i/10),calculated_output_not_applied=0.3*math.cos(i/10),active=True,axis='pitch',ring='rate',reason='SYNTHETIC',flywheel_link_fresh=False,gap=(i==40))

    @QtCore.Slot(str,object)
    def on_event(self,event,data):
        if event == 'sample':
            row = dict(data)
            self.rows.append(row)
            self.last_received = row.get('host_monotonic_s')
            self.details.setText('活动：{} | 轴：{} | 控制环：{} | 原因：{} | 飞轮链路新鲜：{}'.format(*(row.get(k,'未知') for k in ('active','axis','ring','reason','flywheel_link_fresh'))))
            xs = []; ys = [[],[],[]]
            keys = ('angle_deg','rate_dps','calculated_output_not_applied')
            for r in self.rows:
                if r.get('gap'):
                    xs.append(float('nan'))
                    for y in ys: y.append(float('nan'))
                xs.append(r.get('elapsed_ms',float('nan')))
                for y,key in zip(ys,keys):
                    value = r.get(key)
                    y.append(value if isinstance(value,(float,int)) else float('nan'))
            for curve,y in zip(self.curves,ys): curve.setData(xs,y)
            self.update_age()
        elif event == 'identity': self.identity.setText('控制器身份：'+str(data))
        elif event == 'state': self.state.setText(str(data.get('text','')))
        elif event == 'log': self.log.appendPlainText(str(data.get('message','')))
        elif event == 'finish':
            self.last_finish = bool(data.get('confirmed',False))
            self.start_attempted = data.get('start_attempted')
            if self.demo: prefix = '离线模拟：'
            elif self.start_attempted is False: prefix = '未启动预览：'
            elif self.last_finish: prefix = 'MCU 已报告预览停止（不代表物理停机）：'
            else: prefix = '未确认 MCU 预览停止，请保持电机电源断开：'
            self.state.setText(prefix + str(data.get('message','')))
            self.log.appendPlainText(self.state.text())

    def update_age(self):
        if not isinstance(self.last_received,(float,int)) or not math.isfinite(self.last_received):
            self.age.setText('主机样本年龄：未知（尚无有效样本）'); return
        age = max(0,time.monotonic()-self.last_received)
        stale = age > 0.75
        self.age.setText(f'主机样本年龄：{age:.2f} 秒；'+('已过期；保留末次值，不置零' if stale else '主机近期样本（不代表已核验物理输出）'))
        self.age.setStyleSheet('color:#dc6519;font-weight:bold;' if stale else '')

    def stop(self):
        if self.thread is not None:
            self.cancel.set()
            self.state.setText('已请求停止预览，等待控制器确认…' if not self.demo else '正在停止离线演示…')
            self.stop_button.setEnabled(False)

    @QtCore.Slot()
    def on_done(self):
        # The Qt event may arrive before the Python worker has returned.
        if self.thread is not None and self.thread.is_alive():
            QtCore.QTimer.singleShot(25,self.on_done); return
        self.thread = None
        self.stop_button.setEnabled(False)
        self.affirm.setEnabled(True)
        self.affirm.setChecked(False)
        self.start_button.setEnabled(False)
        for field in self.fields.values(): field.setEnabled(True)
        if self.last_finish is None:
            self.state.setText(self.state.text() + '；工作线程已退出，未收到结束状态。请保持电机电源断开。')
            self.log.appendPlainText(self.state.text())
        if self.last_finish is not True and self.start_attempted is not False and not self.demo:
            QtWidgets.QMessageBox.warning(self,'未确认 MCU 预览停止',self.state.text())
        if self.closing: self.close()

    def export(self):
        if not self.rows:
            QtWidgets.QMessageBox.information(self,'暂无数据','没有可导出的样本。'); return
        filename,_ = QtWidgets.QFileDialog.getSaveFileName(self,'将样本导出到新的 CSV 文件（不覆盖已有文件）','','CSV (*.csv)')
        if not filename: return
        fields = ['host_monotonic_s','elapsed_ms','angle_deg','rate_dps','calculated_output_not_applied','active','axis','ring','reason','flywheel_link_fresh','gap']
        try:
            with open(filename,'x',newline='',encoding='utf-8-sig') as f:
                writer=csv.DictWriter(f,fieldnames=['data_source']+fields,extrasaction='ignore')
                writer.writeheader()
                for row in self.rows: writer.writerow(dict(row,data_source='OFFLINE_SYNTHETIC' if self.demo else 'CONTROLLER_CALCULATION_PREVIEW'))
        except Exception as exc:
            QtWidgets.QMessageBox.warning(self,'导出失败',str(exc)); return
        self.log.appendPlainText('已导出：'+filename)

    def closeEvent(self,event):
        if self.thread is not None:
            event.ignore()
            if self.closing: return
            answer=QtWidgets.QMessageBox.question(self,'预览仍在运行','是否请求停止预览，并在工作线程退出后关闭窗口？',QtWidgets.QMessageBox.StandardButton.Yes|QtWidgets.QMessageBox.StandardButton.No)
            if answer == QtWidgets.QMessageBox.StandardButton.Yes:
                self.closing=True; self.stop()
            return
        event.accept()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--demo',action='store_true',help='离线模拟演示；不创建网络连接')
    parser.add_argument('--smoke',metavar='NEW_PNG',help='将离线模拟界面保存为新 PNG 文件后退出；不创建网络连接')
    args=parser.parse_args()
    app=QtWidgets.QApplication(sys.argv[:1])
    window=Window(demo=args.demo or bool(args.smoke))
    window.show()
    if args.smoke:
        target=Path(args.smoke)
        if target.exists(): raise FileExistsError(target)
        for i in range(80): window.on_event('sample',window.synthetic(i))
        window.state.setText('离线界面测试：仅模拟回调，未创建网络连接。')
        def capture():
            # Exclusive creation prevents overwriting a pre-existing file.
            buffer=QtCore.QBuffer(); buffer.open(QtCore.QIODevice.OpenModeFlag.WriteOnly)
            if not window.grab().save(buffer,'PNG'): raise RuntimeError('PNG 截图失败')
            with target.open('xb') as f: f.write(bytes(buffer.data()))
            app.quit()
        QtCore.QTimer.singleShot(300,capture)
    return app.exec()


if __name__ == '__main__':
    raise SystemExit(main())
