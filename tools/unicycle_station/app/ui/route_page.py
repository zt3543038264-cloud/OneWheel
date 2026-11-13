import json
import math
from pathlib import Path
from PySide6 import QtCore, QtWidgets as W
import pyqtgraph as pg
from app.services.routes import resample, document, read_route


class RoutePage(W.QWidget):
    def __init__(self,engine):
        super().__init__()
        self.engine=engine; self.points=((0.,0.),(0.,3.)); self.seen_event=0
        layout=W.QVBoxLayout(self)
        title=W.QLabel('路线设计与惯性导航'); title.setStyleSheet('font-size:22px;font-weight:600'); layout.addWidget(title)
        note=W.QLabel('单位：米。原点为发车位置，+Y 为发车时车头方向，+X 向右。青色为编辑路线，黄色为车内路线，绿色为车端里程计轨迹。')
        note.setWordWrap(True); layout.addWidget(note)
        row=W.QHBoxLayout(); layout.addLayout(row)
        self.length=W.QDoubleSpinBox(); self.length.setRange(.3,100); self.length.setValue(3); self.length.setSuffix(' m')
        self.spacing=W.QDoubleSpinBox(); self.spacing.setRange(.05,.5); self.spacing.setSingleStep(.05); self.spacing.setValue(.2); self.spacing.setSuffix(' m')
        self.speed=W.QDoubleSpinBox(); self.speed.setRange(.05,1.5); self.speed.setSingleStep(.05); self.speed.setValue(.2); self.speed.setSuffix(' m/s')
        for label,widget in [('直线长度',self.length),('采样间距',self.spacing),('执行速度上限',self.speed)]: row.addWidget(W.QLabel(label)); row.addWidget(widget)
        button=W.QPushButton('生成直线路线'); button.clicked.connect(self.straight); row.addWidget(button)
        split=W.QSplitter(); layout.addWidget(split,1)
        self.plot=pg.PlotWidget(); self.plot.setMinimumSize(480,340); self.plot.setAspectLocked(True)
        self.plot.setLabel('bottom','X / 右向','m'); self.plot.setLabel('left','Y / 前向','m'); self.plot.showGrid(x=True,y=True,alpha=.25)
        self.edit_curve=self.plot.plot(pen=pg.mkPen('#39d6ef',width=2),symbol='o',symbolSize=5)
        self.car_curve=self.plot.plot(pen=pg.mkPen('#ffcc55',width=3))
        self.trace_curve=self.plot.plot(pen=pg.mkPen('#6de6aa',width=2))
        self.pose=self.plot.plot(pen=None,symbol='o',symbolSize=11,symbolBrush='#ff716f')
        self.heading=self.plot.plot(pen=pg.mkPen('#ff716f',width=3))
        self.plot.scene().sigMouseClicked.connect(self.click_map); split.addWidget(self.plot)
        editor=W.QWidget(); side=W.QVBoxLayout(editor); split.addWidget(editor)
        self.add_mode=W.QCheckBox('地图点击添加航点（起点固定）'); side.addWidget(self.add_mode)
        self.table=W.QTableWidget(0,2); self.table.setHorizontalHeaderLabels(['X / m','Y / m'])
        self.table.horizontalHeader().setSectionResizeMode(W.QHeaderView.Stretch); side.addWidget(self.table)
        for name,fn in [('添加航点',self.add_point),('删除选中航点',self.delete_point),('按航点生成路线',self.generate),('撤销末尾航点',self.undo),('新建路线',self.clear)]:
            b=W.QPushButton(name); b.clicked.connect(fn); side.addWidget(b)
        local=W.QHBoxLayout(); layout.addLayout(local)
        for name,fn in [('打开路线 JSON',self.open_file),('保存路线 JSON',self.save_file),('车内路线复制到编辑区',self.copy_car),('地图适配',lambda:self.plot.autoRange())]:
            b=W.QPushButton(name); b.clicked.connect(fn); local.addWidget(b)
        transfer=W.QHBoxLayout(); layout.addLayout(transfer); self.device_buttons=[]
        for name,op in [('上传到车内 RAM','upload'),('下载车内路线','download'),('保存车内路线到 Flash','save'),('从 Flash 载入车内','load')]:
            b=W.QPushButton(name); b.clicked.connect(lambda checked=False,o=op:self.device_action(o)); transfer.addWidget(b); self.device_buttons.append(b)
        self.cancel=W.QPushButton('取消传输'); self.cancel.clicked.connect(lambda:self.engine.submit('path','cancel')); transfer.addWidget(self.cancel)
        run=W.QHBoxLayout(); layout.addLayout(run)
        self.start=W.QPushButton('执行已核对的车内路线'); self.start.clicked.connect(self.start_route); run.addWidget(self.start)
        stop=W.QPushButton('■ 立即停车'); stop.setObjectName('emergency'); stop.clicked.connect(engine.emergency_stop); run.addWidget(stop)
        self.status=W.QLabel(); self.status.setWordWrap(True); layout.addWidget(self.status)
        self.details=W.QLabel(); self.details.setWordWrap(True); layout.addWidget(self.details)
        self.local_status=W.QLabel(); self.local_status.setWordWrap(True); layout.addWidget(self.local_status)
        self.set_table(self.points); self.generate()
        self.timer=QtCore.QTimer(self); self.timer.setInterval(200); self.timer.timeout.connect(self.update_data); self.timer.start()

    def set_table(self,points):
        self.table.setRowCount(len(points))
        for i,p in enumerate(points):
            for j,v in enumerate(p):
                item=W.QTableWidgetItem(f'{v:.3f}')
                if i==0: item.setFlags(item.flags() & ~QtCore.Qt.ItemIsEditable)
                self.table.setItem(i,j,item)

    def waypoints(self):
        return [(float(self.table.item(i,0).text()),float(self.table.item(i,1).text())) for i in range(self.table.rowCount())]

    def add_point(self,checked=False,point=None):
        i=self.table.rowCount()
        if i>=256: return
        if point is None:
            try: x,y=self.waypoints()[-1]; point=(x,y+1)
            except (ValueError,IndexError): point=(0,0)
        self.table.insertRow(i)
        for j,v in enumerate(point): self.table.setItem(i,j,W.QTableWidgetItem(f'{v:.3f}'))
        self.local_status.setText('航点已修改；点击“按航点生成路线”更新预览与上传内容')

    def click_map(self,event):
        if self.add_mode.isChecked() and event.button()==QtCore.Qt.LeftButton and self.plot.sceneBoundingRect().contains(event.scenePos()):
            p=self.plot.plotItem.vb.mapSceneToView(event.scenePos()); self.add_point(point=(p.x(),p.y()))

    def delete_point(self):
        for row in sorted({item.row() for item in self.table.selectedItems()},reverse=True):
            if row: self.table.removeRow(row)
        self.local_status.setText('航点已修改，请重新生成路线')

    def undo(self):
        if self.table.rowCount()>1: self.table.removeRow(self.table.rowCount()-1)

    def clear(self):
        self.set_table([(0,0)]); self.points=(); self.edit_curve.setData([],[]); self.local_status.setText('添加航点后生成路线')

    def straight(self):
        self.set_table([(0,0),(0,self.length.value())]); self.generate(); self.plot.autoRange()

    def generate(self):
        try:
            points=resample(self.waypoints(),self.spacing.value())
            self.points=points
            self.edit_curve.setData([p[0] for p in points],[p[1] for p in points])
            total=sum(math.dist(a,b) for a,b in zip(points,points[1:]))
            self.local_status.setText(f'编辑路线：{len(points)} 点 / {total:.3f} m。折线拐角会触发减速；需要圆滑转弯时添加沿圆弧分布的航点。')
            return True
        except (ValueError,AttributeError) as exc:
            self.local_status.setText(str(exc)); return False

    def open_file(self):
        name,_=W.QFileDialog.getOpenFileName(self,'打开路线','','JSON (*.json)')
        if not name:return
        try:
            if Path(name).stat().st_size>100000: raise ValueError('路线文件过大')
            data=read_route(json.loads(Path(name).read_text(encoding='utf-8')))
            self.set_table(data['points']); self.speed.setValue(data['speed_mps']); self.generate(); self.plot.autoRange()
        except (OSError,ValueError,TypeError) as exc:self.local_status.setText(str(exc))

    def save_file(self):
        if not self.generate():return
        name,_=W.QFileDialog.getSaveFileName(self,'保存路线','route.json','JSON (*.json)')
        if name:
            try: Path(name).write_text(json.dumps(document(self.points,self.speed.value()),ensure_ascii=False,indent=2),encoding='utf-8')
            except (OSError,ValueError) as exc:self.local_status.setText(str(exc))

    def copy_car(self):
        if self.engine.routes.car_route:
            self.set_table(self.engine.routes.car_route); self.generate(); self.plot.autoRange()

    def device_action(self,op):
        if op=='upload':
            if self.generate():self.engine.submit('path',op,self.points)
        elif op=='load':
            if W.QMessageBox.question(self,'载入已保存路线','用 Flash 路线替换车内 RAM 路线？未保存的车内修改将丢失。')==W.QMessageBox.Yes:self.engine.submit('path',op)
        else:self.engine.submit('path',op)

    def start_route(self):
        r=self.engine.routes
        if not r.nav:return
        revision=r.car_revision; speed=self.speed.value()
        text=f'执行黄色车内路线：{int(r.nav[3])} 点，{r.nav[9]:.2f} m，上限 {speed:.2f} m/s。\n将车放在路线起点，并朝向图中 +Y。到达终点后保持平衡。\n确认现场空闲并开始行驶？'
        if W.QMessageBox.question(self,'启动惯性导航',text,W.QMessageBox.Yes|W.QMessageBox.No,W.QMessageBox.No)==W.QMessageBox.Yes:
            self.engine.submit('path','start',speed,revision)

    def update_data(self):
        r=self.engine.routes
        with self.engine.lock:
            nav=r.nav; trace=list(r.trace); car=r.car_route
        self.car_curve.setData([p[0] for p in car],[p[1] for p in car])
        self.trace_curve.setData([p[0] for p in trace],[p[1] for p in trace])
        ready=r.available() and self.engine.stopped() and not r.busy and bool(nav) and nav[1] not in (1,3,4) and not nav[13]
        for b in self.device_buttons:b.setEnabled(ready)
        self.cancel.setEnabled(r.busy)
        self.start.setEnabled(ready and bool(car) and r.car_revision==int(nav[14]) and nav[12] and nav[9]>=.3)
        self.status.setText(('Mock 模拟 · ' if self.engine.mode=='mock' else '')+r.message+('' if r.available() else ' · 离线 / 状态过期 / 只读'))
        if nav:
            states=['空闲','录制中','就绪','导航中','终点制动','完成 / 保持平衡','故障']
            reasons=['正常','容量已满','倒退过多','输入异常','偏离路线','停滞或制动超时','已取消']
            self.details.setText(f'{states[int(nav[1])]} · {reasons[int(nav[2])]} · {nav[8]:.2f} / {nav[9]:.2f} m · 目标 {nav[10]:.2f} m/s · {int(nav[3])} 点 · '+('RAM 未保存' if nav[11] else 'RAM 已保存/载入'))
            x,y,yaw=nav[5:8]; angle=math.radians(yaw)
            self.pose.setData([x],[y]); self.heading.setData([x,x+.25*math.sin(angle)],[y,y+.25*math.cos(angle)])
