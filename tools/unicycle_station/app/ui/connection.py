from dataclasses import asdict
from PySide6 import QtWidgets as W
from app.core.models import ConnectionConfig, read_document, write_document
from app.transports.serial_transport import SerialTransport


class ConnectionDialog(W.QDialog):
    def __init__(self,config,parent=None):
        super().__init__(parent)
        self.setWindowTitle("连接设置");self.resize(650,700)
        outer=W.QVBoxLayout(self);form=W.QFormLayout();outer.addLayout(form)
        self.fields={}
        self.original=config
        self.form=form
        self.mode=W.QComboBox();self.mode.addItem("串口", "serial");self.mode.addItem("WiFi · SPI 模块", "wifi")
        self.mode.setCurrentIndex(1 if config.transport == "wifi" else 0);form.addRow("连接方式",self.mode)
        for name,label in (("wifi_host","车辆 WiFi 地址"),("wifi_port","车辆端口"),("wifi_bind","本机接收地址"),("wifi_local_port","本机接收端口")):
            if name.endswith("port"):
                field=W.QSpinBox();field.setRange(1024,65535);field.setValue(getattr(config,name))
            else:
                field=W.QLineEdit(getattr(config,name))
            self.fields[name]=field;form.addRow(label,field)
        self.wifi_note=W.QLabel("电脑与车辆连接同一热点。车辆端口默认 6666，本机接收端口默认 1349。\n车辆固件中的目标 IP 应填写电脑在该热点下的 IPv4 地址；本机接收地址通常保留 0.0.0.0。")
        self.wifi_note.setWordWrap(True);form.addRow(self.wifi_note)
        self.port=W.QComboBox();self.port.setEditable(True)
        self.port.addItem(config.port);self.fields["port"]=self.port
        row=W.QHBoxLayout();row.addWidget(self.port)
        refresh=W.QPushButton("刷新端口");refresh.clicked.connect(self.refresh);row.addWidget(refresh);self.serial_port_row=W.QWidget();self.serial_port_row.setLayout(row);form.addRow("端口",self.serial_port_row)
        combos={"baudrate":["9600","57600","115200","230400","460800","921600"],"bytesize":["5","6","7","8"],"parity":["N","E","O","M","S"],"stopbits":["1","1.5","2"],"flow_control":["none","xonxoff","rtscts","dsrdtr"],"encoding":["ascii","utf-8","gb18030"]}
        labels={"baudrate":"波特率（可输入）","bytesize":"数据位","parity":"校验位","stopbits":"停止位","flow_control":"PC 侧流控","encoding":"文本编码","ending":"行结束符（使用 \\r / \\n）","timeout":"读取超时 / 秒","write_timeout":"写入超时 / 秒","reconnect_interval":"重连间隔 / 秒","dtr":"DTR","rts":"RTS","auto_reconnect":"自动重连","connect_last":"启动时连接上次设备"}
        for name,choices in combos.items():
            field=W.QComboBox();field.setEditable(name in ("baudrate","encoding"));field.addItems(choices);field.setCurrentText(str(getattr(config,name)));self.fields[name]=field;form.addRow(labels[name],field)
        ending=W.QComboBox();ending.setEditable(True);ending.addItems([r"\r\n",r"\n",r"\r"]);ending.setCurrentText(config.ending.replace("\r",r"\r").replace("\n",r"\n"));self.fields["ending"]=ending;form.addRow(labels["ending"],ending)
        for name in ("timeout","write_timeout","reconnect_interval"):
            field=W.QDoubleSpinBox();field.setDecimals(3);field.setRange(.001,60 if name=="reconnect_interval" else .1);field.setValue(getattr(config,name));self.fields[name]=field;form.addRow(labels[name],field)
        for name in ("dtr","rts","auto_reconnect","connect_last"):
            field=W.QCheckBox();field.setChecked(getattr(config,name));self.fields[name]=field;form.addRow(labels[name],field)
        self.info=W.QLabel();self.info.setWordWrap(True);outer.addWidget(self.info)
        row=W.QHBoxLayout()
        for text,slot in [("加载预设",self.load),("保存预设",self.save)]:
            b=W.QPushButton(text);b.clicked.connect(slot);row.addWidget(b)
        outer.addLayout(row)
        buttons=W.QDialogButtonBox(W.QDialogButtonBox.Ok|W.QDialogButtonBox.Cancel);buttons.accepted.connect(self.validate_accept);buttons.rejected.connect(self.reject);outer.addWidget(buttons)
        self.mode.currentIndexChanged.connect(self.update_mode)
        self.refresh()
        self.update_mode()

    def update_mode(self):
        wifi=self.mode.currentData()=="wifi"
        for name in ("wifi_host","wifi_port","wifi_bind","wifi_local_port"):
            self.form.setRowVisible(self.fields[name],wifi)
        self.form.setRowVisible(self.wifi_note,wifi)
        self.form.setRowVisible(self.serial_port_row,not wifi)
        for name in ("baudrate","bytesize","parity","stopbits","flow_control","dtr","rts"):
            self.form.setRowVisible(self.fields[name],not wifi)
        self.info.setVisible(not wifi)

    def refresh(self):
        current=self.port.currentText().split(" · ",1)[0];self.port.clear();lines=[]
        for p in SerialTransport.available_devices():
            self.port.addItem(f"{p['port']} · {p['description']}",p["port"]);lines.append(f"{p['port']} · {p['description']} · VID {p['vid']} PID {p['pid']} · SN {p['serial_number']}")
        index=self.port.findData(current)
        if index>=0:self.port.setCurrentIndex(index)
        else:self.port.setCurrentText(current)
        self.info.setText("\n".join(lines) or "未发现串口，可手动输入 COM 端口")

    def config(self):
        values={}
        for name,field in self.fields.items():
            values[name]=field.isChecked() if isinstance(field,W.QCheckBox) else field.value() if isinstance(field,(W.QSpinBox,W.QDoubleSpinBox)) else field.text().strip() if isinstance(field,W.QLineEdit) else field.currentText()
        values["transport"]=self.mode.currentData()
        values["baudrate"]=int(values["baudrate"]);values["bytesize"]=int(values["bytesize"]);values["stopbits"]=float(values["stopbits"])
        values["ending"]=values["ending"].replace(r"\r","\r").replace(r"\n","\n").replace(r"\t","\t")
        values["port"]=values["port"].split(" · ",1)[0].strip()
        for device in SerialTransport.available_devices():
            if device["port"]==values["port"]:
                values.update(device_vid=device.get("vid"),device_pid=device.get("pid"),device_serial=device.get("serial_number") or "",device_location=device.get("location") or "")
        config=ConnectionConfig(**values);config.validate();return config

    def validate_accept(self):
        try:self.config();self.accept()
        except ValueError as exc:self.info.setText(str(exc));self.info.show()

    def save(self):
        try:
            config=self.config();path,_=W.QFileDialog.getSaveFileName(self,"保存连接预设","connection.json","JSON (*.json)")
            if path:write_document(path,"connection_preset",asdict(config))
        except ValueError as exc:self.info.setText(str(exc));self.info.show()

    def load(self):
        path,_=W.QFileDialog.getOpenFileName(self,"加载连接预设","","JSON (*.json)")
        if not path:return
        try:
            data=read_document(path,"connection_preset")
            self.mode.setCurrentIndex(1 if data.get("transport")=="wifi" else 0)
            for name,field in self.fields.items():
                if name not in data:continue
                value=data[name]
                if isinstance(field,W.QCheckBox):field.setChecked(value)
                elif isinstance(field,(W.QSpinBox,W.QDoubleSpinBox)):field.setValue(value)
                elif isinstance(field,W.QLineEdit):field.setText(str(value))
                else:field.setCurrentText(str(value).replace("\r",r"\r").replace("\n",r"\n"))
        except (ValueError,OSError,TypeError) as exc:self.info.setText(str(exc));self.info.show()
