import socket
import threading
import time
from dataclasses import asdict

import pytest
from app.core.models import ConnectionConfig
from app.transports.wifi_transport import WifiTransport
from app.transports.mock import MockTransport
from app.services.engine import StationEngine
from app.ui.window import builtin_profile


def local_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def wait_for(condition, timeout=6):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if condition():
            return
        time.sleep(.01)
    raise AssertionError("WiFi condition timed out")


@pytest.mark.parametrize("kwargs", [
    {"wifi_host":"invalid"}, {"wifi_host":"0.0.0.0"},
    {"wifi_host":"239.1.2.3"}, {"wifi_host":"255.255.255.255"},
    {"wifi_port":0}, {"wifi_local_port":65536}, {"ending":"\r"},
])
def test_wifi_config_rejects_invalid_endpoint(kwargs):
    with pytest.raises(ValueError):
        ConnectionConfig(transport="wifi", **kwargs).validate()


def test_udp_bound_peer_whole_frames_and_stop():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as car:
        car.bind(("127.0.0.1",0));car.settimeout(.2)
        config=ConnectionConfig(transport="wifi",wifi_host="127.0.0.1",wifi_port=car.getsockname()[1],wifi_bind="127.0.0.1",wifi_local_port=local_port())
        transport=WifiTransport(config);transport.open()
        try:
            transport.write(b"cfg:hello,1\r\n")
            assert car.recvfrom(512)[0]==b"cfg:hello,1\r\n"
            addr=(config.wifi_bind,config.wifi_local_port)
            with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as wrong:
                wrong.sendto(b"att:1,2,3\n",addr)
                assert transport.read()==b""
            car.sendto(b"att:1,2",addr);assert transport.read()==b""
            car.sendto(b"x"*3000+b"\n",addr);assert transport.read()==b""
            car.sendto(b"att:1,2,3\n",addr);assert transport.read()==b"att:1,2,3\n"
            assert transport.dropped_datagrams==2
            with pytest.raises(ValueError):transport.write(b"cfg:set,2,p_vel_kp,0.1")
            transport.emergency_stop()
            assert [car.recvfrom(512)[0] for _ in range(3)]==[b"\r\nstop\r\n"]*3
        finally:
            transport.close()
        assert not transport.status()["connected"]


def test_real_udp_engine_handshake_live_apply_stale_lock_and_stop():
    profile=builtin_profile();mock=MockTransport(profile)
    quit_event=threading.Event();muted=threading.Event();errors=[]
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as car:
        car.bind(("127.0.0.1",0));car.settimeout(.002)
        config=ConnectionConfig(transport="wifi",wifi_host="127.0.0.1",wifi_port=car.getsockname()[1],wifi_bind="127.0.0.1",wifi_local_port=local_port())
        mock.open()
        def serve():
            pending=b""
            try:
                while not quit_event.is_set():
                    try:
                        data,_=car.recvfrom(4096);mock.write(data)
                    except (socket.timeout, ConnectionResetError):pass
                    pending+=mock.read()
                    while b"\n" in pending:
                        line,pending=pending.split(b"\n",1)
                        if not muted.is_set():car.sendto(line+b"\n",(config.wifi_bind,config.wifi_local_port))
            except Exception as exc:errors.append(exc)
        thread=threading.Thread(target=serve);thread.start()
        engine=StationEngine(profile);engine.auto_record=False
        try:
            engine.start("wifi",config)
            wait_for(lambda:engine.protocol_ready and len(engine.params)==len(profile.data["mock_parameters"]))
            assert engine.protocol_ready and engine.mode=="wifi"
            with mock.lock:mock.mode="Balance";mock.manual_stopped=True
            wait_for(lambda:engine.status and engine.status[1]==2 and not engine.status[2])
            assert engine.permission(engine.params["lean_speed_kp"])[0]
            engine.submit("apply",[("lean_speed_kp",.8)],False)
            wait_for(lambda:engine.batch_results)
            assert engine.batch_results[-1][1] and engine.params["lean_speed_kp"].value==.8
            assert not mock.saved
            muted.set();time.sleep(.85)
            assert not engine.permission(engine.params["lean_speed_kp"])[0]
            assert "数据中断" in engine.link_status()
            muted.clear();wait_for(lambda:engine.permission(engine.params["lean_speed_kp"])[0])
            engine.emergency_stop()
            wait_for(lambda:"已确认车辆 STOP" in engine.stop_message)
            engine.close();engine.start("wifi",config)
            wait_for(lambda:engine.protocol_ready and len(engine.params)==len(profile.data["mock_parameters"]))
            assert not errors
        finally:
            engine.close();quit_event.set();thread.join(timeout=2);mock.close()


def test_connection_dialog_mode_roundtrip_and_visible_error(monkeypatch):
    from PySide6 import QtWidgets as W
    from app.ui.connection import ConnectionDialog
    from app.transports.serial_transport import SerialTransport
    monkeypatch.setattr(SerialTransport,"available_devices",staticmethod(lambda:[]))
    app=W.QApplication.instance() or W.QApplication([])
    original=ConnectionConfig(transport="wifi",wifi_host="192.168.2.33",wifi_local_port=1349)
    dialog=ConnectionDialog(original);dialog.show();app.processEvents()
    try:
        assert asdict(dialog.config())==asdict(original)
        assert not dialog.serial_port_row.isVisible() and dialog.wifi_note.isVisible()
        dialog.fields["wifi_host"].setText("bad")
        dialog.validate_accept();assert dialog.info.isVisible() and "IPv4" in dialog.info.text()
        dialog.mode.setCurrentIndex(0);app.processEvents()
        assert dialog.serial_port_row.isVisible() and not dialog.wifi_note.isVisible()
        assert dialog.config().transport=="serial"
    finally:
        dialog.close();dialog.deleteLater();app.processEvents()
