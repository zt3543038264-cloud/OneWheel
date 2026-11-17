import socket
import pytest
from app.transports.wifi_discovery import WifiDiscovery
from app.core.models import ConnectionConfig
from app.ui.connection import ConnectionDialog
from PySide6 import QtWidgets as W


def available_port():
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as s:
        s.bind(('127.0.0.1',0));return s.getsockname()[1]


def test_passive_discovery_filters_and_multiple_peers():
    discovery=WifiDiscovery();port=available_port()
    discovery.open('127.0.0.1',port)
    try:
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as first, socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as second:
            for data in (b'att:1,2',b'unknown:1,2,3\n',b'att:nan,2,3\n',b'x'*3000+b'\n',b'hello\n'):
                first.sendto(data,('127.0.0.1',port));assert discovery.poll()==[]
            first.sendto(b'att:1,2,3\n',('127.0.0.1',port))
            peers=discovery.poll();assert peers==[('127.0.0.1',first.getsockname()[1])]
            first.sendto(b'att:2,3,4\n',('127.0.0.1',port));assert discovery.poll()==[]
            second.sendto(b'stat:'+b','.join([b'0']*15)+b'\n',('127.0.0.1',port));assert len(discovery.poll())==1
            assert len(discovery.peers)==2
            first.settimeout(.02)
            with pytest.raises(socket.timeout):first.recvfrom(512) # no outbound command
    finally:discovery.close()
    discovery.open('127.0.0.1',port);assert not discovery.peers;discovery.close()


def test_gui_success_multiple_cancel_and_close(monkeypatch):
    app=W.QApplication.instance() or W.QApplication([])
    from app.transports.serial_transport import SerialTransport
    monkeypatch.setattr(SerialTransport,'available_devices',lambda:[])
    config=ConnectionConfig(transport='wifi',wifi_bind='127.0.0.1',wifi_local_port=available_port())
    dialog=ConnectionDialog(config)
    dialog.start_discovery()
    monkeypatch.setattr(dialog.discovery,'poll',lambda:[('127.0.0.2',5001)])
    dialog.discovery_ticks=99;dialog.poll_discovery()
    assert dialog.fields['wifi_host'].text()=='127.0.0.2'
    assert dialog.fields['wifi_port'].value()==5001
    assert dialog.discovery.device is None
    dialog.start_discovery()
    monkeypatch.setattr(dialog.discovery,'poll',lambda:[('127.0.0.3',5002),('127.0.0.4',5003)])
    dialog.discovery_ticks=99;dialog.poll_discovery()
    assert dialog.fields['wifi_host'].text()=='127.0.0.2'
    dialog.discovered.setCurrentIndex(1);dialog.apply_discovered()
    assert dialog.fields['wifi_host'].text()=='127.0.0.4'
    dialog.start_discovery();dialog.start_discovery()
    assert not dialog.discovery_timer.isActive() and dialog.discovery.device is None
    dialog.start_discovery();dialog.reject()
    assert dialog.discovery.device is None
    dialog.start_discovery();dialog.mode.setCurrentIndex(0)
    assert dialog.discovery.device is None
    dialog.close()


def test_gui_timeout_and_busy_port(monkeypatch):
    app=W.QApplication.instance() or W.QApplication([])
    from app.transports.serial_transport import SerialTransport
    monkeypatch.setattr(SerialTransport,'available_devices',lambda:[])
    port=available_port()
    dialog=ConnectionDialog(ConnectionConfig(transport='wifi',wifi_bind='127.0.0.1',wifi_local_port=port))
    original=dialog.fields['wifi_host'].text()
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as s:
        s.bind(('127.0.0.1',port));dialog.start_discovery()
        assert not dialog.discovery_timer.isActive()
        assert '无法监听' in dialog.discover_info.text()
    dialog.start_discovery();dialog.discovery_ticks=99;dialog.poll_discovery()
    assert '未收到' in dialog.discover_info.text()
    assert dialog.fields['wifi_host'].text()==original
    dialog.close()
