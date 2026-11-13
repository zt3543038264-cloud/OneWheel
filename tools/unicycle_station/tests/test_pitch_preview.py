import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
from pathlib import Path
import time
import pytest
from app.core.models import DeviceProfile, Parameter
from app.services.engine import StationEngine
from app.services.pitch_preview import PitchPreview, parse_status
from app.protocols.firewater import FireWaterParser


class Clock:
    def __init__(self): self.value=100.
    def __call__(self): return self.value


def engine():
    profile=DeviceProfile.load(Path(__file__).parents[1]/"app/profiles/tc264_unicycle/profile.json")
    e=StationEngine(profile);e.auto_record=False;e.mode="wifi"
    e.connected=e.protocol_ready=True;e.status=[0]*15;e.status_time=time.monotonic()
    clock=Clock();e.pitch_preview=PitchPreview(e,clock)
    sent=[];e.requests.send=sent.append
    return e,clock,sent


def reply(e,clock,payload):
    e.requests.tick(now=clock())
    seq=e.requests.pending.seq
    e.ingest(f"rsp:{seq},ok,{payload}\n".encode())


IDLE="preview_status,1,0,1,0,17,0,1,0,0,0"
ACTIVE="preview_status,1,1,1,0,0,100,1.2,-3,60,0"


def activate(e,clock):
    e._action("pitch_preview",("start","rate"))
    reply(e,clock,IDLE)
    reply(e,clock,"preview_start,1,ZERO_OUTPUT")
    e.pitch_preview.tick()
    reply(e,clock,ACTIVE)


def test_preview_encoder_restricts_axis_and_ring():
    parser=FireWaterParser()
    assert parser.encode_command("preview_start",8,"pitch","rate")==b"cfg:preview_start,8,pitch,rate\r\n"
    assert parser.encode_command("preview_status",9)==b"cfg:preview_status,9\r\n"
    for axis,ring in (("roll","rate"),("pitch","vel"),("yaw","angle")):
        with pytest.raises(ValueError):parser.encode_command("preview_start",8,axis,ring)
    for op in ("cbench_start","bench_start","start"):
        with pytest.raises(ValueError):parser.encode_command(op,1)


@pytest.mark.parametrize("payload",[
    "preview_status,2,1,1,0,0,1,1,2,3,0",
    "preview_status,1,2,1,0,0,1,1,2,3,0",
    "preview_status,1,1,1,0,0,10001,1,2,3,0",
    "preview_status,1,1,1,0,0,1,nan,2,3,0",
    "preview_status,1,1,1,0,0,1,1,2,3,2",
    "preview_status,1,1,1,0,0,1,1,2,3",
])
def test_preview_status_rejects_bad_fields(payload):
    with pytest.raises(ValueError):parse_status(payload.split(','))


def test_uses_actual_preview_commands_and_no_keep():
    e,clock,sent=engine();activate(e,clock)
    assert e.pitch_preview.phase=="active"
    assert all(b"cbench" not in wire and b"keep" not in wire for wire in sent)
    assert sum(b"preview_start" in wire for wire in sent)==1
    latest,_,_=e.pitch_preview.store.snapshot()
    assert latest=={"preview_pitch":1.2,"preview_pitch_rate":-3.,"preview_output":60.}


def test_only_selected_pid_parameters_are_editable():
    e,clock,_=engine();activate(e,clock)
    for name in ("p_rate_kp","p_rate_ki","p_rate_kd"):
        assert e.permission(Parameter(name,value=1))[0]
    for name in ("p_angle_kp","p_vel_kp","r_rate_kp","pitch_zero_init"):
        assert not e.permission(Parameter(name,value=1),advanced=True)[0]
    e.pitch_preview.latest["ring"]=1
    assert e.permission(Parameter("p_angle_kp",value=1))[0]
    assert not e.permission(Parameter("p_vel_kp",value=1))[0]


def test_parameter_apply_uses_existing_set_and_actual_readback():
    e,clock,sent=engine();activate(e,clock)
    parameter=Parameter("p_rate_kp",group="Pitch",value=20,min=-500,max=500)
    e.params={parameter.name:parameter}
    e._apply([("p_rate_kp",21)])
    reply(e,clock,"set,p_rate_kp,21,APPLIED")
    assert parameter.value==21 and parameter.ram_dirty
    assert any(b"cfg:set," in wire and b",p_rate_kp,21" in wire for wire in sent)
    assert not any(b"cbench" in wire for wire in sent)


def test_stop_invalidates_late_preflight_callback():
    e,clock,sent=engine();e._action("pitch_preview",("start","rate"))
    e.requests.tick(now=clock());callback=e.requests.pending.callback
    e.pitch_preview.cancel_for_stop();e.requests.cancel("STOP")
    callback(True,IDLE.split(','))
    assert not e.requests.queue
    assert not any(b"preview_start" in wire for wire in sent)


def test_disconnect_invalidates_late_start_callback():
    e,clock,sent=engine();e._action("pitch_preview",("start","rate"));reply(e,clock,IDLE)
    e.requests.tick(now=clock());callback=e.requests.pending.callback
    e.pitch_preview.reset();e.requests.cancel("DISCONNECTED");callback(True,["preview_start","1","ZERO_OUTPUT"])
    assert e.pitch_preview.phase=="idle" and not e.pitch_preview.latest


def test_ambiguous_start_requests_existing_priority_stop():
    e,clock,_=engine();e._action("pitch_preview",("start","rate"));reply(e,clock,IDLE)
    reply(e,clock,"preview_start,1,UNKNOWN_OUTPUT")
    assert e.stop_requested.is_set()
    assert e.pitch_preview.phase=="stopping"


def test_stale_feedback_stops_and_does_not_fake_freshness():
    e,clock,_=engine();activate(e,clock)
    last=e.pitch_preview.received;clock.value+=.8;e.pitch_preview.tick()
    assert e.stop_requested.is_set() and e.pitch_preview.received==last
    assert e.pitch_preview.phase=="stopping"


def test_preflight_read_timeout_never_sends_start_or_stop():
    e,clock,sent=engine();e._action("pitch_preview",("start","rate"))
    e.requests.tick(now=clock());clock.value+=.8;e.pitch_preview.tick()
    assert e.pitch_preview.phase=="idle" and not e.stop_requested.is_set()
    assert len(sent)==1 and b"preview_status" in sent[0]


def test_existing_active_preview_does_not_start_again():
    e,clock,sent=engine();e._action("pitch_preview",("start","rate"));reply(e,clock,ACTIVE)
    assert e.pitch_preview.phase=="active" and not e.requests.queue
    assert not any(b"preview_start" in wire for wire in sent)


def test_other_actions_are_locked_while_preview_active():
    e,clock,_=engine();activate(e,clock)
    e._action("save",("Pitch",))
    assert not e.requests.queue
    assert any("请先停止 Pitch" in line for line in e.diagnostics)


def test_mock_path_end_to_end_without_hardware():
    from app.transports.mock import MockTransport
    e,_,_=engine();e.mode="mock";transport=MockTransport(e.profile);transport.open()
    e.transport=transport;e.requests.send=transport.write;e._handshake()
    try:
        for _ in range(15):
            e.requests.tick();e.ingest(transport.read())
        assert e.protocol_ready and e.params
        e._action("pitch_preview",("start","rate"))
        for _ in range(15):
            e.pitch_preview.tick();e.requests.tick();e.ingest(transport.read())
        assert e.pitch_preview.latest and e.pitch_preview.latest["active"]
        e.pitch_preview.cancel_for_stop();e.requests.cancel("STOP");transport.emergency_stop()
        e.pitch_preview.tick();e.requests.tick();e.ingest(transport.read())
        assert e.pitch_preview.phase=="idle"
    finally:transport.close()


def test_existing_pitch_workbench_controls_and_safe_stop():
    from PySide6 import QtWidgets as W
    from app.ui.window import MainWindow
    app=W.QApplication.instance() or W.QApplication([])
    window=MainWindow(restore=False);window.persist_enabled=False;window.engine.auto_record=False
    try:
        panel=window.pitch_preview_panel
        assert panel and panel.plots.ring.currentData()=="p_rate_"
        assert not panel.start.isEnabled()
        e=window.engine;e.connected=e.protocol_ready=True;e.status=[0]*15;e.status_time=time.monotonic()
        calls=[];e.submit=lambda *args:calls.append(args)
        panel.update_data();assert panel.start.isEnabled()
        panel.start.click();assert calls[-1]==("pitch_preview","start","rate")
        panel.plots.ring.setCurrentIndex(panel.plots.ring.findData("p_vel_"));panel.update_data()
        assert not panel.start.isEnabled()
        e.pitch_preview.phase="active"
        e.pitch_preview.latest={"axis":1,"ring":0}
        window.params.update_data()
        assert not window.params.save_all.isEnabled()
        assert not window.params.save_group.isEnabled()
        e.emergency_stop=lambda:calls.append(("priority_stop",))
        panel.stop.click();assert calls[-1]==("priority_stop",)
        assert "未施加" in panel.graphs["output"].title
    finally:window.engine.close();window.deleteLater();app.processEvents()
