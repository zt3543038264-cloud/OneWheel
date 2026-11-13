"""Offline interface contracts; also executable without third-party packages."""
import json
from pathlib import Path
ROOT = Path(__file__).resolve().parents[3]
PROFILE = ROOT / 'tools/unicycle_station/app/profiles/tc264_unicycle'

def test_axis_labels_preserve_wire_fields():
    data = json.loads((PROFILE / 'profile.json').read_text())
    run = data['channels']['run']
    assert len(run) == 25
    assert run[13]['name'] == 'flywheel_common_rpm'
    assert run[13]['label'] == 'A Left/Yaw 转速'
    assert run[4]['label'] == 'B Right/Roll 回收反馈'
    fly = data['channels']['fly']
    assert fly[1]['name'] == 'flywheel_a_rpm' and 'Left/Yaw' in fly[1]['label']
    assert fly[2]['name'] == 'flywheel_b_rpm' and 'Right/Roll' in fly[2]['label']
    assert '断开电机动力' in data['safety_rules']['warning']
    assert 'B信号' in data['safety_rules']['warning']
    assert '控制时序/方向待验证' in data['safety_rules']['warning']
    assert '500Hz EKF' in data['safety_rules']['warning']

def test_locked_status_is_explained():
    data = json.loads((PROFILE / 'display.json').read_text())
    assert '锁定' in data['value_labels']['test_status']['14']
    assert '禁用' in data['value_labels']['test_status']['15']
    source = (ROOT / 'code/Protocols/vofa.c').read_text()
    assert '#if !VEHICLE_MOTOR_OUTPUT_ALLOWED\n    if (!control_bench_b_running() && !control_c_bench_running()) v[10] = (uint32)CTRL_TEST_STATUS_ADAPT_LOCKED;' in source
    assert 'v[4] = control_c_bench_running() ? (uint32)MOTOR_JOG_C' in source
    assert '(control_bench_b_running() ? (uint32)MOTOR_JOG_B' in source
    menu = (ROOT / 'code/Menu/menu.c').read_text()
    assert 'case CTRL_TEST_STATUS_ADAPT_LOCKED:' in menu
    assert 'case CTRL_TEST_STATUS_YAW_DISABLED:' in menu
    assert 'CONTROL UNVALIDATED' in menu

def test_storage_failures_are_specific():
    menu = (ROOT / 'code/Menu/menu.c').read_text()
    for state, label in [('DISABLED', 'FLASH DISABLED'),
                         ('INCOMPATIBLE', 'FLASH INCOMPATIBLE'),
                         ('CORRUPT', 'FLASH CORRUPT'),
                         ('IO', 'FLASH I/O FAILED')]:
        assert 'case PARAM_STORAGE_' + state + ':' in menu
        assert label in menu
    assert '"SAVE FAILED"' not in menu
    assert '"ERASE FAILED"' not in menu
    assert 'param_erase() ? "ERASED, DEFAULTS" : storage_failure_text()' in menu
    assert 'param_storage_status() == PARAM_STORAGE_OK ?' in menu


def test_fixed_frame_writes_cannot_claim_direction_change():
    source = (ROOT / 'code/Protocols/vofa.c').read_text()
    branch = source[source.index('/* new RA EKF uses a fixed body frame.'):]
    branch = branch[:branch.index('if (s_save_seq)')]
    assert 'if (value != actual)' in branch
    assert 'FIXED_FRAME,RA EKF body frame is compile-time fixed' in branch
    assert '"set,%s,%.9g,FIXED_FRAME"' in branch
    assert 'g_param_revision++' not in branch
    assert 'param_set_imu_forward_dir(' not in source
    menu = (ROOT / 'code/Menu/menu.c').read_text()
    assert 'param_set_imu_forward_dir(' not in menu
    assert 'menu_status("FIXED RA FRAME")' in menu
    data = json.loads((PROFILE / 'profile.json').read_text())
    assert '固定' in data['parameters']['imu_forward_dir']['label']
    assert 'FIXED_FRAME' in data['parameters']['imu_forward_dir']['description']


if __name__ == "__main__":
    test_axis_labels_preserve_wire_fields()
    test_locked_status_is_explained()
    test_storage_failures_are_specific()
    test_fixed_frame_writes_cannot_claim_direction_change()
    print("4 offline interface contract tests passed")
