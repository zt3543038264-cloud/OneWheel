"""Focused review acceptance. No device connection or flashing.
This does not replace the retained full legacy suite tests/run_all.py.
Install station test dependencies, then run from any directory.
"""
from pathlib import Path
import os,subprocess,sys
ROOT=Path(__file__).resolve().parents[1]
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
for path in ('tests/new_ekf/run.py','tools/tests/test_storage_safety.py',
             'tools/tests/test_flash_wait.py','tools/tests/test_flash_preservation.py','tools/tests/test_path_nav.py'):
    subprocess.run([sys.executable,str(ROOT/path)],cwd=ROOT,check=True)
subprocess.run([sys.executable,'-m','pytest','-q',
 'tests/test_external_driver_safety.py','tests/test_wmotor_safe_bench.py',
 'tests/test_wmotor_driver_integration.py','tests/test_commissioning_control.py','tests/test_commissioning_cli.py',
 'tests/test_pitch_adapter_integration.py','tests/test_orthogonal_review.py','tests/test_qcar_driver.py','tests/test_spi_timeout_port.py',
 'tests/test_linked_menu.py','tools/unicycle_station/tests/test_orthogonal_review_interface.py',
 'tools/unicycle_station/tests/test_build_configuration.py','tools/unicycle_station/tests/test_station_schema_contract.py',
 'tools/unicycle_station/tests/test_path_yaw.py',
 'tools/unicycle_station/tests/test_angle_timing.py','tools/unicycle_station/tests/test_pitch_coordinates.py',
 'tools/unicycle_station/tests/test_firmware_source.py',
 'tools/unicycle_station/tests/test_remote_control.py',
 'tools/unicycle_station/tests/test_imu_forward.py::test_station_fixed_frame_rejects_change_and_noop_preserves_zeros_revision'],cwd=ROOT,check=True)
