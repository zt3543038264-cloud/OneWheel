"""Real controller/PID host integration. No simulated vehicle or hardware claim."""
from pathlib import Path
import subprocess
import pytest

ROOT = Path(__file__).resolve().parents[1]

@pytest.mark.parametrize("unlock_dormant_jog", [False, True])
def test_real_controller_preview_and_jog(tmp_path, unlock_dormant_jog):
    exe = tmp_path / "control_host"
    flags = ["-DHOST_UNLOCK_JOG_TEST=1"] if unlock_dormant_jog else []
    result = subprocess.run([
        "gcc", "-std=c99", "-O2", "-ffunction-sections", "-fdata-sections", *flags,
        "-I"+str(ROOT/"tests/commissioning"), "-I"+str(ROOT/"code"),
        str(ROOT/"tests/commissioning/control_host.c"), "-Wl,--gc-sections", "-lm", "-o", str(exe),
    ], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    result = subprocess.run([str(exe)], capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "PASS" in result.stdout
