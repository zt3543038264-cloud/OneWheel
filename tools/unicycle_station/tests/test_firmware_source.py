from pathlib import Path
import re
import shutil
import subprocess
import pytest

@pytest.mark.parametrize("wifi", [False, True])
def test_mcu_protocol_host_contract(tmp_path, wifi):
    root=Path(__file__).parents[1]
    gcc=shutil.which("gcc") or "D:/Tools/Dev-Cpp/MinGW64/bin/gcc.exe"
    if not Path(gcc).exists():pytest.skip("Optional GCC host compiler not installed")
    exe=tmp_path/"firmware_contract.exe"
    extra=["-DSTATION_WIFI_ENABLED=0"]
    if wifi:
        (tmp_path/"Protocols").mkdir()
        (tmp_path/"config_private.h").write_text('#define STATION_WIFI_SSID "host-test"\n#define STATION_WIFI_PASSWORD "host-only"\n#define STATION_WIFI_PC_IP "127.0.0.1"\n')
        stub=tmp_path/"wifi_stub.c"
        stub.write_text('#include "Protocols/station_wifi.h"\nuint8 station_wifi_init(void){return 1;}\nint station_wifi_step(const uint8 *tx,uint16 n,uint8 *rx,uint16 *got){(void)tx;(void)n;(void)rx;*got=0;return 0;}\n')
        extra=["-DSTATION_WIFI_ENABLED=1","-I",str(tmp_path),str(stub)]
    result=subprocess.run([gcc,"-std=c99","-Wall","-Wextra","-I",str(root/"tests/firmware_host"),"-I",str(root.parents[1]/"code"),*extra,str(root/"tests/firmware_host/harness.c"),"-o",str(exe)],capture_output=True,text=True)
    assert result.returncode==0,result.stderr
    result=subprocess.run([str(exe)],capture_output=True,text=True)
    assert result.returncode==0,result.stdout+result.stderr

def test_isr_bounded_no_configuration_work():
    root=Path(__file__).parents[3]
    text=(root/"code/Protocols/vofa.c").read_text(encoding="utf-8")
    tick=text.split("void vofa_tick1ms(void)")[1].split("void vofa_cmd_poll(void)")[0]
    assert "i < VOFA_TX_FIFO_DEPTH" in tick
    assert "cfg_execute" not in tick and "snprintf" not in tick and "param_save(" not in tick
    control=(root/"code/Controller/control.c").read_text(encoding="utf-8")
    loop=control.split("void control_loop(void)")[1].split("uint8 control_camera_debug_start")[0]
    assert "cfg_" not in loop
