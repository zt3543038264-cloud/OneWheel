"""Compile the public/private configuration contract without vehicle hardware."""
from pathlib import Path
import os
import re
import shutil
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[1]


def preprocess(tmp_path, source, enabled):
    compiler = os.environ.get("ONEWHEEL_HOST_GCC") or shutil.which("gcc")
    if not compiler:
        pytest.skip("Optional GCC preprocessor unavailable")
    path = tmp_path / "config_contract.c"
    path.write_text(source, encoding="utf-8")
    return subprocess.run(
        [compiler, "-E", "-P", f"-DSTATION_WIFI_ENABLED={enabled}",
         "-I" + str(tmp_path), "-I" + str(ROOT / "code"), str(path)],
        capture_output=True, text=True, timeout=15,
    )


def test_wifi_private_file_is_the_only_local_network_source(tmp_path):
    (tmp_path / "config_private.h").write_text(
        '#define STATION_WIFI_SSID "test-network"\n'
        '#define STATION_WIFI_PASSWORD "test-only"\n'
        '#define STATION_WIFI_PC_IP "192.0.2.17"\n', encoding="utf-8",
    )
    result = preprocess(tmp_path, '''#include "config.h"
const char *ssid = STATION_WIFI_SSID;
const char *password = STATION_WIFI_PASSWORD;
const char *host = STATION_WIFI_PC_IP;
const char *host_port = STATION_WIFI_PC_PORT;
const char *local_port = STATION_WIFI_LOCAL_PORT;
''', 1)
    assert result.returncode == 0, result.stderr
    for expected in ['"test-network"', '"test-only"', '"192.0.2.17"', '"1349"', '"6666"']:
        assert expected in result.stdout


def test_uart_build_needs_no_private_file(tmp_path):
    # Isolate the public header from this machine's existing private header.
    shutil.copyfile(ROOT / "code/config.h", tmp_path / "config.h")
    result = preprocess(tmp_path, '#include "config.h"\nint period = CTRL_PERIOD_MS;\n', 0)
    assert result.returncode == 0, result.stderr
    assert "int period = (1);" in result.stdout


def test_incomplete_wifi_settings_fail_at_build_time(tmp_path):
    (tmp_path / "config_private.h").write_text(
        '#define STATION_WIFI_SSID "test-network"\n', encoding="utf-8",
    )
    result = preprocess(tmp_path, '#include "config.h"\n', 1)
    assert result.returncode != 0
    assert "fill local WiFi settings" in result.stderr


def test_no_duplicate_config_pages_or_committed_credentials():
    for old in ["Common/board_config.h", "Common/direction_config.h",
                "Protocols/station_wifi_config.h", "Protocols/station_wifi_private.h"]:
        assert not (ROOT / "code" / old).exists()
    public = (ROOT / "code/config.h").read_text(encoding="utf-8")
    for key in ["STATION_WIFI_SSID", "STATION_WIFI_PASSWORD", "STATION_WIFI_PC_IP"]:
        assert not re.search(r"^\s*#define\s+" + key + r"\b", public, re.M)
    adapter = (ROOT / "user/isr_config.h").read_text(encoding="utf-8")
    assert '#include "config.h"' in adapter
    assert "ISR_PRIORITY" not in adapter
    ignored = subprocess.run(
        ["git", "check-ignore", "code/config_private.h"], cwd=ROOT,
        capture_output=True, text=True, timeout=15,
    )
    assert ignored.returncode == 0


def test_editor_encoding_matches_application_and_vendor_sources():
    prefs = (ROOT / ".settings/org.eclipse.core.resources.prefs").read_text(encoding="utf-8")
    assert "encoding/<project>=UTF-8" in prefs
    assert "encoding//libraries=GBK" in prefs
    assert "encoding//libraries/zf_device/zf_device_mt9v03x.c=UTF-8" in prefs
    for folder in ["code", "user"]:
        for path in (ROOT / folder).rglob("*"):
            if path.suffix in {".c", ".h", ".inc"}:
                assert "\ufffd" not in path.read_text(encoding="utf-8"), path
    main = (ROOT / "user/cpu0_main.c").read_text(encoding="utf-8")
    assert "屏幕菜单" in main and "摄像头采集" in main
