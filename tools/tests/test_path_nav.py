"""Compile the actual portable navigation/Flash code and exercise host models.

Run: python tools/tests/test_path_nav.py
Requires GCC. No connection to the vehicle, no flashing, no proof of balance.
"""
from pathlib import Path
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
gcc = shutil.which("gcc")
if not gcc:
    raise SystemExit("GCC is required for the navigation host tests")
with tempfile.TemporaryDirectory(prefix="unicycle-path-test-") as tmp:
    exe = Path(tmp) / "path_test.exe"
    subprocess.run([
        gcc, "-std=c99", "-O2", "-Wall", "-Wextra", "-Werror", "-Wno-unknown-pragmas",
        "-DSTORAGE_FLASH_VALIDATED=1", "-DSTORAGE_BOOT_LOAD=1", "-DPATH_HOST_TEST", "-DPATH_STORE_TEST", "-I", str(root / "code"), "-I", str(root / "tools/tests"),
        str(root / "code/Controller/Planner/path_nav.c"), str(root / "code/Controller/Planner/path_store.c"),
        str(root / "tools/tests/path_nav_test.c"), "-lm", "-o", str(exe),
    ], check=True)
    subprocess.run([str(exe)], check=True)
