"""Compatibility test entry point: superseded staged port, now new-branch EKF."""
from pathlib import Path
import subprocess,sys
subprocess.run([sys.executable,str(Path(__file__).resolve().parents[1]/"new_ekf/run.py")],check=True)
