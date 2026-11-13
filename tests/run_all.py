"""Run the reference station/firmware tests and Qcar-specific driver contract."""
from pathlib import Path
import os,subprocess,sys
root=Path(__file__).resolve().parents[1]
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
python=root/'.venv/Scripts/python.exe'
if not python.exists():python=Path(sys.executable)
raise SystemExit(subprocess.call([str(python),'-m','pytest','-q',str(root/'tools/unicycle_station/tests'),str(root/'tests'),*sys.argv[1:]],cwd=root))
