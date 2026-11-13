"""Regenerate bundled mock metadata from the checked-in authoritative parameter table."""
from pathlib import Path
import json
import re
import sys
import argparse
import math

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--check", action="store_true", help="Report stale generated files without writing")
args = parser.parse_args()
stale = []
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from app.core.live_parameters import LIVE_CONTROL_PARAMETERS
root = Path(__file__).resolve().parents[3]
station = Path(__file__).resolve().parents[1]
table = (root / "code/Common/param.c").read_text(encoding="utf-8")
board = (root / "code/Common/board_config.h").read_text(encoding="utf-8")
menu = (root / "code/Menu/menu.c").read_text(encoding="utf-8")
macros = dict(re.findall(r"^#define\s+(\w+)\s+([^\n/]+)", board, re.M))
def numeric(value, depth=0):
    value = value.strip().strip("() ").rstrip("fFuU")
    if depth > 8:
        raise ValueError(value)
    if value in macros:
        return numeric(macros[value], depth+1)
    return float(value)

pid = [f"{axis}_{ring}_k{k}" for axis, rings in [("r", ("rate", "angle", "rcy")), ("p", ("rate", "angle", "vel")), ("y", ("rate", "angle"))] for ring in rings for k in "pid"]
groups = ["Roll", "Pitch", "Yaw", "Lean", "Run", "Camera", "Element", "Motor", "Zero", "Odometry", "IPM"]
def group(n):
    for prefix, g in [("r_", "Roll"), ("p_", "Pitch"), ("y_", "Yaw"), ("lean_", "Lean"), ("ipm_", "IPM"), ("odom_", "Odometry"), ("run_", "Run"), ("direction_", "Run"), ("elem_", "Element"), ("ring_", "Element"), ("zebra_", "Element"), ("cam_", "Camera"), ("err_front", "Camera"), ("imu_forward_", "Zero"), ("roll_", "Zero"), ("pitch_", "Zero")]:
        if n.startswith(prefix):
            return g
    return "Motor"
steps = {name: (float(step.rstrip("f")), int(dec)) for _, name, step, dec in re.findall(r'\{\s*"([^"]+)",\s*"([^"]+)",\s*([\d.]+f?),\s*(\d+)\s*\}', menu)}
defaults = dict(re.findall(r'g_param\.([\w\[\]]+)\s*=\s*(\w+_DEFAULT);', table))
labels = {"imu_forward_dir":"IMU 前向", "run_speed_straight":"直道速度", "run_speed_curve":"弯道速度", "cam_exposure":"摄像头曝光", "err_front_row":"前瞻行", "roll_zero_init":"横滚机械零点", "pitch_zero_init":"俯仰机械零点", "roll_protect":"横滚保护角", "pitch_protect":"俯仰保护角", "fly_speed_limit":"飞轮限速", "fly_slew":"飞轮输出变化率", "odom_counts_per_m":"每米编码器计数", "odom_test_speed":"里程测试速度", "steer_dir":"转向极性", "enc_dir_c":"编码器极性", "lean_turn_kp":"方向预压弯", "lean_speed_kp":"速度压弯倍率", "lean_slew_dps":"压弯变化速度", "lean_max_angle":"压弯最大角度", "motor_dir_a":"A Left/Yaw 方向", "motor_dir_b":"B Right/Roll 方向", "motor_dir_c":"C 地轮方向", "r_rate_notch_hz":"Roll 陷波中心频率", "r_rate_notch_q":"Roll 陷波品质因数", "r_rate_notch_db":"Roll 陷波深度"}
lean_units = {'lean_turn_kp': 's', 'lean_speed_kp': '倍', 'lean_slew_dps': '°/s', 'lean_max_angle': '°'}
lean_descriptions = {'lean_turn_kp': '乘以去死区转弯率，低速淡出；当前范围以 MCU Schema 为准。', 'lean_speed_kp': 'v×转弯率/g 的倍率；当前实际范围以 MCU Schema 为准。', 'lean_slew_dps': '正常入弯、反向和回正的变化上限；停车/串级复位直接清零。'}
params, metadata, configured_defaults = [], {}, {}
pattern = r'\{\s*"([^"]+)",\s*&g_param\.([\w\[\]]+),\s*([01]),\s*([^,]+),\s*([^}]+)\}'
for name, member, floating, low, high in re.findall(pattern, table.split("#define PARAM_TABLE_NUM")[0]):
    g = group(name)
    lo, hi = numeric(low), numeric(high)
    # Never hide a missing/unsupported firmware default behind an invented zero.
    value = numeric(defaults[member])
    if not math.isfinite(value) or not lo <= value <= hi:
        raise ValueError(f"{name}: firmware default {value} is outside [{lo}, {hi}]")
    if floating == "0": value = int(value)
    configured_defaults[name] = dict(type="float" if floating == "1" else "int",
        value=value, default_macro=defaults[member])
    step, decimals = steps.get(name, (.0001 if floating == "1" else 1, 6 if floating == "1" else 0))
    if name.startswith("elem_en_"):
        step = 1
    params.append(dict(name=name, type="float" if floating == "1" else "int", min=lo, max=hi, value=value,
                       group=g, step=step, flags=4 | (1 if name in pid or name in LIVE_CONTROL_PARAMETERS else 0) | (16 if name in LIVE_CONTROL_PARAMETERS else 0) | (2 if g in ("Motor", "Zero", "IPM") else 0)))
    label = labels.get(name, "")
    if name in pid:
        axis, ring, gain = name.split("_")
        label = {"r":"横滚", "p":"俯仰", "y":"航向"}[axis] + {"rate":"角速度环", "angle":"角度环", "rcy":"回收环", "vel":"速度环"}[ring] + " " + gain.upper()
    if not label:
        label = {"Run":"跑车", "Element":"元素", "Motor":"电机", "IPM":"逆透视"}.get(g, g) + " · " + name
    metadata[name] = dict(label=label, step=step, decimals=decimals, default=value, description="当前范围、类型与权限由 MCU Schema 确认；默认值仅供参考。")
    if name in lean_units:
        metadata[name]["unit"] = lean_units[name]
    if name in lean_descriptions:
        metadata[name]["description"] = lean_descriptions[name]
    if re.fullmatch(r"[rpy]_angle_k[pid]", name):
        factor = {"kp": "Kp 保持原值", "ki": "Ki×0.2", "kd": "Kd×5"}[name.rsplit("_", 1)[1]]
        metadata[name]["description"] = f"角度参数沿用原 5 ms 基准；1 kHz 固件内部 {factor}，无需手动换算。"
    if name == "r_rate_kd":
        metadata[name]["description"] = "正数表示已确认实车的 D 作用方向，内部系数为负；不与 Roll P 一起翻转。"
    if name.startswith("y_rate_k"):
        metadata[name]["description"] = "正数表示当前实车的 Yaw 作用方向，内部统一乘 -1。"
    if name.startswith("r_rate_notch_"):
        metadata[name]["unit"] = {"r_rate_notch_hz": "Hz", "r_rate_notch_q": "", "r_rate_notch_db": "dB"}[name]
        metadata[name]["description"] = "Roll 角速度陷波参数；深度 0 dB 为严格旁路。在线修改仍需 MCU Schema 权限与状态确认。"
    if name == "steer_dir":
        metadata[name]["description"] = "+1 保持当前视觉/遥控转向纠偏，-1 反转；不是 IMU 或电机方向。"
    if name == "imu_forward_dir":
        metadata[name].update({'label': 'IMU Frame（固定）', 'step': 2.0, 'decimals': 0, 'default': 1, 'description': '兼容字段，只读含义：保留 new 的 IMU660RA + 500Hz EKF 编译期机体系映射。改值会返回 FIXED_FRAME，不改变零点，无需保存或重启。', 'enum_options': {'-1': '固定机体系 −1', '1': '固定机体系 +1'}})
    if "dir" in name and g == "Motor":
        metadata[name]["enum_options"] = {"-1":"相对基准反向 −1", "1":"当前实车基准 +1"}

names = ["uptime_ms", "roll", "roll_target", "roll_rate", "recovery_feedback", "recovery_output", "roll_output", "lean_offset", "yaw_rate_target", "yaw_rate_command", "yaw_rate_actual", "yaw_output_raw", "yaw_output_applied", "flywheel_common_rpm", "direction_offset", "lateral_error", "heading_error", "curvature", "speed_plan", "speed_ramp", "speed_actual", "momentum_scale", "vision_quality", "vision_age_ms", "state_flags"]
cn = ["MCU 时间", "Roll 实际角", "Roll 目标角", "Roll 角速度", "回收反馈 A−B", "回收输出", "Roll 输出", "压弯零点", "Yaw 目标角速度", "Yaw 命令", "Yaw 实际角速度", "Yaw 混控前", "Yaw 实际分配", "A/B 共模转速", "方向像素误差", "BEV 横向误差", "航向误差", "曲率", "规划速度", "速度斜坡", "实测速度", "Yaw 动量权限", "视觉质量", "视觉帧年龄", "状态位"]
units = ["ms", "°", "°", "°/s", "RPM", "°", "duty", "°", "°/s", "°/s", "°/s", "duty", "duty", "RPM", "pixel", "", "°", "", "m/s", "m/s", "m/s", "", "", "ms", ""]
stat = ["stat_uptime", "start_mode", "run_active", "test_mode", "jog_mode", "imu_state", "bldc_state", "cam_state", "att_state", "run_stop", "test_status", "param_revision", "rx_error", "tx_drop", "pid_write_mask"]
profile = dict(schema_version=1, kind="device_profile", id="tc264_unicycle", name='TC264D 正交飞轮 · IMU660RA · 审查版', version="1.1", protocol_plugin="firewater_v1", supports_configuration=True,
    connection_defaults=dict(baudrate=115200, bytesize=8, parity="N", stopbits=1, encoding="ascii", ending="\r\n"),
    channels={"att":[dict(name=n, label=l, unit="°") for n,l in zip(["att_roll","pitch","yaw"], ["Roll 姿态","Pitch 姿态","连续 Yaw"])],
              "run":[dict(name=n,label=c,unit=u) for n,c,u in zip(names,cn,units)], "stat":[dict(name=n,label=n,unit="") for n in stat]},
    parameters=metadata, parameter_groups=groups, mock_parameters=params, pid_mask={n:i for i,n in enumerate(pid)},
    dashboards=["att_roll","pitch","yaw","start_mode","run_active","test_mode","jog_mode","imu_state","bldc_state","cam_state","att_state","run_stop","test_status","speed_plan","speed_ramp","speed_actual","flywheel_common_rpm","momentum_scale","track_valid","ipm_valid","vision_quality","vision_age_ms","element","param_revision"],
    safety_rules=dict(remote_start=False, flash_requires_stopped=True, status_timeout_ms=700,
                      warning='断开电机动力后分阶段验收：IMU660RA原500Hz EKF；普通闭环/A-Yaw/C锁定，专用B信号须匹配外板与物理授权。控制时序/方向待验证；不是可驾驶版本。'),
    orientation_mapping=dict(channels=["att_roll","pitch","yaw"], axes=[0,1,2], signs=[1,1,1], units="deg", order="ZYX", front="+X", verified=False, dimensions=[3,1.3,.65]),
    uptime_tag="run", telemetry_period_ms=20,
    status_bits={name:["state_flags",shift,mask] for name,shift,mask in [("run_flag",0,1),("balance_flag",1,1),("track_valid",2,1),("vision_applied",3,1),("ipm_valid",4,1),("hold",5,1),("zebra_stop",6,1),("path_active",7,1),("element",8,15),("stop_reason",12,7)]},
    derived_channels=dict(roll_error="roll_target - roll", yaw_clip="yaw_output_raw - yaw_output_applied", speed_error="speed_plan - speed_actual", vision_timeout="vision_age_ms > 100"),
    plot_presets={"Roll":names[1:7], "Yaw":names[8:13]+["momentum_scale"], "速度":names[18:21], "视觉":names[14:18]+names[22:24], "状态":["track_valid","ipm_valid","hold","element","stop_reason","telemetry_lost"]},
    default_plot_channels={"Roll":["roll","roll_target"],"Yaw":["yaw_rate_target","yaw_rate_command","yaw_rate_actual"]},
    workbenches={"Roll":names[1:7],"Pitch":["pitch","speed_actual","speed_error"],"Yaw":names[8:13]})
def save(path, data):
    serialized = json.dumps(data, ensure_ascii=False, indent=2)
    if args.check:
        if not path.exists() or path.read_text(encoding="utf-8") != serialized:
            stale.append(str(path.relative_to(station)))
    else:
        path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text(serialized,encoding="utf-8")
display = json.loads((station/"app/profiles/tc264_unicycle/display.json").read_text(encoding="utf-8"))
profile["channels"].update(display.pop("channels", {}))
profile.update(display)
save(station/"app/profiles/tc264_unicycle/profile.json", profile)
generic = dict(schema_version=1, kind="device_profile", id="generic_sensor", name="通用双通道传感器", version="1.0", protocol_plugin="firewater_v1",
               connection_defaults=dict(baudrate=115200), channels={"data":[dict(name="sensor_a",label="传感器 A",unit="V"),dict(name="sensor_b",label="传感器 B",unit="V")]}, parameters={}, parameter_groups=[], dashboards=["sensor_a","sensor_b"], safety_rules={}, orientation_mapping={}, plot_presets={"传感器":["sensor_a","sensor_b"]})
save(station/"app/profiles/generic_sensor/profile.json", generic)
save(station/"examples/default.workspace.json", dict(schema_version=1, kind="workspace", profile=profile, plots=[dict(title=k,channels=v,seconds=10,styles={}) for k,v in list(profile["plot_presets"].items())[:2]], connection=profile["connection_defaults"], favorites=[], orientation=profile["orientation_mapping"]))
save(station/"examples/zero_pid.preset.json", dict(schema_version=1,kind="parameter_preset",profile_id=profile["id"],values={n:0 for n in pid},description="仅演示导入与差异预览；零增益不能用于实车平衡"))
save(station/"docs/qcar-defaults.json", dict(kind="qcar_configured_defaults",
    hardware_readback=False, source_files=["code/Common/param.c", "code/Common/board_config.h"],
    reference_snapshot="defaults-20260921.json (historical 75-entry readback retained separately)",
    transport="serial", count=len(params),
    note="Current review configuration derived from param_load_defaults and board_config.h, not a hardware readback or vehicle tuning result. wire_value is intentionally absent.",
    parameters=configured_defaults))
if stale:
    parser.exit(1, "Stale generated files: " + ", ".join(stale) + "\n")
print(f"{'Checked' if args.check else 'Generated'} profile/defaults from {len(params)} authoritative table entries")
