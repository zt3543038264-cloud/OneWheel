"""Mode 10 interactive WiFi telemetry capture and parameter tuning tool.
Uses standard Python library (socket, struct, time).
"""
import argparse
import socket
import struct
import sys
import time

TAIL = b"\x00\x00\x80\x7f"

CHANNELS_DUAL = [
    "telemetry_ms", "rate_seq", "imu_seq", "imu_status", "rate_state",
    "dual_enabled", "dual_roll_actual_deg", "dual_roll_target_deg", "dual_roll_error_deg",
    "dual_roll_target_rate_dps", "dual_roll_rate_dps", "dual_roll_rate_error_dps", "dual_roll_pwm",
    "dual_pitch_actual_deg", "dual_pitch_target_deg", "dual_pitch_error_deg",
    "dual_pitch_target_rate_dps", "dual_pitch_rate_dps", "dual_pitch_rate_error_dps", "dual_pitch_pwm",
    "fly_angle_kp", "fly_rate_kp", "fly_sign", "fly_rate_limit_dps", "fly_pwm_limit",
    "ground_angle_kp", "ground_rate_kp", "ground_sign", "ground_rate_limit_dps", "ground_pwm_limit",
    "motor_left_pwm", "motor_right_pwm", "motor_ground_pwm", "imu_dt_ms", "fault_reason",
    "fault_imu_status", "fault_imu_dt_ms", "imu_spi_timeouts", "imu_not_ready", "startup_scale",
    "flywheel_left_speed_raw", "flywheel_right_speed_raw", "driver_speed_frames", "driver_feedback_age_ms", "driver_feedback_fresh"
]

def parse_justfloat(payload, channel_names):
    if len(payload) < 4 or payload[-4:] != TAIL:
        return None
    num_floats = (len(payload) - 4) // 4
    if num_floats <= 0:
        return None
    fmt = f"<{num_floats}f"
    values = struct.unpack(fmt, payload[:-4])
    data = {}
    for i, val in enumerate(values):
        name = channel_names[i] if i < len(channel_names) else f"ch_{i}"
        data[name] = val
    return data

def send_slider(sock, peer_ip, peer_port, name, value):
    cmd = f"[slider,{name},{value:.6g}]"
    sock.sendto(cmd.encode("ascii"), (peer_ip, peer_port))

def main():
    parser = argparse.ArgumentParser(description="Mode 10 WiFi Tuner")
    parser.add_argument("--port", type=int, default=8081, help="UDP listening port")
    parser.add_argument("--car-port", type=int, default=5001, help="Car UDP command port")
    parser.add_argument("--car-ip", type=str, default="", help="Car IP (auto-detect if blank)")
    parser.add_argument("--seconds", type=float, default=3.0, help="Sampling duration in seconds")
    parser.add_argument("--timeout", type=float, default=5.0, help="Wait timeout for car packets")
    parser.add_argument("--set", action="append", default=[], help="Set parameter: name=value")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(("0.0.0.0", args.port))
    except Exception as e:
        print(f"ERROR: Cannot bind 0.0.0.0:{args.port}: {e}", file=sys.stderr)
        return 1

    sock.settimeout(0.2)
    car_ip = args.car_ip
    car_port = args.car_port

    print(f"Listening on 0.0.0.0:{args.port} (timeout={args.timeout}s)...")
    start_time = time.monotonic()
    detected = False

    while time.monotonic() - start_time < args.timeout:
        try:
            data, addr = sock.recvfrom(2048)
        except socket.timeout:
            continue

        if not car_ip:
            car_ip = addr[0]
            print(f"Detected car at {car_ip}:{addr[1]}")
        detected = True
        break

    if not detected:
        print(f"TIMEOUT: No UDP packets received from car on port {args.port}.", file=sys.stderr)
        return 2

    # Send parameter updates if requested
    for item in args.set:
        if "=" in item:
            name, val_str = item.split("=", 1)
            val = float(val_str)
            print(f"TX -> [slider,{name},{val}] to {car_ip}:{car_port}")
            send_slider(sock, car_ip, car_port, name, val)
            time.sleep(0.05)

    # Collect samples
    print(f"Sampling telemetry for {args.seconds}s...")
    sample_start = time.monotonic()
    frames = []
    replies = []

    while time.monotonic() - sample_start < args.seconds:
        try:
            data, addr = sock.recvfrom(2048)
        except socket.timeout:
            continue

        if data.startswith(b"MCAR "):
            replies.append(data.decode("ascii", errors="replace").strip())
            continue

        parsed = parse_justfloat(data, CHANNELS_DUAL)
        if parsed is not None:
            frames.append(parsed)

    print(f"Received {len(frames)} frames. Replies: {replies}")

    if not frames:
        print("No valid telemetry frames parsed.", file=sys.stderr)
        return 0

    # Analyze data
    roll_angles = [f.get("dual_roll_actual_deg", 0.0) for f in frames]
    roll_rates = [f.get("dual_roll_rate_dps", 0.0) for f in frames]
    pwms = [f.get("dual_roll_pwm", 0.0) for f in frames]
    speeds = [(f.get("flywheel_left_speed_raw", 0.0) - f.get("flywheel_right_speed_raw", 0.0)) * 0.005 for f in frames]

    avg_roll = sum(roll_angles) / len(roll_angles)
    min_roll, max_roll = min(roll_angles), max(roll_angles)
    avg_rate = sum(roll_rates) / len(roll_rates)
    max_rate = max(abs(r) for r in roll_rates)
    max_pwm = max(abs(p) for p in pwms)
    avg_speed = sum(speeds) / len(speeds)
    speed_drift = speeds[-1] - speeds[0]

    # Rate variance / oscillation amplitude
    rate_var = sum((r - avg_rate) ** 2 for r in roll_rates) / len(roll_rates)
    rate_std = rate_var ** 0.5

    last_f = frames[-1]
    cur_fly_angle_kp = last_f.get("fly_angle_kp", 0.0)
    cur_fly_rate_kp = last_f.get("fly_rate_kp", 0.0)
    rate_state = int(last_f.get("rate_state", 0))

    print("=" * 60)
    print(f"ANALYSIS SUMMARY (Samples: {len(frames)}, State: {rate_state})")
    print(f"  Roll Angle:      avg={avg_roll:+.2f} deg, range=[{min_roll:+.2f}, {max_roll:+.2f}] (span: {max_roll-min_roll:.2f} deg)")
    print(f"  Roll Gyro Rate:  avg={avg_rate:+.2f} dps, max_abs={max_rate:.1f} dps, std={rate_std:.2f} dps")
    print(f"  Flywheel PWM:    max_abs={max_pwm:.0f} / 8000")
    print(f"  Flywheel Speed:  end={speeds[-1]:.1f}, drift={speed_drift:+.1f}")
    print(f"  Parameters:      fly_rate_kp={cur_fly_rate_kp:.2f}, fly_angle_kp={cur_fly_angle_kp:.2f}")
    print("=" * 60)

    # Tuning diagnosis
    if rate_state != 2:
        print("DIAGNOSIS: System is not in running state (rate_state != 2). Start the car in Mode 10.")
    elif max_pwm >= 7900:
        print("DIAGNOSIS: PWM saturated near 8000 limit!")
    elif rate_std > 50.0:
        print("DIAGNOSIS: High-frequency oscillation / shaking detected (Inner gain may be too high).")
    elif max_rate < 5.0 and (max_roll - min_roll) < 1.0:
        print("DIAGNOSIS: Very stable / stationary.")
    else:
        print("DIAGNOSIS: Active oscillation / motion.")

    return 0

if __name__ == "__main__":
    sys.exit(main())
