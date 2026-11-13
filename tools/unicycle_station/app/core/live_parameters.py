"""Mock parity with the MCU explicit runtime control allowlist.

Keep this explicit and compare it to cfg_live_control in the schema regression.
Never grant runtime permission merely from a parameter prefix.
"""
LIVE_CONTROL_PARAMETERS = frozenset([
    "run_speed_straight", "run_speed_curve", "run_speed_cross", "run_speed_ring",
    "run_speed_ramp", "run_speed_lost", "run_accel_mps2", "run_decel_mps2",
    "direction_pixel_kp", "direction_heading_kp", "direction_curve_kff",
    "direction_rate_kd", "lean_turn_kp", "lean_speed_kp", "lean_slew_dps",
    "lean_max_angle", "r_rate_notch_hz", "r_rate_notch_q", "r_rate_notch_db",
])
