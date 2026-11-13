# IMU660RA orthogonal flywheel review profile

This is review-only software. The new vehicle's IMU660RA + 500 Hz EKF estimator
and its exact axis mapping are preserved; no r10h estimator is integrated.
Master control timing and actuator signs have not been validated on hardware. Firmware defaults to all motor outputs locked; A yaw is
also separately disabled. A connected GUI, good IMU status, or a successful
offline test does not establish motor actuation safety.

UART source mapping: A = left yaw flywheel, B = right roll flywheel,
C = ground wheel. Physical wiring and axis signs remain unverified. Roll recovery
uses B feedback only. The existing `flywheel_common_rpm` wire field at run channel
index 13 now means A yaw RPM; its identifier and position remain compatible.
Mode names and parameter keys remain unchanged. No existing PID values are
claimed suitable for the new vehicle.

Status 14 means adaptation locked; status 15 means yaw disabled. The menu and
station show these conditions explicitly. Use Mock for offline UI review.
