#ifndef PATH_NAV_H_
#define PATH_NAV_H_

#include <stdint.h>

/* CPU0 only. All mutators run in the control ISR or with its interrupt masked.
 * Coordinates: x = sin(relative yaw), y = cos(relative yaw), distances in m.
 * Route points include the origin; s is cumulative polyline length. */
#define PATH_MAX_POINTS 256u
#define PATH_SAMPLE_M 0.20f
#define PATH_SPEED_DEFAULT 0.20f
#define PATH_SPEED_MAX 1.50f
#define PATH_ACCEL_MPS2 0.25f
#define PATH_DECEL_MPS2 0.50f
#define PATH_YAW_RATE_LIMIT 45.0f
#define PATH_YAW_ANGLE_KP 3.0f /* deg/s per deg, Path only; no angle integral */
#define PATH_YAW_SLEW_DPS2 180.0f
#define PATH_START_YAW_LIMIT 30.0f

typedef struct { float x, y, s; } path_point_t;
typedef enum {
    PATH_IDLE = 0, PATH_RECORDING, PATH_READY, PATH_RUNNING,
    PATH_BRAKING, PATH_DONE, PATH_ERROR
} path_state_t;
typedef enum {
    PATH_OK = 0, PATH_FULL, PATH_REVERSE, PATH_INPUT, PATH_OFF_ROUTE,
    PATH_STALLED, PATH_ABORTED
} path_reason_t;
typedef struct {
    path_state_t state;
    path_reason_t reason;
    uint16_t count, segment;
    uint8_t dirty, valid;
    float x, y, yaw_deg, progress_m, length_m;
    float speed_mps, yaw_target_deg, yaw_rate_dps, cross_track_m;
    float heading_reference_deg;
    uint8_t heading_locked;
} path_status_t;

void path_nav_init(void);
int path_nav_record(float yaw_deg);
void path_nav_finish_record(void);
int path_nav_start(float yaw_deg, float speed_mps);
/* RAM reference survives stop/restart, invalidated by new/imported routes.
 * Capture only while inactive; caller must enforce vehicle STOP/IMU gates. */
int path_nav_lock_heading(float yaw_deg);
int path_nav_heading_ready(float yaw_deg);
void path_nav_abort(void);
void path_nav_clear(void);
/* 5 ms odometry integration; planner runs once per four calls. */
void path_nav_tick(float delta_m, float yaw_deg, float actual_speed_mps,
                   float momentum_scale);
void path_nav_status(path_status_t *out);
int path_nav_point(uint16_t index, path_point_t *out);
int path_nav_busy(void);
int path_nav_running(void);
int path_nav_recording(void);
/* Foreground only, inactive route. Validate before replacing RAM data. */
int path_nav_validate(const path_point_t *points, uint16_t count);
int path_nav_import(const path_point_t *points, uint16_t count);
void path_nav_saved(void);
void path_nav_modified(void);
uint32_t path_nav_revision(void);
/* Foreground only: bounded route-wide braking envelope before arming. */
void path_nav_prepare(void);

#endif
