#ifndef BEACON_CORE_H
#define BEACON_CORE_H
#include <stdint.h>
/* Shared POD snapshots only. Legacy classifier/planner types stay private. */
typedef struct {
    int mode;                 /* 0=lane, 1=distance, 2=time, 3=time-pro */
    int threshold, radius, min_gray, max_miss;
    float min_std, min_ratio, near_area, match_px;
    float speed_mps, yaw_limit_dps, mid_x;
    float weight_dist, weight_angle, weight_continuity;
    int track_row, turn_row, search_frames;
    float line_kp, line_offset, search_error_px;
} beacon_config_t;
typedef enum {
    BEACON_FOLLOW_HOLD = 0,
    BEACON_FOLLOW_TRACK,
    BEACON_FOLLOW_NEAR,
    BEACON_FOLLOW_SEARCH
} beacon_follow_state_t;

typedef struct {
    uint8_t valid, count, total, next, turn, track;
    uint8_t order[3];
    float cx, cy, distance_m, bearing_deg;
    float route_distance_m, route_angle_deg;
    /* valid remains current-frame detection. Searching never invents a target. */
    uint8_t steer_valid, searching, follow_state;
    uint16_t missed_frames;
    float follow_mid_x, error_px; /* error = target cx - dynamic middle */
} beacon_result_t;
void beacon_defaults(beacon_config_t *config);
void beacon_reset(void); /* CPU1: discard tracking history after camera restart */
/* CPU1, once per processed frame. Run edges discard preview/old-run search state. */
void beacon_follow_update(const beacon_config_t *config, uint8_t run_active,
                          beacon_result_t *result);
/* CPU1 only, exact 188x120 frame; binary borrows the existing vision buffer. */
void beacon_process(const uint8_t *gray, uint8_t (*binary)[188],
                    const beacon_config_t *config, beacon_result_t *result);
#endif
