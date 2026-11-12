#include "beacon_core.h"
#include "config.h"
#include <string.h>
void beacon_defaults(beacon_config_t *c)
{
    memset(c,0,sizeof(*c));
    c->mode=0; c->threshold=60; c->radius=3; c->min_gray=100; c->max_miss=5;
    c->min_std=20; c->min_ratio=1; c->near_area=20; c->match_px=3;
    c->speed_mps=.15f; c->yaw_limit_dps=30; c->mid_x=93;
    c->weight_dist=.5f; c->weight_angle=.5f; c->weight_continuity=.2f;
    c->track_row=BEACON_TRACK_ROW_DEFAULT; c->turn_row=BEACON_TURN_ROW_DEFAULT;
    c->line_kp=BEACON_LINE_KP_DEFAULT; c->line_offset=BEACON_LINE_OFFSET_DEFAULT;
    c->search_error_px=BEACON_SEARCH_ERROR_DEFAULT;
    c->search_frames=BEACON_SEARCH_FRAMES_DEFAULT;
}
