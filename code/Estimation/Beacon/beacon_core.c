#if defined(__TASKING__)
#pragma section all "cpu1_dsram"
#pragma section code "beacon_flash"
#endif
#include "beacon_core.h"
#include "beacon_platform.h"

Mono_beacon mono_beacon;
static int previous_mode = -1;

/* Original track_turn_update shared planner globals with the control ISR.
 * One CPU1 writer now owns pursuit history; CPU0 gets a POD snapshot only. */
static struct {
    int mode;
    uint8_t running, seen, turn, last_turn, track;
    uint16_t missed;
    float last_cx;
} follow;

static void beacon_follow_reset(void)
{
    memset(&follow,0,sizeof(follow));
    follow.mode=-1;
}

void beacon_reset(void)
{
    previous_mode=-1;
    beacon_follow_reset();
}

void beacon_follow_update(const beacon_config_t *c, uint8_t running, beacon_result_t *r)
{
    float shift;
    uint8_t direction;
    if(!r) return;
    r->steer_valid=r->searching=0;
    r->follow_state=BEACON_FOLLOW_HOLD;
    r->missed_frames=0;
    r->error_px=0;
    r->follow_mid_x=0;
    if(!c || c->mode<1 || c->mode>3 ||
       c->track_row<25 || c->turn_row>119 || c->track_row>=c->turn_row ||
       c->search_frames<1 || c->search_frames>100 ||
       !isfinite(c->mid_x) || !isfinite(c->line_kp) ||
       !isfinite(c->line_offset) || !isfinite(c->search_error_px) ||
       c->mid_x<0 || c->mid_x>187 || c->line_kp<0 || c->line_kp>2 ||
       c->line_offset<0 || c->line_offset>187 ||
       c->search_error_px<0 || c->search_error_px>94) {
        beacon_follow_reset();
        return;
    }
    running=(uint8_t)(running!=0);
    if(follow.mode!=c->mode || follow.running!=running) {
        beacon_follow_reset();
        follow.mode=c->mode;
        follow.running=running;
        r->track=0;
        turn_info.turn_flag=turn_info.track_flag=turn_info.last_turn_flag=0;
    }
    r->follow_mid_x=c->mid_x;
    if(r->valid && isfinite(r->cx) && isfinite(r->cy) &&
       r->cx>=0 && r->cx<188 && r->cy>=0 && r->cy<120) {
        follow.seen=1;
        follow.missed=0;
        follow.last_cx=r->cx;
        if(follow.turn) {
            follow.last_turn=follow.turn;
            follow.track=0;
        }
        follow.turn=0;
        if(r->cy<c->track_row) {
            follow.track=(r->track==left || r->track==right) ? r->track :
                         (follow.last_turn==right ? right_right : left_left);
            r->follow_state=BEACON_FOLLOW_TRACK;
        } else {
            direction=(follow.track==right_right ||
                       (!follow.track && follow.last_turn==right)) ? right : left;
            shift=c->line_kp*r->cy-c->line_offset;
            r->follow_mid_x=c->mid_x+(direction==left ? shift : -shift);
            if(r->follow_mid_x<0) r->follow_mid_x=0;
            if(r->follow_mid_x>187) r->follow_mid_x=187;
            if(r->cy>=c->turn_row) follow.turn=direction;
            r->follow_state=BEACON_FOLLOW_NEAR;
        }
        r->error_px=r->cx-r->follow_mid_x;
        r->steer_valid=1;
    } else if(follow.seen) {
        if(follow.missed<65535u) follow.missed++;
        /* Near-turn latch searches immediately; otherwise wait for 20 misses
         * by default. No target history means no autonomous search. */
        if(!follow.turn && follow.missed>=c->search_frames)
            follow.turn=follow.last_cx>=c->mid_x ? left : right;
        if(running && follow.turn && c->search_error_px>0) {
            follow.last_turn=follow.turn;
            follow.track=0;
            r->error_px=follow.turn==left ? -c->search_error_px : c->search_error_px;
            r->steer_valid=r->searching=1;
            r->follow_state=BEACON_FOLLOW_SEARCH;
        }
    }
    r->missed_frames=follow.missed;
    r->turn=follow.turn;
    r->track=follow.track;
    mono_beacon.dynamic_mid_line=r->follow_mid_x;
    turn_info.turn_flag=follow.turn;
    turn_info.track_flag=follow.track;
    turn_info.last_turn_flag=follow.last_turn;
}


void beacon_process(const uint8_t *gray, uint8_t (*binary)[188],
                    const beacon_config_t *c, beacon_result_t *r)
{
    int count,selected;
    if(!r) return;
    memset(r,0,sizeof(*r));
    if(!gray || !binary || !c || c->mode<1 || c->mode>3) {beacon_reset();return;}
    if(previous_mode!=c->mode) {
        beacon_follow_reset();
        /* Hot paths do not need the old heap-allocated morphology kernels. */
        memset(&imageinfo,0,sizeof(imageinfo));
        memset(&turn_info,0,sizeof(turn_info));
        memset(&mono_beacon,0,sizeof(mono_beacon));
        memset(&best_combined_show,0,sizeof(best_combined_show));
        imageinfo.width=188; imageinfo.height=120; imageinfo.minrow=19;
        cnt=0;
        mono_beacon.dynamic_mid_line=c->mid_x;
        previous_mode=c->mode;
    }
    imageinfo.image=(uint8_t *)gray; imageinfo.binaryimage=binary;
    imageinfo.contour_filter_sort=imageinfo.contourinfo;
    imageinfo.fixed_threshold=(float)c->threshold;
    imageinfo.domain_radius=(float)c->radius;
    imageinfo.std_var_threshold=c->min_std;
    imageinfo.boxratio_threshold=c->min_ratio;
    imageinfo.near_contour_max_area=c->near_area;
    img_process_params.max_std=40; img_process_params.max_gray=200;
    img_process_params.iou=.01f; img_process_params.min_std=c->min_std;
    img_process_params.min_gray=(float)c->min_gray;
    img_process_params.max_miss=(float)c->max_miss;
    img_process_params.max_distance=c->match_px;
    img_process_params.small_area=20; img_process_params.range_mean=25;
    img_process_params.range_gray=25; img_process_params.max_dx=8; img_process_params.max_dy=8;
    mono_beacon.img_mid_line=c->mid_x;
    mono_beacon.turn_threshold=(float)c->turn_row;
    mono_beacon.track_threshold=(float)c->track_row;
    turn_info.weight_dist=c->weight_dist; turn_info.weight_angle=c->weight_angle;
    turn_info.weight_first_cx_diff=c->weight_continuity; turn_info.diff_dead_zone=2;
    memset(imageinfo.contourinfo,0,sizeof(imageinfo.contourinfo));
    count=c->mode==1?image_process_distance(&imageinfo,&turn_info):
          c->mode==2?image_process_time(&imageinfo,&turn_info):
                     image_process_time_pro(&imageinfo,&turn_info);
    r->total=(uint8_t)imageinfo.totalContours;
    r->turn=(uint8_t)turn_info.turn_flag; r->track=(uint8_t)turn_info.track_flag;
    if(count<1 || count>MAX_BEACONS) return;
    selected=turn_info.next_beacon-1;
    if(selected<0 || selected>=count) return;
    r->cx=turn_info.cx; r->cy=turn_info.cy;
    r->distance_m=imageinfo.contour_filter[selected].distance;
    r->bearing_deg=imageinfo.contour_filter[selected].angle;
    if(!isfinite(r->cx)||!isfinite(r->cy)||!isfinite(r->distance_m)||
       r->cx<0||r->cx>=188||r->cy<0||r->cy>=120||r->distance_m<=0||r->distance_m>=100) return;
    r->valid=1; r->count=(uint8_t)count; r->next=(uint8_t)(selected+1);
    for(selected=0;selected<count;selected++)r->order[selected]=(uint8_t)best_combined_show.order[selected];
    r->route_distance_m=best_combined_show.total_dist; r->route_angle_deg=best_combined_show.total_angle;
}
