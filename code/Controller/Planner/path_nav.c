#include "Controller/Planner/path_nav.h"
#include <math.h>
#include <string.h>

#ifndef PATH_HOST_TEST
#pragma section all "cpu0_dsram"
#endif

#define RAD (0.017453292519943295f)
#define DEG (57.29577951308232f)
#define LOOKAHEAD_M (0.30f)
#define END_RADIUS_M (0.06f)
#define MAX_CROSS_TRACK_M (0.60f)

static path_point_t s_points[PATH_MAX_POINTS];
static path_status_t s;
static uint32_t s_revision;
static float s_limits[PATH_MAX_POINTS];
static float s_yaw_zero, s_last_yaw, s_cruise, s_reverse;
static float s_progress_seen;
static uint16_t s_stall_ticks, s_brake_ticks;
static uint8_t s_tick;

static int finite_value(float v) { return v == v && v > -1.0e7f && v < 1.0e7f; }
static float clamp(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static float wrap(float deg)
{
    deg = fmodf(deg + 180.0f, 360.0f);
    if (deg < 0.0f) deg += 360.0f;
    return deg - 180.0f;
}
static float distance(float x, float y) { return sqrtf(x*x + y*y); }

static void fail(path_reason_t reason)
{
    if (s.state == PATH_RECORDING) s.valid = 0;
    s.state = PATH_ERROR;
    s.reason = reason;
    s.speed_mps = 0.0f;
    s.yaw_rate_dps = 0.0f;
}

void path_nav_init(void)
{
    s_revision++;
    memset(&s, 0, sizeof(s));
    s_yaw_zero = s_last_yaw = s_reverse = 0.0f;
    s_cruise = PATH_SPEED_DEFAULT;
    s_tick = 0;
    s_stall_ticks = s_brake_ticks = 0;
}

int path_nav_busy(void)
{
    return s.state == PATH_RECORDING || path_nav_running();
}
int path_nav_running(void) { return s.state == PATH_RUNNING || s.state == PATH_BRAKING; }
int path_nav_recording(void) { return s.state == PATH_RECORDING; }
void path_nav_status(path_status_t *out) { if (out) *out = s; }
int path_nav_point(uint16_t i, path_point_t *out)
{
    if (!out || i >= s.count) return 0;
    *out = s_points[i];
    return 1;
}
void path_nav_clear(void)
{
    if (path_nav_busy()) return;
    path_nav_init();
    s.dirty = 1;
}

int path_nav_record(float yaw)
{
    if (path_nav_busy() || !finite_value(yaw)) return 0;
    path_nav_init();
    s_yaw_zero = s_last_yaw = yaw;
    s_points[0].x = s_points[0].y = s_points[0].s = 0.0f;
    s.count = 1;
    s.state = PATH_RECORDING;
    s.dirty = 1;
    return 1;
}

static void append_point(void)
{
    float len = distance(s.x - s_points[s.count-1u].x, s.y - s_points[s.count-1u].y);
    if (len < 0.005f) return;
    if (s.count >= PATH_MAX_POINTS) return;
    s_points[s.count].x = s.x;
    s_points[s.count].y = s.y;
    s.length_m += len;
    s_points[s.count].s = s.length_m;
    s.count++;
    s_revision++;
    if (s.count == PATH_MAX_POINTS)
    {
        s.state = PATH_READY;
        s.valid = 1;
        s.reason = PATH_FULL;
    }
}

void path_nav_finish_record(void)
{
    if (!path_nav_recording()) return;
    append_point();
    s.valid = (uint8_t)(s.count >= 2u && s.length_m >= 0.10f);
    s.state = s.valid ? PATH_READY : PATH_IDLE;
}

void path_nav_abort(void)
{
    s_revision++;
    if (path_nav_recording()) path_nav_finish_record();
    if (path_nav_running())
    {
        s.state = PATH_READY;
        s.reason = PATH_ABORTED;
    }
    s.speed_mps = s.yaw_rate_dps = 0.0f;
}

int path_nav_validate(const path_point_t *p, uint16_t count)
{
    uint16_t i;
    if (!p || count < 2u || count > PATH_MAX_POINTS) return 0;
    if (p[0].x != 0.0f || p[0].y != 0.0f || p[0].s != 0.0f) return 0;
    for (i = 1; i < count; i++)
    {
        float chord, ds;
        if (!finite_value(p[i].x) || !finite_value(p[i].y) || !finite_value(p[i].s)) return 0;
        ds = p[i].s - p[i-1u].s;
        chord = distance(p[i].x - p[i-1u].x, p[i].y - p[i-1u].y);
        if (ds < 0.004f || ds > 1.0f || p[i].s > 1000.0f ||
            fabsf(chord - ds) > 0.002f + ds * 0.001f) return 0;
    }
    return p[count-1u].s >= 0.10f;
}

int path_nav_import(const path_point_t *p, uint16_t count)
{
    if (path_nav_busy() || !path_nav_validate(p, count)) return 0;
    path_nav_init();
    memcpy(s_points, p, (size_t)count * sizeof(p[0]));
    s.count = count;
    s.length_m = p[count-1u].s;
    s.valid = 1;
    s.state = PATH_READY;
    return 1;
}
void path_nav_saved(void) { if (!path_nav_busy()) s.dirty = 0; }
void path_nav_modified(void) { if (!path_nav_busy()) s.dirty = 1; }
uint32_t path_nav_revision(void) { return s_revision; }

void path_nav_prepare(void)
{
    uint16_t i;
    if (path_nav_busy() || s.count < 2u) return;
    for (i=0; i<s.count; i++) s_limits[i] = PATH_SPEED_MAX;
    for (i=1; i+1u<s.count; i++)
    {
        float ax=s_points[i].x-s_points[i-1u].x, ay=s_points[i].y-s_points[i-1u].y;
        float bx=s_points[i+1u].x-s_points[i].x, by=s_points[i+1u].y-s_points[i].y;
        float a=distance(ax,ay), b=distance(bx,by);
        float cosine=clamp((ax*bx+ay*by)/(a*b),-1.0f,1.0f);
        float curvature=sqrtf(fmaxf(0.0f,2.0f*(1.0f-cosine)))/fminf(a,b);
        if (curvature>0.0001f)
            s_limits[i]=fminf(PATH_SPEED_MAX,fminf(sqrtf(0.50f/curvature),0.610865f/curvature));
    }
    s_limits[s.count-1u]=0.0f;
    for (i=s.count-1u; i>0u; i--)
        s_limits[i-1u]=fminf(s_limits[i-1u],sqrtf(s_limits[i]*s_limits[i]+0.80f*(s_points[i].s-s_points[i-1u].s)));
}


int path_nav_start(float yaw, float speed)
{
    if (path_nav_busy() || !s.valid || s.length_m < 0.30f ||
        !path_nav_heading_ready(yaw) || !finite_value(speed) || speed < 0.05f || speed > PATH_SPEED_MAX) return 0;
    if (!s.heading_locked) {
        s.heading_reference_deg = yaw;
        s.heading_locked = 1;
    }
    /* Choose the equivalent reference nearest continuous yaw, preserving
     * physical misalignment rather than rotating the route at every start. */
    s_yaw_zero = yaw - wrap(yaw - s.heading_reference_deg);
    s_last_yaw = yaw;
    s_cruise = speed;
    s.x = s.y = s.progress_m = 0.0f;
    s.yaw_deg = yaw - s_yaw_zero;
    s.speed_mps = s.yaw_rate_dps = s.cross_track_m = 0.0f;
    s.yaw_target_deg = yaw;
    s.segment = 0;
    s.reason = PATH_OK;
    s.state = PATH_RUNNING;
    s_tick = 0;
    s_reverse = s_progress_seen = 0.0f;
    s_stall_ticks = s_brake_ticks = 0;
    return 1;
}

int path_nav_lock_heading(float yaw)
{
    if (path_nav_busy() || !s.valid || !finite_value(yaw)) return 0;
    s.heading_reference_deg = yaw;
    s.heading_locked = 1;
    s_revision++; /* Invalidate an already armed PC start. No Flash change. */
    return 1;
}

int path_nav_heading_ready(float yaw)
{
    return finite_value(yaw) && (!s.heading_locked ||
        fabsf(wrap(yaw - s.heading_reference_deg)) <= PATH_START_YAW_LIMIT);
}

void path_nav_invalidate_heading(void)
{
    s.heading_locked = 0;
    s.heading_reference_deg = 0.0f;
    s_revision++;
}

/* Ordered local projection: never choose a remote crossing or another lap.
 * At most eight segment advances plus 24 preview advances per 20 ms. */
static void plan(float yaw, float speed, float permission)
{
    uint16_t i = s.segment, look;
    uint8_t n;
    float t = 0.0f, px = 0.0f, py = 0.0f, remaining;
    float tx, ty, dx, dy, error, target_s, target_speed, rate;
    if (s.state == PATH_BRAKING)
    {
        s.speed_mps = s.yaw_rate_dps = 0.0f;
        if (fabsf(speed) < 0.04f) s_brake_ticks++; else s_brake_ticks = 0;
        if (s_brake_ticks >= 15u) s.state = PATH_DONE;
        else if (++s_stall_ticks >= 250u) fail(PATH_STALLED);
        return;
    }
    for (n = 0; n < 8u; n++)
    {
        dx = s_points[i+1u].x - s_points[i].x;
        dy = s_points[i+1u].y - s_points[i].y;
        t = ((s.x-s_points[i].x)*dx + (s.y-s_points[i].y)*dy) / (dx*dx+dy*dy);
        if (t < 1.0f || i + 2u >= s.count || n == 7u) break;
        i++;
    }
    t = clamp(t, 0.0f, 1.0f);
    px = s_points[i].x + t*dx;
    py = s_points[i].y + t*dy;
    s.segment = i;
    s.cross_track_m = distance(s.x-px, s.y-py);
    if (s.cross_track_m > MAX_CROSS_TRACK_M) { fail(PATH_OFF_ROUTE); return; }
    target_s = s_points[i].s + t*(s_points[i+1u].s-s_points[i].s);
    if (target_s > s.progress_m) s.progress_m = target_s;
    remaining = s.length_m - s.progress_m;
    if (remaining < END_RADIUS_M &&
        distance(s.x-s_points[s.count-1u].x, s.y-s_points[s.count-1u].y) < END_RADIUS_M)
    {
        s.state = PATH_BRAKING;
        s.speed_mps = s.yaw_rate_dps = 0.0f;
        s.yaw_target_deg = yaw;
        s_brake_ticks = 0;
        s_stall_ticks = 0;
        return;
    }
    if (s.progress_m > s_progress_seen + 0.02f)
    {
        s_progress_seen = s.progress_m;
        s_stall_ticks = 0;
    }
    else if (++s_stall_ticks >= 400u) { fail(PATH_STALLED); return; }

    target_s = clamp(s.progress_m + LOOKAHEAD_M + 0.35f*fabsf(speed), 0.0f, s.length_m);
    look = i;
    for (n = 0; n < 24u && look+2u < s.count && s_points[look+1u].s < target_s; n++) look++;
    t = clamp((target_s-s_points[look].s)/(s_points[look+1u].s-s_points[look].s), 0.0f, 1.0f);
    tx = s_points[look].x + t*(s_points[look+1u].x-s_points[look].x);
    ty = s_points[look].y + t*(s_points[look+1u].y-s_points[look].y);
    error = wrap(atan2f(tx-s.x, ty-s.y)*DEG - s.yaw_deg);
    permission = clamp(permission, 0.25f, 1.0f);
    target_speed = s_cruise * permission * clamp(1.0f-fabsf(error)/100.0f, 0.20f, 1.0f);
    /* Cached curvature/braking envelope; no whole-route scans in the ISR. */
    target_speed=fminf(target_speed,sqrtf(s_limits[i+1u]*s_limits[i+1u]+0.80f*
        fmaxf(0.0f,s_points[i+1u].s-s.progress_m-0.15f*fabsf(speed))));
    /* Decelerate before the endpoint; existing speed PID performs braking. */
    target_speed = fminf(target_speed, sqrtf(2.0f*PATH_DECEL_MPS2*fmaxf(remaining-END_RADIUS_M*0.5f, 0.0f)));
    /* Large heading errors require repositioning, never a stop-and-spin. */
    if (fabsf(error) > 100.0f) { fail(PATH_OFF_ROUTE); return; }
    s.speed_mps = target_speed;
    /* Absolute geometric heading, with shortest-angle and slew constraints.
     * Do not integrate the measured heading error into a drifting setpoint. */
    rate = clamp(wrap(yaw + error - s.yaw_target_deg) / 0.020f,
                 -PATH_YAW_RATE_LIMIT*permission, PATH_YAW_RATE_LIMIT*permission);
    s.yaw_rate_dps = rate;
    s.yaw_target_deg += rate * 0.020f;
}

void path_nav_tick(float ds, float yaw, float speed, float permission)
{
    float mid, rel;
    if (!path_nav_busy()) return;
    if (!finite_value(ds) || fabsf(ds) > 0.05f || !finite_value(yaw) ||
        !finite_value(speed) || !finite_value(permission)) { fail(PATH_INPUT); return; }
    if (fabsf(yaw-s_last_yaw) > 10.0f) { fail(PATH_INPUT); return; }
    mid = (s_last_yaw-s_yaw_zero + 0.5f*(yaw-s_last_yaw))*RAD;
    s.x += ds*sinf(mid);
    s.y += ds*cosf(mid);
    s_last_yaw = yaw;
    rel = yaw-s_yaw_zero;
    s.yaw_deg = rel;
    s_reverse = fmaxf(0.0f, s_reverse-ds);
    if (s_reverse > 0.15f) { fail(PATH_REVERSE); return; }
    if (path_nav_recording())
    {
        float chord = distance(s.x-s_points[s.count-1u].x, s.y-s_points[s.count-1u].y);
        float turn = 0.0f;
        if (s.count >= 2u)
            turn = wrap(rel-atan2f(s_points[s.count-1u].x-s_points[s.count-2u].x,
                                  s_points[s.count-1u].y-s_points[s.count-2u].y)*DEG);
        if (chord >= PATH_SAMPLE_M || (chord >= 0.05f && fabsf(turn) >= 5.0f)) append_point();
        return;
    }
    if (++s_tick >= 4u)
    {
        s_tick = 0;
        plan(yaw, speed, permission);
    }
}

#ifndef PATH_HOST_TEST
#pragma section all restore
#endif
