/* Included by menu.c: CPU0 foreground only, uses the existing menu/key helpers.
 * Each point/status copy masks interrupts for a few assignments, never while
 * drawing or scanning the full route. Route append never rewrites old points. */
#pragma section all "cpu0_dsram"
static float s_path_menu_speed = PATH_SPEED_DEFAULT;
static uint32 s_path_armed_until;
static uint8 s_path_armed_action = 0xffu;
static uint16 s_path_view_index;
#pragma section all restore

static const char *path_state_text(path_state_t state)
{
    switch (state) {
    case PATH_RECORDING: return "RECORDING";
    case PATH_READY: return "READY";
    case PATH_RUNNING: return "RUNNING";
    case PATH_BRAKING: return "BRAKING";
    case PATH_DONE: return "DONE - BALANCE";
    case PATH_ERROR: return "ERROR";
    default: return "EMPTY / TOO SHORT";
    }
}
static const char *path_reason_text(path_reason_t reason)
{
    switch (reason) {
    case PATH_FULL: return "FULL: PREFIX KEPT";
    case PATH_REVERSE: return "REVERSE > 0.15m";
    case PATH_INPUT: return "IMU / ODOM INVALID";
    case PATH_OFF_ROUTE: return "ROUTE / HEADING ERROR";
    case PATH_STALLED: return "NO PROGRESS / BRAKE TIMEOUT";
    case PATH_ABORTED: return "MANUAL STOP";
    default: return "FORWARD ROUTE ONLY";
    }
}

static void render_path(void)
{
    path_status_t st;
    char line[30];
    control_path_status(&st);
    ui_header("Path", s_cursor, 10u);
    draw_action_row(0, 0, "View Route");
    draw_action_row(1, 1, st.state == PATH_RECORDING ? "Finish Recording" : "New Push Record");
    draw_action_row(2, 2, control_remote_running() ? "Stop Teaching" : "New Remote Teach");
    draw_action_row(3, 3, control_path_active() ? "Stop Path / Balance" : "Run Path");
    (void)snprintf(line, sizeof(line), "Speed %.*f m/s%s", (int)Menu_Edit_Decimals(Menu_Edit_Step(Menu_Tree_Child(s_folder,4),.05f,0,0),2), (double)s_path_menu_speed, s_editing ? " *" : "");
    draw_action_row(4, 4, line);
    draw_action_row(5, 5, "Save Path");
    draw_action_row(6, 6, "Load Path");
    draw_action_row(7, 7, "Clear RAM Path");
    draw_action_row(8, 8, "Lock Start Heading");
    draw_action_row(9, 9, "Back");
    ui_line(183, path_state_text(st.state), st.state == PATH_ERROR ? UI_RED : UI_GREEN);
    (void)snprintf(line, sizeof(line), "Points %u/%u  %s", (unsigned)st.count,
                   (unsigned)PATH_MAX_POINTS, st.dirty ? "RAM*" : "SAVED");
    ui_line(203, line, UI_CYAN);
    (void)snprintf(line, sizeof(line), "%.2f / %.2f m", (double)st.progress_m, (double)st.length_m);
    ui_line(223, line, UI_CYAN);
    ui_line(245, path_reason_text(st.reason), UI_YELLOW);
    if (st.heading_locked)
        (void)snprintf(line, sizeof(line), "Yaw ref %+.1f deg", (double)st.heading_reference_deg);
    else (void)snprintf(line, sizeof(line), "Yaw ref: FIRST START");
    ui_line(261, line, UI_CYAN);
    draw_status();
    ui_footer();
}

static uint16 path_px(float x, float center, float scale)
{
    return (uint16)constrain_float(120.0f + (x-center)*scale, 10.0f, 230.0f);
}
static uint16 path_py(float y, float center, float scale)
{
    return (uint16)constrain_float(154.0f - (y-center)*scale, 46.0f, 262.0f);
}
static void render_path_map(void)
{
    path_status_t st;
    path_point_t p, prev, selected;
    uint16 i, y;
    float xmin=0.0f, xmax=0.0f, ymin=0.0f, ymax=0.0f, scale, cx, cy;
    char line[30];
    control_path_status(&st);
    ui_header("Path Map", 0, 0);
    ui_line(23, path_state_text(st.state), UI_GREEN);
    for (y=44; y<266u; y++) ips200_draw_line(0, y, 239, y, UI_BLACK);
    xmin = fminf(0.0f, st.x); xmax = fmaxf(0.0f, st.x);
    ymin = fminf(0.0f, st.y); ymax = fmaxf(0.0f, st.y);
    for (i=0; i<st.count; i++)
    {
        if (!control_path_point(i, &p)) break;
        xmin=fminf(xmin,p.x); xmax=fmaxf(xmax,p.x);
        ymin=fminf(ymin,p.y); ymax=fmaxf(ymax,p.y);
    }
    scale=fminf(210.0f/fmaxf(xmax-xmin,0.40f), 206.0f/fmaxf(ymax-ymin,0.40f));
    cx=(xmin+xmax)*0.5f; cy=(ymin+ymax)*0.5f;
    ips200_draw_line(path_px(0,cx,scale),46,path_px(0,cx,scale),262,UI_GRAY);
    ips200_draw_line(10,path_py(0,cy,scale),230,path_py(0,cy,scale),UI_GRAY);
    if (st.count && s_path_view_index >= st.count) s_path_view_index=st.count-1u;
    memset(&selected,0,sizeof(selected));
    memset(&prev,0,sizeof(prev));
    for (i=0; i<st.count; i++)
    {
        if (!control_path_point(i,&p)) break;
        if (i) ips200_draw_line(path_px(prev.x,cx,scale),path_py(prev.y,cy,scale),
                                path_px(p.x,cx,scale),path_py(p.y,cy,scale),
                                i == st.segment+1u ? UI_YELLOW : UI_CYAN);
        if (i == s_path_view_index) selected=p;
        prev=p;
    }
    /* Start: green cross; selected point: yellow; estimated car: red heading. */
    {
        uint16 x=path_px(0,cx,scale), y0=path_py(0,cy,scale);
        ips200_draw_line(x-3u,y0,x+3u,y0,UI_GREEN);
        ips200_draw_line(x,y0-3u,x,y0+3u,UI_GREEN);
        x=path_px(selected.x,cx,scale); y0=path_py(selected.y,cy,scale);
        ips200_draw_line(x-2u,y0-2u,x+2u,y0+2u,UI_YELLOW);
        x=path_px(st.x,cx,scale); y0=path_py(st.y,cy,scale);
        ips200_draw_line(x-2u,y0,x+2u,y0,UI_RED);
        ips200_draw_line(x,y0,path_px(st.x+0.12f*sinf(st.yaw_deg*0.0174532925f),cx,scale),
                                 path_py(st.y+0.12f*cosf(st.yaw_deg*0.0174532925f),cy,scale),UI_RED);
    }
    (void)snprintf(line,sizeof(line),"P%u X%+.2f Y%+.2f",(unsigned)s_path_view_index,
                   (double)selected.x,(double)selected.y);
    ui_line(277,line,UI_WHITE);
    ui_line(302,"ENTER:Controls BACK:Stop",UI_GRAY);
}

static uint8 path_confirm(uint8 action)
{
    uint32 now=g_control_uptime_ms;
    if (s_path_armed_action != action || !s_path_armed_until ||
        (int32)(now-s_path_armed_until) >= 0)
    {
        s_path_armed_action=action;
        s_path_armed_until=now+UI_BALANCE_CONFIRM_MS;
        menu_status(action==3u ? "PRESS AGAIN TO RUN PATH" : "PRESS AGAIN TO REPLACE PATH");
        return 0;
    }
    s_path_armed_until=0;
    return 1;
}

static void handle_path(uint8 up, uint8 down, uint8 enter, uint8 back)
{
    if (back || (!s_editing && enter && s_cursor==9u))
    {
        control_path_record_finish();
        if (start_flag != START_STOP) control_stop();
        s_path_armed_until=0;
        menu_back();
        return;
    }
    if (s_editing)
    {
        if (up) s_path_menu_speed += current_edit_step();
        if (down) s_path_menu_speed -= current_edit_step();
        s_path_menu_speed=constrain_float(s_path_menu_speed,0.05f,PATH_SPEED_MAX);
        if (enter) s_editing=0;
        s_dirty=1;
        return;
    }
    if (up) move_cursor(-1);
    if (down) move_cursor(1);
    if (up || down) s_path_armed_until=0;
    if (!enter) return;
    s_dirty=1;
    if (s_cursor==0u) { s_path_armed_until=0; menu_enter(); return; }
    if ((s_cursor==1u || s_cursor==2u) && path_nav_recording())
    { control_path_record_finish(); menu_status("RECORD ENDED - SAVE IN STOP"); return; }
    if ((s_cursor==3u && start_flag != START_STOP) ||
        (s_cursor==2u && control_remote_running()))
    { control_stop(); menu_status("STOPPED"); return; }
    if (start_flag != START_STOP || control_test_running() ||
        control_jog_running()!=MOTOR_JOG_NONE || path_nav_busy())
    { menu_status("STOP CONTROL FIRST"); return; }
    if (s_cursor==1u || s_cursor==2u)
    {
        if (!path_confirm(s_cursor)) return;
        if (menu_imu_recovery_intercept()) return;
        if (control_path_record_start((uint8)(s_cursor==2u))) menu_status("RECORD START: FORWARD ONLY");
        else menu_status(test_status_text(control_test_last_status()));
    }
    else if (s_cursor==3u)
    {
        path_status_t st;
        control_path_status(&st);
        if (!st.valid || st.length_m < 0.30f) { menu_status("NEED VALID PATH >= 0.30m"); return; }
        if (!control_imu_recovery_needed() && !path_nav_heading_ready(imu_get_angle_yaw())) { menu_status("ALIGN CAR TO LOCKED YAW"); return; }
        if (!path_confirm(3u)) return;
        if (menu_imu_recovery_intercept()) return;
        if (control_path_start(s_path_menu_speed)) menu_status("PATH RUNNING");
        else menu_status(test_status_text(control_test_last_status()));
    }
    else if (s_cursor==4u) s_editing=1;
    else if (s_cursor==5u || s_cursor==6u)
    {
        if (!control_path_storage_ready()) { menu_status("WAIT STOP / RX / BLDC / IPM"); return; }
        if (s_cursor==6u && !path_confirm(6u)) return;
        menu_status((s_cursor==5u ? path_store_save() : path_store_load()) ? "PATH STORED / LOADED" : "NO VALID PATH / FLASH ERROR");
    }
    else if (s_cursor==7u && path_confirm(7u))
    { path_nav_clear(); menu_status("RAM CLEARED - SAVE TO ERASE"); }
    else if (s_cursor==8u)
    {
        s_path_armed_until=0;
        menu_status(control_path_lock_heading() ? "START HEADING LOCKED" : "NEED READY IMU / ROUTE");
    }
}

static void handle_path_map(uint8 up, uint8 down, uint8 enter, uint8 back)
{
    path_status_t st;
    control_path_status(&st);
    if (back)
    {
        control_path_record_finish();
        if (start_flag != START_STOP) control_stop();
        menu_back();
        return;
    }
    if (enter) { menu_back(); return; }
    if (up && s_path_view_index) s_path_view_index--;
    if (down && s_path_view_index+1u < st.count) s_path_view_index++;
    if (up || down) s_dirty=1;
}
