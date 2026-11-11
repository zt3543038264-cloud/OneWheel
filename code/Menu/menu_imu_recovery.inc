/* CPU0-only menu bookkeeping in spare globally addressed DSPR1 data RAM. */
#if defined(__TASKING__)
#pragma section farbss "cpu1_dsram"
#pragma section fardata "cpu1_dsram"
#endif
static uint8 s_menu_imu_seen_busy, s_menu_imu_wait_release;
static control_imu_recovery_t s_menu_imu_last_state = CTRL_IMU_IDLE;
static uint32 s_menu_imu_last_draw, s_menu_imu_release_since;
#if defined(__TASKING__)
#pragma section fardata restore
#pragma section farbss restore
#endif

/* This helper is reached only after an existing two-press start confirmation.
 * A confirmed restart consumes that action; it never falls through to start. */
static uint8 menu_imu_recovery_intercept(void)
{
    if (!control_imu_recovery_needed()) return 0;
    if (control_imu_recovery_request()) {
        s_menu_imu_seen_busy = 1; /* Also observe an immediate hardware-init failure. */
        menu_status("IMU INIT / KEEP STILL");
    }
    else menu_status(imu_forward_pending() ? "SAVE + REBOOT" : "IMU RETRY: STOP FIRST");
    return 1;
}

static void menu_imu_discard_confirmations(void)
{
    uint8 i;
    s_balance_armed_until_ms = s_remote_armed_until_ms = s_run_armed_until_ms = 0;
    s_reset_armed_until_ms = s_path_armed_until = 0;
    s_path_armed_action = 0xffu;
    s_enter_event_long = 0;
    for (i = 0; i < KEY_NUMBER; ++i) {
        key_clear_state((key_index_enum)i);
        s_long_active[i] = 0;
        s_last_repeat_ms[i] = 0;
    }
}

/* Keep the same page/cursor. Suppress queued presses and require all physical
 * keys released for 150 ms after completion (includes scanner debounce).
 * Clearing key events alone does not suppress a still-held ENTER key. */
static uint8 menu_imu_recovery_guard(void)
{
    static const gpio_pin_enum pins[KEY_NUMBER] = KEY_LIST;
    control_imu_recovery_t state = control_imu_recovery_state();
    uint32 now = g_control_uptime_ms;
    uint8 i;
    if (control_imu_recovery_busy()) {
        menu_imu_discard_confirmations();
        if (!s_menu_imu_seen_busy || state != s_menu_imu_last_state ||
            (uint32)(now-s_menu_imu_last_draw) >= UI_LIVE_PERIOD_MS) {
            const char *text = "IMU INIT / KEEP STILL";
            if (state == CTRL_IMU_CALIBRATING) text = "IMU CALIBRATING / KEEP STILL";
            else if (state == CTRL_IMU_CONVERGING) text = "IMU CONVERGING / KEEP STILL";
            menu_status(text);
            render_page();
            ui_line(UI_STATUS_Y, text, UI_YELLOW);
            s_menu_imu_last_draw = now;
        }
        s_menu_imu_seen_busy = 1;
        s_menu_imu_last_state = state;
        return 1;
    }
    if (s_menu_imu_seen_busy) {
        s_menu_imu_seen_busy = 0;
        s_menu_imu_wait_release = 1;
        s_menu_imu_release_since = now;
        menu_imu_discard_confirmations();
        s_balance_on = s_test_on = s_run_on = s_editing = 0;
        s_active_test = 0xffu;
        s_status[0] = '\0';
        if (state == CTRL_IMU_FAILED)
            menu_status(test_status_text(control_test_last_status()));
        s_dirty = 2;
        render_page();              /* Restore the original live menu. */
        return 1;
    }
    if (s_menu_imu_wait_release) {
        menu_imu_discard_confirmations();
        for (i = 0; i < KEY_NUMBER; ++i)
            if (gpio_get_level(pins[i]) != KEY_RELEASE_LEVEL) s_menu_imu_release_since = now;
        if ((uint32)(now-s_menu_imu_release_since) >= 150u) s_menu_imu_wait_release = 0;
        return 1;
    }
    return 0;
}
