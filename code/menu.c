/* menu.c：菜单与参数调节。创建日期：2025年1月11日；上游作者姓名已损坏。 */
#include "menu.h"
#include "menu_tree.h"
#include "wifispi.h"
#include "IfxStm.h"
#include "wheel_encoder.h"
#include "wheel_hold.h"
char txt[256];


static uint8 wheel_ready, driver_ready, imu_ready;
static bool control_timers_ready;
static void ensure_wheel(void)
{
    if (!wheel_ready) { motor_int(); wheel_ready = 1; }
}

static uint8 ensure_imu(void)
{
    if (!imu_ready) {
        imu_init();
        if (imu_attitude_status==IMU_ATTITUDE_INIT_FAILED) return 0;
        imu_ready = 1;
    }
    return 1;
}



void menu_init(void)
{
    /* Up to 39 characters fit the demo's 240-pixel portrait width. */
    ips200_set_font(IPS200_6X8_FONT);
    gpio_init(QCAR_KEY2, GPI, 1, GPI_PULL_UP);
    gpio_init(QCAR_KEY3, GPI, 1, GPI_PULL_UP);
    gpio_init(QCAR_KEY1, GPI, 1, GPI_PULL_UP);
    gpio_init(QCAR_EC11_KEY, GPI, 1, GPI_PULL_UP);

    Menu_Init();
    Menu_Show();
    /* Load local settings only after the first menu frame is visible. */
    get_flash();
}


/* 树形节点、循环兄弟链表与编辑交互移植自 mcar-mailun/project。
 * 运行业务接入 Qcar；菜单轮询不再进入阻塞调参循环。 */
static Menu_Item Root;
static Menu_Item *pointer;
static float steps[] = APP_MENU_STEPS;
static uint8 step_index = 2, run_mode = APP_DEFAULT_MODE, running, preview;
static uint8 redraw = 1, refresh_divider;
static bool start_switch, stop_switch, imu_switch, save_switch, load_switch, wheel_switch;
static int wheel_test_pwm;
static bool wifi_switch;
static const char *message = "Ready";
static bool wifi_connecting;
static Menu_Item *wifi_deferred_item;
static bool wifi_deferred_value;
static void stop_car(void)
{
    /* CCU60 is clock-gated until Start or IMU initializes it. */
    if (control_timers_ready) {
        pit_disable(CCU60_CH0);
        if (!imu_ready) pit_disable(CCU60_CH1);
    }
    roll_rate_test_stop();
    shacheflag = 0;
    running = preview = 0;
    small_driver_test_left = small_driver_test_right = 0;
    wheel_switch = false;
    if (wheel_ready) motor_m_ctrl(0);
    if (driver_ready) small_driver_set_duty(0, 0);
    clear_all_cascade_out();
    message = "Stopped";
    redraw = 1;
}

static void start_car(void)
{
    stop_car();
    if (run_mode != 8 && !ensure_imu()) { message = "IMU init failed"; return; }
    ensure_wheel();
    if (!driver_ready) { small_driver_uart_init(); driver_ready = 1; }
    shacheflag = 1;
    running = 1;
    if (run_mode != 8) imu_prepare_start();
    if (run_mode == 2) speed_balance_arm();
    if (run_mode == 3) roll_rate_test_arm();
    if (run_mode == 4) ground_rate_test_arm();
    if (run_mode == 5) dual_balance_arm();
    if (run_mode == 6) roll_rate_only_arm();
    if (run_mode == 7) ground_rate_only_arm();
    if (run_mode == 8) pwm_debug_arm();
    if (run_mode == 10) mode10_balance_arm();
    if (wifispi_is_ready()) {
        if (run_mode==2) wifispi_telemetry_hold_profile();
        else if (run_mode==10) wifispi_telemetry_roll_profile();
        else if (run_mode==5 || run_mode==8) wifispi_telemetry_dual_profile();
        else wifispi_telemetry_startup_profile();
    }
    if (imu_ready) pit_init(CCU60_CH1, APP_IMU_PERIOD_US);
    pit_init(CCU60_CH0, APP_CONTROL_TICK_US);
    control_timers_ready = true;
    message = run_mode==8 ? "PWM debug: no tilt protection" : "Running: Startup/Stop";
}

static void save_params(void)
{
    if (running || preview) { message = "Stop before saving"; return; }
    if (imu_ready) pit_disable(CCU60_CH1);
    int saved=write_flash();
    if (imu_ready) { imu_prepare_start(); pit_init(CCU60_CH1,APP_IMU_PERIOD_US); }
    message = saved ? "Parameters saved/verified" : "Save failed/invalid params";
}

static void load_params(void)
{
    if (running || preview) { message = "Stop before loading"; return; }
    get_flash();
    message = "Parameters loaded";
}



static void imu_preview(void)
{
    stop_car();
    if (!ensure_imu()) { message = "IMU init failed"; return; }
    imu_request_recalibration();
    imu_service();
    pit_init(CCU60_CH1, APP_IMU_PERIOD_US);
    control_timers_ready = true;
    preview = 2;
    message = "IMU active";
}

static void wheel_pwm_test(void)
{
    if (run_mode == 2) {
        stop_car();
        message = "Mode2: use Start, not Open loop";
        return;
    }
    if (preview == 3) return;
    stop_car();
    ensure_wheel();
    preview = 3;
    wheel_switch = true;
    message = "Open loop: PWM active";
}

void Menu_Create(void)
{
    Menu_Item *startup=Create_Menu_Folder_dynamic(&Root,"Startup");
    Menu_Item *pid=Create_Menu_Folder_dynamic(&Root,"PID");
    Menu_Item *storage=Create_Menu_Folder_dynamic(&Root,"Storage");
    Menu_Item *debug=Create_Menu_Folder_dynamic(&Root,"PWM debug");
    Create_Menu_File_dynamic(startup,"Mode 2..8",&run_mode,uint8_Box);
    Create_Menu_File_dynamic(startup,"Start",&start_switch,bool_Box);
    Create_Menu_File_dynamic(startup,"Stop",&stop_switch,bool_Box);
    Create_Menu_File_dynamic(startup,"IMU",&imu_switch,bool_Box);
    Create_Menu_File_dynamic(startup,"Open loop",&wheel_switch,bool_Box);
    Create_Menu_File_dynamic(startup,"Wheel PWM",&wheel_test_pwm,int32_Box);
#if QCAR_WIFI_ENABLED
    Create_Menu_File_dynamic(startup,"WiFi connect",&wifi_switch,bool_Box);
#endif
    Menu_Item *fly=Create_Menu_Folder_dynamic(pid,"Gyro test");
    Create_Menu_File_dynamic(fly,"KP",&roll_rate_test_kp,float_Box);
    Create_Menu_File_dynamic(fly,"KI",&roll_rate_test_ki,float_Box);
    Create_Menu_File_dynamic(fly,"PWM limit",&roll_rate_test_limit,float_Box);
    Create_Menu_File_dynamic(fly,"Sign +/-1",&roll_rate_test_sign,float_Box);
    Create_Menu_File_dynamic(fly,"Angle KP",&pitch_balance_kp,float_Box);
    Create_Menu_File_dynamic(fly,"Angle KD",&pitch_balance_kd,float_Box);
    Create_Menu_File_dynamic(fly,"Vel KP",&flywheel_vel_kp,float_Box);
    Create_Menu_File_dynamic(fly,"Vel KD",&flywheel_vel_kd,float_Box);
    Create_Menu_File_dynamic(fly,"Rate limit",&pitch_balance_rate_limit,float_Box);
    Create_Menu_File_dynamic(fly,"Balance Y",&mono_params.balance_angle_y,float_Box);
    Menu_Item *ground=Create_Menu_Folder_dynamic(pid,"Pitch balance");
    Create_Menu_File_dynamic(ground,"KP",&ground_rate_test_kp,float_Box);
    Create_Menu_File_dynamic(ground,"PWM limit",&ground_rate_test_limit,float_Box);
    Create_Menu_File_dynamic(ground,"Sign +/-1",&ground_rate_test_sign,float_Box);
    Create_Menu_File_dynamic(ground,"Angle KP",&ground_angle_kp,float_Box);
    Create_Menu_File_dynamic(ground,"Rate limit",&ground_angle_rate_limit,float_Box);
    Create_Menu_File_dynamic(ground,"Balance X",&mono_params.balance_angle_x,float_Box);
    Menu_Item *speed=Create_Menu_Folder_dynamic(pid,"Speed hold RAM");
    Create_Menu_File_dynamic(speed,"Speed KP",&wheel_speed_kp,float_Box);
    speed->First_Son->step_scale=0.001f;
    Create_Menu_File_dynamic(speed,"Speed KI",&wheel_speed_ki,float_Box);
    speed->First_Son->Next_Brother->step_scale=0.001f;
    Create_Menu_File_dynamic(speed,"Tilt limit deg",&wheel_tilt_limit,float_Box);
    Create_Menu_File_dynamic(speed,"I limit deg",&wheel_i_limit,float_Box);
    Create_Menu_File_dynamic(speed,"Position KP",&wheel_pos_kp,float_Box);
    Create_Menu_File_dynamic(speed,"Speed limit cps",&wheel_speed_limit,float_Box);
    speed->First_Son->Next_Brother->Next_Brother->Next_Brother->Next_Brother->step_scale=0.1f;
    /* 左右无刷开环测试：停车状态下直接给占空比，用于单独排查飞轮通道。
     * 与 WiFi 滑杆 fly_test_left/right 是同一组变量，两处都能改。 */
    Menu_Item *fly_test=Create_Menu_Folder_dynamic(pid,"Flywheel");
    Create_Menu_File_dynamic(fly_test,"Left PWM",&small_driver_test_left,float_Box);
    Create_Menu_File_dynamic(fly_test,"Right PWM",&small_driver_test_right,float_Box);
    Create_Menu_File_dynamic(storage,"Save",&save_switch,bool_Box);
    Create_Menu_File_dynamic(storage,"Load",&load_switch,bool_Box);
    Create_Menu_File_dynamic(debug,"Fly left PWM",(void *)&pwm_debug_fly_left,float_Box);
    Create_Menu_File_dynamic(debug,"Fly right PWM",(void *)&pwm_debug_fly_right,float_Box);
    Create_Menu_File_dynamic(debug,"Wheel PWM",(void *)&pwm_debug_ground,float_Box);
}

void Menu_Init(void)
{
    Menu_Tree_Reset();
    memset(&Root, 0, sizeof(Root));
    Root.name = "Qcar";
    Root.kind = MENU_Folder;
    Menu_Create();
    All_Folder_Menu_Init(&Root);
    pointer = Root.First_Son;
    /* 四个输入启动时按松开处理，防止首轮产生伪按键事件。 */
    key_init(APP_KEY_PERIOD_MS);
    pit_init(CCU61_CH1, APP_KEY_PERIOD_MS*1000u);
    redraw = 1;
    refresh_divider = 4;
}

/* 业务回调对应参考工程的 Menu_Handle_Control_*，菜单创建只使用统一接口。 */
static void Menu_Request_Redraw(void) { redraw = 1; }

static void wifi_connect_progress(void)
{
    /* Navigation and parameter edits remain available during a network wait.
     * Hardware actions are deferred by Menu_Handle_Control_Bool. */
    if (key_get_state(KEY_4) == KEY_SHORT_PRESS)
        wifispi_cancel_init();
    Menu_Switch();
    refresh_divider=4;
    Menu_Show();
}

/* Runtime waits pump navigation too. Actions are deferred until SPI unwinds;
 * ordinary K4/back navigation must not cancel a healthy network connection. */
static void wifi_runtime_progress(void)
{
    Menu_Switch();
    refresh_divider=4;
    Menu_Show();
}
static void menu_wifi_io_begin(void)
{
    wifi_connecting=true;
    wifi_spi_clear_cancel_wait();
    wifi_spi_set_wait_callback(wifi_runtime_progress);
}
static void menu_wifi_io_end(void)
{
    wifi_spi_set_wait_callback(NULL);
    wifi_spi_clear_cancel_wait();
    wifi_connecting=false;
}
static void menu_wifi_poll(void)
{
    if (wifi_deferred_item || !wifispi_is_ready() || imu_wifi_status==-4) return;
    menu_wifi_io_begin();
    wifispi_telemetry_service();
    menu_wifi_io_end();
}
static void menu_wifi_announce(void)
{
    if (wifi_deferred_item || !wifispi_is_ready() || imu_wifi_status==-4) return;
    menu_wifi_io_begin();
    wifispi_telemetry_announce();
    menu_wifi_io_end();
}

static uint32 auto_wifi_per_ms, auto_wifi_last_announce;

static void menu_wifi_connect(void)
{
    message="WiFi connecting; K4 cancels";
    wifi_connecting=true;
    wifispi_set_progress_hook(wifi_connect_progress);
    wifispi_telemetry_init();
    wifispi_set_progress_hook(NULL);
    wifi_connecting=false;
    message=wifispi_is_ready()?"WiFi UDP connected":
            imu_wifi_status==-6?"WiFi connection cancelled":"WiFi failed; retry Connect";
    if (wifispi_is_ready() && !wifi_deferred_item) {
        if (run_mode==2) wifispi_telemetry_hold_profile();
        else if (run_mode==10) wifispi_telemetry_roll_profile();
        else if (run_mode==5 || run_mode==8) wifispi_telemetry_dual_profile();
        else wifispi_telemetry_startup_profile();
        menu_wifi_announce();
    }
    auto_wifi_last_announce=IfxStm_getLower(&MODULE_STM0);
}

void menu_hardware_init(void)
{
    /* Menu remains available; only an explicit Start enables control. */
    stop_car();
    ensure_wheel(); motor_m_ctrl(0);
    if (!driver_ready) { small_driver_uart_init(); driver_ready=1; }
    small_driver_set_duty(0,0);
#if QCAR_BOOT_IMU_INIT
    if (ensure_imu()) {
        pit_init(CCU60_CH1,APP_IMU_PERIOD_US);
        control_timers_ready=true;
    }
#endif
    auto_wifi_per_ms=(uint32)(IfxStm_getFrequency(&MODULE_STM0)/1000.0f);
    message="Ready; Startup/WiFi connect";
}

static void menu_auto_wifi_service(void)
{
    uint32 now=IfxStm_getLower(&MODULE_STM0);
    if (!wifi_deferred_item && wifispi_is_ready() && auto_wifi_per_ms &&
        (uint32)(now-auto_wifi_last_announce)>=auto_wifi_per_ms*APP_WIFI_ANNOUNCE_MS) {
        menu_wifi_announce(); auto_wifi_last_announce=now;
    }
}

static bool Menu_Handle_Control_Bool(Menu_Item *item, bool value)
{
    if (wifi_connecting) {
        /* Unwind the current SPI transaction before starting/stopping a device
         * or saving Flash. The outer menu loop applies the request once. */
        wifi_deferred_item=item;
        wifi_deferred_value=value;
        wifispi_cancel_init();
        return true;
    }
    if (item->data == &start_switch) {
        if (value) start_car(); else stop_car();
        start_switch = running != 0;
    } else if (item->data == &stop_switch) {
        if (value) stop_car();
        stop_switch = false;
    } else if (item->data == &imu_switch) {
        if (value) imu_preview(); else stop_car();
        imu_switch = preview == 2;
    } else if (item->data == &wheel_switch) {
        if (value) wheel_pwm_test(); else stop_car();
        wheel_switch = preview == 3;
    } else if (item->data == &wifi_switch) {
        if (running || preview) message = "Stop before WiFi setup";
        else if (value) {
            menu_wifi_connect();
        } else {
            wifispi_telemetry_stop();
            message = "WiFi disconnected";
        }
        wifi_switch = wifispi_is_ready() != 0;
    } else if (item->data == &save_switch) {
        if (value) save_params();
        save_switch = false;
    } else if (item->data == &load_switch) {
        if (value) load_params();
        load_switch = false;
    } else return false;
    return true;
}

static void menu_apply_deferred_control(void)
{
    Menu_Item *item=wifi_deferred_item;
    bool value=wifi_deferred_value;
    if (!item || wifi_connecting) return;
    wifi_deferred_item=NULL;
    (void)Menu_Handle_Control_Bool(item,value);
}

static bool Menu_Handle_Control_Uint8(Menu_Item *item, int16 delta)
{
    if (item->data == &run_mode) {
        int value = run_mode + delta;
        if (wifi_connecting) {message="Wait for WiFi then change Mode";return true;}
        if (running || preview || value <= 2) stop_car();
        if (value == 9) value = delta >= 0 ? 10 : 8;
        run_mode = (uint8)(value < 2 ? 2 : value > 10 ? 10 : value);
        return true;
    }
    return false;
}

/* 与参考工程一致：未选中时导航，选中时加减；重复确认循环减小步进。 */

/* Copy ISR-owned values before formatting; never hold interrupts off for LCD IO. */
static const char *control_error_text(int32 state,int32 imu_status)
{
    if (state==-4 && control_fault_reason==10) return "ERR -4: Position lost; Stop/Start";
    if (state==-4) return control_fault_reason==8 ? "ERR -4: Encoder stale; Stop/Start" : "ERR -4: Encoder invalid; Stop/Start";
    if (state==-3 && control_fault_reason==9) return "ERR -3: Speed config/target invalid";
    if (state==-2) return "ERR -2: Tilt >30; Stop/Start";
    if (state==-3) return "ERR -3: invalid parameter/PWM";
    if (state==-1) {
        if(control_fault_reason==4) {
            static char stale_text[40];
            snprintf(stale_text,sizeof(stale_text),"ERR -1: IMU stale >=%lums",(unsigned long)BALANCE_IMU_STALE_MS);
            return stale_text;
        }
        if(control_fault_reason==2)return "ERR -1: IMU no sample";
        if(control_fault_reason==3)return "ERR -1: IMU invalid data";
        if(control_fault_imu_status<0)imu_status=control_fault_imu_status;
        if (imu_status==IMU_ATTITUDE_INIT_FAILED) return "ERR -1: IMU init failed";
        if (imu_status==IMU_ATTITUDE_TIMEOUT) return "ERR -1: IMU read timeout";
        if (imu_status==IMU_ATTITUDE_BAD_DT) return "ERR -1: IMU dt too long";
        if (imu_status==IMU_ATTITUDE_FILTER_ERROR) return "ERR -1: IMU filter failed";
        return "ERR -1: IMU stale/no sample";
    }
    return "Control stopped";
}

static void Menu_Show_Attitude(void)
{
    char line[40];
    const char *imu_message = message;
    float roll, pitch, progress, dt;
    int32 status;
    uint32 sequence;
#if QCAR_WIFI_ENABLED
    /* Connection failures must remain visible even while the gyro/IMU view
     * would normally replace the menu's status message. */
    if (pointer->Father && pointer->data == &wifi_switch &&
        !(roll_rate_test_enabled && roll_rate_test_state<0)) {
        snprintf(line, sizeof(line), "WiFi:%d Err:%d Try:%lu          ",
                 imu_wifi_status, imu_wifi_last_error, (unsigned long)imu_wifi_init_attempts);
        ips200_show_string(0, 152, line);
        snprintf(line, sizeof(line), "RC:%d FW:%.11s Tx:%lu    ",
                 imu_wifi_module_result, wifi_spi_version, (unsigned long)imu_wifi_tx_packets);
        ips200_show_string(0, 172, line);
        if (imu_wifi_status == -2 || imu_wifi_status == -3) {
            snprintf(line, sizeof(line), "Cmd:%02lX Fail:%lu Ack:%02lX Len:%lu    ",
                     (unsigned long)wifi_spi_config_command, (unsigned long)wifi_spi_config_error,
                     (unsigned long)wifi_spi_config_reply, (unsigned long)wifi_spi_config_reply_length);
        } else snprintf(line, sizeof(line), "%-39.39s", message);
        ips200_show_string(0, 192, line);
        return;
    }
#endif
    if (run_mode == 2 && !roll_rate_test_enabled) {
        ips200_show_string(0,152,"Mode2: POSITION hold; stopped           ");
        ips200_show_string(0,172,"Hand-turn encoder; Start enables PWM    ");
        ips200_show_string(0,192,"Start resets position reference         ");
        return;
    }
    uint32 irq = interrupt_global_disable();
    roll = imu_roll_deg;
    pitch = imu_pitch_deg;
    progress = imu_calibration_percent;
    dt = imu_sample_dt_ms;
    status = imu_attitude_status;
    sequence = imu_sample_sequence;
    interrupt_global_enable(irq);

    if (imu_ready) {
        if (status == IMU_ATTITUDE_CALIBRATING)
            imu_message = "IMU init OK - keep still";
        else if (status == IMU_ATTITUDE_RUNNING)
            imu_message = "IMU ready - calibration OK";
        else
            imu_message = "IMU error - toggle IMU Off/On";
    }

    if (roll_rate_test_enabled) {
        float rate, output;
        int32 rate_state;
        if (pwm_debug_enabled) {
            float left,right,ground;
            irq=interrupt_global_disable();
            left=(float)(QCAR_FLYWHEEL_LEFT_SIGN * costrained_flywheel_speed_left);
            right=(float)(QCAR_FLYWHEEL_RIGHT_SIGN * -costrained_flywheel_speed_right);
            ground=(float)costrained_wheel_speed;rate_state=roll_rate_test_state;
            interrupt_global_enable(irq);
            snprintf(line,sizeof(line),"PWM L:%5.0f R:%5.0f W:%5.0f          ",left,right,ground);
            ips200_show_string(0,152,line);
            ips200_show_string(0,172,"Mode8: RAW PWM / no PID                 ");
            snprintf(line,sizeof(line),"%-39.39s",rate_state<0 ? control_error_text(rate_state,status) : "No tilt interlock; Startup/Stop");
            ips200_show_string(0,192,line);
            return;
        }
        if (dual_balance_enabled) {
            float pitch_rate,pitch_pwm,speed,target,offset,pitch_target;
            irq=interrupt_global_disable();
            rate=dual_rate[0];pitch_rate=dual_rate[1];
            output=dual_pwm[0];pitch_pwm=dual_pwm[1];rate_state=roll_rate_test_state;
            speed=wheel_speed_cps;target=wheel_target_cps;offset=wheel_tilt_offset;pitch_target=dual_target[1];
            interrupt_global_enable(irq);
            if (wheel_hold_enabled) snprintf(line,sizeof(line),"V:%6.0f T:%6.0f O:%+.2f        ",speed,target,offset);
            else snprintf(line,sizeof(line),"Rate R:%6.1f P:%6.1f             ",rate,pitch_rate);
            ips200_show_string(0,152,line);
            if (wheel_hold_enabled) snprintf(line,sizeof(line),"Pitch:%6.2f Target:%6.2f         ",roll,pitch_target);
            else snprintf(line,sizeof(line),"Roll:%6.1f Pitch:%6.1f           ",pitch,roll);
            ips200_show_string(0,172,line);
            if (rate_state<0)
                snprintf(line,sizeof(line),"%-39.39s",control_error_text(rate_state,status));
            else if (rate_state==1 && wheel_hold_enabled && status!=IMU_ATTITUDE_CALIBRATING)
                snprintf(line,sizeof(line),"WAIT encoder window; motors OFF         ");
            else if (rate_state==1)
                snprintf(line,sizeof(line),"CAL %5.1f%%: keep still               ",progress);
            else
                snprintf(line,sizeof(line),"PWM R:%5.0f P:%5.0f S:%ld       ",output,pitch_pwm,(long)rate_state);
            ips200_show_string(0,192,line);
            return;
        }
        irq = interrupt_global_disable();
        rate = roll_rate_test_actual;
        output = roll_rate_test_pwm;
        rate_state = roll_rate_test_state;
        interrupt_global_enable(irq);
        snprintf(line, sizeof(line), "Rate:%7.2f PWM:%6.0f       ", rate, output);
        ips200_show_string(0, 152, line);
        snprintf(line, sizeof(line), "State:%ld Pitch:%6.1f Roll:%6.1f ", (long)rate_state, roll, pitch);
        ips200_show_string(0, 172, line);
        snprintf(line,sizeof(line),"%-39.39s",rate_state<0 ? control_error_text(rate_state,status) :
                 rate_only_test ? (ground_rate_test_enabled ? "Pitch RATE ONLY: wheel" : "Roll RATE ONLY: flywheel") :
                 ground_rate_test_enabled ? "Pitch angle/rate: wheel" : "Roll angle/rate: flywheel");
        ips200_show_string(0,192,line);
        return;
    }
    snprintf(line, sizeof(line), "Pitch:%7.2f Roll:%7.2f", roll, pitch);
    ips200_show_string(0, 152, line);
    if (!imu_ready)
        snprintf(line, sizeof(line), "IMU:OFF Enable Startup/IMU");
    else if (status == IMU_ATTITUDE_CALIBRATING)
        snprintf(line, sizeof(line), "IMU:CAL %5.1f%% Seq:%lu", progress, (unsigned long)sequence);
    else if (status == IMU_ATTITUDE_RUNNING)
        snprintf(line, sizeof(line), "IMU:RUN Seq:%lu dt:%4.1fms", (unsigned long)sequence, dt);
    else {
        const char *error = status == IMU_ATTITUDE_TIMEOUT ? "TIMEOUT" :
                            status == IMU_ATTITUDE_UNTRUSTED ? "UNTRUSTED" :
                            status == IMU_ATTITUDE_BAD_DT ? "BAD_DT" :
                            status == IMU_ATTITUDE_FILTER_ERROR ? "FILTER_ERR" : "INIT_FAIL";
        snprintf(line, sizeof(line), "IMU:%s Seq:%lu", error, (unsigned long)sequence);
    }
    /* Fixed width clears characters left by a longer previous status. */
    {
        char padded[40];
        snprintf(padded, sizeof(padded), "%-39.39s", line);
        ips200_show_string(0, 172, padded);
        snprintf(padded, sizeof(padded), "%-39.39s", imu_message);
        ips200_show_string(0, 192, padded);
    }
}

static const char *test_mode_name(void)
{
    switch(run_mode) {
    case 2:return "DUAL + POSITION hold";
    case 3:return "Roll angle: flywheel";case 4:return "Pitch angle: wheel";
    case 5:return "DUAL angle: both";case 6:return "Roll RATE ONLY: flywheel";
    case 7:return "Pitch RATE ONLY: wheel";case 8:return "PWM DEBUG: fly + wheel";
    case 10:return "Mode10: Roll inc + Pitch hold";
    default:return "Invalid mode";
    }
}

void Key_Up(void)
{
    Menu_Request_Redraw();
    if (pointer->Last_Brother != NULL)
        pointer = pointer->Last_Brother;
}
void Key_Down(void)
{
    Menu_Request_Redraw();
    if (pointer->Next_Brother != NULL)
        pointer = pointer->Next_Brother;
}
static float menu_edit_step(void)
{
    return steps[step_index]*(pointer->step_scale>0.0f?pointer->step_scale:1.0f);
}
void Key_Plus(void)
{
    Menu_Request_Redraw();
    switch (pointer->kind)
    {
    case int32_Box:
        *(int32_t *)pointer->data += steps[step_index] < 1.0f ? 1 : (int32_t)steps[step_index];
        break;
    case uint32_Box:
        *(uint32_t *)pointer->data += (uint32_t)steps[step_index];
        break;
    case int16_Box:
        *(int16_t *)pointer->data += (int16_t)steps[step_index];
        break;
    case uint16_Box:
        *(uint16_t *)pointer->data += (uint16_t)steps[step_index];
        break;
    case int8_Box:
        *(int8_t *)pointer->data += (int8_t)steps[step_index];
        break;
    case uint8_Box:
        if (!Menu_Handle_Control_Uint8(pointer, (int16)steps[step_index]))
        {
            *(uint8_t *)pointer->data += (uint8_t)steps[step_index];
        }
        break;
    case float_Box:
        *(float *)pointer->data += menu_edit_step();
        break;
    case bool_Box:
        if (!Menu_Handle_Control_Bool(pointer, true))
        {
            *(bool *)pointer->data = true;
        }
        break;
    default:
        break;
    }
}
void Key_Sub(void)
{
    Menu_Request_Redraw();
    switch (pointer->kind)
    {
    case int32_Box:
        *(int32_t *)pointer->data -= steps[step_index] < 1.0f ? 1 : (int32_t)steps[step_index];
        break;
    case uint32_Box:
        *(uint32_t *)pointer->data -= (uint32_t)steps[step_index];
        break;
    case int16_Box:
        *(int16_t *)pointer->data -= (int16_t)steps[step_index];
        break;
    case uint16_Box:
        *(uint16_t *)pointer->data -= (uint16_t)steps[step_index];
        break;
    case int8_Box:
        *(int8_t *)pointer->data -= (int8_t)steps[step_index];
        break;
    case uint8_Box:
        if (!Menu_Handle_Control_Uint8(pointer, -(int16)steps[step_index]))
        {
            *(uint8_t *)pointer->data -= (uint8_t)steps[step_index];
        }
        break;
    case float_Box:
        *(float *)pointer->data -= menu_edit_step();
        break;
    case bool_Box:
        if (!Menu_Handle_Control_Bool(pointer, false))
        {
            *(bool *)pointer->data = false;
        }
        break;
    default:
        break;
    }
}
void Key_Enter(void)
{
    Menu_Request_Redraw();
    if (pointer->kind == MENU_Folder)
    {
        pointer = pointer->First_Son;
        ips200_show_string(0, 0, "                              ");
        ips200_show_string(0, 16, "                              ");
        ips200_show_string(0, 32, "                              ");
        ips200_show_string(0, 48, "                              ");
        ips200_show_string(0, 64, "                              ");
        ips200_show_string(0, 80, "                              ");
        ips200_show_string(0, 96, "                              ");
        ips200_show_string(0, 112, "                              ");
    }
}
void Key_Exit(void)
{
    Menu_Request_Redraw();
    if (pointer->Father->Father != NULL)
    {
        pointer = pointer->Father;
        ips200_show_string(0, 0, "                              ");
        ips200_show_string(0, 16, "                              ");
        ips200_show_string(0, 32, "                              ");
        ips200_show_string(0, 48, "                              ");
        ips200_show_string(0, 64, "                              ");
        ips200_show_string(0, 80, "                              ");
        ips200_show_string(0, 96, "                              ");
        ips200_show_string(0, 112, "                              ");
    }
}
void Key_Select(void)
{
    Menu_Request_Redraw();
    if (pointer->kind != MENU_Folder)
        pointer->selected = 1;
}
void Key_Deselect(void)
{
    Menu_Request_Redraw();
    if (pointer->kind != MENU_Folder)
        pointer->selected = 0;
    /* Qcar parameters are explicitly saved through Storage/Save. */
}
void Key_SetupCtrl_Sub(void)
{
    Menu_Request_Redraw();
    step_index = (step_index - 1 + 5) % 5;
}


void Menu_Switch(void)
{

    key_state_enum k3 = key_get_state(KEY_3);
    key_state_enum k1 = key_get_state(KEY_1);
    key_state_enum k4 = key_get_state(KEY_4);
    key_state_enum k2 = key_get_state(KEY_2);

    if (k1 == KEY_SHORT_PRESS)
    {
        key_consume_state(KEY_1, KEY_SHORT_PRESS);

        if (pointer->selected == false)
            Key_Up();
        else
            Key_Plus();
    }
    else if (k2 == KEY_SHORT_PRESS)
    {
        key_consume_state(KEY_2, KEY_SHORT_PRESS);

        if (pointer->selected == false)
            Key_Down();
        else
            Key_Sub();
    }
    else if (k3 == KEY_SHORT_PRESS)
    {
        key_consume_state(KEY_3, KEY_SHORT_PRESS);



        if (pointer->kind == MENU_Folder)
            Key_Enter();
        else if (pointer->selected == false)
            Key_Select();
        else
            Key_SetupCtrl_Sub();
    }

    else if (k4 == KEY_SHORT_PRESS)
    {
        key_consume_state(KEY_4, KEY_SHORT_PRESS);

        if (pointer->kind != MENU_Folder && pointer->selected == true)
            Key_Deselect();
        else
            Key_Exit();
    }

    /* Consume before callbacks: a WiFi progress callback may poll keys again. */
    if (k1 == KEY_LONG_PRESS) key_consume_state(KEY_1, KEY_LONG_PRESS);
    if (k2 == KEY_LONG_PRESS) key_consume_state(KEY_2, KEY_LONG_PRESS);
    if (k3 == KEY_LONG_PRESS) key_consume_state(KEY_3, KEY_LONG_PRESS);
    if (k4 == KEY_LONG_PRESS) key_consume_state(KEY_4, KEY_LONG_PRESS);
}



void Menu_Show(void)
{
    Menu_Item *item;
    uint8 i, page;
    char line[40];
    if (++refresh_divider < APP_DISPLAY_DIVIDER) return;
    refresh_divider = 0;
    if (redraw) { ips200_clear(); redraw = 0; }
    snprintf(line, sizeof(line), "%-20.20s <%8.5g>", pointer->Father->name, menu_edit_step());
    ips200_show_string(0, 0, line);
        page = (pointer->rank - 1) / 7;
        item = pointer->Father->First_Son;
        for (i = 0; i < page * 7; i++) item = item->Next_Brother;
        for (i = 0; i < 7 && page * 7 + i < pointer->Father->sons; i++) {
            char value[16] = "";
            if (item->kind == MENU_Folder) strcpy(value, ">>");
            else if (item->kind == bool_Box) strcpy(value, *(bool *)item->data ? "On" : "Off");
            else if (item->kind == float_Box) snprintf(value, sizeof(value), "%.6g", *(float *)item->data);
            else if (item->kind == int32_Box) snprintf(value, sizeof(value), "%ld", (long)*(int32_t *)item->data);
            else if (item->kind == uint32_Box) snprintf(value, sizeof(value), "%lu", (unsigned long)*(uint32_t *)item->data);
            else if (item->kind == int16_Box) snprintf(value, sizeof(value), "%d", (int)*(int16_t *)item->data);
            else if (item->kind == uint16_Box) snprintf(value, sizeof(value), "%u", (unsigned)*(uint16_t *)item->data);
            else if (item->kind == int8_Box) snprintf(value, sizeof(value), "%d", (int)*(int8_t *)item->data);
            else if (item->kind == uint8_Box) snprintf(value, sizeof(value), "%u", (unsigned)*(uint8 *)item->data);
            snprintf(line, sizeof(line), "%c%c %-15.15s %12.12s", item == pointer ? '>' : ' ', item->selected ? '*' : ' ', item->name, value);
            ips200_show_string(0, 20 + 16 * i, line);
            item = item->Next_Brother;
        }
    Menu_Show_Attitude();
    ips200_show_string(0, 216, "K1:up/+ K2:down/- K3:OK K4:back");
    snprintf(line,sizeof(line),"WiFi:%d Tx:%lu                  ",imu_wifi_status,(unsigned long)imu_wifi_tx_packets);
    ips200_show_string(0, 232, line);
    snprintf(line,sizeof(line),"M%u %-30.30s",(unsigned)run_mode,test_mode_name());
    ips200_show_string(0,250,line);
    snprintf(line,sizeof(line),"F:%ld IMU:%ld dt:%6.1f            ",(long)control_fault_reason,(long)control_fault_imu_status,control_fault_dt_ms);
    ips200_show_string(0,266,line);
    snprintf(line,sizeof(line),"SPI:%lu NR:%lu Reg:%02lX           ",(unsigned long)imu_spi_timeouts,(unsigned long)imu_not_ready_samples,(unsigned long)imu_last_status_reg);
    ips200_show_string(0,282,line);
    /* Read-only observation stays visible after Stop. */
    {
        int32 delta,position,position_error;
        uint32 seq,errors,irq;
        uint8 valid,position_valid;
        float speed;
        irq=interrupt_global_disable();
        delta=wheel_encoder_delta;position=wheel_position_count;
        seq=wheel_encoder_seq;errors=wheel_encoder_errors;
        valid=wheel_encoder_valid;position_valid=wheel_position_valid;
        speed=wheel_speed_cps;
        position_error=wheel_pos_error;
        interrupt_global_enable(irq);
        snprintf(line,sizeof(line),"Enc d:%ld v:%.0f/s E:%lu",(long)delta,speed,(unsigned long)errors);
        ips200_show_string(0,298,"                                       ");
        ips200_show_string(0,298,line);
        if (wheel_hold_enabled) snprintf(line,sizeof(line),"P:%ld E:%ld OK:%u/%u",(long)position,(long)position_error,(unsigned)valid,(unsigned)position_valid);
        else snprintf(line,sizeof(line),"P:%ld S:%lu OK:%u/%u",(long)position,(unsigned long)seq,(unsigned)valid,(unsigned)position_valid);
        ips200_show_string(0,310,"                                       ");
        ips200_show_string(0,310,line);
    }
}

void menu_poll(void)
{
    menu_apply_deferred_control();
    imu_service();
    start_switch = running != 0;
    imu_switch = preview == 2;
    wheel_switch = preview == 3;
    Menu_Switch();
    /* Clamp the setpoint even while disabled; only open loop owns this output. */
    if (wheel_test_pwm > (int)PWM_DUTY_MAX) wheel_test_pwm = (int)PWM_DUTY_MAX;
    if (wheel_test_pwm < -(int)PWM_DUTY_MAX) wheel_test_pwm = -(int)PWM_DUTY_MAX;
    if (preview == 3) motor_m_ctrl(wheel_test_pwm);
    /* 停车时按滑杆开环驱动左右无刷，用于单独排查飞轮通道；
     * 运行中主循环不参与，控制中断独占驱动。 */
    if (!running && !preview && driver_ready) {
        if (run_mode == 2) small_driver_test_left=small_driver_test_right=0;
        small_driver_set_duty((int16)small_driver_test_left, (int16)small_driver_test_right);
    }
    wifi_switch=wifispi_is_ready() && imu_wifi_status!=-4;
    menu_auto_wifi_service();
    menu_wifi_poll();
    Menu_Show();
    system_delay_ms(APP_POLL_PERIOD_MS);
}
