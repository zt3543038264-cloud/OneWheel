"""Run production menu callbacks inside a delayed WiFi transaction."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root / 'code/menu.c').read_text(encoding='utf-8')

def function(signature):
    start = source.index(signature)
    end = source.index('{', start) + 1
    depth = 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end] + '\n'

prefix = r'''
#include <assert.h>
#include <stdio.h>
#include "menu_tree.h"
typedef uint8_t uint8; typedef uint32_t uint32; typedef int16_t int16;
typedef enum { KEY_1, KEY_2, KEY_3, KEY_4 } key_index_enum;
typedef enum { KEY_RELEASE, KEY_SHORT_PRESS, KEY_LONG_PRESS } key_state_enum;
static key_state_enum states[4];
static Menu_Item *pointer;
static float steps[]={.01f,.1f,1,10,100};
static uint8_t step_index=2;
static bool wifi_connecting;
static Menu_Item *wifi_deferred_item;
static bool wifi_deferred_value;
static bool start_switch,stop_switch,imu_switch,wheel_switch,wifi_switch,save_switch,load_switch;
static int running,preview,refresh_divider,run_mode;
static int imu_wifi_status,ready,in_driver,scenario,cancels,starts,stops,saves,network_calls,draws;
static int MODULE_STM0;
static uint32 ticks,auto_wifi_per_ms=1,auto_wifi_last_announce;
static const char *message;
static void (*progress)(void),(*runtime_progress)(void);
static int runtime_polls;
static void Menu_Request_Redraw(void) {}
static void Menu_Show(void) { ++draws; }
static void Menu_Switch(void);
static void ips200_show_string(int x,int y,const char *s) {(void)x;(void)y;(void)s;}
static key_state_enum key_get_state(key_index_enum key) {return states[key];}
static void key_consume_state(key_index_enum key,key_state_enum event) {
    assert(states[key]==event);states[key]=KEY_RELEASE;
}
static void start_car(void) {assert(!in_driver && !wifi_connecting);++starts;running=1;}
static void stop_car(void) {assert(!in_driver && !wifi_connecting);++stops;running=preview=0;}
static void imu_preview(void) {assert(!in_driver);preview=2;}
static void wheel_pwm_test(void) {assert(!in_driver);preview=3;}
static void save_params(void) {assert(!in_driver && !wifi_connecting);++saves;}
static void load_params(void) {assert(!in_driver);}
static void wifispi_cancel_init(void) {++cancels;imu_wifi_status=-6;}
static int wifispi_is_ready(void) {return ready;}
static void wifispi_set_progress_hook(void (*p)(void)) {progress=p;}
static void wifispi_telemetry_stop(void) {assert(!in_driver);ready=0;}
static void wifispi_telemetry_dual_profile(void) {}
static void wifispi_telemetry_roll_profile(void) {}
static void wifispi_telemetry_startup_profile(void) {}
static void wifispi_telemetry_hold_profile(void) {}
static void wifispi_telemetry_announce(void) {assert(!in_driver);}
static uint32 IfxStm_getLower(int *stm) {(void)stm;return ticks;}
static void wifispi_telemetry_init(void);
static void wifi_spi_clear_cancel_wait(void) {}
static void wifi_spi_set_wait_callback(void (*p)(void)) {runtime_progress=p;}
static void wifispi_telemetry_service(void) {
    runtime_polls++; in_driver=1;assert(runtime_progress && wifi_connecting);
    states[KEY_1]=KEY_SHORT_PRESS;runtime_progress();
    assert(wifi_deferred_item==pointer);in_driver=0;
}

'''

parts = ''.join(function(signature) for signature in (
    'static void wifi_connect_progress(void)',
    'static void wifi_runtime_progress(void)',
    'static void menu_wifi_io_begin(void)',
    'static void menu_wifi_io_end(void)',
    'static void menu_wifi_poll(void)',
    'static void menu_wifi_announce(void)',
    'static void menu_wifi_connect(void)',
    'static void menu_auto_wifi_service(void)',
    'static bool Menu_Handle_Control_Bool(Menu_Item *item, bool value)',
    'static void menu_apply_deferred_control(void)',
    'static bool Menu_Handle_Control_Uint8(Menu_Item *item, int16 delta)',
))
parts += source[source.index('void Key_Up(void)'):source.index('void Menu_Show(void)')]

main = r'''
static void wifispi_telemetry_init(void) {
    ++network_calls;in_driver=1;
    assert(wifi_connecting && progress && states[KEY_1]==KEY_RELEASE);
    if (scenario==0) {
        /* The driver yields several times while waiting for module INT. */
        states[KEY_2]=KEY_SHORT_PRESS;progress(); /* Navigate to another item. */
        assert(pointer->data==&start_switch);
        states[KEY_3]=KEY_SHORT_PRESS;progress(); /* Select Start. */
        assert(pointer->selected);
        states[KEY_1]=KEY_SHORT_PRESS;progress(); /* Request Start during wait. */
        assert(wifi_deferred_item==pointer && cancels && !starts);
        assert(states[KEY_1]==KEY_RELEASE);
    } else if (scenario==1) {
        states[KEY_1]=KEY_SHORT_PRESS;progress(); /* Edit numeric parameter. */
        assert(*(float *)pointer->data==7.0f);
    } else if (scenario==2) {
        states[KEY_4]=KEY_SHORT_PRESS;progress();
        assert(cancels && !pointer->selected && !wifi_deferred_item);
    } else if (scenario==3) {
        states[KEY_1]=KEY_SHORT_PRESS;progress(); /* Stop during wait. */
        assert(wifi_deferred_item==pointer && !stops);
    } else if (scenario==4) {
        states[KEY_1]=KEY_SHORT_PRESS;progress(); /* Flash write during wait. */
        assert(wifi_deferred_item==pointer && !saves);
    } else {
        /* An outer Connect key must already be consumed before yielding. */
        assert(pointer->data==&wifi_switch && !wifi_deferred_item);
        progress();
        assert(network_calls==1);
    }
    in_driver=0;ready=0;
    if (!cancels) imu_wifi_status=-2;
}

int main(void) {
    Menu_Item root={0};root.kind=MENU_Folder;
    Menu_Tree_Reset();
    Menu_Item *folder=Create_Menu_Folder_dynamic(&root,"Startup");
    uint8_t mode=0;
    Create_Menu_File_dynamic(folder,"Mode",&mode,uint8_Box);
    Create_Menu_File_dynamic(folder,"Start",&start_switch,bool_Box);
    Create_Menu_File_dynamic(folder,"Stop",&stop_switch,bool_Box);
    Create_Menu_File_dynamic(folder,"Save",&save_switch,bool_Box);
    Create_Menu_File_dynamic(folder,"Connect",&wifi_switch,bool_Box);
    float gain=6;
    Create_Menu_File_dynamic(folder,"KP",&gain,float_Box);
    All_Folder_Menu_Init(&root);pointer=folder->First_Son;
    menu_wifi_connect();
    assert(!wifi_connecting && !progress && draws>=3 && !starts);
    ticks=10000;menu_auto_wifi_service(); /* Don't retry before deferred Start. */
    assert(network_calls==1);
    menu_apply_deferred_control();assert(starts==1 && running && !wifi_deferred_item);
    menu_apply_deferred_control();assert(starts==1);
    running=0;scenario=1;pointer=folder->First_Son->Last_Brother;pointer->selected=true;
    menu_wifi_connect();assert(gain==7 && !wifi_deferred_item);
    scenario=2;cancels=0;menu_wifi_connect();assert(!pointer->selected);
    scenario=3;pointer=folder->First_Son->Next_Brother->Next_Brother;pointer->selected=true;
    menu_wifi_connect();assert(!stops);menu_apply_deferred_control();assert(stops==1);
    scenario=4;pointer=pointer->Next_Brother;pointer->selected=true;
    menu_wifi_connect();assert(!saves);menu_apply_deferred_control();assert(saves==1);
    scenario=5;network_calls=cancels=0;pointer=pointer->Next_Brother;pointer->selected=true;
    states[KEY_1]=KEY_SHORT_PRESS;Menu_Switch();
    assert(network_calls==1 && !wifi_connecting && !wifi_deferred_item && !progress);
    pointer=folder->First_Son->Next_Brother->Next_Brother;pointer->selected=true;
    int old_stops=stops;ready=0;imu_wifi_status=0;menu_wifi_poll();assert(!runtime_polls);
    ready=1;imu_wifi_status=-4;menu_wifi_poll();assert(!runtime_polls);
    imu_wifi_status=1;menu_wifi_poll();assert(runtime_polls==1);
    assert(stops==old_stops && !runtime_progress && !wifi_connecting);
    menu_wifi_poll(); /* no second IO before pending Stop */
    menu_apply_deferred_control();assert(stops==old_stops+1);
    ready=1;menu_wifi_announce();assert(!runtime_progress && !wifi_connecting);
    Menu_Item mode_item={0};mode_item.data=&run_mode;
    running=1;run_mode=5;int prior_stops=stops;
    Menu_Handle_Control_Uint8(&mode_item,1);
    assert(run_mode==6 && !running && stops==prior_stops+1);
    Menu_Handle_Control_Uint8(&mode_item,1);assert(run_mode==7);Menu_Handle_Control_Uint8(&mode_item,1);assert(run_mode==8);
    assert(run_mode>=8);
    Menu_Handle_Control_Uint8(&mode_item,1);assert(run_mode==10);
    Menu_Handle_Control_Uint8(&mode_item,1);assert(run_mode==10);
    Menu_Handle_Control_Uint8(&mode_item,-1);assert(run_mode==8);
    assert(run_mode>=8);
    assert(run_mode>=8);
    Menu_Handle_Control_Uint8(&mode_item,-1);assert(run_mode==7);Menu_Handle_Control_Uint8(&mode_item,1);assert(run_mode==8);
    wifi_connecting=true;Menu_Handle_Control_Uint8(&mode_item,-1);assert(run_mode==8);wifi_connecting=false;
    assert(run_mode>=8);
    puts("PASS: navigation/editing during WiFi wait; deferred Start/Stop/Flash; cancel; no retry before action; initiating key consumed before callbacks");
}
'''

with tempfile.TemporaryDirectory(prefix='qcar-menu-wifi-') as directory:
    work = Path(directory)
    test = work / 'test.c'
    exe = work / 'test.exe'
    test.write_text(prefix + parts + main, encoding='utf-8')
    subprocess.run(['gcc', '-std=c99', '-Wall', '-Wextra', '-Werror',
                    '-I'+str(root/'code'), str(root/'code/menu_tree.c'), str(test),
                    '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True, timeout=5)
