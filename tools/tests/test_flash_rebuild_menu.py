from pathlib import Path
import tempfile,subprocess
ROOT=Path(__file__).resolve().parents[2]
s=(ROOT/'code/Menu/menu.c').read_text();a=s.index('static void rebuild_flash_action(void)');b=s.index('static void save_params_action(void)',a)
source=r'''
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
typedef uint32_t uint32;typedef int32_t int32;
#define UI_RESET_CONFIRM_MS 3000u
static uint32 g_control_uptime_ms=100,g_param_revision,s_rebuild_armed_until_ms,s_rebuild_revision;
static int s_enter_event_long,busy,ready=1,active,calls;static char status[64];
static int save_blocked(void){return busy;}static int control_path_storage_ready(void){return ready;}
static int control_c_bench_running(void){return active;}static int control_run_running(void){return active;}static int control_remote_running(void){return active;}
static int param_rebuild_current(void){calls++;return 1;}
static void menu_status(const char*s){strcpy(status,s);}
'''+s[a:b]+r'''
int main(void){
rebuild_flash_action();assert(!calls&&s_rebuild_armed_until_ms);
s_enter_event_long=1;rebuild_flash_action();assert(!calls);s_enter_event_long=0;
rebuild_flash_action();assert(calls==1&&!s_rebuild_armed_until_ms);
rebuild_flash_action();g_param_revision++;rebuild_flash_action();assert(calls==1);
busy=1;rebuild_flash_action();assert(calls==1&&!s_rebuild_armed_until_ms);busy=0;
rebuild_flash_action();g_control_uptime_ms+=3001;rebuild_flash_action();assert(calls==1);
ready=0;rebuild_flash_action();assert(calls==1&&!s_rebuild_armed_until_ms);ready=1;
rebuild_flash_action();active=1;rebuild_flash_action();assert(calls==1&&!s_rebuild_armed_until_ms);
puts("PASS actual rebuild UI double-confirmation, hold, expiry, parameter revision and busy guard");return 0;}
'''
assert 'if (back) s_rebuild_armed_until_ms = 0;' in s
assert 'if (up || down) { s_reset_armed_until_ms = 0; s_rebuild_armed_until_ms = 0; }' in s
with tempfile.TemporaryDirectory() as td:
 p=Path(td);(p/'test.c').write_text(source);subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror',str(p/'test.c'),'-o',str(p/'test')],check=True);subprocess.run([str(p/'test')],check=True)
