from pathlib import Path
import subprocess,shutil
from test_linked_menu import function
ROOT=Path(__file__).resolve().parents[1]
def run_c(tmp_path,text,extra=()):
 p=tmp_path/'test.c';p.write_text(text);exe=tmp_path/'test.exe'
 r=subprocess.run(['gcc','-std=c99','-I'+str(tmp_path),str(p),*[str(x) for x in extra],'-o',str(exe)],capture_output=True,text=True);assert r.returncode==0,r.stderr
 r=subprocess.run([str(exe)],capture_output=True,text=True,timeout=5);assert r.returncode==0,r.stdout+r.stderr

def test_real_scanner_enter_300ms_other_keys_1000ms_and_release(tmp_path):
 (tmp_path/'zf_device_key.h').write_text(r'''#include <stdint.h>
#include <assert.h>
typedef uint8_t uint8;typedef uint32_t uint32;typedef int gpio_pin_enum;
typedef enum {KEY_1,KEY_2,KEY_3,KEY_4,KEY_NUMBER} key_index_enum;
typedef enum {KEY_RELEASE,KEY_SHORT_PRESS,KEY_LONG_PRESS} key_state_enum;
#define KEY_LIST {0,1,2,3}
#define KEY_RELEASE_LEVEL 1
#define KEY_MAX_SHOCK_PERIOD 10
#define KEY_LONG_PRESS_PERIOD 1000
#define GPIO_HIGH 1
#define GPI 0
#define GPI_PULL_UP 0
#define zf_assert assert
int gpio_get_level(int);void gpio_init(int,int,int,int);
void key_init(uint32);void key_scanner(void);key_state_enum key_get_state(key_index_enum);void key_clear_state(key_index_enum);
uint8 key_set_long_press_period(key_index_enum,uint32);
''')
 (tmp_path/'zf_common_debug.h').write_text('#include <assert.h>\n#define zf_assert assert\n')
 shutil.copy2(ROOT/'libraries/zf_device/zf_device_key.c',tmp_path/'key.c')
 c=r'''#include "zf_device_key.h"
static int level[4]={1,1,1,1};
int gpio_get_level(int n){return level[n];}void gpio_init(int a,int b,int c,int d){(void)a;(void)b;(void)c;(void)d;}
int main(void){
 key_init(10);assert(key_set_long_press_period(KEY_3,300));level[2]=0;
 for(int i=0;i<29;i++)key_scanner();assert(key_get_state(KEY_3)==KEY_RELEASE);
 key_scanner();assert(key_get_state(KEY_3)==KEY_LONG_PRESS);
 for(int i=0;i<100;i++)key_scanner();assert(key_get_state(KEY_3)==KEY_LONG_PRESS);
 level[2]=1;key_scanner();assert(key_get_state(KEY_3)==KEY_RELEASE);
 level[2]=0;for(int i=0;i<10;i++)key_scanner();level[2]=1;key_scanner();assert(key_get_state(KEY_3)==KEY_SHORT_PRESS);key_clear_state(KEY_3);
 level[0]=0;for(int i=0;i<99;i++)key_scanner();assert(key_get_state(KEY_1)==KEY_RELEASE);key_scanner();assert(key_get_state(KEY_1)==KEY_LONG_PRESS);
 level[0]=1;key_scanner();assert(key_set_long_press_period(KEY_3,1000));level[2]=0;
 for(int i=0;i<99;i++)key_scanner();assert(key_get_state(KEY_3)==KEY_RELEASE);key_scanner();assert(key_get_state(KEY_3)==KEY_LONG_PRESS);
 level[2]=1;key_scanner();key_init(7);assert(key_set_long_press_period(KEY_3,300));level[2]=0;
 for(int i=0;i<42;i++)key_scanner();assert(key_get_state(KEY_3)==KEY_RELEASE);key_scanner();assert(key_get_state(KEY_3)==KEY_LONG_PRESS);
 assert(!key_set_long_press_period(KEY_NUMBER,300));assert(!key_set_long_press_period(KEY_3,0));return 0;}
'''
 run_c(tmp_path,c,[tmp_path/'key.c'])

def test_editing_only_override_cached_and_restored(tmp_path):
 s=(ROOT/'code/Menu/menu.c').read_text(encoding='utf-8')
 c=r'''#include <stdint.h>
#include <assert.h>
typedef uint8_t uint8;typedef uint32_t uint32;
#define MENU_KEY_ENTER 2
#define UI_STEP_HOLD_MS 300
#define KEY_LONG_PRESS_PERIOD 1000
enum {MENU_PAGE_GROUP,MENU_PAGE_PATH,MENU_PAGE_MAIN};
static uint8 s_editing;static int s_page,calls;static uint32 s_enter_hold_period,last;
static uint8 key_set_long_press_period(int k,uint32 p){assert(k==2);calls++;last=p;return 1;}
'''+function(s,'static void update_edit_hold_period(void)')+r'''
int main(void){
 s_page=MENU_PAGE_GROUP;update_edit_hold_period();assert(calls==1&&last==1000);
 update_edit_hold_period();assert(calls==1);s_editing=1;update_edit_hold_period();assert(calls==2&&last==300);
 s_page=MENU_PAGE_PATH;update_edit_hold_period();assert(calls==2);s_editing=0;update_edit_hold_period();assert(calls==3&&last==1000);
 return 0;}
'''
 run_c(tmp_path,c)
