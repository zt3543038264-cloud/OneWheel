from pathlib import Path
import subprocess
ROOT=Path(__file__).resolve().parents[1]
def test_real_step_scales_precision_integer_and_independent_nodes(tmp_path):
 c=r'''#include "Menu/menu_tree.h"
#include <math.h>
#include <assert.h>
int main(void){
 Menu_Tree_Reset();Menu_Item *r=Menu_Tree_Add(0,"root",MENU_Folder,0,0,0);
 Menu_Item *a=Menu_Tree_Add(r,"Ki",MENU_Parameter,0,0,0),*b=Menu_Tree_Add(r,"Kp",MENU_Parameter,0,0,0);
 assert(fabsf(Menu_Edit_Step(a,.01f,0,0)-.01f)<1e-8f);
 float expected[]={.1f,1,.0001f,.001f,.01f};
 for(int i=0;i<5;i++){assert(Menu_Edit_Cycle(a,.01f,0,0));assert(fabsf(Menu_Edit_Step(a,.01f,0,0)-expected[i])<1e-7f);}
 assert(b->step_index==2);assert(Menu_Edit_Cycle(a,.01f,0,0));assert(b->step_index==2);
 for(int i=0;i<20;i++){float step=Menu_Edit_Step(b,1,1,0);assert(step==1||step==10||step==100);assert(Menu_Edit_Cycle(b,1,1,0));}
 assert(Menu_Edit_Step(a,2,1,1)==2);unsigned old=a->step_index;assert(!Menu_Edit_Cycle(a,2,1,1)&&a->step_index==old);
 assert(Menu_Edit_Decimals(.0001f,3)==4);assert(Menu_Edit_Decimals(.000001f,4)==6);assert(Menu_Edit_Decimals(.25f,0)==2);assert(Menu_Edit_Decimals(.05f*.01f,2)==4);
 assert(Menu_Edit_Decimals(100,3)==3);assert(!Menu_Edit_Cycle(a,NAN,0,0));assert(Menu_Edit_Step(a,0,0,0)==0);
 Menu_Tree_Reset();r=Menu_Tree_Add(0,"root",MENU_Folder,0,0,0);a=Menu_Tree_Add(r,"Ki",MENU_Parameter,0,0,0);assert(a->step_index==2);
 return 0;}
'''
 (tmp_path/'step.c').write_text(c);exe=tmp_path/'step.exe'
 x=subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror','-I'+str(ROOT/'code'),str(tmp_path/'step.c'),str(ROOT/'code/Menu/menu_tree.c'),'-lm','-o',str(exe)],capture_output=True,text=True,encoding='utf-8',errors='replace');assert x.returncode==0,x.stderr
 x=subprocess.run([str(exe)],capture_output=True,text=True,timeout=5);assert x.returncode==0,x.stdout+x.stderr

def test_hold_enter_cycles_once_and_short_enter_still_finishes(tmp_path):
 from test_linked_menu import function
 s=(ROOT/'code/Menu/menu.c').read_text(encoding='utf-8')
 c=r'''#include <stdint.h>
#include <assert.h>
typedef uint8_t uint8;typedef uint32_t uint32;typedef int key_index_enum;
typedef enum {KEY_RELEASE,KEY_SHORT_PRESS,KEY_LONG_PRESS} key_state_enum;
enum {MENU_PAGE_MAIN,MENU_PAGE_GROUP,MENU_PAGE_PATH};
#define UI_LONG_REPEAT_MS 120
#define MENU_KEY_ENTER 0
static key_state_enum state;static uint8 s_long_active[4],s_editing,s_enter_event_long;static uint32 s_last_repeat_ms[4],g_control_uptime_ms;static int s_page,cycles;
static key_state_enum key_get_state(int n){(void)n;return state;}
static void key_clear_state(int n){(void)n;state=KEY_RELEASE;}
static void cycle_edit_step(void){cycles++;}
'''
 c+=function(s,'static uint8 key_event(key_index_enum key, uint8 repeat)')+function(s,'static uint8 editing_enter(uint8 enter,key_state_enum state)')
 c+=r'''
int main(void){
 s_page=MENU_PAGE_GROUP;s_editing=1;state=KEY_LONG_PRESS;
 assert(!editing_enter(key_event(0,0),KEY_LONG_PRESS)&&cycles==1);
 for(int i=0;i<20;i++){g_control_uptime_ms+=200;assert(!editing_enter(key_event(0,0),KEY_LONG_PRESS));}assert(cycles==1);
 state=KEY_RELEASE;key_event(0,0);state=KEY_LONG_PRESS;assert(!editing_enter(key_event(0,0),KEY_LONG_PRESS)&&cycles==2);
 state=KEY_RELEASE;key_event(0,0);state=KEY_SHORT_PRESS;assert(editing_enter(key_event(0,0),KEY_SHORT_PRESS)==1&&cycles==2);
 s_editing=0;state=KEY_RELEASE;key_event(0,0);state=KEY_LONG_PRESS;assert(editing_enter(key_event(0,0),KEY_LONG_PRESS)==1&&cycles==2);
 s_editing=1;s_page=MENU_PAGE_PATH;state=KEY_RELEASE;key_event(0,0);state=KEY_LONG_PRESS;assert(!editing_enter(key_event(0,0),KEY_LONG_PRESS)&&cycles==3);
 return 0;}
'''
 (tmp_path/'key.c').write_text(c);exe=tmp_path/'key.exe'
 r=subprocess.run(['gcc','-std=c99',str(tmp_path/'key.c'),'-o',str(exe)],capture_output=True,text=True);assert r.returncode==0,r.stderr
 r=subprocess.run([str(exe)],capture_output=True,text=True,timeout=5);assert r.returncode==0,r.stdout+r.stderr
