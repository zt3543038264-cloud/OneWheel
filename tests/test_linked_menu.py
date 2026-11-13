from pathlib import Path
import subprocess,re,hashlib
from menu_render_contract import RENDER_SHA256,DISPLAY_SHA256
ROOT=Path(__file__).resolve().parents[1]
def function(s,sig):
 a=s.index(sig);b=s.index('{',a);depth=1;j=b+1
 while depth:depth+=(s[j]=='{')-(s[j]=='}');j+=1
 return s[a:j]+'\n'
def test_same_draw_functions_font_and_image_renderer():
 s=(ROOT/'code/Menu/menu.c').read_text(encoding='utf-8')
 for name,wanted in RENDER_SHA256.items():
  sig=re.search(r'^static (?:void|const char \*)\s*'+name+r'\(',s,re.M).group()
  body=function(s,sig).rstrip('\n').replace('imu_fault_text()', '"IMU  LINK LOST"' if name=='render_main_live' else '"IMU LINK LOST"')
  assert hashlib.sha256(body.encode()).hexdigest()==wanted,name
 assert 'ips200_set_font(IPS200_8X16_FONT)' in function(s,'void menu_init(void)')
 assert hashlib.sha256((ROOT/'code/Menu/display.c').read_bytes().replace(b'\r\n',b'\n')).hexdigest()==DISPLAY_SHA256

def test_actual_hierarchy_navigation_callbacks_and_exhaustion(tmp_path):
 s=(ROOT/'code/Menu/menu.c').read_text(encoding='utf-8')
 prefix=s[s.index('#define UI_BLACK'):s.index('static void ui_text(')]
 # Strip only comments immediately before the first helper, keep all real group descriptors/state.
 c='#include <assert.h>\n#include <string.h>\n#include "Controller/control.h"\n#include "Protocols/vofa.h"\n#include "Estimation/Vision/vision_core.h"\n#include "Menu/menu_tree.h"\n#define KEY_NUMBER 4\n'+prefix
 c+='\nvolatile vofa_mode_t g_vofa_mode; uint8 g_vision_ipm_ok; static int stops,runs;\n'
 c+='static void stop_local_test(void){stops++;}\nstatic void save_params_action(void){}\nstatic void reset_params_action(void){}\nstatic void run_action(void){runs++;}\nstatic void menu_status(const char *s){(void)s;}\n'
 c+='uint8 control_camera_debug_start(void){return 1;}\nvision_core_state_t vision_core_state(void){return VISION_CORE_OFF;}\nvoid display_init(void){}\nvoid display_image_page_enter(void){}\n'
 c+='static const char *test_status_text(control_test_status_t x){(void)x;return "fault";}\n'
 for name in ['render_main','render_params','render_attitude','render_group','render_run_test','render_path','render_path_map']:c+='static void '+name+'(void){}\n'
 for name in ['handle_attitude','handle_group','handle_run_test','handle_image','handle_ipm','handle_path','handle_path_map']:c+='static void '+name+'(uint8 a,uint8 b,uint8 c,uint8 d){(void)a;(void)b;(void)c;(void)d;}\n'
 c+='static const char *action_name(uint8 n){(void)n;return "action";}\n'
 for sig in ['static uint8 current_count(void)','static void save_page_position(void)','static void load_page_position(menu_page_t page)','static void navigate_to(Menu_Item *folder)','static void menu_enter(void)','static void menu_back(void)','static void move_cursor(int direction)','static void handle_main(uint8 up, uint8 down, uint8 enter)','static void handle_params(uint8 up, uint8 down, uint8 enter, uint8 back)','static void handle_main_keys(uint8 up,uint8 down,uint8 enter,uint8 back)','static Menu_Item *add_folder(','static int menu_tree_build(void)']:
  # Forward declarations for builder/enter occur earlier, select the definition for no-arg helpers.
  if sig in ['static void menu_enter(void)','static void menu_back(void)','static int menu_tree_build(void)']:
   match=re.search(re.escape(sig)+r'\s*\{',s);c+=function(s[match.start():],sig)
  else:c+=function(s,sig)
 c+=r'''
int main(void){
 assert(menu_tree_build());assert(s_folder==s_root&&s_root->sons==5);assert(Menu_Tree_Used()<MENU_TREE_CAPACITY);
 move_cursor(-1);assert(s_cursor==4);move_cursor(1);assert(s_cursor==0);
 handle_main(0,0,1);assert(s_page==MENU_PAGE_PARAMS&&s_folder->Father==s_root&&current_count()==PARAM_PAGE_COUNT);
 for(unsigned i=0;i<GROUP_COUNT;i++){
  s_folder->cursor=Menu_Tree_Child(s_folder,i+1);load_page_position(s_page);handle_params(0,0,1,0);
  assert(s_page==MENU_PAGE_GROUP&&s_group_index==i);assert(current_count()==s_groups[i].item_count+s_groups[i].action_count+2);
  assert(s_folder->cursor->data==&s_groups[i].items[0]);
  move_cursor(-1);assert(s_cursor==current_count()-1);menu_back();assert(s_page==MENU_PAGE_PARAMS&&s_cursor==i+1);
  menu_enter();assert(s_cursor==current_count()-1);move_cursor(1);assert(s_cursor==0);menu_back();
 }
 s_folder->cursor=Menu_Tree_Child(s_folder,0);load_page_position(s_page);handle_params(0,0,1,0);assert(s_page==MENU_PAGE_ATTITUDE);menu_back();
 menu_back();move_cursor(1);handle_main(0,0,1);assert(s_page==MENU_PAGE_IMAGE);menu_back();assert(s_cursor==1);
 move_cursor(1);handle_main(0,0,1);assert(runs==1&&s_folder==s_root);
 move_cursor(1);handle_main(0,0,1);assert(s_page==MENU_PAGE_RUN_TEST&&current_count()==3);menu_back();
 move_cursor(1);handle_main(0,0,1);assert(s_page==MENU_PAGE_PATH);menu_enter();assert(s_page==MENU_PAGE_PATH_MAP);menu_back();menu_back();assert(s_folder==s_root&&s_cursor==4);
 s_root->cursor=Menu_Tree_Child(s_root,0);load_page_position(s_page);menu_enter();
 s_folder->cursor=Menu_Tree_Child(s_folder,6);load_page_position(s_page);menu_enter();assert(s_groups[s_group_index].kind==GROUP_KIND_CAMERA);
 s_folder->cursor=Menu_Tree_Child(s_folder,s_groups[s_group_index].item_count);load_page_position(s_page);menu_enter();assert(s_page==MENU_PAGE_IPM);menu_back();assert(s_page==MENU_PAGE_GROUP);
 assert(stops>20);Menu_Tree_Reset();Menu_Item *root=Menu_Tree_Add(0,"root",MENU_Folder,0,0,0);assert(root);
 for(unsigned i=1;i<MENU_TREE_CAPACITY;i++)assert(Menu_Tree_Add(root,"item",MENU_Action,0,0,0));assert(!Menu_Tree_Add(root,"overflow",MENU_Action,0,0,0));
 return 0;}
'''
 (tmp_path/'tree.c').write_text(c,encoding='utf-8');exe=tmp_path/'tree.exe'
 r=subprocess.run(['gcc','-std=c99','-DMT9V03X_W=188','-DMT9V03X_H=120','-I'+str(ROOT/'tools/unicycle_station/tests/firmware_host'),'-I'+str(ROOT/'code'),str(tmp_path/'tree.c'),str(ROOT/'code/Menu/menu_tree.c'),'-lm','-o',str(exe)],capture_output=True,text=True,encoding="utf-8",errors="replace");assert r.returncode==0,r.stderr
 r=subprocess.run([str(exe)],capture_output=True,text=True,encoding="utf-8",errors="replace",timeout=8);assert r.returncode==0,r.stdout+r.stderr
