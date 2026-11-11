"""Motor-page encoder directions: real Flash, menu save/guards and UART rules."""
import re

from test_lean_limits import ROOT, _compile_run, _function
from test_imu_forward import param_host_source


def test_motor_page_saves_each_encoder_direction_and_loads_old_flash(tmp_path):
    menu = (ROOT/'code/Menu/menu.c').read_text(encoding='utf-8')
    items = re.search(r'static const menu_param_item_t s_motor_items\[\] =\s*\{.*?^\};', menu, re.M|re.S).group()
    source = param_host_source()+r'''
typedef struct {const char *label,*name;float step;uint8 decimals;} menu_param_item_t;
'''+items+r'''
#define UI_MAX_GROUP_ITEMS 20
typedef struct {const char *title;const menu_param_item_t *items;uint8 count;} menu_group_t;
static const menu_group_t s_groups[]={{"Motor",s_motor_items,sizeof(s_motor_items)/sizeof(s_motor_items[0])}};
static uint8 s_group_index;
static char s_status[80];
static const menu_param_item_t *group_items(const menu_group_t *g){return g->items;}
static uint8 group_item_count(const menu_group_t *g){return g->count;}
static int save_blocked(void){return 0;}
static void menu_status(const char *s){strcpy(s_status,s);}
'''+_function(menu, 'save_group_action')+r'''
static void old_page_without_encoder_keys(void){
 unsigned n=0;
 memset(storage,0,sizeof(storage));storage[0].uint32_type=PARAM_MAGIC;
 for(unsigned i=0;i<PARAM_TABLE_NUM;i++){
  const param_desc_t *d=&g_param_table[i];
  if(!strcmp(d->name,"enc_dir_a")||!strcmp(d->name,"enc_dir_b"))continue;
  storage[2+2*n].uint32_type=param_name_key(d->name,d->is_float);
  if(d->is_float)storage[3+2*n].float_type=*(float*)d->ptr;
  else storage[3+2*n].int32_type=*(int*)d->ptr;
  n++;
 }
 storage[1].uint32_type=n;
 storage[2+2*n].uint32_type=param_crc32_words((const uint32*)storage,2+2*n);
}
int main(void){
 param_load_defaults();g_param.r_rate_kp=27;g_param.roll_zero_init=1.25f;
 old_page_without_encoder_keys();param_init();
 assert(g_param.enc_dir_a==ENC_DIR_A_DEFAULT && g_param.enc_dir_b==ENC_DIR_B_DEFAULT);
 assert(g_param.r_rate_kp==27 && g_param.roll_zero_init==1.25f);
 assert(sizeof(s_motor_items)/sizeof(s_motor_items[0])<=UI_MAX_GROUP_ITEMS);
 unsigned a=0,b=0;
 for(unsigned i=0;i<sizeof(s_motor_items)/sizeof(s_motor_items[0]);i++){
  assert(param_find(s_motor_items[i].name));
  if(!strcmp(s_motor_items[i].name,"enc_dir_a")){a++;assert(s_motor_items[i].step==2);}
  if(!strcmp(s_motor_items[i].name,"enc_dir_b")){b++;assert(s_motor_items[i].step==2);}
 }
 assert(a==1&&b==1);
 assert(!param_set_by_name("enc_dir_a",0));assert(!param_set_by_name("enc_dir_b",.5f));
 assert(!param_set_by_name("enc_dir_a",3));assert(!param_set_by_name("enc_dir_b",NAN));
 for(int ea=-1;ea<=1;ea+=2)for(int eb=-1;eb<=1;eb+=2){
  assert(param_set_by_name("enc_dir_a",ea));assert(param_set_by_name("enc_dir_b",eb));
  int da=g_param.motor_dir_a,db=g_param.motor_dir_b;
  g_param.r_rate_kp=99; // Motor Save must preserve the stored Roll value (27).
  save_group_action();assert(!strcmp(s_status,"Motor PAGE SAVED"));
  param_load_defaults();param_init();
  assert(g_param.enc_dir_a==ea&&g_param.enc_dir_b==eb);
  assert(g_param.motor_dir_a==da&&g_param.motor_dir_b==db);
  assert(g_param.r_rate_kp==27 && g_param.roll_zero_init==1.25f);
 }
 assert(param_save());param_init();assert(g_param.enc_dir_a==1&&g_param.enc_dir_b==1);
 // Saving an unrelated group also preserves both stored encoder directions.
 assert(param_set_by_name("enc_dir_a",-1));save_group_action();
 g_param.enc_dir_a=1;const char *names[]={"cam_exposure"};assert(param_save_names(names,1));
 param_init();assert(g_param.enc_dir_a==-1&&g_param.enc_dir_b==1);
 return 0;
}
'''
    _compile_run(tmp_path, source)


def test_motor_edits_blocked_during_jog_balance_and_axis_tests(tmp_path):
    menu = (ROOT/'code/Menu/menu.c').read_text(encoding='utf-8')
    source = r'''
#include <assert.h>
#include <string.h>
#include <stdint.h>
typedef uint8_t uint8;
enum {GROUP_KIND_AXIS,GROUP_KIND_ZERO,GROUP_KIND_MOTOR,GROUP_KIND_ODOMETRY,GROUP_KIND_BEACON};
enum {TUNE_AXIS_ROLL,TUNE_AXIS_PITCH};
enum {TUNE_RING_RATE,TUNE_RING_ANGLE,TUNE_RING_VEL};
#define START_STOP 0
#define ODOM_TEST_IDLE 0
#define MOTOR_JOG_NONE 0
static int start_flag,g_tune_axis,g_tune_ring,test,jog;
static int path_nav_busy(void){return 0;}static int control_path_active(void){return 0;}
static int control_test_running(void){return test;}static int control_jog_running(void){return jog;}
static int control_odometry_test_state(void){return ODOM_TEST_IDLE;}
typedef struct {const char *name;} menu_param_item_t;
typedef struct {const menu_param_item_t *items;int axis,kind;} menu_group_t;
'''+_function(menu, 'group_param_edit_allowed')+r'''
int main(void){
 menu_param_item_t items[]={{"enc_dir_a"},{"enc_dir_b"}};
 menu_group_t g={items,TUNE_AXIS_ROLL,GROUP_KIND_MOTOR};
 for(unsigned i=0;i<2;i++){
  start_flag=test=jog=0;assert(group_param_edit_allowed(&g,(uint8)i));
  jog=1;assert(!group_param_edit_allowed(&g,(uint8)i));jog=0;
  test=1;assert(!group_param_edit_allowed(&g,(uint8)i));test=0;
  start_flag=1;assert(!group_param_edit_allowed(&g,(uint8)i));
 }
 return 0;
}
'''
    _compile_run(tmp_path, source)


def test_encoder_parameters_use_stopped_motor_protocol_permissions(tmp_path):
    protocol = (ROOT/'code/Protocols/vofa.c').read_text(encoding='utf-8')
    source = r'''
#include <stdint.h>
#include <string.h>
#include <assert.h>
typedef uint8_t uint8;
#define START_STOP 0
#define MOTOR_JOG_NONE 0
static int start_flag,test,jog,run,remote,path,active;
static int control_test_running(void){return test;}
static int control_jog_running(void){return jog;}
static int control_run_running(void){return run;}
static int control_remote_running(void){return remote;}
static int path_nav_busy(void){return path;}
static int control_path_active(void){return active;}
static int recovering;
static int control_imu_recovery_busy(void){return recovering;}
'''
    for name in ('cfg_group','cfg_dangerous','cfg_stopped','cfg_live_control'):
        if name == 'cfg_group':
            source += re.search(r'^static const char \*cfg_group\([^\n]*\)\s*\{.*?^\}',protocol,re.M|re.S).group()+'\n'
        else:
            source += _function(protocol,name)+'\n'
    source += r'''
int main(void){
 assert(!strcmp(cfg_group("enc_dir_a"),"Motor"));assert(!strcmp(cfg_group("enc_dir_b"),"Motor"));
 assert(cfg_dangerous("enc_dir_a")&&cfg_dangerous("enc_dir_b"));
 assert(!cfg_live_control("enc_dir_a")&&!cfg_live_control("enc_dir_b"));
 assert(cfg_stopped());jog=1;assert(!cfg_stopped());jog=0;test=1;assert(!cfg_stopped());
 test=0;start_flag=1;assert(!cfg_stopped());start_flag=0;
 recovering=1;assert(!cfg_stopped());return 0;
}
'''
    _compile_run(tmp_path, source)
