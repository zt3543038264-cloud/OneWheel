"""Preserve the confirmed vehicle's physical signs across parameter-format changes."""
import json,re
from test_lean_limits import ROOT,_compile_run,_function
from test_imu_forward import param_host_source


def test_legacy_snapshot_preserved_and_current_schema_roundtrip(tmp_path):
    snapshot=json.loads((ROOT/'tools/unicycle_station/docs/defaults-20260921.json').read_text(encoding='utf-8'))
    rows=[]
    for n,p in snapshot['raw_parameters'].items():
        f=p['type']=='float';v=repr(float(p['value']))+'f'
        rows.append('{"'+n+'",'+str(int(f))+','+v+'}')
    source=param_host_source()+r'''
typedef struct {const char *name;int floating;float old;} sample;
static sample samples[]={'''+','.join(rows)+r'''};
static flash_data_union before[EEPROM_PAGE_LENGTH];
static param_t defaults,expected;
static void prepare(uint32 magic,int reverse) {
    unsigned count=sizeof(samples)/sizeof(samples[0]);
    memset(storage,0,sizeof(storage));storage[0].uint32_type=magic;storage[1].uint32_type=count;
    for(unsigned i=0;i<count;i++) {
        storage[2+2*i].uint32_type=param_name_key(samples[i].name,samples[i].floating);
        if(samples[i].floating) storage[3+2*i].float_type=samples[i].old;
        else storage[3+2*i].int32_type=(int)samples[i].old;
        if(reverse && !samples[i].floating && strstr(samples[i].name,"dir"))
            storage[3+2*i].int32_type=-storage[3+2*i].int32_type;
    }
    storage[2+2*count].uint32_type=param_crc32_words((const uint32*)storage,2+2*count);
}
int main(void) {
    const char *camera[]={"cam_exposure"};
    const uint32 old_magic[]={PARAM_MAGIC_LEGACY,PARAM_MAGIC_MASTER};
    param_load_defaults();defaults=g_param;
    for(unsigned magic=0;magic<2;magic++) for(int reverse=0;reverse<2;reverse++) {
        unsigned writes=storage_writes,erases=storage_erases;
        prepare(old_magic[magic],reverse);memcpy(before,storage,sizeof(storage));
        param_init();assert(param_storage_status()==PARAM_STORAGE_INCOMPATIBLE);
        assert(!memcmp(&g_param,&defaults,sizeof(g_param)));
        assert(!param_save_names(camera,1) && !param_save() && !param_erase());
        param_init();assert(!memcmp(&g_param,&defaults,sizeof(g_param)));
        assert(!memcmp(before,storage,sizeof(storage)) && writes==storage_writes && erases==storage_erases);
    }
    /* Every recognized current parameter, including negative gains/signs,
       round-trips without migration or silent dropping on a partial save. */
    memset(storage,0,sizeof(storage));param_init();
    for(unsigned i=0;i<PARAM_TABLE_NUM;i++) {
        const param_desc_t *d=&g_param_table[i];
        if(d->is_float) *(float *)d->ptr=d->vmin+(d->vmax-d->vmin)*.25f;
        else *(int *)d->ptr=(int)d->vmin;
    }
    g_param.imu_forward_dir=1;expected=g_param;
    assert(param_save());param_init();assert(!memcmp(&g_param,&expected,sizeof(g_param)));
    g_param.cam_exposure=321;expected.cam_exposure=321;
    g_param.r_rate_kp=0;g_param.y_rate_kp=0;g_param.roll_zero_init=0;
    assert(param_save_names(camera,1));param_init();
    assert(!memcmp(&g_param,&expected,sizeof(g_param)));
    assert(param_save_names(camera,1));param_init();assert(!memcmp(&g_param,&expected,sizeof(g_param)));
    assert(param_save());param_init();assert(!memcmp(&g_param,&expected,sizeof(g_param)));
    return 0;
}
'''
    _compile_run(tmp_path,source)


def test_real_gain_refresh_and_pid_outputs_match_signed_baseline(tmp_path):
    control=(ROOT/'code/Controller/control.c').read_text(encoding='utf-8')
    pid=re.sub(r'^#include[^\n]*\n','',(ROOT/'code/Common/pid.c').read_text(encoding='utf-8'),flags=re.M)
    source=r'''
#include "Common/board_config.h"
#include "Common/param.h"
#include "Common/pid.h"
#include <assert.h>
param_t g_param;
static pid_t r_rcy_pid,r_angle_pid,r_rate_pid,p_vel_pid,p_angle_pid,p_rate_pid,y_angle_pid,y_rate_pid;
'''+pid+'\n#include "Controller/control_angle.inc"\n'+_function(control,'cascade_gain_refresh')+r'''
int main(void) {
    pid_t old_roll,old_yaw;
    /* Nonzero I/D also exercise integral hold, output saturation and reversal. */
    g_param.r_rate_kp=20;g_param.r_rate_ki=.2f;g_param.r_rate_kd=8;
    g_param.y_rate_kp=56;g_param.y_rate_ki=.3f;g_param.y_rate_kd=4;
    pid_set(&old_roll,20,.2f,-8,100);pid_set(&r_rate_pid,0,0,0,100);
    pid_set(&old_yaw,-56,-.3f,-4,100);pid_set(&y_rate_pid,0,0,0,100);
    for(int i=0;i<1000;i++) {
        float error=(float)((i*17)%79-39);
        cascade_gain_refresh();
        assert(pid_loc_calc_limited(&old_roll,error,-100,100)==pid_loc_calc_limited(&r_rate_pid,error,-100,100));
        assert(pid_loc_calc_limited(&old_yaw,error,-100,100)==pid_loc_calc_limited(&y_rate_pid,error,-100,100));
        assert(old_roll.integrator==r_rate_pid.integrator && old_yaw.integrator==y_rate_pid.integrator);
    }
    /* All user-facing direction defaults are +1, effective physical signs are unchanged. */
    for(int d=-1;d<=1;d+=2) {
        g_param.motor_dir_a=g_param.motor_dir_b=g_param.motor_dir_c=g_param.enc_dir_c=g_param.steer_dir=g_param.imu_forward_dir=d;
        assert(500*MOTOR_DIR_A==500*d && -800*MOTOR_DIR_B==-800*d);
        assert(500*MOTOR_DIR_C==-500*d && 20*ENC_DIR_C==-20*d);
        assert(30*STEER_DIR==-30*d && IMU_FORWARD_DIR==-d);
    }
    return 0;
}
'''
    _compile_run(tmp_path,source)
