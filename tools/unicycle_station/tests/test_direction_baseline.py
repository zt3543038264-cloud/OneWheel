"""Preserve the confirmed vehicle's physical signs across parameter-format changes."""
import json,re
from test_lean_limits import ROOT,_compile_run,_function
from test_imu_forward import param_host_source


def test_latest_complete_flash_snapshot_roundtrip_and_partial_save(tmp_path):
    snapshot=json.loads((ROOT/'tools/unicycle_station/docs/defaults-20260921.json').read_text(encoding='utf-8'))
    rows=[]
    for n,p in snapshot['raw_parameters'].items():
        f=p['type']=='float';v=repr(float(p['value']))+'f'
        expected = 0.0 if n == 'r_rate_kd' else float(snapshot['parameters'][n]['value'])
        rows.append('{"'+n+'",'+str(int(f))+','+v+','+repr(expected)+'f}')
    source=param_host_source()+r'''
typedef struct {const char *name;int floating;float old,expected;} sample;
static sample samples[]={'''+','.join(rows)+r'''};
static void prepare(void) {
    memset(storage,0,sizeof(storage));storage[0].uint32_type=PARAM_MAGIC_LEGACY;storage[1].uint32_type=75;
    for(unsigned i=0;i<75;i++) {
        storage[2+2*i].uint32_type=param_name_key(samples[i].name,samples[i].floating);
        if(samples[i].floating) storage[3+2*i].float_type=samples[i].old;
        else storage[3+2*i].int32_type=(int)samples[i].old;
    }
    storage[152].uint32_type=param_crc32_words((const uint32*)storage,152);
}
static void check(void) {
    for(unsigned i=0;i<75;i++) {float v;assert(param_get_by_name(samples[i].name,&v));assert(v==samples[i].expected);}
    assert(IMU_FORWARD_DIR==-1 && MOTOR_DIR_A==1 && MOTOR_DIR_B==1);
    assert(MOTOR_DIR_C==-1 && ENC_DIR_C==-1 && STEER_DIR==-1);
    assert(R_RATE_KP==20 && R_RATE_KD==0 && Y_RATE_KP==56);
}
int main(void) {
    const char *camera[]={"cam_exposure"};
    prepare();param_init();check();assert(storage[0].uint32_type==PARAM_MAGIC_LEGACY);
    param_init();check(); /* loading does not persist or double negate */
    assert(param_save_names(camera,1));assert(storage[0].uint32_type==PARAM_MAGIC);param_init();check();
    assert(param_save_names(camera,1));param_init();check();
    assert(param_save());param_init();check();
    /* A deliberately reversed old setting must remain reversed, never abs(). */
    prepare();
    for(unsigned i=0;i<75;i++) {
        if(!samples[i].floating && strstr(samples[i].name,"dir")) storage[3+2*i].int32_type=-storage[3+2*i].int32_type;
        if(!strcmp(samples[i].name,"y_rate_kp")) storage[3+2*i].float_type=13;
    }
    storage[152].uint32_type=param_crc32_words((const uint32*)storage,152);
    param_init();assert(g_param.motor_dir_a==-1 && g_param.motor_dir_c==-1 && g_param.enc_dir_c==-1);
    assert(g_param.imu_forward_dir==-1 && g_param.steer_dir==-1);
    assert(g_param.y_rate_kp==13 && Y_RATE_KP==13);
    assert(g_param.roll_zero_init==-.6f && g_param.pitch_zero_init==-2.999999f);
    assert(param_save_names(camera,1));param_init();assert(Y_RATE_KP==13 && IMU_FORWARD_DIR==1);
    return 0;
}
'''
    _compile_run(tmp_path,source)


def test_real_gain_refresh_and_pid_outputs_match_signed_baseline(tmp_path):
    control=(ROOT/'code/Controller/control.c').read_text(encoding='utf-8')
    pid=re.sub(r'^#include[^\n]*\n','',(ROOT/'code/Common/pid.c').read_text(encoding='utf-8'),flags=re.M)
    source=r'''
#include "config.h"
#include "Common/param.h"
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
    pid_set(&old_roll,20,.2f,0,100);pid_set(&r_rate_pid,0,0,0,100);
    pid_set(&old_yaw,56,.3f,4,100);pid_set(&y_rate_pid,0,0,0,100);
    for(int i=0;i<1000;i++) {
        float error=(float)((i*17)%79-39);
        cascade_gain_refresh();
        assert(pid_inc_pi_calc_limited(&old_roll,error,-100,100)==pid_inc_pi_calc_limited(&r_rate_pid,error,-100,100));
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
