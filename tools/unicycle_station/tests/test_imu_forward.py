"""Exercise production IMU mapping and Flash migration; host tests are not vehicle tests."""
import re
from test_lean_limits import ROOT, _compile_run, _function


def test_real_imu_mapping_bias_acceleration_and_restart(tmp_path):
    import subprocess, sys
    result=subprocess.run([sys.executable,str(ROOT/'tools/attitude_r10h/run_tests.py')],capture_output=True,text=True)
    assert result.returncode==0,result.stdout+result.stderr


def param_host_source():
    param=re.sub(r'^#include[^\n]*\n','',(ROOT/'code/Common/param.c').read_text(encoding='utf-8'),flags=re.M)
    return r'''
#include "config.h"
#include "Common/param.h"
#include "Common/param.h"
#include "Estimation/Beacon/beacon_config.c"
#include <assert.h>
#include <math.h>
float g_roll_zero,g_pitch_zero;
typedef union {float float_type;uint32 uint32_type;int32 int32_type;} flash_data_union;
#define EEPROM_PAGE_LENGTH 512
static flash_data_union flash_union_buffer[EEPROM_PAGE_LENGTH], storage[EEPROM_PAGE_LENGTH];
static uint8 flash_check(uint32 s,uint32 p) {(void)s;(void)p;return storage[0].uint32_type!=0;}
static void flash_read_page_to_buffer(uint32 s,uint32 p) {(void)s;(void)p;memcpy(flash_union_buffer,storage,sizeof(storage));}
static void flash_erase_page(uint32 s,uint32 p) {(void)s;(void)p;memset(storage,0,sizeof(storage));}
static uint8 flash_write_page_from_buffer(uint32 s,uint32 p) {(void)s;(void)p;memcpy(storage,flash_union_buffer,sizeof(storage));return 0;}
'''+param


def test_real_flash_legacy_migration_group_save_and_direction_roundtrip(tmp_path):
    source=param_host_source()+r'''
static void old_record(unsigned n,const char *name,float v) {
    storage[2+2*n].uint32_type=param_name_key(name,1);storage[3+2*n].float_type=v;
}
static void legacy(void) {
    memset(storage,0,sizeof(storage));storage[0].uint32_type=PARAM_MAGIC_LEGACY;storage[1].uint32_type=3;
    old_record(0,"roll_zero_init",1.25f);old_record(1,"pitch_zero_init",-4.5f);old_record(2,"p_angle_kp",9);
    storage[8].uint32_type=param_crc32_words((const uint32*)storage,8);
}
int main(void) {
    const char *motor[]={"motor_dir_c"}, *direction[]={"imu_forward_dir"}, *roll[]={"roll_zero_init"};
    param_init();assert(g_param.imu_forward_dir==1 && g_param.roll_zero_init==ROLL_ZERO_INIT_DEFAULT);
    legacy();param_init();assert(g_param.imu_forward_dir==1);
    assert(g_param.roll_zero_init==-1.25f && g_param.pitch_zero_init==4.5f && g_param.p_angle_kp==9);
    param_init();assert(g_param.roll_zero_init==-1.25f); /* no implicit write or double flip */
    assert(param_save_names(motor,1));param_init();assert(g_param.roll_zero_init==-1.25f);
    assert(param_save_names(motor,1));param_init();assert(g_param.pitch_zero_init==4.5f);
    assert(!param_set_by_name("imu_forward_dir",0));assert(!param_set_by_name("imu_forward_dir",2));
    assert(!param_set_by_name("imu_forward_dir",NAN));assert(g_param.imu_forward_dir==1);
    assert(param_set_by_name("imu_forward_dir",-1));assert(g_param.roll_zero_init==1.25f && g_param.pitch_zero_init==-4.5f);
    assert(g_roll_zero==1.25f && g_pitch_zero==-4.5f);
    assert(param_set_by_name("imu_forward_dir",-1));assert(g_param.roll_zero_init==1.25f);
    /* Saving an unrelated group must preserve the stored coordinate frame. */
    assert(param_save_names(motor,1));param_init();assert(g_param.imu_forward_dir==1 && g_param.roll_zero_init==-1.25f);
    assert(param_set_by_name("imu_forward_dir",-1));assert(param_save_names(direction,1));param_init();
    assert(g_param.imu_forward_dir==-1 && g_param.roll_zero_init==1.25f && g_param.pitch_zero_init==-4.5f);
    assert(param_set_by_name("imu_forward_dir",1));assert(param_set_by_name("roll_zero_init",-.75f));
    assert(param_save_names(roll,1));param_init();assert(g_param.imu_forward_dir==1 && g_param.roll_zero_init==-.75f && g_param.pitch_zero_init==4.5f);
    assert(param_save());param_init();assert(g_param.roll_zero_init==-.75f && g_param.p_angle_kp==9);
    /* Changing before the first save of a legacy page still stores one coherent frame. */
    legacy();param_init();assert(param_set_by_name("imu_forward_dir",-1));assert(param_save_names(direction,1));
    param_init();assert(g_param.imu_forward_dir==-1 && g_param.roll_zero_init==1.25f);
    storage[8].uint32_type^=1;param_init();assert(g_param.imu_forward_dir==1 && g_param.roll_zero_init==ROLL_ZERO_INIT_DEFAULT);
    assert(param_erase());param_init();assert(g_param.pitch_zero_init==PITCH_ZERO_INIT_DEFAULT);
    return 0;
}
'''
    _compile_run(tmp_path,source)


def test_restart_blocks_real_control_entry_gate(tmp_path):
    control=(ROOT/'code/Controller/control.c').read_text(encoding='utf-8')
    block=re.search(r'typedef enum\s*\{[^}]*CTRL_BLOCK_NONE[^}]*\}\s*control_block_t;',control,re.S).group()
    source=r'''
#include "Controller/control.h"
#include <assert.h>
static uint8 pending;
volatile uint8 g_imu_ok=1;
uint8 control_imu_recovery_busy(void) {return 0;}
static uint8 imu_forward_pending(void) {return pending;}
static uint8 imu_link_lost(void) {return 0;}
static uint8 imu_calib_state(void) {return 1;}
#define IMU_CALIB_OK 1
static uint8 attitude_diverged(void) {return 0;}
static uint8 attitude_converged(void) {return 1;}
'''+block+re.search(r'^static control_block_t control_attitude_block_reason\(void\)\s*\{.*?^\}',control,re.M|re.S).group()+r'''
int main(void) {
    assert(control_attitude_block_reason()==CTRL_BLOCK_NONE);
    pending=1;assert(control_attitude_block_reason()==CTRL_BLOCK_IMU_RESTART);
    return 0;
}
'''
    _compile_run(tmp_path,source)


def test_station_direction_reads_linked_zeros_and_respects_explicit_batch_values():
    from app.core.models import DeviceProfile
    from app.services.engine import StationEngine
    from test_integration import wait_until
    p=DeviceProfile.load(ROOT/'tools/unicycle_station/app/profiles/tc264_unicycle/profile.json')
    e=StationEngine(p);e.auto_record=False
    def idle(): return not e.requests.pending and not e.requests.queue
    try:
        e.start('mock');wait_until(lambda:len(e.params)==len(p.data['mock_parameters']));wait_until(e.stopped)
        with e.transport.lock: e.transport.mode='STOP';e.transport.manual_stopped=True
        assert e.params['imu_forward_dir'].group=='Zero'
        assert not e.permission(e.params['imu_forward_dir'],False)[0]
        e.submit('apply',[('imu_forward_dir',-1)],True)
        wait_until(lambda:e.params['imu_forward_dir'].value==-1 and abs(e.params['roll_zero_init'].value-0.200891557)<1e-5 and idle())
        assert abs(e.params['pitch_zero_init'].value-5.399967004)<1e-5
        assert e.params['roll_zero_init'].ram_dirty
        e.submit('save','Zero');wait_until(lambda:e.params['imu_forward_dir'].flash_state=='已回读验证')
        assert e.transport.saved['pitch_zero_init']==5.399967004
        e.submit('apply',[('roll_zero_init',-1.25),('imu_forward_dir',1)],True)
        wait_until(lambda:e.params['imu_forward_dir'].value==1 and e.params['roll_zero_init'].value==-1.25 and idle())
        assert abs(e.params['pitch_zero_init'].value+5.399967004)<1e-5
    finally:e.close()
