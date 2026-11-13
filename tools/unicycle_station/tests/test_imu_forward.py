"""Exercise production IMU mapping and no-migration Flash safety; host tests are not vehicle tests."""
import re
from test_lean_limits import ROOT, _compile_run, _function


def test_real_imu_mapping_bias_acceleration_and_restart(tmp_path):
    import subprocess, sys
    result=subprocess.run([sys.executable,str(ROOT/'tools/attitude_r10h/run_tests.py')],capture_output=True,text=True)
    assert result.returncode==0,result.stdout+result.stderr


def param_host_source():
    # Compile the complete production parameter implementation, not selected
    # functions from the obsolete migration path. Opt in only in this RAM test.
    param=re.sub(r'^#include[^\n]*\n','',(ROOT/'code/Common/param.c').read_text(encoding='utf-8'),flags=re.M)
    return r'''
#include "Common/board_config.h"
#include "Common/param.h"
#include <assert.h>
#include <math.h>
#define STORAGE_FLASH_VALIDATED 1
#define STORAGE_BOOT_LOAD 1
#define FLASH_STATUS_OK 0
float g_roll_zero,g_pitch_zero;
typedef union {float float_type;uint32 uint32_type;int32 int32_type;} flash_data_union;
#define EEPROM_PAGE_LENGTH 1024
static flash_data_union flash_union_buffer[EEPROM_PAGE_LENGTH], storage[EEPROM_PAGE_LENGTH];
static unsigned storage_writes,storage_erases;
static int flash_last_status(void) {return FLASH_STATUS_OK;}
static uint8 flash_check(uint32 s,uint32 p) {(void)s;(void)p;for(unsigned i=0;i<EEPROM_PAGE_LENGTH;i++)if(storage[i].uint32_type)return 1;return 0;}
static void flash_read_page_to_buffer(uint32 s,uint32 p) {(void)s;(void)p;memcpy(flash_union_buffer,storage,sizeof(storage));}
static void flash_erase_page(uint32 s,uint32 p) {(void)s;(void)p;++storage_erases;memset(storage,0,sizeof(storage));}
static uint8 flash_write_page_from_buffer(uint32 s,uint32 p) {(void)s;(void)p;++storage_writes;memcpy(storage,flash_union_buffer,sizeof(storage));return 0;}
static void flash_buffer_clear(void) {memset(flash_union_buffer,0,sizeof(flash_union_buffer));}
'''+param


def test_real_flash_no_migration_group_save_and_fixed_frame_roundtrip(tmp_path):
    source=param_host_source()+r'''
static flash_data_union before[EEPROM_PAGE_LENGTH];
static void old_record(unsigned n,const char *name,float v) {
    storage[2+2*n].uint32_type=param_name_key(name,1);storage[3+2*n].float_type=v;
}
static void legacy(uint32 magic) {
    memset(storage,0,sizeof(storage));storage[0].uint32_type=magic;storage[1].uint32_type=3;
    old_record(0,"roll_zero_init",1.25f);old_record(1,"pitch_zero_init",-4.5f);old_record(2,"p_angle_kp",9);
    storage[8].uint32_type=param_crc32_words((const uint32*)storage,8);
}
int main(void) {
    const char *motor[]={"motor_dir_c"}, *roll[]={"roll_zero_init"};
    const uint32 old_magic[]={PARAM_MAGIC_LEGACY,PARAM_MAGIC_MASTER};
    for(unsigned i=0;i<2;i++) {
        unsigned writes=storage_writes,erases=storage_erases;
        legacy(old_magic[i]);memcpy(before,storage,sizeof(storage));
        param_init();assert(param_storage_status()==PARAM_STORAGE_INCOMPATIBLE);
        assert(g_param.imu_forward_dir==1 && g_param.roll_zero_init==ROLL_ZERO_INIT_DEFAULT);
        assert(g_param.pitch_zero_init==PITCH_ZERO_INIT_DEFAULT && g_param.p_angle_kp==P_ANGLE_KP_DEFAULT);
        assert(!param_save_names(motor,1) && !param_save() && !param_erase());
        param_init();assert(param_storage_status()==PARAM_STORAGE_INCOMPATIBLE);
        assert(!memcmp(before,storage,sizeof(storage)) && writes==storage_writes && erases==storage_erases);
    }
    /* A blank test fixture is independent of the rejected legacy sector. */
    memset(storage,0,sizeof(storage));param_init();
    g_param.roll_zero_init=-1.25f;g_param.pitch_zero_init=4.5f;g_param.p_angle_kp=9;
    assert(param_save());param_init();assert(g_param.roll_zero_init==-1.25f && g_param.pitch_zero_init==4.5f);
    assert(g_param.p_angle_kp==9 && storage[0].uint32_type==PARAM_MAGIC);
    g_param.roll_zero_init=2;g_param.pitch_zero_init=-3;g_param.p_angle_kp=11;
    assert(param_save_names(motor,1));param_init();
    assert(g_param.roll_zero_init==-1.25f && g_param.pitch_zero_init==4.5f && g_param.p_angle_kp==9);
    assert(!param_set_by_name("imu_forward_dir",0));assert(!param_set_by_name("imu_forward_dir",2));
    assert(!param_set_by_name("imu_forward_dir",NAN));assert(!param_set_by_name("imu_forward_dir",-1));
    assert(param_set_by_name("imu_forward_dir",1));
    assert(g_param.imu_forward_dir==1 && g_param.roll_zero_init==-1.25f && g_param.pitch_zero_init==4.5f);
    g_param.roll_zero_init=-.75f;g_param.pitch_zero_init=2.5f;
    assert(param_save_names(roll,1));param_init();
    assert(g_param.imu_forward_dir==1 && g_param.roll_zero_init==-.75f && g_param.pitch_zero_init==2.5f && g_param.p_angle_kp==9);
    assert(param_save());param_init();assert(g_param.roll_zero_init==-.75f && g_param.p_angle_kp==9);
    assert(param_erase());param_init();assert(param_storage_status()==PARAM_STORAGE_EMPTY && g_param.pitch_zero_init==PITCH_ZERO_INIT_DEFAULT);
    assert(param_save());storage[2+2*storage[1].uint32_type].uint32_type^=1;
    memcpy(before,storage,sizeof(storage));
    {unsigned writes=storage_writes,erases=storage_erases;
     param_init();assert(param_storage_status()==PARAM_STORAGE_CORRUPT);
     assert(g_param.roll_zero_init==ROLL_ZERO_INIT_DEFAULT && !param_save() && !param_erase());
     assert(!memcmp(before,storage,sizeof(storage)) && writes==storage_writes && erases==storage_erases);}
    return 0;
}
'''
    _compile_run(tmp_path,source)


def test_review_lock_precedes_restart_and_ready_control_entry(tmp_path):
    control=(ROOT/'code/Controller/control.c').read_text(encoding='utf-8')
    block=re.search(r'typedef enum\s*\{[^}]*CTRL_BLOCK_NONE[^}]*\}\s*control_block_t;',control,re.S).group()
    source=r'''
#include "Controller/control.h"
#include "Common/vehicle_profile.h"
#include <assert.h>
static uint8 pending;
uint8 g_imu_ok=1;
static uint8 imu_forward_pending(void) {return pending;}
static uint8 imu_link_lost(void) {return 0;}
static uint8 imu_calib_state(void) {return 1;}
#define IMU_CALIB_OK 1
static uint8 attitude_diverged(void) {return 0;}
static uint8 attitude_converged(void) {return 1;}
'''+block+re.search(r'^static control_block_t control_attitude_block_reason\(void\)\s*\{.*?^\}',control,re.M|re.S).group()+r'''
int main(void) {
    assert(VEHICLE_MOTOR_OUTPUT_ALLOWED==0);
    assert(control_attitude_block_reason()==CTRL_BLOCK_ADAPT_LOCKED);
    pending=1;assert(control_attitude_block_reason()==CTRL_BLOCK_ADAPT_LOCKED);
    return 0;
}
'''
    _compile_run(tmp_path,source)


def test_station_fixed_frame_rejects_change_and_noop_preserves_zeros_revision():
    from app.core.models import DeviceProfile
    from app.services.engine import StationEngine
    from test_integration import wait_until
    p=DeviceProfile.load(ROOT/'tools/unicycle_station/app/profiles/tc264_unicycle/profile.json')
    e=StationEngine(p);e.auto_record=False
    def idle(): return not e.requests.pending and not e.requests.queue
    try:
        e.start('mock');wait_until(lambda:len(e.params)==len(p.data['mock_parameters']));wait_until(e.stopped)
        with e.transport.lock: e.transport.mode='STOP';e.transport.manual_stopped=True
        names=('imu_forward_dir','roll_zero_init','pitch_zero_init')
        before={name:e.params[name].value for name in names}
        revision=e.transport.revision
        # A rejected frame write aborts before any explicit zero in the batch.
        e.submit('apply',[('roll_zero_init',-1.25),('imu_forward_dir',-1)],True)
        wait_until(lambda:len(e.batch_results)==1 and idle())
        assert not e.batch_results[0][1]
        assert 'FIXED_FRAME' in e.batch_results[0][2]
        assert {name:e.params[name].value for name in names}==before
        assert e.transport.revision==revision
        assert all(not e.params[name].ram_dirty for name in names)
        e.batch_results=[]
        e.submit('apply',[('imu_forward_dir',before['imu_forward_dir'])],True)
        wait_until(lambda:len(e.batch_results)==1 and idle())
        assert e.batch_results[0][1]
        assert e.batch_results[0][2][-1]=='FIXED_FRAME'
        assert {name:e.params[name].value for name in names}==before
        assert e.transport.revision==revision
        assert all(not e.params[name].ram_dirty for name in names)
    finally:e.close()
