"""Use the ACTUAL public macro with production serialization/boot loader.
Unlike logical-page mocks, this memory boundary preserves physical 8-byte pages.
"""
from pathlib import Path
import subprocess,tempfile
root=Path(__file__).resolve().parents[1]
header=(root/'code/flash.h').read_text()
macro=next(line for line in header.splitlines() if line.startswith('#define my_flash_read') or line.startswith('#define     my_flash_read'))
source=(root/'code/flash.c').read_text()
payload=source[source.index('#define BALANCE_STORAGE_MAGIC'):]
# Reuse declarations/default values, but NOT its logical-page read macro.
other=(root/'tests/check_storage.py').read_text()
prefix=other.split("prefix=r'''",1)[1].split("'''",1)[0]
prefix=prefix.replace('static uint32 memory[16];','static uint32 memory[32];')
prefix=prefix.replace('#define my_flash_read(sector,page,type) memory[page]', '#define QCAR_EEPROM_BASE_ADDR ((uintptr_t)memory)\n#define QCAR_EEPROM_SECTOR_SIZE 8192u\n#define QCAR_EEPROM_PAGE_SIZE 8u\n'+macro)
prefix=prefix.replace('memory[p]=*v','memory[p*2u]=*v')
main=r'''
int main(void){
 unsigned i,sector=1;
 for(i=0;i<15;i++) {
   assert((uintptr_t)&my_flash_read(0,i,uint32)==(uintptr_t)memory+i*8u);
   assert((uintptr_t)&my_flash_read(0,i+1,uint32)==(uintptr_t)memory+(i+1)*8u);
   assert((uintptr_t)&my_flash_read(0,1+i,uint32)==(uintptr_t)memory+(i+1)*8u);
   assert((uintptr_t)&my_flash_read(sector+1,i+1,uint32)==(uintptr_t)memory+(sector+1)*8192u+(i+1)*8u);
 }
 balance_control_defaults();roll_rate_test_kp=47.25f;roll_rate_test_limit=1234;
 mono_params.balance_angle_x=2.5f;mono_params.balance_angle_y=-1.25f;
 control_startup_ramp_ms=777;
 assert(write_flash());
 assert(memory[0]==BALANCE_STORAGE_MAGIC);
 /* Every second word is unused, not the next stored parameter. */
 for(i=0;i<15;i++)assert(memory[i*2+1]==0);
 roll_rate_test_kp=1;roll_rate_test_limit=1;control_startup_ramp_ms=1;
 get_flash();
 assert(roll_rate_test_kp==47.25f && roll_rate_test_limit==1234);
 assert(mono_params.balance_angle_x==2.5f && mono_params.balance_angle_y==-1.25f);
 assert(control_startup_ramp_ms==777);
 memory[14*2]^=1;get_flash();assert(roll_rate_test_kp==FLY_RATE_KP);
 puts("Flash actual macro: expression addressing, physical 8-byte layout, production Save/boot roundtrip PASS");
}
'''
with tempfile.TemporaryDirectory(prefix='onewheel-flash-address-') as d:
 p=Path(d);c=p/'test.c';exe=p/'test';c.write_text(prefix+payload+main)
 subprocess.run(['gcc','-std=c99','-I'+str(root/'code'),str(c),'-o',str(exe)],check=True)
 subprocess.run([str(exe)],check=True)
