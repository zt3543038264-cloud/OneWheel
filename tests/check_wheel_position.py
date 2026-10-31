"""Real position P -> speed PI: origin, polarity, measured scheduling, continuity."""
from pathlib import Path
import subprocess,tempfile
root=Path(__file__).resolve().parents[1]
test=r'''
#include "wheel_hold.h"
#include "config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static wheel_encoder_sample_t sample;
static void close_to(float a,float b){if(fabsf(a-b)>.00001f)fprintf(stderr,"got %.9g expected %.9g\n",a,b);assert(fabsf(a-b)<.00001f);}
static void reset(int32_t position){
    wheel_hold_defaults();sample.sequence=1;sample.age_ms=0;sample.dt_ms=20;
    sample.position_epoch=1;sample.position_count=position;sample.valid=sample.position_valid=1;sample.speed_cps=0;
    assert(wheel_hold_update(&sample,0)==1);assert(wheel_ref_count==position && !wheel_target_cps);
}
static void next(int32_t position,float milliseconds){
    ++sample.sequence;sample.position_count=position;sample.dt_ms=milliseconds;
    assert(wheel_hold_update(&sample,0)==1);
}
int main(void){
    reset(100);
    for(int i=0;i<4;++i){next(612,20);assert(!wheel_target_cps);}
    assert(wheel_pos_error==-512);
    for(int i=0;i<30;++i){assert(wheel_hold_update(&sample,0)==1);assert(!wheel_target_cps);}
    next(612,20);close_to(wheel_target_cps,-252);close_to(wheel_tilt_offset,.252f);
    assert(wheel_speed_error==-252); /* Right offset asks for negative/left speed. */
    for(int i=0;i<5;++i)next(-412,20);
    close_to(wheel_target_cps,252);close_to(wheel_tilt_offset,-.252f);
    for(int i=0;i<5;++i)next(100100,20);
    close_to(wheel_target_cps,-300);close_to(wheel_tilt_offset,.3f);
    wheel_speed_limit=20;assert(wheel_hold_update(&sample,0)==1);close_to(wheel_target_cps,-20);
    next(100100,20);close_to(wheel_tilt_offset,.02f);
    wheel_pos_kp=0;next(100100,20);assert(!wheel_target_cps && !wheel_tilt_offset);
    wheel_pos_kp=.5f;wheel_speed_limit=0;next(100100,20);assert(!wheel_target_cps);
    reset(100);for(int i=0;i<5;++i)next(108,20);
    assert(wheel_pos_error==-8 && !wheel_target_cps);
    for(int i=0;i<5;++i){next(109,20);}close_to(wheel_target_cps,-.5f);
    reset(100);for(int i=0;i<4;++i){next(612,21);assert(!wheel_target_cps);}
    next(612,21);close_to(wheel_target_cps,-252); /* 105ms of actual windows. */
    wheel_hold_suspend();assert(wheel_ref_count==100 && !wheel_target_cps && !wheel_tilt_offset);
    for(int i=0;i<5;++i){next(612,20);}assert(wheel_ref_count==100);close_to(wheel_target_cps,-252);
    sample.position_epoch=2;assert(wheel_hold_update(&sample,0)==-10);
    assert(!wheel_tilt_offset && !wheel_target_cps && !wheel_feedback_valid && wheel_ref_count==100);
    sample.position_epoch=1;assert(wheel_hold_update(&sample,0)==-10); /* Do not silently move origin. */
    wheel_hold_reset();sample.position_epoch=2;sample.position_count=612;
    assert(wheel_hold_update(&sample,0)==1 && wheel_ref_count==612 && !wheel_pos_error && !wheel_target_cps);
    sample.position_valid=0;assert(wheel_hold_update(&sample,0)==-10);
    reset(INT32_MAX);sample.position_count=INT32_MIN;assert(wheel_hold_update(&sample,0)==-10);
    reset(0);wheel_pos_kp=NAN;assert(wheel_hold_update(&sample,0)==-9);
    reset(0);wheel_speed_limit=2001;assert(wheel_hold_update(&sample,0)==-9);
    reset(0);wheel_speed_limit=-1;assert(wheel_hold_update(&sample,0)==-9);
    reset(0);wheel_speed_kp=wheel_speed_ki=0;
    for(int i=0;i<5;++i)next(1000,20);
    assert(wheel_target_cps==-300 && !wheel_tilt_offset); /* A disabled speed PI cannot execute position requests. */
    wheel_speed_kp=.001f;next(1000,20);close_to(wheel_tilt_offset,.3f);
    wheel_hold_reset();sample.sequence=123;sample.valid=sample.position_valid=0;sample.dt_ms=0;sample.age_ms=0;
    assert(wheel_hold_update(&sample,0)==0 && !wheel_target_cps);
    sample.position_count=0;sample.position_epoch=3;sample.valid=sample.position_valid=1;sample.dt_ms=20;++sample.sequence;
    assert(wheel_hold_update(&sample,0)==1 && !wheel_ref_count && !wheel_pos_error);
    puts("PASS: position origin, +/-polarity, 100ms measured/new-frame scheduling, deadband, velocity cap/live edits, epoch/lost-position latch, restart and speed-PI dependency");
}
'''
with tempfile.TemporaryDirectory(prefix='onewheel-position-') as directory:
    work=Path(directory);(work/'test.c').write_text(test);exe=work/'test.exe'
    subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror','-I'+str(root/'code'),
                    str(root/'code/wheel_hold.c'),str(root/'code/pid.c'),str(work/'test.c'),'-lm','-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True,timeout=10)
