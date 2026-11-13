#include "Controller/Planner/path_nav.h"
#include "Controller/Planner/path_store.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static path_point_t route[PATH_MAX_POINTS], decoded[PATH_MAX_POINTS];
static uint32_t words[PATH_RECORD_WORDS];
static uint32_t flash_pages[12][1024];
static int flash_ready=1, write_limit=-1;
int control_path_storage_ready(void) { return flash_ready; }
int flash_check(unsigned sec,unsigned page)
{
    unsigned i; (void)sec; assert(page==9 || page==10);
    for(i=0;i<1024;i++) if(flash_pages[page][i]) return 1;
    return 0;
}
void flash_read_page(unsigned sec,unsigned page,uint32_t *out,uint16_t count)
{ (void)sec; assert(page==9 || page==10); memcpy(out,flash_pages[page],count*4u); }
void flash_erase_page(unsigned sec,unsigned page)
{ (void)sec; assert(page==9 || page==10); memset(flash_pages[page],0,sizeof(flash_pages[page])); }
void flash_write_page(unsigned sec,unsigned page,const uint32_t *in,uint16_t count)
{
    (void)sec; assert(page==9 || page==10);
    if(write_limit>=0 && count>(unsigned)write_limit) count=(uint16_t)write_limit;
    memcpy(flash_pages[page],in,count*4u);
}

static path_status_t status(void) { path_status_t s; path_nav_status(&s); return s; }
static void straight(float length,float yaw)
{
    unsigned i,steps=(unsigned)lroundf(length/0.001f);
    for(i=0;i<steps;i++) path_nav_tick(0.001f,yaw,0.2f,1.0f);
}
static void record_straight(float length)
{
    path_nav_init(); assert(path_nav_record(345.0f));
    straight(length,345.0f); path_nav_finish_record();
    assert(status().valid);
}
static void copy_route(void)
{
    unsigned i;
    for(i=0;i<status().count;i++) assert(path_nav_point((uint16_t)i,&route[i]));
}

static void test_record_and_format(void)
{
    uint16_t count; uint32_t gen; unsigned n,i;
    record_straight(1.137f);
    assert(fabsf(status().length_m-1.137f)<0.003f); /* sub-spacing tail */
    copy_route();
    assert(fabsf(route[status().count-1].x)<0.001f);
    n=path_record_encode(words,route,status().count,42);
    assert(n && path_record_decode(words,decoded,&count,&gen));
    assert(gen==42 && count==status().count);
    assert(!memcmp(route,decoded,count*sizeof(route[0])));
    for(i=0;i<n;i++)
    {
        words[i]^=0x100u;
        assert(!path_record_decode(words,decoded,&count,&gen));
        words[i]^=0x100u;
    }
    route[1].s=-1.0f; assert(!path_nav_validate(route,status().count));
    copy_route(); route[1].x=NAN; assert(!path_nav_validate(route,status().count));
    copy_route(); route[1].s+=0.2f; assert(!path_nav_validate(route,status().count));
    assert(!path_nav_import(route,status().count)); /* old route survives rejection */
    assert(fabsf(status().length_m-1.137f)<0.003f);
    puts("PASS recording tail, coordinate frame, codec and corruption validation");
}

static void test_bounds_and_faults(void)
{
    unsigned i;
    path_nav_init(); assert(path_nav_record(0));
    for(i=0;i<60000 && path_nav_recording();i++) path_nav_tick(.001f,0,.2f,1);
    assert(status().count==PATH_MAX_POINTS && status().reason==PATH_FULL && status().valid);
    copy_route(); assert(path_nav_validate(route,status().count));
    path_nav_prepare(); assert(path_nav_start(0,.2f)); assert(!path_nav_record(0)); assert(!path_nav_import(route,status().count));
    path_nav_tick(NAN,0,0,1); assert(status().state==PATH_ERROR && status().speed_mps==0);
    path_nav_prepare(); assert(path_nav_start(0,.2f)); path_nav_tick(1.0f,0,0,1); assert(status().reason==PATH_INPUT);
    path_nav_prepare(); assert(path_nav_start(0,.2f)); path_nav_tick(0,20,0,1); assert(status().reason==PATH_INPUT);
    assert(path_nav_record(0));
    for(i=0;i<200;i++) path_nav_tick(-.001f,0,-.2f,1);
    assert(status().reason==PATH_REVERSE && !status().valid);
    record_straight(1);
    path_nav_prepare(); assert(path_nav_start(0,.2f));
    for(i=0;i<2000;i++) path_nav_tick(0,0,0,1);
    assert(status().reason==PATH_STALLED && status().speed_mps==0);
    path_nav_prepare(); assert(path_nav_start(0,.2f));
    for(i=0;i<2000 && status().state==PATH_RUNNING;i++) path_nav_tick(.001f,0,.2f,1);
    for(i=0;i<100;i++) path_nav_tick(0,0,0,1);
    assert(status().state==PATH_DONE);
    path_nav_prepare(); assert(path_nav_start(720,.2f)); assert(status().segment==0 && status().progress_m==0);
    path_nav_abort(); assert(status().state==PATH_READY && status().reason==PATH_ABORTED);
    path_nav_prepare(); assert(path_nav_start(0,.2f));
    for(i=0;i<950;i++) path_nav_tick(.001f,0,.2f,1);
    assert(status().state==PATH_BRAKING);
    for(i=0;i<1100;i++) path_nav_tick(0,0,.2f,1);
    assert(status().reason==PATH_STALLED); /* Failure to brake must not run forever. */
    path_nav_prepare(); assert(path_nav_start(0,.2f));
    for(i=0;i<100;i++) path_nav_tick(0,(float)i*.9f,0,1);
    for(i=0;i<900 && path_nav_running();i++) path_nav_tick(.001f,90,.2f,1);
    assert(status().reason==PATH_OFF_ROUTE);
    puts("PASS capacity, invalid input, reverse, off-route, stall, braking and restart");
}

static void test_closed_loop(float initial_yaw, float direction)
{
    unsigned i;
    float angle=0,yaw=initial_yaw,speed=0,maxcross=0;
    path_nav_init(); assert(path_nav_record(0));
    straight(1.0f,0);
    for(i=0;i<1571;i++) { angle+=direction*.0572957795f; path_nav_tick(.001f,angle,.2f,1); }
    straight(1.0f,angle); path_nav_finish_record();
    path_nav_prepare(); assert(path_nav_start(initial_yaw,.3f));
    for(i=0;i<40000 && path_nav_running();i++)
    {
        path_status_t s=status();
        float dv=s.speed_mps-speed;
        float step=(dv>0 ? PATH_ACCEL_MPS2 : PATH_DECEL_MPS2)*.005f;
        speed+=fmaxf(-step,fminf(step,dv));
        yaw+=fmaxf(-PATH_YAW_RATE_LIMIT,fminf(PATH_YAW_RATE_LIMIT,(s.yaw_target_deg-yaw)*PATH_YAW_ANGLE_KP))*.005f;
        path_nav_tick(speed*.005f,yaw,speed,1);
        if(status().cross_track_m>maxcross) maxcross=status().cross_track_m;
    }
    printf("closed-loop yaw %.0f dir %.0f state %d reason %d progress %.3f/%.3f cross %.3f ticks %u\n",
        initial_yaw,direction,status().state,status().reason,status().progress_m,status().length_m,maxcross,i);
    assert(status().state==PATH_DONE);
    assert(maxcross<.30f);
    path_nav_prepare(); assert(path_nav_start(initial_yaw+360,.3f));
    assert(fabsf(status().yaw_target_deg-(initial_yaw+360))<.001f);
    puts("PASS closed-loop straight/quarter-circle/straight tracking");
}

static void test_fast_route(int curved)
{
    unsigned i;
    float yaw=0,speed=0,peak=0,maxcross=0,angle=0;
    path_nav_init(); assert(path_nav_record(0)); straight(8.0f,0);
    if(curved) {
        for(i=0;i<3142;i++) {angle+=.02864789f;path_nav_tick(.001f,angle,.2f,1);}
        straight(4.0f,angle);
    } else straight(4.0f,0);
    path_nav_finish_record(); path_nav_prepare(); assert(path_nav_start(0,1.5f));
    for(i=0;i<60000 && path_nav_running();i++) {
        path_status_t s=status();
        float dv=s.speed_mps-speed,step=(dv>0?PATH_ACCEL_MPS2:PATH_DECEL_MPS2)*.005f;
        speed+=fmaxf(-step,fminf(step,dv));
        yaw+=fmaxf(-PATH_YAW_RATE_LIMIT,fminf(PATH_YAW_RATE_LIMIT,(s.yaw_target_deg-yaw)*PATH_YAW_ANGLE_KP))*.005f;
        path_nav_tick(speed*.005f,yaw,speed,1);
        peak=fmaxf(peak,speed);maxcross=fmaxf(maxcross,status().cross_track_m);
    }
    printf("fast curve %d state %d reason %d peak %.3f cross %.3f end %.3f/%.3f ticks %u\n",curved,status().state,status().reason,peak,maxcross,status().progress_m,status().length_m,i);
    assert(status().state==PATH_DONE); assert(peak>1.45f); assert(maxcross<.3f);
    assert(fabsf(status().progress_m-status().length_m)<.1f);
}

static void test_heading_reference(void)
{
    unsigned k;
    uint32_t rev;
    record_straight(8);
    assert(!status().heading_locked);
    rev=path_nav_revision();
    assert(path_nav_lock_heading(359));
    assert(path_nav_revision()!=rev && status().heading_locked);
    assert(!path_nav_lock_heading(NAN));
    assert(!path_nav_heading_ready(400));
    path_nav_prepare(); assert(!path_nav_start(400,.2f));
    assert(path_nav_start(369,.2f));
    assert(fabsf(status().yaw_deg-10)<.001f);
    assert(!path_nav_lock_heading(369));
    for(k=0;k<4;k++) path_nav_tick(.001f,369,.2f,1);
    assert(status().x>0 && status().yaw_target_deg<369);
    path_nav_abort();
    assert(path_nav_start(709,.2f)); /* Reference 719, error -10: no 360-degree spin. */
    assert(fabsf(status().yaw_deg+10)<.001f);
    path_nav_abort();
    copy_route(); assert(path_nav_import(route,status().count));
    assert(!status().heading_locked); /* Load/upload changes the route frame. */
    path_nav_prepare(); assert(path_nav_start(42,.2f));
    path_nav_abort(); assert(path_nav_start(47,.2f));
    assert(fabsf(status().heading_reference_deg-42)<.001f && fabsf(status().yaw_deg-5)<.001f);
    path_nav_abort(); path_nav_clear(); assert(!status().heading_locked);
    puts("PASS heading lock, repeated starts, multi-turn yaw and route replacement");
}

static void test_misaligned_start(float offset,float cruise)
{
    unsigned i;
    float yaw=720+offset,speed=0,cmd=0,rate=0,maxcross=0,midyaw=999;
    record_straight(8); assert(path_nav_lock_heading(0)); path_nav_prepare();
    assert(path_nav_start(yaw,cruise));
    for(i=0;i<60000 && path_nav_running();i++)
    {
        path_status_t s=status();
        float dv=s.speed_mps-speed, step=(dv>0?PATH_ACCEL_MPS2:PATH_DECEL_MPS2)*.005f;
        float limit,desired;
        speed+=fmaxf(-step,fminf(step,dv));
        limit=PATH_YAW_RATE_LIMIT*fmaxf(0,fminf(1,(speed-.03f)/.17f));
        desired=fmaxf(-limit,fminf(limit,PATH_YAW_ANGLE_KP*(s.yaw_target_deg-yaw)));
        cmd+=fmaxf(-PATH_YAW_SLEW_DPS2*.005f,fminf(PATH_YAW_SLEW_DPS2*.005f,desired-cmd));
        cmd=fmaxf(-limit,fminf(limit,cmd));
        rate+=(cmd-rate)*(.005f/.08f); /* Illustrative rate lag, not wheel dynamics. */
        yaw+=rate*.005f;
        path_nav_tick(speed*.005f,yaw,speed,1);
        maxcross=fmaxf(maxcross,status().cross_track_m);
        if(status().progress_m>2 && status().progress_m<2.02f) midyaw=yaw-720;
    }
    printf("yaw offset %.0f speed %.2f state %d cross %.3f final yaw %.2f\n",
        offset,cruise,status().state,maxcross,yaw-720);
    assert(status().state==PATH_DONE && maxcross<.25f);
    printf("mid-route yaw %.2f final x %.3f\n",midyaw,status().x);
    assert(fabsf(status().x)<.06f && fabsf(midyaw)<3);
}

static void test_flash_slots(void)
{
    unsigned cut, record_words;
    uint32_t baseline[12][1024];
    record_straight(1.0f); assert(path_store_save());
    assert(!status().dirty);
    memcpy(baseline,flash_pages,sizeof(baseline));
    /* Interrupt a replacement at every word. Reboot must always load old data. */
    record_straight(2.0f); copy_route();
    record_words=4u+3u*status().count;
    for(cut=0;cut<record_words;cut++)
    {
        memcpy(flash_pages,baseline,sizeof(baseline));
        record_straight(2.0f);
        write_limit=(int)cut;
        assert(!path_store_save());
        path_store_init();
        assert(fabsf(status().length_m-1.0f)<.003f);
    }
    write_limit=-1;
    /* A torn occupied target is preserved, not silently reclaimed. */
    assert(!path_store_save());
    memcpy(flash_pages,baseline,sizeof(baseline));
    flash_pages[10][0]=0x11223344;
    assert(!path_store_save() && flash_pages[10][0]==0x11223344);
    memcpy(flash_pages,baseline,sizeof(baseline));
    record_straight(2); assert(path_store_save());
    path_store_init(); assert(fabsf(status().length_m-2.0f)<.003f);
    flash_ready=0; assert(!path_store_save() && !path_store_load()); flash_ready=1;
    path_nav_clear(); assert(path_store_save());
    path_store_init(); assert(status().count==0); /* tombstone suppresses older slot */
    puts("PASS redundant Flash slots, torn writes, readback, stopped gate and saved clear");
}

int main(void)
{
    test_record_and_format(); test_bounds_and_faults();
    test_closed_loop(0,1); test_closed_loop(725,-1);
    test_fast_route(0); test_fast_route(1);
    test_heading_reference();
    test_misaligned_start(20,.2f); test_misaligned_start(-20,.8f);
    test_misaligned_start(20,1.5f); test_misaligned_start(-20,1.5f);
    test_flash_slots();
    puts("ALL PATH TESTS PASSED"); return 0;
}

int flash_last_status(void) { return 0; }
