/* Exercise the real recorder and cfg entry points in both UART and WiFi builds. */
{
    start_flag=START_STOP; mock_test=mock_jog=mock_run=mock_remote=0;
    mock_path_busy=mock_path_active=0;
    feed("cfg:trace_arm,200,2\n");assert(strstr(drain(),"ok,trace_arm"));
    assert(s_trace_state==TRACE_ARMED && s_trace_count==0);
    trace_tick1ms();assert(s_trace_count==0); /* STOP does not start motors */
    feed("cfg:trace_get,201,0\n");assert(strstr(drain(),"BUSY"));
    feed("cfg:trace_arm,202,0\n");assert(strstr(drain(),"BUSY"));
    mock_test=1;g_tune_axis=TUNE_AXIS_ROLL;g_tune_ring=TUNE_RING_RATE;
    ++g_control_uptime_ms;trace_tick1ms();assert(s_trace_count==0);
    ++g_control_uptime_ms;trace_tick1ms();assert(s_trace_count==0);
    ++g_control_uptime_ms;trace_tick1ms();assert(s_trace_count==1);
    for(unsigned i=1;i<TRACE_CAPACITY;i++){++g_control_uptime_ms;trace_tick1ms();}
    assert(s_trace_state==TRACE_DONE && s_trace_count==TRACE_CAPACITY && s_trace_reason==1);
    assert(s_trace_data[0].gx==2.0f && s_trace_data[0].az==1.0f && s_trace_data[0].flags==7);
    uint32 first=s_trace_data[0].ticks;
    ++g_control_uptime_ms;trace_tick1ms();assert(s_trace_data[0].ticks==first);
    feed("cfg:trace_get,203,0\n");assert(strstr(drain(),"BUSY")); /* still running */
    mock_test=0;
    feed("cfg:trace_get,204,0\n");assert(strstr(drain(),"ok,trace_get,0,"));
    feed("cfg:trace_get,205,255\n");assert(strstr(drain(),"ok,trace_get,255,"));
    feed("cfg:trace_get,206,256\n");assert(strstr(drain(),"INVALID_VALUE"));
    feed("cfg:trace_arm,207,0\n");drain();
    ++g_param_revision;trace_tick1ms();assert(s_trace_state==TRACE_DONE && s_trace_reason==3);
    feed("cfg:trace_arm,208,0\n");drain();
    mock_test=1;g_tune_ring=TUNE_RING_ANGLE;trace_tick1ms();
    assert(s_trace_reason==5 && s_trace_count==0);mock_test=0;
    feed("cfg:trace_arm,209,0\n");drain();
    g_tune_ring=TUNE_RING_RATE;mock_test=1;trace_tick1ms();mock_test=0;
    trace_tick1ms();assert(s_trace_count==1 && s_trace_reason==2);
    feed("cfg:trace_arm,210,0\n");drain();
    g_control_uptime_ms+=30000;trace_tick1ms();assert(s_trace_reason==4);
    feed("cfg:trace_arm,211,0\n");drain();
    feed("cfg:trace_cancel,212\n");drain();assert(s_trace_reason==6);
    feed("cfg:trace_arm,213,-1\n");assert(strstr(drain(),"INVALID_VALUE"));
    feed("cfg:trace_arm,214,0\n");drain();
    feed("stop\n");drain();assert(s_trace_state==TRACE_DONE && s_trace_reason==2);
    feed("cfg:trace_status,215\n");assert(strstr(drain(),"ok,trace_status,1,3,0,256,1000,"));
}
