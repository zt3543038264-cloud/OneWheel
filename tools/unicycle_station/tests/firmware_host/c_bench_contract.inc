/* Execute the real byte parser and cfg routing; controller fixture above is
 * deliberately not a hardware/model claim. Core integration is tested apart. */
{
    unsigned begins=cbench_starts, stops=cbench_stops;
    unsigned b_before=bench_starts;
    uint32 revision=g_param_revision;
    int saved=save_count;
    start_flag=START_STOP;mock_run=mock_remote=mock_test=mock_jog=mock_path_active=0;
    mock_path_busy=0;s_save_seq=s_path_save_seq=0;
    memset(&cbench,0,sizeof(cbench));drain();
    feed("cfg:cbench_status,501\n");
    assert(strstr(drain(),"rsp:501,ok,cbench_status,1,0,0,0,0,0,0,0,"));
    assert(cbench_starts==begins && cbench_stops==stops);
    const char *bad[]={
      "cfg:cbench_start,502,0,dry_raw,501,0,0,100,500\n",
      "cfg:cbench_start,502,0,dry_raw,1,1,0,100,500\n",
      "cfg:cbench_start,502,0,dry_rate,nan,1,0,100,500\n",
      "cfg:cbench_start,502,0,dry_rate,0,inf,0,100,500\n",
      "cfg:cbench_start,502,0,dry_rate,31,1,0,100,500\n",
      "cfg:cbench_start,502,0,dry_rate,0,101,0,100,500\n",
      "cfg:cbench_start,502,0,dry_rate,0,1,1001,100,500\n",
      "cfg:cbench_start,502,0,dry_rate,0,1,0,3001,500\n",
      "cfg:cbench_start,502,0,dry_rate,0,1,0,100,0\n",
      "cfg:cbench_start,502,0,dry_rate,0,1,0,100,500,extra\n",
      "cfg:cbench_start,502,0,dry_rate,0,1,0,100\n",
      "cfg:cbench_start,502,4294967296,dry_rate,0,1,0,100,500\n",
      "cfg:cbench_start,502,0,dry_raw,1e99,0,0,100,500\n"
    };
    for(unsigned i=0;i<sizeof(bad)/sizeof(bad[0]);++i) {
        feed(bad[i]);drain();assert(cbench_starts==begins && !cbench.active);
    }
    /* Oversize line remains poisoned to its newline; no suffix resync. */
    char oversized[320];memset(oversized,'x',220);
    strcpy(oversized+220,"cfg:cbench_start,503,0,dry_raw,1,0,0,100,100\n");
    feed(oversized);drain();assert(cbench_starts==begins);
    /* Exactly 191 payload bytes fit the 192 buffer; 192 do not. */
    {
        const char *tail=",dry_raw,1,0,0,100,100";
        int n=sprintf(oversized,"cfg:cbench_start,504,");
        unsigned zeros=191u-(unsigned)n-(unsigned)strlen(tail);
        memset(oversized+n,'0',zeros);strcpy(oversized+n+zeros,tail);strcat(oversized,"\n");
        assert(strlen(oversized)==192u);feed(oversized);
        assert(strstr(drain(),"rsp:504,ok,cbench_start,1,1,"));assert(cbench_starts==begins+1);
        feed("cfg:cbench_stop,505\n");drain();
        n=sprintf(oversized,"cfg:cbench_start,506,");
        zeros=192u-(unsigned)n-(unsigned)strlen(tail);
        memset(oversized+n,'0',zeros);strcpy(oversized+n+zeros,tail);strcat(oversized,"\n");
        feed(oversized);drain();assert(cbench_starts==begins+1);
    }
    /* Default profile refuses signal modes and never silently falls back. */
    feed("cfg:cbench_start,507,1,signal_raw,10,0,0,100,100\n");assert(strstr(drain(),"CBENCH_BLOCKED"));
    feed("cfg:cbench_start,508,1,signal_rate,10,1,0,100,100\n");assert(strstr(drain(),"CBENCH_BLOCKED"));
    s_save_seq=33;
    feed("cfg:cbench_start,509,1,dry_rate,0,1,0,3000,500\n");assert(strstr(drain(),"BUSY,Flash operation pending"));
    s_save_seq=0;
    feed("cfg:cbench_start,510,1,dry_rate,0,1,0,3000,500\n");
    assert(strstr(drain(),"rsp:510,ok,cbench_start,1,1,1,1,"));
    assert(cbench_starts==begins+2 && cbench.session==2 && !cfg_stopped() && cfg_pid_mask()==0);
    assert(cfg_status());assert(strstr(drain(),",0,0,0,3,"));
    feed("cfg:set,511,r_rate_kp,35\n");assert(strstr(drain(),"RUNNING_LOCKED"));
    feed("cfg:save,512,all\n");assert(strstr(drain(),"SAVE_BLOCKED"));
    feed("cfg:cbench_keep,513,2,1\n");assert(strstr(drain(),"rsp:513,ok,cbench_keep,"));assert(cbench.heartbeat_sequence==1);
    unsigned keeps=cbench_keeps;
    feed("cfg:cbench_keep,514,2,1\n");assert(strstr(drain(),"CBENCH_BLOCKED"));assert(cbench_keeps==keeps);
    feed("cfg:cbench_keep,515,1,2\n");assert(strstr(drain(),"CBENCH_BLOCKED"));assert(cbench_keeps==keeps);
    feed("cfg:cbench_stop,516\n");assert(strstr(drain(),"rsp:516,ok,cbench_stop,1,0,"));
    feed("cfg:cbench_start,517,1,dry_rate,0,1,0,3000,500\n");assert(strstr(drain(),"CBENCH_BLOCKED"));
    /* Profile-enabled protocol includes signal PI only as an explicit mode. */
    cbench.signal_allowed=1;
    feed("cfg:cbench_start,518,2,signal_rate,-2,0.5,0,100,100\n");
    assert(strstr(drain(),"rsp:518,ok,cbench_start,1,1,3,"));
    feed("cfg:cbench_stop,519\n");drain();cbench.signal_allowed=0;
    assert(g_param_revision==revision && save_count==saved && bench_starts==b_before);
    printf("C bench parser contract passed\n");
}
