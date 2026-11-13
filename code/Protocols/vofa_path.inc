/* Included by vofa.c. CPU0 foreground only; all arrays are static. */
#ifndef PATH_HOST_TEST
#pragma section all "cpu0_dsram"
#endif
static path_point_t s_upload[PATH_MAX_POINTS];
static uint16 s_upload_token, s_upload_count, s_upload_next, s_path_save_seq;
static uint32 s_upload_crc, s_upload_at, s_upload_revision;
static uint32 s_arm_token, s_arm_revision, s_arm_at, s_arm_counter;
static uint8 s_path_stream, s_path_store_save;
#ifndef PATH_HOST_TEST
#pragma section all restore
#endif

static void path_transfer_cancel(void)
{
    s_upload_token=0; s_arm_token=0; s_path_save_seq=0;
}

static uint8 path_uint(const char *text, uint32 *out)
{
    uint32 value=0, digit;
    if (!*text) return 0;
    while (*text)
    {
        if (*text<'0' || *text>'9') return 0;
        digit=(uint32)(*text++-'0');
        if (value>429496729u || (value==429496729u && digit>5u)) return 0;
        value=value*10u+digit;
    }
    *out=value; return 1;
}

static uint32 path_crc_word(uint32 crc, uint32 word)
{
    uint8 i;
    crc^=word;
    for (i=0;i<32u;i++) crc=(crc>>1)^((crc&1u)?0xEDB88320u:0u);
    return crc;
}

static uint8 path_tx_status(void)
{
    path_status_t p;
    char line[224];
    int len;
    control_path_status(&p);
    len=snprintf(line,sizeof(line),"nav:%lu,%u,%u,%u,%u,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%u,%u,%u,%lu\n",
        (unsigned long)g_control_uptime_ms,(unsigned)p.state,(unsigned)p.reason,
        (unsigned)p.count,(unsigned)p.segment,(double)p.x,(double)p.y,(double)p.yaw_deg,
        (double)p.progress_m,(double)p.length_m,(double)p.speed_mps,
        (unsigned)p.dirty,(unsigned)p.valid,(unsigned)control_path_active(),(unsigned long)path_nav_revision());
    return len>0 && len<(int)sizeof(line) ? tx_push((const uint8 *)line,(uint32)len) : 0;
}

static void path_cfg(char **f, uint8 n, uint16 seq)
{
    uint32 a=0,b=0;
    float x,y;
    char reply[144];
    const char *error="BAD_FORMAT,Invalid path request";
    const char *op=f[0];
    if (!strcmp(op,"path_status") && n==2u)
    {
        s_path_stream=1;
        if (!path_tx_status()) error="BUSY,Telemetry queue full";
        else { (void)cfg_reply(seq,"ok","path_status,1,256,1.50"); return; }
    }
    else if (!strcmp(op,"path_cancel") && n==2u)
    { path_transfer_cancel(); (void)cfg_reply(seq,"ok",op); return; }
    else if (!cfg_stopped()) error="RUNNING_LOCKED,Stop required";
    else if (s_path_save_seq || s_save_seq || s_schema_seq) error="BUSY,Operation pending";
    else if (!strcmp(op,"path_begin") && n==3u && path_uint(f[2],&a) && a>=2u && a<=PATH_MAX_POINTS)
    {
        s_arm_token=0; s_upload_token=seq; s_upload_count=(uint16)a; s_upload_next=0;
        s_upload_crc=0xFFFFFFFFu; s_upload_at=g_control_uptime_ms; s_upload_revision=path_nav_revision();
        (void)snprintf(reply,sizeof(reply),"path_begin,%u",(unsigned)s_upload_token);
        (void)cfg_reply(seq,"ok",reply); return;
    }
    else if (!strcmp(op,"path_point") && n==6u && path_uint(f[2],&a) && path_uint(f[3],&b) &&
        cmd_parse_float_strict(f[4],&x) && cmd_parse_float_strict(f[5],&y))
    {
        if (!s_upload_token || a!=s_upload_token || b!=s_upload_next || b>=s_upload_count ||
            path_nav_revision()!=s_upload_revision || (uint32)(g_control_uptime_ms-s_upload_at)>5000u)
            error="PATH_SESSION,Upload expired or out of order";
        else if (fabsf(x)>1000000.0f || fabsf(y)>1000000.0f || x!=floorf(x) || y!=floorf(y))
            error="OUT_OF_RANGE,Integer millimetres required";
        else
        {
            s_upload[b].x=x*0.001f; s_upload[b].y=y*0.001f; s_upload[b].s=0.0f;
            if (b)
            {
                float dx=s_upload[b].x-s_upload[b-1u].x, dy=s_upload[b].y-s_upload[b-1u].y;
                s_upload[b].s=s_upload[b-1u].s+sqrtf(dx*dx+dy*dy);
            }
            s_upload_crc=path_crc_word(path_crc_word(s_upload_crc,(uint32)(int32)x),(uint32)(int32)y);
            s_upload_next++; s_upload_at=g_control_uptime_ms;
            (void)snprintf(reply,sizeof(reply),"path_point,%lu",(unsigned long)b);
            (void)cfg_reply(seq,"ok",reply); return;
        }
    }
    else if (!strcmp(op,"path_commit") && n==4u && path_uint(f[2],&a) && path_uint(f[3],&b))
    {
        if (!s_upload_token || a!=s_upload_token || s_upload_next!=s_upload_count ||
            path_nav_revision()!=s_upload_revision || (uint32)(g_control_uptime_ms-s_upload_at)>5000u ||
            b!=(s_upload_crc^0xFFFFFFFFu)) error="PATH_SESSION,Incomplete or CRC mismatch";
        else if (!path_nav_import(s_upload,s_upload_count)) error="INVALID_PATH,Invalid geometry";
        else
        {
            path_nav_modified(); s_upload_token=0;
            (void)snprintf(reply,sizeof(reply),"path_commit,%lu",(unsigned long)path_nav_revision());
            (void)cfg_reply(seq,"ok",reply); return;
        }
    }
    else if (!strcmp(op,"path_get") && n==4u && path_uint(f[2],&a) && path_uint(f[3],&b))
    {
        path_point_t point;
        if (a!=path_nav_revision() || b>=PATH_MAX_POINTS || !control_path_point((uint16)b,&point))
            error="PATH_CHANGED,Refresh route before download";
        else
        {
            (void)snprintf(reply,sizeof(reply),"path_get,%lu,%.6f,%.6f",(unsigned long)b,(double)point.x,(double)point.y);
            (void)cfg_reply(seq,"ok",reply); return;
        }
    }
    else if (!strcmp(op,"path_arm") && n==3u && path_uint(f[2],&a))
    {
        path_status_t p; control_path_status(&p);
        if (s_upload_token || a!=path_nav_revision() || !p.valid || p.length_m<0.30f)
            error="INVALID_PATH,Upload or refresh route first";
        else
        {
            if (++s_arm_counter==0u) s_arm_counter=1u;
            s_arm_token=s_arm_counter; s_arm_revision=a; s_arm_at=g_control_uptime_ms;
            (void)snprintf(reply,sizeof(reply),"path_arm,%lu",(unsigned long)s_arm_token);
            (void)cfg_reply(seq,"ok",reply); return;
        }
    }
    else if (!strcmp(op,"path_start") && n==4u && path_uint(f[2],&a) && cmd_parse_float_strict(f[3],&x))
    {
        uint8 valid=(uint8)(a && a==s_arm_token && path_nav_revision()==s_arm_revision &&
            (uint32)(g_control_uptime_ms-s_arm_at)<=3000u);
        s_arm_token=0; /* One shot, including rejected starts. */
        if (!valid) error="PATH_SESSION,Start token expired";
        else if (!control_path_start(x)) error="START_BLOCKED,Vehicle gates or speed rejected";
        else { (void)cfg_reply(seq,"ok",op); return; }
    }
    else if (!strcmp(op,"path_save") && n==2u && !s_upload_token)
    { s_arm_token=0; s_path_save_seq=seq; s_path_store_save=1; return; }
    else if (!strcmp(op,"path_load") && n==2u && !s_upload_token)
    { s_arm_token=0; s_path_save_seq=seq; s_path_store_save=0; return; }
    /* Save/load are deferred until RX quiet. */
    s_upload_token=0; s_arm_token=0;
    (void)cfg_reply(seq,"err",error);
}

static void path_cfg_poll(void)
{
    static uint32 last;
    uint32 now=g_control_uptime_ms;
    if (s_upload_token && ((uint32)(now-s_upload_at)>5000u || !cfg_stopped() || path_nav_revision()!=s_upload_revision)) s_upload_token=0;
    if (s_arm_token && ((uint32)(now-s_arm_at)>3000u || !cfg_stopped() || path_nav_revision()!=s_arm_revision)) s_arm_token=0;
    if (s_path_stream && (uint32)(now-last)>=200u)
    { last=now; (void)path_tx_status(); }
    if (s_path_save_seq && vofa_flash_idle())
    {
        uint16 seq=s_path_save_seq;
        uint8 save=s_path_store_save;
        s_path_save_seq=0;
        if (!control_path_storage_ready()) (void)cfg_reply(seq,"err","SAVE_BLOCKED,Stable stopped wheels required");
        else if (!(save?path_store_save():path_store_load())) (void)cfg_reply(seq,"err","FLASH_FAILED,Route storage failed");
        else (void)cfg_reply(seq,"ok",save?"path_save,VERIFIED":"path_load,VERIFIED");
    }
}
