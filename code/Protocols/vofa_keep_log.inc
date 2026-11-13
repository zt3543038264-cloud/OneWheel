/* RAM-only keep evidence, CPU0 foreground only. Never renews a lease, writes
 * motors/Flash, or runs in the 1ms ISR. RX means cfg parser dispatch, not radio
 * ingress. SPI means bytes handed to the module, not UDP delivery confirmation.
 * Raw host replies can be compared with reply_bytes + FNV-1a reply_hash.
 */
#define KLOG_CAPACITY 64u
typedef struct {
    uint32 id, session, keep;
    uint32 rx_hi, rx_lo, reply_hi, reply_lo, spi_hi, spi_lo;
    uint32 reply_hash, tx_end;
    uint16 cfg_seq, reply_bytes;
    uint8 result, queue_result, spi_seen, reserved;
} keep_log_record_t;
#pragma section all "cpu0_dsram"
static keep_log_record_t s_keep_log[KLOG_CAPACITY];
static uint32 s_keep_log_last;
static uint8 s_keep_log_exhausted;
static keep_log_record_t *s_keep_log_context;
#pragma section all restore
static uint64 lprobe_ticks(void);
static void keep_log_cfg(char **f, uint8 count, uint16 seq);

static void keep_log_reset(void)
{
    memset(s_keep_log,0,sizeof(s_keep_log));
    s_keep_log_last=0;
    s_keep_log_exhausted=0;
    s_keep_log_context=0;
}
static void keep_log_begin(uint16 seq)
{
    uint64 ticks;
    keep_log_record_t *r;
    s_keep_log_context=0;
    if(s_keep_log_last==0xffffffffu) {s_keep_log_exhausted=1;return;}
    ++s_keep_log_last;
    r=&s_keep_log[(s_keep_log_last-1u)&(KLOG_CAPACITY-1u)];
    memset(r,0,sizeof(*r));
    ticks=lprobe_ticks();
    r->id=s_keep_log_last;r->cfg_seq=seq;r->result=3u;
    r->rx_hi=(uint32)(ticks>>32);r->rx_lo=(uint32)ticks;
    s_keep_log_context=r;
}
static void keep_log_fields(uint32 session,uint32 keep)
{
    if(!s_keep_log_context)return;
    s_keep_log_context->session=session;s_keep_log_context->keep=keep;
}
static void keep_log_result(uint8 result)
{if(s_keep_log_context)s_keep_log_context->result=result;}
static void keep_log_reply(uint16 seq,const char *line,uint16 length,uint8 queued,uint32 end)
{
    uint64 ticks;
    uint32 hash=2166136261u,i;
    keep_log_record_t *r=s_keep_log_context;
    if(!r||r->cfg_seq!=seq)return;
    ticks=lprobe_ticks();
    r->reply_hi=(uint32)(ticks>>32);r->reply_lo=(uint32)ticks;
    r->queue_result=queued?1u:2u;r->reply_bytes=length;r->tx_end=end;
    for(i=0;i<length;i++) {hash^=(uint8)line[i];hash*=16777619u;}
    r->reply_hash=length?hash:0u;
    s_keep_log_context=0;
}
static void keep_log_spi(uint32 new_tail)
{
    uint32 i;
    uint64 ticks=lprobe_ticks();
    for(i=0;i<KLOG_CAPACITY;i++) {
        keep_log_record_t *r=&s_keep_log[i];
        if(r->id&&r->queue_result==1u&&!r->spi_seen&&
           (uint32)(new_tail-r->tx_end)<0x80000000u) {
            r->spi_hi=(uint32)(ticks>>32);r->spi_lo=(uint32)ticks;r->spi_seen=1;
        }
    }
}
