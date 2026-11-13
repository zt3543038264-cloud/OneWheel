/* Read-only retrieval after stopping. One record per request, no retries,
 * no clear/arm/start/keep operation, and no controller state mutation.
 */
static void keep_log_cfg(char **f,uint8 count,uint16 seq)
{
    char payload[320];
    uint32 first=s_keep_log_last>KLOG_CAPACITY?s_keep_log_last-KLOG_CAPACITY+1u:1u;
    uint32 id,hz,used=s_keep_log_last>KLOG_CAPACITY?KLOG_CAPACITY:s_keep_log_last;
    keep_log_record_t *r;
    if(!cfg_stopped()) {(void)cfg_reply(seq,"err","BUSY,Stop before keep-log retrieval");return;}
    if(!strcmp(f[0],"klog_status")&&count==2u) {
        hz=(uint32)IfxStm_getFrequency(&MODULE_STM0);
        (void)snprintf(payload,sizeof(payload),"klog_status,1,%u,%lu,%lu,%lu,%lu,%u",
            (unsigned)KLOG_CAPACITY,(unsigned long)(used?first:0u),(unsigned long)s_keep_log_last,
            (unsigned long)(s_keep_log_last-used),(unsigned long)hz,(unsigned)s_keep_log_exhausted);
        (void)cfg_reply(seq,"ok",payload);return;
    }
    if(!strcmp(f[0],"klog_get")&&count==3u) {
        if(!commission_u32(f[2],&id)||!id||id<first||id>s_keep_log_last) {
            (void)cfg_reply(seq,"err","LOG_NOT_RETAINED,Unknown or overwritten record id");return;
        }
        r=&s_keep_log[(id-1u)&(KLOG_CAPACITY-1u)];
        if(r->id!=id) {(void)cfg_reply(seq,"err","LOG_NOT_RETAINED,Record id mismatch");return;}
        (void)snprintf(payload,sizeof(payload),
            "klog_get,1,%lu,%u,%lu,%lu,%u,%lu,%lu,%lu,%lu,%u,%u,%lu,%lu,%lu,%lu,%u",
            (unsigned long)r->id,(unsigned)r->cfg_seq,(unsigned long)r->session,(unsigned long)r->keep,
            (unsigned)r->result,(unsigned long)r->rx_hi,(unsigned long)r->rx_lo,
            (unsigned long)r->reply_hi,(unsigned long)r->reply_lo,(unsigned)r->queue_result,
            (unsigned)r->reply_bytes,(unsigned long)r->reply_hash,(unsigned long)r->tx_end,
            (unsigned long)r->spi_hi,(unsigned long)r->spi_lo,(unsigned)r->spi_seen);
        (void)cfg_reply(seq,"ok",payload);return;
    }
    (void)cfg_reply(seq,"err","BAD_FORMAT,Unknown keep-log read operation");
}
