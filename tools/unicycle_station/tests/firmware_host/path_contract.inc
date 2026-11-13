/* Included inside main: protocol tests use existing vehicle gate mocks. */
start_flag=START_STOP;mock_path_busy=mock_path_active=mock_test=mock_jog=mock_run=0;
s_schema_seq=s_save_seq=0;drain();
feed("cfg:path_status,100\n");assert(strstr(drain(),"path_status,1,256,1.50"));
feed("cfg:path_begin,101,3\n");assert(strstr(drain(),"path_begin,101"));
feed("cfg:path_point,102,101,0,0,0\n");drain();
feed("cfg:path_point,103,101,1,0,200\n");drain();
feed("cfg:path_commit,104,101,0\n");assert(strstr(drain(),"PATH_SESSION"));assert(mock_path.count==0);
feed("cfg:path_begin,105,3\n");drain();
feed("cfg:path_point,106,105,0,0,0\n");drain();
feed("cfg:path_point,107,105,1,0,200\n");drain();
feed("cfg:path_point,108,105,2,0,400\n");drain();
assert((s_upload_crc^0xffffffffu)==1240199005u);
{char line[80];snprintf(line,sizeof(line),"cfg:path_commit,109,105,%lu\n",(unsigned long)(s_upload_crc^0xffffffffu));feed(line);}
assert(strstr(drain(),"ok,path_commit"));assert(mock_path.count==3 && mock_path.dirty);
feed("cfg:path_begin,110,2\n");drain();g_control_uptime_ms+=5001;
feed("cfg:path_point,111,110,0,0,0\n");assert(strstr(drain(),"PATH_SESSION"));assert(mock_path.count==3);
{char line[80];snprintf(line,sizeof(line),"cfg:path_arm,112,%lu\n",(unsigned long)mock_path_revision);feed(line);drain();
 snprintf(line,sizeof(line),"cfg:path_start,113,%lu,1.50\n",(unsigned long)s_arm_token);feed(line);assert(strstr(drain(),"ok,path_start"));
 assert(mock_path_active);start_flag=START_STOP;mock_path_active=0;feed(line);assert(strstr(drain(),"PATH_SESSION"));}
{char line[80];snprintf(line,sizeof(line),"cfg:path_arm,114,%lu\n",(unsigned long)mock_path_revision);feed(line);drain();
 snprintf(line,sizeof(line),"cfg:path_start,115,%lu,1.50\n",(unsigned long)s_arm_token);mock_path_revision++;feed(line);assert(strstr(drain(),"PATH_SESSION"));}
feed("cfg:path_save,116\n");assert(s_path_save_seq==116);feed("stop\n");assert(!s_path_save_seq);drain();
/* Oversize sequence/CRC, duplicates and point reordering fail closed. */
feed("cfg:path_begin,117,2\n");drain();
feed("cfg:path_point,118,117,1,0,200\n");assert(strstr(drain(),"PATH_SESSION"));
feed("cfg:path_commit,119,117,4294967296\n");assert(strstr(drain(),"BAD_FORMAT"));
assert(mock_path.count==3);
