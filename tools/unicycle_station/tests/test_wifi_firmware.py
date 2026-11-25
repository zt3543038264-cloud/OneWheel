"""Compile and execute the actual foreground SPI state machine on the host."""
from pathlib import Path
import shutil
import subprocess
import pytest

ROOT=Path(__file__).resolve().parents[3]

@pytest.mark.parametrize("enabled",[0,1])
def test_spi_step_bounded_handshake_payload_and_invalid_length(tmp_path,enabled):
    gcc=shutil.which("gcc") or "D:/Tools/Dev-Cpp/MinGW64/bin/gcc.exe"
    if not Path(gcc).exists():pytest.skip("Optional GCC host compiler not installed")
    (tmp_path/"Protocols").mkdir()
    (tmp_path/"config_private.h").write_text('#define STATION_WIFI_SSID "host-test"\n#define STATION_WIFI_PASSWORD "host-only"\n#define STATION_WIFI_PC_IP "127.0.0.1"\n')
    (tmp_path/"zf_common_headfile.h").write_text(r'''#ifndef WIFI_HOST_SHIM
#define WIFI_HOST_SHIM
#include <stdint.h>
#include <string.h>
typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
int gpio_get_level(int pin);
void system_delay_ms(uint32 ms);
void gpio_low(int pin);
void gpio_high(int pin);
void spi_transfer_8bit(int index, const uint8 *tx, uint8 *rx, uint32 length);
void spi_write_8bit_array(int index, const uint8 *tx, uint32 length);
#endif
''')
    (tmp_path/"zf_device_wifi_spi.h").write_text(r'''#include "zf_common_headfile.h"
#define WIFI_SPI_INT_PIN 1
#define WIFI_SPI_CS_PIN 2
#define WIFI_SPI_INDEX 3
#define WIFI_SPI_DATA 2
#define WIFI_SPI_UDP_SEND 3
#define WIFI_SPI_REPLY_OK 0x80
#define WIFI_SPI_REPLY_DATA_START 0x90
#define WIFI_SPI_REPLY_DATA_END 0x91
typedef struct { uint8 command, reserve; uint16 length; } wifi_spi_head_struct;
uint8 wifi_spi_init(char *ssid, char *password);
uint8 wifi_spi_socket_connect(char *transport, char *ip, char *port, char *local);
''')
    source=r'''#include <assert.h>
#include "Protocols/station_wifi.c"
#if STATION_WIFI_ENABLED
static int ready=1, phase, packets, transfers, writes, selected, oversized, bad_ack;
static int init_attempts, socket_attempts;
static uint32 delay_ms;
void system_delay_ms(uint32 ms) {delay_ms+=ms;}
uint8 wifi_spi_init(char *ssid,char *password) {
    (void)password;assert(*ssid);return ++init_attempts==1;
}
uint8 wifi_spi_socket_connect(char *type,char *ip,char *port,char *local) {
    (void)ip;(void)port;(void)local;assert(!strcmp(type,"UDP"));return ++socket_attempts==1;
}
int gpio_get_level(int pin) {(void)pin;return ready;}
void gpio_low(int pin) {(void)pin;assert(!selected);selected=1;packets++;transfers=0;writes=0;}
void gpio_high(int pin) {(void)pin;assert(selected);selected=0;phase=(phase+1)%3;}
void spi_transfer_8bit(int index,const uint8 *tx,uint8 *rx,uint32 length) {
    (void)index;assert(selected);transfers++;
    if(transfers==1) {
        wifi_spi_head_struct sent,reply={0};memcpy(&sent,tx,4);assert(length==4 && sent.reserve==0);
        assert(sent.command==(phase==1 ? WIFI_SPI_UDP_SEND:WIFI_SPI_DATA));
        assert(sent.length==(phase==0 ? 70:0));
        reply.command=phase==2 && !bad_ack ? WIFI_SPI_REPLY_OK:WIFI_SPI_REPLY_DATA_END;
        reply.length=oversized ? 33:(phase==0 ? 3:2);memcpy(rx,&reply,4);
    } else {
        assert(phase!=2 && transfers==2 && length==32);
        if(phase==0)assert(tx[0]=='x');
        memcpy(rx,phase==0 ? "abc":"de",phase==0 ? 3:2);
    }
}
void spi_write_8bit_array(int index,const uint8 *tx,uint32 length) {
    (void)index;assert(selected && phase==0 && length==38 && tx[length-1]=='\n');writes++;
}
#endif
int main(void) {
    uint8 tx[70],rx[32];uint16 received=99;memset(tx,'x',sizeof(tx));tx[69]='\n';
#if STATION_WIFI_ENABLED
    station_wifi_diag_t diag;
    assert(station_wifi_init()==1);
    assert(init_attempts==3 && socket_attempts==2 && delay_ms==1300);
    ready=0;assert(station_wifi_step(tx,0,rx,&received,0)==0 && packets==0);
    assert(station_wifi_step(tx,70,rx,&received,1)==0 && received==0 && packets==0);
    ready=1;assert(station_wifi_step(tx,70,rx,&received,5)==70);
    assert(received==3 && !memcmp(rx,"abc",3) && packets==1 && writes==1);
    ready=0;assert(station_wifi_step(tx,70,rx,&received,6)==0 && packets==1);
    ready=1;assert(station_wifi_step(tx,70,rx,&received,13)==0);
    assert(received==2 && !memcmp(rx,"de",2) && packets==2);
    ready=0;assert(station_wifi_step(tx,70,rx,&received,14)==0 && packets==2);
    assert(station_wifi_step(tx,70,rx,&received,21)==0 && packets==2);
    ready=1;assert(station_wifi_step(tx,70,rx,&received,25)==0 && received==0 && packets==3 && transfers==1);
    station_wifi_get_diag(&diag);
    assert(diag.wait_count[0]==1 && diag.wait_max_ms[0]==4);
    assert(diag.wait_count[1]==1 && diag.wait_max_ms[1]==7);
    assert(diag.wait_count[2]==1 && diag.wait_max_ms[2]==11);
    assert(diag.poll_gap_max_ms==7 && diag.submitted==1 && diag.acknowledged==1);
    assert(diag.ready==1 && diag.phase==0 && diag.errors==2);
    oversized=1;assert(station_wifi_step(tx,70,rx,&received,26)==70 && received==0);
    oversized=0;
    assert(station_wifi_step(tx,70,rx,&received,27)==0 && packets==5);
    bad_ack=1;
    assert(station_wifi_step(tx,70,rx,&received,28)==0 && packets==6);
    bad_ack=0;
    assert(station_wifi_step(tx,70,rx,&received,29)==70 && received==3);
    ready=0;
    assert(station_wifi_step(tx,0,rx,&received,30)==0);
    assert(station_wifi_step(tx,0,rx,&received,1030)==0 && packets==7);
    station_wifi_get_diag(&diag);
    assert(diag.ready==1 && diag.phase==0 && diag.errors==5);
    assert(diag.wait_max_ms[1]==1000);
    ready=1;phase=0;
    assert(station_wifi_step(tx,70,rx,&received,1031)==70 && received==3);
#else
    assert(station_wifi_init()==0);
    assert(station_wifi_step(tx,70,rx,&received,0)==0 && received==0);
#endif
    return 0;
}
'''
    c=tmp_path/"wifi.c";c.write_text(source)
    exe=tmp_path/"wifi.exe"
    result=subprocess.run([gcc,"-std=c99","-Wall","-Wextra",f"-DSTATION_WIFI_ENABLED={enabled}","-I",str(tmp_path),"-I",str(ROOT/"code"),str(c),"-o",str(exe)],capture_output=True,text=True)
    assert result.returncode==0,result.stdout+result.stderr
    result=subprocess.run([str(exe)],capture_output=True,text=True)
    assert result.returncode==0,result.stdout+result.stderr
