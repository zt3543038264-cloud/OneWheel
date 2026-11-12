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
static int ready=1, phase, packets, transfers, writes, selected, oversized;
uint8 wifi_spi_init(char *ssid,char *password) {(void)password;assert(*ssid);return 0;}
uint8 wifi_spi_socket_connect(char *type,char *ip,char *port,char *local) {
    (void)ip;(void)port;(void)local;assert(!strcmp(type,"UDP"));return 0;
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
        reply.command=phase==2 ? WIFI_SPI_REPLY_OK:WIFI_SPI_REPLY_DATA_END;
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
    assert(station_wifi_init()==1);
    ready=0;assert(station_wifi_step(tx,70,rx,&received)==0 && received==0 && packets==0);
    ready=1;assert(station_wifi_step(tx,70,rx,&received)==70);
    assert(received==3 && !memcmp(rx,"abc",3) && packets==1 && writes==1);
    assert(station_wifi_step(tx,70,rx,&received)==0);
    assert(received==2 && !memcmp(rx,"de",2) && packets==2);
    assert(station_wifi_step(tx,70,rx,&received)==0 && received==0 && packets==3 && transfers==1);
    oversized=1;assert(station_wifi_step(tx,70,rx,&received)==-1 && received==0);
    assert(station_wifi_step(tx,70,rx,&received)==0 && packets==4);
#else
    assert(station_wifi_init()==0);
    assert(station_wifi_step(tx,70,rx,&received)==0 && received==0);
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
