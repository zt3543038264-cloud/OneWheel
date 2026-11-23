"""Exercise the Qcar foreground worker with a mock TC264 ADC and real solver."""
from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[1]
GATE = ROOT / "code/Estimation/Gate"


def test_gate_acquisition_and_bearing(tmp_path):
    vadc = tmp_path / "Vadc/Std/IfxVadc.h"
    vadc.parent.mkdir(parents=True)
    vadc.write_text("""
typedef union { unsigned U; struct { unsigned data:31; unsigned VF:1; } B; } Ifx_VADC_RES;
typedef struct { Ifx_VADC_RES result[4]; } mock_group;
extern struct mock_vadc { mock_group G[1]; } MODULE_VADC;
static inline Ifx_VADC_RES IfxVadc_getResult(mock_group *g, unsigned ch) {
    Ifx_VADC_RES r = g->result[ch];
    g->result[ch].B.VF = 0;
    return r;
}
""")
    (tmp_path / "zf_driver_adc.h").write_text("""
typedef enum {ADC0_CH0_A0, ADC0_CH1_A1, ADC0_CH2_A2, ADC0_CH3_A3} adc_channel_enum;
enum {ADC_12BIT = 2};
void adc_init(adc_channel_enum channel, int bits);
""")
    main = tmp_path / "test.c"
    main.write_text(r"""
#include "gate.h"
#include "bearing_candidate.h"
#include "Controller/control.h"
#include "Estimation/Attitude/attitude.h"
#include "Vadc/Std/IfxVadc.h"
#include "zf_driver_adc.h"
#include <assert.h>
#include <math.h>

volatile uint32 g_control_uptime_ms;
volatile uint8 g_imu_ok = 1;
attitude_t att;
struct mock_vadc MODULE_VADC;
unsigned configured;
uint8 attitude_converged(void) { return g_imu_ok; }
void adc_init(adc_channel_enum ch, int bits) {
    assert(bits == ADC_12BIT);
    configured |= 1u << ch;
}
static void feed(const double values[4], unsigned mask) {
    for (unsigned i = 0; i < 4; ++i) {
        assert(values[i] >= 0 && values[i] < 4096);
        MODULE_VADC.G[0].result[i].U = (unsigned)(values[i]+0.5);
        MODULE_VADC.G[0].result[i].B.VF = (mask >> i) & 1;
    }
    gate_poll();
}
int main(void) {
    gate_observation_t out;
    double adc[4], zero[4] = {0}, spike[4] = {4000,4000,4000,4000};
    g_control_uptime_ms = UINT32_MAX - 5u;
    gate_init();
    assert(configured == 15 && !gate_get(&out));
    att.pitch = -14; att.roll = -9; att.yaw_wrapped = 35;
    BC_Forward(32, 0.8, 14, -9, adc);
    ++g_control_uptime_ms;
    feed(adc, 7); /* one invalid channel */
    assert(!gate_get(&out));
    feed(adc, 15); /* no replay in the same tick */
    assert(!gate_get(&out));
    ++g_control_uptime_ms;
    feed(adc, 15);
    assert(gate_get(&out) && out.available && out.sample_seq == 1);
    assert(fabs(out.bearing_deg - 32) < 1 && out.yaw_deg == -35);
    for (int i = 1; i <= 10; ++i) {
        ++g_control_uptime_ms;
        feed(i == 10 ? spike : adc, 15);
        assert(gate_get(&out));
        assert(out.sample_seq == (i == 10 ? 11u : 1u));
    }
    assert(out.raw[0] == 4000 && fabs(out.filtered[0]-adc[0]) < 1);
    assert(out.available && fabs(out.bearing_deg - 32) < 1);
    g_control_uptime_ms += 80;
    feed(adc, 7);
    assert(gate_get(&out) && out.available && out.age_ms == 80);
    ++g_control_uptime_ms;
    assert(gate_get(&out) && !out.available && (out.flags & BC_STALE) && isnan(out.bearing_deg));
    feed(zero, 15); /* fresh frame after gap must not retain old filter values */
    assert(gate_get(&out) && !out.available && (out.flags & BC_NO_SIGNAL) && out.age_ms == 0);
    g_control_uptime_ms += 10;
    g_imu_ok = 0;
    feed(adc, 15);
    assert(gate_get(&out) && !out.available && (out.flags & BC_INPUT_BAD));
    g_control_uptime_ms += 10;
    g_imu_ok = 1;
    BC_Forward(-27, 0.8, 14, -9, adc);
    feed(adc, 15);
    assert(gate_get(&out) && out.available && fabs(out.bearing_deg + 27) < 1);
    return 0;
}
""")
    exe = tmp_path / "gate.exe"
    includes = [tmp_path, ROOT / "tools/unicycle_station/tests/firmware_host", ROOT / "code", GATE]
    subprocess.run(["gcc", "-std=c99", *["-I" + str(p) for p in includes],
                    str(main), *map(str, GATE.glob("*.c")), "-lm", "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True, timeout=10)


def test_gate_menu_snapshot_display(tmp_path):
    from test_linked_menu import function

    menu = (ROOT / "code/Menu/menu.c").read_text(encoding="utf-8")
    source = r"""
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "gate.h"
#include "bearing_candidate.h"
typedef uint16_t uint16;
"""
    source += menu[menu.index("#define UI_BLACK"):menu.index("typedef enum")]
    source += r"""
static uint32_t s_last_live_ms, g_control_uptime_ms;
static gate_observation_t sample;
static int seen, reads;
static char screen[320][31];
static int gate_read_mock(gate_observation_t *out) {
    ++reads;
    if (seen) *out = sample;
    return seen;
}
#define gate_get gate_read_mock
static void ui_text(unsigned x, unsigned y, const char *s, unsigned color) {
    (void)color;
    assert(y+16 <= 320 && x/8+strlen(s) <= 30);
    memcpy(screen[y]+x/8, s, strlen(s));
}
static void ui_line(unsigned y, const char *s, unsigned color) {
    memset(screen[y],' ',30);
    ui_text(0,y,s,color);
}
static void ui_header(const char *s,unsigned idx,unsigned total) { (void)s;(void)idx;(void)total; }
static void draw_status(void) {}
"""
    source += function(menu, "static void render_gate_live(void)")
    source += function(menu, "static void render_gate(void)")
    source += r"""
int main(void) {
    for (int y=0;y<320;y++) memset(screen[y],' ',30);
    render_gate();
    assert(reads==1 && strstr(screen[150],"WAITING FOR ADC"));
    seen=1; sample.available=1; sample.bearing_deg=12.5f; sample.field_norm=.125f;
    for (int i=0;i<4;i++) { sample.raw[i]=4095-i; sample.filtered[i]=1000.5f+i; }
    render_gate_live();
    assert(reads==2 && strstr(screen[150],"VALID"));
    for (int i=0;i<4;i++) {
        unsigned ch,raw; float value;
        assert(sscanf(screen[54+i*20],"AN%u %*s %u %f",&ch,&raw,&value)==3);
        assert(ch==(unsigned)i && raw==sample.raw[i] && value==sample.filtered[i]);
    }
    assert(strstr(screen[178],"+12.5 deg") && strstr(screen[198],"0.125"));
    sample.bearing_deg=-4.2f; sample.flags=BC_SIGN_UNCERTAIN;
    render_gate_live();
    assert(strstr(screen[178],"-4.2 deg") && strstr(screen[150],"VALID / SIGN UNCERTAIN"));
    sample.available=0; sample.age_ms=81; sample.flags=BC_STALE;
    render_gate_live();
    assert(strstr(screen[150],"INVALID / STALE") && strstr(screen[218],"81 ms"));
    assert(strstr(screen[54],"4095") && strstr(screen[178],"-- deg") && strstr(screen[198],"--"));
    assert(!strstr(screen[178],"-4.2") && !strstr(screen[198],"0.125"));
    sample.flags=BC_NO_SIGNAL; render_gate_live();
    assert(strstr(screen[150],"INVALID / NO SIGNAL"));
    return 0;
}
"""
    main = tmp_path / "menu_gate.c"
    main.write_text(source, encoding="utf-8")
    exe = tmp_path / "menu_gate.exe"
    subprocess.run(["gcc", "-std=c99", "-I" + str(GATE), str(main), "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True, timeout=10)
