#include "gate.h"
#include "bearing_candidate.h"
#include "syh_filter.h"
#include "Controller/control.h"
#include "Estimation/Attitude/attitude.h"
#include "Vadc/Std/IfxVadc.h"
#include "zf_driver_adc.h"
#include <math.h>

#define GATE_FILTER_GAP_MS 4u
#define GATE_SOLVE_INTERVAL_MS 10u
#define GATE_MAX_AGE_MS 80u

/* Wire the receiver in model order. All four inputs use group 0, with one
 * background result register per channel as configured by adc_init(). */
static const adc_channel_enum channels[4] = {
    ADC0_CH0_A0, ADC0_CH1_A1, ADC0_CH2_A2, ADC0_CH3_A3
};
static SyhFilter filter;
static BC_Track track;
static BC_Solver solver;
static gate_observation_t frame;
static uint32_t last_poll_ms, last_sample_ms, sample_seq;
static uint8_t have_frame;

void gate_init(void)
{
    unsigned i;
    for (i = 0; i < 4; ++i) adc_init(channels[i], ADC_12BIT);
    SyhFilter_Reset(&filter);
    last_sample_ms = last_poll_ms = g_control_uptime_ms;
    track.initialized = have_frame = 0;
    sample_seq = 0;
}

void gate_poll(void)
{
    uint32_t now = g_control_uptime_ms;
    uint16_t raw[4];
    float input[4], filtered[4];
    double adc[4];
    uint32 irq;
    uint8_t complete = 1, imu_ready;
    unsigned i;

    if (now == last_poll_ms) return;
    /* Freeze the ADC reads, timestamp and attitude from the same control tick.
     * Read each register exactly once. No waiting for a conversion in this section. */
    irq = interrupt_global_disable();
    now = g_control_uptime_ms;
    last_poll_ms = now;
    for (i = 0; i < 4; ++i) {
        Ifx_VADC_RES result = IfxVadc_getResult(&MODULE_VADC.G[0], channels[i]);
        if (!result.B.VF) complete = 0;
        raw[i] = (uint16_t)(result.U & 0xfffu);
    }
    imu_ready = g_imu_ok && attitude_converged();
    /* Qcar X-forward/Y-left/Z-up, coil front aligned with active IMU front.
     * BC tilt uses nose-up and right-side-down; heading increases to the right. */
    float pitch = -att.pitch, roll = att.roll, yaw = -att.yaw_wrapped;
    interrupt_global_enable(irq);
    if (!complete) return;

    if ((uint32_t)(now - last_sample_ms) > GATE_FILTER_GAP_MS) {
        SyhFilter_Reset(&filter);
        track.initialized = 0;
    }
    last_sample_ms = now;
    ++sample_seq;
    for (i = 0; i < 4; ++i) input[i] = (float)raw[i];
    SyhFilter_Push(&filter, input, filtered);
    if (have_frame && (uint32_t)(now - frame.sample_ms) < GATE_SOLVE_INTERVAL_MS) return;

    for (i = 0; i < 4; ++i) {
        frame.raw[i] = raw[i];
        frame.filtered[i] = filtered[i];
        adc[i] = filtered[i];
    }
    BC_Begin(&solver, adc, imu_ready ? pitch : NAN, roll);
    BC_TrackApply(&solver, &track, yaw, now * 1000u);
    frame.sample_seq = sample_seq;
    frame.sample_ms = now;
    frame.pitch_deg = pitch;
    frame.roll_deg = roll;
    frame.yaw_deg = yaw;
    frame.bearing_deg = (float)solver.result.bearing_deg;
    frame.field_norm = (float)solver.result.field_norm;
    frame.flags = solver.result.flags;
    frame.available = solver.result.available;
    have_frame = 1;
}

int gate_get(gate_observation_t *out)
{
    if (!have_frame || !out) return 0;
    *out = frame;
    out->age_ms = (uint32_t)(g_control_uptime_ms - frame.sample_ms);
    if (out->age_ms > GATE_MAX_AGE_MS) {
        out->flags |= BC_STALE;
        out->available = 0;
        out->bearing_deg = NAN;
    }
    return 1;
}
