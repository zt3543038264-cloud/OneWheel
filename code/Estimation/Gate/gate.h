#ifndef QCAR_GATE_H
#define QCAR_GATE_H

#include <stdint.h>

/* Frozen observation in model channel order (up, front, +120 deg, -120 deg).
 * available describes a fresh mathematical estimate, not a motion command. */
typedef struct {
    uint32_t sample_seq, sample_ms, age_ms, flags;
    uint16_t raw[4];
    float filtered[4];
    float pitch_deg, roll_deg, yaw_deg; /* converted to the solver convention */
    float bearing_deg, field_norm;
    uint8_t available;
} gate_observation_t;

/* CPU0 foreground only. Poll once per loop; get returns 0 until first frame. */
void gate_init(void);
void gate_poll(void);
int gate_get(gate_observation_t *out);

#endif
