#ifndef ONEWHEEL_FLYWHEEL_LAYOUT_H
#define ONEWHEEL_FLYWHEEL_LAYOUT_H
#include <stdint.h>
#include "config.h"
#if QCAR_FLYWHEEL_LAYOUT == QCAR_FLYWHEEL_PARALLEL
#define FLYWHEEL_BALANCE_LABEL "flywheel pair"
#elif QCAR_VERTICAL_FLYWHEEL_CHANNEL == 0
#define FLYWHEEL_BALANCE_LABEL "vertical L"
#else
#define FLYWHEEL_BALANCE_LABEL "vertical R"
#endif

/* Logical commands BEFORE the independent final UART direction signs.
 * Keep the original per-channel polarity (+u left, -u right), while removing
 * the horizontal wheel from all upright modes on the orthogonal chassis. */
static inline void flywheel_balance_mix(int16_t output, int16_t *left, int16_t *right)
{
#if QCAR_FLYWHEEL_LAYOUT == QCAR_FLYWHEEL_PARALLEL
    *left=output; *right=(int16_t)-output;
#elif QCAR_VERTICAL_FLYWHEEL_CHANNEL == 0
    *left=output; *right=0;
#else
    *left=0; *right=(int16_t)-output;
#endif
}

/* Legacy speed-loop units: raw speed * 0.01. A parallel pair used its mean
 * signed speed; a single vertical wheel uses its own full measurement.
 * The independent horizontal wheel must never bias the balance target.
 * These raw units are not asserted to be calibrated wheel RPM. */
static inline float flywheel_balance_speed(float left, float right)
{
#if QCAR_FLYWHEEL_LAYOUT == QCAR_FLYWHEEL_PARALLEL
    return (left-right)*0.005f;
#elif QCAR_VERTICAL_FLYWHEEL_CHANNEL == 0
    (void)right;
    return left*0.01f;
#else
    (void)left;
    return -right*0.01f;
#endif
}
#endif
