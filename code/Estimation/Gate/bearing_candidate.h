#ifndef BEARING_CANDIDATE_H
#define BEARING_CANDIDATE_H
#include <stdint.h>
#define BC_STARTS 8U /* eight relative signs; constant-time linear candidates */
#ifndef BC_LINEAR_SIGN_LIMIT
#define BC_LINEAR_SIGN_LIMIT BC_STARTS
#endif
enum {
 BC_INPUT_BAD=1U,BC_NO_SIGNAL=2U,BC_WEAK=4U,BC_EXTRAPOLATED=8U,
 BC_BOUND=16U,BC_ALTERNATIVE=32U,BC_LIMIT=64U,BC_NUMERIC=128U,
 BC_ADC_RANGE=256U,BC_SATURATED=512U,BC_STALE=1024U,BC_DISABLED=2048U,
 BC_LINEAR_FAST=4096U,BC_TRACKED_BRANCH=8192U,BC_SIGN_UNCERTAIN=16384U,BC_FIELD_INCOHERENT=32768U
};
typedef struct {
 double bearing_deg,field_norm,cost,adc_error_percent,prediction[4];
 double alternative_deg,alternative_gap,alternative_separation;
 uint32_t flags,evaluations,iterations;
 uint8_t available,success,weak_mask,extrapolated_mask,saturated_mask;
} BC_Result;
typedef struct {
 BC_Result result;
 float observed[4],projection[4],sigma[4],body[8][3],level[8][3],costs[8];
 unsigned best,start,nfev,iterations,total_evaluations,total_iterations;
 uint8_t done;
} BC_Solver;
typedef struct {
 float angle_deg,yaw_deg,world_rate_dps;
 uint32_t timestamp_us;
 uint8_t initialized;
} BC_Track;
/* RFU; right-positive local field angle, modulo180. Pure matrix reconstruction.
 * No optimization, yaw/position input, PID or hardware calls in BC_Begin/Run. */
double BC_Response(unsigned channel,double q,double *derivative);
double BC_InverseResponse(unsigned channel,double adc);
void BC_Forward(double beta_deg,double amplitude,double pitch_deg,double roll_deg,double prediction[4]);
void BC_ForwardVector(const double level[3],double pitch_deg,double roll_deg,double prediction[4]);
void BC_Begin(BC_Solver *solver,const double adc[4],double pitch_deg,double roll_deg);
int BC_MathStep(BC_Solver *solver); /* already complete; compatibility API */
void BC_Run(const double adc[4],double pitch_deg,double roll_deg,BC_Result *result);
/* Relative yaw only helps select physically comparable signs; never a bearing substitute. */
void BC_TrackApply(BC_Solver *solver,BC_Track *track,float yaw_deg,uint32_t timestamp_us);
#endif
