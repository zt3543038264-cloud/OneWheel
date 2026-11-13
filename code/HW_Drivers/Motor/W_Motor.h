#ifndef W_MOTOR_H_
#define W_MOTOR_H_
#include "zf_common_headfile.h"

/* Ordinary control paths are permanently zero-only in this commissioning
 * build. Release does not release the hardware or enable a bench pulse. */
void W_Motor_Init(void);
void W_Motor_SetDuty(int32 motor1_duty, int32 motor2_duty);
void W_Motor_Stop(void);     /* Explicit emergency stop: cancel grant + zero. */
void W_Motor_Release(void);
/* Idle observation zero, preserves a driver-local KEY1 grant. Do not call
 * while BenchBusy: an unexpected call aborts the pulse, rather than ignoring it. */
void W_Motor_KeepZero(void);
void W_Motor_RxHandler(void);
void W_Motor_TxHandler(void);
void W_Motor_Tick1ms(void);
int16 W_Motor_GetSpeed1(void);
int16 W_Motor_GetSpeed2(void);
uint8 W_Motor_LinkLost(void); /* Legacy RPM age; never evidence of SAFE identity. */

#define W_BENCH_IDLE          0u
#define W_BENCH_WAIT_ZERO     1u
#define W_BENCH_WAIT_ARM      2u
#define W_BENCH_PULSE         3u
#define W_BENCH_LATCHED       4u
#define W_BENCH_REASON_NONE      0u
#define W_BENCH_REASON_STOP      1u
#define W_BENCH_REASON_INVALID   2u
#define W_BENCH_REASON_LINK      3u
#define W_BENCH_REASON_TIMEOUT   4u
#define W_BENCH_REASON_SESSION   5u
#define W_BENCH_REASON_DRIVER    6u
#define W_BENCH_REASON_PROTOCOL  7u
#define W_BENCH_REASON_COMPLETE  8u
#define W_BENCH_MAX_DUTY       500
#define W_BENCH_MAX_MS         150u

typedef struct {
    uint8 host_state, driver_state, ready, busy;
    uint8 reason, driver_reason;
    uint32 session, last_seq;
    int16 right_duty;       /* Raw driver B channel, not MOTOR_DIR_B transformed. */
    uint16 elapsed_ms;
} w_motor_bench_status_t;

/* Explicit one-shot manual B/right-only pulse. Returns 1 only when a fresh
 * SAFE v1 capability1 LOCAL_READY grant was accepted. No auto retry/rearm.
 * Starts with ZERO only; the controller must validate current sensors before
 * EVERY BenchService call. A/C must remain hard zero in that controller branch.
 * expected_session must match the fresh grant displayed to the operator.
 * Raw duty is signed [-500,500], excluding0; duration is [1,150] real STM ms. */
uint8 W_Motor_BenchRequestB(int16 raw_duty, uint16 duration_ms, uint32 expected_session);
/* May advance zero -> arm -> duty ONLY after controller safety validation. */
void W_Motor_BenchService(void);
/* Tick1ms independently checks deadlines/freshness and can only stop, never
 * arm or energize. Foreground may call Service after the same safety checks. */
void W_Motor_BenchAbort(uint8 reason);
uint8 W_Motor_BenchBusy(void);
void W_Motor_BenchReadStatus(w_motor_bench_status_t *out);
#endif
