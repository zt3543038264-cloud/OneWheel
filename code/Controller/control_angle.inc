/* Included only by control.c. Public angle PID values retain the original
 * 5 ms reference, so Flash/menu/cfg values are portable between trial rates.
 * Only the three angle loops use this adapter; body-rate/speed signs and
 * gains remain owned by their existing paths. */
static void angle_pid_apply_gains(pid_t *p, float kp, float ki, float kd, float imax)
{
    p->kp = kp;
    p->ki = ki * ANGLE_PID_PERIOD_SCALE;
    p->kd = kd / ANGLE_PID_PERIOD_SCALE;
    p->imax = imax / ANGLE_PID_PERIOD_SCALE;
}

static void angle_pid_init(pid_t *p, float kp, float ki, float kd, float imax)
{
    angle_pid_apply_gains(p, kp, ki, kd, imax);
    pid_reset(p);
}
