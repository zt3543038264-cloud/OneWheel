/* CPU0-only r10h acquisition. Included by imu.c, never compiled alone. */
static imu_data_t r10h_raw_body;
static uint8 r10h_sample_valid, r10h_identity_ticks;
static int16 r10h_signed16(const uint8 *p)
{ return (int16)((uint16)p[0] | ((uint16)p[1] << 8)); }

static uint8 r10h_read_body(void)
{
    uint8 bytes[12], j, all_ff=1, acc_zero=1;
    const char *reason = "IMU INVALID SAMPLE";
    int16 v[6];
    r10h_sample_valid=0;
    if (link_lost) return 0;
    if ((imu_read_reg(0x1Eu) & 3u) != 3u) { reason="IMU NOT READY"; goto invalid_sample; }
    if (!r10h_bus_read(IMU660RB_OUTX_L_G,bytes,sizeof(bytes))) { reason="IMU SPI READ FAIL"; goto invalid_sample; }
    for (j=0; j<12u; ++j)
    {
        if (bytes[j]!=0xFFu) all_ff=0;
        if (j>=6u && bytes[j]!=0u) acc_zero=0;
    }
    if (all_ff) { reason="IMU DATA ALL FF"; goto invalid_sample; }
    if (acc_zero) { reason="IMU ACC ALL ZERO"; goto invalid_sample; }
    for (j=0; j<6u; ++j)
    {
        v[j]=r10h_signed16(bytes+2u*j);
        if (v[j]>=32760 || v[j]<=-32760) { reason="IMU SATURATED"; goto invalid_sample; }
    }
    if (++r10h_identity_ticks>=100u)
    {
        r10h_identity_ticks=0;
        if (imu_read_reg(IMU660RB_WHO_AM_I)!=0x6Bu) { reason="IMU BAD CHIP ID"; goto invalid_sample; }
    }
    imu_sensor_to_body(imu660rb_gyro_transition(v[0]), imu660rb_gyro_transition(v[1]),
        imu660rb_gyro_transition(v[2]), &r10h_raw_body.gyro_x,&r10h_raw_body.gyro_y,&r10h_raw_body.gyro_z);
    imu_sensor_to_body(imu660rb_acc_transition(v[3]), imu660rb_acc_transition(v[4]),
        imu660rb_acc_transition(v[5]), &r10h_raw_body.acc_x,&r10h_raw_body.acc_y,&r10h_raw_body.acc_z);
    if (!isfinite(r10h_raw_body.gyro_x) || !isfinite(r10h_raw_body.gyro_y) || !isfinite(r10h_raw_body.gyro_z) ||
        !isfinite(r10h_raw_body.acc_x) || !isfinite(r10h_raw_body.acc_y) || !isfinite(r10h_raw_body.acc_z)) { reason="IMU NONFINITE"; goto invalid_sample; }
    if (link_lost) goto invalid_sample;
    r10h_sample_valid=1;
    return 1;
invalid_sample:
    imu_latch_fault(reason); /* Fixed-dt core needs explicit stopped reinit/calibration, never auto-resume. */
    return 0;
}
uint8 imu_get_raw_sample(imu_data_t *out)
{
    if (!out || !r10h_sample_valid || link_lost) return 0;
    *out=r10h_raw_body;
    return 1;
}
uint8 imu_get_startup_mean(imu_data_t *out)
{
    if (!out || calib_state!=IMU_CALIB_OK || link_lost) return 0;
    out->gyro_x=gyro_bias_x; out->gyro_y=gyro_bias_y; out->gyro_z=gyro_bias_z;
    out->acc_x=static_acc_x; out->acc_y=static_acc_y; out->acc_z=static_acc_z;
    return 1;
}
