#include "syh_filter.h"
#include <string.h>

/* 三数中值。 */
static float Syh_Median3(float a, float b, float c)
{
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { float t = b; b = c; c = t; }
    if (a > b) { b = a; }
    return b;
}

void SyhFilter_Reset(SyhFilter *f)
{
    if (f == NULL) return;
    memset(f, 0, sizeof(*f));
}

void SyhFilter_Push(SyhFilter *f, const float raw[4], float out[4])
{
    unsigned ch, k, n;
    if (f == NULL || raw == NULL || out == NULL) return;

    for (ch = 0U; ch < SYH_CH_MAX; ch++)
    {
        /* 1) 写本帧到中值环形缓冲 */
        f->med_buf[ch][f->med_head[ch]] = raw[ch];
        f->med_head[ch] = (f->med_head[ch] + 1U) % SYH_MED_MAX;
        if (f->med_count[ch] < SYH_MED_MAX) f->med_count[ch]++;

        if (f->med_count[ch] >= SYH_MED_MAX)
        {
            /* 2) 三帧中值去尖峰 */
            float med = Syh_Median3(f->med_buf[ch][0], f->med_buf[ch][1], f->med_buf[ch][2]);

            /* 3) 中值输出入滑动均值窗口，4) 求均值（未满窗用运行平均） */
            f->mean_buf[ch][f->mean_head[ch]] = med;
            f->mean_head[ch] = (f->mean_head[ch] + 1U) % SYH_MEAN_MAX;
            if (f->mean_count[ch] < SYH_MEAN_MAX) f->mean_count[ch]++;
            n = f->mean_count[ch];
            {
                float sum = 0.0f;
                for (k = 0U; k < n; k++) sum += f->mean_buf[ch][k];
                out[ch] = sum / (float)n;
            }
        }
        else
        {
            /* 中值窗口未满：用已有帧的平均作过渡输出，避免全 0 打乱解算 */
            n = f->med_count[ch];
            {
                float sum = 0.0f;
                for (k = 0U; k < n; k++) sum += f->med_buf[ch][k];
                out[ch] = (n > 0U) ? (sum / (float)n) : 0.0f;
            }
        }
    }
}
