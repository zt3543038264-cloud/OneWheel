#ifndef SYH_FILTER_H_
#define SYH_FILTER_H_ /* 三帧中值去尖峰 + N 窗口滑动均值 级联滤波 */

#include <stdint.h>
#include <stddef.h>

#define SYH_MED_MAX      (3U)   /* 中值滤波窗口（帧） */
#define SYH_MEAN_MAX     (5U)   /* 滑动均值窗口（帧） */
#define SYH_CH_MAX       (4U)   /* 通道数 */

typedef struct
{
    float med_buf[SYH_CH_MAX][SYH_MED_MAX];  /* 每通道最近 N 帧原始码 */
    float mean_buf[SYH_CH_MAX][SYH_MEAN_MAX];/* 每通道最近 N 个中值输出 */
    unsigned med_head[SYH_CH_MAX];           /* 中值环形写指针 */
    unsigned med_count[SYH_CH_MAX];          /* 已收集中值帧数 */
    unsigned mean_head[SYH_CH_MAX];          /* 均值环形写指针 */
    unsigned mean_count[SYH_CH_MAX];         /* 已收集均值帧数 */
    float out[SYH_CH_MAX];                   /* 滑动均值输出 */
} SyhFilter;

/* 清空全部历史与输出为 0。 */
void SyhFilter_Reset(SyhFilter *f);

/* 推入一帧四路原始码，输出中值->均值的级联结果到 out[4]。 */
void SyhFilter_Push(SyhFilter *f, const float raw[4], float out[4]);

#endif /* SYH_FILTER_H_ */
