/* Portable CRT declarations for the optional TCC in-memory test runner.
 * Its bundled Win64 math.h contains unsupported x87 inline-asm constraints.
 * These wrappers use the system double-precision CRT, not mock math results.
 * Only on the TCC include path; never included by the target firmware. */
#ifndef HOST_TCC_MATH_H
#define HOST_TCC_MATH_H
#include <float.h>
double fabs(double);
double sqrt(double);
double sin(double);
double cos(double);
double tan(double);
double atan2(double, double);
double asin(double);
double acos(double);
double atan(double);
double exp(double);
double log(double);
double pow(double, double);
double floor(double);
double ceil(double);
double fmod(double, double);
static inline float fabsf(float x) { return (float)fabs(x); }
static inline float sqrtf(float x) { return (float)sqrt(x); }
static inline float sinf(float x) { return (float)sin(x); }
static inline float cosf(float x) { return (float)cos(x); }
static inline float tanf(float x) { return (float)tan(x); }
static inline float atan2f(float y,float x) { return (float)atan2(y,x); }
static inline float asinf(float x) { return (float)asin(x); }
static inline float acosf(float x) { return (float)acos(x); }
static inline float atanf(float x) { return (float)atan(x); }
static inline float expf(float x) { return (float)exp(x); }
static inline float logf(float x) { return (float)log(x); }
static inline float powf(float x,float y) { return (float)pow(x,y); }
static inline float floorf(float x) { return (float)floor(x); }
static inline float ceilf(float x) { return (float)ceil(x); }
static inline float fmodf(float x,float y) { return (float)fmod(x,y); }
static inline float fmaxf(float x,float y) {
    if(x!=x)return y;if(y!=y)return x;return x>y?x:y;
}
static inline float fminf(float x,float y) {
    if(x!=x)return y;if(y!=y)return x;return x<y?x:y;
}
#define NAN (0.0f/0.0f)
#define INFINITY (1.0f/0.0f)
#define isfinite(x) ((x)==(x) && (x)<=DBL_MAX && (x)>=-DBL_MAX)
#define isnan(x) ((x)!=(x))
#endif
