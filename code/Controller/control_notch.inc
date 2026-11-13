/* Included only by control.c.
 *
 * Roll 角速度反馈陷波器：二阶 RBJ notch + 深度混合。
 *   全通带增益 1，f0 处为 0；深度 m = 1 - 10^(db/20)，db<=0。
 *   y = (1-m)*x + m*notch(x)
 * db=0 或参数非法时严格直通：不执行差分方程、清空状态，输出与未加滤波逐位相同。
 * 只用于测量反馈 att.roll_rate，不作用于角度环输出或最终 duty。
 */
#define ROLL_NOTCH_FS_HZ        (1000.0f)                       // Roll 角速度内环 1 kHz
#define ROLL_NOTCH_PI           (3.14159265358979323846f)

typedef struct
{
    float b0, b1, b2;                   // 分子系数（已按 a0 归一）
    float a1, a2;                       // 分母系数（已按 a0 归一）
    float x1, x2;                       // 输入历史
    float y1, y2;                       // 纯 notch 输出历史
    float cached_hz, cached_q, cached_db; // 上次参与计算的参数
    float mix;                          // 深度混合系数 m
    float last;                         // 本拍送进角速度环的值，供诊断
    uint8 cached_valid;                 // cached_* 已初始化
    uint8 bypass;                       // 1=严格直通
} roll_notch_t;

static roll_notch_t s_roll_notch;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     清空 notch 的输入/输出历史，启动、停车和参数变化时调用
// 参数说明     void
// 返回参数     void
// 使用示例     roll_notch_reset_state();
//-------------------------------------------------------------------------------------------------------------------
static void roll_notch_reset_state(void)
{
    s_roll_notch.x1 = 0.0f;
    s_roll_notch.x2 = 0.0f;
    s_roll_notch.y1 = 0.0f;
    s_roll_notch.y2 = 0.0f;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     判断 notch 参数是否可用；越界或非有限值一律按直通处理
// 参数说明     hz/q/db         中心频率(Hz)、品质因数、深度(dB)
// 返回参数     uint8           1=可计算系数 0=非法
// 使用示例     if (!roll_notch_params_valid(hz, q, db)) ...
//-------------------------------------------------------------------------------------------------------------------
static uint8 roll_notch_params_valid(float hz, float q, float db)
{
    if (!ctrl_is_finite(hz) || !ctrl_is_finite(q) || !ctrl_is_finite(db)) return 0;
    if (hz < R_RATE_NOTCH_HZ_MIN || hz > R_RATE_NOTCH_HZ_MAX) return 0;
    if (q < R_RATE_NOTCH_Q_MIN || q > R_RATE_NOTCH_Q_MAX) return 0;
    if (db < R_RATE_NOTCH_DB_MIN || db > R_RATE_NOTCH_DB_MAX) return 0;
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     参数变化时重算 biquad 系数和深度，并清状态；只有这里调用 tanf
// 参数说明     hz/q/db         中心频率(Hz)、品质因数、深度(dB)
// 返回参数     void
// 使用示例     roll_notch_update(R_RATE_NOTCH_HZ, R_RATE_NOTCH_Q, R_RATE_NOTCH_DB);
//-------------------------------------------------------------------------------------------------------------------
static void roll_notch_update(float hz, float q, float db)
{
    float k, k2, denom;

    s_roll_notch.cached_hz = hz;
    s_roll_notch.cached_q  = q;
    s_roll_notch.cached_db = db;
    s_roll_notch.cached_valid = 1;
    roll_notch_reset_state();

    // db=0 与非法参数都严格直通：不计算系数，状态保持清零
    if (!roll_notch_params_valid(hz, q, db) || db >= 0.0f)
    {
        s_roll_notch.bypass = 1;
        s_roll_notch.mix = 0.0f;
        return;
    }

    // 预畸变双线性变换的 notch：K = tan(pi*f0/fs)
    // b0=b2=(1+K^2)/D，b1=a1=2(K^2-1)/D，a2=(1+K^2-K/Q)/D，D=1+K/Q+K^2
    k = tanf(ROLL_NOTCH_PI * hz / ROLL_NOTCH_FS_HZ);
    k2 = k * k;
    denom = 1.0f + k / q + k2;
    if (!ctrl_is_finite(denom) || denom <= 0.0f)
    {
        s_roll_notch.bypass = 1;
        s_roll_notch.mix = 0.0f;
        return;
    }

    s_roll_notch.b0 = (1.0f + k2) / denom;
    s_roll_notch.b1 = 2.0f * (k2 - 1.0f) / denom;
    s_roll_notch.b2 = s_roll_notch.b0;
    s_roll_notch.a1 = s_roll_notch.b1;
    s_roll_notch.a2 = (1.0f + k2 - k / q) / denom;

    // db<=0，m=0 即旁路，m=1 为全深度
    s_roll_notch.mix = 1.0f - powf(10.0f, db / 20.0f);
    s_roll_notch.bypass = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     Roll 角速度反馈陷波，每 1ms 调用一次；参数变化自动重算
// 参数说明     x               原始 att.roll_rate(deg/s)
// 返回参数     float           滤波后的反馈(deg/s)；旁路时原样返回 x
// 使用示例     rate = roll_rate_notch_apply(att.roll_rate);
//-------------------------------------------------------------------------------------------------------------------
static float roll_rate_notch_apply(float x)
{
    float hz = R_RATE_NOTCH_HZ;
    float q  = R_RATE_NOTCH_Q;
    float db = R_RATE_NOTCH_DB;
    float notch, y;

    if (!s_roll_notch.cached_valid ||
        hz != s_roll_notch.cached_hz ||
        q  != s_roll_notch.cached_q ||
        db != s_roll_notch.cached_db)
        roll_notch_update(hz, q, db);

    if (s_roll_notch.bypass)
    {
        s_roll_notch.last = x;          // 严格直通，不碰历史状态
        return x;
    }

    if (!ctrl_is_finite(x))
    {
        roll_notch_reset_state();
        s_roll_notch.last = x;
        return x;
    }

    notch = s_roll_notch.b0 * x +
            s_roll_notch.b1 * s_roll_notch.x1 +
            s_roll_notch.b2 * s_roll_notch.x2 -
            s_roll_notch.a1 * s_roll_notch.y1 -
            s_roll_notch.a2 * s_roll_notch.y2;

    if (!ctrl_is_finite(notch))
    {
        roll_notch_reset_state();
        s_roll_notch.last = x;
        return x;
    }

    s_roll_notch.x2 = s_roll_notch.x1;
    s_roll_notch.x1 = x;
    s_roll_notch.y2 = s_roll_notch.y1;
    s_roll_notch.y1 = notch;

    // 深度混合：db=0 时 m=0，但那条路径已在上面提前返回
    y = (1.0f - s_roll_notch.mix) * x + s_roll_notch.mix * notch;
    if (!ctrl_is_finite(y))
    {
        roll_notch_reset_state();
        s_roll_notch.last = x;
        return x;
    }

    s_roll_notch.last = y;
    return y;
}
