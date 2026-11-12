#include "Estimation/Vision/vision_core.h"

#include <string.h>

#include "Estimation/Vision/image.h"
#include "Estimation/Vision/perspective.h"
#include "zf_device_mt9v03x.h"

typedef struct
{
    volatile uint32 seq;
    vision_result_t payload;
} vision_result_box_t;

typedef struct
{
    volatile uint32 seq;
    vision_feedback_t payload;
} vision_feedback_box_t;

#define VISION_COMMAND_CAPACITY 8u
typedef struct { vision_command_t command; uint16 value; } vision_command_entry_t;
typedef struct
{
    volatile uint32 head, tail;
    vision_command_entry_t entries[VISION_COMMAND_CAPACITY];
} vision_command_box_t;
typedef struct { volatile uint32 seq; vision_camera_status_t payload; } vision_camera_box_t;

#pragma section all "vision_shared"
static vision_result_box_t   s_result_box;
static vision_feedback_box_t s_feedback_box;
static vision_command_box_t  s_command_box;
static vision_camera_box_t s_camera_box;
static volatile uint8 s_camera_reconfiguring;
static volatile uint32       s_heartbeat;
static volatile uint8        s_display_request;
static volatile uint8        s_display_ready;
static volatile uint8        s_display_mode;
static volatile vision_core_state_t s_core_state;
static vision_display_frame_t s_display_frame;
#pragma section all restore

#pragma section all "cpu1_dsram"
static uint32 s_frame_seq;
static vision_camera_status_t s_camera_status;
static uint16 s_feedback_exposure;
static uint8 s_exposure_pending;
static uint16 s_camera_exposure;
static vision_feedback_t s_feedback_cache;
static uint8  s_ipm_calib_seq;      // 本核完成的标定次数，随结果发回 CPU0
static float  s_ipm_applied[9];     // 已经从 CPU0 快照载入过的矩阵，避免每帧重复载入
static uint8  s_ipm_collecting;     // 正在累计多帧直道标定点
static uint8  s_ipm_collect_count;  // 已累计的有效直道帧数
static float  s_ipm_point_sum[4];   // 四个标定列坐标累加值
static float  s_ipm_near_sum;       // 近端标定行累加值
static float  s_ipm_far_sum;        // 远端标定行累加值
static float  s_ipm_ratio_sum;      // 纵横尺度比累加值
static uint32 s_ipm_param_revision; // 已应用的 CPU0 参数修订号
#pragma section all restore

#pragma section code "vision_flash"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将整数压缩到图像列坐标范围
// 参数说明     value           原始列坐标
// 返回参数     uint8           0 到 IMG_W-1 的列坐标
// 使用示例     frame->left_line[row] = vision_clip_col(my_image.Left_Line[row]);
//-------------------------------------------------------------------------------------------------------------------
static uint8 vision_clip_col(int value)
{
    if (value < 0) value = 0;
    if (value >= IMG_W) value = IMG_W - 1;
    return (uint8)value;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取 CPU0 发布的反馈快照
// 参数说明     feedback        反馈结果输出地址
// 返回参数     uint8           1=读取成功 0=快照正在更新
// 使用示例     if (vision_feedback_read(&feedback)) { ... }
//-------------------------------------------------------------------------------------------------------------------
static uint8 vision_feedback_read(vision_feedback_t *feedback)
{
    uint32 seq_begin;
    uint32 seq_end;

    if (feedback == 0) return 0;
    seq_begin = s_feedback_box.seq;
    if (seq_begin == 0u || (seq_begin & 1u)) return 0;
    __dsync();
    *feedback = s_feedback_box.payload;
    __dsync();
    seq_end = s_feedback_box.seq;
    return (uint8)(seq_begin == seq_end && !(seq_end & 1u) && feedback->input_seq != 0u);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     载入 CPU1 在首份 CPU0 反馈到达前使用的视觉默认参数
// 参数说明     feedback        默认反馈输出地址
// 返回参数     void
// 使用示例     vision_feedback_defaults(&s_feedback_cache);
//-------------------------------------------------------------------------------------------------------------------
static void vision_feedback_defaults(vision_feedback_t *feedback)
{
    memset(feedback, 0, sizeof(*feedback));
    beacon_defaults(&feedback->beacon);
    feedback->speed_cross_mps = RUN_SPEED_CROSS_DEFAULT;
    feedback->speed_ring_mps = RUN_SPEED_RING_DEFAULT;
    feedback->speed_ramp_mps = RUN_SPEED_RAMP_DEFAULT;
    feedback->ring_angle = RING_ANGLE_DEFAULT;
    feedback->ring_s2_cnt_l = RING_S2_CNT_L_DEFAULT;
    feedback->ring_s2_cnt_r = RING_S2_CNT_R_DEFAULT;
    feedback->cam_exposure = CAM_EXPOSURE_DEFAULT;
    // 元素使能默认全关，CPU0 的首份快照到达前 CPU1 不跑任何元素
    feedback->elem_en_zebra = ELEM_EN_ZEBRA_DEFAULT;
    feedback->elem_en_cross = ELEM_EN_CROSS_DEFAULT;
    feedback->elem_en_ring = ELEM_EN_RING_DEFAULT;
    feedback->elem_en_ramp = ELEM_EN_RAMP_DEFAULT;
    feedback->err_front_row = ERR_FRONT_ROW_DEFAULT;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     发布一份完整视觉结果并更新帧序号
// 参数说明     result          待发布结果
// 返回参数     void
// 使用示例     vision_result_publish(&result);
//-------------------------------------------------------------------------------------------------------------------
static void vision_result_publish(const vision_result_t *result)
{
    uint32 seq = s_result_box.seq;

    s_result_box.seq = seq + 1u;
    __dsync();
    s_result_box.payload = *result;
    __dsync();
    s_result_box.seq = seq + 2u;
    __dsync();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取 CPU0 最新命令且跳过已执行命令
// 参数说明     command/value   命令与参数输出地址
// 返回参数     uint8           1=读取到新命令 0=无新命令
// 使用示例     if (vision_command_read(&command, &value)) { ... }
//-------------------------------------------------------------------------------------------------------------------
static uint8 vision_command_read(vision_command_t *command, uint16 *value)
{
    uint32 tail = s_command_box.tail;
    if (tail == s_command_box.head) return 0;
    __dsync();
    *command = s_command_box.entries[tail % VISION_COMMAND_CAPACITY].command;
    *value = s_command_box.entries[tail % VISION_COMMAND_CAPACITY].value;
    __dsync();
    if (*command == VISION_CMD_RESTART_CAMERA || *command == VISION_CMD_SET_PROFILE)
    {
        s_camera_reconfiguring = 1;
        __dsync();
    }
    s_command_box.tail = tail + 1u;
    return 1;
}

static void vision_camera_status_publish(void)
{
    uint32 seq = s_camera_box.seq;
    s_camera_status.vsync_count = mt9v03x_vsync_count;
    s_camera_status.dma_count = mt9v03x_dma_count;
    s_camera_status.busy_drop_count = mt9v03x_busy_drop_count;
    s_camera_status.exposure_requested = s_camera_exposure;
    s_camera_box.seq = seq + 1u;
    __dsync();
    s_camera_box.payload = s_camera_status;
    __dsync();
    s_camera_box.seq = seq + 2u;
    __dsync();
}

static void vision_exposure_apply(void)
{
    if (!s_exposure_pending || s_core_state != VISION_CORE_READY) return;
    s_camera_status.exposure_state = VISION_EXPOSURE_PENDING;
    vision_camera_status_publish();
    if (mt9v03x_set_exposure_time(s_camera_exposure) == 0u)
    {
        s_camera_status.exposure_applied = s_camera_exposure;
        s_camera_status.exposure_state = VISION_EXPOSURE_OK;
    }
    else s_camera_status.exposure_state = VISION_EXPOSURE_FAILED;
    s_exposure_pending = 0; /* retry only on a new request, never block every frame */
    vision_camera_status_publish();
}

static void vision_capture_status(void)
{
    const uint8 *gray = image_gray_data();
    uint32 i, sum = 0;
    uint8 lo = 255, hi = 0;
    for (i = 0; i < (uint32)(IMG_W * IMG_H); ++i)
    {
        uint8 v = gray[i];
        if (v < lo) lo = v;
        if (v > hi) hi = v;
        sum += v;
    }
    s_camera_status.frame_seq = ++s_frame_seq;
    s_camera_status.gray_min = lo;
    s_camera_status.gray_max = hi;
    s_camera_status.gray_mean = (uint8)((sum + IMG_W * IMG_H / 2u) / (IMG_W * IMG_H));
    s_camera_status.gray_valid = 1;
    vision_camera_status_publish();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     根据当前图像结果生成 CPU0 屏幕快照
// 参数说明     mode            快照显示模式
// 返回参数     void
// 使用示例     vision_display_publish(VISION_DISPLAY_BIN_LINE);
//-------------------------------------------------------------------------------------------------------------------
static void vision_display_publish(vision_display_mode_t mode)
{
    int row;

    if (s_display_ready || !s_display_request) return;
    s_display_frame.generation = s_camera_status.generation;
    s_display_frame.frame_seq = s_frame_seq;
    s_display_frame.threshold = (uint16)my_image.Threshold;
    s_display_frame.search_stop_line = (uint16)my_image.Search_Stop_Line;
    {
        // 标定页要看的东西：选中的两行、四个角点、以及选不出来时的原因
        float pts[4];
        float ratio;
        int rn, rf;
        ipm_pick_t why = image_ipm_pick_points(pts, &rn, &rf, &ratio);

        s_display_frame.ipm_pick = (uint8)why;
        s_display_frame.ipm_row_near = (uint8)rn;
        s_display_frame.ipm_row_far = (uint8)rf;
        for (row = 0; row < 4; row++)
            s_display_frame.ipm_col[row] = vision_clip_col((int)(pts[row] + 0.5f));
    }
    s_display_frame.mode = (uint8)mode;
    s_display_frame.track_valid = (uint8)my_image.Track_Valid;
    s_display_frame.active_elem = (uint8)g_elem_action.active_elem;
    s_display_frame.source_mode = (uint8)s_feedback_cache.beacon.mode;

    if (mode == VISION_DISPLAY_GRAY)
        image_copy_gray(&s_display_frame.pixels[0][0]);
    else
        memcpy(s_display_frame.pixels, my_image.image_two_value, sizeof(s_display_frame.pixels));

    for (row = 0; row < IMG_H; row++)
    {
        s_display_frame.left_line[row] = vision_clip_col(my_image.Left_Line[row]);
        s_display_frame.right_line[row] = vision_clip_col(my_image.Right_Line[row]);
        s_display_frame.mid_line[row] = vision_clip_col(my_image.Mid_Line[row]);
        s_display_frame.left_valid[row] = (uint8)!my_image.Left_Lost_Flag[row];
        s_display_frame.right_valid[row] = (uint8)!my_image.Right_Lost_Flag[row];
        s_display_frame.mid_valid[row] = (uint8)!my_image.Mid_Lost_Flag[row];
    }

    s_display_request = 0;
    __dsync();
    s_display_ready = 1;
    __dsync();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     执行摄像头初始化并发布工作状态
// 参数说明     void
// 返回参数     uint8           1=初始化成功 0=初始化失败
// 使用示例     if (!vision_camera_start()) { ... }
//-------------------------------------------------------------------------------------------------------------------
static uint8 vision_camera_start(void)
{
    uint8 ok;
    s_core_state = VISION_CORE_STARTING;
    s_camera_status.generation++;
    s_ipm_collecting = 0;
    s_ipm_collect_count = 0;
    s_camera_status.gray_valid = 0;
    s_camera_status.frame_seq = 0;
    s_camera_status.exposure_applied = 0;
    s_camera_status.exposure_state = VISION_EXPOSURE_PENDING;
    vision_camera_status_publish();
    beacon_reset();
    image_init();
    element_init();
    ok = (uint8)(mt9v03x_init_profile(s_camera_status.configured_fps, s_camera_exposure) == 0u);
    if (ok)
    {
        s_core_state = VISION_CORE_READY;
        s_exposure_pending = 1;
        vision_exposure_apply();
    }
    else
    {
        s_core_state = VISION_CORE_FAILED;
        s_camera_status.exposure_state = VISION_EXPOSURE_UNKNOWN;
        s_exposure_pending = 1;
    }
    vision_camera_status_publish();
    __dsync();
    s_camera_reconfiguring = 0;
    return ok;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     执行一条 CPU0 摄像头命令
// 参数说明     void
// 返回参数     void
// 使用示例     vision_command_service();
//-------------------------------------------------------------------------------------------------------------------
static void vision_command_service(void)
{
    vision_command_t command;
    uint16 value;
    if (!vision_command_read(&command, &value)) return;
    if (command == VISION_CMD_SET_EXPOSURE)
    {
        s_camera_exposure = value;
        s_camera_status.exposure_state = VISION_EXPOSURE_PENDING;
        s_exposure_pending = 1;
        vision_camera_status_publish();
    }
    else if (command == VISION_CMD_RESTART_CAMERA)
        (void)vision_camera_start();
    else if (command == VISION_CMD_SET_PROFILE)
    {
        s_camera_status.diagnostic = (uint8)value;
        s_camera_status.configured_fps = value ? CAM_DIAG_FPS : MT9V03X_FPS_DEF;
        s_camera_exposure = value ? CAM_DIAG_EXPOSURE : s_feedback_cache.cam_exposure;
        (void)vision_camera_start();
    }
    else if (command == VISION_CMD_CALIB_IPM && !s_camera_status.diagnostic &&
             s_feedback_cache.beacon.mode == 0 && s_core_state == VISION_CORE_READY)
    {
        memset(s_ipm_point_sum, 0, sizeof(s_ipm_point_sum));
        s_ipm_near_sum = 0.0f;
        s_ipm_far_sum = 0.0f;
        s_ipm_ratio_sum = 0.0f;
        s_ipm_collect_count = 0;
        s_ipm_collecting = 1;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     累计有效直道帧并完成逆透视标定
// 参数说明     void
// 返回参数     void
// 使用示例     vision_ipm_collect();
//-------------------------------------------------------------------------------------------------------------------
static void vision_ipm_collect(void)
{
    float pts[4];
    float ratio;
    int rn, rf, i;

    if (!s_ipm_collecting) return;
    if (image_ipm_pick_points(pts, &rn, &rf, &ratio) != IPM_PICK_OK)
        return;                                 // 跳过无效帧，保留已经累计的直道样本

    for (i = 0; i < 4; i++) s_ipm_point_sum[i] += pts[i];
    s_ipm_near_sum += (float)rn;
    s_ipm_far_sum += (float)rf;
    s_ipm_ratio_sum += ratio;
    s_ipm_collect_count++;
    if (s_ipm_collect_count < IPM_CALIB_FRAMES) return;

    for (i = 0; i < 4; i++) pts[i] = s_ipm_point_sum[i] / (float)s_ipm_collect_count;
    if (ipm_calibrate(pts[0], pts[1], s_ipm_near_sum / (float)s_ipm_collect_count,
                      pts[2], pts[3], s_ipm_far_sum / (float)s_ipm_collect_count,
                      s_ipm_ratio_sum / (float)s_ipm_collect_count))
    {
        ipm_store(s_ipm_applied);
        s_ipm_calib_seq++;
    }
    s_ipm_collecting = 0;
    s_ipm_collect_count = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     在 CPU1 初始化摄像头与普通图像算法
// 参数说明     void
// 返回参数     void
// 使用示例     vision_core_init();
//-------------------------------------------------------------------------------------------------------------------
void vision_core_init(void)
{
    vision_feedback_t feedback;

    memset(&s_display_frame, 0, sizeof(s_display_frame));
    vision_feedback_defaults(&s_feedback_cache);
    if (vision_feedback_read(&feedback))
        s_feedback_cache = feedback;
    s_frame_seq = 0;
    s_ipm_calib_seq = 0;
    s_ipm_collecting = 0;
    s_ipm_collect_count = 0;
    s_ipm_ratio_sum = 0.0f;
    memset(s_ipm_point_sum, 0, sizeof(s_ipm_point_sum));
    memset(s_ipm_applied, 0, sizeof(s_ipm_applied));
    memset(&s_camera_status, 0, sizeof(s_camera_status));
    s_camera_status.configured_fps = MT9V03X_FPS_DEF;
    s_camera_exposure = s_feedback_cache.cam_exposure;
    s_feedback_exposure = s_camera_exposure;
    s_exposure_pending = 1;
    s_heartbeat = 0;
    s_display_request = 0;
    s_display_ready = 0;
    s_display_mode = (uint8)VISION_DISPLAY_BIN_LINE;
    s_core_state = VISION_CORE_OFF;
    vision_camera_status_publish();
    if (QCAR_CAMERA_AUTO_START) (void)vision_camera_start();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     在 CPU1 主循环处理命令与一帧新图像
// 参数说明     void
// 返回参数     void
// 使用示例     vision_core_run();
//-------------------------------------------------------------------------------------------------------------------
void vision_core_run(void)
{
    vision_feedback_t feedback;
    vision_result_t result;
    element_motion_t motion;
    float base_error = 0.0f;
    uint8 base_track_valid = 0;

    s_heartbeat++;
    /* Read parameters without requiring a frame: exposure recovery works while
     * the camera is dark or stopped. A direct queued command has final priority. */
    if (vision_feedback_read(&feedback) && feedback.input_seq != s_feedback_cache.input_seq)
    {
        s_feedback_cache = feedback;
        if (!s_camera_status.diagnostic && feedback.cam_exposure != s_feedback_exposure)
        {
            s_camera_exposure = feedback.cam_exposure;
            s_exposure_pending = 1;
            s_camera_status.exposure_state = VISION_EXPOSURE_PENDING;
        }
        s_feedback_exposure = feedback.cam_exposure;
        vision_camera_status_publish();
    }
    feedback = s_feedback_cache;
    vision_command_service();
    vision_exposure_apply();
    if (s_core_state != VISION_CORE_READY || !image_grab()) return;
    vision_capture_status();

    /* Optional diagnostic profile follows the reference's raw-camera path.
     * Publish camera/FPS health but never an eligible Run target. */
    if (s_camera_status.diagnostic)
    {
        int row, col;
        const uint8 *gray = image_gray_data();
        my_image.Track_Valid = 0;
        my_image.Threshold = 128;
        my_image.Search_Stop_Line = 0;
        g_elem_action.active_elem = ELEM_NONE;
        for (row = 0; row < IMG_H; ++row)
        {
            my_image.Left_Lost_Flag[row] = my_image.Right_Lost_Flag[row] = my_image.Mid_Lost_Flag[row] = 1;
            for (col = 0; col < IMG_W; ++col)
                my_image.image_two_value[row][col] = gray[row * IMG_W + col] >= 128 ? 255 : 0;
        }
        memset(&result, 0, sizeof(result));
        result.frame_seq = s_frame_seq;
        result.source_mode = (uint8)feedback.beacon.mode;
        result.input_seq = feedback.input_seq;
        result.heartbeat = s_heartbeat;
        result.camera_ok = 1;
        result.track_mode = TRACK_MODE_HOLD;
        result.threshold = 128;
        vision_display_publish((vision_display_mode_t)s_display_mode);
        vision_result_publish(&result);
        return;
    }

    // 前瞻行是运行参数，每帧从 CPU0 的快照里取，改完当帧生效。
    g_err_front_row = (int)feedback.err_front_row;

    if (feedback.param_revision != s_ipm_param_revision)
    {
        s_ipm_param_revision = feedback.param_revision;
        if (ipm_load(feedback.ipm_h))
            memcpy(s_ipm_applied, feedback.ipm_h, sizeof(s_ipm_applied));
        else
        {
            memset(s_ipm_applied, 0, sizeof(s_ipm_applied));
            ipm_clear();
        }
    }

    if (feedback.beacon.mode != 0) {
        int row;
        memset(&result,0,sizeof(result));
        beacon_process(image_gray_data(),my_image.image_two_value,&feedback.beacon,&result.beacon);
        beacon_follow_update(&feedback.beacon,feedback.run_active,&result.beacon);
        my_image.Track_Valid=(uint8)(result.beacon.valid && result.beacon.steer_valid);
        my_image.Threshold=feedback.beacon.threshold;
        my_image.Search_Stop_Line=0;
        g_elem_action.active_elem=ELEM_NONE;
        for(row=0;row<IMG_H;row++) {
            my_image.Left_Lost_Flag[row]=my_image.Right_Lost_Flag[row]=my_image.Mid_Lost_Flag[row]=1;
            if(result.beacon.steer_valid) {
                my_image.Left_Line[row]=(int)(result.beacon.follow_mid_x+0.5f);
                my_image.Left_Lost_Flag[row]=0; /* green = dynamic aim column */
            }
            if(result.beacon.valid && abs(row-(int)result.beacon.cy)<=4) {
                my_image.Mid_Line[row]=(int)result.beacon.cx;
                my_image.Mid_Lost_Flag[row]=0;
            }
        }
        result.source_mode=(uint8)feedback.beacon.mode;
        result.frame_seq=s_frame_seq;result.input_seq=feedback.input_seq;result.heartbeat=s_heartbeat;
        result.track_valid=(uint8)my_image.Track_Valid;result.camera_ok=1;result.run_active=feedback.run_active;
        result.track_mode=result.beacon.steer_valid?TRACK_MODE_MIDDLE:TRACK_MODE_HOLD;
        result.active_elem=ELEM_NONE;result.threshold=(uint16)feedback.beacon.threshold;
        result.ipm_ok=ipm_ready();ipm_store(result.ipm_h);
        if(result.beacon.steer_valid) {
            /* Upstream uses cx-dynamic_mid; current Run uses the opposite
             * image error followed by STEER_DIR. Apply that sign exactly once. */
            result.lateral_error=-result.beacon.error_px/94.0f;
            result.direction_camera=-result.beacon.error_px*DIRECTION_CAMERA_WEIGHT_SUM;
            result.quality=result.track_valid?1.0f:0.0f;
            result.speed_limit_mps=feedback.beacon.speed_mps;
        }
        vision_display_publish((vision_display_mode_t)s_display_mode);
        vision_result_publish(&result);
        return;
    }
    {beacon_result_t unused;beacon_process(0,0,&feedback.beacon,&unused);}

    // 元素检测只读本帧原始边线；补线完成后统一生成一次中线和转向量。
    if (my_image.Track_Valid)
    {
        base_error = g_mid_error;
        base_track_valid = 1;
    }
    image_process_edges((uint8)!feedback.run_active);
    vision_ipm_collect();

    motion.drive_count_total = feedback.drive_count_total;
    motion.element_yaw = feedback.element_yaw;
    motion.pitch = feedback.pitch;
    motion.pitch_rate = feedback.pitch_rate;
    motion.drive_speed_mps = feedback.drive_speed_mps;
    motion.drive_output = feedback.drive_output;
    motion.track_error = base_error;
    motion.track_valid = base_track_valid;
    motion.speed_cross_mps = feedback.speed_cross_mps;
    motion.speed_ring_mps = feedback.speed_ring_mps;
    motion.speed_ramp_mps = feedback.speed_ramp_mps;
    motion.uptime_ms = feedback.uptime_ms;
    motion.ring_angle = (int)feedback.ring_angle;
    motion.ring_s2_cnt_l = (int)feedback.ring_s2_cnt_l;
    motion.ring_s2_cnt_r = (int)feedback.ring_s2_cnt_r;
    motion.en_zebra = feedback.elem_en_zebra;
    motion.en_cross = feedback.elem_en_cross;
    motion.en_ring = feedback.elem_en_ring;
    motion.en_ramp = feedback.elem_en_ramp;
    motion.run_active = feedback.run_active;

    element_set_motion(&motion);
    element_process();
    image_process_finish();

    vision_display_publish((vision_display_mode_t)s_display_mode);

    memset(&result, 0, sizeof(result));
    result.frame_seq = s_frame_seq;
    result.input_seq = feedback.input_seq;
    result.heartbeat = s_heartbeat;
    result.lateral_error = g_track_lateral;
    result.heading_error = g_track_heading;
    result.curvature = g_track_curvature;
    result.direction_camera = g_direction_camera;
    result.quality = g_track_quality;
    result.speed_limit_mps = g_elem_action.speed_limit_mps;
    result.ipm_calib_seq = s_ipm_calib_seq;
    result.ipm_ok = ipm_ready();
    ipm_store(result.ipm_h);
    result.threshold = (uint16)my_image.Threshold;
    result.search_stop_line = (uint16)my_image.Search_Stop_Line;
    result.left_lost = (uint16)my_image.Left_Lost_Counter;
    result.right_lost = (uint16)my_image.Right_Lost_Counter;
    result.both_lost = (uint16)my_image.Both_Lost_Counter;
    result.active_elem = g_elem_action.active_elem;
    result.track_mode = g_elem_action.track_mode;
    result.island_state = (uint8)g_island.island_state;
    result.camera_ok = 1;
    result.track_valid = g_direction_valid;
    result.stop_request = g_elem_action.stop_request;
    result.run_active = feedback.run_active;
    vision_result_publish(&result);
}

#pragma section code restore

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     原子读取 CPU1 发布的最新视觉结果
// 参数说明     result          视觉结果输出地址
// 返回参数     uint8           1=读取成功 0=结果正在更新
// 使用示例     if (vision_result_read(&result)) { ... }
//-------------------------------------------------------------------------------------------------------------------
uint8 vision_result_read(vision_result_t *result)
{
    uint32 seq_begin;
    uint32 seq_end;

    if (result == 0) return 0;
    seq_begin = s_result_box.seq;
    if (seq_begin & 1u) return 0;
    __dsync();
    *result = s_result_box.payload;
    __dsync();
    seq_end = s_result_box.seq;
    return (uint8)(seq_begin == seq_end && !(seq_end & 1u));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     向 CPU1 发布普通图像处理所需参数
// 参数说明     feedback        CPU0 反馈快照
// 返回参数     void
// 使用示例     vision_feedback_publish(&feedback);
//-------------------------------------------------------------------------------------------------------------------
void vision_feedback_publish(const vision_feedback_t *feedback)
{
    uint32 seq;

    if (feedback == 0) return;
    seq = s_feedback_box.seq;
    s_feedback_box.seq = seq + 1u;
    __dsync();
    s_feedback_box.payload = *feedback;
    __dsync();
    s_feedback_box.seq = seq + 2u;
    __dsync();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     请求 CPU1 调整曝光或重新初始化摄像头
// 参数说明     command/value   命令类型与命令参数
// 返回参数     uint8           1=请求已发布 0=参数无效
// 使用示例     vision_command_request(VISION_CMD_SET_EXPOSURE, 512);
//-------------------------------------------------------------------------------------------------------------------
uint8 vision_command_request(vision_command_t command, uint16 value)
{
    uint32 head;
    if (command <= VISION_CMD_NONE || command >= VISION_CMD_COUNT) return 0;
    if (command == VISION_CMD_SET_EXPOSURE && (value < 4u || value > 1600u)) return 0;
    if (command == VISION_CMD_SET_PROFILE && value > 1u) return 0;
    /* Single producer: CPU0 foreground only. CPU1 is the only consumer. */
    head = s_command_box.head;
    if ((uint32)(head - s_command_box.tail) >= VISION_COMMAND_CAPACITY) return 0;
    s_command_box.entries[head % VISION_COMMAND_CAPACITY].command = command;
    s_command_box.entries[head % VISION_COMMAND_CAPACITY].value = value;
    __dsync();
    s_command_box.head = head + 1u;
    __dsync();
    return 1;
}

uint8 vision_camera_status_read(vision_camera_status_t *status)
{
    uint32 begin, end;
    if (!status) return 0;
    begin = s_camera_box.seq;
    if (!begin || (begin & 1u)) return 0;
    __dsync();
    *status = s_camera_box.payload;
    __dsync();
    end = s_camera_box.seq;
    return (uint8)(begin == end && !(end & 1u));
}

/* CPU0 foreground gate. Pending reconfiguration cannot reuse a fresh old target. */
uint8 vision_camera_run_allowed(void)
{
    vision_camera_status_t status;
    uint32 head, tail;
    if (s_camera_reconfiguring || s_core_state != VISION_CORE_READY ||
        !vision_camera_status_read(&status) || status.diagnostic || !status.gray_valid) return 0;
    tail = s_command_box.tail;
    head = s_command_box.head;
    __dsync();
    if ((uint32)(head - tail) > VISION_COMMAND_CAPACITY) return 0;
    while (tail != head)
    {
        vision_command_t command = s_command_box.entries[tail % VISION_COMMAND_CAPACITY].command;
        if (command == VISION_CMD_RESTART_CAMERA || command == VISION_CMD_SET_PROFILE) return 0;
        ++tail;
    }
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     请求 CPU1 在下一帧生成指定模式的屏幕快照
// 参数说明     mode            灰度、二值或二值加边线
// 返回参数     void
// 使用示例     vision_display_request(VISION_DISPLAY_BIN_LINE);
//-------------------------------------------------------------------------------------------------------------------
void vision_display_request(vision_display_mode_t mode)
{
    if (mode > VISION_DISPLAY_GRAY) mode = VISION_DISPLAY_BIN_LINE;
    s_display_mode = (uint8)mode;
    __dsync();
    s_display_request = 1;
    __dsync();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取一份由 CPU1 锁定的屏幕图像快照
// 参数说明     frame           返回快照只读指针
// 返回参数     uint8           1=快照可用 0=尚未准备完成
// 使用示例     if (vision_display_read(&frame)) { ... }
//-------------------------------------------------------------------------------------------------------------------
uint8 vision_display_read(const vision_display_frame_t **frame)
{
    if (frame == 0 || !s_display_ready) return 0;
    __dsync();
    *frame = &s_display_frame;
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     释放屏幕快照并允许 CPU1 生成下一帧
// 参数说明     void
// 返回参数     void
// 使用示例     vision_display_release();
//-------------------------------------------------------------------------------------------------------------------
void vision_display_release(void)
{
    __dsync();
    s_display_ready = 0;
    __dsync();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取 CPU1 摄像头工作状态
// 参数说明     void
// 返回参数     vision_core_state_t 当前工作状态
// 使用示例     if (vision_core_state() == VISION_CORE_READY) { ... }
//-------------------------------------------------------------------------------------------------------------------
vision_core_state_t vision_core_state(void)
{
    __dsync();
    return s_core_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取 CPU1 主循环心跳计数
// 参数说明     void
// 返回参数     uint32          心跳计数
// 使用示例     heartbeat = vision_core_heartbeat();
//-------------------------------------------------------------------------------------------------------------------
uint32 vision_core_heartbeat(void)
{
    __dsync();
    return s_heartbeat;
}
