#include "lf_harness_core.h"

#include <math.h>
#include <stddef.h>

#include "lf_app.h"
#include "lf_platform.h"

_Static_assert(LF_SENSOR_COUNT == 8U, "lf_autotest_harness assumes 8 line sensors.");

typedef struct {
    double x;
    double y;
} LFH_Point2;

typedef struct {
    double x;
    double y;
    double radius_m;
    bool active;
} LFH_Obstacle;

typedef struct {
    double progress_m;
    double lateral_error_m;
    uint32_t segment_index;
    bool valid;
} LFH_TrackProjection;

#define LFH_MAX_OBSTACLES 12U
#define LFH_RADAR_FRAME_SIZE 8U
#define LFH_RADAR_MINIMAL_FRAME_SIZE 5U
#define LFH_PATIO_REAL_CORRIDOR_HALF_WIDTH_M 0.35

static LFH_Pose g_pose;
static int16_t g_left_cmd = 0;
static int16_t g_right_cmd = 0;
static double g_sim_time_sec = 0.0;
static bool g_led_on = false;

static LFH_TrackType g_track = LFH_TRACK_CIRCLE;
static double g_line_width_m = 0.03;
static double g_max_wheel_speed_mps = 1.0;
static double g_track_width_m = 0.2;
static double g_left_wheel_speed_mps = 0.0;
static double g_right_wheel_speed_mps = 0.0;
static double g_motor_tau_sec = 0.10;
static double g_battery_drop_max = 0.03;
static double g_sensor_mount_x_offset_m = 0.0;
static double g_sensor_mount_y_offset_m = 0.0;
static double g_sensor_mount_yaw_rad = 0.0;
static double g_radar_frame_period_sec = 0.050;
static double g_radar_miss_prob = 0.01;
static double g_radar_false_prob = 0.002;
static double g_radar_jitter_mm = 30.0;
static double g_radar_next_frame_time_sec = 0.0;

static LFH_Scenario g_runtime_scenario;
static const LFH_Scenario *g_active_scenario = NULL;
static uint32_t g_rng_state = 1U;
static double g_sensor_gain[LF_SENSOR_COUNT];
static double g_sensor_bias[LF_SENSOR_COUNT];
static double g_sensor_gain_span = 0.03;
static double g_sensor_bias_span = 0.006;
static double g_dropout_hold_scale = 0.20;
static LFH_Obstacle g_obstacles[LFH_MAX_OBSTACLES];
static uint32_t g_obstacle_count = 0U;
static uint32_t g_radar_detection_count = 0U;
static double g_progress_m = 0.0;
static double g_max_progress_m = 0.0;
static double g_course_length_m = 0.0;
static double g_lateral_error_m = 0.0;
static uint8_t g_radar_frame[LFH_RADAR_FRAME_SIZE];
static uint8_t g_radar_frame_len = 0U;
static uint8_t g_radar_frame_index = 0U;
static double g_last_sensor_norm[LF_SENSOR_COUNT];
static uint16_t g_last_sensor_raw[LF_SENSOR_COUNT];
static bool g_sensor_sample_cached = false;

static const double k_sensor_xy[LF_SENSOR_COUNT][2] = {
    {0.16, 0.0875},
    {0.16, 0.0625},
    {0.16, 0.0375},
    {0.16, 0.0125},
    {0.16, -0.0125},
    {0.16, -0.0375},
    {0.16, -0.0625},
    {0.16, -0.0875},
};

static const LFH_Point2 k_patio_path[] = {
    {0.00, 0.00},
    {0.00, 4.00},
    {0.10, 4.18},
    {0.25, 4.27},
    {0.40, 4.18},
    {0.50, 4.00},
    {0.50, 2.25},
    {0.66, 1.92},
    {0.36, 1.56},
    {0.68, 1.20},
    {0.36, 0.86},
    {0.52, 0.52},
    {0.52, 0.18},
    {1.24, 0.18},
    {1.24, 2.45},
    {2.58, 2.45},
    {2.58, 1.50},
    {2.78, 1.24},
    {3.02, 1.12},
    {3.24, 1.30},
    {3.36, 1.58},
    {3.36, 2.02},
    {3.95, 2.02},
};

static const LFH_Point2 k_patio_real_path[] = {
    {0.50, 0.50},
    {0.50, 4.50},
    {0.62, 4.78},
    {0.88, 4.95},
    {1.12, 4.78},
    {1.25, 4.50},
    {1.25, 2.60},
    {1.55, 2.20},
    {1.05, 1.80},
    {1.55, 1.38},
    {1.05, 0.96},
    {1.25, 0.50},
    {1.95, 0.50},
    {1.95, 2.55},
    {2.45, 2.55},
    {2.45, 3.55},
    {3.95, 3.55},
    {3.95, 2.45},
    {4.28, 2.20},
    {4.55, 2.45},
    {4.55, 3.20},
    {5.35, 3.20},
    {5.35, 2.15},
    {5.52, 1.86},
    {5.80, 1.72},
    {6.08, 1.86},
    {6.25, 2.15},
    {6.55, 2.15},
    {7.05, 2.15},
};

static double lfh_clamp_d(double v, double lo, double hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

static double lfh_profiled_motor_scale(double scale, LFH_DisturbanceProfile profile)
{
    const double delta = scale - 1.0;
    const double factor = (profile == LFH_PROFILE_STRESS) ? 1.45 : 1.0;

    return lfh_clamp_d(1.0 + delta * factor, 0.50, 1.50);
}

static void lfh_apply_disturbance_profile(const LFH_Scenario *scenario, const LFH_TestConfig *cfg)
{
    g_runtime_scenario = *scenario;
    g_sensor_gain_span = 0.03;
    g_sensor_bias_span = 0.006;
    g_dropout_hold_scale = 0.20;
    g_motor_tau_sec = 0.10;
    g_battery_drop_max = 0.03;
    g_radar_frame_period_sec = 0.050;
    g_radar_miss_prob = 0.01;
    g_radar_false_prob = 0.002;
    g_radar_jitter_mm = 30.0;

    if (cfg->disturbance_profile == LFH_PROFILE_STRESS) {
        g_runtime_scenario.noise_std = lfh_clamp_d(scenario->noise_std * 1.55, 0.0, 0.20);
        g_runtime_scenario.dropout_prob = lfh_clamp_d(scenario->dropout_prob * 1.80 + 0.002, 0.0, 0.25);
        g_runtime_scenario.contrast = lfh_clamp_d(scenario->contrast * 0.92, 0.50, 1.20);
        g_runtime_scenario.left_motor_scale =
            lfh_profiled_motor_scale(scenario->left_motor_scale, cfg->disturbance_profile);
        g_runtime_scenario.right_motor_scale =
            lfh_profiled_motor_scale(scenario->right_motor_scale, cfg->disturbance_profile);
        g_sensor_gain_span = 0.06;
        g_sensor_bias_span = 0.012;
        g_dropout_hold_scale = 0.10;
        g_motor_tau_sec = 0.18;
        g_battery_drop_max = 0.09;
        g_radar_frame_period_sec = 0.120;
        g_radar_miss_prob = 0.07;
        g_radar_false_prob = 0.008;
        g_radar_jitter_mm = 80.0;
    }

    g_active_scenario = &g_runtime_scenario;
}

static int32_t lfh_clamp_i32(int32_t v, int32_t lo, int32_t hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

static double lfh_normalize_angle(double angle)
{
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

static uint32_t lfh_xorshift32(void)
{
    uint32_t x = g_rng_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    g_rng_state = (x == 0U) ? 1U : x;
    return g_rng_state;
}

static uint32_t lfh_xorshift32_state(uint32_t *state)
{
    uint32_t x = *state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *state = (x == 0U) ? 1U : x;
    return *state;
}

static double lfh_uniform01_state(uint32_t *state)
{
    return ((double)(lfh_xorshift32_state(state) >> 8)) * (1.0 / 16777216.0);
}

static double lfh_rng_uniform01(void)
{
    return ((double)(lfh_xorshift32() >> 8)) * (1.0 / 16777216.0);
}

static double lfh_rng_symm(void)
{
    return 2.0 * lfh_rng_uniform01() - 1.0;
}

static double lfh_dist_point_segment(double px,
                                     double py,
                                     double x1,
                                     double y1,
                                     double x2,
                                     double y2)
{
    const double vx = x2 - x1;
    const double vy = y2 - y1;
    const double wx = px - x1;
    const double wy = py - y1;
    const double vv = vx * vx + vy * vy;

    if (vv <= 1e-12) {
        return hypot(px - x1, py - y1);
    }

    {
        const double t = lfh_clamp_d((wx * vx + wy * vy) / vv, 0.0, 1.0);
        const double cx = x1 + t * vx;
        const double cy = y1 + t * vy;
        return hypot(px - cx, py - cy);
    }
}

static double lfh_distance_to_path_centerline(const LFH_Point2 *path, size_t point_count, double x, double y)
{
    size_t i;
    double best = 1e9;

    for (i = 0U; i + 1U < point_count; ++i) {
        const LFH_Point2 *a = &path[i];
        const LFH_Point2 *b = &path[i + 1U];
        const double d = lfh_dist_point_segment(x, y, a->x, a->y, b->x, b->y);
        if (d < best) {
            best = d;
        }
    }

    return best;
}

static double lfh_path_length(const LFH_Point2 *path, size_t point_count)
{
    size_t i;
    double length = 0.0;

    for (i = 0U; i + 1U < point_count; ++i) {
        length += hypot(path[i + 1U].x - path[i].x, path[i + 1U].y - path[i].y);
    }

    return length;
}

static LFH_TrackProjection lfh_project_to_path(const LFH_Point2 *path, size_t point_count, double x, double y)
{
    size_t i;
    double cumulative = 0.0;
    double best_dist = 1e9;
    LFH_TrackProjection best;

    best.progress_m = 0.0;
    best.lateral_error_m = 0.0;
    best.segment_index = 0U;
    best.valid = false;

    for (i = 0U; i + 1U < point_count; ++i) {
        const LFH_Point2 *a = &path[i];
        const LFH_Point2 *b = &path[i + 1U];
        const double vx = b->x - a->x;
        const double vy = b->y - a->y;
        const double wx = x - a->x;
        const double wy = y - a->y;
        const double vv = vx * vx + vy * vy;
        const double seg_len = sqrt(vv);

        if (vv > 1e-12) {
            const double t = lfh_clamp_d((wx * vx + wy * vy) / vv, 0.0, 1.0);
            const double cx = a->x + t * vx;
            const double cy = a->y + t * vy;
            const double dx = x - cx;
            const double dy = y - cy;
            const double dist = hypot(dx, dy);
            if (dist < best_dist) {
                const double cross = vx * dy - vy * dx;
                best_dist = dist;
                best.progress_m = cumulative + t * seg_len;
                best.lateral_error_m = (cross >= 0.0) ? dist : -dist;
                best.segment_index = (uint32_t)i;
                best.valid = true;
            }
        }

        cumulative += seg_len;
    }

    return best;
}

static LFH_TrackProjection lfh_project_to_patio_real(double x, double y)
{
    return lfh_project_to_path(k_patio_real_path, sizeof(k_patio_real_path) / sizeof(k_patio_real_path[0]), x, y);
}

static double lfh_distance_to_patio_centerline(double x, double y)
{
    return lfh_distance_to_path_centerline(k_patio_path,
                                           sizeof(k_patio_path) / sizeof(k_patio_path[0]),
                                           x,
                                           y);
}

static double lfh_distance_to_patio_real_centerline(double x, double y)
{
    return lfh_distance_to_path_centerline(k_patio_real_path,
                                           sizeof(k_patio_real_path) / sizeof(k_patio_real_path[0]),
                                           x,
                                           y);
}

static bool lfh_point_in_rect(double x, double y, double x_min, double y_min, double x_max, double y_max)
{
    return x >= x_min && x <= x_max && y >= y_min && y <= y_max;
}

static bool lfh_is_full_course_scenario(const LFH_Scenario *scenario)
{
    return scenario != NULL &&
           scenario->track == LFH_TRACK_PATIO_REAL &&
           scenario->checkpoint_enable &&
           scenario->checkpoint_expected_count >= 2U &&
           scenario->finish_enable &&
           scenario->boundary_enable;
}

static void lfh_update_course_state(void)
{
    if (g_track != LFH_TRACK_PATIO_REAL) {
        g_progress_m = 0.0;
        g_max_progress_m = 0.0;
        g_course_length_m = 0.0;
        g_lateral_error_m = 0.0;
        return;
    }

    {
        const LFH_TrackProjection projection = lfh_project_to_patio_real(g_pose.x, g_pose.y);
        if (g_course_length_m <= 1e-9) {
            g_course_length_m = lfh_path_length(k_patio_real_path, sizeof(k_patio_real_path) / sizeof(k_patio_real_path[0]));
        }
        if (projection.valid) {
            g_progress_m = projection.progress_m;
            g_lateral_error_m = projection.lateral_error_m;
            if (projection.progress_m > g_max_progress_m) {
                g_max_progress_m = projection.progress_m;
            }
        }
    }
}

static void lfh_generate_obstacles(const LFH_Scenario *scenario)
{
    uint32_t i;
    uint32_t state;
    uint32_t requested;

    g_obstacle_count = 0U;
    for (i = 0U; i < LFH_MAX_OBSTACLES; ++i) {
        g_obstacles[i].active = false;
    }

    if (scenario->obstacle_mode == LFH_OBSTACLE_NONE || scenario->obstacle_count == 0U) {
        return;
    }

    requested = scenario->obstacle_count;
    if (requested > LFH_MAX_OBSTACLES) {
        requested = LFH_MAX_OBSTACLES;
    }

    if (scenario->obstacle_mode == LFH_OBSTACLE_FIXED) {
        static const LFH_Point2 fixed_points[] = {
            {2.72, 3.88},
            {2.72, 3.58},
            {2.72, 3.28},
            {2.72, 2.98},
        };
        for (i = 0U; i < requested; ++i) {
            const LFH_Point2 *p = &fixed_points[i % (sizeof(fixed_points) / sizeof(fixed_points[0]))];
            g_obstacles[i].x = p->x;
            g_obstacles[i].y = p->y;
            g_obstacles[i].radius_m = 0.10;
            g_obstacles[i].active = true;
        }
        g_obstacle_count = requested;
        return;
    }

    state = scenario->obstacle_seed ^ 0x9e3779b9U;
    if (state == 0U) {
        state = 1U;
    }

    for (i = 0U; i < requested; ++i) {
        g_obstacles[i].x = 2.50 + lfh_uniform01_state(&state) * 0.95;
        g_obstacles[i].y = 2.70 + lfh_uniform01_state(&state) * 1.35;
        g_obstacles[i].radius_m = 0.09 + lfh_uniform01_state(&state) * 0.04;
        g_obstacles[i].active = true;
    }
    g_obstacle_count = requested;
}

static double lfh_distance_to_track_centerline(double x, double y)
{
    if (g_track == LFH_TRACK_CIRCLE) {
        const double r = 1.15;
        const double dx = x - r;
        const double dy = y;
        return fabs(hypot(dx, dy) - r);
    }

    if (g_track == LFH_TRACK_FIGURE8) {
        const double r = 0.7;
        const double d1 = fabs(hypot(x + r, y) - r);
        const double d2 = fabs(hypot(x - r, y) - r);
        return (d1 < d2) ? d1 : d2;
    }

    if (g_track == LFH_TRACK_PATIO_REAL) {
        return lfh_distance_to_patio_real_centerline(x, y);
    }

    return lfh_distance_to_patio_centerline(x, y);
}

static double lfh_line_intensity(double distance_m)
{
    const double half = g_line_width_m * 0.5;

    if (half <= 1e-9) {
        return 0.0;
    }

    if (distance_m <= half) {
        return 1.0;
    }

    {
        const double sigma = half * 0.85;
        const double err = distance_m - half;
        return exp(-(err * err) / (2.0 * sigma * sigma));
    }
}

static double lfh_command_to_speed_mps(int16_t cmd)
{
    const double normalized = lfh_clamp_d((double)cmd, -1000.0, 1000.0) / 1000.0;
    const double mag = fabs(normalized);
    const double shaped = 0.68 * mag + 0.32 * mag * mag;
    const double signed_shaped = (normalized < 0.0) ? -shaped : shaped;

    return signed_shaped * g_max_wheel_speed_mps;
}

void LFH_Core_Reset(const LFH_Scenario *scenario, const LFH_TestConfig *cfg)
{
    uint32_t i;

    g_pose = scenario->start_pose;
    g_left_cmd = 0;
    g_right_cmd = 0;
    g_left_wheel_speed_mps = 0.0;
    g_right_wheel_speed_mps = 0.0;
    g_sim_time_sec = 0.0;
    g_led_on = false;
    g_track = scenario->track;
    g_line_width_m = cfg->line_width_m;
    g_max_wheel_speed_mps = cfg->max_wheel_speed_mps;
    g_track_width_m = cfg->track_width_m;
    g_radar_detection_count = 0U;
    g_progress_m = 0.0;
    g_max_progress_m = 0.0;
    g_course_length_m = (scenario->track == LFH_TRACK_PATIO_REAL) ?
        lfh_path_length(k_patio_real_path, sizeof(k_patio_real_path) / sizeof(k_patio_real_path[0])) :
        0.0;
    g_lateral_error_m = 0.0;
    g_radar_frame_len = 0U;
    g_radar_frame_index = 0U;
    g_radar_next_frame_time_sec = 0.0;
    g_sensor_sample_cached = false;
    lfh_apply_disturbance_profile(scenario, cfg);

    g_rng_state = (cfg->base_seed ^ g_runtime_scenario.seed);
    if (g_rng_state == 0U) {
        g_rng_state = 1U;
    }

    g_line_width_m *= 1.0 + ((cfg->disturbance_profile == LFH_PROFILE_STRESS) ? 0.20 : 0.10) * lfh_rng_symm();
    g_sensor_mount_x_offset_m = ((cfg->disturbance_profile == LFH_PROFILE_STRESS) ? 0.003 : 0.0015) * lfh_rng_symm();
    g_sensor_mount_y_offset_m = ((cfg->disturbance_profile == LFH_PROFILE_STRESS) ? 0.003 : 0.0015) * lfh_rng_symm();
    g_sensor_mount_yaw_rad = ((cfg->disturbance_profile == LFH_PROFILE_STRESS) ? 0.035 : 0.017) * lfh_rng_symm();
    lfh_generate_obstacles(&g_runtime_scenario);
    lfh_update_course_state();

    for (i = 0U; i < LF_SENSOR_COUNT; ++i) {
        g_sensor_gain[i] = 1.0 + g_sensor_gain_span * lfh_rng_symm();
        g_sensor_bias[i] = g_sensor_bias_span * lfh_rng_symm();
        g_last_sensor_norm[i] = 0.0;
        g_last_sensor_raw[i] = 0U;
    }
}

static void lfh_read_calibration_sensor_raw(uint16_t out_raw[LF_SENSOR_COUNT])
{
    uint32_t i;
    const double period_sec = 1.20;
    const double phase = fmod(g_sim_time_sec, period_sec) / period_sec;
    const double tri = (phase < 0.5) ? (phase * 2.0) : (2.0 - phase * 2.0);
    const double line_center_y = -0.115 + tri * 0.230;

    for (i = 0U; i < LF_SENSOR_COUNT; ++i) {
        const double dist = fabs(k_sensor_xy[i][1] - line_center_y);
        double intensity = lfh_line_intensity(dist);
        int32_t raw_i;

        intensity *= g_sensor_gain[i];
        intensity += g_sensor_bias[i];
        intensity = lfh_clamp_d(intensity, 0.0, 1.0);
        raw_i = lfh_clamp_i32((int32_t)llround(150.0 + intensity * 3650.0), 0, 4095);
        out_raw[i] = (uint16_t)raw_i;
    }
}

void LFH_Core_ReadSensorNormAndRaw(double out_norm[LF_SENSOR_COUNT], uint16_t out_raw[LF_SENSOR_COUNT])
{
    uint32_t i;
    const double mount_c = cos(g_sensor_mount_yaw_rad);
    const double mount_s = sin(g_sensor_mount_yaw_rad);
    const double c = cos(g_pose.theta);
    const double s = sin(g_pose.theta);

    for (i = 0U; i < LF_SENSOR_COUNT; ++i) {
        const double local_x = g_sensor_mount_x_offset_m + k_sensor_xy[i][0] * mount_c - k_sensor_xy[i][1] * mount_s;
        const double local_y = g_sensor_mount_y_offset_m + k_sensor_xy[i][0] * mount_s + k_sensor_xy[i][1] * mount_c;
        const double wx = g_pose.x + local_x * c - local_y * s;
        const double wy = g_pose.y + local_x * s + local_y * c;
        const double dist = lfh_distance_to_track_centerline(wx, wy);
        double intensity = lfh_line_intensity(dist);
        double raw_f;
        int32_t raw_i;

        intensity *= (g_active_scenario != NULL) ? g_active_scenario->contrast : 1.0;
        intensity *= g_sensor_gain[i];
        intensity += g_sensor_bias[i];

        if (g_active_scenario != NULL && g_active_scenario->noise_std > 0.0) {
            intensity += g_active_scenario->noise_std * lfh_rng_symm();
        }

        if (g_active_scenario != NULL && g_active_scenario->dropout_prob > 0.0) {
            if (lfh_rng_uniform01() < g_active_scenario->dropout_prob) {
                intensity *= g_dropout_hold_scale;
            }
        }

        intensity = lfh_clamp_d(intensity, 0.0, 1.0);
        raw_f = 150.0 + intensity * 3650.0;
        raw_i = lfh_clamp_i32((int32_t)llround(raw_f), 0, 4095);

        out_norm[i] = intensity;
        out_raw[i] = (uint16_t)raw_i;
        g_last_sensor_norm[i] = intensity;
        g_last_sensor_raw[i] = (uint16_t)raw_i;
    }
    g_sensor_sample_cached = true;
}

void LFH_Core_ClearSensorSampleCache(void)
{
    g_sensor_sample_cached = false;
}

void LFH_Core_GetLastSensorNormAndRaw(double out_norm[LF_SENSOR_COUNT], uint16_t out_raw[LF_SENSOR_COUNT])
{
    uint32_t i;

    for (i = 0U; i < LF_SENSOR_COUNT; ++i) {
        if (out_norm != NULL) {
            out_norm[i] = g_last_sensor_norm[i];
        }
        if (out_raw != NULL) {
            out_raw[i] = g_last_sensor_raw[i];
        }
    }
}

LFH_LineState LFH_Core_EstimateLineState(const double sensor_norm[LF_SENSOR_COUNT], double threshold)
{
    uint32_t i;
    double signal_sum = 0.0;
    double weighted_y = 0.0;
    double max_value = 0.0;
    LFH_LineState st;

    for (i = 0U; i < LF_SENSOR_COUNT; ++i) {
        const double v = lfh_clamp_d(sensor_norm[i], 0.0, 1.0);
        signal_sum += v;
        weighted_y += k_sensor_xy[i][1] * v;
        if (v > max_value) {
            max_value = v;
        }
    }

    st.line_detected = (max_value >= threshold);
    st.line_error_m = (signal_sum > 1e-12) ? (weighted_y / signal_sum) : 0.0;
    st.confidence = max_value;
    return st;
}

void LFH_Core_AdvanceTime(double dt_sec)
{
    g_sim_time_sec += dt_sec;
}

static bool lfh_find_front_obstacle(double *out_distance_m)
{
    uint32_t i;
    bool found = false;
    double best = 10.0;
    const double c = cos(g_pose.theta);
    const double s = sin(g_pose.theta);

    for (i = 0U; i < g_obstacle_count; ++i) {
        const LFH_Obstacle *obs = &g_obstacles[i];
        const double dx = obs->x - g_pose.x;
        const double dy = obs->y - g_pose.y;
        const double forward = dx * c + dy * s;
        const double lateral = -dx * s + dy * c;
        double distance;

        if (!obs->active || forward <= 0.0 || forward > 1.20) {
            continue;
        }
        if (fabs(lateral) > 0.28) {
            continue;
        }

        distance = hypot(dx, dy) - obs->radius_m;
        if (distance < 0.0) {
            distance = 0.0;
        }
        if (distance < best) {
            best = distance;
            found = true;
        }
    }

    if (found && out_distance_m != NULL) {
        *out_distance_m = best;
    }
    return found;
}

static void lfh_prepare_radar_frame(void)
{
    double distance_m;
    uint16_t distance_mm;
    bool has_target;

    g_radar_frame_len = 0U;
    g_radar_frame_index = 0U;

    if (g_active_scenario == NULL || !g_active_scenario->radar_enable) {
        return;
    }
    if (g_sim_time_sec < g_radar_next_frame_time_sec) {
        return;
    }
    g_radar_next_frame_time_sec = g_sim_time_sec + g_radar_frame_period_sec;

    has_target = lfh_find_front_obstacle(&distance_m);
    if (has_target && lfh_rng_uniform01() < g_radar_miss_prob) {
        has_target = false;
    }
    if (!has_target) {
        if (lfh_rng_uniform01() >= g_radar_false_prob) {
            return;
        }
        distance_m = 0.70 + 0.35 * lfh_rng_uniform01();
        has_target = true;
    }
    if (!has_target) {
        return;
    }

    distance_m += (g_radar_jitter_mm * lfh_rng_symm()) / 1000.0;
    distance_mm = (uint16_t)lfh_clamp_d(distance_m * 1000.0, 0.0, 65535.0);
    {
        const uint16_t distance_cm = (uint16_t)lfh_clamp_d((double)distance_mm / 10.0, 0.0, 65535.0);
        g_radar_frame[0] = 0x6EU;
        g_radar_frame[1] = 2U;
        g_radar_frame[2] = (uint8_t)(distance_cm & 0xFFU);
        g_radar_frame[3] = (uint8_t)(distance_cm >> 8);
        g_radar_frame[4] = 0x62U;
    }
    g_radar_frame_len = LFH_RADAR_MINIMAL_FRAME_SIZE;
    g_radar_detection_count += 1U;
}

double LFH_Core_UpdatePoseFromCommand(double dt_sec)
{
    double target_l = lfh_command_to_speed_mps(g_left_cmd);
    double target_r = lfh_command_to_speed_mps(g_right_cmd);
    double v_l;
    double v_r;
    const double old_x = g_pose.x;
    const double old_y = g_pose.y;
    const double battery_scale = 1.0 - g_battery_drop_max * lfh_clamp_d(g_sim_time_sec / 120.0, 0.0, 1.0);
    const double response = 1.0 - exp(-dt_sec / fmax(g_motor_tau_sec, 1e-4));

    if (g_active_scenario != NULL) {
        target_l *= g_active_scenario->left_motor_scale;
        target_r *= g_active_scenario->right_motor_scale;
    }

    target_l *= battery_scale;
    target_r *= battery_scale;
    g_left_wheel_speed_mps += (target_l - g_left_wheel_speed_mps) * response;
    g_right_wheel_speed_mps += (target_r - g_right_wheel_speed_mps) * response;
    v_l = g_left_wheel_speed_mps;
    v_r = g_right_wheel_speed_mps;

    {
        const double slip_indicator = lfh_clamp_d(
            fabs(v_r - v_l) / (2.0 * g_max_wheel_speed_mps + 1e-9),
            0.0,
            1.0);
        const double traction = 1.0 - 0.20 * slip_indicator;
        v_l *= traction;
        v_r *= traction;
    }

    {
        const double vx = 0.5 * (v_l + v_r);
        const double omega = (v_r - v_l) / g_track_width_m;

        if (fabs(omega) > 1e-9) {
            const double new_theta = g_pose.theta + omega * dt_sec;
            g_pose.x += (vx / omega) * (sin(new_theta) - sin(g_pose.theta));
            g_pose.y += -(vx / omega) * (cos(new_theta) - cos(g_pose.theta));
            g_pose.theta = lfh_normalize_angle(new_theta);
        } else {
            g_pose.x += vx * cos(g_pose.theta) * dt_sec;
            g_pose.y += vx * sin(g_pose.theta) * dt_sec;
        }
    }

    lfh_update_course_state();
    return hypot(g_pose.x - old_x, g_pose.y - old_y);
}

int16_t LFH_Core_GetLeftCommand(void)
{
    return g_left_cmd;
}

int16_t LFH_Core_GetRightCommand(void)
{
    return g_right_cmd;
}

bool LFH_Core_IsInRawFinishZone(void)
{
    if (g_active_scenario == NULL || !g_active_scenario->finish_enable || g_track != LFH_TRACK_PATIO_REAL) {
        return false;
    }
    return lfh_point_in_rect(g_pose.x, g_pose.y, 6.85, 1.70, 8.00, 2.60);
}

bool LFH_Core_IsInFinishZone(void)
{
    return LFH_Core_IsInRawFinishZone();
}

bool LFH_Core_GetCheckpointId(uint32_t *checkpoint_id)
{
    if (g_active_scenario == NULL || !g_active_scenario->checkpoint_enable || g_track != LFH_TRACK_PATIO_REAL) {
        return false;
    }

    if (lfh_point_in_rect(g_pose.x, g_pose.y, 2.25, 3.25, 2.75, 3.75)) {
        if (checkpoint_id != NULL) {
            *checkpoint_id = 21U;
        }
        return true;
    }

    if (lfh_point_in_rect(g_pose.x, g_pose.y, 5.85, 1.70, 6.35, 2.25)) {
        if (checkpoint_id != NULL) {
            *checkpoint_id = 22U;
        }
        return true;
    }

    return false;
}

bool LFH_Core_IsBoundaryViolated(void)
{
    if (g_active_scenario == NULL || !g_active_scenario->boundary_enable || g_track != LFH_TRACK_PATIO_REAL) {
        return false;
    }
    if (lfh_is_full_course_scenario(g_active_scenario)) {
        return fabs(g_lateral_error_m) > LFH_PATIO_REAL_CORRIDOR_HALF_WIDTH_M;
    }
    return !lfh_point_in_rect(g_pose.x, g_pose.y, 0.0, 0.0, 8.0, 5.0);
}

double LFH_Core_GetProgressM(void)
{
    return g_progress_m;
}

double LFH_Core_GetMaxProgressM(void)
{
    return g_max_progress_m;
}

double LFH_Core_GetCourseLengthM(void)
{
    return g_course_length_m;
}

double LFH_Core_GetProgressPercent(void)
{
    if (g_course_length_m <= 1e-9) {
        return 0.0;
    }
    return lfh_clamp_d((g_max_progress_m / g_course_length_m) * 100.0, 0.0, 100.0);
}

double LFH_Core_GetLateralErrorM(void)
{
    return g_lateral_error_m;
}

bool LFH_Core_IsColliding(void)
{
    uint32_t i;

    for (i = 0U; i < g_obstacle_count; ++i) {
        const LFH_Obstacle *obs = &g_obstacles[i];
        if (obs->active && hypot(g_pose.x - obs->x, g_pose.y - obs->y) <= (obs->radius_m + 0.16)) {
            return true;
        }
    }
    return false;
}

uint32_t LFH_Core_GetObstacleCount(void)
{
    return g_obstacle_count;
}

uint32_t LFH_Core_GetRadarDetectionCount(void)
{
    return g_radar_detection_count;
}

void LF_Platform_BoardInit(void)
{
    g_left_cmd = 0;
    g_right_cmd = 0;
    g_led_on = false;
}

uint32_t LF_Platform_GetMillis(void)
{
    return (uint32_t)llround(g_sim_time_sec * 1000.0);
}

void LF_Platform_DelayMs(uint32_t ms)
{
    g_sim_time_sec += ((double)ms) / 1000.0;
}

void LF_Platform_ReadLineSensorRaw(uint16_t out_raw[LF_SENSOR_COUNT])
{
    double sensor_norm[LF_SENSOR_COUNT];
    const LF_AppContext *ctx;

    if (out_raw == NULL) {
        return;
    }

    ctx = LF_App_GetContext();
    if (ctx != NULL && ctx->state == LF_APP_STATE_CALIBRATING) {
        uint32_t i;
        lfh_read_calibration_sensor_raw(out_raw);
        for (i = 0U; i < LF_SENSOR_COUNT; ++i) {
            g_last_sensor_raw[i] = out_raw[i];
            g_last_sensor_norm[i] = lfh_clamp_d(((double)out_raw[i] - 150.0) / 3650.0, 0.0, 1.0);
        }
        g_sensor_sample_cached = true;
        return;
    }

    if (g_sensor_sample_cached) {
        uint32_t i;
        for (i = 0U; i < LF_SENSOR_COUNT; ++i) {
            out_raw[i] = g_last_sensor_raw[i];
        }
        return;
    }

    LFH_Core_ReadSensorNormAndRaw(sensor_norm, out_raw);
}

uint16_t LF_Platform_RadarRead(uint8_t *out_buf, uint16_t max_len)
{
    uint16_t count = 0U;

    if (out_buf == NULL || max_len == 0U) {
        return 0U;
    }

    if (g_radar_frame_index >= g_radar_frame_len) {
        lfh_prepare_radar_frame();
    }

    while (count < max_len && g_radar_frame_index < g_radar_frame_len) {
        out_buf[count++] = g_radar_frame[g_radar_frame_index++];
    }

    return count;
}

void LF_Platform_SetMotorCommand(int16_t left_cmd, int16_t right_cmd)
{
    g_left_cmd = left_cmd;
    g_right_cmd = right_cmd;
}

void LF_Platform_SetStatusLed(bool on)
{
    g_led_on = on;
}

bool LF_Platform_IsStartButtonPressed(void)
{
    return false;
}

void LF_Platform_DebugPrint(const char *msg)
{
    (void)msg;
}
