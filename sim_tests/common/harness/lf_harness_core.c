#include "lf_harness_core.h"

#include <math.h>
#include <stddef.h>

#include "lf_platform.h"

_Static_assert(LF_SENSOR_COUNT == 8U, "lf_autotest_harness assumes 8 line sensors.");

typedef struct {
    double x;
    double y;
} LFH_Point2;

static LFH_Pose g_pose;
static int16_t g_left_cmd = 0;
static int16_t g_right_cmd = 0;
static double g_sim_time_sec = 0.0;
static bool g_led_on = false;

static LFH_TrackType g_track = LFH_TRACK_CIRCLE;
static double g_line_width_m = 0.03;
static double g_max_wheel_speed_mps = 1.0;
static double g_track_width_m = 0.2;

static LFH_Scenario g_runtime_scenario;
static const LFH_Scenario *g_active_scenario = NULL;
static uint32_t g_rng_state = 1U;
static double g_sensor_gain[LF_SENSOR_COUNT];
static double g_sensor_bias[LF_SENSOR_COUNT];
static double g_sensor_gain_span = 0.03;
static double g_sensor_bias_span = 0.006;
static double g_dropout_hold_scale = 0.20;

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

static double lfh_distance_to_patio_centerline(double x, double y)
{
    size_t i;
    double best = 1e9;

    for (i = 0U; i + 1U < (sizeof(k_patio_path) / sizeof(k_patio_path[0])); ++i) {
        const LFH_Point2 *a = &k_patio_path[i];
        const LFH_Point2 *b = &k_patio_path[i + 1U];
        const double d = lfh_dist_point_segment(x, y, a->x, a->y, b->x, b->y);
        if (d < best) {
            best = d;
        }
    }

    return best;
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
    return normalized * g_max_wheel_speed_mps;
}

void LFH_Core_Reset(const LFH_Scenario *scenario, const LFH_TestConfig *cfg)
{
    uint32_t i;

    g_pose = scenario->start_pose;
    g_left_cmd = 0;
    g_right_cmd = 0;
    g_sim_time_sec = 0.0;
    g_led_on = false;
    g_track = scenario->track;
    g_line_width_m = cfg->line_width_m;
    g_max_wheel_speed_mps = cfg->max_wheel_speed_mps;
    g_track_width_m = cfg->track_width_m;
    lfh_apply_disturbance_profile(scenario, cfg);

    g_rng_state = (cfg->base_seed ^ g_runtime_scenario.seed);
    if (g_rng_state == 0U) {
        g_rng_state = 1U;
    }

    for (i = 0U; i < LF_SENSOR_COUNT; ++i) {
        g_sensor_gain[i] = 1.0 + g_sensor_gain_span * lfh_rng_symm();
        g_sensor_bias[i] = g_sensor_bias_span * lfh_rng_symm();
    }
}

void LFH_Core_ReadSensorNormAndRaw(double out_norm[LF_SENSOR_COUNT], uint16_t out_raw[LF_SENSOR_COUNT])
{
    uint32_t i;

    for (i = 0U; i < LF_SENSOR_COUNT; ++i) {
        const double local_x = k_sensor_xy[i][0];
        const double local_y = k_sensor_xy[i][1];
        const double c = cos(g_pose.theta);
        const double s = sin(g_pose.theta);
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

double LFH_Core_UpdatePoseFromCommand(double dt_sec)
{
    double v_l = lfh_command_to_speed_mps(g_left_cmd);
    double v_r = lfh_command_to_speed_mps(g_right_cmd);
    const double old_x = g_pose.x;
    const double old_y = g_pose.y;

    if (g_active_scenario != NULL) {
        v_l *= g_active_scenario->left_motor_scale;
        v_r *= g_active_scenario->right_motor_scale;
    }

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

    if (out_raw == NULL) {
        return;
    }

    LFH_Core_ReadSensorNormAndRaw(sensor_norm, out_raw);
}

uint16_t LF_Platform_RadarRead(uint8_t *out_buf, uint16_t max_len)
{
    (void)out_buf;
    (void)max_len;
    return 0U;
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
