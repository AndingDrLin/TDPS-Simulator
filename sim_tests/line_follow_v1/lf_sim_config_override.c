#include "lf_config.h"

#ifndef TDPS_SIM_CALIBRATION_DURATION_MS
#define TDPS_SIM_CALIBRATION_DURATION_MS 3000U
#endif

#ifndef TDPS_SIM_CALIBRATION_SWITCH_MS
#define TDPS_SIM_CALIBRATION_SWITCH_MS 300U
#endif

#ifndef TDPS_SIM_CALIBRATION_SPIN_SPEED
#define TDPS_SIM_CALIBRATION_SPIN_SPEED 180
#endif

#ifndef TDPS_SIM_SENSOR_ALPHA
#define TDPS_SIM_SENSOR_ALPHA 0.35f
#endif

#ifndef TDPS_SIM_LINE_MIN_SUM
#define TDPS_SIM_LINE_MIN_SUM 780U
#endif

#ifndef TDPS_SIM_EDGE_HINT
#define TDPS_SIM_EDGE_HINT 120U
#endif

#ifndef TDPS_SIM_KP
#define TDPS_SIM_KP 0.48f
#endif

#ifndef TDPS_SIM_KI
#define TDPS_SIM_KI 0.0f
#endif

#ifndef TDPS_SIM_KD
#define TDPS_SIM_KD 1.90f
#endif

#ifndef TDPS_SIM_BASE_SPEED
#define TDPS_SIM_BASE_SPEED 320
#endif

#ifndef TDPS_SIM_MAX_CORRECTION
#define TDPS_SIM_MAX_CORRECTION 340
#endif

#ifndef TDPS_SIM_RECOVER_TURN
#define TDPS_SIM_RECOVER_TURN 220
#endif

#ifndef TDPS_SIM_RECOVER_TIMEOUT
#define TDPS_SIM_RECOVER_TIMEOUT 900U
#endif

#ifndef TDPS_SIM_DYNAMIC_CALIBRATION
#define TDPS_SIM_DYNAMIC_CALIBRATION true
#endif

const LF_Config g_lf_config = {
    .control_period_ms = 10U,
    .auto_start_delay_ms = 1200U,
    .calibration_duration_ms = TDPS_SIM_CALIBRATION_DURATION_MS,
    .calibration_switch_interval_ms = TDPS_SIM_CALIBRATION_SWITCH_MS,
    .calibration_spin_speed = TDPS_SIM_CALIBRATION_SPIN_SPEED,
    .sensor_filter_alpha = TDPS_SIM_SENSOR_ALPHA,
    .sensor_input_mode = LF_SENSOR_INPUT_ANALOG_ADC,
    .sensor_invert_polarity = false,
    .sensor_digital_threshold = 2048U,
    .sensor_digital_active_high = false,
    .sensor_use_dynamic_calibration = TDPS_SIM_DYNAMIC_CALIBRATION,
    .line_detect_min_sum = TDPS_SIM_LINE_MIN_SUM,
    .edge_hint_threshold = TDPS_SIM_EDGE_HINT,
    .sensor_weights = {-1750, -1250, -750, -250, 250, 750, 1250, 1750},
    .kp = TDPS_SIM_KP,
    .ki = TDPS_SIM_KI,
    .kd = TDPS_SIM_KD,
    .base_speed = TDPS_SIM_BASE_SPEED,
    .max_correction = TDPS_SIM_MAX_CORRECTION,
    .max_motor_cmd = 900,
    .motor_deadband = 120,
    .recover_turn_speed = TDPS_SIM_RECOVER_TURN,
    .recover_timeout_ms = TDPS_SIM_RECOVER_TIMEOUT,
    .radar_enable = true,
    .radar_uart_baudrate = 256000U,
    .radar_trigger_distance_mm = 450U,
    .radar_release_distance_mm = 650U,
    .radar_debounce_frames = 3U,
    .radar_frame_timeout_ms = 120U,
    .obstacle_warn_speed = 220,
    .obstacle_avoid_enable = true,
    .obstacle_preferred_side = 0,
    .obstacle_max_attempts = 2U,
    .obstacle_confirm_ms = 100U,
    .obstacle_stop_ms = 120U,
    .obstacle_turn_out_ms = 360U,
    .obstacle_bypass_ms = 900U,
    .obstacle_turn_in_ms = 360U,
    .obstacle_reacquire_timeout_ms = 1600U,
    .obstacle_turn_speed = 240,
    .obstacle_bypass_inner_speed = 160,
    .obstacle_bypass_outer_speed = 320,
    .obstacle_emergency_distance_mm = 280U,
};
