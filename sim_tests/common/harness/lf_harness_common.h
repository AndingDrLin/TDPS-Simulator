#ifndef LF_HARNESS_COMMON_H
#define LF_HARNESS_COMMON_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "lf_config.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define LFH_REPORT_SCHEMA_VERSION 6U
#define LFH_MAX_ISSUES 16U

#define LFH_GATE_OVERALL_SCORE_MIN 82.0
#define LFH_GATE_DETECTION_MIN 0.94
#define LFH_GATE_MAX_LOST_SEC 0.35
#define LFH_GATE_MIN_SCENARIO_SCORE 70.0

#define LFH_FULL_COURSE_PROGRESS_MIN_PERCENT 95.0
#define LFH_FULL_COURSE_DETECTION_MIN 0.90
#define LFH_FULL_COURSE_MAX_LOST_SEC 0.60

#define LFH_STABILITY_MIN_SCORE 80.0
#define LFH_STABILITY_MIN_DETECTION_PERCENT 93.0
#define LFH_STABILITY_MAX_LOST_SEC 0.40

#define LFH_REGRESSION_GATE_PCT 5.0
#define LFH_CONFIDENCE_RULE_VERSION "lf_confidence_v1"

typedef enum {
    LFH_PROFILE_NORMAL = 0,
    LFH_PROFILE_STRESS,
} LFH_DisturbanceProfile;

typedef enum {
    LFH_TRACK_CIRCLE = 0,
    LFH_TRACK_FIGURE8,
    LFH_TRACK_PATIO_PROXY,
    LFH_TRACK_PATIO_REAL,
} LFH_TrackType;

typedef enum {
    LFH_OBSTACLE_NONE = 0,
    LFH_OBSTACLE_FIXED,
    LFH_OBSTACLE_RANDOM,
} LFH_ObstacleMode;

typedef struct {
    double x;
    double y;
    double theta;
} LFH_Pose;

typedef struct {
    const char *id;
    const char *name;
    LFH_TrackType track;
    LFH_Pose start_pose;
    double noise_std;
    double dropout_prob;
    double contrast;
    double left_motor_scale;
    double right_motor_scale;
    uint32_t seed;
    bool radar_enable;
    LFH_ObstacleMode obstacle_mode;
    uint32_t obstacle_count;
    uint32_t obstacle_seed;
    bool checkpoint_enable;
    uint32_t checkpoint_expected_count;
    bool wireless_enable;
    bool finish_enable;
    bool boundary_enable;
} LFH_Scenario;

typedef struct {
    bool line_detected;
    double line_error_m;
    double confidence;
} LFH_LineState;

typedef struct {
    const char *id;
    const char *name;
    const char *preset;
    double duration_target_sec;
    double duration_simulated_sec;
    uint32_t steps;
    double line_detection_rate;
    uint32_t line_lost_transitions;
    uint32_t line_recovered_transitions;
    double longest_lost_sec;
    double total_lost_sec;
    double mean_abs_error_m;
    double rms_error_m;
    double max_abs_error_m;
    double motor_saturation_rate;
    double max_motor_command_delta;
    double mean_motor_command_delta;
    double max_motor_jerk;
    double reacquire_time_ms;
    double lost_line_duration_ms;
    bool obstacle_recovery_success;
    double distance_m;
    double score;
    double task_score;
    bool finished;
    double finish_time_sec;
    bool collided;
    uint32_t boundary_violation_count;
    uint32_t boundary_violation_steps;
    double boundary_violation_sec;
    bool full_course_enabled;
    bool valid_finish;
    bool full_course_passed;
    double progress_m;
    double max_progress_m;
    double course_length_m;
    double progress_percent;
    uint32_t obstacle_count;
    uint32_t obstacle_detected_count;
    uint32_t obstacle_avoid_started_count;
    uint32_t obstacle_avoid_completed_count;
    uint32_t checkpoint_expected_count;
    uint32_t checkpoint_triggered_count;
    uint32_t checkpoint_missed_count;
    uint32_t checkpoint_duplicate_count;
    uint32_t checkpoint_out_of_order_count;
    uint32_t checkpoint_last_id;
    uint32_t radar_detection_count;
    bool wireless_enabled;
    uint16_t wireless_queue_depth;
    uint16_t wireless_queue_dropped;
    uint16_t wireless_tx_success_count;
    uint16_t wireless_tx_fail_count;
    uint16_t wireless_retry_count;
    uint16_t wireless_checkpoint_enqueued_count;
    uint16_t wireless_checkpoint_enqueue_fail_count;
    uint16_t wireless_checkpoint_throttled_count;
    bool has_runtime_error;
    char runtime_error[160];
} LFH_ScenarioResult;

typedef struct {
    double duration_sec;
    double dt_sec;
    double line_threshold;
    double line_width_m;
    double max_wheel_speed_mps;
    double track_width_m;
    LFH_DisturbanceProfile disturbance_profile;
    const char *report_path;
    uint32_t base_seed;
} LFH_TestConfig;

typedef struct {
    uint32_t scenario_count;
    uint32_t completed_count;
    bool aborted;
    double overall_score;
    double avg_line_detection_rate;
    double max_longest_lost_sec;
    double max_motor_command_delta;
    double mean_motor_command_delta;
    double max_motor_jerk;
    double reacquire_time_ms;
    double lost_line_duration_ms;
    bool obstacle_recovery_success;
    uint32_t full_course_scenario_count;
    uint32_t full_course_passed_count;
    bool full_course_all_passed;
    double full_course_min_progress_percent;
    double full_course_max_finish_time_sec;
} LFH_SuiteSummary;

typedef struct {
    const char *issues[LFH_MAX_ISSUES];
    size_t issue_count;
} LFH_IssueList;

typedef struct {
    const char *level;
    const char *profile;
    const char *rule_version;
    bool passed_hard_gate;
    double score;
    double detect_percent;
    double max_lost_sec;
    double high_min_score;
    double high_min_detect_percent;
    double high_max_lost_sec;
    double medium_min_score;
    double medium_min_detect_percent;
    double medium_max_lost_sec;
} LFH_ConfidenceAssessment;

typedef struct {
    bool enabled;
    bool loaded;
    bool within_regression_gate;
    double regression_gate_pct;

    const char *baseline_report_path;
    double baseline_overall_score;
    double baseline_avg_line_detection_rate;
    double baseline_max_longest_lost_sec;

    double current_overall_score;
    double current_avg_line_detection_rate;
    double current_max_longest_lost_sec;

    double score_regression_pct;
    double detection_regression_pct;
    double max_lost_regression_pct;

    char error_message[160];
} LFH_BaselineComparison;

#endif
