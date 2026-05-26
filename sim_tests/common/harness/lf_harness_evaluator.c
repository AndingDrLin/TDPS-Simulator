#include "lf_harness_evaluator.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "lf_app.h"
#include "lf_config.h"
#include "lf_harness_core.h"
#include "lf_harness_scenarios.h"
#include "lf_platform.h"
#include "wireless_hooks.h"
#include "wl_app.h"
#include "wl_lora.h"

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

static bool lfh_is_full_course_scenario(const LFH_Scenario *scenario)
{
    return scenario != NULL &&
           scenario->track == LFH_TRACK_PATIO_REAL &&
           scenario->checkpoint_enable &&
           scenario->checkpoint_expected_count >= 2U &&
           scenario->finish_enable &&
           scenario->boundary_enable;
}

static bool lfh_checkpoint_in_expected_sequence(uint32_t checkpoint_id, const uint32_t *sequence, size_t count)
{
    size_t i;

    for (i = 0U; i < count; ++i) {
        if (sequence[i] == checkpoint_id) {
            return true;
        }
    }
    return false;
}

static double lfh_score_scenario(const LFH_ScenarioResult *r)
{
    double score;

    if (r->has_runtime_error) {
        return 0.0;
    }

    score = 100.0;
    score -= (1.0 - r->line_detection_rate) * 60.0;
    score -= fmin(r->mean_abs_error_m / 0.08, 1.0) * 20.0;
    score -= fmin(r->longest_lost_sec / 1.2, 1.0) * 15.0;
    score -= fmin(r->motor_saturation_rate / 0.6, 1.0) * 5.0;
    return lfh_clamp_d(score, 0.0, 100.0);
}

static double lfh_score_task(const LFH_Scenario *scenario, const LFH_ScenarioResult *r)
{
    double score = 100.0;

    if (r->has_runtime_error) {
        return 0.0;
    }
    if (scenario->finish_enable && !r->finished) {
        score -= 35.0;
    }
    if (r->full_course_enabled && !r->valid_finish) {
        score -= 30.0;
    }
    if (r->full_course_enabled && r->progress_percent < LFH_FULL_COURSE_PROGRESS_MIN_PERCENT) {
        score -= fmin((LFH_FULL_COURSE_PROGRESS_MIN_PERCENT - r->progress_percent) / 20.0, 1.0) * 25.0;
    }
    if (r->collided) {
        score -= 35.0;
    }
    if (r->boundary_violation_count > 0U) {
        score -= 20.0;
    }
    if (r->checkpoint_missed_count > 0U) {
        score -= 15.0 * (double)r->checkpoint_missed_count;
    }
    if (r->checkpoint_triggered_count > r->checkpoint_expected_count && r->checkpoint_expected_count > 0U) {
        score -= 10.0 * (double)(r->checkpoint_triggered_count - r->checkpoint_expected_count);
    }
    if (r->checkpoint_duplicate_count > 0U) {
        score -= 5.0 * (double)r->checkpoint_duplicate_count;
    }
    if (r->checkpoint_out_of_order_count > 0U) {
        score -= 25.0 * (double)r->checkpoint_out_of_order_count;
    }
    if (scenario->obstacle_mode != LFH_OBSTACLE_NONE && r->obstacle_avoid_started_count == 0U) {
        score -= 20.0;
    }
    if (scenario->obstacle_mode != LFH_OBSTACLE_NONE &&
        r->obstacle_avoid_completed_count < r->obstacle_avoid_started_count) {
        score -= 15.0;
    }

    return lfh_clamp_d(score, 0.0, 100.0);
}

LFH_ScenarioResult LFH_Evaluator_RunScenario(const LFH_Scenario *scenario, const LFH_TestConfig *cfg)
{
    const uint32_t target_steps = (uint32_t)fmax(1.0, llround(cfg->duration_sec / cfg->dt_sec));
    LFH_ScenarioResult result;
    double current_lost_sec = 0.0;
    double abs_error_accum = 0.0;
    double sq_error_accum = 0.0;
    uint32_t error_samples = 0U;
    uint32_t motor_saturation_steps = 0U;
    double prev_motor_command_mean = 0.0;
    double prev_motor_command_delta = 0.0;
    double motor_command_delta_accum = 0.0;
    uint32_t motor_command_delta_samples = 0U;
    double lost_start_sec = 0.0;
    double reacquire_time_accum = 0.0;
    uint32_t reacquire_events = 0U;
    bool motor_command_valid = false;
    bool motor_delta_valid = false;
    bool lost_start_valid = false;
    bool prev_valid = false;
    bool prev_line_detected = false;
    bool checkpoint_seen[32] = {false};
    bool checkpoint_inside = false;
    uint32_t active_checkpoint_id = 0U;
    bool avoid_was_active = false;
    bool boundary_was_violated = false;
    const bool full_course_enabled = lfh_is_full_course_scenario(scenario);
    const uint32_t expected_sequence[] = {21U, 22U};
    bool full_checkpoint_seen[2] = {false};
    size_t expected_index = 0U;
    const size_t expected_count = sizeof(expected_sequence) / sizeof(expected_sequence[0]);
    uint32_t step;

    memset(&result, 0, sizeof(result));
    result.id = scenario->id;
    result.name = scenario->name;
    result.preset = LFH_Scenarios_TrackName(scenario->track);
    result.duration_target_sec = cfg->duration_sec;
    result.obstacle_count = scenario->obstacle_count;
    result.checkpoint_expected_count = scenario->checkpoint_enable ? scenario->checkpoint_expected_count : 0U;
    result.wireless_enabled = scenario->wireless_enable;
    result.full_course_enabled = full_course_enabled;

    LFH_Core_Reset(scenario, cfg);
    LF_Platform_BoardInit();
    Wireless_Hooks_Reset();
    if (scenario->wireless_enable) {
        (void)Wireless_Hooks_Init();
    }
    LF_App_Init();

    for (step = 0U; step < target_steps; ++step) {
        double sensor_norm[LF_SENSOR_COUNT];
        uint16_t sensor_raw[LF_SENSOR_COUNT];
        LFH_LineState st;
        const LF_AppContext *ctx;
        uint32_t checkpoint_id = 0U;

        LFH_Core_ClearSensorSampleCache();
        LFH_Core_ReadSensorNormAndRaw(sensor_norm, sensor_raw);
        st = LFH_Core_EstimateLineState(sensor_norm, cfg->line_threshold);

        LFH_Core_AdvanceTime(cfg->dt_sec);
        LF_App_RunStep();
        if (scenario->wireless_enable) {
            Wireless_Hooks_Tick();
        }
        result.distance_m += LFH_Core_UpdatePoseFromCommand(cfg->dt_sec);
        result.steps += 1U;

        if (st.line_detected) {
            result.line_detection_rate += 1.0;
            abs_error_accum += fabs(st.line_error_m);
            sq_error_accum += st.line_error_m * st.line_error_m;
            if (fabs(st.line_error_m) > result.max_abs_error_m) {
                result.max_abs_error_m = fabs(st.line_error_m);
            }
            error_samples += 1U;
            if (current_lost_sec > 0.0) {
                if (current_lost_sec > result.longest_lost_sec) {
                    result.longest_lost_sec = current_lost_sec;
                }
                current_lost_sec = 0.0;
            }
        } else {
            current_lost_sec += cfg->dt_sec;
            result.total_lost_sec += cfg->dt_sec;
            if (prev_valid && prev_line_detected) {
                result.line_lost_transitions += 1U;
                lost_start_sec = (double)result.steps * cfg->dt_sec;
                lost_start_valid = true;
            }
        }

        if (prev_valid && (!prev_line_detected) && st.line_detected) {
            result.line_recovered_transitions += 1U;
            if (lost_start_valid) {
                reacquire_time_accum += fmax(0.0, (double)result.steps * cfg->dt_sec - lost_start_sec);
                reacquire_events += 1U;
                lost_start_valid = false;
            }
        }
        prev_line_detected = st.line_detected;
        prev_valid = true;

        {
            double motor_command_mean =
                ((double)abs(LFH_Core_GetLeftCommand()) + (double)abs(LFH_Core_GetRightCommand())) * 0.5;
            if (motor_command_valid) {
                double delta = fabs(motor_command_mean - prev_motor_command_mean);
                if (delta > result.max_motor_command_delta) {
                    result.max_motor_command_delta = delta;
                }
                motor_command_delta_accum += delta;
                motor_command_delta_samples += 1U;
                if (motor_delta_valid) {
                    double jerk = fabs(delta - prev_motor_command_delta);
                    if (jerk > result.max_motor_jerk) {
                        result.max_motor_jerk = jerk;
                    }
                }
                prev_motor_command_delta = delta;
                motor_delta_valid = true;
            }
            prev_motor_command_mean = motor_command_mean;
            motor_command_valid = true;
        }

        if ((abs(LFH_Core_GetLeftCommand()) >= (g_lf_config.max_motor_cmd - 1)) ||
            (abs(LFH_Core_GetRightCommand()) >= (g_lf_config.max_motor_cmd - 1))) {
            motor_saturation_steps += 1U;
        }

        ctx = LF_App_GetContext();
        if (ctx->obstacle_state != LF_RADAR_OBSTACLE_CLEAR) {
            result.obstacle_detected_count += 1U;
        }
        if (ctx->state == LF_APP_STATE_AVOID_PREP ||
            ctx->state == LF_APP_STATE_AVOID_TURN_OUT ||
            ctx->state == LF_APP_STATE_AVOID_BYPASS ||
            ctx->state == LF_APP_STATE_AVOID_TURN_IN ||
            ctx->state == LF_APP_STATE_AVOID_REACQUIRE) {
            if (!avoid_was_active) {
                result.obstacle_avoid_started_count += 1U;
            }
            avoid_was_active = true;
        } else {
            if (avoid_was_active && ctx->state == LF_APP_STATE_RUNNING) {
                result.obstacle_avoid_completed_count += 1U;
            }
            avoid_was_active = false;
        }

        if (LFH_Core_GetCheckpointId(&checkpoint_id)) {
            if (!checkpoint_inside || checkpoint_id != active_checkpoint_id) {
                checkpoint_inside = true;
                active_checkpoint_id = checkpoint_id;
                result.checkpoint_last_id = checkpoint_id;
                if (full_course_enabled) {
                    size_t seq_i;
                    bool known_checkpoint = false;
                    for (seq_i = 0U; seq_i < expected_count; ++seq_i) {
                        if (expected_sequence[seq_i] == checkpoint_id) {
                            known_checkpoint = true;
                            if (full_checkpoint_seen[seq_i]) {
                                result.checkpoint_duplicate_count += 1U;
                            } else if (expected_index == seq_i) {
                                full_checkpoint_seen[seq_i] = true;
                                result.checkpoint_triggered_count += 1U;
                                expected_index += 1U;
                                LF_App_NotifyCheckpoint(checkpoint_id);
                            } else {
                                result.checkpoint_out_of_order_count += 1U;
                            }
                            break;
                        }
                    }
                    if (!known_checkpoint && lfh_checkpoint_in_expected_sequence(checkpoint_id, expected_sequence, expected_count)) {
                        result.checkpoint_out_of_order_count += 1U;
                    }
                } else if (checkpoint_id < (sizeof(checkpoint_seen) / sizeof(checkpoint_seen[0]))) {
                    if (!checkpoint_seen[checkpoint_id]) {
                        checkpoint_seen[checkpoint_id] = true;
                        result.checkpoint_triggered_count += 1U;
                        LF_App_NotifyCheckpoint(checkpoint_id);
                    } else {
                        result.checkpoint_duplicate_count += 1U;
                    }
                }
            }
        } else {
            checkpoint_inside = false;
            active_checkpoint_id = 0U;
        }

        if (LFH_Core_IsBoundaryViolated()) {
            result.boundary_violation_steps += 1U;
            result.boundary_violation_sec += cfg->dt_sec;
            if (!boundary_was_violated) {
                result.boundary_violation_count += 1U;
            }
            boundary_was_violated = true;
        } else {
            boundary_was_violated = false;
        }
        if (LFH_Core_IsColliding()) {
            result.collided = true;
            break;
        }
        if (LFH_Core_IsInFinishZone()) {
            result.finished = true;
            result.finish_time_sec = (double)result.steps * cfg->dt_sec;
            result.progress_m = LFH_Core_GetProgressM();
            result.max_progress_m = LFH_Core_GetMaxProgressM();
            result.course_length_m = LFH_Core_GetCourseLengthM();
            result.progress_percent = LFH_Core_GetProgressPercent();
                result.valid_finish = !full_course_enabled ||
                (result.progress_percent >= LFH_FULL_COURSE_PROGRESS_MIN_PERCENT &&
                 expected_index >= expected_count &&
                 result.checkpoint_out_of_order_count == 0U &&
                 result.checkpoint_duplicate_count == 0U &&
                 result.boundary_violation_count == 0U &&
                 !result.collided &&
                 !result.has_runtime_error);
            if (!full_course_enabled || result.valid_finish) {
                break;
            }
        }

        if (ctx->state == LF_APP_STATE_FAULT) {
            result.has_runtime_error = true;
            snprintf(result.runtime_error, sizeof(result.runtime_error), "State entered FAULT");
            break;
        }
    }

    if (current_lost_sec > result.longest_lost_sec) {
        result.longest_lost_sec = current_lost_sec;
    }

    result.duration_simulated_sec = result.steps * cfg->dt_sec;
    result.progress_m = LFH_Core_GetProgressM();
    result.max_progress_m = LFH_Core_GetMaxProgressM();
    result.course_length_m = LFH_Core_GetCourseLengthM();
    result.progress_percent = LFH_Core_GetProgressPercent();
    if (result.steps > 0U) {
        result.line_detection_rate /= (double)result.steps;
        result.motor_saturation_rate = (double)motor_saturation_steps / (double)result.steps;
    }
    if (motor_command_delta_samples > 0U) {
        result.mean_motor_command_delta = motor_command_delta_accum / (double)motor_command_delta_samples;
    }
    if (reacquire_events > 0U) {
        result.reacquire_time_ms = (reacquire_time_accum / (double)reacquire_events) * 1000.0;
    }
    result.lost_line_duration_ms = result.longest_lost_sec * 1000.0;
    result.obstacle_recovery_success =
        scenario->obstacle_mode == LFH_OBSTACLE_NONE ||
        (result.obstacle_avoid_started_count == result.obstacle_avoid_completed_count && !result.collided && !result.has_runtime_error);
    if (error_samples > 0U) {
        result.mean_abs_error_m = abs_error_accum / (double)error_samples;
        result.rms_error_m = sqrt(sq_error_accum / (double)error_samples);
    }

    result.radar_detection_count = LFH_Core_GetRadarDetectionCount();
    result.obstacle_count = LFH_Core_GetObstacleCount();
    if (scenario->wireless_enable) {
        const WL_LoRa_LinkStatus *link = WL_LoRa_GetLinkStatus();
        const WL_App_Diag *diag = WL_App_GetDiag();
        if (link != NULL) {
            result.wireless_queue_depth = link->queue_depth;
            result.wireless_queue_dropped = link->queue_dropped;
            result.wireless_tx_success_count = link->tx_success_count;
            result.wireless_tx_fail_count = link->tx_fail_count;
            result.wireless_retry_count = link->retry_count;
        }
        if (diag != NULL) {
            result.wireless_checkpoint_enqueued_count = diag->checkpoint_enqueued_count;
            result.wireless_checkpoint_enqueue_fail_count = diag->checkpoint_enqueue_fail_count;
            result.wireless_checkpoint_throttled_count = diag->checkpoint_throttled_count;
        }
    }
    if (result.checkpoint_expected_count > result.checkpoint_triggered_count) {
        result.checkpoint_missed_count = result.checkpoint_expected_count - result.checkpoint_triggered_count;
    }
    if (full_course_enabled) {
        result.valid_finish = result.finished &&
            result.progress_percent >= LFH_FULL_COURSE_PROGRESS_MIN_PERCENT &&
            expected_index >= expected_count &&
            result.checkpoint_missed_count == 0U &&
            result.checkpoint_out_of_order_count == 0U &&
            result.checkpoint_duplicate_count == 0U &&
            result.boundary_violation_count == 0U &&
            !result.collided &&
            !result.has_runtime_error;
        result.full_course_passed = result.valid_finish &&
            result.line_detection_rate >= LFH_FULL_COURSE_DETECTION_MIN &&
            result.longest_lost_sec <= LFH_FULL_COURSE_MAX_LOST_SEC &&
            (!scenario->wireless_enable ||
             (result.wireless_checkpoint_enqueued_count >= result.checkpoint_triggered_count &&
              result.wireless_checkpoint_enqueue_fail_count == 0U &&
              result.wireless_checkpoint_throttled_count == 0U &&
              result.wireless_tx_fail_count == 0U &&
              result.wireless_queue_dropped == 0U &&
              result.wireless_queue_depth == 0U));
    } else {
        result.valid_finish = result.finished;
        result.full_course_passed = false;
    }

    result.score = lfh_score_scenario(&result);
    result.task_score = lfh_score_task(scenario, &result);
    return result;
}

void LFH_Evaluator_BuildSuiteSummary(const LFH_ScenarioResult *results,
                                     size_t result_count,
                                     LFH_SuiteSummary *summary,
                                     uint32_t *low_score_count,
                                     char runtime_ids[256],
                                     size_t *runtime_issue_count)
{
    size_t i;
    uint32_t completed = 0U;
    uint32_t full_course_count = 0U;
    uint32_t full_course_passed = 0U;
    double score_sum = 0.0;
    double detect_sum = 0.0;
    double max_longest = 0.0;
    double max_motor_delta = 0.0;
    double motor_delta_sum = 0.0;
    double max_motor_jerk = 0.0;
    double reacquire_sum = 0.0;
    uint32_t reacquire_count = 0U;
    double max_lost_line_ms = 0.0;
    bool obstacle_recovery_success = true;
    double min_progress_percent = 100.0;
    double max_finish_time_sec = 0.0;

    *low_score_count = 0U;
    *runtime_issue_count = 0U;
    runtime_ids[0] = '\0';

    for (i = 0U; i < result_count; ++i) {
        if (!results[i].has_runtime_error) {
            completed += 1U;
        }

        if (results[i].longest_lost_sec > max_longest) {
            max_longest = results[i].longest_lost_sec;
        }
        if (results[i].max_motor_command_delta > max_motor_delta) {
            max_motor_delta = results[i].max_motor_command_delta;
        }
        if (results[i].max_motor_jerk > max_motor_jerk) {
            max_motor_jerk = results[i].max_motor_jerk;
        }
        if (results[i].lost_line_duration_ms > max_lost_line_ms) {
            max_lost_line_ms = results[i].lost_line_duration_ms;
        }
        if (results[i].reacquire_time_ms > 0.0) {
            reacquire_sum += results[i].reacquire_time_ms;
            reacquire_count += 1U;
        }
        if (!results[i].obstacle_recovery_success) {
            obstacle_recovery_success = false;
        }

        score_sum += results[i].score;
        detect_sum += results[i].line_detection_rate;
        motor_delta_sum += results[i].mean_motor_command_delta;

        if (results[i].has_runtime_error) {
            if (*runtime_issue_count > 0U) {
                strncat(runtime_ids, ", ", 255U - strlen(runtime_ids));
            }
            strncat(runtime_ids, results[i].id, 255U - strlen(runtime_ids));
            *runtime_issue_count += 1U;
        }

        if (results[i].full_course_enabled) {
            full_course_count += 1U;
            if (results[i].full_course_passed) {
                full_course_passed += 1U;
            }
            if (results[i].progress_percent < min_progress_percent) {
                min_progress_percent = results[i].progress_percent;
            }
            if (results[i].finish_time_sec > max_finish_time_sec) {
                max_finish_time_sec = results[i].finish_time_sec;
            }
        }

        if (results[i].score < LFH_GATE_MIN_SCENARIO_SCORE) {
            *low_score_count += 1U;
        }
    }

    summary->scenario_count = (uint32_t)result_count;
    summary->completed_count = completed;
    summary->aborted = false;
    summary->overall_score = score_sum / (double)result_count;
    summary->avg_line_detection_rate = detect_sum / (double)result_count;
    summary->max_longest_lost_sec = max_longest;
    summary->max_motor_command_delta = max_motor_delta;
    summary->mean_motor_command_delta = motor_delta_sum / (double)result_count;
    summary->max_motor_jerk = max_motor_jerk;
    summary->reacquire_time_ms = reacquire_count > 0U ? reacquire_sum / (double)reacquire_count : 0.0;
    summary->lost_line_duration_ms = max_lost_line_ms;
    summary->obstacle_recovery_success = obstacle_recovery_success;
    summary->full_course_scenario_count = full_course_count;
    summary->full_course_passed_count = full_course_passed;
    summary->full_course_all_passed = full_course_count > 0U && full_course_passed == full_course_count;
    summary->full_course_min_progress_percent = (full_course_count > 0U) ? min_progress_percent : 0.0;
    summary->full_course_max_finish_time_sec = max_finish_time_sec;
}

void LFH_Evaluator_CollectIssues(const LFH_SuiteSummary *summary,
                                 uint32_t low_score_count,
                                 const char runtime_ids[256],
                                 size_t runtime_issue_count,
                                 LFH_IssueList *issue_list)
{
    static char runtime_issue[320];

    issue_list->issue_count = 0U;

    if (runtime_issue_count > 0U) {
        snprintf(runtime_issue, sizeof(runtime_issue), "Runtime errors in: %s", runtime_ids);
        issue_list->issues[issue_list->issue_count++] = runtime_issue;
    }
    if (summary->avg_line_detection_rate < LFH_GATE_DETECTION_MIN) {
        issue_list->issues[issue_list->issue_count++] =
            "Average line detection is below 94%; tracking robustness is insufficient.";
    }
    if (summary->max_longest_lost_sec > LFH_GATE_MAX_LOST_SEC) {
        issue_list->issues[issue_list->issue_count++] =
            "Longest line loss exceeds 0.35s in at least one scenario.";
    }
    if (summary->overall_score < LFH_GATE_OVERALL_SCORE_MIN) {
        issue_list->issues[issue_list->issue_count++] =
            "Overall score is below 82; tune speed/PID/recovery for stable behavior.";
    }
    if (low_score_count > 0U) {
        issue_list->issues[issue_list->issue_count++] =
            "At least one scenario score is below 70; local weakness remains.";
    }
}

void LFH_Evaluator_CollectFullCourseIssues(const LFH_ScenarioResult *results,
                                           size_t result_count,
                                           const LFH_SuiteSummary *summary,
                                           const char runtime_ids[256],
                                           size_t runtime_issue_count,
                                           LFH_IssueList *issue_list)
{
    static char runtime_issue[320];
    static char finish_issue[160];
    static char progress_issue[160];
    static char checkpoint_issue[160];
    static char boundary_issue[160];
    static char collision_issue[160];
    static char tracking_issue[160];
    static char wireless_issue[160];
    size_t i;
    uint32_t invalid_finish_count = 0U;
    uint32_t checkpoint_problem_count = 0U;
    uint32_t boundary_problem_count = 0U;
    uint32_t collision_count = 0U;
    uint32_t tracking_problem_count = 0U;
    uint32_t wireless_problem_count = 0U;

    issue_list->issue_count = 0U;

    if (runtime_issue_count > 0U) {
        snprintf(runtime_issue, sizeof(runtime_issue), "Runtime errors in: %s", runtime_ids);
        issue_list->issues[issue_list->issue_count++] = runtime_issue;
    }

    if (summary->full_course_scenario_count == 0U) {
        issue_list->issues[issue_list->issue_count++] = "No full-course scenarios were found in the config.";
        return;
    }

    for (i = 0U; i < result_count; ++i) {
        if (!results[i].full_course_enabled) {
            continue;
        }
        if (!results[i].valid_finish) {
            invalid_finish_count += 1U;
        }
        if (results[i].checkpoint_missed_count > 0U ||
            results[i].checkpoint_out_of_order_count > 0U ||
            results[i].checkpoint_duplicate_count > 0U) {
            checkpoint_problem_count += 1U;
        }
        if (results[i].boundary_violation_count > 0U) {
            boundary_problem_count += 1U;
        }
        if (results[i].collided) {
            collision_count += 1U;
        }
        if (results[i].line_detection_rate < LFH_FULL_COURSE_DETECTION_MIN ||
            results[i].longest_lost_sec > LFH_FULL_COURSE_MAX_LOST_SEC) {
            tracking_problem_count += 1U;
        }
        if (results[i].wireless_enabled &&
            (results[i].wireless_checkpoint_enqueued_count < results[i].checkpoint_triggered_count ||
             results[i].wireless_checkpoint_enqueue_fail_count > 0U ||
             results[i].wireless_checkpoint_throttled_count > 0U ||
             results[i].wireless_tx_fail_count > 0U ||
             results[i].wireless_queue_dropped > 0U ||
             results[i].wireless_queue_depth > 0U)) {
            wireless_problem_count += 1U;
        }
    }

    if (!summary->full_course_all_passed) {
        snprintf(finish_issue,
                 sizeof(finish_issue),
                 "Full-course pass count is %u/%u.",
                 summary->full_course_passed_count,
                 summary->full_course_scenario_count);
        issue_list->issues[issue_list->issue_count++] = finish_issue;
    }
    if (invalid_finish_count > 0U) {
        issue_list->issues[issue_list->issue_count++] = "At least one full-course scenario did not reach a valid finish.";
    }
    if (summary->full_course_min_progress_percent < LFH_FULL_COURSE_PROGRESS_MIN_PERCENT) {
        snprintf(progress_issue,
                 sizeof(progress_issue),
                 "Minimum full-course progress is %.2f%%, below %.2f%%.",
                 summary->full_course_min_progress_percent,
                 LFH_FULL_COURSE_PROGRESS_MIN_PERCENT);
        issue_list->issues[issue_list->issue_count++] = progress_issue;
    }
    if (checkpoint_problem_count > 0U) {
        snprintf(checkpoint_issue,
                 sizeof(checkpoint_issue),
                 "Checkpoint sequence failed in %u full-course scenario(s).",
                 checkpoint_problem_count);
        issue_list->issues[issue_list->issue_count++] = checkpoint_issue;
    }
    if (boundary_problem_count > 0U) {
        snprintf(boundary_issue,
                 sizeof(boundary_issue),
                 "Track corridor boundary was violated in %u full-course scenario(s).",
                 boundary_problem_count);
        issue_list->issues[issue_list->issue_count++] = boundary_issue;
    }
    if (collision_count > 0U) {
        snprintf(collision_issue,
                 sizeof(collision_issue),
                 "Collision occurred in %u full-course scenario(s).",
                 collision_count);
        issue_list->issues[issue_list->issue_count++] = collision_issue;
    }
    if (tracking_problem_count > 0U) {
        snprintf(tracking_issue,
                 sizeof(tracking_issue),
                 "Tracking robustness failed full-course gates in %u scenario(s).",
                 tracking_problem_count);
        issue_list->issues[issue_list->issue_count++] = tracking_issue;
    }
    if (wireless_problem_count > 0U) {
        snprintf(wireless_issue,
                 sizeof(wireless_issue),
                 "Wireless checkpoint delivery failed in %u full-course scenario(s).",
                 wireless_problem_count);
        issue_list->issues[issue_list->issue_count++] = wireless_issue;
    }
}
