#include "lf_harness_report.h"

#include <stdio.h>
#include <time.h>

static const char *lfh_profile_name(LFH_DisturbanceProfile profile)
{
    if (profile == LFH_PROFILE_STRESS) {
        return "stress";
    }
    return "normal";
}

static LFH_ConfidenceAssessment lfh_default_confidence(const LFH_TestConfig *cfg, const LFH_SuiteSummary *summary)
{
    LFH_ConfidenceAssessment conf;

    conf.level = "Low";
    conf.profile = lfh_profile_name(cfg->disturbance_profile);
    conf.rule_version = LFH_CONFIDENCE_RULE_VERSION;
    conf.passed_hard_gate = false;
    conf.score = summary->overall_score;
    conf.detect_percent = summary->avg_line_detection_rate * 100.0;
    conf.max_lost_sec = summary->max_longest_lost_sec;
    conf.high_min_score = 0.0;
    conf.high_min_detect_percent = 0.0;
    conf.high_max_lost_sec = 0.0;
    conf.medium_min_score = 0.0;
    conf.medium_min_detect_percent = 0.0;
    conf.medium_max_lost_sec = 0.0;

    return conf;
}

bool LFH_Report_WriteJson(const char *path,
                          const LFH_TestConfig *cfg,
                          const LFH_ScenarioResult *results,
                          size_t result_count,
                          const LFH_SuiteSummary *summary,
                          const LFH_IssueList *issue_list,
                          const LFH_BaselineComparison *baseline_comparison,
                          const LFH_ConfidenceAssessment *confidence)
{
    FILE *fp;
    size_t i;
    const long now_epoch = (long)time(NULL);
    LFH_ConfidenceAssessment fallback_conf;
    const LFH_ConfidenceAssessment *conf = confidence;

    if (conf == NULL) {
        fallback_conf = lfh_default_confidence(cfg, summary);
        conf = &fallback_conf;
    }

    fp = fopen(path, "w");
    if (fp == NULL) {
        return false;
    }

    fprintf(fp, "{\n");
    fprintf(fp, "  \"version\": %u,\n", LFH_REPORT_SCHEMA_VERSION);
    fprintf(fp, "  \"schema\": \"lf_autotest_report_v6\",\n");
    fprintf(fp, "  \"generatedAtEpochSec\": %ld,\n", now_epoch);
    fprintf(fp, "  \"config\": {\n");
    fprintf(fp, "    \"durationSec\": %.6f,\n", cfg->duration_sec);
    fprintf(fp, "    \"dt\": %.6f,\n", cfg->dt_sec);
    fprintf(fp, "    \"lineThreshold\": %.6f,\n", cfg->line_threshold);
    fprintf(fp, "    \"baseSeed\": %u,\n", cfg->base_seed);
    fprintf(fp, "    \"disturbanceProfile\": \"%s\"\n", lfh_profile_name(cfg->disturbance_profile));
    fprintf(fp, "  },\n");
    fprintf(fp, "  \"summary\": {\n");
    fprintf(fp, "    \"scenarioCount\": %u,\n", summary->scenario_count);
    fprintf(fp, "    \"completedCount\": %u,\n", summary->completed_count);
    fprintf(fp, "    \"aborted\": %s,\n", summary->aborted ? "true" : "false");
    fprintf(fp, "    \"overallScore\": %.6f,\n", summary->overall_score);
    fprintf(fp, "    \"avgLineDetectionRate\": %.6f,\n", summary->avg_line_detection_rate);
    fprintf(fp, "    \"maxLongestLostSec\": %.6f,\n", summary->max_longest_lost_sec);
    fprintf(fp, "    \"fullCourse\": {\n");
    fprintf(fp, "      \"scenarioCount\": %u,\n", summary->full_course_scenario_count);
    fprintf(fp, "      \"passedCount\": %u,\n", summary->full_course_passed_count);
    fprintf(fp, "      \"allPassed\": %s,\n", summary->full_course_all_passed ? "true" : "false");
    fprintf(fp, "      \"minProgressPercent\": %.6f,\n", summary->full_course_min_progress_percent);
    fprintf(fp, "      \"maxFinishTimeSec\": %.6f\n", summary->full_course_max_finish_time_sec);
    fprintf(fp, "    }\n");
    fprintf(fp, "  },\n");

    fprintf(fp, "  \"confidence\": {\n");
    fprintf(fp, "    \"level\": \"%s\",\n", conf->level);
    fprintf(fp, "    \"profile\": \"%s\",\n", conf->profile);
    fprintf(fp, "    \"ruleVersion\": \"%s\",\n", conf->rule_version);
    fprintf(fp, "    \"passedHardGate\": %s,\n", conf->passed_hard_gate ? "true" : "false");
    fprintf(fp, "    \"score\": %.6f,\n", conf->score);
    fprintf(fp, "    \"detectPercent\": %.6f,\n", conf->detect_percent);
    fprintf(fp, "    \"maxLostSec\": %.6f,\n", conf->max_lost_sec);
    fprintf(fp, "    \"thresholds\": {\n");
    fprintf(fp, "      \"high\": {\n");
    fprintf(fp, "        \"minScore\": %.6f,\n", conf->high_min_score);
    fprintf(fp, "        \"minDetectPercent\": %.6f,\n", conf->high_min_detect_percent);
    fprintf(fp, "        \"maxLostSec\": %.6f\n", conf->high_max_lost_sec);
    fprintf(fp, "      },\n");
    fprintf(fp, "      \"medium\": {\n");
    fprintf(fp, "        \"minScore\": %.6f,\n", conf->medium_min_score);
    fprintf(fp, "        \"minDetectPercent\": %.6f,\n", conf->medium_min_detect_percent);
    fprintf(fp, "        \"maxLostSec\": %.6f\n", conf->medium_max_lost_sec);
    fprintf(fp, "      }\n");
    fprintf(fp, "    }\n");
    fprintf(fp, "  },\n");

    if (baseline_comparison == NULL) {
        fprintf(fp, "  \"baselineComparison\": {\n");
        fprintf(fp, "    \"enabled\": false,\n");
        fprintf(fp, "    \"loaded\": false,\n");
        fprintf(fp, "    \"withinRegressionGate\": true,\n");
        fprintf(fp, "    \"regressionGatePct\": %.6f,\n", LFH_REGRESSION_GATE_PCT);
        fprintf(fp, "    \"baselineReportPath\": null,\n");
        fprintf(fp, "    \"scoreRegressionPct\": 0.000000,\n");
        fprintf(fp, "    \"detectionRegressionPct\": 0.000000,\n");
        fprintf(fp, "    \"maxLostRegressionPct\": 0.000000,\n");
        fprintf(fp, "    \"error\": null\n");
        fprintf(fp, "  },\n");
    } else {
        fprintf(fp, "  \"baselineComparison\": {\n");
        fprintf(fp, "    \"enabled\": %s,\n", baseline_comparison->enabled ? "true" : "false");
        fprintf(fp, "    \"loaded\": %s,\n", baseline_comparison->loaded ? "true" : "false");
        fprintf(fp,
                "    \"withinRegressionGate\": %s,\n",
                baseline_comparison->within_regression_gate ? "true" : "false");
        fprintf(fp, "    \"regressionGatePct\": %.6f,\n", baseline_comparison->regression_gate_pct);

        if (baseline_comparison->baseline_report_path == NULL) {
            fprintf(fp, "    \"baselineReportPath\": null,\n");
        } else {
            fprintf(fp, "    \"baselineReportPath\": \"%s\",\n", baseline_comparison->baseline_report_path);
        }

        fprintf(fp, "    \"baselineOverallScore\": %.6f,\n", baseline_comparison->baseline_overall_score);
        fprintf(fp,
                "    \"baselineAvgLineDetectionRate\": %.6f,\n",
                baseline_comparison->baseline_avg_line_detection_rate);
        fprintf(fp,
                "    \"baselineMaxLongestLostSec\": %.6f,\n",
                baseline_comparison->baseline_max_longest_lost_sec);
        fprintf(fp, "    \"currentOverallScore\": %.6f,\n", baseline_comparison->current_overall_score);
        fprintf(fp,
                "    \"currentAvgLineDetectionRate\": %.6f,\n",
                baseline_comparison->current_avg_line_detection_rate);
        fprintf(fp,
                "    \"currentMaxLongestLostSec\": %.6f,\n",
                baseline_comparison->current_max_longest_lost_sec);
        fprintf(fp,
                "    \"scoreRegressionPct\": %.6f,\n",
                baseline_comparison->score_regression_pct);
        fprintf(fp,
                "    \"detectionRegressionPct\": %.6f,\n",
                baseline_comparison->detection_regression_pct);
        fprintf(fp,
                "    \"maxLostRegressionPct\": %.6f,\n",
                baseline_comparison->max_lost_regression_pct);

        if (baseline_comparison->error_message[0] == '\0') {
            fprintf(fp, "    \"error\": null\n");
        } else {
            fprintf(fp, "    \"error\": \"%s\"\n", baseline_comparison->error_message);
        }

        fprintf(fp, "  },\n");
    }

    fprintf(fp, "  \"issues\": [");
    for (i = 0U; i < issue_list->issue_count; ++i) {
        fprintf(fp, "%s\"%s\"", (i == 0U) ? "" : ", ", issue_list->issues[i]);
    }
    fprintf(fp, "],\n");

    fprintf(fp, "  \"scenarios\": [\n");
    for (i = 0U; i < result_count; ++i) {
        const LFH_ScenarioResult *r = &results[i];
        fprintf(fp, "    {\n");
        fprintf(fp, "      \"id\": \"%s\",\n", r->id);
        fprintf(fp, "      \"name\": \"%s\",\n", r->name);
        fprintf(fp, "      \"preset\": \"%s\",\n", r->preset);
        fprintf(fp, "      \"durationTargetSec\": %.6f,\n", r->duration_target_sec);
        fprintf(fp, "      \"durationSimulatedSec\": %.6f,\n", r->duration_simulated_sec);
        fprintf(fp, "      \"steps\": %u,\n", r->steps);
        fprintf(fp, "      \"lineDetectionRate\": %.6f,\n", r->line_detection_rate);
        fprintf(fp, "      \"lineLostTransitions\": %u,\n", r->line_lost_transitions);
        fprintf(fp, "      \"lineRecoveredTransitions\": %u,\n", r->line_recovered_transitions);
        fprintf(fp, "      \"longestLostSec\": %.6f,\n", r->longest_lost_sec);
        fprintf(fp, "      \"totalLostSec\": %.6f,\n", r->total_lost_sec);
        fprintf(fp, "      \"meanAbsErrorM\": %.6f,\n", r->mean_abs_error_m);
        fprintf(fp, "      \"rmsErrorM\": %.6f,\n", r->rms_error_m);
        fprintf(fp, "      \"maxAbsErrorM\": %.6f,\n", r->max_abs_error_m);
        fprintf(fp, "      \"motorSaturationRate\": %.6f,\n", r->motor_saturation_rate);
        fprintf(fp, "      \"distanceM\": %.6f,\n", r->distance_m);
        fprintf(fp, "      \"taskScore\": %.6f,\n", r->task_score);
        fprintf(fp, "      \"task\": {\n");
        fprintf(fp, "        \"finished\": %s,\n", r->finished ? "true" : "false");
        fprintf(fp, "        \"finishTimeSec\": %.6f,\n", r->finish_time_sec);
        fprintf(fp, "        \"collided\": %s,\n", r->collided ? "true" : "false");
        fprintf(fp, "        \"boundaryViolationCount\": %u,\n", r->boundary_violation_count);
        fprintf(fp, "        \"boundaryViolationSteps\": %u\n", r->boundary_violation_steps);
        fprintf(fp, "      },\n");
        fprintf(fp, "      \"course\": {\n");
        fprintf(fp, "        \"enabled\": %s,\n", r->full_course_enabled ? "true" : "false");
        fprintf(fp, "        \"fullCoursePassed\": %s,\n", r->full_course_passed ? "true" : "false");
        fprintf(fp, "        \"validFinish\": %s,\n", r->valid_finish ? "true" : "false");
        fprintf(fp, "        \"progressM\": %.6f,\n", r->progress_m);
        fprintf(fp, "        \"maxProgressM\": %.6f,\n", r->max_progress_m);
        fprintf(fp, "        \"courseLengthM\": %.6f,\n", r->course_length_m);
        fprintf(fp, "        \"progressPercent\": %.6f,\n", r->progress_percent);
        fprintf(fp, "        \"boundaryViolationSec\": %.6f\n", r->boundary_violation_sec);
        fprintf(fp, "      },\n");
        fprintf(fp, "      \"obstacles\": {\n");
        fprintf(fp, "        \"count\": %u,\n", r->obstacle_count);
        fprintf(fp, "        \"detected\": %u,\n", r->obstacle_detected_count);
        fprintf(fp, "        \"avoidStarted\": %u,\n", r->obstacle_avoid_started_count);
        fprintf(fp, "        \"avoidCompleted\": %u\n", r->obstacle_avoid_completed_count);
        fprintf(fp, "      },\n");
        fprintf(fp, "      \"checkpoints\": {\n");
        fprintf(fp, "        \"expected\": %u,\n", r->checkpoint_expected_count);
        fprintf(fp, "        \"triggered\": %u,\n", r->checkpoint_triggered_count);
        fprintf(fp, "        \"missed\": %u,\n", r->checkpoint_missed_count);
        fprintf(fp, "        \"duplicates\": %u,\n", r->checkpoint_duplicate_count);
        fprintf(fp, "        \"outOfOrder\": %u,\n", r->checkpoint_out_of_order_count);
        fprintf(fp, "        \"lastId\": %u\n", r->checkpoint_last_id);
        fprintf(fp, "      },\n");
        fprintf(fp, "      \"radar\": {\n");
        fprintf(fp, "        \"detections\": %u\n", r->radar_detection_count);
        fprintf(fp, "      },\n");
        fprintf(fp, "      \"wireless\": {\n");
        fprintf(fp, "        \"enabled\": %s,\n", r->wireless_enabled ? "true" : "false");
        fprintf(fp, "        \"queueDepth\": %u,\n", (unsigned)r->wireless_queue_depth);
        fprintf(fp, "        \"queueDropped\": %u,\n", (unsigned)r->wireless_queue_dropped);
        fprintf(fp, "        \"txSuccess\": %u,\n", (unsigned)r->wireless_tx_success_count);
        fprintf(fp, "        \"txFail\": %u,\n", (unsigned)r->wireless_tx_fail_count);
        fprintf(fp, "        \"retryCount\": %u,\n", (unsigned)r->wireless_retry_count);
        fprintf(fp, "        \"checkpointEnqueued\": %u,\n", (unsigned)r->wireless_checkpoint_enqueued_count);
        fprintf(fp, "        \"checkpointEnqueueFail\": %u,\n", (unsigned)r->wireless_checkpoint_enqueue_fail_count);
        fprintf(fp, "        \"checkpointThrottled\": %u\n", (unsigned)r->wireless_checkpoint_throttled_count);
        fprintf(fp, "      },\n");
        fprintf(fp,
                "      \"runtimeError\": %s,\n",
                r->has_runtime_error ? "\"State entered FAULT\"" : "null");
        fprintf(fp, "      \"score\": %.6f\n", r->score);
        fprintf(fp, "    }%s\n", (i + 1U < result_count) ? "," : "");
    }
    fprintf(fp, "  ]\n");
    fprintf(fp, "}\n");

    fclose(fp);
    return true;
}
