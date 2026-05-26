#include "lf_harness_cli.h"

#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>

#include "lf_harness_baseline.h"
#include "lf_harness_common.h"
#include "lf_harness_evaluator.h"
#include "lf_harness_report.h"
#include "lf_harness_scenarios.h"

#define LFH_PATH_BUF 512U

typedef enum {
    LFH_GATE_MODE_QUICK = 0,
    LFH_GATE_MODE_FULL_COURSE,
} LFH_GateMode;

static bool lfh_parse_double_arg(const char *text, double min_v, double max_v, double *out)
{
    char *end = NULL;
    double value;

    errno = 0;
    value = strtod(text, &end);
    if (errno == 0 && (end == text) == false && *end == '\0') {
        if (value >= min_v && value <= max_v) {
            *out = value;
            return true;
        }
    }

    return false;
}

static bool lfh_parse_u32_arg(const char *text, uint32_t *out)
{
    char *end = NULL;
    unsigned long value;

    errno = 0;
    value = strtoul(text, &end, 10);
    if (errno == 0 && (end == text) == false && *end == '\0' && value <= 0xffffffffUL) {
        *out = (uint32_t)value;
        return true;
    }

    return false;
}

static const char *lfh_profile_name(LFH_DisturbanceProfile profile)
{
    if (profile == LFH_PROFILE_STRESS) {
        return "stress";
    }
    return "normal";
}

static bool lfh_parse_profile_arg(const char *text, LFH_DisturbanceProfile *out)
{
    if (strcmp(text, "normal") == 0) {
        *out = LFH_PROFILE_NORMAL;
        return true;
    }
    if (strcmp(text, "stress") == 0) {
        *out = LFH_PROFILE_STRESS;
        return true;
    }
    return false;
}

static bool lfh_mkdir_p(const char *path)
{
    char tmp[LFH_PATH_BUF];
    size_t len;
    char *p;

    if (path == NULL || path[0] == '\0') {
        return false;
    }

    len = strlen(path);
    if (len >= sizeof(tmp)) {
        return false;
    }

    snprintf(tmp, sizeof(tmp), "%s", path);
    if (len > 0U && tmp[len - 1U] == '/') {
        tmp[len - 1U] = '\0';
    }

    p = tmp + 1;
    while (*p != '\0') {
        if (*p == '/') {
            int mk;
            *p = '\0';
            mk = mkdir(tmp, 0777);
            if (mk != 0 && (errno == EEXIST) == false) {
                return false;
            }
            *p = '/';
        }
        p += 1;
    }

    {
        int mk = mkdir(tmp, 0777);
        if (mk != 0 && (errno == EEXIST) == false) {
            return false;
        }
    }

    return true;
}

static bool lfh_ensure_parent_dir(const char *file_path)
{
    char tmp[LFH_PATH_BUF];
    char *slash;

    if (file_path == NULL || file_path[0] == '\0') {
        return false;
    }

    if (strlen(file_path) >= sizeof(tmp)) {
        return false;
    }

    snprintf(tmp, sizeof(tmp), "%s", file_path);
    slash = strrchr(tmp, '/');
    if (slash == NULL) {
        return true;
    }

    *slash = '\0';
    if (tmp[0] == '\0') {
        return true;
    }

    return lfh_mkdir_p(tmp);
}

static bool lfh_meets_gate(double score,
                           double detect_percent,
                           double max_lost_sec,
                           double min_score,
                           double min_detect_percent,
                           double max_lost_gate)
{
    if (score < min_score) {
        return false;
    }
    if (detect_percent < min_detect_percent) {
        return false;
    }
    if (max_lost_sec > max_lost_gate) {
        return false;
    }
    return true;
}

static void lfh_fill_confidence_thresholds(LFH_DisturbanceProfile profile,
                                           LFH_ConfidenceAssessment *confidence)
{
    if (profile == LFH_PROFILE_STRESS) {
        confidence->high_min_score = 86.0;
        confidence->high_min_detect_percent = 95.0;
        confidence->high_max_lost_sec = 0.22;
        confidence->medium_min_score = 80.0;
        confidence->medium_min_detect_percent = 93.0;
        confidence->medium_max_lost_sec = 0.35;
    } else {
        confidence->high_min_score = 90.0;
        confidence->high_min_detect_percent = 97.0;
        confidence->high_max_lost_sec = 0.15;
        confidence->medium_min_score = 85.0;
        confidence->medium_min_detect_percent = 95.0;
        confidence->medium_max_lost_sec = 0.25;
    }
}

static void lfh_compute_confidence(double score,
                                   double detect_percent,
                                   double max_lost_sec,
                                   LFH_DisturbanceProfile profile,
                                   double hard_min_score,
                                   double hard_min_detect_percent,
                                   double hard_max_lost_sec,
                                   LFH_ConfidenceAssessment *confidence)
{
    memset(confidence, 0, sizeof(*confidence));

    confidence->profile = lfh_profile_name(profile);
    confidence->rule_version = LFH_CONFIDENCE_RULE_VERSION;
    confidence->score = score;
    confidence->detect_percent = detect_percent;
    confidence->max_lost_sec = max_lost_sec;
    confidence->passed_hard_gate = lfh_meets_gate(
        score,
        detect_percent,
        max_lost_sec,
        hard_min_score,
        hard_min_detect_percent,
        hard_max_lost_sec);

    lfh_fill_confidence_thresholds(profile, confidence);

    if (lfh_meets_gate(score,
                       detect_percent,
                       max_lost_sec,
                       confidence->high_min_score,
                       confidence->high_min_detect_percent,
                       confidence->high_max_lost_sec)) {
        confidence->level = "High";
    } else if (lfh_meets_gate(score,
                              detect_percent,
                              max_lost_sec,
                              confidence->medium_min_score,
                              confidence->medium_min_detect_percent,
                              confidence->medium_max_lost_sec)) {
        confidence->level = "Medium";
    } else {
        confidence->level = "Low";
    }
}

static void lfh_compute_quick_confidence(const LFH_SuiteSummary *summary,
                                         LFH_DisturbanceProfile profile,
                                         LFH_ConfidenceAssessment *confidence)
{
    lfh_compute_confidence(summary->overall_score,
                           summary->avg_line_detection_rate * 100.0,
                           summary->max_longest_lost_sec,
                           profile,
                           LFH_GATE_OVERALL_SCORE_MIN,
                           LFH_GATE_DETECTION_MIN * 100.0,
                           LFH_GATE_MAX_LOST_SEC,
                           confidence);
}

static void lfh_compute_stability_confidence(double min_score,
                                             double min_detect_percent,
                                             double max_lost_sec,
                                             LFH_DisturbanceProfile profile,
                                             LFH_ConfidenceAssessment *confidence)
{
    lfh_compute_confidence(min_score,
                           min_detect_percent,
                           max_lost_sec,
                           profile,
                           LFH_STABILITY_MIN_SCORE,
                           LFH_STABILITY_MIN_DETECTION_PERCENT,
                           LFH_STABILITY_MAX_LOST_SEC,
                           confidence);
}

static bool lfh_write_stability_summary(const char *report_dir,
                                        double duration_sec,
                                        double dt_sec,
                                        double line_threshold,
                                        uint32_t seed_start,
                                        uint32_t runs,
                                        LFH_DisturbanceProfile profile,
                                        double min_score,
                                        double min_detect_percent,
                                        double max_lost_sec,
                                        const LFH_ConfidenceAssessment *confidence)
{
    FILE *fp;
    int n;
    char summary_path[LFH_PATH_BUF];
    const long now_epoch = (long)time(NULL);

    n = snprintf(summary_path, sizeof(summary_path), "%s/stability_summary.json", report_dir);
    if (n < 0 || (size_t)n >= sizeof(summary_path)) {
        return false;
    }

    fp = fopen(summary_path, "w");
    if (fp == NULL) {
        return false;
    }

    fprintf(fp, "{\n");
    fprintf(fp, "  \"version\": 1,\n");
    fprintf(fp, "  \"schema\": \"lf_stability_summary_v1\",\n");
    fprintf(fp, "  \"generatedAtEpochSec\": %ld,\n", now_epoch);
    fprintf(fp, "  \"config\": {\n");
    fprintf(fp, "    \"durationSec\": %.6f,\n", duration_sec);
    fprintf(fp, "    \"dt\": %.6f,\n", dt_sec);
    fprintf(fp, "    \"lineThreshold\": %.6f,\n", line_threshold);
    fprintf(fp, "    \"seedStart\": %u,\n", seed_start);
    fprintf(fp, "    \"runs\": %u,\n", runs);
    fprintf(fp, "    \"disturbanceProfile\": \"%s\"\n", lfh_profile_name(profile));
    fprintf(fp, "  },\n");
    fprintf(fp, "  \"summary\": {\n");
    fprintf(fp, "    \"minScore\": %.6f,\n", min_score);
    fprintf(fp, "    \"minDetectPercent\": %.6f,\n", min_detect_percent);
    fprintf(fp, "    \"maxLostSec\": %.6f\n", max_lost_sec);
    fprintf(fp, "  },\n");
    fprintf(fp, "  \"gates\": {\n");
    fprintf(fp, "    \"minScore\": %.6f,\n", LFH_STABILITY_MIN_SCORE);
    fprintf(fp, "    \"minDetectPercent\": %.6f,\n", LFH_STABILITY_MIN_DETECTION_PERCENT);
    fprintf(fp, "    \"maxLostSec\": %.6f\n", LFH_STABILITY_MAX_LOST_SEC);
    fprintf(fp, "  },\n");
    fprintf(fp, "  \"confidence\": {\n");
    fprintf(fp, "    \"level\": \"%s\",\n", confidence->level);
    fprintf(fp, "    \"profile\": \"%s\",\n", confidence->profile);
    fprintf(fp, "    \"ruleVersion\": \"%s\",\n", confidence->rule_version);
    fprintf(fp, "    \"passedHardGate\": %s,\n", confidence->passed_hard_gate ? "true" : "false");
    fprintf(fp, "    \"score\": %.6f,\n", confidence->score);
    fprintf(fp, "    \"detectPercent\": %.6f,\n", confidence->detect_percent);
    fprintf(fp, "    \"maxLostSec\": %.6f,\n", confidence->max_lost_sec);
    fprintf(fp, "    \"thresholds\": {\n");
    fprintf(fp, "      \"high\": {\n");
    fprintf(fp, "        \"minScore\": %.6f,\n", confidence->high_min_score);
    fprintf(fp, "        \"minDetectPercent\": %.6f,\n", confidence->high_min_detect_percent);
    fprintf(fp, "        \"maxLostSec\": %.6f\n", confidence->high_max_lost_sec);
    fprintf(fp, "      },\n");
    fprintf(fp, "      \"medium\": {\n");
    fprintf(fp, "        \"minScore\": %.6f,\n", confidence->medium_min_score);
    fprintf(fp, "        \"minDetectPercent\": %.6f,\n", confidence->medium_min_detect_percent);
    fprintf(fp, "        \"maxLostSec\": %.6f\n", confidence->medium_max_lost_sec);
    fprintf(fp, "      }\n");
    fprintf(fp, "    }\n");
    fprintf(fp, "  }\n");
    fprintf(fp, "}\n");

    fclose(fp);
    return true;
}

static void lfh_set_default_config(LFH_TestConfig *cfg)
{
    cfg->duration_sec = 15.0;
    cfg->dt_sec = 0.01;
    cfg->line_threshold = 0.12;
    cfg->line_width_m = 0.03;
    cfg->max_wheel_speed_mps = 1.0;
    cfg->track_width_m = 0.2;
    cfg->disturbance_profile = LFH_PROFILE_NORMAL;
    cfg->report_path = "TDPS-Simulator/artifacts/line_follow_v1/reports/single_run/last_autotest_report.json";
    cfg->base_seed = 20260319U;
}

static int lfh_parse_trailing_options(int argc,
                                      char **argv,
                                      int first_optional_index,
                                      bool allow_scenario_config,
                                      const char **scenario_config_path,
                                      const char **baseline_report_path,
                                      LFH_DisturbanceProfile *profile)
{
    int cursor = first_optional_index;
    int end = argc - 1;

    if (scenario_config_path != NULL) {
        *scenario_config_path = NULL;
    }
    if (baseline_report_path != NULL) {
        *baseline_report_path = NULL;
    }

    if (end >= cursor) {
        LFH_DisturbanceProfile parsed_profile;
        if (lfh_parse_profile_arg(argv[end], &parsed_profile)) {
            *profile = parsed_profile;
            end -= 1;
        }
    }

    if (allow_scenario_config && end >= cursor) {
        if (scenario_config_path != NULL) {
            *scenario_config_path = argv[cursor];
        }
        cursor += 1;
    }

    if (end >= cursor) {
        if (baseline_report_path != NULL) {
            *baseline_report_path = argv[cursor];
        }
        cursor += 1;
    }

    if (end >= cursor) {
        fprintf(stderr, "Too many optional args, expected [scenario_config] [baseline_report] [profile]\n");
        return 2;
    }

    return 0;
}

static int lfh_execute_suite(const LFH_TestConfig *cfg,
                             const char *scenario_config_path,
                             const char *baseline_report_path,
                             LFH_GateMode gate_mode,
                             LFH_SuiteSummary *out_summary)
{
    LFH_SuiteSummary summary;
    LFH_IssueList issues;
    LFH_ConfidenceAssessment confidence;
    LFH_BaselineComparison baseline_comparison;
    uint32_t low_score_count = 0U;
    size_t runtime_issue_count = 0U;
    char runtime_ids[256];
    char scenario_error[256];
    char baseline_error[256];
    size_t scenario_count = 0U;
    const LFH_Scenario *scenarios = NULL;
    LFH_ScenarioResult *results;
    size_t i;

    if (scenario_config_path == NULL) {
        scenario_config_path = LFH_Scenarios_DefaultConfigPath();
    }

    if (LFH_Scenarios_LoadFromConfig(scenario_config_path, scenario_error, sizeof(scenario_error)) == false) {
        fprintf(stderr,
                "Invalid scenario config: %s\npath: %s\n",
                (scenario_error[0] == '\0') ? "unknown error" : scenario_error,
                scenario_config_path);
        return 2;
    }

    scenarios = LFH_Scenarios_Get(&scenario_count);
    if (scenarios == NULL || scenario_count == 0U) {
        fprintf(stderr, "No scenarios loaded from config: %s\n", scenario_config_path);
        return 2;
    }

    if (lfh_ensure_parent_dir(cfg->report_path) == false) {
        fprintf(stderr, "Failed to create report directory for: %s\n", cfg->report_path);
        return 2;
    }

    results = (LFH_ScenarioResult *)calloc(scenario_count, sizeof(*results));
    if (results == NULL) {
        fprintf(stderr, "Out of memory while allocating scenario results\n");
        return 2;
    }

    for (i = 0U; i < scenario_count; ++i) {
        results[i] = LFH_Evaluator_RunScenario(&scenarios[i], cfg);

        printf("[%zu/%zu] %-20s score=%6.2f detect=%6.2f%% longestLost=%6.3fs sat=%6.2f%%\n",
               i + 1U,
               scenario_count,
               results[i].id,
               results[i].score,
               results[i].line_detection_rate * 100.0,
               results[i].longest_lost_sec,
               results[i].motor_saturation_rate * 100.0);
    }

    LFH_Evaluator_BuildSuiteSummary(
        results, scenario_count, &summary, &low_score_count, runtime_ids, &runtime_issue_count);
    if (gate_mode == LFH_GATE_MODE_FULL_COURSE) {
        LFH_Evaluator_CollectFullCourseIssues(
            results, scenario_count, &summary, runtime_ids, runtime_issue_count, &issues);
    } else {
        LFH_Evaluator_CollectIssues(&summary, low_score_count, runtime_ids, runtime_issue_count, &issues);
    }

    lfh_compute_quick_confidence(&summary, cfg->disturbance_profile, &confidence);
    if (gate_mode == LFH_GATE_MODE_FULL_COURSE) {
        confidence.passed_hard_gate = (issues.issue_count == 0U);
        if (summary.full_course_all_passed && summary.full_course_min_progress_percent >= 99.0) {
            confidence.level = "High";
        } else if (summary.full_course_all_passed) {
            confidence.level = "Medium";
        } else {
            confidence.level = "Low";
        }
    }

    LFH_Baseline_Init(&baseline_comparison);
    baseline_comparison.current_overall_score = summary.overall_score;
    baseline_comparison.current_avg_line_detection_rate = summary.avg_line_detection_rate;
    baseline_comparison.current_max_longest_lost_sec = summary.max_longest_lost_sec;

    if (baseline_report_path == NULL) {
        baseline_comparison.enabled = false;
    } else {
        if (LFH_Baseline_CompareReport(
                baseline_report_path,
                &summary,
                &baseline_comparison,
                baseline_error,
                sizeof(baseline_error)) == false) {
            free(results);
            fprintf(stderr,
                    "Baseline comparison failed: %s\n",
                    (baseline_error[0] == '\0') ? "unknown error" : baseline_error);
            return 2;
        }
    }

    if (LFH_Report_WriteJson(
            cfg->report_path,
            cfg,
            results,
            scenario_count,
            &summary,
            &issues,
            &baseline_comparison,
            &confidence) == false) {
        free(results);
        fprintf(stderr, "Failed to write report: %s\n", cfg->report_path);
        return 2;
    }

    printf("\nAuto test done: overall score %.2f/100, avg detection %.2f%%, max lost %.3fs\n",
           summary.overall_score,
           summary.avg_line_detection_rate * 100.0,
           summary.max_longest_lost_sec);
    if (gate_mode == LFH_GATE_MODE_FULL_COURSE) {
        printf("Full course: passed %u/%u, min progress %.2f%%, max finish %.3fs\n",
               summary.full_course_passed_count,
               summary.full_course_scenario_count,
               summary.full_course_min_progress_percent,
               summary.full_course_max_finish_time_sec);
    }
    printf("Report: %s\n", cfg->report_path);
    printf("Confidence: %s (profile=%s, rule=%s, hardGate=%s)\n",
           confidence.level,
           confidence.profile,
           confidence.rule_version,
           confidence.passed_hard_gate ? "PASS" : "FAIL");

    if (baseline_comparison.enabled) {
        printf("Baseline compare: scoreReg=%.3f%% detectReg=%.3f%% maxLostReg=%.3f%% gate<=%.2f%% result=%s\n",
               baseline_comparison.score_regression_pct,
               baseline_comparison.detection_regression_pct,
               baseline_comparison.max_lost_regression_pct,
               baseline_comparison.regression_gate_pct,
               baseline_comparison.within_regression_gate ? "PASS" : "FAIL");
    }

    if (issues.issue_count > 0U) {
        size_t k;
        printf("Issues:\n");
        for (k = 0U; k < issues.issue_count; ++k) {
            printf("- %s\n", issues.issues[k]);
        }
        free(results);
        return 1;
    }

    if (baseline_comparison.enabled && baseline_comparison.within_regression_gate == false) {
        printf("Issues:\n");
        printf("- Baseline regression exceeds %.2f%% guard.\n", baseline_comparison.regression_gate_pct);
        free(results);
        return 1;
    }

    if (out_summary != NULL) {
        *out_summary = summary;
    }

    free(results);
    return 0;
}

static int lfh_execute_quick(const LFH_TestConfig *cfg,
                             const char *scenario_config_path,
                             const char *baseline_report_path,
                             LFH_SuiteSummary *out_summary)
{
    return lfh_execute_suite(cfg, scenario_config_path, baseline_report_path, LFH_GATE_MODE_QUICK, out_summary);
}

static int lfh_execute_full_course(const LFH_TestConfig *cfg,
                                   const char *scenario_config_path,
                                   const char *baseline_report_path,
                                   LFH_SuiteSummary *out_summary)
{
    return lfh_execute_suite(cfg, scenario_config_path, baseline_report_path, LFH_GATE_MODE_FULL_COURSE, out_summary);
}

int LFH_Cli_RunLegacyAutotest(int argc, char **argv)
{
    LFH_TestConfig cfg;
    const char *scenario_config_path = NULL;
    const char *baseline_report_path = NULL;

    lfh_set_default_config(&cfg);

    if (argc >= 2 && lfh_parse_double_arg(argv[1], 1.0, 180.0, &cfg.duration_sec) == false) {
        fprintf(stderr, "Invalid durationSec: %s (expected 1..180)\n", argv[1]);
        return 2;
    }
    if (argc >= 3 && lfh_parse_double_arg(argv[2], 0.002, 0.05, &cfg.dt_sec) == false) {
        fprintf(stderr, "Invalid dtSec: %s (expected 0.002..0.05)\n", argv[2]);
        return 2;
    }
    if (argc >= 4 && lfh_parse_double_arg(argv[3], 0.01, 0.95, &cfg.line_threshold) == false) {
        fprintf(stderr, "Invalid lineThreshold: %s (expected 0.01..0.95)\n", argv[3]);
        return 2;
    }
    if (argc >= 5) {
        cfg.report_path = argv[4];
    }
    if (argc >= 6 && lfh_parse_u32_arg(argv[5], &cfg.base_seed) == false) {
        fprintf(stderr, "Invalid baseSeed: %s\n", argv[5]);
        return 2;
    }

    if (lfh_parse_trailing_options(argc,
                                   argv,
                                   6,
                                   true,
                                   &scenario_config_path,
                                   &baseline_report_path,
                                   &cfg.disturbance_profile) != 0) {
        return 2;
    }

    return lfh_execute_quick(&cfg, scenario_config_path, baseline_report_path, NULL);
}

static void lfh_print_usage(const char *prog)
{
    printf("Usage:\n");
    printf("  %s quick [duration] [dt] [line_threshold] [report_path] [base_seed] [scenario_config] [baseline_report] [profile]\n", prog);
    printf("  %s full-course <config_path> [duration] [dt] [line_threshold] [report_path] [base_seed] [baseline_report] [profile]\n", prog);
    printf("  %s stability [duration] [dt] [line_threshold] [seed_start] [runs] [report_dir] [scenario_config] [baseline_report] [profile]\n", prog);
    printf("  %s run-config <config_path> [duration] [dt] [line_threshold] [report_path] [base_seed] [baseline_report] [profile]\n", prog);
    printf("  %s [legacy args]    # backward-compatible quick mode\n", prog);
    printf("\n");
    printf("profile: normal (default) | stress\n");
}

static int lfh_run_quick_command(int argc, char **argv)
{
    LFH_TestConfig cfg;
    const char *scenario_config_path = NULL;
    const char *baseline_report_path = NULL;

    lfh_set_default_config(&cfg);

    if (argc >= 3 && lfh_parse_double_arg(argv[2], 1.0, 180.0, &cfg.duration_sec) == false) {
        fprintf(stderr, "Invalid durationSec: %s\n", argv[2]);
        return 2;
    }
    if (argc >= 4 && lfh_parse_double_arg(argv[3], 0.002, 0.05, &cfg.dt_sec) == false) {
        fprintf(stderr, "Invalid dtSec: %s\n", argv[3]);
        return 2;
    }
    if (argc >= 5 && lfh_parse_double_arg(argv[4], 0.01, 0.95, &cfg.line_threshold) == false) {
        fprintf(stderr, "Invalid lineThreshold: %s\n", argv[4]);
        return 2;
    }
    if (argc >= 6) {
        cfg.report_path = argv[5];
    }
    if (argc >= 7 && lfh_parse_u32_arg(argv[6], &cfg.base_seed) == false) {
        fprintf(stderr, "Invalid baseSeed: %s\n", argv[6]);
        return 2;
    }

    if (lfh_parse_trailing_options(argc,
                                   argv,
                                   7,
                                   true,
                                   &scenario_config_path,
                                   &baseline_report_path,
                                   &cfg.disturbance_profile) != 0) {
        return 2;
    }

    printf("[lf-cli][quick] duration=%.3f dt=%.3f threshold=%.3f seed=%u profile=%s\n",
           cfg.duration_sec,
           cfg.dt_sec,
           cfg.line_threshold,
           cfg.base_seed,
           lfh_profile_name(cfg.disturbance_profile));

    return lfh_execute_quick(&cfg, scenario_config_path, baseline_report_path, NULL);
}

static int lfh_run_full_course_command(int argc, char **argv)
{
    LFH_TestConfig cfg;
    const char *scenario_config_path;
    const char *baseline_report_path = NULL;

    if (argc < 3) {
        fprintf(stderr, "full-course requires <config_path>\n");
        return 2;
    }

    lfh_set_default_config(&cfg);
    cfg.duration_sec = 90.0;
    cfg.report_path = "TDPS-Simulator/artifacts/line_follow_v1/reports/single_run/last_full_course_report.json";
    scenario_config_path = argv[2];

    if (argc >= 4 && lfh_parse_double_arg(argv[3], 1.0, 180.0, &cfg.duration_sec) == false) {
        fprintf(stderr, "Invalid durationSec: %s\n", argv[3]);
        return 2;
    }
    if (argc >= 5 && lfh_parse_double_arg(argv[4], 0.002, 0.05, &cfg.dt_sec) == false) {
        fprintf(stderr, "Invalid dtSec: %s\n", argv[4]);
        return 2;
    }
    if (argc >= 6 && lfh_parse_double_arg(argv[5], 0.01, 0.95, &cfg.line_threshold) == false) {
        fprintf(stderr, "Invalid lineThreshold: %s\n", argv[5]);
        return 2;
    }
    if (argc >= 7) {
        cfg.report_path = argv[6];
    }
    if (argc >= 8 && lfh_parse_u32_arg(argv[7], &cfg.base_seed) == false) {
        fprintf(stderr, "Invalid baseSeed: %s\n", argv[7]);
        return 2;
    }

    if (lfh_parse_trailing_options(argc,
                                   argv,
                                   8,
                                   false,
                                   NULL,
                                   &baseline_report_path,
                                   &cfg.disturbance_profile) != 0) {
        return 2;
    }

    printf("[lf-cli][full-course] config=%s duration=%.3f dt=%.3f threshold=%.3f seed=%u profile=%s\n",
           scenario_config_path,
           cfg.duration_sec,
           cfg.dt_sec,
           cfg.line_threshold,
           cfg.base_seed,
           lfh_profile_name(cfg.disturbance_profile));

    return lfh_execute_full_course(&cfg, scenario_config_path, baseline_report_path, NULL);
}

static int lfh_run_run_config_command(int argc, char **argv)
{
    LFH_TestConfig cfg;
    const char *scenario_config_path;
    const char *baseline_report_path = NULL;

    if (argc < 3) {
        fprintf(stderr, "run-config requires <config_path>\n");
        return 2;
    }

    lfh_set_default_config(&cfg);
    cfg.report_path = "TDPS-Simulator/artifacts/line_follow_v1/reports/single_run/last_runconfig_report.json";
    scenario_config_path = argv[2];

    if (argc >= 4 && lfh_parse_double_arg(argv[3], 1.0, 180.0, &cfg.duration_sec) == false) {
        fprintf(stderr, "Invalid durationSec: %s\n", argv[3]);
        return 2;
    }
    if (argc >= 5 && lfh_parse_double_arg(argv[4], 0.002, 0.05, &cfg.dt_sec) == false) {
        fprintf(stderr, "Invalid dtSec: %s\n", argv[4]);
        return 2;
    }
    if (argc >= 6 && lfh_parse_double_arg(argv[5], 0.01, 0.95, &cfg.line_threshold) == false) {
        fprintf(stderr, "Invalid lineThreshold: %s\n", argv[5]);
        return 2;
    }
    if (argc >= 7) {
        cfg.report_path = argv[6];
    }
    if (argc >= 8 && lfh_parse_u32_arg(argv[7], &cfg.base_seed) == false) {
        fprintf(stderr, "Invalid baseSeed: %s\n", argv[7]);
        return 2;
    }

    if (lfh_parse_trailing_options(argc,
                                   argv,
                                   8,
                                   false,
                                   NULL,
                                   &baseline_report_path,
                                   &cfg.disturbance_profile) != 0) {
        return 2;
    }

    printf("[lf-cli][run-config] config=%s duration=%.3f dt=%.3f threshold=%.3f seed=%u profile=%s\n",
           scenario_config_path,
           cfg.duration_sec,
           cfg.dt_sec,
           cfg.line_threshold,
           cfg.base_seed,
           lfh_profile_name(cfg.disturbance_profile));

    return lfh_execute_quick(&cfg, scenario_config_path, baseline_report_path, NULL);
}

static int lfh_run_stability_command(int argc, char **argv)
{
    double duration_sec = 15.0;
    double dt_sec = 0.01;
    double line_threshold = 0.12;
    uint32_t seed_start = 20260319U;
    uint32_t runs = 8U;
    const char *report_dir = "TDPS-Simulator/artifacts/line_follow_v1/reports/stability_runs";
    const char *scenario_config_path = NULL;
    const char *baseline_report_path = NULL;
    LFH_DisturbanceProfile profile = LFH_PROFILE_NORMAL;
    LFH_ConfidenceAssessment confidence;
    double min_score = 101.0;
    double min_detect = 101.0;
    double max_lost = 0.0;
    uint32_t i;

    if (argc >= 3 && lfh_parse_double_arg(argv[2], 1.0, 180.0, &duration_sec) == false) {
        fprintf(stderr, "Invalid durationSec: %s\n", argv[2]);
        return 2;
    }
    if (argc >= 4 && lfh_parse_double_arg(argv[3], 0.002, 0.05, &dt_sec) == false) {
        fprintf(stderr, "Invalid dtSec: %s\n", argv[3]);
        return 2;
    }
    if (argc >= 5 && lfh_parse_double_arg(argv[4], 0.01, 0.95, &line_threshold) == false) {
        fprintf(stderr, "Invalid lineThreshold: %s\n", argv[4]);
        return 2;
    }
    if (argc >= 6 && lfh_parse_u32_arg(argv[5], &seed_start) == false) {
        fprintf(stderr, "Invalid seedStart: %s\n", argv[5]);
        return 2;
    }
    if (argc >= 7 && lfh_parse_u32_arg(argv[6], &runs) == false) {
        fprintf(stderr, "Invalid runs: %s\n", argv[6]);
        return 2;
    }
    if (runs == 0U) {
        fprintf(stderr, "Invalid runs: must be >= 1\n");
        return 2;
    }
    if (argc >= 8) {
        report_dir = argv[7];
    }

    if (lfh_parse_trailing_options(argc,
                                   argv,
                                   8,
                                   true,
                                   &scenario_config_path,
                                   &baseline_report_path,
                                   &profile) != 0) {
        return 2;
    }

    if (lfh_mkdir_p(report_dir) == false) {
        fprintf(stderr, "Failed to create report dir: %s\n", report_dir);
        return 2;
    }

    printf("[lf-cli][stability] runs=%u start_seed=%u duration=%.3f dt=%.3f threshold=%.3f profile=%s\n",
           runs,
           seed_start,
           duration_sec,
           dt_sec,
           line_threshold,
           lfh_profile_name(profile));

    for (i = 0U; i < runs; ++i) {
        LFH_TestConfig cfg;
        LFH_SuiteSummary run_summary;
        char report_path[LFH_PATH_BUF];
        int n;
        int ec;
        const uint32_t seed = seed_start + i;

        lfh_set_default_config(&cfg);
        cfg.duration_sec = duration_sec;
        cfg.dt_sec = dt_sec;
        cfg.line_threshold = line_threshold;
        cfg.base_seed = seed;
        cfg.disturbance_profile = profile;

        n = snprintf(report_path, sizeof(report_path), "%s/report_seed_%u.json", report_dir, seed);
        if (n < 0 || (size_t)n >= sizeof(report_path)) {
            fprintf(stderr, "Report path is too long for seed=%u\n", seed);
            return 2;
        }
        cfg.report_path = report_path;

        printf("===== stability run %u/%u seed=%u =====\n", i + 1U, runs, seed);
        ec = lfh_execute_quick(&cfg, scenario_config_path, baseline_report_path, &run_summary);
        if (ec != 0) {
            printf("stability check failed at seed=%u\n", seed);
            return ec;
        }

        if (run_summary.overall_score < min_score) {
            min_score = run_summary.overall_score;
        }
        if (run_summary.avg_line_detection_rate * 100.0 < min_detect) {
            min_detect = run_summary.avg_line_detection_rate * 100.0;
        }
        if (run_summary.max_longest_lost_sec > max_lost) {
            max_lost = run_summary.max_longest_lost_sec;
        }
    }

    lfh_compute_stability_confidence(min_score, min_detect, max_lost, profile, &confidence);

    printf("===== stability summary =====\n");
    printf("runs=%u min_score=%.2f min_detect=%.2f%% max_lost=%.3fs\n",
           runs,
           min_score,
           min_detect,
           max_lost);
    printf("confidence=%s profile=%s rule=%s hardGate=%s\n",
           confidence.level,
           confidence.profile,
           confidence.rule_version,
           confidence.passed_hard_gate ? "PASS" : "FAIL");
    printf("confidence thresholds: High(score>=%.2f, detect>=%.2f%%, lost<=%.3fs) ",
           confidence.high_min_score,
           confidence.high_min_detect_percent,
           confidence.high_max_lost_sec);
    printf("Medium(score>=%.2f, detect>=%.2f%%, lost<=%.3fs)\n",
           confidence.medium_min_score,
           confidence.medium_min_detect_percent,
           confidence.medium_max_lost_sec);
    printf("reports: %s\n", report_dir);

    if (lfh_write_stability_summary(report_dir,
                                    duration_sec,
                                    dt_sec,
                                    line_threshold,
                                    seed_start,
                                    runs,
                                    profile,
                                    min_score,
                                    min_detect,
                                    max_lost,
                                    &confidence) == false) {
        fprintf(stderr, "Failed to write stability summary to %s/stability_summary.json\n", report_dir);
        return 2;
    }

    if (min_score < LFH_STABILITY_MIN_SCORE ||
        min_detect < LFH_STABILITY_MIN_DETECTION_PERCENT ||
        max_lost > LFH_STABILITY_MAX_LOST_SEC) {
        printf("stability gate failed: expected min_score>=%.2f min_detect>=%.2f%% max_lost<=%.3fs\n",
               LFH_STABILITY_MIN_SCORE,
               LFH_STABILITY_MIN_DETECTION_PERCENT,
               LFH_STABILITY_MAX_LOST_SEC);
        return 1;
    }

    return 0;
}

int LFH_Cli_Run(int argc, char **argv)
{
    if (argc >= 2) {
        if (strcmp(argv[1], "quick") == 0) {
            return lfh_run_quick_command(argc, argv);
        }
        if (strcmp(argv[1], "full-course") == 0) {
            return lfh_run_full_course_command(argc, argv);
        }
        if (strcmp(argv[1], "stability") == 0) {
            return lfh_run_stability_command(argc, argv);
        }
        if (strcmp(argv[1], "run-config") == 0) {
            return lfh_run_run_config_command(argc, argv);
        }
        if (strcmp(argv[1], "--help") == 0 || strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "help") == 0) {
            lfh_print_usage(argv[0]);
            return 0;
        }
    }

    return LFH_Cli_RunLegacyAutotest(argc, argv);
}
