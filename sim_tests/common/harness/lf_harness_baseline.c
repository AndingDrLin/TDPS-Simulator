#include "lf_harness_baseline.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void lfh_copy_error(char *dst, size_t dst_size, const char *message)
{
    if ((dst == NULL) == false && dst_size > 0U) {
        snprintf(dst, dst_size, "%s", message);
    }
}

static bool lfh_extract_json_number(const char *json, const char *key, double *out)
{
    const char *pos;
    const char *colon;
    char *end = NULL;
    double value;

    pos = strstr(json, key);
    if (pos == NULL) {
        return false;
    }

    colon = strchr(pos, ':');
    if (colon == NULL) {
        return false;
    }

    errno = 0;
    value = strtod(colon + 1, &end);
    if (errno == 0 && (end == colon + 1) == false) {
        *out = value;
        return true;
    }

    return false;
}

void LFH_Baseline_Init(LFH_BaselineComparison *comparison)
{
    memset(comparison, 0, sizeof(*comparison));
    comparison->regression_gate_pct = LFH_REGRESSION_GATE_PCT;
    comparison->within_regression_gate = true;
}

bool LFH_Baseline_CompareReport(const char *baseline_report_path,
                                const LFH_SuiteSummary *current,
                                LFH_BaselineComparison *comparison,
                                char *error_buf,
                                size_t error_buf_size)
{
    FILE *fp;
    long file_size;
    char *json_buf;
    size_t read_size;
    double baseline_overall;
    double baseline_detect;
    double baseline_max_lost;

    LFH_Baseline_Init(comparison);
    comparison->enabled = true;
    comparison->baseline_report_path = baseline_report_path;
    comparison->current_overall_score = current->overall_score;
    comparison->current_avg_line_detection_rate = current->avg_line_detection_rate;
    comparison->current_max_longest_lost_sec = current->max_longest_lost_sec;

    if (baseline_report_path == NULL || baseline_report_path[0] == '\0') {
        lfh_copy_error(error_buf, error_buf_size, "baseline report path is empty");
        lfh_copy_error(comparison->error_message, sizeof(comparison->error_message), "baseline report path is empty");
        comparison->within_regression_gate = false;
        return false;
    }

    fp = fopen(baseline_report_path, "r");
    if (fp == NULL) {
        lfh_copy_error(error_buf, error_buf_size, "cannot open baseline report");
        lfh_copy_error(comparison->error_message, sizeof(comparison->error_message), "cannot open baseline report");
        comparison->within_regression_gate = false;
        return false;
    }

    if (fseek(fp, 0L, SEEK_END) == 0) {
        file_size = ftell(fp);
    } else {
        fclose(fp);
        lfh_copy_error(error_buf, error_buf_size, "failed to seek baseline report");
        lfh_copy_error(comparison->error_message, sizeof(comparison->error_message), "failed to seek baseline report");
        comparison->within_regression_gate = false;
        return false;
    }

    if (file_size <= 0L) {
        fclose(fp);
        lfh_copy_error(error_buf, error_buf_size, "baseline report is empty");
        lfh_copy_error(comparison->error_message, sizeof(comparison->error_message), "baseline report is empty");
        comparison->within_regression_gate = false;
        return false;
    }

    if ((fseek(fp, 0L, SEEK_SET) == 0) == false) {
        fclose(fp);
        lfh_copy_error(error_buf, error_buf_size, "failed to rewind baseline report");
        lfh_copy_error(comparison->error_message, sizeof(comparison->error_message), "failed to rewind baseline report");
        comparison->within_regression_gate = false;
        return false;
    }

    json_buf = (char *)malloc((size_t)file_size + 1U);
    if (json_buf == NULL) {
        fclose(fp);
        lfh_copy_error(error_buf, error_buf_size, "out of memory while reading baseline report");
        lfh_copy_error(comparison->error_message, sizeof(comparison->error_message), "out of memory while reading baseline report");
        comparison->within_regression_gate = false;
        return false;
    }

    read_size = fread(json_buf, 1U, (size_t)file_size, fp);
    fclose(fp);
    json_buf[read_size] = '\0';

    if (read_size == (size_t)file_size) {
        if (lfh_extract_json_number(json_buf, "\"overallScore\"", &baseline_overall) == false ||
            lfh_extract_json_number(json_buf, "\"avgLineDetectionRate\"", &baseline_detect) == false ||
            lfh_extract_json_number(json_buf, "\"maxLongestLostSec\"", &baseline_max_lost) == false) {
            free(json_buf);
            lfh_copy_error(error_buf, error_buf_size, "baseline report missing required summary fields");
            lfh_copy_error(comparison->error_message, sizeof(comparison->error_message), "baseline report missing required summary fields");
            comparison->within_regression_gate = false;
            return false;
        }
    } else {
        free(json_buf);
        lfh_copy_error(error_buf, error_buf_size, "failed to read full baseline report");
        lfh_copy_error(comparison->error_message, sizeof(comparison->error_message), "failed to read full baseline report");
        comparison->within_regression_gate = false;
        return false;
    }

    free(json_buf);

    comparison->loaded = true;
    comparison->baseline_overall_score = baseline_overall;
    comparison->baseline_avg_line_detection_rate = baseline_detect;
    comparison->baseline_max_longest_lost_sec = baseline_max_lost;

    comparison->score_regression_pct = 0.0;
    if (baseline_overall > 1e-12 && current->overall_score < baseline_overall) {
        comparison->score_regression_pct =
            ((baseline_overall - current->overall_score) / baseline_overall) * 100.0;
    }

    comparison->detection_regression_pct = 0.0;
    if (baseline_detect > 1e-12 && current->avg_line_detection_rate < baseline_detect) {
        comparison->detection_regression_pct =
            ((baseline_detect - current->avg_line_detection_rate) / baseline_detect) * 100.0;
    }

    comparison->max_lost_regression_pct = 0.0;
    if (current->max_longest_lost_sec > baseline_max_lost) {
        if (baseline_max_lost > 1e-12) {
            comparison->max_lost_regression_pct =
                ((current->max_longest_lost_sec - baseline_max_lost) / baseline_max_lost) * 100.0;
        } else {
            comparison->max_lost_regression_pct = 100.0;
        }
    }

    comparison->within_regression_gate = true;
    if (comparison->score_regression_pct > comparison->regression_gate_pct ||
        comparison->detection_regression_pct > comparison->regression_gate_pct ||
        comparison->max_lost_regression_pct > comparison->regression_gate_pct) {
        comparison->within_regression_gate = false;
    }

    if ((error_buf == NULL) == false && error_buf_size > 0U) {
        error_buf[0] = '\0';
    }
    comparison->error_message[0] = '\0';
    return true;
}
