#include "lf_harness_scenarios.h"

#include <ctype.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define LFH_SCENARIO_MAX 48U
#define LFH_ID_MAX 64U
#define LFH_NAME_MAX 96U
#define LFH_LINE_MAX 768U
#define LFH_BASE_FIELD_COUNT 12U
#define LFH_EXT_FIELD_COUNT 21U

static LFH_Scenario g_scenarios[LFH_SCENARIO_MAX];
static char g_ids[LFH_SCENARIO_MAX][LFH_ID_MAX];
static char g_names[LFH_SCENARIO_MAX][LFH_NAME_MAX];
static size_t g_entry_count = 0U;
static const char *k_default_config_path = "TDPS-Simulator/sim_tests/line_follow_v1/config/scenarios_default.csv";

static char *lfh_trim(char *text)
{
    char *start = text;
    char *end;

    while ((*start == '\0') == false && isspace((unsigned char)(*start)) > 0) {
        start += 1;
    }

    end = start + strlen(start);
    while (end > start) {
        const unsigned char ch = (unsigned char)end[-1];
        if (isspace(ch) == 0) {
            break;
        }
        end -= 1;
        *end = '\0';
    }

    return start;
}

static size_t lfh_split_csv(char *line, char *fields[LFH_EXT_FIELD_COUNT])
{
    size_t count = 0U;
    char *cursor = line;

    while (count < LFH_EXT_FIELD_COUNT) {
        fields[count] = cursor;
        count += 1U;

        {
            char *comma = strchr(cursor, ',');
            if (comma == NULL) {
                break;
            }
            *comma = '\0';
            cursor = comma + 1;
        }
    }

    return count;
}

static bool lfh_parse_double_range(const char *text, double min_v, double max_v, double *out)
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

static bool lfh_parse_u32(const char *text, uint32_t *out)
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

static bool lfh_parse_bool(const char *text, bool *out)
{
    if (strcmp(text, "true") == 0 || strcmp(text, "1") == 0 || strcmp(text, "yes") == 0) {
        *out = true;
        return true;
    }
    if (strcmp(text, "false") == 0 || strcmp(text, "0") == 0 || strcmp(text, "no") == 0) {
        *out = false;
        return true;
    }
    return false;
}

static bool lfh_parse_obstacle_mode(const char *text, LFH_ObstacleMode *mode)
{
    if (strcmp(text, "none") == 0) {
        *mode = LFH_OBSTACLE_NONE;
        return true;
    }
    if (strcmp(text, "fixed") == 0) {
        *mode = LFH_OBSTACLE_FIXED;
        return true;
    }
    if (strcmp(text, "random") == 0) {
        *mode = LFH_OBSTACLE_RANDOM;
        return true;
    }
    return false;
}

const char *LFH_Scenarios_TrackName(LFH_TrackType track)
{
    if (track == LFH_TRACK_CIRCLE) {
        return "circle";
    }
    if (track == LFH_TRACK_FIGURE8) {
        return "figure8";
    }
    if (track == LFH_TRACK_PATIO_REAL) {
        return "patio_real";
    }
    return "patio_proxy";
}

static bool lfh_parse_track(const char *text, LFH_TrackType *track)
{
    if (strcmp(text, "circle") == 0) {
        *track = LFH_TRACK_CIRCLE;
        return true;
    }
    if (strcmp(text, "figure8") == 0) {
        *track = LFH_TRACK_FIGURE8;
        return true;
    }
    if (strcmp(text, "patio_proxy") == 0) {
        *track = LFH_TRACK_PATIO_PROXY;
        return true;
    }
    if (strcmp(text, "patio_real") == 0) {
        *track = LFH_TRACK_PATIO_REAL;
        return true;
    }

    return false;
}

const char *LFH_Scenarios_DefaultConfigPath(void)
{
    return k_default_config_path;
}

static void lfh_set_error(char *error_buf, size_t error_buf_size, const char *message)
{
    if ((error_buf == NULL) == false && error_buf_size > 0U) {
        snprintf(error_buf, error_buf_size, "%s", message);
    }
}

bool LFH_Scenarios_LoadFromConfig(const char *path, char *error_buf, size_t error_buf_size)
{
    FILE *fp;
    char line[LFH_LINE_MAX];
    size_t line_no = 0U;
    size_t count = 0U;

    if (path == NULL || path[0] == '\0') {
        lfh_set_error(error_buf, error_buf_size, "scenario config path is empty");
        return false;
    }

    fp = fopen(path, "r");
    if (fp == NULL) {
        if ((error_buf == NULL) == false && error_buf_size > 0U) {
            snprintf(error_buf, error_buf_size, "cannot open scenario config: %s", path);
        }
        return false;
    }

    g_entry_count = 0U;

    while ((fgets(line, (int)sizeof(line), fp) == NULL) == false) {
        char *trimmed;
        char *fields[LFH_EXT_FIELD_COUNT];
        size_t field_count;
        LFH_Scenario *scenario;
        size_t i;

        line_no += 1U;
        trimmed = lfh_trim(line);

        if (trimmed[0] == '\0' || trimmed[0] == '#') {
            continue;
        }

        field_count = lfh_split_csv(trimmed, fields);
        if (field_count != LFH_BASE_FIELD_COUNT && field_count != LFH_EXT_FIELD_COUNT) {
            if ((error_buf == NULL) == false && error_buf_size > 0U) {
                snprintf(error_buf,
                         error_buf_size,
                         "line %zu: expected %u or %u fields, got %zu",
                         line_no,
                         (unsigned)LFH_BASE_FIELD_COUNT,
                         (unsigned)LFH_EXT_FIELD_COUNT,
                         field_count);
            }
            fclose(fp);
            return false;
        }

        for (i = 0U; i < field_count; ++i) {
            fields[i] = lfh_trim(fields[i]);
        }

        if (strcmp(fields[0], "id") == 0 && strcmp(fields[1], "name") == 0) {
            continue;
        }

        if (count >= LFH_SCENARIO_MAX) {
            if ((error_buf == NULL) == false && error_buf_size > 0U) {
                snprintf(error_buf, error_buf_size, "too many scenarios, max is %u", (unsigned)LFH_SCENARIO_MAX);
            }
            fclose(fp);
            return false;
        }

        if (fields[0][0] == '\0' || fields[1][0] == '\0') {
            if ((error_buf == NULL) == false && error_buf_size > 0U) {
                snprintf(error_buf, error_buf_size, "line %zu: id/name cannot be empty", line_no);
            }
            fclose(fp);
            return false;
        }

        if (strlen(fields[0]) >= LFH_ID_MAX || strlen(fields[1]) >= LFH_NAME_MAX) {
            if ((error_buf == NULL) == false && error_buf_size > 0U) {
                snprintf(error_buf, error_buf_size, "line %zu: id or name too long", line_no);
            }
            fclose(fp);
            return false;
        }

        for (i = 0U; i < count; ++i) {
            if (strcmp(fields[0], g_ids[i]) == 0) {
                if ((error_buf == NULL) == false && error_buf_size > 0U) {
                    snprintf(error_buf, error_buf_size, "line %zu: duplicate scenario id '%s'", line_no, fields[0]);
                }
                fclose(fp);
                return false;
            }
        }

        snprintf(g_ids[count], LFH_ID_MAX, "%s", fields[0]);
        snprintf(g_names[count], LFH_NAME_MAX, "%s", fields[1]);

        scenario = &g_scenarios[count];
        scenario->id = g_ids[count];
        scenario->name = g_names[count];

        if (lfh_parse_track(fields[2], &scenario->track) == false) {
            if ((error_buf == NULL) == false && error_buf_size > 0U) {
                snprintf(error_buf,
                         error_buf_size,
                         "line %zu: unsupported track '%s', allowed: circle/figure8/patio_proxy/patio_real",
                         line_no,
                         fields[2]);
            }
            fclose(fp);
            return false;
        }

        if (lfh_parse_double_range(fields[3], -10.0, 10.0, &scenario->start_pose.x) == false ||
            lfh_parse_double_range(fields[4], -10.0, 10.0, &scenario->start_pose.y) == false ||
            lfh_parse_double_range(fields[5], -6.5, 6.5, &scenario->start_pose.theta) == false) {
            if ((error_buf == NULL) == false && error_buf_size > 0U) {
                snprintf(error_buf, error_buf_size, "line %zu: invalid pose values", line_no);
            }
            fclose(fp);
            return false;
        }

        if (lfh_parse_double_range(fields[6], 0.0, 0.20, &scenario->noise_std) == false ||
            lfh_parse_double_range(fields[7], 0.0, 0.20, &scenario->dropout_prob) == false ||
            lfh_parse_double_range(fields[8], 0.50, 1.20, &scenario->contrast) == false ||
            lfh_parse_double_range(fields[9], 0.50, 1.50, &scenario->left_motor_scale) == false ||
            lfh_parse_double_range(fields[10], 0.50, 1.50, &scenario->right_motor_scale) == false) {
            if ((error_buf == NULL) == false && error_buf_size > 0U) {
                snprintf(error_buf, error_buf_size, "line %zu: invalid noise/dropout/contrast/motor scale", line_no);
            }
            fclose(fp);
            return false;
        }

        if (lfh_parse_u32(fields[11], &scenario->seed) == false || scenario->seed == 0U) {
            if ((error_buf == NULL) == false && error_buf_size > 0U) {
                snprintf(error_buf, error_buf_size, "line %zu: invalid seed", line_no);
            }
            fclose(fp);
            return false;
        }

        scenario->radar_enable = false;
        scenario->obstacle_mode = LFH_OBSTACLE_NONE;
        scenario->obstacle_count = 0U;
        scenario->obstacle_seed = scenario->seed;
        scenario->checkpoint_enable = false;
        scenario->checkpoint_expected_count = 0U;
        scenario->wireless_enable = false;
        scenario->finish_enable = false;
        scenario->boundary_enable = false;

        if (field_count == LFH_EXT_FIELD_COUNT) {
            if (lfh_parse_bool(fields[12], &scenario->radar_enable) == false ||
                lfh_parse_obstacle_mode(fields[13], &scenario->obstacle_mode) == false ||
                lfh_parse_u32(fields[14], &scenario->obstacle_count) == false ||
                lfh_parse_u32(fields[15], &scenario->obstacle_seed) == false ||
                lfh_parse_bool(fields[16], &scenario->checkpoint_enable) == false ||
                lfh_parse_u32(fields[17], &scenario->checkpoint_expected_count) == false ||
                lfh_parse_bool(fields[18], &scenario->wireless_enable) == false ||
                lfh_parse_bool(fields[19], &scenario->finish_enable) == false ||
                lfh_parse_bool(fields[20], &scenario->boundary_enable) == false) {
                if ((error_buf == NULL) == false && error_buf_size > 0U) {
                    snprintf(error_buf, error_buf_size, "line %zu: invalid extended task fields", line_no);
                }
                fclose(fp);
                return false;
            }
        }

        count += 1U;
    }

    fclose(fp);

    if (count == 0U) {
        lfh_set_error(error_buf, error_buf_size, "scenario config has no valid scenario entries");
        return false;
    }

    g_entry_count = count;
    if ((error_buf == NULL) == false && error_buf_size > 0U) {
        error_buf[0] = '\0';
    }
    return true;
}

bool LFH_Scenarios_LoadDefault(char *error_buf, size_t error_buf_size)
{
    return LFH_Scenarios_LoadFromConfig(k_default_config_path, error_buf, error_buf_size);
}

const LFH_Scenario *LFH_Scenarios_Get(size_t *scenario_count)
{
    if ((scenario_count == NULL) == false) {
        *scenario_count = g_entry_count;
    }

    if (g_entry_count == 0U) {
        return NULL;
    }

    return g_scenarios;
}
