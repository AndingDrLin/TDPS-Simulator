#ifndef LF_HARNESS_SCENARIOS_H
#define LF_HARNESS_SCENARIOS_H

#include <stdbool.h>
#include <stddef.h>

#include "lf_harness_common.h"

const char *LFH_Scenarios_TrackName(LFH_TrackType track);
const char *LFH_Scenarios_DefaultConfigPath(void);

bool LFH_Scenarios_LoadDefault(char *error_buf, size_t error_buf_size);
bool LFH_Scenarios_LoadFromConfig(const char *path, char *error_buf, size_t error_buf_size);
const LFH_Scenario *LFH_Scenarios_Get(size_t *scenario_count);

#endif
