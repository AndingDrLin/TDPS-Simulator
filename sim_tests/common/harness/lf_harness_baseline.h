#ifndef LF_HARNESS_BASELINE_H
#define LF_HARNESS_BASELINE_H

#include <stddef.h>

#include "lf_harness_common.h"

void LFH_Baseline_Init(LFH_BaselineComparison *comparison);
bool LFH_Baseline_CompareReport(const char *baseline_report_path,
                                const LFH_SuiteSummary *current,
                                LFH_BaselineComparison *comparison,
                                char *error_buf,
                                size_t error_buf_size);

#endif
