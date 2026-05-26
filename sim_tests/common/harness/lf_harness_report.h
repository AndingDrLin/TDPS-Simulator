#ifndef LF_HARNESS_REPORT_H
#define LF_HARNESS_REPORT_H

#include <stddef.h>

#include "lf_harness_common.h"

bool LFH_Report_WriteJson(const char *path,
                          const LFH_TestConfig *cfg,
                          const LFH_ScenarioResult *results,
                          size_t result_count,
                          const LFH_SuiteSummary *summary,
                          const LFH_IssueList *issue_list,
                          const LFH_BaselineComparison *baseline_comparison,
                          const LFH_ConfidenceAssessment *confidence);

#endif
