#ifndef LF_HARNESS_EVALUATOR_H
#define LF_HARNESS_EVALUATOR_H

#include <stddef.h>

#include "lf_harness_common.h"

LFH_ScenarioResult LFH_Evaluator_RunScenario(const LFH_Scenario *scenario, const LFH_TestConfig *cfg);

void LFH_Evaluator_BuildSuiteSummary(const LFH_ScenarioResult *results,
                                     size_t result_count,
                                     LFH_SuiteSummary *summary,
                                     uint32_t *low_score_count,
                                     char runtime_ids[256],
                                     size_t *runtime_issue_count);

void LFH_Evaluator_CollectIssues(const LFH_SuiteSummary *summary,
                                 uint32_t low_score_count,
                                 const char runtime_ids[256],
                                 size_t runtime_issue_count,
                                 LFH_IssueList *issue_list);

void LFH_Evaluator_CollectFullCourseIssues(const LFH_ScenarioResult *results,
                                           size_t result_count,
                                           const LFH_SuiteSummary *summary,
                                           const char runtime_ids[256],
                                           size_t runtime_issue_count,
                                           LFH_IssueList *issue_list);

#endif
