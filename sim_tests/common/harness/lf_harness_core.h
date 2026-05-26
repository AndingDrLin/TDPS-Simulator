#ifndef LF_HARNESS_CORE_H
#define LF_HARNESS_CORE_H

#include <stdint.h>

#include "lf_harness_common.h"

void LFH_Core_Reset(const LFH_Scenario *scenario, const LFH_TestConfig *cfg);
void LFH_Core_ReadSensorNormAndRaw(double out_norm[LF_SENSOR_COUNT], uint16_t out_raw[LF_SENSOR_COUNT]);
void LFH_Core_ClearSensorSampleCache(void);
void LFH_Core_GetLastSensorNormAndRaw(double out_norm[LF_SENSOR_COUNT], uint16_t out_raw[LF_SENSOR_COUNT]);
LFH_LineState LFH_Core_EstimateLineState(const double sensor_norm[LF_SENSOR_COUNT], double threshold);

void LFH_Core_AdvanceTime(double dt_sec);
double LFH_Core_UpdatePoseFromCommand(double dt_sec);
int16_t LFH_Core_GetLeftCommand(void);
int16_t LFH_Core_GetRightCommand(void);

bool LFH_Core_IsInFinishZone(void);
bool LFH_Core_IsInRawFinishZone(void);
bool LFH_Core_GetCheckpointId(uint32_t *checkpoint_id);
bool LFH_Core_IsBoundaryViolated(void);
bool LFH_Core_IsColliding(void);
double LFH_Core_GetProgressM(void);
double LFH_Core_GetMaxProgressM(void);
double LFH_Core_GetCourseLengthM(void);
double LFH_Core_GetProgressPercent(void);
double LFH_Core_GetLateralErrorM(void);
uint32_t LFH_Core_GetObstacleCount(void);
uint32_t LFH_Core_GetRadarDetectionCount(void);

#endif
