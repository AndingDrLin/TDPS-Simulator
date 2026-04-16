# line_follow_v1 Sim Tests

This suite validates line-follow control behavior in offline simulation.

## Directory

```text
TDPS-Simulator/sim_tests/
├── common/
│   └── harness/
└── line_follow_v1/
    ├── config/
    │   └── scenarios_default.csv
  ├── lf_autotest_harness.c
  └── lf_radar_autotest.c
```

## Commands

Run from repository root:

```bash
# help
bash TDPS-Simulator/scripts/line_follow_cli.sh help

# quick
bash TDPS-Simulator/scripts/line_follow_cli.sh quick

# stability
bash TDPS-Simulator/scripts/line_follow_cli.sh stability 15 0.01 0.12 20260319 20 \
  TDPS-Simulator/artifacts/line_follow_v1/reports/stability_runs

# run-config
bash TDPS-Simulator/scripts/line_follow_cli.sh run-config \
  TDPS-Simulator/sim_tests/line_follow_v1/config/scenarios_default.csv

# radar parser
bash TDPS-Simulator/scripts/run_radar_autotest.sh

# full system regression
bash TDPS-Simulator/scripts/run_system_autotest.sh
```

## Gates

### quick

- `overallScore >= 82`
- `avgLineDetectionRate >= 0.94`
- `maxLongestLostSec <= 0.35`
- no runtime error
- no scenario `score < 70`

### stability

- `minScore >= 80`
- `minDetectPercent >= 93`
- `maxLostSec <= 0.40`

### baseline regression guard

- `scoreRegressionPct <= 5%`
- `detectionRegressionPct <= 5%`
- `maxLostRegressionPct <= 5%`

## Outputs

- quick: `TDPS-Simulator/artifacts/line_follow_v1/reports/single_run/*.json`
- stability: `TDPS-Simulator/artifacts/line_follow_v1/reports/stability_runs/report_seed_*.json`
- summary: `TDPS-Simulator/artifacts/line_follow_v1/reports/stability_runs/stability_summary.json`

Key report metrics include `maxMotorCommandDelta`, `meanMotorCommandDelta`, `maxMotorJerk`, `reacquireTimeMs`, `lostLineDurationMs`, and `obstacleRecoverySuccess`.
