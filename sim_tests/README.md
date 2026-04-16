# Simulation Test Layout

`sim_tests/` hosts offline simulation test code and scenario suites.

## Structure

```text
sim_tests/
├── common/
│   └── harness/               # reusable test harness code
├── line_follow_v1/
│   ├── config/                # scenario CSVs
│   ├── lf_autotest_harness.c  # suite entrypoint
│   ├── lf_radar_autotest.c    # radar parser unit test
│   └── README.md
├── wireless_v1/
│   ├── wl_async_autotest.c    # LoRa async queue/ACK/retry tests
│   └── README.md
└── integration/
    ├── lf_radar_lora_integration_autotest.c
    └── README.md
```

## Extension Strategy

When adding new algorithm iterations:

1. Create a new suite folder next to `line_follow_v1/`.
2. Reuse `common/harness/` unless core simulation behavior changes.
3. Keep all outputs in `../artifacts/<suite_name>/`.

## Common Entrypoints

- `bash TDPS-Simulator/scripts/line_follow_cli.sh quick`
- `bash TDPS-Simulator/scripts/run_radar_autotest.sh`
- `bash TDPS-Simulator/scripts/run_wireless_autotest.sh`
- `bash TDPS-Simulator/scripts/run_system_autotest.sh`
