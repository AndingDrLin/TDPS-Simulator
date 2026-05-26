# Integration Sim Tests

This suite validates cross-module behavior with stubbed hardware.

## Files

- `lf_radar_lora_integration_autotest.c`: line-follow state machine + radar obstacle frames + LoRa checkpoint TX path.

## Run

```bash
bash TDPS-Simulator/scripts/run_wireless_autotest.sh
```

For full regression including quick/radar/wireless, use:

```bash
bash TDPS-Simulator/scripts/run_system_autotest.sh
```
