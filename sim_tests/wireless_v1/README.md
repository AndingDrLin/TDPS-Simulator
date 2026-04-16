# wireless_v1 Sim Tests

This suite validates asynchronous LoRa behavior in stub mode.

## Files

- `wl_async_autotest.c`: queue enqueue, ACK success, ACK timeout retry, AUX busy timeout.

## Run

```bash
bash TDPS-Simulator/scripts/run_wireless_autotest.sh
```

The script also runs the integration autotest that links line-follow + radar + LoRa modules together.
