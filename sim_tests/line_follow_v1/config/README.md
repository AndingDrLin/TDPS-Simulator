# line_follow_v1 场景配置

## scenarios_default.csv

默认 quick/stability 回归场景，保留旧巡线门禁用途。包含：

- `circle` / `figure8`
- `patio_proxy`
- `patio_real` 的短时巡线覆盖

默认 quick 仍以巡线评分为主，不要求 15 秒内跑完整 patio 任务。

## scenarios_task.csv

任务语义专项场景，用于单独验证真实 patio 相关模块：

- checkpoint 位置触发
- wireless/LoRa stub 发送链路
- finish 区域判定
- 固定/随机障碍物
- 雷达 mock 与避障状态机

示例：

```bash
bash TDPS-Simulator/scripts/line_follow_cli.sh quick 20 0.01 0.12 \
  TDPS-Simulator/artifacts/line_follow_v1/reports/single_run/task_semantic.json \
  20260319 \
  TDPS-Simulator/sim_tests/line_follow_v1/config/scenarios_task.csv
```

`scenarios_task.csv` 中部分场景是局部切片，不代表整条赛道门禁；主要看 JSON 报告中的 `taskScore`、`task`、`obstacles`、`checkpoints`、`radar`、`wireless` 字段。
