#!/usr/bin/env python3
import argparse
import csv
import itertools
import json
import os
import subprocess
import sys
from pathlib import Path

PARAM_NAMES = [
    "TDPS_SIM_BASE_SPEED",
    "TDPS_SIM_KP",
    "TDPS_SIM_KD",
    "TDPS_SIM_MAX_CORRECTION",
]

COARSE_GRID = {
    "TDPS_SIM_BASE_SPEED": [280, 320, 360, 400, 440],
    "TDPS_SIM_KP": [0.32, 0.38, 0.42, 0.48, 0.54],
    "TDPS_SIM_KD": [1.20, 1.50, 1.65, 1.90, 2.20],
    "TDPS_SIM_MAX_CORRECTION": [260, 300, 340, 380],
}

SUMMARY_FIELDS = [
    "rank",
    "paramSetId",
    "mode",
    "exitCode",
    "runs",
    "profiles",
    "fullCoursePassRate",
    "fullCoursePassed",
    "fullCourseTotal",
    "minProgressPercent",
    "avgFinishTimeSec",
    "maxFinishTimeSec",
    "avgLineDetectionRate",
    "maxLongestLostSec",
    "avgMotorSaturationRate",
    "avgTaskScore",
    "checkpointMissed",
    "checkpointOutOfOrder",
    "checkpointDuplicates",
    "boundaryViolations",
    "boundaryViolationSteps",
    "wirelessCheckpointEnqueued",
    "wirelessCheckpointEnqueueFail",
    "wirelessCheckpointThrottled",
    "wirelessTxFail",
    "wirelessQueueDropped",
    "collisions",
    "runtimeErrors",
    *PARAM_NAMES,
]


def repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run TDPS line-follow full-course parameter sweeps.")
    parser.add_argument("--mode", choices=["coarse", "refine", "validate"], required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--duration", type=float, default=90.0)
    parser.add_argument("--dt", type=float, default=0.01)
    parser.add_argument("--threshold", type=float, default=0.12)
    parser.add_argument("--seed", type=int, default=20260319)
    parser.add_argument("--seed-start", type=int, default=None)
    parser.add_argument("--runs", type=int, default=1)
    parser.add_argument("--profiles", default="normal")
    parser.add_argument("--out", required=True)
    parser.add_argument("--input", default=None)
    parser.add_argument("--top", type=int, default=20)
    parser.add_argument("--params", default=None)
    parser.add_argument("--max-combinations", type=int, default=0)
    return parser.parse_args()


def load_json(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as fp:
        return json.load(fp)


def write_json(path: Path, data: dict) -> None:
    with path.open("w", encoding="utf-8") as fp:
        json.dump(data, fp, indent=2, sort_keys=True)
        fp.write("\n")


def fmt_value(value) -> str:
    if isinstance(value, float):
        return f"{value:.6g}"
    return str(value)


def normalize_params(row: dict) -> dict:
    params = {}
    for name in PARAM_NAMES:
        value = row[name]
        if name in ("TDPS_SIM_BASE_SPEED", "TDPS_SIM_MAX_CORRECTION"):
            params[name] = int(float(value))
        else:
            params[name] = float(value)
    return params


def coarse_params() -> list[dict]:
    keys = list(COARSE_GRID.keys())
    values = [COARSE_GRID[key] for key in keys]
    return [dict(zip(keys, combo)) for combo in itertools.product(*values)]


def read_summary_rows(path: Path) -> list[dict]:
    with path.open("r", encoding="utf-8", newline="") as fp:
        return list(csv.DictReader(fp))


def top_params_from_summary(path: Path, top: int) -> list[dict]:
    rows = read_summary_rows(path)
    rows.sort(key=rank_key)
    return [normalize_params(row) for row in rows[:top]]


def refine_params(input_path: Path, top: int) -> list[dict]:
    bases = top_params_from_summary(input_path, top)
    seen = set()
    refined = []
    for base in bases:
        candidates = {
            "TDPS_SIM_BASE_SPEED": sorted({int(base["TDPS_SIM_BASE_SPEED"]) + delta for delta in (-20, 0, 20)}),
            "TDPS_SIM_KP": sorted({round(float(base["TDPS_SIM_KP"]) + delta, 4) for delta in (-0.04, 0.0, 0.04)}),
            "TDPS_SIM_KD": sorted({round(float(base["TDPS_SIM_KD"]) + delta, 4) for delta in (-0.20, 0.0, 0.20)}),
            "TDPS_SIM_MAX_CORRECTION": sorted({int(base["TDPS_SIM_MAX_CORRECTION"]) + delta for delta in (-30, 0, 30)}),
        }
        for params in itertools.product(*(candidates[name] for name in PARAM_NAMES)):
            item = dict(zip(PARAM_NAMES, params))
            if item["TDPS_SIM_BASE_SPEED"] <= 0 or item["TDPS_SIM_MAX_CORRECTION"] <= 0:
                continue
            key = tuple(item[name] for name in PARAM_NAMES)
            if key in seen:
                continue
            seen.add(key)
            refined.append(item)
    return refined


def params_from_file(path: Path) -> dict:
    data = load_json(path)
    if "params" in data and isinstance(data["params"], dict):
        data = data["params"]
    return {name: data[name] for name in PARAM_NAMES}


def build_param_sets(args: argparse.Namespace) -> list[dict]:
    if args.params:
        return [params_from_file(Path(args.params))]
    if args.mode == "coarse":
        sets = coarse_params()
    elif args.mode == "refine":
        if not args.input:
            raise SystemExit("--input is required for refine mode")
        sets = refine_params(Path(args.input), args.top)
    else:
        if args.input:
            sets = top_params_from_summary(Path(args.input), args.top)
        else:
            if not args.params:
                raise SystemExit("validate mode requires --input or --params")
            sets = [params_from_file(Path(args.params))]
    if args.max_combinations > 0:
        sets = sets[: args.max_combinations]
    return sets


def scenario_metrics(report: dict) -> dict:
    scenarios = report.get("scenarios", [])
    full = [s for s in scenarios if s.get("course", {}).get("enabled")]
    source = full if full else scenarios
    passed = sum(1 for s in full if s.get("course", {}).get("fullCoursePassed"))
    finish_times = [s.get("task", {}).get("finishTimeSec", 0.0) for s in full if s.get("task", {}).get("finishTimeSec", 0.0) > 0.0]
    progress_values = [s.get("course", {}).get("progressPercent", 0.0) for s in full]
    return {
        "fullCoursePassed": passed,
        "fullCourseTotal": len(full),
        "minProgressPercent": min(progress_values) if progress_values else 0.0,
        "avgFinishTimeSec": sum(finish_times) / len(finish_times) if finish_times else 0.0,
        "maxFinishTimeSec": max(finish_times) if finish_times else 0.0,
        "avgLineDetectionRate": avg([s.get("lineDetectionRate", 0.0) for s in source]),
        "maxLongestLostSec": max([s.get("longestLostSec", 0.0) for s in source], default=0.0),
        "avgMotorSaturationRate": avg([s.get("motorSaturationRate", 0.0) for s in source]),
        "avgTaskScore": avg([s.get("taskScore", 0.0) for s in source]),
        "checkpointMissed": sum(int(s.get("checkpoints", {}).get("missed", 0)) for s in source),
        "checkpointOutOfOrder": sum(int(s.get("checkpoints", {}).get("outOfOrder", 0)) for s in source),
        "checkpointDuplicates": sum(int(s.get("checkpoints", {}).get("duplicates", 0)) for s in source),
        "boundaryViolations": sum(int(s.get("task", {}).get("boundaryViolationCount", 0)) for s in source),
        "boundaryViolationSteps": sum(int(s.get("task", {}).get("boundaryViolationSteps", 0)) for s in source),
        "wirelessCheckpointEnqueued": sum(int(s.get("wireless", {}).get("checkpointEnqueued", 0)) for s in source),
        "wirelessCheckpointEnqueueFail": sum(int(s.get("wireless", {}).get("checkpointEnqueueFail", 0)) for s in source),
        "wirelessCheckpointThrottled": sum(int(s.get("wireless", {}).get("checkpointThrottled", 0)) for s in source),
        "wirelessTxFail": sum(int(s.get("wireless", {}).get("txFail", 0)) for s in source),
        "wirelessQueueDropped": sum(int(s.get("wireless", {}).get("queueDropped", 0)) for s in source),
        "collisions": sum(1 for s in source if s.get("task", {}).get("collided")),
        "runtimeErrors": sum(1 for s in source if s.get("runtimeError") is not None),
    }


def avg(values: list[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def aggregate_reports(reports: list[tuple[int, str, int, Path]]) -> dict:
    totals = {
        "runs": len(reports),
        "profiles": ",".join(sorted({profile for _, profile, _, _ in reports})),
        "exitCode": 0,
        "fullCoursePassed": 0,
        "fullCourseTotal": 0,
        "minProgressPercent": 100.0,
        "avgFinishTimeSec": 0.0,
        "maxFinishTimeSec": 0.0,
        "avgLineDetectionRate": 0.0,
        "maxLongestLostSec": 0.0,
        "avgMotorSaturationRate": 0.0,
        "avgTaskScore": 0.0,
        "checkpointMissed": 0,
        "checkpointOutOfOrder": 0,
        "checkpointDuplicates": 0,
        "boundaryViolations": 0,
        "boundaryViolationSteps": 0,
        "wirelessCheckpointEnqueued": 0,
        "wirelessCheckpointEnqueueFail": 0,
        "wirelessCheckpointThrottled": 0,
        "wirelessTxFail": 0,
        "wirelessQueueDropped": 0,
        "collisions": 0,
        "runtimeErrors": 0,
    }
    finish_avgs = []
    detection_avgs = []
    saturation_avgs = []
    task_avgs = []

    if not reports:
        totals["minProgressPercent"] = 0.0
        return totals

    for exit_code, _, _, report_path in reports:
        totals["exitCode"] = max(totals["exitCode"], exit_code)
        if not report_path.exists():
            totals["runtimeErrors"] += 1
            totals["minProgressPercent"] = 0.0
            continue
        metrics = scenario_metrics(load_json(report_path))
        totals["fullCoursePassed"] += metrics["fullCoursePassed"]
        totals["fullCourseTotal"] += metrics["fullCourseTotal"]
        totals["minProgressPercent"] = min(totals["minProgressPercent"], metrics["minProgressPercent"])
        totals["maxFinishTimeSec"] = max(totals["maxFinishTimeSec"], metrics["maxFinishTimeSec"])
        totals["maxLongestLostSec"] = max(totals["maxLongestLostSec"], metrics["maxLongestLostSec"])
        totals["checkpointMissed"] += metrics["checkpointMissed"]
        totals["checkpointOutOfOrder"] += metrics["checkpointOutOfOrder"]
        totals["checkpointDuplicates"] += metrics["checkpointDuplicates"]
        totals["boundaryViolations"] += metrics["boundaryViolations"]
        totals["boundaryViolationSteps"] += metrics["boundaryViolationSteps"]
        totals["wirelessCheckpointEnqueued"] += metrics["wirelessCheckpointEnqueued"]
        totals["wirelessCheckpointEnqueueFail"] += metrics["wirelessCheckpointEnqueueFail"]
        totals["wirelessCheckpointThrottled"] += metrics["wirelessCheckpointThrottled"]
        totals["wirelessTxFail"] += metrics["wirelessTxFail"]
        totals["wirelessQueueDropped"] += metrics["wirelessQueueDropped"]
        totals["collisions"] += metrics["collisions"]
        totals["runtimeErrors"] += metrics["runtimeErrors"]
        if metrics["avgFinishTimeSec"] > 0.0:
            finish_avgs.append(metrics["avgFinishTimeSec"])
        detection_avgs.append(metrics["avgLineDetectionRate"])
        saturation_avgs.append(metrics["avgMotorSaturationRate"])
        task_avgs.append(metrics["avgTaskScore"])

    totals["avgFinishTimeSec"] = avg(finish_avgs)
    totals["avgLineDetectionRate"] = avg(detection_avgs)
    totals["avgMotorSaturationRate"] = avg(saturation_avgs)
    totals["avgTaskScore"] = avg(task_avgs)
    totals["fullCoursePassRate"] = (
        totals["fullCoursePassed"] / totals["fullCourseTotal"] if totals["fullCourseTotal"] else 0.0
    )
    if totals["minProgressPercent"] == 100.0 and totals["fullCourseTotal"] == 0:
        totals["minProgressPercent"] = 0.0
    return totals


def rank_key(row: dict):
    return (
        -float(row.get("fullCoursePassRate", 0.0)),
        int(float(row.get("runtimeErrors", 0))),
        int(float(row.get("collisions", 0))),
        int(float(row.get("boundaryViolations", 0))),
        int(float(row.get("boundaryViolationSteps", 0))),
        int(float(row.get("wirelessCheckpointEnqueueFail", 0))) +
        int(float(row.get("wirelessCheckpointThrottled", 0))) +
        int(float(row.get("wirelessTxFail", 0))) +
        int(float(row.get("wirelessQueueDropped", 0))),
        int(float(row.get("checkpointMissed", 0))) +
        int(float(row.get("checkpointOutOfOrder", 0))) +
        int(float(row.get("checkpointDuplicates", 0))),
        -float(row.get("minProgressPercent", 0.0)),
        float(row.get("maxLongestLostSec", 999.0)),
        -float(row.get("avgLineDetectionRate", 0.0)),
        float(row.get("avgFinishTimeSec", 999.0)) if float(row.get("avgFinishTimeSec", 0.0)) > 0 else 999.0,
        float(row.get("avgMotorSaturationRate", 999.0)),
    )


def run_one(root: Path, args: argparse.Namespace, params: dict, set_id: str, out_dir: Path) -> dict:
    profiles = [p.strip() for p in args.profiles.split(",") if p.strip()]
    seed_start = args.seed_start if args.seed_start is not None else args.seed
    run_count = args.runs if args.mode == "validate" or args.runs > 1 else 1
    report_dir = out_dir / "reports" / set_id
    report_dir.mkdir(parents=True, exist_ok=True)
    reports = []
    env = os.environ.copy()
    env.update({name: fmt_value(params[name]) for name in PARAM_NAMES})

    build = subprocess.run(
        ["bash", "TDPS-Simulator/scripts/build_line_follow_runner.sh"],
        cwd=root,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    (report_dir / "build.log").write_text(build.stdout, encoding="utf-8")
    runner = root / "TDPS-Simulator/artifacts/line_follow_v1/bin/lf_autotest_runner"
    if build.returncode != 0 or not runner.exists():
        row = aggregate_reports([])
        row["exitCode"] = build.returncode or 2
        row["runtimeErrors"] = 1
        row["paramSetId"] = set_id
        row["mode"] = args.mode
        row.update(params)
        return row

    for profile in profiles:
        for run_index in range(run_count):
            seed = seed_start + run_index
            report_path = report_dir / f"report_{profile}_seed_{seed}.json"
            cmd = [
                str(runner),
                "full-course",
                args.config,
                fmt_value(args.duration),
                fmt_value(args.dt),
                fmt_value(args.threshold),
                str(report_path),
                str(seed),
                profile,
            ]
            result = subprocess.run(cmd, cwd=root, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            (report_dir / f"report_{profile}_seed_{seed}.log").write_text(result.stdout, encoding="utf-8")
            reports.append((result.returncode, profile, seed, report_path))

    row = aggregate_reports(reports)
    row["paramSetId"] = set_id
    row["mode"] = args.mode
    row.update(params)
    return row


def write_summary(path: Path, rows: list[dict]) -> None:
    with path.open("w", encoding="utf-8", newline="") as fp:
        writer = csv.DictWriter(fp, fieldnames=SUMMARY_FIELDS)
        writer.writeheader()
        for rank, row in enumerate(rows, 1):
            out = {field: row.get(field, "") for field in SUMMARY_FIELDS}
            out["rank"] = rank
            writer.writerow(out)


def write_best(out_dir: Path, best: dict) -> None:
    params = {name: best[name] for name in PARAM_NAMES}
    write_json(out_dir / "best_params.json", {"params": params, "metrics": {k: best[k] for k in best if k not in PARAM_NAMES}})
    with (out_dir / "best_params.env").open("w", encoding="utf-8") as fp:
        for name in PARAM_NAMES:
            fp.write(f"export {name}={fmt_value(params[name])}\n")


def main() -> int:
    args = parse_args()
    root = repo_root()
    out_dir = root / args.out
    out_dir.mkdir(parents=True, exist_ok=True)

    param_sets = build_param_sets(args)
    if not param_sets:
        print("No parameter sets to run", file=sys.stderr)
        return 2

    rows = []
    for index, params in enumerate(param_sets, 1):
        set_id = f"set_{index:04d}"
        print(f"[{index}/{len(param_sets)}] {set_id} {params}", flush=True)
        rows.append(run_one(root, args, params, set_id, out_dir))

    rows.sort(key=rank_key)
    write_summary(out_dir / "sweep_summary.csv", rows)
    write_best(out_dir, rows[0])
    print(f"summary: {out_dir / 'sweep_summary.csv'}")
    print(f"best: {out_dir / 'best_params.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
