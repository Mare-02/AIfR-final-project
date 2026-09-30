"""
Evaluation loop for the PDDLego pipeline.

Runs main.run_task() over a set of WorkBenchMark tasks and aggregates the
metrics named in the project proposal:

    planning success rate   p / n   tasks with a plan / all tasks
    execution success rate  e / p   tasks assembled within tolerance / tasks with a plan
    planning time                   reading the PDDL files, grounding and search
                                    (without the one-off start-up of the PDDL library)
    plan length                     number of actions in the plan
    tier comparison                 all of the above per tier

Every task runs in its own Python process. The pipeline keeps its state in
module-level variables, and a collapsing tower can exhaust MuJoCo's contact
memory and take the process down with it; one process per task keeps such a
crash from ending the whole evaluation and lets it be counted as a failure.

Examples:
    python evaluate.py --workers 8 --out ../evaluation/seed42     # Tier 1 and 2, all tasks, seed 42
    python evaluate.py --tiers 2 --first 20 --workers 4
    python evaluate.py --seeds 0 1 2 --first 20         # repeat with other brick layouts
    python evaluate.py --plan-only                      # planning metrics only, no simulation
    python evaluate.py --tiers 2 --set bridge.STABILITY_BIAS=0   # ablation: no stability bias
"""
import argparse
import csv
import json
import shutil
import statistics
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parent

# lego_sim copies the robot meshes (33 MB) into every run directory.
BULKY_ARTIFACTS = ("assets", "panda.xml", "hand.xml")

CSV_COLUMNS = [
    "tier", "task", "seed", "n_bricks",
    "plan_found", "plan_length", "t_parse_s", "t_startup_s", "t_plan_s",
    "executed", "plan_completed", "steps_executed", "failed_action", "failure_reason",
    "success", "placed", "total",
    "max_xy_error_mm", "max_z_error_mm", "max_yaw_error_deg",
    "sim_time_s", "t_exec_s", "outcome",
]


def run_one(tier, task, seed, out_dir, plan_only, timeout, overrides=()):
    """
    Runs one task in a separate process and returns its record.
    """
    work = out_dir / "runs" / f"tier{tier}_task{task}_seed{seed}"
    record_path = work / "record.json"
    if work.exists():
        shutil.rmtree(work)
    work.mkdir(parents=True)

    command = [
        sys.executable, str(PROJECT_DIR / "main.py"),
        "--tier", str(tier), "--task", task, "--seed", str(seed),
        "--work-dir", str(work), "--json", str(record_path),
    ]
    if plan_only:
        command.append("--plan-only")
    if overrides:
        command += ["--set", *overrides]

    crash = None
    try:
        process = subprocess.run(command, capture_output=True, text=True, timeout=timeout)
        if not record_path.exists():
            lines = process.stderr.strip().splitlines()
            crash = lines[-1] if lines else f"process ended with code {process.returncode}"
    except subprocess.TimeoutExpired:
        crash = f"timeout after {timeout} s"

    if record_path.exists():
        record = json.loads(record_path.read_text())
    else:
        record = {
            "tier": int(tier), "task": task, "seed": seed, "plan_found": None,
            "executed": False, "plan_completed": False, "success": False, "error": crash,
        }

    for name in BULKY_ARTIFACTS:
        path = work / "run" / name
        if path.is_dir():
            shutil.rmtree(path)
        elif path.exists():
            path.unlink()

    return record


def outcome(record, plan_only=False):
    """
    One label per task for the failure analysis.
    """
    if record.get("plan_found") is None:
        return "simulator crash"
    if not record["plan_found"]:
        return "no plan" if not record.get("error") else "planning error"
    if plan_only:
        return "planned"
    if record.get("error"):
        return "execution error"
    if record["success"]:
        return "success"
    if not record["plan_completed"]:
        return f"{record['failed_action'].split('(')[0]} failed"
    return "out of tolerance"


def flatten(record, plan_only=False):
    """
    CSV row of a record: per-brick errors are reduced to their maxima.
    """
    row = {column: record.get(column) for column in CSV_COLUMNS}
    blocks = [b for b in (record.get("blocks") or []) if "xy_error_m" in b]
    if blocks:
        row["max_xy_error_mm"] = round(1000 * max(b["xy_error_m"] for b in blocks), 3)
        row["max_z_error_mm"] = round(1000 * max(b["z_error_m"] for b in blocks), 3)
        row["max_yaw_error_deg"] = round(max(b["yaw_error_deg"] for b in blocks), 3)
    row["outcome"] = outcome(record, plan_only)
    return row


def mean(values):
    values = [v for v in values if v is not None]
    return statistics.mean(values) if values else float("nan")


def summarise(records, plan_only=False):
    """
    Aggregates the metrics of the proposal per tier.
    """
    summary = []
    for tier in sorted({r["tier"] for r in records}):
        rows = [r for r in records if r["tier"] == tier]
        planned = [r for r in rows if r.get("plan_found")]
        succeeded = [r for r in planned if r["success"]]
        outcomes = {}
        for r in rows:
            label = outcome(r, plan_only)
            outcomes[label] = outcomes.get(label, 0) + 1

        entry = {
            "tier": tier,
            "n": len(rows),
            "planned": len(planned),
            "planning_success_rate": len(planned) / len(rows),
            "plan_length_mean": mean(r["plan_length"] for r in planned),
            "plan_length_per_brick": mean(r["plan_length"] / r["n_bricks"] for r in planned),
            "t_plan_mean_s": mean(r["t_plan_s"] for r in planned),
            "t_plan_median_s": statistics.median([r["t_plan_s"] for r in planned]) if planned else float("nan"),
            "t_plan_max_s": max((r["t_plan_s"] for r in planned), default=float("nan")),
            "outcomes": outcomes,
        }
        if not plan_only:
            entry.update({
                "succeeded": len(succeeded),
                "execution_success_rate": len(succeeded) / len(planned) if planned else float("nan"),
                "bricks_placed_rate": sum(r.get("placed", 0) for r in rows) / sum(r.get("total") or r.get("n_bricks", 0) for r in rows),
                "sim_time_mean_s": mean(r.get("sim_time_s") for r in planned),
                "t_exec_mean_s": mean(r.get("t_exec_s") for r in planned),
            })
        summary.append(entry)
    return summary


def to_markdown(summary, plan_only=False):
    lines = []
    if plan_only:
        lines.append("| Tier | n | planning success p/n | plan length | actions per brick | planning time mean / median / max [s] |")
        lines.append("|---|---|---|---|---|---|")
        for s in summary:
            lines.append(
                f"| {s['tier']} | {s['n']} | {s['planned']}/{s['n']} = {s['planning_success_rate']:.0%} "
                f"| {s['plan_length_mean']:.2f} | {s['plan_length_per_brick']:.2f} "
                f"| {s['t_plan_mean_s']:.3f} / {s['t_plan_median_s']:.3f} / {s['t_plan_max_s']:.3f} |"
            )
    else:
        lines.append("| Tier | n | planning success p/n | execution success e/p | bricks placed | plan length | planning time mean [s] | simulated time mean [s] |")
        lines.append("|---|---|---|---|---|---|---|---|")
        for s in summary:
            lines.append(
                f"| {s['tier']} | {s['n']} | {s['planned']}/{s['n']} = {s['planning_success_rate']:.0%} "
                f"| {s['succeeded']}/{s['planned']} = {s['execution_success_rate']:.0%} "
                f"| {s['bricks_placed_rate']:.1%} | {s['plan_length_mean']:.2f} "
                f"| {s['t_plan_mean_s']:.3f} | {s['sim_time_mean_s']:.0f} |"
            )
    lines.append("")
    lines.append("Outcomes per tier:")
    for s in summary:
        parts = ", ".join(f"{label}: {count}" for label, count in sorted(s["outcomes"].items(), key=lambda kv: -kv[1]))
        lines.append(f"- Tier {s['tier']}: {parts}")
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--tiers", nargs="+", default=["1", "2"])
    parser.add_argument("--first", type=int, default=100, help="evaluate task_001 ... task_<first> of every tier")
    parser.add_argument("--tasks", nargs="+", help="explicit task IDs (e.g. 001 017), overrides --first")
    parser.add_argument("--seeds", nargs="+", type=int, default=[42], help="seeds for the initial brick layout")
    parser.add_argument("--workers", type=int, default=4, help="tasks simulated in parallel")
    parser.add_argument("--timeout", type=int, default=1200, help="seconds before a task is aborted")
    parser.add_argument("--plan-only", action="store_true", help="planning metrics only")
    parser.add_argument("--out", default="results/evaluation", help="output directory")
    parser.add_argument("--set", nargs="*", default=[], metavar="MODULE.NAME=VALUE",
                        help="override module constants for ablations, e.g. bridge.STABILITY_BIAS=0")
    args = parser.parse_args()

    out_dir = Path(args.out).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    task_ids = args.tasks or [f"{i:03d}" for i in range(1, args.first + 1)]
    jobs = [(tier, task, seed) for seed in args.seeds for tier in args.tiers for task in task_ids]

    records = []
    n_planned = n_success = 0
    start = time.perf_counter()

    with ThreadPoolExecutor(args.workers) as pool:
        futures = [pool.submit(run_one, tier, task, seed, out_dir, args.plan_only, args.timeout, args.set)
                   for tier, task, seed in jobs]
        for count, future in enumerate(as_completed(futures), start=1):
            record = future.result()
            records.append(record)
            n_planned += bool(record.get("plan_found"))
            n_success += bool(record.get("success"))
            print(
                f"[{time.perf_counter() - start:7.0f} s] {count:4d}/{len(jobs)}  "
                f"tier {record['tier']} task {record['task']} seed {record['seed']}: {outcome(record, args.plan_only):<18}"
                f"  | planned {n_planned}, successful {n_success}",
                flush=True,
            )

    records.sort(key=lambda r: (r["seed"], r["tier"], r["task"]))
    summary = summarise(records, args.plan_only)
    table = to_markdown(summary, args.plan_only)
    if args.set:
        table = "Overrides: " + ", ".join(args.set) + "\n\n" + table

    (out_dir / "results.json").write_text(json.dumps(records, indent=1))
    with open(out_dir / "results.csv", "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_COLUMNS)
        writer.writeheader()
        writer.writerows(flatten(r, args.plan_only) for r in records)
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=1))
    (out_dir / "summary.md").write_text(table + "\n")

    print()
    print(table)
    print(f"\n{len(jobs)} tasks in {time.perf_counter() - start:.0f} s. Results written to {out_dir}")


if __name__ == "__main__":
    main()
