"""
Planner comparison on the symbolic level (no simulation).

For every task the same PDDL problem is solved by

    bfs            our breadth-first forward search from the exercise sheets (upf.py)
    fast-downward  Fast Downward through the Unified Planning Framework
    pyperplan      pyperplan through the Unified Planning Framework
    abd            the Assembly-by-Disassembly reference planner that ships with
                   lego_sim (mj_bridge.executor_planner.plan_assembly). It works
                   on the target geometry directly and needs no PDDL.

and the script records whether a plan was found, its length, the time it took
and whether the resulting assembly order is physically valid, i.e. every brick
is placed after the bricks it rests on.

A PDDL plan has one pick and one place/stack per brick. ABD returns one
pick-and-place step per brick; its length is reported in the same unit
(two actions per step).

Every planner call runs in its own process so that a time limit can be
enforced; an unsolvable problem makes a blind search enumerate its whole
state space.

Examples:
    python compare_planners.py
    python compare_planners.py --tiers 3 4 --tasks-dir ../dataset/ground_truth --timeout 20 --repeats 1
"""
import argparse
import csv
import json
import multiprocessing
import statistics
import tempfile
import time
from pathlib import Path

import yaml

from parser import generate_pddl, block_rename


PROJECT_DIR = Path(__file__).resolve().parent
DOMAIN = PROJECT_DIR / "domain.pddl"
PLANNERS = ["bfs", "fast-downward", "pyperplan", "abd"]
LAYER_HEIGHT = 0.0191
BRICK_SIZE = {"brick_4x2": (0.064, 0.032), "brick_2x2": (0.032, 0.032)}


def task_path(tasks_dir, tier, task):
    """
    Accepts the layout of this repository (Tier_1/...) and of the dataset (tier1/...).
    """
    for name in (f"Tier_{tier}", f"tier{tier}"):
        path = Path(tasks_dir) / name / f"task_{task}.yaml"
        if path.exists():
            return path
    raise FileNotFoundError(f"task {task} of tier {tier} not found in {tasks_dir}")


def footprint(block):
    length, width = BRICK_SIZE[block["type"]]
    if int(round(block["rotation"][2])) % 180 == 90:
        length, width = width, length
    x, y = block["pos"][0], block["pos"][1]
    return x - length / 2, x + length / 2, y - width / 2, y + width / 2


def supports(blocks):
    """
    {brick: set of bricks it rests on}, from the target geometry: one layer
    lower and overlapping footprints.
    """
    result = {}
    for upper in blocks:
        a = footprint(upper)
        below = set()
        for lower in blocks:
            if abs(upper["pos"][2] - lower["pos"][2] - LAYER_HEIGHT) > 1e-4:
                continue
            b = footprint(lower)
            if min(a[1], b[1]) - max(a[0], b[0]) > 1e-6 and min(a[3], b[3]) - max(a[2], b[2]) > 1e-6:
                below.add(block_rename(lower["name"]))
        result[block_rename(upper["name"])] = below
    return result


def valid_order(order, blocks):
    """
    True if every brick is placed exactly once and after all bricks below it.
    """
    below = supports(blocks)
    if sorted(order) != sorted(below):
        return False
    position = {brick: index for index, brick in enumerate(order)}
    return all(position[lower] < position[upper] for upper in below for lower in below[upper])


def solve(planner, yaml_path, repeats):
    """
    Plans one task with one planner. Returns plain data (runs in a child process).
    """
    with open(yaml_path, "r") as f:
        blocks = yaml.safe_load(f)["blocks"]

    result = {"solved": False, "plan_length": None, "order": None, "t_read_s": None, "t_solve_s": None}
    times = []

    if planner == "abd":
        from mj_bridge.benchmark_core import normalize_product
        from mj_bridge.executor_planner import plan_assembly

        for _ in range(repeats):
            start = time.perf_counter()
            steps = plan_assembly(normalize_product({"blocks": blocks}))["steps"]
            times.append(time.perf_counter() - start)
        order = [block_rename(step["id"]) for step in steps]
        result.update(solved=True, plan_length=2 * len(steps), order=order, t_read_s=0.0)

    else:
        from unified_planning.io import PDDLReader
        from unified_planning.shortcuts import OneshotPlanner, get_environment
        from upf import strips_planner

        get_environment().credits_stream = None

        with tempfile.TemporaryDirectory() as tmp:
            pddl_path = Path(tmp) / "problem.pddl"
            generate_pddl(yaml_path, pddl_path)

            start = time.perf_counter()
            problem = PDDLReader().parse_problem(str(DOMAIN), str(pddl_path))
            result["t_read_s"] = time.perf_counter() - start

            plan = None
            for _ in range(repeats):
                start = time.perf_counter()
                if planner == "bfs":
                    plan = strips_planner(problem)
                else:
                    with OneshotPlanner(name=planner) as engine:
                        found = engine.solve(problem).plan
                    plan = None if found is None else list(found.actions)
                times.append(time.perf_counter() - start)

        if plan is not None:
            order = [str(a.actual_parameters[0]) for a in plan if a.action.name in ("place", "stack")]
            result.update(solved=True, plan_length=len(plan), order=order)

    result["t_solve_s"] = statistics.median(times)
    if result["solved"]:
        result["valid_order"] = valid_order(result["order"], blocks)
    return result


def child(connection, planner, yaml_path, repeats):
    try:
        connection.send(solve(planner, yaml_path, repeats))
    except Exception as error:
        connection.send({"solved": False, "error": repr(error)})
    connection.close()


def solve_with_timeout(planner, yaml_path, repeats, timeout):
    parent, other = multiprocessing.Pipe(duplex=False)
    process = multiprocessing.Process(target=child, args=(other, planner, yaml_path, repeats))
    process.start()
    other.close()
    if parent.poll(timeout):
        result = parent.recv()
        process.join()
    else:
        process.terminate()
        process.join()
        result = {"solved": False, "timeout": True}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--tiers", nargs="+", default=["1", "2"])
    parser.add_argument("--first", type=int, default=100)
    parser.add_argument("--tasks-dir", default=str(PROJECT_DIR / "tasks"))
    parser.add_argument("--planners", nargs="+", default=PLANNERS, choices=PLANNERS)
    parser.add_argument("--repeats", type=int, default=3, help="solve this often, report the median time")
    parser.add_argument("--timeout", type=float, default=60.0, help="seconds per planner call")
    parser.add_argument("--out", default="results/planners")
    args = parser.parse_args()

    out_dir = Path(args.out).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    rows = []
    start = time.perf_counter()
    for tier in args.tiers:
        for index in range(1, args.first + 1):
            task = f"{index:03d}"
            path = task_path(args.tasks_dir, tier, task)
            with open(path, "r") as f:
                n_bricks = len(yaml.safe_load(f)["blocks"])
            for planner in args.planners:
                result = solve_with_timeout(planner, path, args.repeats, args.timeout)
                rows.append({"tier": int(tier), "task": task, "n_bricks": n_bricks, "planner": planner, **result})
        print(f"[{time.perf_counter() - start:6.0f} s] tier {tier} done", flush=True)

    # Do the planners agree on the assembly order?
    reference = {(r["tier"], r["task"]): r.get("order") for r in rows if r["planner"] == args.planners[0]}
    for row in rows:
        row["same_order_as_first"] = row.get("order") is not None and row["order"] == reference[(row["tier"], row["task"])]

    columns = ["tier", "task", "n_bricks", "planner", "solved", "plan_length", "valid_order",
               "same_order_as_first", "t_read_s", "t_solve_s", "timeout", "error"]
    with open(out_dir / "planners.csv", "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    (out_dir / "planners.json").write_text(json.dumps(rows, indent=1))

    lines = [
        "| Tier | planner | solved | valid order | optimal length (2 per brick) | plan length mean | solve time median / mean / max [ms] | PDDL read mean [ms] |",
        "|---|---|---|---|---|---|---|---|",
    ]
    for tier in sorted({r["tier"] for r in rows}):
        for planner in args.planners:
            group = [r for r in rows if r["tier"] == tier and r["planner"] == planner]
            solved = [r for r in group if r["solved"]]
            if solved:
                solve_ms = [1000 * r["t_solve_s"] for r in solved]
                lines.append(
                    f"| {tier} | {planner} | {len(solved)}/{len(group)} "
                    f"| {sum(r['valid_order'] for r in solved)}/{len(solved)} "
                    f"| {sum(r['plan_length'] == 2 * r['n_bricks'] for r in solved)}/{len(solved)} "
                    f"| {statistics.mean(r['plan_length'] for r in solved):.2f} "
                    f"| {statistics.median(solve_ms):.2f} / {statistics.mean(solve_ms):.2f} / {max(solve_ms):.2f} "
                    f"| {1000 * statistics.mean(r['t_read_s'] for r in solved):.1f} |"
                )
            else:
                timeouts = sum(bool(r.get("timeout")) for r in group)
                lines.append(f"| {tier} | {planner} | 0/{len(group)} ({timeouts} timeouts) | - | - | - | - | - |")
    table = "\n".join(lines)
    (out_dir / "summary.md").write_text(table + "\n")
    print()
    print(table)
    print(f"\nResults written to {out_dir}")


if __name__ == "__main__":
    main()
