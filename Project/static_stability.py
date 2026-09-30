"""
Static stability of the target structures, without any robot.

lego_sim's direct Python API (LegoBenchEnv) simulates contact physics only. The
stud clutch that holds real LEGO together exists in lego_sim's ROS 2 bridge but
not here. A structure whose bricks are not held up by gravity and contact alone
therefore cannot be assembled in this environment, no matter how good the plan
or the motion is.

This script measures that upper bound. For every task it

  1. teleports all bricks exactly onto their target poses, lets the simulation
     run and scores the result with the benchmark ("stable in simulation"), and
  2. computes a geometric margin: for every joint between two bricks, the
     distance from the centre of mass of everything above the joint to the edge
     of the area the two bricks share. A negative margin means the centre of
     mass hangs over the edge.

Example:
    python static_stability.py --out ../evaluation/static_stability
"""
import argparse
import contextlib
import csv
import io
import math
import shutil
import tempfile
from pathlib import Path

import mujoco
import yaml

from AI_module import convert_workbenchmark_yaml
from mj_bridge.gym_env import LegoBenchEnv


PROJECT_DIR = Path(__file__).resolve().parent
TASKS = PROJECT_DIR / "tasks"
BRICK_SIZE = {"brick_4x2": (0.064, 0.032), "brick_2x2": (0.032, 0.032)}
BRICK_MASS = {"brick_4x2": 0.022, "brick_2x2": 0.012}   # lego_sim part registry
SETTLE_SECONDS = 3.0


def footprint(block):
    length, width = BRICK_SIZE[block["type"]]
    if int(round(block["rotation"][2])) % 180 == 90:
        length, width = width, length
    x, y = block["pos"][0], block["pos"][1]
    return x - length / 2, x + length / 2, y - width / 2, y + width / 2


def support_margin(blocks):
    """
    Smallest distance (m) from the centre of mass of the bricks above a joint to
    the edge of the shared area of that joint. Valid for single-column towers.
    """
    tower = sorted(blocks, key=lambda b: b["pos"][2])
    worst = math.inf
    for k in range(len(tower) - 1):
        lower, upper = footprint(tower[k]), footprint(tower[k + 1])
        shared = (max(lower[0], upper[0]), min(lower[1], upper[1]),
                  max(lower[2], upper[2]), min(lower[3], upper[3]))
        above = tower[k + 1:]
        mass = sum(BRICK_MASS[b["type"]] for b in above)
        cx = sum(BRICK_MASS[b["type"]] * b["pos"][0] for b in above) / mass
        cy = sum(BRICK_MASS[b["type"]] * b["pos"][1] for b in above) / mass
        worst = min(worst, cx - shared[0], shared[1] - cx, cy - shared[2], shared[3] - cy)
    return worst


def settle(yaml_path, work_dir):
    """
    Places every brick exactly on its target, simulates and returns the benchmark score.
    """
    product = work_dir / "product.yaml"
    convert_workbenchmark_yaml(yaml_path, product)
    with contextlib.redirect_stdout(io.StringIO()):
        env = LegoBenchEnv(product, seed=42, output_dir=work_dir / "run")

    model, data = env.model, env.data
    for target in env.episode_manifest["target_blocks"]:
        body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, target["body_name"])
        joint = model.body_jntadr[body]
        qpos, dof = model.jnt_qposadr[joint], model.jnt_dofadr[joint]
        data.qpos[qpos:qpos + 3] = target["position"]
        data.qpos[qpos + 3:qpos + 7] = [math.cos(target["yaw_rad"] / 2), 0, 0, math.sin(target["yaw_rad"] / 2)]
        data.qvel[dof:dof + 6] = 0
    mujoco.mj_forward(model, data)

    action = env.neutral_action
    for _ in range(int(SETTLE_SECONDS / (model.opt.timestep * env.frame_skip))):
        env.step(action)

    score = env.score()
    env.close()
    return score


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--tiers", nargs="+", default=["1", "2"])
    parser.add_argument("--first", type=int, default=100)
    parser.add_argument("--out", default="results/static_stability")
    args = parser.parse_args()

    out_dir = Path(args.out).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    rows = []
    for tier in args.tiers:
        for index in range(1, args.first + 1):
            task = f"{index:03d}"
            yaml_path = TASKS / f"Tier_{tier}" / f"task_{task}.yaml"
            with open(yaml_path, "r") as f:
                blocks = yaml.safe_load(f)["blocks"]
            work_dir = Path(tempfile.mkdtemp())
            try:
                score = settle(yaml_path, work_dir)
            finally:
                shutil.rmtree(work_dir, ignore_errors=True)
            rows.append({
                "tier": int(tier),
                "task": task,
                "n_bricks": len(blocks),
                "support_margin_mm": round(1000 * support_margin(blocks), 2),
                "stable_in_simulation": score["success"],
                "bricks_in_tolerance": score["placed"],
                "max_xy_error_mm": round(1000 * max(b["xy_error_m"] for b in score["blocks"]), 2),
                "max_z_error_mm": round(1000 * max(b["z_error_m"] for b in score["blocks"]), 2),
            })
        print(f"tier {tier} done", flush=True)

    with open(out_dir / "static_stability.csv", "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    lines = ["| Tier | tasks | stable without clutch | margin > 0 (stable / unstable) | margin = 0 (stable / unstable) | margin < 0 (stable / unstable) |",
             "|---|---|---|---|---|---|"]
    for tier in sorted({r["tier"] for r in rows}):
        group = [r for r in rows if r["tier"] == tier]
        cells = []
        for test in (lambda m: m > 0.05, lambda m: abs(m) <= 0.05, lambda m: m < -0.05):
            sub = [r for r in group if test(r["support_margin_mm"])]
            cells.append(f"{sum(r['stable_in_simulation'] for r in sub)} / {sum(not r['stable_in_simulation'] for r in sub)}")
        lines.append(f"| {tier} | {len(group)} | {sum(r['stable_in_simulation'] for r in group)}/{len(group)} | " + " | ".join(cells) + " |")
    unstable = [f"{r['tier']}/{r['task']}" for r in rows if not r["stable_in_simulation"]]
    lines += ["", "Unstable tasks (tier/task): " + (", ".join(unstable) or "none")]
    table = "\n".join(lines)
    (out_dir / "summary.md").write_text(table + "\n")
    print()
    print(table)


if __name__ == "__main__":
    main()
