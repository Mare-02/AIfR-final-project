import argparse
import ast
import importlib
import json
import time
import traceback
import yaml
from pathlib import Path 
from unified_planning.io import PDDLReader
from parser import generate_pddl
from upf import generate_plan


PROJECT_DIR = Path(__file__).resolve().parent
DOMAIN = PROJECT_DIR / "domain.pddl"
TASKS = PROJECT_DIR / "tasks"


def run_task(tier_id, task_id, work_dir="tmp", seed=42, execute=True):
    """
    Runs the pipeline for one task and returns a record with all measurements.

    The record is what evaluate.py collects: planning success, planning time,
    plan length, execution success and the per-brick pose errors reported by
    the lego_sim benchmark.
    """
    yaml_path = TASKS / f"Tier_{tier_id}" / f"task_{task_id}.yaml"

    work_dir = Path(work_dir)
    work_dir.mkdir(parents=True, exist_ok=True)

    with open(yaml_path, "r") as f:
        n_bricks = len(yaml.safe_load(f)["blocks"])

    record = {
        "tier": int(tier_id),
        "task": task_id,
        "seed": seed,
        "n_bricks": n_bricks,
        "plan_found": False,
        "plan_length": None,
        "plan": None,
        "t_parse_s": None,
        "t_startup_s": None,
        "t_plan_s": None,
        "executed": False,
        "plan_completed": False,
        "steps_executed": 0,
        "failed_action": None,
        "failure_reason": None,
        "success": False,
        "placed": 0,
        "total": n_bricks,
        "sim_time_s": None,
        "t_exec_s": None,
        "blocks": None,
        "error": None,
    }

    try:
        """
        Step 0.
        Create the PDDL Domain.
        I did that manually, can be found in "domain.pddl".
        """
        
        # :)
        
        """
        Step 1.
        Generate the PDDL file from the corresponding YAML file.
        For this I created the parser.py module with all necessary functions.
        """
        pddl_path = work_dir / f"task_{task_id}.pddl"

        start = time.perf_counter()
        generate_pddl(yaml_path, pddl_path)
        record["t_parse_s"] = time.perf_counter() - start

        """    
        Step 2.
        Use the Universal Planning Framework to create an ordered plan.
        For this I created the upf.py module with all necessary functions.
        I have recycled some of the helper functions from the exercises we solved during the semester.

        The planning time covers reading the PDDL files, grounding and search.
        Writing the PDDL problem from the YAML file (Step 1) is excluded, and so
        is the start-up of the library: the first PDDLReader of a process builds
        its grammar, which takes about half a second and is independent of the
        task. It is measured separately as t_startup_s.
        """
        start = time.perf_counter()
        PDDLReader()
        record["t_startup_s"] = time.perf_counter() - start

        start = time.perf_counter()
        plan = generate_plan(DOMAIN, pddl_path)
        record["t_plan_s"] = time.perf_counter() - start

        if plan is None:
            return record

        record["plan_found"] = True
        record["plan_length"] = len(plan)
        record["plan"] = [
            f"{a.action.name}({', '.join(str(p) for p in a.actual_parameters)})"
            for a in plan
        ]

        if not execute:
            return record

        """
        Step 3. 
        Given: The PDDL domain, YAML (ground truth, for simplification), generated PDDL file and UPF plan.
        Use TamPanda/DomainBridge, the Lego_Sim Environment and WorkBenchMark to execute symbolic plan.
        """
        # Imported here so that planning-only runs do not need MuJoCo.
        from bridge import execute_plan

        start = time.perf_counter()
        res = execute_plan(plan, yaml_path, output_dir=work_dir / "run", seed=seed)
        record["t_exec_s"] = time.perf_counter() - start

        """
        Step 4.
        Evaluate the executed plan.
        We of course do that here for the actual evaluation of our approach.
        """
        record["executed"] = True
        record["sim_time_s"] = res["simulation_time_s"]
        for key in ("plan_completed", "steps_executed", "failed_action", "failure_reason",
                    "success", "placed", "total", "blocks"):
            record[key] = res[key]

    except Exception:
        record["error"] = traceback.format_exc()

    return record


def apply_overrides(overrides):
    """
    Overrides module constants, e.g. "bridge.STABILITY_BIAS=0". Used for the
    ablation runs of the evaluation, so that they need no code changes.
    """
    for item in overrides:
        target, value = item.split("=", 1)
        module_name, name = target.rsplit(".", 1)
        module = importlib.import_module(module_name)
        if not hasattr(module, name):
            raise SystemExit(f"unknown constant: {target}")
        setattr(module, name, ast.literal_eval(value))


def main():
    parser = argparse.ArgumentParser(description="Plan and execute one WorkBenchMark task.")
    parser.add_argument("--tier", help="Tier-ID, e.g. 2 for Tier 2 tasks")
    parser.add_argument("--task", help="Task-ID, e.g. 001 for task_001")
    parser.add_argument("--seed", type=int, default=42, help="seed for the initial brick layout")
    parser.add_argument("--work-dir", default="tmp", help="directory for the PDDL problem and the run artifacts")
    parser.add_argument("--plan-only", action="store_true", help="stop after planning")
    parser.add_argument("--json", help="write the result record to this file")
    parser.add_argument("--set", nargs="*", default=[], metavar="MODULE.NAME=VALUE",
                        help="override module constants for ablations, e.g. bridge.STABILITY_BIAS=0")
    args = parser.parse_args()

    apply_overrides(args.set)

    task_id = args.task or input("Please enter the Task-ID (e.g., 001 for task_001): ")
    tier_id = args.tier or input("Please enter the Tier-ID: (e.g., 2 for Tier 2 tasks): ")

    record = run_task(tier_id, task_id, args.work_dir, args.seed, execute=not args.plan_only)

    if args.json:
        Path(args.json).write_text(json.dumps(record))

    if record["error"]:
        print(record["error"])

    print(f"Plan found:      {record['plan_found']}")
    if record["plan_found"]:
        print(f"Plan length:     {record['plan_length']}  ({' -> '.join(record['plan'])})")
        print(f"Planning time:   {record['t_plan_s']:.3f} s")
    if record["executed"]:
        print(f"Plan completed:  {record['plan_completed']}" + ("" if record["plan_completed"] else f"  (failed at {record['failed_action']}: {record['failure_reason']})"))
        print(f"Success:         {record['success']}  ({record['placed']}/{record['total']} bricks within tolerance)")
        for block in record["blocks"]:
            print("   ", block)

if __name__ == "__main__":
    main()
