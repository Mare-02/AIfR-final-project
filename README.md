# PDDLego

PDDL modelling for the [WorkBenchMark](https://arxiv.org/abs/2606.19358) LEGO assembly
benchmark. Final project of the lab course *AI for Robotics* (RWTH Aachen, summer 2026),
Project 1: *PDDL Modeling for WorkBenchMark*.

A task of the benchmark is a YAML file with the target pose of every brick. The pipeline
turns it into a PDDL problem, plans with a general-purpose planner and executes the plan in
MuJoCo through TAMPanda's `DomainBridge`:

```
tasks/Tier_N/task_XXX.yaml ──parser.py──▶ PDDL problem ──upf.py──▶ plan ──bridge.py──▶ lego_sim ──▶ score
                                              ▲
                                         domain.pddl
```

Scope, as proposed: tiers 1 and 2 (200 tasks, two to four bricks), simulation only, ground-truth
brick poses instead of perception.

## Result

All 200 tasks, initial layout of seed 42, scored by the benchmark (6 mm / 4 mm / 8° tolerance):

| | Tier 1 | Tier 2 |
|---|---|---|
| Planning success | 100/100 | 100/100 |
| Execution success | 100/100 | 90/100 |
| Execution success before the fixes described in the report | 100/100 | 34/100 |

With three other initial layouts the first 20 tasks of each tier give 60/60 and 56/60.

All tier 2 structures with two or three bricks are built. The ten failures are four-brick
towers in which a brick's centre of mass lies on or beyond the edge of its support. Real LEGO
holds these by the stud clutch; the simulation interface used here does not model it. The
report in [`report/`](report/report.pdf) has the details, [`evaluation/`](evaluation/) the
tables behind every number.

## Repository

| Path | Content |
|---|---|
| `Project/domain.pddl` | the PDDL domain: one type, seven predicates, `pick` / `place` / `stack` |
| `Project/parser.py` | task YAML → PDDL problem |
| `Project/upf.py` | breadth-first STRIPS planner on the Unified Planning Framework |
| `Project/bridge.py` | `DomainBridge` executors, grasp selection, placement, stability bias |
| `Project/AI_module.py` | inverse kinematics and motion primitives for the `lego_sim` Panda |
| `Project/main.py` | plan and execute one task |
| `Project/evaluate.py` | evaluation loop over many tasks, one process per task |
| `Project/compare_planners.py` | our search vs. Fast Downward, pyperplan and the ABD reference planner |
| `Project/static_stability.py` | do the target structures stand at all in this simulator? |
| `Project/tasks/` | the 200 task files of tiers 1 and 2 (copies from the WorkBenchMark dataset) |
| `evaluation/` | result tables of the runs reported in the report |
| `report/` | the report, its figures and the script that draws them |

## Setup

See [`SETUP.md`](SETUP.md). In short: Python ≥ 3.10, TAMPanda and
[`lego_sim`](https://github.com/ma-haha-hehe/lego_sim) checked out next to this repository.

## Usage

All commands are run from `Project/` with the interpreter of the virtual environment.

One task, planned and executed (about 35 s for two bricks):

```bash
../.venv/bin/python main.py --tier 2 --task 001
```

The evaluation of the report. All 200 tasks take about 35 minutes with eight workers:

```bash
../.venv/bin/python evaluate.py --workers 8 --out ../evaluation/seed42
```

Planning only, without simulation:

```bash
../.venv/bin/python evaluate.py --plan-only
```

Other initial layouts, and an ablation that switches one mechanism off (`--set` overrides a
constant of `bridge.py` or `AI_module.py` for that run):

```bash
../.venv/bin/python evaluate.py --first 20 --seeds 0 1 2 --out ../evaluation/seeds
```

```bash
../.venv/bin/python evaluate.py --tiers 2 --first 20 --seeds 0 1 2 42 --set bridge.STABILITY_BIAS=0 --out ../evaluation/ablation_no_bias
```

Planner comparison and static stability:

```bash
../.venv/bin/python compare_planners.py --out ../evaluation/planners
```

```bash
../.venv/bin/python static_stability.py --out ../evaluation/static_stability
```

The report is built from `report/report.md` with pandoc and Chrome, no LaTeX needed:

```bash
./report/build.sh
```

## Simulation environment

The simulation environment of WorkBenchMark was not available during the project. We use
`lego_sim` by Wenbo Ma through its direct Python interface (`mj_bridge.gym_env.LegoBenchEnv`).
That interface simulates contact physics only: the stud clutch that keeps real LEGO together is
implemented in the ROS 2 bridge of `lego_sim`, not here. A released brick is held by gravity and
contact alone, which makes 8 of the 100 tier 2 target structures collapse even when every brick
is placed exactly (`static_stability.py`).
