# Evaluation results

Tables behind the numbers of the report. Every folder holds `results.csv` (one row per run),
`summary.md` and, where the run was made with `evaluate.py`, the full records in
`results.json`. The per-run artifacts of lego_sim (`runs/`) are not versioned.

All runs: lego_sim commit `326d572`, MuJoCo 3.11.0, Python 3.12, Apple silicon, 10 cores.
The simulation is deterministic; the same code and seed reproduce every number.

| Folder | Content | Command (from `Project/`) |
|---|---|---|
| `seed42/` | main result: all 200 tasks of tiers 1 and 2, seed 42 | `evaluate.py --workers 8 --out ../evaluation/seed42` |
| `seeds/` | first 20 tasks per tier with seeds 0, 1, 2 | `evaluate.py --first 20 --seeds 0 1 2 --out ../evaluation/seeds` |
| `planning_only/` | planning metrics without simulation, one worker | `evaluate.py --plan-only --workers 1 --out ../evaluation/planning_only` |
| `planners/` | our search vs. Fast Downward, pyperplan and ABD | `compare_planners.py --out ../evaluation/planners` |
| `planners_tier3_4/` | the same PDDL model on the first 20 tasks of tiers 3 and 4 | `compare_planners.py --tiers 3 4 --first 20 --tasks-dir <dataset>/ground_truth --planners bfs fast-downward pyperplan --timeout 15 --repeats 1` |
| `static_stability/` | do the targets stand when placed exactly, without a robot? | `static_stability.py --out ../evaluation/static_stability` |
| `ablation_full/` | reference for the ablations: first 20 tier 2 tasks, seeds 0, 1, 2, 42 | `evaluate.py --tiers 2 --first 20 --seeds 0 1 2 42` |
| `ablation_press_1mm/` | as above, every stacked brick pressed by 1 mm | `... --set bridge.PRESS_DEPTH=0.001` |
| `ablation_no_second_correction/` | as above, without the pose correction 8 mm above the seat | `... --set AI_module.FINE_CORRECTION_HEIGHT=0` |
| `ablation_no_bias/` | as above, without the stability bias | `... --set bridge.STABILITY_BIAS=0` |
| `baseline_8dc8f92/` | the pipeline before the execution fixes (commit `8dc8f92`), seed 42 | measured with the same one-process-per-task procedure |

Planning times in `seed42/` and `seeds/` were measured while up to ten simulations ran in
parallel; the planning times of the report come from `planning_only/` and `planners/`.
