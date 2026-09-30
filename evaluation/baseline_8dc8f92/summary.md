Baseline: the pipeline as of commit 8dc8f92, before the execution fixes. Seed 42, all 200 tasks,
one process per task. A run that ended in a MuJoCo error (contact memory exhausted while the tower
was being crushed) has no record of its own and is counted as an execution failure.

| Tier | n | execution success | bricks placed | 2 bricks | 3 bricks | 4 bricks | simulator crashes / errors |
|---|---|---|---|---|---|---|---|
| 1 | 100 | 100/100 = 100% | 100.0% | 100/100 | - | - | 0 |
| 2 | 100 | 34/100 = 34% | 45.3% | 16/20 | 16/60 | 2/20 | 22 |
