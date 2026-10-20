# Diagnostic Scenario Expansion (Stage 7 — FROZEN)

## Purpose and controls
Observational experiment only. No changes to production Stage 7 Java classes, scoring formula, sensor model, 3x3 pheromone neighborhood, Dijkstra, FTFD, communication or collision handling. `tests/ExpansionExperiment.java` generates deterministic worlds and starting locations. `summarize_expansion.py` reads CSV/logs; it does not control drones.

Environment matrix (predeclared): grid sizes 32x40 and 48x60; layouts `open`, `clutter` (seeded 12% obstacles), and `rooms` (seeded door positions); 2 and 4 drones; seeds 11,29,47; modes FU (`utility`) and FU-RP (`pheromone`). Fixed radius 8 cells for sensing; communication radius 16 cells (default 2x sensor radius); rho=0.05, deposit=0.4, lambda=1.0; 360 degrees and sample interval 0.125; max steps 2500, stall steps 30. Drone corner-starts are cleared of obstacles by the test environment generator. Each paired trial has exactly the same world, starts, and parameters except frontier mode. These synthetic maps have not been vetted as benchmark maps; they are diagnostic tests only.

## Execution status (important)
The matrix has 36 paired configurations (72 runs). Due to compute/time limits in this session, **22 paired configurations / 44 runs completed**: ALL 18 pairs at 32x40, three 48x60/open/2-drone pairs, and one 48x60/clutter/4-drone pair. The remaining 14 48x60 pairs were **not run**. They must not be silently counted as successful, failures or zeros. Run logs and CSVs cover only completed runs. Results below are descriptive for completed trials, NOT representative of the missing large-map matrix.

## Results for completed trials
- All 44 completed runs terminated `EXPLORATION_COMPLETE`.
- 22 FU-RP runs produced 3,817 `DECISION` observations; **13** decision events had at least one frontier candidate with mean 3x3 pheromone > 0 (15 positive candidate occurrences total), distributed across **five** FU-RP runs.
- Maximum logged frontier pheromone `p_max` was **0.076**. Pheromone was present in the local maps more broadly, but was generally zero at frontier neighborhoods.
- **0** shadow FU-versus-FU-RP target differences at decision events; **0** differences in steps, visited coverage, revisit or overlap within the 22 paired run comparisons.
- **7** FU-RP `REPLAN` events flagged peer pheromone updates. These event flags are not mutually exclusive with occupancy triggers, and do not imply a different selected target.

## Interpretation
Compared with 18x22 pilot maps (all-zero frontier pheromone), these more varied maps provide direct evidence that nonzero candidate pheromone can arise under unchanged 3x3 scoring and unchanged parameter settings. Nevertheless, in tested events the signal was insufficient to change ranking/selected target. This does not prove FU-RP is universally equivalent to FU; nor is there evidence for superior FU-RP performance here. The planner's score contribution exists but this test indicates low *decision sensitivity*. No formula alteration is warranted as part of FROZEN v1.0.

## Key artifacts
- `results_expansion/runs_summary.csv`: one row per completed method run, plus diagnostic counts.
- `results_expansion/paired_summary.csv`: FU/FU-RP matched comparisons (22 rows).
- `results_expansion/*_utility.csv`, `*_pheromone.csv`: raw decision/replanning events for completed trials.
- `results_expansion/run_console.txt`: raw simulator run output and termination statuses.
- `tests/ExpansionExperiment.java`: deterministic map generator, no algorithm changes.
- `run_expansion.sh`: runs entire 72-run matrix (can take substantial time; up to many minutes) and re-summarizes.
- `summarize_expansion.py`: summarizes whichever paired trials completed in the log.

## Reproducibility
Requires Java 21, Python 3, and shell. To run complete matrix, execute `bash run_expansion.sh` from extracted directory. It resets the run console log before running; preserve the provided CSV/results before doing so. To re-summarize existing logs, execute `python3 summarize_expansion.py`. For reruns of one case, use the Java arguments in `run_expansion.sh`. This runner is headless and requires no JavaFX SDK; GUI classes and original visuals are unchanged, but not tested in a JavaFX runtime.

## Caveats
`DECISION` counter counts target evaluation events, not all candidate/timestep combinations. `changed_shadow_decisions` is same-snapshot counterfactual ranking, not a randomized causal-effect estimate. Identical paired aggregate metrics alone do not prove trajectory identity; the paired comparison establishes equality of those logged aggregate results, while shadow comparisons establish equality at evaluated decisions. Output is exploratory diagnostic, not a preregistered inferential benchmark or proof of algorithmic superiority.
