# IJRCS Diagnostic Scenario Expansion — COMPLETED (Algorithm Specification v1.0 FROZEN)

## Scope
Completed the prespecified 36 FU/FU-RP scenario pairs (72 runs). Two grid sizes (32x40, 48x60), three layouts (open, clutter, rooms), two drone counts (2,4), three seeds (11,29,47). FU and FU-RP receive identical worlds, starts and fixed parameters: sensing radius=8, communication radius=16, rho=.05, deposit=.4, lambda=1.0, 3x3 frontier pheromone, 360 rays every 1 degree and step 0.125, 2500 max steps, 30 stall steps. No changes to navigational algorithm, scoring, underlying planner or visualization. The only changes in the two production Java source files are additional read-only diagnostic fields and calculations inside the pre-existing `diagnostics.enabled()` conditional branch. For scientific comparisons, use only original decision logic, never diagnostic fields, as the policy.

## Completed matrix
72/72 runs completed (36 paired configurations). All 72 returned EXPLORATION_COMPLETE; no run differences in logged completion steps, visited coverage, revisit and overlap across each pair. All 36 FU-RP runs contain 9,050 DECISION events. 69 decision events (across 12 runs) have at least one positive pheromone candidate; positive candidate occurrences = 106. Shadow FU vs FU-RP rankings differ in **0/9,050** decisions. 13 peer pheromone-triggered replanning flags (not necessarily sole trigger). Maximum candidate pheromone across all runs = 0.076.

## Utility margin: exact passive definition
At each DECISION, for the shared reachable frontier candidate set, compute FU_i(f) = IGnorm_i(f) - Dnorm_i(f); Proposed_i(f) = FU_i(f) - lambda * P_i(f). The best and runner-up candidates under each score are found with exactly the same deterministic comparator/tie-breaking as the existing planner. `fu_margin` is best FU score less runner-up FU score, and `rp_margin` the analog for FU-RP. Diagnostic ranking and margin are computed **after** actual target selection and do not feed into movement. For a singleton frontier margin is NaN (none occurred in the logged DECISION events in this matrix).

Across the 9,050 FU-RP shadow decision events, FU margin median = 0.0116784905, mean = 0.0258213093, min = 0.0, max = 0.876755047. On the 69 positive-candidate decision events, FU margin median = 0.017993001 and mean = 0.0214105191. **Pheromone at FU's chosen winning frontier is 0 in all 9,050 decisions**, including all 69 events with positive-pheromone alternative candidates. Thus, for any candidate j: FU(f*) >= FU(j) while P(f*) = 0 <= P(j), so FU(f*) - lambda*P(f*) >= FU(j) - lambda*P(j) for any lambda >= 0. Fixed tie-breaker preserves the FU winner. This is a direct mathematical explanation of why the ranking did not change in this matrix; it does NOT assert the zero-winner property for other scenarios.

`global_sufficient_margin` is a conservative sufficient test (FU margin > lambda * max candidate pheromone). It holds for 8,690/9,050 decisions, but the zero-pheromone winning frontier property is stronger and explains the remaining decisions too. A non-positive/zero FU margin does not imply a tie-breaker error; the comparator's deterministic tie-breaker is unchanged.

### Scientific interpretation
- Positive repulsive pheromone near SOME reachable frontier candidates does occur, mainly in clutter layouts, but is typically sparse.
- FU already chooses a candidate with zero penalty, so nonnegative repulsion cannot improve/re-rank the chosen target at the tested decision snapshots.
- Zero difference in selected frontier and aggregate outcomes is an observed result. It is not evidence that FU-RP improves swarm coverage.
- This experiment was diagnostic, not a powered statistical efficacy benchmark. Do not adjust lambda, deposition, evaporation, neighborhood, IG, Dijkstra or replanning rules in response. Algorithm Spec v1.0 remains FROZEN.

## Files
- `results_expansion/run_console.txt`: 72 original run outcomes.
- `results_expansion/remaining_run_status.tsv`: successful status codes for 28 newly completed large-grid runs.
- `results_expansion/runs_summary.csv`: 72 rows (one per method/run).
- `results_expansion/paired_summary.csv`: 36 paired comparisons.
- `results_expansion/utility_margin_decisions.csv`: 9,050 per-decision FU-RP observations with exact margins.
- `results_expansion/utility_margin_by_group.csv`: grouping by map size, layout, drone count.
- `results_expansion/*_utility.csv` and `*_pheromone.csv`: all 72 per-method raw event CSVs.
- `tests/ExpansionExperiment.java`: deterministic world generator.
- `complete_missing.sh`: executes previously missing 14 pairs; skips pairs for which both CSVs exist. `refresh_margins.sh`, `refresh_large_original.sh`, `finish_margin_refresh.sh` reproduce margin-column logs for old runs.
- `summarize_expansion.py`, `analyze_utility_margins.py`: passive postprocessing.

## Reproducibility and constraints
Requires Java 21, Python 3. To reproduce all 36 scenario pairs from scratch, run `bash run_expansion.sh` (rewrites run_console and raw CSVs), followed by `python3 analyze_utility_margins.py`. To inspect current archived results without rerunning, run `python3 summarize_expansion.py` and `python3 analyze_utility_margins.py`. Headless runs use `tests/Drone.java` as a stub; do not place that stub into the NetBeans/JavaFX production project. Stage 3–7 unit checks were executed, and Stage 4–7 tests printed PASS under appropriate `-Dswarm.navigation=local -Dswarm.frontier=pheromone`. GUI runtime remains unverified. Numerical results are based on deterministic synthetic worlds; they do not establish efficacy or generalizability beyond this test matrix.
