# IJRCS — Diagnostic Experiment (following Stage 7)

## Scope and design lock

**Algorithm Specification v1.0 remains FROZEN.** This package is based on `IJRCS_Stage7_FinalIntegration.zip` and only adds passive CSV diagnostics. The planner's frontier utility, Dijkstra, FTFD, occupancy exchange, pheromone deposition/evaporation/max-merge, collision resolution, and termination decisions are unchanged. It does not optimize parameters or establish a performance claim. JavaFX source files were not changed.

## Changes

- `FrontierDiagnostics.java`: optional, append-only CSV observer; receives scalar scoring outcomes and replanning event flags. No access to planner/world arrays.
- `DecentralizedExploration.java`: instrumentation hooks invoked only if `-Dswarm.diagnostic.csv=...` is supplied. A read-only counterfactual computes **FU** and **FU-RP** winners from the exact same set of candidate frontiers at each actual scoring event. The real `chosen` target is computed by the preexisting Stage 7 logic, independently of the diagnostics.
- `tests/DiagnosticExperiment.java`: deterministic four-scenario runner.
- `run_diagnostics.sh`, `summarize_diagnostics.py`: compile, run, and tabulate eight scenarios (FU and FU-RP × four environments).
- `results/*.csv`: raw event logs; `results/summary.csv` and `run_console.txt` contain aggregates and simulator metrics.

## Run

Requires JDK 21, bash, Python 3 (standard library only). From the directory containing the Java files:

```bash
bash run_diagnostics.sh
```

To run through NetBeans with unchanged JavaFX UI, supply JVM options similar to:

```
-Dswarm.navigation=local -Dswarm.frontier=pheromone -Dswarm.diagnostic.csv=diagnostic.csv
```

Leave off `swarm.diagnostic.csv` to disable instrumentation.

## CSV definitions

- `REPLAN`: observed actual target-selection event. `occupancy_event`, `peer_pheromone_event`, `invalid_goal`, and `short_route` are nonexclusive triggers, not unique root-cause assignments. `peer_pheromone_event` records increases in known-FREE neighborhoods of *current frontier candidates* detected after communication, not arbitrary remote-map differences.
- `DECISION`: actual evaluation of all reachable candidates; includes min/max/mean/standard deviation of **mean 3×3 known-free frontier pheromone** and number with positive pheromone, actual chosen target, shadow FU and shadow FU-RP targets, shadow winning utility values, and **nonzero pheromone locations anywhere in the local pheromone map**. The shadow FU-RP ranking uses the configured lambda, which was 1.0 in the provided runs.
- Pheromone values near candidates are mean over known-FREE neighbors only, just as in Stage 7. A nonzero map can coexist with all-zero frontier penalties.
- Evaluations and replans are counts of target-selection events, **not** necessarily target changes.

## Observations — deterministic small scenarios

World size 18×22, two agents starting at (2,2) and (15,19), sensor radius 3, communication radius 6, rho=0.05, deposit=0.4, lambda=1.0; Java 21 and Stage 7 motion/simulation logic.

| Scenario | Proposed decision events | Nonzero pheromone in local map at decision | Frontier candidate pheromone > 0 | Shadow target FU vs FU-RP differs | Peer-triggered replans |
| --- | ---: | ---: | ---: | ---: | ---: |
| Open | 74 | 74 | 0 | 0 | 0 |
| Vertical wall | 115 | 115 | 0 | 0 | 0 |
| Horizontal wall | 61 | 61 | 0 | 0 | 0 |
| Cluttered | 53 | 53 | 0 | 0 | 0 |
| **TOTAL** | **303** | **303** | **0** | **0** | **0** |

The local pheromone maximum across decision events is positive (0.723, 0.903, 0.600, 0.690 respectively), so deposition and evaporation are active. Yet the local pheromone average at every *reachable frontier candidate* remains 0 in these four small scenarios. Consequently, the FU and FU-RP formulas have identical score and target rankings. This directly explains identical measured trajectories without modifying the design.

**Timing:** Every recorded Proposed replan is accompanied by an occupancy update and **none** is triggered by a frontier-neighborhood pheromone increase received from a peer in these runs. This is evidence about these scenarios only, not a universal property of the event trigger.

**Hypothesis, not proven mechanism:** In these short environments, frontier cells likely remain separated from pheromone-rich traveled cells by more than the 3×3 neighborhood, particularly with sensor radius 3. Targeted inspection with larger maps, obstacles, swarm counts and prespecified parameters is needed before interpreting effectiveness. It would be methodologically incorrect to make changes simply to force an improvement.

## Validation and limits

- All Stage 3–7 headless tests compiled and passed, including 2,000 random collision safety cases.
- The four FU and four FU-RP regression runs with diagnostics enabled match their runs with diagnostics disabled on completion status, step count, coverage/revisit/overlap, target reselect count, and blocked movement count. Shadow scoring cannot select navigation targets.
- This records *decision-event* data rather than a comprehensive per-timestep record of all pheromone transfers. Zero peer-triggered replans does **not** mean no pheromone was exchanged.
- The four small scenarios are engineering diagnostics, **not** scientific benchmarking, statistical significance testing, or a performance improvement result.
- JavaFX GUI was not executed; `MainView.java`, `CanvasView.java`, `Drone.java` remain identical to Stage 7.

## Recommended next validation (without algorithmic changes)

1. Make a prespecified scenario matrix varying layout complexity and fixed sensor/communication radius, drone count, random seeds, while keeping FU and FU-RP paired.
2. Examine `p_positive`, `p_sd`, and `changed_FU_vs_RP` over time to identify settings with naturally meaningful pheromone signal.
3. Only after identifying signal and documenting all configurations, run a separate multi-seed scientific benchmark and report failures and negative results transparently.
