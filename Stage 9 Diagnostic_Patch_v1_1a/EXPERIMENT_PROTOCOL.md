# Experimental Protocol — preregistration draft (NOT article results)

## Research hypotheses
H1: Global repulsive path-pheromone frontier selection reduces steps-to-full-observed-coverage relative to the same independent Dijkstra nearest-frontier algorithm with lambda 0.
H2: It reduces physical revisit / inter-drone overlap without impairing completion probability.
H3: Benefits depend on topology, start positions and repulsion strength; narrow serpentine corridors may be neutral or harmful.

## Controlled variables
- Identical `.sc` files, drone starts, ray radius 4, local-map Dijkstra, 8-neighbor topology, collision physics, termination, maximum 2500 steps and stall threshold 30.
- Compare lambda=0 vs lambda=0.5,1,2,4 (development only). Fixed q=1, rho=.05; optionally rho ∈ {.01,.05,.1} in sensitivity analysis, but do not tune on test set.
- Predeclare lambda on development maps and validate unchanged on *held-out* maps. All failed runs count as failed runs, not dropped observations.

## Supplied examples (32x32)
- S01 OpenSpace: free room, three starts.
- S02 Partitions: vertical partition walls with passageways, three starts.
- S03 Branches: crossing branch-like partitions, three starts.
- S04 Serpentine: one-cell route, two starts (stress test; pheromone should not prohibit traversal).
- S05 TwoRooms: split room and narrow gateways, four starts.

## Repeated experiments for the actual article
- Generate at least 20 independent maps/start configurations per topology/robot-count stratum, or predefine a power analysis. For paired comparison, NF and RP use identical instance IDs and initial placements.
- Include N=1,2,3,4,6 drones where feasible (N=1 negative control, to separate self-repulsion from swarm coordination).
- Use development/test split with no maps shared between tuning and evaluation; vary sensor radius separately only after primary experiment.
- Primary outcomes: completion rate and steps to observed coverage=100% (compare steps conditional on success ONLY with explicit warning of selection bias, and report censored times for failures).
- Secondary: observed coverage at fixed budget, move count, revisit, cross-agent overlap, decision change rate, diagnostic path penalty spread.
- Suggested analysis: paired success contingency test; bootstrap CIs on paired differences (especially fixed-budget coverage), paired Wilcoxon or permutation tests for suitable fully observed outcomes; report effect size and multiple comparisons correction if testing many lambda values.
- Discuss any failure/STALL, map topology interactions and caveats; do not cherry-pick.

## Quick run
`bash tests/run_core_tests.sh` saves `tests/results.csv` (25 runs). This is a preliminary functional diagnostic only; use a full paired-seed suite before publication.

## GUI compatibility manual test
The headless tests verify algorithm and replay contract only. Use the existing GUI environment to visually inspect Open/Save, editor, Run, Replay, color selection, sensor overlay and path rendering; visual rendering not automatically regression-tested here.
