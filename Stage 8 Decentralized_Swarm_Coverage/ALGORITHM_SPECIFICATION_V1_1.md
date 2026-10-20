# Algorithm Specification v1.1 — Implemented candidate

Research title: **Decentralized Swarm Coverage Using Frontier-Based Exploration and Repulsive Pheromone Stigmergy**.

## Frozen source/UI
- Original `CanvasView.java`, `MainView.java`, `Drone.java`, `CSS.java`, `SVGIcon.java`, `Point.java`, `State.java`, `Launcher.java` remain byte-for-byte identical to P000_Baseline.zip.
- All algorithm implementation is in `DecentralizedExploration.java`. Production class count stays nine.
- GUI public API retained: constructor `(int[][], ArrayList<Drone>)`; `getFirstObservedAt`, `getMustExploreMask`, `getSteps`, `getCoverage`, `getRevisit`, `getOverlap`, `getObservedRatio`, `getTerminationReason`.
- NOTE: Existing README.md is the untouched *legacy* baseline README and describes the earlier algorithm. This specification overrides its algorithm section.

## Decisions
1. Independent local occupancy grids `localMaps[drone][row][col]`, 0 unknown / 1 observed free / -1 observed obstacle.
2. Sensor: exactly 360 radial rays, occlusion at walls, radius from each `Drone.sensorFoV` (4 in the locked GUI).
3. Frontier: observed free cell with an 8-neighbor unknown cell; never allocate exclusive ownership.
4. Path planner: deterministic single-source Dijkstra on the drone's observed free cells; 8-neighbor steps with costs 1 and sqrt(2), diagonal corner cutting disabled.
5. For all reachable frontiers, reconstruct unique deterministic shortest path from Dijkstra's predecessor tree, including the frontier but excluding the start when summing pheromone.
6. Choose minimum `score = shortestPathLength + lambda * sum(pheromone over path except start)`, tie-breaking by distance then row/column. Pheromone is nonnegative and is a soft repulsive penalty ONLY for target selection, not Dijkstra edge weights.
7. Each drone independently retains a goal until it is no longer frontier, next step is invalid, or route is exhausted. No central assignment.
8. Shared globally available pheromone field (ideal global communication assumption, NOT fully distributed storage). All drones read pre-move values. End of each timestep: evaporate all cells once (`P*=1-rho`) then deposit `q` into each destination successfully entered. No deposit on wait, no diffusion.
9. Simulator maintains ground truth for sensing, physics, reachable masks, and metrics; the local planner cannot use ground truth.
10. Collision handling uses the unchanged baseline simultaneous collision filtering. A prevented move invalidates route for replanning; 30 consecutive steps without ANY move trigger STALL. Max steps default 2500.
11. No changes to visual components and replay path contract (steps + 1 frames). Simulation itself remains synchronous; don't claim asynchronous simulation.

## Public experiment properties (set as JVM -D flags)
- `swarm.lambda` default 1.0, `swarm.rho` default 0.05, `swarm.deposit` default 1.0
- `swarm.maxSteps` default 2500; `swarm.stallLimit` default 30
- `lambda=0` is the ablation on the SAME code path. Deposit and evaporate remain active, but target utility ignores them.
- Values are read at *object construction*, allowing multiple experiments in one JVM.

## Diagnostics
- `decisions`: planning events; `changedDecisions`: selected frontier differs from geometric-nearest at those events; `positivePathDecisions`: any candidate with positive pheromone on reconstructed path; `differingPenaltyDecisions`: some candidates have non-identical path penalties; `moves`: actual successful movements.
- These are within-run counterfactual *decisions*, not evidence of causal gains in coverage on their own.

## Limitations for the article
- Centralized simulator + globally shared virtual pheromone are idealized assumptions. Decentralized target choice does NOT imply distributed storage or actual asynchronous communication.
- Dijkstra chooses a single shortest predecessor path per frontier; alternative equal-length paths are not pheromone-optimized.
- 8-neighbor frontier detection may include frontier cells whose nearby unknown region is occluded; a sensing deadlock is possible. Stall is a failure, never silently treated as completion.
- A shared virtual pheromone array encodes traffic, not guaranteed explored free space. The simulation does not model physical pheromone sensing, network delay or packet loss.
- Obstacles are static. No energy model or 3-D flight dynamics.
- Current 5 example scenarios / single deterministic start per scenario are integration tests, not sufficient for statistically significant performance claims.

## Run
From the directory containing the nine Java sources: `bash tests/run_core_tests.sh`.
Uses a stubbed *Drone only at test time* so no JavaFX installation is needed for algorithm tests; it does NOT validate pixel-perfect or runtime GUI behavior. GUI files remain unchanged, but run them in your existing JavaFX project for manual visual acceptance.

## GUI manual regression checklist
- Launch; Open each `.sc`; obstacle boundaries and drone count should match.
- Run, Play/Pause, Replay; paths and observation overlay should remain synchronized.
- Check zoom/pan, add/remove drone, line/obstacle editor, save/reopen `.sc`, path palette and sensor visuals.
- Check exception dialog for malformed `.sc` and ensure the prior scene remains intact.
