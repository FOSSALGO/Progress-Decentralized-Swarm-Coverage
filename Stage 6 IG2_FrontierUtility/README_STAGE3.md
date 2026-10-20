# IJRCS Stage 3 — Incremental FTFD Frontier Detection

**Frozen paper title:** Decentralized Swarm Coverage Using Frontier-Based Exploration and Repulsive Pheromone Stigmergy

## Scope: only incremental frontier detection
- Keeps Stage 2's two navigation modes, with default `-Dswarm.navigation=legacy` and opt-in `-Dswarm.navigation=local`.
- Adds `IncrementalFrontierSet.java`, an independent frontier tracker for each drone.
- `LocalDroneMap.observe()` returns true when a cell actually changes from UNKNOWN to FREE/OCCUPIED; the corresponding cell and its up to eight neighbors are marked dirty.
- After sensing in each simulation timestep, only dirty cells are re-evaluated. `localShortestFrontier()` checks `localFrontiers[id].contains(r,c)` instead of rescanning all eight neighbors to determine frontier status. The existing Stage 2 Dijkstra graph traversal is unchanged.
- Duplicate frontiers are prevented with a `BitSet`; cells no longer satisfying the frontier rule are removed.
- `IncrementalFrontierSet.fullScan(map)` is an INDEPENDENT full-scan oracle for verification only, not used in navigation.
- `-Dswarm.verifyFtfd=true` checks equality between incremental FTFD and the independent full scan for each drone at EVERY sensor update and at the final sensing update, throwing an exception on mismatch. Disabled by default to avoid full-scanning in normal simulation.
- Legacy global `owner` / legacy global map / oracle A* remain intact for regression and are NOT inputs of local-map navigation.
- No JavaFX GUI or visualization source file was modified; original palette, paths, animation, interface and replay are retained.
- Sensing remains 360 rays, 1-degree angular resolution, 0.125-cell samples, up to `sensorFoV+0.5` legacy sampling limit.

## New class
`IncrementalFrontierSet`: `changed(row,col)`, `update(LocalDroneMap)`, `contains(row,col)`, `size()`, `snapshot()`, and static `fullScan(LocalDroneMap)`.

## Run from NetBeans
Use the same existing project and JavaFX dependencies as in the previous stages. Replace/add the Java source files in the same package. VM options:

- `-Dswarm.navigation=legacy`: existing Stage 2 legacy navigation (default)
- `-Dswarm.navigation=local`: Stage 3 incremental frontier detection + Stage 2 local-map Dijkstra
- `-Dswarm.navigation=local -Dswarm.verifyFtfd=true`: same as local, with independent frontier equality assertions after each sensing update

## Validation executed in Java 21 (headless)
- Compiled Stage 3 algorithm classes (`DecentralizedExploration`, `LocalDroneMap`, `IncrementalFrontierSet`, `Point`) with test-only stand-in for JavaFX `Drone` (only rendering class was stubbed).
- 12 randomized independent occupancy map sequences, 252 UNKNOWN->FREE/OCCUPIED changes per sequence: after each change, incremental frontier `BitSet` EXACTLY equaled independent full-scan oracle. Total 3,024 checks.
- Four deterministic 18x22 maps (open, vertical wall with passage, horizontal wall with passage, clutter), two drones, radius 3: verifier enabled on each drone at every sensing timestep and final update; zero mismatch.
- Compared Stage 2 original against Stage 3 on these four maps in **both** `legacy` and `local` modes. Termination, steps, observed coverage, visited coverage, revisit rate, overlap rate all matched exactly for each map and mode.
- Scenario 1 (vertical wall) in local mode produced STALL_30_STEPS and observed coverage 99.35% on BOTH Stage 2 and Stage 3. This is an inherited Stage 2 navigation/collision limitation; it is not FTFD divergence and must not be misrepresented as successful full exploration.
- This is controlled functional/regression testing, NOT journal experimental data. The actual NetBeans/JavaFX UI runtime was not exercised in this headless environment.

## Explicit exclusions — frozen specification not fully activated
NO local peer map communication, pheromone, IG-2 utility, event-triggered replanning final logic, rotating collision priority, or revised termination logic in this stage. Stage 2 legacy collision resolver and early termination conditions still apply in both modes. Do not claim full paper algorithm implemented yet.

## Next stage
Stage 4: activate bounded local peer-to-peer sharing of occupancy observations, snapshots, and dirty cell propagation into incremental FTFD. Keep pheromone OFF until verified independently.
