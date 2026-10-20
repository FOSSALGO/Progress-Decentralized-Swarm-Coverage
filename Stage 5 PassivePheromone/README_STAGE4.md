# IJRCS Stage 4 — Local Peer-to-Peer Occupancy Communication

**Article title (FROZEN):** Decentralized Swarm Coverage Using Frontier-Based Exploration and Repulsive Pheromone Stigmergy

## Scope
Built exclusively on Stage 3. Activates occupancy-only peer-to-peer map exchange in `local` navigation mode, using distance-limited links, a simultaneous pre-exchange snapshot, and exactly one hop per timestep. No pheromone and no new frontier utility. Dijkstra, sensor scanning (360 rays / 1 degree / 0.125-cell samples), incremental FTFD, and all GUI files remain unchanged. Legacy mode is unchanged and remains the default.

## Source changes
- **NEW** `LocalMapExchange.java`: pure local-map exchange helper; receives local occupancy arrays, drone positions, radius, and incremental frontier trackers. Has NO reference to environment ground truth. Copies maps once at start of each communication phase, computes neighbors by Euclidean squared distance, and copies each known FREE/OCCUPIED cell to unknown cells in directly connected peers only. Conflicting known reports raise an exception rather than silently overwrite. Each new cell calls `frontiers[i].changed(row,col)`.
- `DecentralizedExploration.java`: sets `communicationRadius` to twice the *minimum* drone sensor radius by default; optional system property `swarm.communicationRadius` overrides. Calls exchange **after all drones sense** and **before** updating any incremental FTFD tracker, ONLY when `swarm.navigation=local`. Provides `getCommunicationRadius()` and `getReceivedCellUpdates()` diagnostics. This stage does not modify simulator termination, event-triggered replanning, or collision handling inherited from Stage 3.
- `tests/Stage4Test.java`: independent protocol tests for one-hop chain, delayed two-hop arrival, disconnected peers, idempotence, frontier creation/removal after communication, and defensive map snapshot.
- GUI source files `CanvasView.java`, `MainView.java`, and `Drone.java` have byte-for-byte identical contents to Stage 3.

## Operation per simulation timestep
1. All drones individually perform actual 360-degree sensing into their own local occupancy maps.
2. Create one snapshot of each local map **before** any peer merges.
3. Each drone learns all known map cells from drones whose Euclidean distance at the current timestep is <= Rc. No messages are forwarded within the same timestep.
4. Incremental FTFD updates its dirty cells (new physical observations AND communication updates). Optional `swarm.verifyFtfd=true` independently compares incremental FTFD with a full scan after every timestep.
5. Existing local Dijkstra/navigation/collision behavior proceeds as in Stage 3.

**Important:** The Stage 3 `map` aggregate still exists for the legacy benchmark, animation, and the evaluator's physical-observation timestamps. Local pathfinding does not read it. Communication-only knowledge does **not** count as a new physical observation in `getObservedRatio()`: the simulator's observed ratio is union of real sensed FREE cells. This is intentional. The global shared map MUST NOT be used for local planning.

## Properties
- `-Dswarm.navigation=legacy` (default): Stage 3 legacy oracle path planner and global frontier ownership; no peer exchange.
- `-Dswarm.navigation=local`: independent local maps + Stage 3 incremental FTFD + Stage 4 occupancy sharing.
- `-Dswarm.navigation=local -Dswarm.communicationRadius=6`: explicit communication range of 6 grid cells.
- `-Dswarm.navigation=local -Dswarm.verifyFtfd=true`: check equality with independent full scan on every sensing/communication update.

The default radius `Rc = 2 * min(sensorFoV_i)` provides a reproducible homogeneous-drone default, **not** a scientifically validated final value; configure explicitly for journal experiments.

## Reproducible headless tests (Java 21)
In a copy of this folder:

```bash
mkdir -p build
javac -d build tests/Drone.java tests/Stage3Test.java tests/Stage4Test.java tests/RegressionTest.java \
    Point.java LocalDroneMap.java IncrementalFrontierSet.java LocalMapExchange.java DecentralizedExploration.java
java -cp build swarmdrone.decentralizedswarmcoverage_ijrcs.Stage4Test
java -cp build -Dswarm.navigation=local -Dswarm.verifyFtfd=true \
    swarmdrone.decentralizedswarmcoverage_ijrcs.Stage3Test
java -cp build -Dswarm.navigation=legacy -Dswarm.verifyFtfd=true \
    swarmdrone.decentralizedswarmcoverage_ijrcs.Stage3Test
```

`tests/Drone.java` is a **headless test stub** and must NEVER replace the real `Drone.java` from the root source directory when using NetBeans/JavaFX.

## Actual checks run in this environment
- Java 21 compilation: PASS.
- Protocol test (three-drone chain A--B--C, no A--C link): PASS. A and C do not learn each other's unique cells in the first timestep; they do learn them on the second timestep.
- Range exclusion, repeat/idempotent exchanges, occupancy propagation, changed-cell dirty tracking, defensive snapshots: PASS.
- Incremental frontier vs independent full scan across all tested timesteps: PASS, no mismatch, for all 4 regression maps and 2 drones.
- 12 randomized map sequences * 252 occupancy changes (Stage 3 test): PASS, 3024 comparisons.
- Stage 3 vs Stage 4 **legacy** regression across four maps: exact equality of status, timestep, observed/visited coverage, revisit, and overlap: PASS.
- Stage 4 local mode on four 18x22 maps, two drones, sensing radius 3 and default communication radius 6:

| Scenario | Status | Steps | Observed % | Visited % | Revisit % | Overlap % |
|---|---|---:|---:|---:|---:|---:|
| Open | ALL_REACHABLE_FREE_CELLS_OBSERVED | 37 | 100.00 | 22.81 | 3.95 | 2.74 |
| Vertical wall | ALL_REACHABLE_FREE_CELLS_OBSERVED | 43 | 100.00 | 25.24 | 11.36 | 8.97 |
| Horizontal wall | ALL_REACHABLE_FREE_CELLS_OBSERVED | 51 | 100.00 | 27.87 | 18.27 | 11.76 |
| Cluttered | ALL_REACHABLE_FREE_CELLS_OBSERVED | 35 | 100.00 | 21.86 | 5.56 | 4.41 |

These are **engineering regression checks, not peer-reviewed experimental results**. No actual JavaFX GUI window was run; visual sources were left unchanged and need runtime testing in NetBeans.

## Explicit exclusions
No pheromone state, evaporation, deposition, max merge, IG-2, pheromone-aware utility, final event-triggered replanning, rotating collision priority, or finalized termination logic. These stay as future stages under Algorithm Specification v1.0 FROZEN.

## Next
Stage 5: add a single independent per-drone pheromone grid and separately validate entry-only deposition, evaporation, and snapshot-based local max-merge before enabling pheromone utility.
