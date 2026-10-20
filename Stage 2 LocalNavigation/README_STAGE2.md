# IJRCS — Stage 2: Local Map-Based Navigation

**Frozen paper title:** Decentralized Swarm Coverage Using Frontier-Based Exploration and Repulsive Pheromone Stigmergy

## Scope and limits
Stage 2 adds an opt-in **local-map-based nearest-frontier Dijkstra** navigation mode while retaining the Stage 1 **legacy oracle A*** mode unchanged for regression. Default remains legacy to avoid affecting the original UI launch behavior.

Only `DecentralizedExploration.java` differs from Stage 1; all visualization/UI source (`CanvasView`, `MainView`, `Drone`, etc.) is byte-identical. `LocalDroneMap.java` is unchanged.

### Select navigation mode
From VM options (NOT program arguments):

* `-Dswarm.navigation=legacy` (default): original shared aggregate map, global frontier ownership and ground-truth A* planner. For legacy comparison ONLY.
* `-Dswarm.navigation=local`: every drone selects a nearest known frontier from its **own** local occupancy map and plans a shortest path with Dijkstra across its known-FREE cells. No shared owner and no truth or legacy aggregate occupancy access in the local navigation decision. 8-connected moves, diagonal cost sqrt(2), and no corner cutting through unknown or known obstacles.

**Important:** This is an intermediate engineering test mode, not yet the frozen final proposed algorithm. Frontier candidates are currently identified through a simple full local-map scan during Dijkstra, rather than incremental FTFD. No local map communication, pheromone, IG-2, pheromone-aware utility, event-triggered replan, rotating collision priority, or frozen termination implementation is active. Legacy sensing (360 rays, 1° increment, sample 0.125 cell, distance <= radius+0.5) and the original shared GUI evaluation/animation recording remain unchanged. Legacy collision filtering and termination remain enabled in BOTH modes temporarily. Do not publish these intermediate benchmark results as the IJRCS proposed method.

### Visualizations
No source modifications to CanvasView.java, MainView.java, or Drone.java. The original animation still reads each drone's path and the common first-observation timeline. New local maps are decision inputs only in local mode. In local mode, the display's global discovery visualization is an evaluator aggregate, **not** an occupancy map visible to the drone planner.

### What was tested
Java 21 headless compilation using a minimal test-only Drone stub and original `Point.java` and `LocalDroneMap.java` passed. Deterministic test environment: 18 × 22 bordered free grid, two drone starts (3,3), (13,18), sensing radius 3. Both modes passed a trajectory-in-free-cell check and end-of-timestep distinct-occupancy check:

| Mode | Termination | Steps | Observed | Visited | Revisit | Overlap |
|---|---|---:|---:|---:|---:|---:|
| legacy | all reachable cells observed | 31 | 100.00% | 18.44% | 3.28% | 0.00% |
| local | all reachable cells observed | 46 | 100.00% | 26.56% | 9.57% | 9.41% |

The legacy row matches the prior Stage 1 regression fixture exactly. These figures describe ONE fixture and are not scientific performance comparisons. Full NetBeans JavaFX runtime was not executed here because no build configuration/library dependencies were included with the uploaded ZIP.

### Boundary and stage constraints
- Source uses world `truth` for physical sensing, collision validation, and evaluator reachability; NOT for local-mode path planning. This is intentional.
- Local-mode frontier scanning is provisional. No pheromone/utility is included yet.
- Keep original Clean Baseline and Stage 1 ZIPs immutable as regression references.
- Future stage must activate incremental FTFD and validate equivalence against the full local-map scan before replacing this temporary frontier lookup.

### Suggested incremental follow-up
Stage 3: implement incremental FTFD from each drone's changed local observations, with an independent full-scan oracle **for tests only**; keep GUI and both mode switches unchanged. Then add local peer-to-peer exchange and pheromone in separately testable stages.
