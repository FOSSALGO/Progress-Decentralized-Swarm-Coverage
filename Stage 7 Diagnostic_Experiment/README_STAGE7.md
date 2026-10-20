# IJRCS Stage 7 — Final Integration & Validation

**Frozen specification:** Algorithm Specification v1.0. Title unchanged: *Decentralized Swarm Coverage Using Frontier-Based Exploration and Repulsive Pheromone Stigmergy*.

## Scope
Built on Stage 6. Local modes retain independent occupancy and pheromone maps, radial sensing 360 degrees / 1 degree / 0.125-cell samples, incremental frontier detection, one-hop occupancy/pheromone exchange, Dijkstra, IG-2, and NF/FU/FU-RP. No new optimization formula. `MainView.java`, `CanvasView.java`, `Drone.java` are byte-for-byte unchanged from Stage 6.

- **Replanning:** retain a valid route unless there is no target, the route is exhausted or invalid, or the local known-cell count changes. For Proposed, a new larger pheromone value received through communication in the known-free 3x3 vicinity of a candidate frontier also triggers reevaluation. Evaporation alone and own deposit do not trigger reevaluation. WAIT without new information does not repeatedly search. An event triggers *reevaluation*, not necessarily target replacement. `getTargetReselections()` counts selection evaluations (not necessarily changed target).
- **Collision:** `RotatingCollisionResolver.java` is a simulator-only synchronous safety filter. It rejects swaps, simultaneous segment intersections, same-cell destinations, and entry into a cell occupied by a waiting/blocked drone; it repeatedly checks cascading conflicts. Priority rank `(droneId - timestep) mod droneCount` (zero-based IDs). No peer locations or truth map are passed to the local frontier planner.
- **Termination:** for local mode, statuses `EXPLORATION_COMPLETE`, `STALLED`, `TIMEOUT` with precedence in that order. Progress for stall condition is at least one successful move OR at least one newly physically sensed reachable FREE cell. Initial configuration without a drone throws `IllegalArgumentException` (`INVALID_SETUP`). Legacy statuses/logic remain unchanged. Sensor observations on finalization are still recorded and may update COMPLETE.
- **Metrics:** observed coverage is physically sensed reachable cells, while visited coverage is unique *entered* reachable cells. Successful cell-entry is counted once, blocked movement and hovering do not add a visit or deposit. `getBlockedMoves()` is a diagnostic counter of rejected proposed steps. `getTargetReselections()` is another diagnostic counter.

## Build & run headless (Java 21)

The `tests/Drone.java` stub is **only** for headless tests, never copy it into the graphical NetBeans source folder.

```sh
mkdir -p build
javac -d build Point.java State.java LocalDroneMap.java IncrementalFrontierSet.java LocalMapExchange.java LocalPheromoneMap.java LocalPheromoneExchange.java RotatingCollisionResolver.java DecentralizedExploration.java tests/Drone.java tests/Stage3Test.java tests/Stage4Test.java tests/Stage5Test.java tests/Stage6Test.java tests/Stage7Test.java tests/RegressionTest.java
java -Dswarm.navigation=local -Dswarm.frontier=pheromone -cp build swarmdrone.decentralizedswarmcoverage_ijrcs.Stage7Test
java -Dswarm.navigation=local -Dswarm.frontier=pheromone -Dswarm.verifyFtfd=true -cp build swarmdrone.decentralizedswarmcoverage_ijrcs.RegressionTest
```

VM options for NetBeans GUI: `-Dswarm.navigation=local -Dswarm.frontier=pheromone -Dswarm.pheromone.lambda=1.0`. Other frontier modes: `nearest`, `utility`. Defaults still select legacy. Existing JavaFX project setup still required.

## Validation results (2026-10-20)

- Java 21 headless compilation succeeded.
- Stage 4 one-hop exchange, Stage 5 pheromone unit tests, Stage 6 scoring tests and Stage 7 collision tests passed. 2,000 seeded random four-drone movement proposals resolved without segment crossing or destination overlap. Same-cell rotating tie, swap, diagonal crossing and cascading cancellation explicitly tested.
- All 4 local regression scenarios achieved `EXPLORATION_COMPLETE` (100% observed coverage). FTFD verification against independent full scan passed when enabled.
- The 4 legacy scenarios produced the same metrics as the Stage 6 legacy benchmark.
- For 4 small regression maps, FU and FU-RP still give identical metrics. **No demonstrated performance advantage from repulsive pheromone yet.** No optimization or new algorithmic component was introduced to force a difference.
- With `-Dswarm.legacy.maxSteps=1`, regression scenarios terminate with `TIMEOUT` as expected. `STALLED` trigger is implemented but has not yet been demonstrated by a dedicated end-to-end scenario reliably generating stalled motion. Collision tests independently cover blocked movement behavior.
- JavaFX GUI visual runtime, larger map/scaling behavior, statistical experiments, and physical UAV validation **not performed**.

## Design caveats and future diagnostics

1. The algorithm continues to use the Stage 6 expected sensing gain and known-free 3x3 pheromone averaging. Small maps may not provide ranking changes despite synthetic ranking tests.
2. `RotatingCollisionResolver` checks geometric intersection of *closed straight segments* over one timestep. This is conservative for consecutive following maneuvers. It models 2D fixed-altitude separation at the cell/segment level, not 3D UAV dynamics.
3. `EXPLORATION_COMPLETE` is evaluator/oracle completion based on ground-truth reachable free cells; drone frontier decisions and path planning never use that oracle.
4. Since `LocalMapExchange` shares observed cells, physically sensed coverage is measured from simulator observation timestamps, not simply from a locally merged map.
5. Stage 7 does not claim completed experimental validation. Next action: instrumentation and diagnosis of whether received pheromone modifies a frontier ranking at actual replanning events, before any scientific benchmark.
