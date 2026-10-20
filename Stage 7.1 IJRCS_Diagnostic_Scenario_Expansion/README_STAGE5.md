# IJRCS Stage 5 — Passive Repulsive Digital Pheromone

**Frozen Algorithm Specification v1.0 remains unchanged.** Based directly on `IJRCS_Stage4_LocalCommunication.zip`.

## Scope
Stage 5 introduces a separate, bounded, dimensionless pheromone map for each drone. Pheromone is **passive**: no scoring, target selection, Dijkstra input, frontier update, or motion rule reads it. Stage 4 communication and incremental FTFD are unchanged. Legacy mode remains a regression baseline; in legacy mode no pheromone updates are performed.

### New Java classes
- `LocalPheromoneMap.java`: one drone's `double[][]` values (initial 0, upper limit 1), `deposit`, `evaporate`, and defensive `snapshot`.
- `LocalPheromoneExchange.java`: peer-to-peer `max` merge from pre-exchange snapshots, limited to Euclidean communication radius and one hop in each timestep.

### Minimal changes in `DecentralizedExploration.java`
- Create one pheromone field per drone.
- One deposit at each initial drone position (only local mode).
- Per timestep: sense -> evaporate -> occupancy exchange -> pheromone exchange -> FTFD -> existing frontier selection and navigation -> simultaneous movement -> deposit on each successful cell entry.
- No deposit on a blocked move or hovering; no deposit on receiving a peer's pheromone.
- Defensive accessor `getLocalPheromoneSnapshot(int)`; diagnostics `getSharedPheromoneUpdates()`, `getEvaporationRate()` and `getDepositAmount()`.

### Equations
`P_i^*(c,t+1) = min(1, (1-rho) P_i(c,t) + deltaP * v_i(c,t))`.

The implementation separates time phases: evaporation before local communication, and `deposit()` only after a **successful** movement. Initial-cell deposit occurs once before the first timestep. Accordingly, a newly deposited cell-entry trace is propagated beginning with the next timestep.

For each direct communication neighbor `j`, `P_i(c) <- max(P_i(c), P_j(c))`, based on snapshot values taken before exchange. Information cannot traverse multiple peer links in the same timestep.

**Important limitation:** `max` propagation does not represent an exact sum/count of collective visits; it is a locally known, recent visitation intensity. Repeated communication does not amplify a signal by itself.

### Configuration, local mode
VM options:
```
-Dswarm.navigation=local
-Dswarm.communicationRadius=6
-Dswarm.pheromone.rho=0.05
-Dswarm.pheromone.deposit=0.4
-Dswarm.verifyFtfd=true
```
`rho` must be in [0,1]; `deposit` in (0,1]. These are **initial engineering values**, not experimentally optimized IJRCS values. For exact legacy behavior use `-Dswarm.navigation=legacy` (default).

### Headless verification (JDK 21)
The source uses a JavaFX-dependent `Drone.java` for the GUI. For headless algorithm testing **only**, compile the supplied `tests/Drone.java` minimal stub instead. Do NOT copy the test stub into the GUI source package.

```
mkdir -p build
javac -d build tests/Drone.java Point.java LocalDroneMap.java IncrementalFrontierSet.java LocalMapExchange.java LocalPheromoneMap.java LocalPheromoneExchange.java DecentralizedExploration.java tests/Stage3Test.java tests/Stage4Test.java tests/Stage5Test.java tests/RegressionTest.java
java -cp build -Dswarm.navigation=local swarmdrone.decentralizedswarmcoverage_ijrcs.Stage5Test
java -cp build -Dswarm.navigation=local swarmdrone.decentralizedswarmcoverage_ijrcs.Stage4Test
java -cp build -Dswarm.navigation=local -Dswarm.verifyFtfd=true swarmdrone.decentralizedswarmcoverage_ijrcs.RegressionTest
java -cp build -Dswarm.navigation=legacy swarmdrone.decentralizedswarmcoverage_ijrcs.RegressionTest
```

### Verified results on this build
- Java 21 headless compile: PASS.
- Stage 5 deterministic tests: PASS (initial zero, additive deposits, saturation, evaporation, defensive copy, snapshot one-hop chain, delayed forwarding, disconnected pairs, idempotence, max not sum, simulator integration).
- Existing Stage 4 peer exchange tests: PASS.
- Stage 3 FTFD comparisons: PASS.
- Four scenarios, legacy and local: printed `SCENARIO` lines identical to Stage 4 for both modes; stage4 vs stage5 diff returned no differences.
- GUI files `CanvasView.java`, `MainView.java`, `Drone.java` unchanged; JavaFX rendering was not executed in this environment.

### Explicit deferrals
- Do not use pheromone during frontier evaluation until Stage 6.
- IG-2, pheromone-aware utility, event-triggered replanning, rotating collision priority, and frozen-spec termination refinements are not implemented here.
- Stage 5 is an engineering test, not empirical evidence for IJRCS performance improvements.
