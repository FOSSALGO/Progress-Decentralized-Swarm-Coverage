# Stage 6 — Expected Sensing Gain (IG-2) and Pheromone-Aware Frontier Selection

Algorithm Specification v1.0 remains FROZEN. This stage begins from IJRCS_Stage5_PassivePheromone.zip. No graphical files were modified.

## Three algorithms, one local navigation engine

Choose using VM properties (not program arguments):

- Legacy: `-Dswarm.navigation=legacy` (original globally informed legacy comparator)
- Nearest Frontier: `-Dswarm.navigation=local -Dswarm.frontier=nearest` (default local frontier mode)
- Frontier Utility: `-Dswarm.navigation=local -Dswarm.frontier=utility`
- Proposed: `-Dswarm.navigation=local -Dswarm.frontier=pheromone -Dswarm.pheromone.lambda=1.0`

Other initial properties: `-Dswarm.pheromone.rho=0.05`, `-Dswarm.pheromone.deposit=0.4`, `-Dswarm.communicationRadius=6`, `-Dswarm.verifyFtfd=true`.

No change to `MainView.java`, `CanvasView.java`, `Drone.java`. The default remains legacy for UI compatibility; explicitly set local + frontier method to compare algorithms.

## Equations

- Expected sensing gain IG-2 is the number of unique UNKNOWN cells intercepted by 360 rays (one degree each) launched from the center of each reachable frontier. Samples every 0.125 cell, through distance <= Rs + 0.5 exactly as baseline sensing. Known OCCUPIED cells stop rays; UNKNOWN cells do not. Ground truth is NEVER read.
- Each frontier is evaluated using one local-map Dijkstra expansion with eight-connected known-free movement and no diagonal corner cutting. Dijkstra computes shortest-path distance for all reachable frontier cells and predecessor links for final reconstruction.
- Gain normalization = gain / max gain over reachable candidates (zero if maximum is zero).
- Distance normalization = distance / max distance over reachable candidates (zero if maximum is zero).
- Pheromone penalty = mean of local repulsive pheromone intensity in FREE cells of the 3x3 neighborhood centered at the frontier, in [0,1].
- Utility: U = normalized gain - normalized distance - lambda * local pheromone mean.
- Nearest Frontier retains Stage 5 nearest-Dijkstra target selection; Frontier Utility uses lambda=0; Proposed uses configured lambda >= 0.
- Utility ties within 1e-9 are resolved by shortest path distance, then row-major frontier index.

## Triggers / important limits

- All local algorithms share sensing, FTFD, peer-to-peer occupancy exchange, one-hop pheromone exchange, local map movement constraints, and existing collision mechanics. Legacy remains a separate unchanged comparator, NOT a decentralized scientific baseline.
- In utility modes, changes to locally known occupancy trigger goal reevaluation. For proposed mode, received pheromone updates trigger conservative reevaluation across agents; evaporation and own deposition alone do not. These are simple event triggers, not a fully optimized event-dependency system.
- **Stage 7 remains responsible for final rotating collision priority, termination policy, and more rigorous replanning validation.** Stage 6 still inherits fixed-ID collision arbitration and Stage 5 termination behavior.
- The proposed mode's pheromone dynamics run in all local methods for code parity; the only difference between Frontier Utility and Proposed is applying the pheromone penalty during scoring.
- The existing JavaFX view has not been executed in this headless environment. No visual source changed.

## Verified regression results (engineering only)

Compile using Java 21 from this directory:

```bash
mkdir -p classes
javac -d classes Point.java State.java LocalDroneMap.java IncrementalFrontierSet.java LocalMapExchange.java LocalPheromoneMap.java LocalPheromoneExchange.java DecentralizedExploration.java tests/Drone.java tests/RegressionTest.java tests/Stage3Test.java tests/Stage4Test.java tests/Stage5Test.java tests/Stage6Test.java
java -cp classes -Dswarm.navigation=local -Dswarm.frontier=pheromone -Dswarm.verifyFtfd=true swarmdrone.decentralizedswarmcoverage_ijrcs.Stage6Test
java -cp classes -Dswarm.navigation=local -Dswarm.frontier=nearest swarmdrone.decentralizedswarmcoverage_ijrcs.RegressionTest
java -cp classes -Dswarm.navigation=local -Dswarm.frontier=utility swarmdrone.decentralizedswarmcoverage_ijrcs.RegressionTest
java -cp classes -Dswarm.navigation=local -Dswarm.frontier=pheromone swarmdrone.decentralizedswarmcoverage_ijrcs.RegressionTest
```

**Important:** tests/Drone.java is a headless stub; NEVER put it in the same source folder as the actual GUI Drone.java.

On 18x22, 2-drone, sensing radius 3, four test maps:

| Map | Nearest steps | FU steps | Proposed steps | Observation |
|---|---:|---:|---:|---|
| Open | 37 | 52 | 52 | All completed |
| Vertical wall | 43 | 73 | 73 | All completed |
| Horizontal wall | 51 | 43 | 43 | All completed |
| Cluttered | 35 | 42 | 42 | All completed |

FU and Proposed showed the same coverage, revisit and overlap on these four small scenarios at lambda=1. This is **NOT evidence of improved swarm coverage or lower overlap**. A deterministic synthetic frontier scoring test demonstrates the penalty can change frontier rankings. Proposed with lambda=0 exactly reproduced FU on the four regression maps. More maps/seeds, parameter sweeps, and Stage 7 completion are necessary before drawing scientific conclusions.

Regression verification: `Stage3Test`, `Stage4Test`, `Stage5Test`, `Stage6Test` pass in headless Java 21; Stage 6 nearest reproduces Stage 5 steps on all four cases. Incremental frontiers match full scan with `-Dswarm.verifyFtfd=true`.
