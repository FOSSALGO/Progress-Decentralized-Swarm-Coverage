# IJRCS Algorithm Specification v1.0 — FROZEN
## Java implementation Stage 1: independent local occupancy records

**Locked article title:** Decentralized Swarm Coverage Using Frontier-Based Exploration and Repulsive Pheromone Stigmergy

## Scope of this release
- Adds `LocalDroneMap.java`, storing one private occupancy grid per drone (`0 UNKNOWN`, `1 FREE`, `-1 OCCUPIED`).
- Changes `DecentralizedExploration.java` to write each sensor observation into the observing drone's own local map, alongside the original aggregate legacy map.
- Exposes `getLocalMapSnapshot(droneId)` and `getLocalKnownCellCount(droneId)` for diagnostics/tests.
- Does not change the visual interface, rendering code, drone colors, or playback.

**Important:** This is a compatibility scaffold, NOT yet the decentralized proposed algorithm. Legacy aggregate map, global `owner` frontier assignment, ground-truth A* planning, original collision handling, and original termination remain active for direct baseline regression. Repulsive pheromone, proximity map exchange, FTFD incremental update, IG-2, Dijkstra, and event-triggered replanning are NOT yet enabled. Do not use Stage 1 runs as Proposed Method experimental data.

## Verification actually performed
1. Java 21 `LocalDroneMap.java` compilation: PASS.
2. Headless compile of `DecentralizedExploration.java` with a minimal test-only Drone stub: PASS. This is NOT a full JavaFX UI compilation.
3. Basic unit tests for UNKNOWN initialization, observation and duplicate observation, map snapshot isolation, and independent per-drone maps: PASS.
4. Comparison with the original unmodified baseline on an 18x22 boundary-obstacle map with two agents at (3,3) and (13,18), sensor radius 3: exact match in recorded results and console output.

Results for this test case: 31 steps, observed coverage 100%, visited coverage 18.4375%, revisit 3.2786885246%, overlap 0%. This is a deterministic regression fixture, not a research result.

## How to integrate
Keep all Java files in the same package `swarmdrone.decentralizedswarmcoverage_ijrcs` as the uploaded project. Add the new `LocalDroneMap.java` to the IDE source tree. Keep the original build/module/JavaFX setup (no build files were included in Baseline.zip). Launch as you launched the baseline.

## Next stages (not part of this release)
1. Add a local-map based navigation/test mode with no oracle ground-truth access, preserving a separate legacy compatibility mode.
2. FTFD + IG-2 with locked radial ray casting (360 rays, 1 degree, 0.125-cell sampling).
3. Proximity-based local occupancy exchange and pheromone maps.
4. Dijkstra and frozen utility with event-triggered replanning.
5. Rotating-priority collision resolution and termination protocol.
6. Reproducibility tests and controlled IJRCS benchmarks.

The frozen design is not permission to infer experimental parameters. Values proposed earlier for rho, deltaP, lambda and radii are preliminary test settings until validated.
