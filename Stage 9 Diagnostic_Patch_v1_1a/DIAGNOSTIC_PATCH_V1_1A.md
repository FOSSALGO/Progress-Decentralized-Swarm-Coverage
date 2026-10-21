# Diagnostic Patch v1.1a — deadlock & frontier-exhaustion recovery

## Locked research architecture
Decentralized decisions; private occupancy maps updated by 360-degree ray-casting; frontier-based exploration; single-source Dijkstra; shared global movement-deposited repulsive pheromone. The formula `J(f)=distance(f)+lambda*sum(pheromone along geometric shortest path)` and pheromone evaporation/deposit are unchanged. No GUI or `Drone.java` edits. No new production class.

## Two modes in ONE implementation
- `-Dswarm.recovery=false`: diagnostic-only, legacy collision response (now consistently logged).
- `-Dswarm.recovery=true`: default, includes collision recovery and frontier exhaustion handling. Same mechanism for lambda 0 and positive lambda.

## Changes within DecentralizedExploration.java
1. Deterministic collision resolver: rejects shared destinations, head-on swaps, and opposite crossing diagonal motions; stationary occupants have priority. Moving destination ties alternate with tick parity.
2. Recovery: blocked drones may yield one step into a known-free and physically legal cell, provided it is unoccupied and has no competing next destination. Candidate yield cells are evaluated by simple Euclidean distance to current frontier target, without any pheromone term. This is independent of lambda and therefore does not contaminate the no-pheromone control. This is a navigation-only escape rule, not a frontier allocator.
3. Frontier exhaustion: if a drone reaches a frontier which remains frontier after its next sense cycle, mark that particular cell exhausted at its current local map version. The cell is eligible again as soon as a new observation changes the local map. It is not permanently forbidden and does not edit the physical map.
4. Diagnostic CSV per timestep and drone: current position, goal, desired / accepted next step, reason, consecutive blocks, map version, frontier count, global observed count and current pheromone. Enable via `-Dswarm.trace=/path/trace.csv`; `-Dswarm.traceLast=50` retains only last 50 ticks in output (the Java process still buffers trace rows in memory).
5. Print additional collision / recovery counters; GUI API unchanged.

## IMPORTANT scientific caveats
- Recovery yield scoring is geometric only, including for lambda=0; there is no pheromone-dependent behavior in the collision recovery rule.
- The exact custom `.sc` shown in the user's screenshots was not provided; the original screenshot failure has **not** been reproduced verbatim.
- A collision resolver is a deterministic simulator safety layer, not proof of formal deadlock-freedom.
- Global pheromone assumes an ideal shared storage/communication service, not fully distributed networking.

## Run
From project directory containing 9 Java classes and tests:
```
bash tests/run_diagnostic.sh environments/S05_TwoRooms_4D.sc 0 false tests/trace_before.csv
bash tests/run_diagnostic.sh environments/S05_TwoRooms_4D.sc 0 true  tests/trace_after.csv
bash tests/run_core_tests.sh
```
For IntelliJ / JavaFX, set VM Options:
```
-Dswarm.lambda=1.0 -Dswarm.recovery=true -Dswarm.trace=C:/temp/swarm_trace.csv
```
Use the SAME custom `.sc` and initial positions when comparing lambda=0 and lambda=1.

## Example from existing environment S05 (4 drones, lambda=0)
Diagnostic-only: `STALL_30_STEPS` at 67 steps, observed 75.69%, 164 collision rejects.
Recovery enabled: success at 133 steps, observed 100%, 86 collision rejects, 32 yielding moves.
This is evidence from one provided test scenario, NOT proof of general success or superiority of repulsive pheromone.

## Regression checks
- 25 / 25 combinations of 5 shipped `.sc` environments and lambda {0,0.5,1,2,4} achieved full observed coverage with recovery enabled.
- Per-frame replay length, obstacle avoidance, and duplicate drone positions checked in headless tests.
- JavaFX rendering was not executed here; GUI source files are byte-identical to the P000_Baseline provided in v1.1.
