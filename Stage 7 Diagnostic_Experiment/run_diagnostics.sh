#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
mkdir -p build results
javac -d build Point.java LocalDroneMap.java IncrementalFrontierSet.java LocalMapExchange.java LocalPheromoneMap.java LocalPheromoneExchange.java RotatingCollisionResolver.java FrontierDiagnostics.java DecentralizedExploration.java tests/Drone.java tests/*.java
for mode in utility pheromone; do
  for scenario in 0 1 2 3; do
    java -cp build -Dswarm.navigation=local -Dswarm.frontier="$mode" \
      -Dswarm.diagnostic.csv="results/${mode}_${scenario}.csv" \
      -Dswarm.verifyFtfd=true \
      swarmdrone.decentralizedswarmcoverage_ijrcs.DiagnosticExperiment "$scenario"
  done
done
python3 summarize_diagnostics.py
