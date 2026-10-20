#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
mkdir -p build results_expansion
javac -d build Point.java LocalDroneMap.java IncrementalFrontierSet.java LocalMapExchange.java LocalPheromoneMap.java LocalPheromoneExchange.java RotatingCollisionResolver.java FrontierDiagnostics.java DecentralizedExploration.java tests/Drone.java tests/*.java
: > results_expansion/run_console.txt
for size in '32 40' '48 60'; do
  read -r rows cols <<< "$size"
  for layout in open clutter rooms; do
    for drones in 2 4; do
      for seed in 11 29 47; do
        for mode in utility pheromone; do
          name="${rows}x${cols}_${layout}_n${drones}_s${seed}_${mode}"
          java -Xmx1g -cp build -Dswarm.navigation=local -Dswarm.frontier="$mode" \
            -Dswarm.diagnostic.csv="results_expansion/${name}.csv" \
            -Dswarm.verifyFtfd=true \
            swarmdrone.decentralizedswarmcoverage_ijrcs.ExpansionExperiment \
            "$rows" "$cols" "$layout" "$drones" "$seed" >> results_expansion/run_console.txt
        done
      done
    done
  done
done
python3 summarize_expansion.py
