#!/usr/bin/env bash
set -uo pipefail
cd "$(dirname "$0")"
: > results_expansion/margin_refresh_status.tsv
for layout in open clutter rooms; do
for drones in 2 4; do
for seed in 11 29 47; do
for mode in utility pheromone; do
 name="32x40_${layout}_n${drones}_s${seed}_${mode}"
 echo "START $name" >> results_expansion/margin_refresh_status.tsv
 timeout 180 java -Xmx1g -cp build -Dswarm.navigation=local -Dswarm.frontier="$mode" -Dswarm.diagnostic.csv="results_expansion/${name}.csv" -Dswarm.verifyFtfd=true swarmdrone.decentralizedswarmcoverage_ijrcs.ExpansionExperiment 32 40 "$layout" "$drones" "$seed" >> results_expansion/margin_refresh_console.txt 2>> results_expansion/margin_refresh_errors.txt
 echo "END $name $?" >> results_expansion/margin_refresh_status.tsv
done
done
done
done
