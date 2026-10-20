#!/usr/bin/env bash
set -uo pipefail
cd "$(dirname "$0")"
: > results_expansion/large_refresh_status.tsv
for spec in 'open 2 11' 'open 2 29' 'open 2 47' 'clutter 4 11'; do
 read -r layout drones seed <<< "$spec"
 for mode in utility pheromone; do
 name="48x60_${layout}_n${drones}_s${seed}_${mode}"
 echo "START $name" >> results_expansion/large_refresh_status.tsv
 timeout 180 java -Xmx1g -cp build -Dswarm.navigation=local -Dswarm.frontier="$mode" -Dswarm.diagnostic.csv="results_expansion/${name}.csv" -Dswarm.verifyFtfd=true swarmdrone.decentralizedswarmcoverage_ijrcs.ExpansionExperiment 48 60 "$layout" "$drones" "$seed" >> results_expansion/large_refresh_console.txt 2>>results_expansion/large_refresh_errors.txt
 echo "END $name $?" >> results_expansion/large_refresh_status.tsv
 done
done
