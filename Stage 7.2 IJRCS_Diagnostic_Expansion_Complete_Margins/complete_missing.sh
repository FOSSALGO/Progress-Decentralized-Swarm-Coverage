#!/usr/bin/env bash
set -uo pipefail
cd "$(dirname "$0")"
: > results_expansion/remaining_run_status.tsv
for layout in open clutter rooms; do
for drones in 2 4; do
for seed in 11 29 47; do
  u="results_expansion/48x60_${layout}_n${drones}_s${seed}_utility.csv"
  p="results_expansion/48x60_${layout}_n${drones}_s${seed}_pheromone.csv"
  if [[ -s "$u" && -s "$p" ]]; then continue; fi
  for mode in utility pheromone; do
    name="48x60_${layout}_n${drones}_s${seed}_${mode}"
    outfile="results_expansion/${name}.csv"
    echo "START $(date -u +%T) $name" >> results_expansion/remaining_run_status.tsv
    timeout 240 java -Xmx1g -cp build -Dswarm.navigation=local -Dswarm.frontier="$mode" -Dswarm.diagnostic.csv="$outfile" -Dswarm.verifyFtfd=true swarmdrone.decentralizedswarmcoverage_ijrcs.ExpansionExperiment 48 60 "$layout" "$drones" "$seed" >> results_expansion/run_console.txt 2>>results_expansion/remaining_errors.log
    rc=$?
    echo "END $(date -u +%T) $name $rc" >> results_expansion/remaining_run_status.tsv
  done
done
done
done
