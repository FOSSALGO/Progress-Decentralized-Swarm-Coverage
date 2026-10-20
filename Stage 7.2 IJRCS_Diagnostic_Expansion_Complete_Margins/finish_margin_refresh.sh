#!/usr/bin/env bash
cd "$(dirname "$0")" || exit
for spec in 'open 2 47 pheromone' 'clutter 4 11 utility' 'clutter 4 11 pheromone'; do
 read -r layout drones seed mode <<< "$spec"
 name="48x60_${layout}_n${drones}_s${seed}_${mode}"
 echo "START $name $(date -u +%T)" >> results_expansion/final_margin_status.tsv
 timeout 240 java -Xmx1g -cp build -Dswarm.navigation=local -Dswarm.frontier="$mode" -Dswarm.diagnostic.csv="results_expansion/${name}.csv" -Dswarm.verifyFtfd=true swarmdrone.decentralizedswarmcoverage_ijrcs.ExpansionExperiment 48 60 "$layout" "$drones" "$seed" >> results_expansion/final_margin_console.txt 2>>results_expansion/final_margin_errors.txt
 echo "END $name $? $(date -u +%T)" >> results_expansion/final_margin_status.tsv
done
