#!/usr/bin/env python3
"""Standard-library summary of 8 deterministic diagnostic CSV files."""
import csv
from pathlib import Path
root = Path(__file__).parent / 'results'
lines=[]
for method in ('utility','pheromone'):
    for scenario in range(4):
        path = root/f'{method}_{scenario}.csv'
        with path.open(newline='', encoding='utf-8') as file: rows=list(csv.DictReader(file))
        decisions=[r for r in rows if r['event']=='DECISION']
        replans=[r for r in rows if r['event']=='REPLAN']
        lines.append(dict(method=method,scenario=scenario,decisions=len(decisions),
             with_pheromone_on_any_frontier=sum(int(r['p_positive'])>0 for r in decisions),
             local_map_with_pheromone=sum(int(r['map_nonzero'])>0 for r in decisions),
             max_local_pheromone=max((float(r['map_max']) for r in decisions),default=0),
             changed_FU_vs_RP=sum((r['fu_row'],r['fu_col'])!=(r['rp_row'],r['rp_col']) for r in decisions),
             replans=len(replans),peer_pheromone_trigger=sum(r['peer_pheromone_event']=='true' for r in replans),
             occupancy_trigger=sum(r['occupancy_event']=='true' for r in replans)))
with (root/'summary.csv').open('w',newline='',encoding='utf-8') as file:
    writer=csv.DictWriter(file,fieldnames=list(lines[0])); writer.writeheader();writer.writerows(lines)
for line in lines:print(line)
