#!/usr/bin/env python3
"""Read-only postprocessing of prespecified FU/RP paired runs."""
import csv,re,statistics
from pathlib import Path
root=Path(__file__).resolve().parent/'results_expansion'
run_lines=(root/'run_console.txt').read_text().splitlines()
rx=re.compile(r'^EXPERIMENT,size=(\d+x\d+),layout=(\w+),drones=(\d+),seed=(\d+),method=(\w+),status=(\w+),steps=(\d+),observed=([\d.]+),visited=([\d.]+),revisit=([\d.]+),overlap=([\d.]+),replans=(\d+),blocked=(\d+)')
rows=[]
for line in run_lines:
    m=rx.match(line)
    if not m:continue
    size,layout,n,seed,mode,status,steps,observed,visited,revisit,overlap,replans,blocked=m.groups()
    path=root/f'{size}_{layout}_n{n}_s{seed}_{mode}.csv'
    with path.open(newline='') as f: events=list(csv.DictReader(f))
    decisions=[e for e in events if e['event']=='DECISION']
    replan=[e for e in events if e['event']=='REPLAN']
    positive=[e for e in decisions if int(e['p_positive'])>0]
    changed=[e for e in decisions if (e['fu_row'],e['fu_col'])!=(e['rp_row'],e['rp_col'])]
    rows.append(dict(size=size,layout=layout,drones=n,seed=seed,method=mode,status=status,steps=steps,observed=observed,visited=visited,revisit=revisit,overlap=overlap,replans=replans,blocked=blocked,
        decisions=len(decisions),positive_decisions=len(positive),positive_candidate_total=sum(int(e['p_positive']) for e in decisions),changed_shadow_decisions=len(changed),peer_replan=sum(e['peer_pheromone_event']=='true' for e in replan),
        max_candidate_pheromone=max((float(e['p_max']) for e in decisions),default=0),max_map_pheromone=max((float(e['map_max']) for e in decisions),default=0),avg_candidate_sd=statistics.mean(float(e['p_sd']) for e in decisions) if decisions else 0))
fields=list(rows[0])
with (root/'runs_summary.csv').open('w',newline='') as f:
    w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(rows)
by={(r['size'],r['layout'],r['drones'],r['seed'],r['method']):r for r in rows}
paired=[]
for key,u in by.items():
    if key[-1]!='utility':continue
    p=by[key[:-1]+('pheromone',)]
    paired.append(dict(size=key[0],layout=key[1],drones=key[2],seed=key[3],fu_steps=u['steps'],rp_steps=p['steps'],step_difference=int(p['steps'])-int(u['steps']),
        fu_status=u['status'],rp_status=p['status'],rp_positive_decisions=p['positive_decisions'],rp_changed_shadow=p['changed_shadow_decisions'],rp_peer_replan=p['peer_replan'],
        fu_visited=u['visited'],rp_visited=p['visited'],fu_revisit=u['revisit'],rp_revisit=p['revisit'],fu_overlap=u['overlap'],rp_overlap=p['overlap']))
with (root/'paired_summary.csv').open('w',newline='') as f:
    w=csv.DictWriter(f,fieldnames=list(paired[0]));w.writeheader();w.writerows(paired)
for layout in ['open','clutter','rooms']:
    r=[x for x in paired if x['layout']==layout]
    print('LAYOUT',layout,'pairs',len(r),'rp_positive',sum(int(x['rp_positive_decisions'])>0 for x in r),'rp_shadow_changed',sum(int(x['rp_changed_shadow'])>0 for x in r),'trajectory_differed',sum(int(x['step_difference'])!=0 or x['fu_visited']!=x['rp_visited'] or x['fu_revisit']!=x['rp_revisit'] or x['fu_overlap']!=x['rp_overlap'] for x in r),'statuses',[(x['fu_status'],x['rp_status']) for x in r if x['fu_status']!='EXPLORATION_COMPLETE' or x['rp_status']!='EXPLORATION_COMPLETE'])
print('TOTAL_RUNS',len(rows),'PAIRS',len(paired))
print('NONZERO',sum(int(x['positive_decisions'])>0 for x in rows if x['method']=='pheromone'),'SHADOW_CHANGED',sum(int(x['changed_shadow_decisions'])>0 for x in rows if x['method']=='pheromone'))
