#!/usr/bin/env python3
"""Passive analysis of preexisting per-decision diagnostic CSVs. No simulator calls."""
import csv,glob,math,statistics
from collections import Counter,defaultdict
from pathlib import Path
root=Path(__file__).parent/'results_expansion'
rows=[]
for filename in sorted(root.glob('*_pheromone.csv')):
    parts=filename.stem.split('_')
    if len(parts)!=5:continue
    size,layout,n,seed,_=parts
    for e in csv.DictReader(filename.open()):
        if e['event']!='DECISION' or not e.get('fu_margin'): continue
        pmax=float(e['p_max']);fm=float(e['fu_margin']);rm=float(e['rp_margin'])
        winnerp=float(e['fu_winner_p']);secondp=float(e['fu_second_p'])
        changed=(e['fu_row'],e['fu_col'])!=(e['rp_row'],e['rp_col'])
        # General sufficient condition against ANY candidate overtaking the FU winner:
        # max possible relative penalization <= lambda * max observed pheromone.
        sufficient=fm > float(e['lambda'])*pmax+1e-9
        # Exact top-two score difference under same lambda, may be <0 even if runner-up not RP winner.
        top2_net=fm-float(e['lambda'])*(winnerp-secondp)
        rows.append(dict(size=size,layout=layout,drones=n,seed=seed,timestep=e['timestep'],drone=e['drone'],candidates=e['candidates'],fu_margin=fm,rp_margin=rm,p_max=pmax,p_positive=int(e['p_positive']),fu_winner_p=winnerp,fu_second_p=secondp,top2_penalty_advantage=float(e['lambda'])*(winnerp-secondp),top2_net_margin=top2_net,global_sufficient_margin=sufficient,shadow_changed=changed))
if not rows: raise SystemExit('No margin observations yet')
with (root/'utility_margin_decisions.csv').open('w',newline='') as f:
 w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
summary=[]
for (size,layout,n),rr in sorted({(r['size'],r['layout'],r['drones']):None for r in rows}.items()):
 x=[r for r in rows if (r['size'],r['layout'],r['drones'])==(size,layout,n)]
 m=[r['fu_margin'] for r in x]
 p=[r for r in x if r['p_positive']>0]
 summary.append(dict(size=size,layout=layout,drones=n,decision_events=len(x),positive_events=len(p),shadow_changed=sum(r['shadow_changed'] for r in x),median_fu_margin=statistics.median(m),min_fu_margin=min(m),max_p=max(r['p_max'] for r in x),sufficient_count=sum(r['global_sufficient_margin'] for r in x),nonzero_fu_winner_p=sum(r['fu_winner_p']>0 for r in x),positive_top2_penalty=sum(r['top2_penalty_advantage']>0 for r in x)))
with (root/'utility_margin_by_group.csv').open('w',newline='') as f:
 w=csv.DictWriter(f,fieldnames=list(summary[0]));w.writeheader();w.writerows(summary)
m=[r['fu_margin'] for r in rows]
print('TOTAL',len(rows),'POSITIVE_EVENTS',sum(r['p_positive']>0 for r in rows),'SHADOW_CHANGES',sum(r['shadow_changed'] for r in rows),'SUFFICIENT_FU_MARGIN',sum(r['global_sufficient_margin'] for r in rows))
print('MEDIAN_MARGIN',statistics.median(m),'MIN_MARGIN',min(m),'MAX_P',max(r['p_max'] for r in rows),'WINNER_P_POS',sum(r['fu_winner_p']>0 for r in rows))
for r in summary:print('GROUP',r)
