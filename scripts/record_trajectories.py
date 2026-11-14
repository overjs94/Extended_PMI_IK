#!/usr/bin/env python3
"""Run actual trajectory IK, keeping failed knots and independent timings."""
import argparse
from datetime import datetime,timezone
import csv
import gzip
import json
from pathlib import Path
from extended_pmi_ik.api import describe
from extended_pmi_ik.trajectories import TRAJECTORY_CATALOG,make_trajectory,solve_trajectory
from extended_pmi_ik.solver import Settings
from extended_pmi_ik.benchmarks.runner import environment

ROOT=Path(__file__).resolve().parents[1]
METHODS={'maximal-schur':'Recorded Skew-PMI / Schur','pink-osqp':'Recorded native Pink / OSQP','mink-osqp':'Recorded native Mink / OSQP'}


def save(path,value):
    path.parent.mkdir(exist_ok=True,parents=True)
    path.write_bytes(gzip.compress(json.dumps(value,allow_nan=False,separators=(',',':')).encode(),mtime=0))


def main():
    p=argparse.ArgumentParser();p.add_argument('--overwrite',action='store_true');p.add_argument('--only',nargs='*');args=p.parse_args()
    output=ROOT/'web/recordings';env=environment()
    save(output/'tracking/catalog.json.gz',{'trajectories':TRAJECTORY_CATALOG,'methods':METHODS,'execution':'recorded'})
    for item in TRAJECTORY_CATALOG:
        identifier=item['id']
        if args.only and identifier not in args.only:continue
        model,q,plan,grasps=make_trajectory(identifier)
        (ROOT/'benchmarks/trajectory-inputs').mkdir(exist_ok=True,parents=True)
        (ROOT/'benchmarks/trajectory-inputs'/f'{identifier}.json').write_text(json.dumps(plan,indent=2)+'\n')
        save(output/'tracking/scenarios'/f'{identifier}.json.gz',{**describe(model,q),'plan':plan,'targets':plan['samples'][0]['targets'],'grasps':[{'a':a,'b':b,'relative':T.tolist()} for a,b,T in grasps]})
        for method in METHODS:
            path=output/'tracking/runs'/identifier/f'{method}.json.gz'
            if path.exists() and not args.overwrite:continue
            result=solve_trajectory(model,q,plan,grasps,method,Settings(wall_time_limit_s=2.))
            result['recording']={'measured_at':datetime.now(timezone.utc).isoformat(),'environment':env,'execution':'recorded','frame_policy':'One actual FK configuration per input time knot; no pose interpolation; not certified robot motion'}
            save(path,result)
            print(identifier,method,result['status'],result['tracking']['successes'],'/',result['tracking']['samples'],'max_mm',result['tracking']['max_position_error_m']*1000,flush=True)
    rows=[];details=[]
    for item in TRAJECTORY_CATALOG:
        for method in METHODS:
            path=output/'tracking/runs'/item['id']/f'{method}.json.gz'
            if not path.exists():continue
            r=json.loads(gzip.decompress(path.read_bytes()));t=r['tracking']
            rows.append({'trajectory':item['id'],'method':method,'status':r['status'],**{k:t[k] for k in ('successes','samples','median_knot_ms','p95_knot_ms','rms_position_error_m','max_position_error_m','max_contact_position_error_m','interval_misses')},'wall_time_ms':r['metrics']['wall_time_ms'],'input_provenance':t['plan']['provenance']['kind'],'plan_sha256':t['plan_sha256']})
            details.extend({'trajectory':item['id'],'method':method,**k} for k in t['knots'])
    reports=ROOT/'benchmarks/reports'
    (reports/'tracking.json').write_text(json.dumps({'summary':rows,'knots':details,'notice':'Offline per-knot IK; authored example contacts are not an executed CITO planner result'},indent=2,allow_nan=False)+'\n')
    with (reports/'tracking.csv').open('w') as f:
        writer=csv.DictWriter(f,fieldnames=rows[0].keys(),lineterminator="\n");writer.writeheader();writer.writerows(rows)


if __name__=='__main__':main()
