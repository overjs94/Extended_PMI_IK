#!/usr/bin/env python3
"""Measure solver runs for the public read-only Pages viewer.

This deliberately executes the actual solver. It is not used during Pages
publication: publishing copies the already measured, versioned records.
"""
import argparse
from dataclasses import asdict
from datetime import datetime,timezone
import gzip
import json
from pathlib import Path
import subprocess
import numpy as np
from extended_pmi_ik.api import describe
from extended_pmi_ik.scenarios import CATALOG,make_scenario
from extended_pmi_ik.solver import Settings,solve_ik,errors
from extended_pmi_ik.benchmarks.native import solve_native
from extended_pmi_ik.benchmarks.runner import environment,score

ROOT=Path(__file__).resolve().parents[1]
METHODS={'maximal-schur':'Recorded Skew-PMI / Schur','pink-osqp':'Recorded native Pink / OSQP','mink-osqp':'Recorded native Mink / OSQP'}


def save(path,data):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_bytes(gzip.compress(json.dumps(data,separators=(',',':'),allow_nan=False).encode(),mtime=0))


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--scenarios',nargs='*');parser.add_argument('--overwrite',action='store_true');args=parser.parse_args()
    output=ROOT/'web/recordings'
    catalog={'scenarios':CATALOG,'methods':METHODS,'execution':'recorded','seed':7,'notice':'Public viewer replays measured runs; it does not run the optimizer. Recordings have a 5-second loop budget and may show timeouts or failures.'}
    save(output/'catalog.json.gz',catalog)
    cfg=Settings(iterations=600,wall_time_limit_s=5.,record_frames=True,preview=True,strict_closure_check=True)
    env=environment()
    for item in CATALOG:
        scenario=item['id']
        if args.scenarios and scenario not in args.scenarios:continue
        model,q,targets,grasps=make_scenario(scenario,7)
        metadata={**describe(model,q),'source':item.get('source'),'targets':{n:T.tolist() for n,T in targets.items()},'grasps':[dict(a=a,b=b,relative=T.tolist()) for a,b,T in grasps]}
        save(output/'scenarios'/f'{scenario}.json.gz',metadata)
        for method in METHODS:
            path=output/'runs'/scenario/f'{method}.json.gz'
            if path.exists() and not args.overwrite:continue
            try:
                if method=='maximal-schur':result=solve_ik(model,q,targets,grasps,method,cfg)
                else:result=solve_native(method.split('-')[0],model,q,targets,grasps,cfg)
                assessment=score(model,result['final_q'],targets,grasps,cfg)
                if result['status']=='converged' and not assessment['endpoint_feasible']:
                    result['status']='validation_failed';result['failure']='Independent endpoint FK/constraint validation rejected the result'
            except Exception as error:
                poses=model.fk(q);p,r=errors(poses,targets);assessment=score(model,q,targets,grasps,cfg)
                row=dict(iteration=0,position_error_m=p,rotation_error_rad=r,joint_residual=0.,grasp_residual=max(assessment['closure_position_m'],assessment['closure_rotation_rad']),joint_limit_violation=assessment['joint_limit_violation'],energy=None,elapsed_ms=0.)
                result=dict(method=method,method_label=METHODS[method],status='unsupported',failure=f'{type(error).__name__}: {error}',
                            final_q=q.tolist(),qnames=model.qnames,settings=asdict(cfg),metrics={**row,'iterations':0,'wall_time_ms':None,'trajectory_min_clearance_m':None},
                            frames=[{n:T.tolist() for n,T in poses.items()}],history=[row],targets={n:T.tolist() for n,T in targets.items()},provenance={'implementation':'No successful adapter execution; initial configuration only'})
            count=len(result['frames']);indices=sorted(set(np.linspace(0,count-1,min(72,count),dtype=int).tolist()))
            result['frames']=[result['frames'][i] for i in indices];result['frame_iterations']=indices
            result['recording']={'measured_at':datetime.now(timezone.utc).isoformat(),'seed':7,'scenario':scenario,'environment':env,
                                 'original_frame_count':count,'stored_frames':len(indices),'frame_policy':'Actual sampled solver states, no interpolation; virtual solver flow is not physical robot dynamics',
                                 'timing_policy':'Recorded loop time includes original frame capture. Not interchangeable with frame-disabled pilot timings.',
                                 'endpoint_assessment':assessment}
            save(path,result)
            print(scenario,method,result['status'],result['metrics']['wall_time_ms'],len(result['frames']),flush=True)


if __name__=='__main__':main()
