"""Matched-input CPU benchmark with explicit unsupported/unrun outcomes.

Time limits are checked at solver iteration boundaries. Any successful result
over the deadline is counted as a timeout; overshoot is retained, not hidden.
"""
from collections import defaultdict
from dataclasses import asdict,replace
from datetime import datetime,timezone
import argparse
import csv
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import random
import subprocess
from time import perf_counter
import numpy as np
from ..scenarios import ROOT
from ..solver import METHODS,Settings,solve_ik,errors,closure_errors
from ..constraints import ConstraintGraph
from .cases import freeze_case,thaw_case
from .native import solve_native,UnsupportedProblem
from .sqp import solve_slsqp

NATIVE={'pink-osqp','mink-osqp','scipy-slsqp'}
DEFAULT_METHODS=['reduced-osqp','maximal-qp-schur','maximal-schur','maximal-admm','maximal-drs','pink-osqp','mink-osqp','scipy-slsqp']


def environment():
    packages={}
    for name in ['numpy','scipy','osqp','casadi','pin-pink','pin','mink','mujoco','qpsolvers']:
        try:packages[name]=importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:packages[name]=None
    try:commit=subprocess.check_output(['git','-C',str(ROOT),'rev-parse','HEAD'],text=True,stderr=subprocess.DEVNULL).strip()
    except subprocess.CalledProcessError:commit=None
    source_hash=hashlib.sha256()
    for path in sorted((ROOT/'src/extended_pmi_ik').rglob('*.py')):
        source_hash.update(str(path.relative_to(ROOT)).encode());source_hash.update(path.read_bytes())
    cpu=next((line.split(':',1)[1].strip() for line in Path('/proc/cpuinfo').read_text().splitlines() if line.startswith('model name')),'unknown') if Path('/proc/cpuinfo').exists() else platform.processor()
    quota=Path('/sys/fs/cgroup/cpu.max').read_text().split() if Path('/sys/fs/cgroup/cpu.max').exists() else ['max','1']
    return dict(python=platform.python_version(),platform=platform.platform(),cpu=cpu,packages=packages,
                cpu_quota_cores=None if quota[0]=='max' else int(quota[0])/int(quota[1]),affinity_cpus=len(os.sched_getaffinity(0)) if hasattr(os,'sched_getaffinity') else os.cpu_count(),
                blas_threads=os.environ.get('OPENBLAS_NUM_THREADS','unspecified'),cuda_devices=[p.name for p in Path('/dev').glob('nvidia[0-9]*')],
                git_commit=commit,source_sha256=source_hash.hexdigest())


def score(model,q,targets,grasps,cfg):
    q=model.validate_q(q);poses=model.fk(q)
    p,r=errors(poses,targets);gp,gr=closure_errors(poses,grasps)
    graph=ConstraintGraph(model,grasps)
    g,J,h,Jh=graph.evaluate({n:poses[n] for n in model.bodies})
    limit=float(np.max(-h[:graph.limit_rows],initial=0))
    clearance=float(np.min(h[graph.limit_rows:])) if len(h)>graph.limit_rows else None
    ok=p<=cfg.position_tolerance and r<=cfg.rotation_tolerance and gp<=cfg.constraint_tolerance and gr<=cfg.constraint_tolerance and limit<=cfg.constraint_tolerance and (clearance is None or clearance>=-cfg.constraint_tolerance)
    return dict(endpoint_feasible=bool(ok),position_error_m=p,rotation_error_rad=r,closure_position_m=gp,closure_rotation_rad=gr,joint_limit_violation=limit,min_clearance_m=clearance)


def dispatch(method,model,q,t,g,cfg):
    if '-numba-' in method:
        base,workers=method.split('-numba-')
        if base not in ('maximal-admm','maximal-drs','maximal-qp-drs'):raise UnsupportedProblem('Only ADMM/DRS have compiled body/factor kernels')
        return solve_ik(model,q,t,g,base,replace(cfg,splitting_backend='numba',cpu_workers=int(workers)))
    if method in METHODS:return solve_ik(model,q,t,g,method,cfg)
    if method in ('pink-osqp','mink-osqp'):return solve_native(method.split('-')[0],model,q,t,g,cfg)
    if method=='scipy-slsqp':
        if cfg.preview and model.obstacles:raise UnsupportedProblem('SLSQP does not implement the time-varying preview objective')
        return solve_slsqp(model,q,t,g,cfg)
    raise UnsupportedProblem('No validated adapter for this method; no proxy solver is substituted')


def run_case(case,method,cfg,warmup=True):
    row=dict(case_id=case['id'],case_sha256=case['sha256'],scenario=case['scenario'],recipe=case['recipe'],seed=case['seed'],method=method,
             expected_feasible=case['expected_feasible'],status='adapter_error',failure=None,wall_time_ms=None,endpoint_feasible=False,within_budget=False)
    try:
        model_start=perf_counter()
        model,q,t,g=thaw_case(case)
        row.update(model_setup_time_ms=(perf_counter()-model_start)*1000,configuration_dof=model.dof,bodies=len(model.bodies))
        if method.startswith('reduced') and model.obstacles:raise UnsupportedProblem('Reduced PMI adapter does not yet enforce obstacle rows')
        if warmup:
            # Warm libraries/one step, then discard every solver state and reset
            # to the frozen initial q for the measured solve.
            dispatch(method,model,q.copy(),t,g,replace(cfg,iterations=1,wall_time_limit_s=None,record_frames=False))
        result=dispatch(method,model,q.copy(),t,g,cfg)
        row.update(result['metrics']);row.update(reported_status=result['status'],failure=result['failure'],provenance=result['provenance'])
        row['within_budget']=cfg.wall_time_limit_s is None or row['wall_time_ms']<=cfg.wall_time_limit_s*1000
        row['budget_overshoot_ms']=max(0,row['wall_time_ms']-(cfg.wall_time_limit_s or np.inf)*1000)
        row['assessment']=score(model,result['final_q'],t,g,cfg)
        row['endpoint_feasible']=row['assessment']['endpoint_feasible']
        if not row['within_budget']:row['status']='timeout'
        elif row['endpoint_feasible']:row['status']='success'
        elif result['status']=='converged':row['status']='validation_failed'
        else:row['status']=result['status']
        clearances=[r.get('min_clearance_m') for r in result['history'] if r.get('min_clearance_m') is not None]
        row['sampled_proxy_path_clear']=None if not clearances else min(clearances)>=-cfg.constraint_tolerance
        # Final independent FK is mandatory. Iterates are solver iterations, not
        # a certified executable trajectory, and are scored separately.
        row['final_q']=result['final_q']
    except (ImportError,ModuleNotFoundError) as error:row.update(status='unavailable',failure=str(error))
    except UnsupportedProblem as error:row.update(status='unsupported',failure=str(error))
    except Exception as error:row.update(status='adapter_error',failure=f'{type(error).__name__}: {error}')
    return row


def summaries(rows):
    groups=defaultdict(list)
    for row in rows:groups[(row['recipe'],row['method'])].append(row)
    result=[]
    for (recipe,method),group in sorted(groups.items()):
        eligible=[r for r in group if r['status'] not in ('unsupported','unavailable') and r['expected_feasible']]
        successful=[r for r in eligible if r['status']=='success']
        times=[r['wall_time_ms'] for r in successful]
        alltimes=[r['wall_time_ms'] for r in eligible if r['wall_time_ms'] is not None]
        result.append(dict(recipe=recipe,method=method,cases=len(group),eligible_feasible_cases=len(eligible),successes=len(successful),
                           success_rate=len(successful)/len(eligible) if eligible else None,
                           median_success_ms=float(np.median(times)) if times else None,p95_success_ms=float(np.percentile(times,95)) if times else None,
                           median_all_attempts_ms=float(np.median(alltimes)) if alltimes else None,
                           timeouts=sum(r['status']=='timeout' for r in group),solver_failures=sum(r['status'] in ('solver_failed','adapter_error','validation_failed') for r in group),
                           unsupported=sum(r['status']=='unsupported' for r in group),unavailable=sum(r['status']=='unavailable' for r in group),
                           infeasible_control_cases=sum(not r['expected_feasible'] for r in group)))
    return result


def write_report(output,report):
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    columns=['case_id','scenario','recipe','seed','method','expected_feasible','status','wall_time_ms','iterations','endpoint_feasible','within_budget','budget_overshoot_ms','failure']
    with output.with_suffix('.csv').open('w',newline='') as file:
        writer=csv.DictWriter(file,fieldnames=columns,extrasaction='ignore',lineterminator='\n');writer.writeheader();writer.writerows(report['rows'])
    lines=['# Native-baseline pilot benchmark','',
           'Literature-informed synthetic/adapted cases, not a reproduction of published paper performance. No overall SOTA ranking. Native software baselines are distinct from peer-reviewed algorithms.','',
           f"Settings: {report['settings']}",'',
           '| Recipe | Method | Success / eligible feasible | Median success (ms) | p95 success (ms) | Timeouts | Unsupported |',
           '|---|---|---:|---:|---:|---:|---:|']
    for s in report['summary']:
        median='—' if s['median_success_ms'] is None else f"{s['median_success_ms']:.1f}"
        p95='—' if s['p95_success_ms'] is None else f"{s['p95_success_ms']:.1f}"
        lines.append(f"| {s['recipe']} | {s['method']} | {s['successes']} / {s['eligible_feasible_cases']} | {median} | {p95} | {s['timeouts']} | {s['unsupported']} |")
    lines+=['','Timing is end-to-end iteration-loop time including common FK/constraint diagnostics; model conversion and setup are recorded separately. No frame serialization is timed. Successful runs exceeding the budget are timeouts. Inner calls can overshoot the boundary; overshoot is recorded.',
            '', 'Do not rank methods by median-success latency without their success counts. Unsupported cases are explicitly marked, never relaxed or counted as successes. Negative unreachable controls are separate from feasible-case success rates. No sampled solver path is claimed as a continuous-time collision-safe motion plan.']
    output.with_suffix('.md').write_text('\n'.join(lines)+'\n')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='command',required=True)
    generate=sub.add_parser('generate');generate.add_argument('--spec',default='benchmarks/pilot-spec.json');generate.add_argument('--output',default='benchmarks/cases/pilot.json')
    run=sub.add_parser('run');run.add_argument('--cases',default='benchmarks/cases/pilot.json');run.add_argument('--output',default='benchmarks/reports/pilot.json')
    run.add_argument('--methods',nargs='+',default=DEFAULT_METHODS);run.add_argument('--budget',type=float,default=1.0);run.add_argument('--iterations',type=int,default=600)
    run.add_argument('--resume',action='store_true');run.add_argument('--no-warmup',action='store_true')
    args=parser.parse_args()
    if args.command=='generate':
        spec=json.loads(Path(args.spec).read_text());cases=[]
        for item in spec['cases']:
            for seed in item['seeds']:
                case=freeze_case(item['scenario'],item['recipe'],seed);cases.append(case);print('frozen',case['id'],flush=True)
        path=Path(args.output);path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(dict(spec=spec,cases=cases),indent=2)+'\n');return
    if args.budget<=0 or args.iterations<1:parser.error('Budget and iterations must be positive')
    frozen=json.loads(Path(args.cases).read_text());cases=frozen['cases'];output=Path(args.output);output.parent.mkdir(parents=True,exist_ok=True)
    cfg=Settings(iterations=args.iterations,wall_time_limit_s=args.budget,record_frames=False,preview=False,strict_closure_check=True)
    env=environment();run_key=hashlib.sha256(json.dumps(dict(settings=asdict(cfg),source=env['source_sha256'],packages=env['packages'],methods=args.methods,cases=[c['sha256'] for c in cases]),sort_keys=True).encode()).hexdigest()
    checkpoint=output.with_suffix('.jsonl');rows=[]
    if args.resume and checkpoint.exists():
        for line in checkpoint.read_text().splitlines():
            record=json.loads(line)
            if record['run_key']!=run_key:raise ValueError('Resume rejected: code, versions, settings or cases changed')
            rows.append(record['row'])
    else:checkpoint.write_text('')
    done={(r['case_id'],r['method']) for r in rows}
    report=dict(schema_version=1,scope='CPU matched-case pilot; literature adaptations, not original dataset reproduction',run_key=run_key,
                generated_at=datetime.now(timezone.utc).isoformat(),settings=asdict(cfg),environment=env,cases=cases,rows=rows,summary=summaries(rows))
    for case in cases:
        methods=list(args.methods);random.Random(case['seed']).shuffle(methods)
        for method in methods:
            if (case['id'],method) in done:continue
            row=run_case(case,method,cfg,warmup=not args.no_warmup);rows.append(row)
            with checkpoint.open('a') as file:file.write(json.dumps(dict(run_key=run_key,row=row),allow_nan=False)+'\n')
            report['summary']=summaries(rows);write_report(output,report)
            print(f"{len(rows)}/{len(cases)*len(args.methods)} {case['scenario']} {case['recipe']} {case['seed']} {method}: {row['status']} {row['wall_time_ms']}",flush=True)


if __name__=='__main__':main()
