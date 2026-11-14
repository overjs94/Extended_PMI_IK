"""Separate within-query CPU-thread speed from independent-query throughput."""
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict
import argparse
import json
import multiprocessing as mp
import os
from pathlib import Path
import random
from statistics import median
from time import perf_counter
import numpy as np
from scipy.linalg import block_diag
from ..constraints import ConstraintGraph
from ..lie import gyro
from ..scenarios import make_scenario
from ..solver import Settings,body_task,vi_step
from ..splitting import ConsensusSolver
from .cases import freeze_case
from .runner import environment,run_case


def linearized_problem(scenario,kind):
    model,q,targets,grasps=make_scenario(scenario,7)
    poses=model.fk(q);X={n:poses[n] for n in model.bodies}
    graph=ConstraintGraph(model,grasps);g,J,h,Jh=graph.evaluate(X)
    grad,taskH,_=body_task(model,X,targets);nb=len(model.bodies)
    G6=.1*np.diag([.2,.2,.2,1,1,1]);G=block_diag(*[G6 for _ in range(nb)])
    predictor=np.random.default_rng(11).normal(0,.03,6*nb)
    H=2*G/.04+4*np.eye(6*nb)+.04*120*taskH
    C=np.zeros_like(H) if kind=='admm' else block_diag(*[gyro(G6,predictor[6*i:6*i+6]) for i in range(nb)])
    return H+C,-120*grad,J,.25*g/.04,Jh,-.25*h/.04


def kernel_benchmark(scenarios,workers,repeats):
    rows=[]
    for scenario in scenarios:
        for kind in ['admm','drs']:
            problem=linearized_problem(scenario,kind)
            try:expected,_,_=vi_step(*problem,schur=True)
            except Exception:expected=None
            variants=[('numpy',1)]+[('numba',w) for w in workers]
            for backend,w in variants:
                # Warm compilation and every code path before timing. Restore a
                # cold solver iterate each repetition (no solution warm start).
                try:ConsensusSolver(kind,backend=backend,workers=w).solve(*problem)
                except RuntimeError:pass
            jobs=[(backend,w,repeat) for repeat in range(repeats) for backend,w in variants]
            random.Random(7).shuffle(jobs)
            local=[]
            for backend,w,repeat in jobs:
                solver=ConsensusSolver(kind,backend=backend,workers=w)
                start=perf_counter();failure=None;status='converged';iterations=residual=error=None
                try:
                    v,iterations,residual=solver.solve(*problem)
                    error=float(np.max(np.abs(v-expected))) if expected is not None else None
                    if error is not None and error>1e-5:status='parity_failed'
                except RuntimeError as exc:status='inner_limit';failure=str(exc)
                row=dict(scenario=scenario,kind=kind,backend=backend,workers=w,repeat=repeat,status=status,failure=failure,
                         wall_time_ms=(perf_counter()-start)*1000,iterations=iterations,residual=residual,reference_error_inf=error,variables=len(problem[1]))
                rows.append(row);local.append(row)
            for backend,w in variants:
                group=[r for r in local if r['backend']==backend and r['workers']==w]
                print(scenario,kind,backend,w,'median ms',round(median(r['wall_time_ms'] for r in group),3),'success',sum(r['status']=='converged' for r in group),flush=True)
    summary=[]
    for scenario in scenarios:
        for kind in ['admm','drs']:
            for backend,w in [('numpy',1)]+[('numba',w) for w in workers]:
                group=[r for r in rows if (r['scenario'],r['kind'],r['backend'],r['workers'])==(scenario,kind,backend,w)]
                summary.append(dict(scenario=scenario,kind=kind,backend=backend,workers=w,repeats=len(group),converged=sum(r['status']=='converged' for r in group),
                                    median_ms=median(r['wall_time_ms'] for r in group),variables=group[0]['variables']))
    return dict(scope='One frozen linearized midpoint problem per URDF, not end-to-end IK. Compilation and warmup excluded; inverse/incidence setup and inner solve included.',
                workers_meaning='Real Numba CPU threads for body updates, factor projections and CSR consensus. NumPy is the serial vectorized reference.',
                environment=environment(),rows=rows,summary=summary)


_METHOD=None
_CFG=None


def _initialize(method,cfg,warm_case,ready):
    global _METHOD,_CFG
    os.environ['OPENBLAS_NUM_THREADS']='1';os.environ['NUMBA_NUM_THREADS']='1'
    _METHOD=method;_CFG=cfg
    run_case(warm_case,method,cfg,warmup=True)
    ready.put(os.getpid())


def _job(case):
    row=run_case(case,_METHOD,_CFG,warmup=False)
    return dict(case_id=case['id'],case_sha256=case['sha256'],pid=os.getpid(),status=row['status'],solve_wall_time_ms=row['wall_time_ms'],
                assessment=row.get('assessment'),failure=row['failure'])


def _ping():return os.getpid()


def batch_benchmark(workers,queries,method):
    cases=[freeze_case('panda','nearby',seed) for seed in range(queries)]
    cfg=Settings(iterations=600,wall_time_limit_s=5.,record_frames=False,preview=False,strict_closure_check=True)
    context=mp.get_context('spawn');runs=[]
    for w in workers:
        ready=context.Queue();tick=perf_counter()
        with ProcessPoolExecutor(max_workers=w,mp_context=context,initializer=_initialize,initargs=(method,cfg,cases[0],ready)) as pool:
            # Trigger every worker and wait for all initializers to finish their
            # warmup; do not count process creation/imports as steady-state work.
            futures=[pool.submit(_ping) for _ in range(w)]
            pids=[ready.get(timeout=180) for _ in range(w)]
            for future in futures:future.result()
            startup=(perf_counter()-tick)*1000
            start=perf_counter();rows=list(pool.map(_job,cases,chunksize=1));elapsed=(perf_counter()-start)*1000
        successes=sum(r['status']=='success' for r in rows)
        result=dict(workers=w,pool_initialization_ms=startup,batch_wall_time_ms=elapsed,queries=len(cases),successes=successes,
                    successful_queries_per_second=successes/(elapsed/1000),all_queries_per_second=len(cases)/(elapsed/1000),
                    worker_pids=pids,observed_job_pids=sorted({r['pid'] for r in rows}),rows=rows)
        runs.append(result);print('batch workers',w,'wall ms',round(elapsed,1),'successes',successes,'pids',result['observed_job_pids'],flush=True)
    one=next((r['batch_wall_time_ms'] for r in runs if r['workers']==1),None)
    for r in runs:r['batch_wall_speedup_vs_one_process']=one/r['batch_wall_time_ms'] if one else None
    return dict(scope='Independent IK-query process parallelism; throughput, not single-query latency or body-level parallel speedup.',
                method=method,settings=asdict(cfg),input_sha256=[c['sha256'] for c in cases],environment=environment(),runs=runs)


def main():
    p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='mode',required=True)
    kernel=sub.add_parser('kernels');kernel.add_argument('--scenarios',nargs='+',default=['panda','dual','quad','unitree-g1','ergocub']);kernel.add_argument('--repeats',type=int,default=5)
    batch=sub.add_parser('batch');batch.add_argument('--queries',type=int,default=24);batch.add_argument('--method',default='maximal-schur')
    for parser in [kernel,batch]:
        parser.add_argument('--workers',type=int,nargs='+',default=[1,2,4]);parser.add_argument('--output',required=True)
    args=p.parse_args()
    if any(w<1 for w in args.workers):p.error('Worker counts must be positive')
    if args.mode=='kernels':result=kernel_benchmark(args.scenarios,args.workers,args.repeats)
    else:result=batch_benchmark(args.workers,args.queries,args.method)
    path=Path(args.output);path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')


if __name__=='__main__':main()
