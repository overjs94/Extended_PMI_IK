import argparse
import csv
import json
from pathlib import Path
from .scenarios import CATALOG,make_scenario
from .kinematics import RobotModel
from .solver import METHODS,Settings,solve_ik


def main():
    parser = argparse.ArgumentParser(description='URDF kinematics generation and reproducible PMI benchmarks')
    sub = parser.add_subparsers(dest='command',required=True)
    gen = sub.add_parser('generate'); gen.add_argument('urdf'); gen.add_argument('--output',default='generated')
    bench = sub.add_parser('benchmark'); bench.add_argument('--scenario',choices=[x['id'] for x in CATALOG],default='dual')
    bench.add_argument('--seeds',type=int,nargs='+',default=[7,11,19]); bench.add_argument('--methods',choices=list(METHODS),nargs='+',default=list(METHODS))
    bench.add_argument('--output',default='results/benchmark.json')
    track=sub.add_parser('track');track.add_argument('--trajectory',default='dual-carry')
    track.add_argument('--plan',help='Contact/EE plan JSON exported from CITO or another planner')
    track.add_argument('--method',default='maximal-schur');track.add_argument('--iterations',type=int,default=600)
    track.add_argument('--kernel',choices=['numpy','numba'],default='numpy');track.add_argument('--workers',type=int,choices=[1,2,4],default=1)
    track.add_argument('--budget',type=float,default=2.);track.add_argument('--output',default='results/tracking.json')
    args = parser.parse_args()
    if args.command == 'generate':
        RobotModel.from_file(args.urdf).export(args.output)
        print(f'Generated C functions, header and manifest in {args.output}')
        return
    if args.command=='track':
        from .trajectories import make_trajectory,solve_trajectory
        m,q,plan,g=make_trajectory(args.trajectory)
        if args.plan:plan=json.loads(Path(args.plan).read_text())
        result=solve_trajectory(m,q,plan,g,args.method,Settings(iterations=args.iterations,wall_time_limit_s=args.budget,splitting_backend=args.kernel,cpu_workers=args.workers))
        path=Path(args.output);path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text(json.dumps(result,allow_nan=False,indent=2)+'\n')
        with path.with_suffix('.csv').open('w') as f:
            fields=[k for k in result['tracking']['knots'][0] if k not in ('q','contacts')]
            writer=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');writer.writeheader();writer.writerows(result['tracking']['knots'])
        print(result['status'],result['tracking']['successes'],'/',result['tracking']['samples'],'knots',flush=True)
        return
    rows = []
    for seed in args.seeds:
        m,q,t,g = make_scenario(args.scenario,seed)
        for method in args.methods:
            if m.obstacles and method.startswith('reduced'): continue
            r = solve_ik(m,q,t,g,method)
            row = dict(scenario=args.scenario,seed=seed,method=method,status=r['status'],failure=r['failure'],**r['metrics'])
            rows.append(row)
            print(f'{method}: {r["status"]}, {row["wall_time_ms"]:.1f} ms, {row["position_error_m"]:.3g} m',flush=True)
    path = Path(args.output); path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(dict(rows=rows,settings=r['settings'],provenance=r['provenance']),indent=2,allow_nan=False)+'\n')
    with path.with_suffix('.csv').open('w') as f:
        writer = csv.DictWriter(f,fieldnames=rows[0].keys(),lineterminator='\n'); writer.writeheader(); writer.writerows(rows)


if __name__ == '__main__': main()
