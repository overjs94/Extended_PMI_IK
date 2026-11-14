"""Local research API. No credential, robot-hardware or external mesh access."""
from functools import lru_cache
from threading import Lock
from typing import Literal
from time import perf_counter
import numpy as np
from fastapi import FastAPI,HTTPException
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel,Field
from .scenarios import CATALOG,ROOT,make_scenario
from .kinematics import RobotModel
from .solver import METHODS,Settings,solve_ik

app = FastAPI(title='Extended PMI-IK · Research Workbench',version='0.1.0')
compute_lock = Lock()


class RunRequest(BaseModel):
    scenario: str = Field('dual',min_length=1,max_length=64)
    method: Literal['reduced-osqp','reduced-qpoases','maximal-qp','maximal-skew','maximal-schur','maximal-admm','maximal-drs','maximal-qp-qpoases','maximal-qp-schur','maximal-qp-drs'] = 'maximal-skew'
    seed: int = Field(7,ge=0,le=1_000_000)
    iterations: int = Field(600,ge=1,le=800)
    dt: float = Field(.04,ge=.001,le=.1)
    contraction: float = Field(.25,ge=0,le=1)
    preview: bool = True


class BenchmarkRequest(BaseModel):
    scenario: str = Field('dual',min_length=1,max_length=64)
    seeds: list[int] = Field(default_factory=lambda:[7],min_length=1,max_length=3)


class ImportRequest(BaseModel):
    xml: str = Field(max_length=2_000_000)
    q: list[float] | None = None


class CustomRun(ImportRequest):
    link: str
    target: list[list[float]]
    method: Literal['reduced-osqp','reduced-qpoases','maximal-qp','maximal-skew','maximal-schur','maximal-admm','maximal-drs','maximal-qp-qpoases','maximal-qp-schur','maximal-qp-drs'] = 'reduced-osqp'
    iterations: int = Field(600,ge=1,le=800)


def guarded(fn):
    if not compute_lock.acquire(blocking=False):
        raise HTTPException(409,'A solver is already running. Wait for it to finish.')
    try:
        return fn()
    except (ValueError,KeyError) as exc:
        raise HTTPException(422,str(exc)) from exc
    finally:
        compute_lock.release()


def describe(m,q):
    return dict(name=m.name,xml=m.xml,dof=m.dof,bodies=len(m.bodies),links=m.links,qnames=m.qnames,q=q.tolist(),
                obstacles=m.obstacles,collision_spheres=m.collision_spheres,joints=[dict(parent=j.parent,child=j.child,kind=j.kind) for j in m.joints],visuals=m.visuals,collisions=m.collisions,
                initial={n:T.tolist() for n,T in m.fk(q).items()},mesh_policy='Supplied Panda collision STL and visual DAE. Unresolved custom meshes show frame skeletons; no external fetching.')


@lru_cache(maxsize=12)
def cached_scenario(scenario,seed):
    return make_scenario(scenario,seed)


@app.get('/api/health')
def health():
    return {'status':'ok','qp_backends':['osqp','qpoases'],'gpu':False,'ros':False,'collision_checking':'explicit sphere proxies in obstacle scenario'}


@app.get('/api/catalog')
def catalog():
    return dict(scenarios=CATALOG,methods=METHODS,notice='CPU research reference. Synthetic multi-Panda assemblies. No paper reproduction, collision guarantees or parallel speedup claim.')


@app.get('/api/scenarios/{scenario}')
def scenario_info(scenario: str,seed: int=7):
    def fn():
        m,q,t,g = cached_scenario(scenario,seed)
        source = next((s.get('source') for s in CATALOG if s['id']==scenario),None)
        return {**describe(m,q),'source':source,'targets':{n:T.tolist() for n,T in t.items()},'grasps':[dict(a=a,b=b,relative=T.tolist()) for a,b,T in g]}
    return guarded(fn)


@app.post('/api/solve')
def run(req:RunRequest):
    def fn():
        start = perf_counter()
        m,q,t,g = cached_scenario(req.scenario,req.seed)
        result = solve_ik(m,q,t,g,req.method,Settings(iterations=req.iterations,dt=req.dt,contraction=req.contraction,preview=req.preview))
        result.update(scenario=req.scenario,seed=req.seed,request_compute_ms=(perf_counter()-start)*1000)
        return result
    return guarded(fn)


@app.post('/api/benchmark')
def benchmark(req:BenchmarkRequest):
    def fn():
        if any(s<0 or s>1_000_000 for s in req.seeds): raise ValueError('Invalid seed')
        rows = []
        for seed in req.seeds:
            m,q,t,g = cached_scenario(req.scenario,seed)
            for method in METHODS:
                if m.obstacles and method.startswith('reduced'): continue
                r = solve_ik(m,q,t,g,method)
                rows.append(dict(scenario=req.scenario,seed=seed,method=method,status=r['status'],failure=r['failure'],**r['metrics']))
        return dict(rows=rows,settings=Settings().__dict__,protocol='Serial CPU runs, identical initial state and seed per method; model generation excluded. Single-seed UI runs are exploratory, not publication statistics.')
    return guarded(fn)


@app.post('/api/urdf/inspect')
def inspect_urdf(req:ImportRequest):
    def fn():
        m = RobotModel(req.xml)
        q = m.validate_q(req.q) if req.q is not None else np.clip(np.zeros(m.dof),m.lower,m.upper)
        return describe(m,q)
    return guarded(fn)


@app.post('/api/urdf/solve')
def custom_solve(req:CustomRun):
    def fn():
        m = RobotModel(req.xml)
        if m.dof>70: raise ValueError('Interactive solves are limited to 70 configuration DoF')
        q = m.validate_q(req.q) if req.q is not None else np.clip(np.zeros(m.dof),m.lower,m.upper)
        T = np.asarray(req.target,dtype=float)
        if T.shape!=(4,4) or not np.all(np.isfinite(T)) or not np.allclose(T[3],[0,0,0,1]) or not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-6) or np.linalg.det(T[:3,:3])<.99:
            raise ValueError('Target must be a finite SE(3) matrix')
        if req.link not in m.links: raise ValueError('Unknown target link')
        return solve_ik(m,q,{req.link:T},method=req.method,settings=Settings(iterations=req.iterations))
    return guarded(fn)


app.mount('/models',StaticFiles(directory=ROOT/'models'),name='models')
if (ROOT/'web/dist').exists():
    app.mount('/',StaticFiles(directory=ROOT/'web/dist',html=True),name='web')
