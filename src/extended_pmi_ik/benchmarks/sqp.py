"""Actual SciPy SLSQP with exact generated task/constraint derivatives.

This is a conventional constrained NLP control, not TRAC-IK, TrajOpt, Drake
GlobalInverseKinematics, or a reproduction of their algorithms.
"""
from dataclasses import asdict
from time import perf_counter
import casadi as ca
import numpy as np
from scipy.optimize import minimize,Bounds
from ..constraints import ConstraintGraph
from ..solver import errors,closure_errors


def solve_slsqp(model,q0,targets,grasps,cfg):
    setup=perf_counter();graph=ConstraintGraph(model,grasps)
    q=ca.SX.sym('q',model.dof);all_poses=model._fk(q)
    def pose(name):
        i=model.links.index(name);return all_poses[:,4*i:4*i+4]
    x=ca.vertcat(*[ca.vertcat(ca.reshape(pose(n)[:3,:3],9,1),pose(n)[:3,3]) for n in model.bodies])
    g,J,h,Jh=graph.function(x);g=g[graph.joint_rows:]
    cost=0
    for name,T in targets.items():
        A=pose(name);dp=A[:3,3]-ca.DM(T[:3,3])
        cost+=.5*(ca.dot(dp,dp)+ca.trace(ca.DM.eye(3)-ca.DM(T[:3,:3]).T@A[:3,:3]))
    ff=ca.Function('sqp_cost',[q],[cost,ca.gradient(cost,q)])
    gf=ca.Function('sqp_g',[q],[g,ca.jacobian(g,q)])
    hf=ca.Function('sqp_h',[q],[h,ca.jacobian(h,q)])
    setup_ms=(perf_counter()-setup)*1000
    history=[];frames=[];accepted=np.asarray(q0).copy();status='iteration_limit';failure=None
    class Stop(Exception):pass
    start=perf_counter()
    def record(state):
        nonlocal accepted,status
        accepted=np.asarray(state).copy();poses=model.fk(accepted)
        gp,j,hp,jh=graph.evaluate({n:poses[n] for n in model.bodies});pe,re=errors(poses,targets)
        row=dict(iteration=len(history),elapsed_ms=(perf_counter()-start)*1000,position_error_m=pe,rotation_error_rad=re,
                 joint_residual=float(np.max(np.abs(gp[:graph.joint_rows]),initial=0)),grasp_residual=float(np.max(np.abs(gp[graph.joint_rows:]),initial=0)),
                 joint_limit_violation=float(np.max(-hp[:graph.limit_rows],initial=0)),min_clearance_m=float(np.min(hp[graph.limit_rows:])) if len(hp)>graph.limit_rows else None)
        history.append(row)
        if cfg.record_frames:frames.append({n:T.tolist() for n,T in poses.items()})
        if pe<=cfg.position_tolerance and re<=cfg.rotation_tolerance and max(closure_errors(poses,grasps))<=cfg.constraint_tolerance and row['joint_limit_violation']<=cfg.constraint_tolerance and (row['min_clearance_m'] is None or row['min_clearance_m']>=-cfg.constraint_tolerance):
            status='converged';raise Stop
    def deadline():
        nonlocal status
        if cfg.wall_time_limit_s is not None and perf_counter()-start>=cfg.wall_time_limit_s:status='timeout';raise Stop
    def objective(state):deadline();return float(ff(state)[0])
    def jacobian(state):deadline();return np.asarray(ff(state)[1]).ravel()
    def callback(state):record(state);deadline()
    constraints=[]
    if int(g.numel()):constraints.append(dict(type='eq',fun=lambda q:np.asarray(gf(q)[0]).ravel(),jac=lambda q:np.asarray(gf(q)[1])))
    if int(h.numel()):constraints.append(dict(type='ineq',fun=lambda q:np.asarray(hf(q)[0]).ravel(),jac=lambda q:np.asarray(hf(q)[1])))
    try:
        record(q0)
        result=minimize(objective,q0,jac=jacobian,bounds=Bounds(model.lower,model.upper),constraints=constraints,method='SLSQP',callback=callback,
                        options={'maxiter':cfg.iterations,'ftol':1e-12,'disp':False})
        if not np.allclose(accepted,result.x,atol=0,rtol=0):record(result.x)
        if not result.success:failure=result.message
        status='not_converged' if result.success else 'solver_failed'
    except Stop:pass
    except Exception as error:status='solver_failed';failure=f'{type(error).__name__}: {error}'
    return dict(method='scipy-slsqp',method_label='SciPy constrained SLSQP',status=status,failure=failure,
                final_q=accepted.tolist(),qnames=model.qnames,frames=frames,history=history,settings=asdict(cfg),targets={n:T.tolist() for n,T in targets.items()},
                metrics={**history[-1],'wall_time_ms':(perf_counter()-start)*1000,'setup_time_ms':setup_ms,'iterations':max(len(history)-1,0),
                         'trajectory_min_clearance_m':min((r['min_clearance_m'] for r in history if r['min_clearance_m'] is not None),default=None)},
                provenance={'implementation':'scipy.optimize.minimize SLSQP; CasADi exact derivatives; geometric PMI task potential',
                            'constraints':'nonlinear hard relative poses, joint bounds and declared sphere/box inequalities',
                            'preview':False,'parallel':False})
