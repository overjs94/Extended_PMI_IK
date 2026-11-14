"""Central maximal QP/VI and reduced PMI reference implementations.

No line search, normal-equation conversion, or pose projection is used in the
maximal methods. The Schur implementation is a CPU reference, not a parallel
speedup claim. Inequality multipliers use lambda_h <= 0 for h >= 0.
"""
from dataclasses import asdict,dataclass
from time import perf_counter
import platform
import numpy as np
from scipy.linalg import block_diag, solve, lstsq
from scipy.spatial.transform import Rotation
from .backends import QPBackend
from .constraints import ConstraintGraph
from .lie import gyro,exp_se3,skew,inverse
from .splitting import ConsensusSolver
from .guidance import PreviewGuide

METHODS = {
    'reduced-osqp':'Reduced PMI / OSQP',
    'reduced-qpoases':'Reduced PMI / qpOASES',
    'maximal-qp':'Maximal QP-PMI / OSQP',
    'maximal-skew':'Maximal Skew-PMI / dense VI',
    'maximal-schur':'Maximal Skew-PMI / block Schur',
    'maximal-admm':'Maximal QP-PMI / body-factor ADMM',
    'maximal-drs':'Maximal Skew-PMI / body-factor DRS',
    'maximal-qp-qpoases':'Maximal QP-PMI / qpOASES',
    'maximal-qp-schur':'Maximal QP-PMI / block Schur',
    'maximal-qp-drs':'Maximal QP-PMI / body-factor DRS',
}


@dataclass
class Settings:
    dt: float = .04
    iterations: int = 600
    position_tolerance: float = 1e-4
    rotation_tolerance: float = 1e-3
    constraint_tolerance: float = 1e-5
    contraction: float = .25
    alpha: float = 1.
    stiffness: float = 120.
    damping: float = 4.
    mass: float = .1
    preview: bool = True


def errors(poses,targets):
    p = max(np.linalg.norm(poses[n][:3,3]-T[:3,3]) for n,T in targets.items())
    r = max(np.linalg.norm(Rotation.from_matrix(T[:3,:3].T@poses[n][:3,:3]).as_rotvec()) for n,T in targets.items())
    return float(p),float(r)


def body_task(model,X,targets):
    n = len(model.bodies)*6
    gradient = np.zeros(n); hessian = np.zeros((n,n)); potential = 0.
    poses = model.link_poses(X)
    for link,desired in targets.items():
        body,offset = model.attachments[link]
        if body == model.root: raise ValueError('Cannot optimize an anchored target')
        i = 6*model.bodies.index(body)
        T = poses[link]; R = T[:3,:3]
        Q = desired[:3,:3].T@R
        er = np.array([Q[2,1]-Q[1,2],Q[0,2]-Q[2,0],Q[1,0]-Q[0,1]])/2
        ep = T[:3,3]-desired[:3,3]
        J = np.zeros((6,6)); J[:3,:3] = offset[:3,:3].T
        J[3:,:3] = -X[body][:3,:3]@skew(offset[:3,3]); J[3:,3:] = X[body][:3,:3]
        gradient[i:i+6] += J.T@np.r_[er,ep]
        hessian[i:i+6,i:i+6] += J.T@J
        potential += .5*(float(ep@ep)+float(np.trace(np.eye(3)-Q)))
    return gradient,hessian,potential


def vi_step(D,b,J,e,Jh,lower,schur=False):
    """Primal-dual active set for Dv + J.T lambda = b, Jh v >= lower."""
    active = []
    total_inner = 0
    for _ in range(80):
        A = np.vstack([J,Jh[active]])
        rhs = np.r_[-e,lower[active]]
        if schur:
            # Every inverse application is an independent 6x6 solve.
            Y = np.vstack([solve(D[i:i+6,i:i+6],np.column_stack([b[i:i+6],A[:,i:i+6].T]),check_finite=False) for i in range(0,len(b),6)])
            z,W = Y[:,0],Y[:,1:]
            S = A@W
            lam = lstsq(S,A@z-rhs,cond=1e-11,check_finite=False)[0] if len(A) else np.zeros(0)
            v = z-W@lam
        else:
            K = np.block([[D,A.T],[A,np.zeros((len(A),len(A)))]])
            try:
                sol = solve(K,np.r_[b,rhs],check_finite=False)
            except np.linalg.LinAlgError:
                # Redundant contacts may make multipliers nonunique. Retain
                # the unique primal solution and check the full KKT residual.
                sol = lstsq(K,np.r_[b,rhs],cond=1e-11,check_finite=False)[0]
            v,lam = sol[:len(b)],sol[len(b):]
        total_inner += 1
        violation = Jh@v-lower
        inactive = [i for i in range(len(lower)) if i not in active]
        if inactive and min(violation[inactive]) < -1e-7:
            active.append(min(inactive,key=lambda i:violation[i])); continue
        if active and max(lam[len(J):]) > 1e-7:
            active.pop(int(np.argmax(lam[len(J):]))); continue
        residual = max(float(np.linalg.norm(D@v + A.T@lam-b,np.inf)),float(np.max(np.abs(A@v-rhs),initial=0)),float(np.max(-violation,initial=0)))
        if residual > 1e-5: raise RuntimeError(f'VI KKT residual {residual:.3g}')
        return v,total_inner,residual
    raise RuntimeError('VI active set did not converge')


def solve_ik(model,q0,targets,grasps=(),method='maximal-skew',settings=None):
    if method not in METHODS: raise ValueError('Unknown method')
    cfg = settings or Settings()
    if not targets: raise ValueError('At least one target is required')
    q = model.validate_q(q0).copy()
    if np.any(q<model.lower) or np.any(q>model.upper): raise ValueError('Initial q violates joint limits')
    initial = model.fk(q)
    X = {n:initial[n].copy() for n in model.bodies}
    setup_started = perf_counter()
    graph = ConstraintGraph(model,grasps)
    reduced = method.startswith('reduced')
    if reduced and model.obstacles:
        raise ValueError('Obstacle scenarios currently require a maximal-coordinate method')
    nv = model.dof if reduced else 6*len(model.bodies)
    velocity = np.zeros(nv)
    G6 = cfg.mass*np.diag([.2,.2,.2,1,1,1])
    G = cfg.mass*np.eye(nv) if reduced else block_diag(*[G6 for _ in model.bodies])
    backend = None
    if reduced:
        # Exact AD of rigid-grasp equalities in reduced joint coordinates.
        import casadi as ca
        z = ca.SX.sym('q',model.dof)
        all_poses = model._fk(z)
        def pose(name):
            i = model.links.index(name); return all_poses[:,4*i:4*i+4]
        eq = []
        for a,b,T in grasps:
            A,B = pose(a)@ca.DM(T),pose(b)
            R = A[:3,:3].T@B[:3,:3]
            eq.extend([A[:3,:3].T@(B[:3,3]-A[:3,3]),ca.vertcat(R[2,1]-R[1,2],R[0,2]-R[2,0],R[1,0]-R[0,1])/2])
        g = ca.vertcat(*eq) if eq else ca.SX.zeros(0,1)
        gf = ca.Function('grasp',[z],[g,ca.jacobian(g,z)])
        backend = QPBackend(method.split('-')[1],nv,int(g.numel())+nv)
    elif method in ('maximal-qp','maximal-qp-qpoases'):
        g,J,h,Jh = graph.evaluate(X)
        backend = QPBackend('qpoases' if method.endswith('qpoases') else 'osqp',nv,len(g)+len(h))
    setup_ms = (perf_counter()-setup_started)*1000
    history,frames = [],[]
    splitting = ConsensusSolver('admm' if method=='maximal-admm' else 'drs') if method in ('maximal-admm','maximal-drs','maximal-qp-drs') else None
    guide = PreviewGuide(model.obstacles,model.collision_spheres[0]['radius']) if cfg.preview and model.obstacles else None
    inner_total = 0; status = 'iteration_limit'; failure = None
    start = perf_counter()
    for iteration in range(cfg.iterations+1):
        poses = model.fk(q) if reduced else model.link_poses(X)
        if reduced: X = {n:poses[n] for n in model.bodies}
        g,Jmax,h,Jhmax = graph.evaluate(X)
        pos,rot = errors(poses,targets)
        joint_res = float(np.max(np.abs(g[:graph.joint_rows]),initial=0))
        grasp_res = float(np.max(np.abs(g[graph.joint_rows:]),initial=0))
        violation = float(np.max(-h[:graph.limit_rows],initial=0))
        clearance = float(np.min(h[graph.limit_rows:])) if len(h)>graph.limit_rows else None
        references = guide.reference(poses,targets) if guide else targets
        grad,taskH,potential = body_task(model,X,references)
        kinetic = float(.5*velocity@G@velocity)
        history.append(dict(iteration=iteration,elapsed_ms=(perf_counter()-start)*1000,position_error_m=pos,rotation_error_rad=rot,
                            joint_residual=joint_res,grasp_residual=grasp_res,joint_limit_violation=violation,min_clearance_m=clearance,
                            preview_active=guide is not None and guide.waypoint is not None,energy=kinetic+cfg.stiffness*potential))
        frames.append({n:T.tolist() for n,T in poses.items()})
        if pos<=cfg.position_tolerance and rot<=cfg.rotation_tolerance and max(joint_res,grasp_res,violation)<=cfg.constraint_tolerance and (clearance is None or clearance>=-cfg.constraint_tolerance):
            status = 'converged'; break
        if iteration == cfg.iterations: break
        try:
            if reduced:
                gradient = np.zeros(nv); taskH = np.zeros((nv,nv))
                for link,desired in targets.items():
                    T = poses[link]; Q = desired[:3,:3].T@T[:3,:3]
                    er = np.array([Q[2,1]-Q[1,2],Q[0,2]-Q[2,0],Q[1,0]-Q[0,1]])/2
                    Jac = model.jacobian(q,link).copy()
                    Jac[:3] = T[:3,:3].T@Jac[:3]
                    gradient += Jac.T@np.r_[er,T[:3,3]-desired[:3,3]]
                    taskH += Jac.T@Jac
                H = 2*G/cfg.dt + cfg.damping*np.eye(nv) + cfg.alpha*cfg.dt*cfg.stiffness*taskH
                b = 2*G@velocity/cfg.dt - cfg.stiffness*gradient
                gr,Jr = [np.asarray(a) for a in gf(q)]; gr = gr.ravel()
                A = np.vstack([Jr,np.eye(nv)])
                lower = np.r_[-cfg.contraction*gr/cfg.dt,(model.lower-q)/cfg.dt]
                upper = np.r_[-cfg.contraction*gr/cfg.dt,(model.upper-q)/cfg.dt]
                mid,inner = backend.solve(H,b,A,lower,upper)
                q += cfg.dt*mid
            else:
                H = 2*G/cfg.dt + cfg.damping*np.eye(nv) + cfg.alpha*cfg.dt*cfg.stiffness*taskH
                b = 2*G@velocity/cfg.dt - cfg.stiffness*grad
                C = np.zeros_like(H) if method.startswith('maximal-qp') or method=='maximal-admm' else block_diag(*[gyro(G6,velocity[i:i+6]) for i in range(0,nv,6)])
                # Small recovery gain corrects second-order manifold drift.
                e = cfg.contraction*g/cfg.dt
                lower = -cfg.contraction*h/cfg.dt
                if backend:
                    mid,inner = backend.solve(H,b,np.vstack([Jmax,Jhmax]),np.r_[-e,lower],np.r_[-e,np.full(len(h),np.inf)])
                elif splitting:
                    mid,inner,kkt = splitting.solve(H+C,b,Jmax,e,Jhmax,lower)
                    history[-1]['inner_residual'] = kkt
                else:
                    mid,inner,kkt = vi_step(H+C,b,Jmax,e,Jhmax,lower,schur=method.endswith('schur'))
                for i,n in enumerate(model.bodies): X[n] = X[n]@exp_se3(cfg.dt*mid[6*i:6*i+6])
                history[-1]['gyro_power'] = float(mid@C@mid)
            velocity = 2*mid-velocity
            inner_total += inner
        except (RuntimeError,np.linalg.LinAlgError) as error:
            status,failure = 'solver_failed',str(error); break
    elapsed = (perf_counter()-start)*1000
    last = history[-1]
    return dict(method=method,method_label=METHODS[method],status=status,failure=failure,
                metrics={**last,'wall_time_ms':elapsed,'setup_time_ms':setup_ms,'iterations':last['iteration'],'inner_iterations':inner_total,
                         'actuated_dof':sum(j.kind in ('revolute','continuous','prismatic') for j in model.joints),'configuration_dof':model.dof,'bodies':len(model.bodies),'variables':nv,
                         'trajectory_min_clearance_m':min((v['min_clearance_m'] for v in history if v['min_clearance_m'] is not None),default=None)},
                history=history,frames=frames,final_q=(q if reduced else model.coordinates(X)).tolist(),qnames=model.qnames,targets={k:v.tolist() for k,v in targets.items()},settings=asdict(cfg),
                provenance={'implementation':'URDF-driven research reference; not original ROS executable','timing':'perf_counter; CPU solve, FK, constraints and frame capture; excludes model/constraint generation and HTTP','parallel':False,'platform':platform.platform(),'python':platform.python_version(),'collision_checking':'declared sphere proxies only' if model.obstacles else False,'guidance':'receding geometric waypoint preview; not full trajectory MPC' if guide else None})
