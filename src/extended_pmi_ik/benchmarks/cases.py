"""Freeze literature-informed test cases; do not label them paper reproductions."""
import hashlib
import json
import numpy as np
from ..scenarios import make_scenario
from ..constraints import ConstraintGraph


def digest(case):
    payload={k:v for k,v in case.items() if k not in ('id','sha256')}
    return hashlib.sha256(json.dumps(payload,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()


def _bounds(model):
    if any(j.kind=='floating' for j in model.joints):raise ValueError('Random global joint-box sampling excludes floating bases')
    lo=np.where(np.isfinite(model.lower),model.lower,-np.pi)
    hi=np.where(np.isfinite(model.upper),model.upper,np.pi)
    return lo,hi


def freeze_case(scenario,recipe,seed):
    model,q,targets,grasps=make_scenario(scenario,seed)
    rng=np.random.default_rng(seed)
    source='project-local extension'
    expected=True;certificate=None
    if recipe in ('reachable-random','joint-limits','near-singular','unreachable'):
        if grasps:raise ValueError('Single-chain recipe must not silently remove closed-loop constraints')
        lo,hi=_bounds(model);q=(lo+hi)/2
        if recipe=='joint-limits':
            target_q=np.where(rng.random(model.dof)>.5,hi-rng.uniform(.002,.02,model.dof)*(hi-lo),lo+rng.uniform(.002,.02,model.dof)*(hi-lo))
        elif recipe=='near-singular':
            q=np.clip(np.zeros(model.dof),lo,hi)
            target_q=np.clip(q+rng.uniform(-.08,.08,model.dof),lo,hi)
        else:target_q=rng.uniform(lo,hi)
        poses=model.fk(target_q);targets={n:poses[n] for n in targets}
        source='trac-ik-2015: reachable FK sampling / joint-limit stress; adapted models and error norms'
        if recipe=='unreachable':
            if any(j.kind=='prismatic' and not np.isfinite([j.lower,j.upper]).all() for j in model.joints):raise ValueError('No bounded reach certificate')
            # A conservative upper bound suffices; rotations cannot increase the
            # sum of translation lengths along the entire tree.
            radius=sum(np.linalg.norm(j.T[:3,3])+(max(abs(j.lower),abs(j.upper)) if j.kind=='prismatic' else 0) for j in model.joints)
            name=next(iter(targets));targets[name]=targets[name].copy();targets[name][:3,3]=[radius+1,0,0]
            expected=False;certificate={'type':'translation-length upper bound','maximum_root_reach_m':float(radius),'target_root_distance_m':float(radius+1)}
    elif recipe=='clutter-boxes':
        if scenario!='panda':raise ValueError('Clutter recipe is currently validated for the supplied Panda')
        source='motionbenchmaker-2022 / curobo: primitive clutter and perturbed queries; synthetic proxy adaptation'
        model.collision_spheres=[dict(link=f'a0_panda_link{i}',center=[0,0,0],radius=.025) for i in range(1,9)]
        initial=model.fk(q);link='a0_panda_link8';start=initial[link][:3,3]
        accepted=False
        for _ in range(80):
            target_q=np.clip(q+np.array([.55,.22,-.2,-.25,.08,.22,.05])+rng.uniform(-.12,.12,model.dof),model.lower,model.upper)
            goal=model.fk(target_q);point=goal[link][:3,3]
            if np.linalg.norm(point-start)<.14:continue
            middle=(start+point)/2
            model.obstacles=[dict(type='box',center=middle.tolist(),half_extents=[.015,.025,.025])]
            for side in (-1,1):
                model.obstacles.append(dict(type='box',center=(middle+np.array([0,side*.20,-.02])).tolist(),half_extents=[.10,.018,.10]))
            graph=ConstraintGraph(model,[])
            clear=[]
            for poses in [initial,goal]:
                _,_,h,_=graph.evaluate({n:poses[n] for n in model.bodies});clear.append(float(np.min(h[graph.limit_rows:])))
            if min(clear)>.002:
                targets={link:goal[link]};accepted=True;break
        if not accepted:raise ValueError('Could not generate collision-free initial/goal states; case rejected before benchmarking')
    elif recipe=='closed-loop':
        if not grasps:raise ValueError('Closed-loop recipe requires hard relative constraints')
        source='pink / mink relative-frame and humanoid-contact examples; project synthetic object grasp extension'
    elif recipe!='nearby':raise ValueError('Unknown recipe')
    case=dict(schema_version=1,scenario=scenario,recipe=recipe,seed=seed,source=source,
              reproduction='literature-informed adaptation, not original paper dataset reproduction',
              urdf_sha256=hashlib.sha256(model.xml.encode()).hexdigest(),q_initial=q.tolist(),
              targets={n:T.tolist() for n,T in targets.items()},grasps=[dict(a=a,b=b,relative=T.tolist()) for a,b,T in grasps],
              obstacles=model.obstacles,collision_spheres=model.collision_spheres,expected_feasible=expected,feasibility_certificate=certificate)
    case['sha256']=digest(case);case['id']=f'{scenario}/{recipe}/{seed}/{case["sha256"][:10]}'
    return case


def thaw_case(case):
    if digest(case)!=case['sha256']:raise ValueError('Frozen case checksum mismatch')
    model,_,_,_=make_scenario(case['scenario'],case['seed'])
    if hashlib.sha256(model.xml.encode()).hexdigest()!=case['urdf_sha256']:raise ValueError('URDF changed since the case was frozen')
    model.obstacles=case['obstacles'];model.collision_spheres=case['collision_spheres']
    q=model.validate_q(case['q_initial'])
    targets={n:np.asarray(T) for n,T in case['targets'].items()}
    grasps=[(g['a'],g['b'],np.asarray(g['relative'])) for g in case['grasps']]
    return model,q,targets,grasps
