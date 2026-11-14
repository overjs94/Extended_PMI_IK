"""Timed EE-pose/contact inputs and measured, configuration-warm-started IK.

This layer consumes contact plans; it does not claim to implement a CITO planner,
balance control or a dynamics simulation. Each knot performs an actual IK solve.
"""
from dataclasses import asdict,replace
from hashlib import sha256
from time import perf_counter
import json
import xml.etree.ElementTree as ET
import numpy as np
from .kinematics import RobotModel
from .lie import transform
from .scenarios import ROOT,make_scenario
from .solver import Settings,METHODS,solve_ik,errors,closure_errors

TRAJECTORY_CATALOG=[
    {'id':'lasdra-lemniscate','name':'LASDRA / 3D lemniscate tracking','description':'21 DoF · source-derived kinematics · adapted Cartesian lemniscate'},
    {'id':'dual-carry','name':'Dual Panda / object trajectory','description':'Two rigid grasps · translating and rotating object'},
    {'id':'quad-carry','name':'Four-arm / object trajectory','description':'Four rigid grasps · coupled object tracking'},
    {'id':'g1-contact-walk','name':'G1 / contact-sequence walking IK','description':'Floating base + 29 joints · authored contact/EE input example; CITO result import supported'},
]
NATIVE_METHODS={'pink-osqp':'Native Pink / OSQP','mink-osqp':'Native Mink / OSQP'}


def se3(value):
    T=np.asarray(value,dtype=float)
    if T.shape!=(4,4) or not np.all(np.isfinite(T)) or not np.allclose(T[3],[0,0,0,1],atol=1e-9) or not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-7) or not np.isclose(np.linalg.det(T[:3,:3]),1,atol=1e-7):
        raise ValueError('Trajectory poses must be finite SE(3) matrices')
    return T


def validate_plan(model,plan):
    if plan.get('version')!=1:raise ValueError('Expected contact-EE plan version 1')
    provenance=plan.get('provenance',{})
    if provenance.get('kind') not in ('authored_example','external_cito_output','external_trajectory'):
        raise ValueError('Declare authored_example, external_cito_output or external_trajectory provenance')
    if provenance['kind']=='external_cito_output' and not provenance.get('source'):
        raise ValueError('CITO results require their actual source or run identifier')
    samples=plan.get('samples',[])
    if not 2<=len(samples)<=300:raise ValueError('Trajectory requires 2–300 sampled knots')
    previous=-1.
    for sample in samples:
        if not isinstance(sample,dict) or not isinstance(sample.get('targets'),dict) or not isinstance(sample.get('contacts',{}),dict):raise ValueError('Targets and contacts must map link names to poses')
        t=float(sample['time_s'])
        if not np.isfinite(t) or t<0 or t<=previous:raise ValueError('Trajectory times must be finite, nonnegative and strictly increasing')
        previous=t
        if not sample.get('targets'):raise ValueError('Each knot requires at least one EE/body target')
        for group in ('targets','contacts'):
            for name,T in sample.get(group,{}).items():
                if name not in model.links or name==model.root:raise ValueError('Unknown or anchored trajectory frame: '+name)
                se3(T)
        for name,T in sample.get('contacts',{}).items():
            if name in sample['targets'] and not np.allclose(T,sample['targets'][name],atol=1e-9):raise ValueError('Contact pose conflicts with its target')
    # An active stance contact cannot silently slide between consecutive knots.
    for a,b in zip(samples,samples[1:]):
        for name in a.get('contacts',{}).keys()&b.get('contacts',{}).keys():
            if not np.allclose(a['contacts'][name],b['contacts'][name],atol=1e-9):raise ValueError('Active contact moved without a release: '+name)
    return plan


def floating_g1():
    model,q,_,_=make_scenario('unitree-g1')
    root=ET.fromstring(model.xml);root.set('name','Unitree_G1_floating_contact_tracking')
    ET.SubElement(root,'link',name='trajectory_world')
    joint=ET.SubElement(root,'joint',name='trajectory_base',type='floating')
    ET.SubElement(joint,'parent',link='trajectory_world');ET.SubElement(joint,'child',link=model.root)
    robot=RobotModel(ET.tostring(root,encoding='unicode'))
    x=np.clip(np.zeros(robot.dof),robot.lower,robot.upper)
    for j in robot.joints:
        if j.kind=='revolute':
            if 'hip_pitch' in j.name:x[j.index]=-.25
            if 'knee' in j.name:x[j.index]=.5
            if 'ankle_pitch' in j.name:x[j.index]=-.25
    poses=robot.fk(x)
    x[2]=.035-min(poses[n][2,3] for n in ('left_ankle_roll_link','right_ankle_roll_link'))
    return robot,x,model.root


def make_trajectory(identifier):
    if identifier=='g1-contact-walk':
        model,q,pelvis=floating_g1();poses=model.fk(q)
        left,right='left_ankle_roll_link','right_ankle_roll_link'
        feet={n:poses[n].copy() for n in (left,right)}
        samples=[{'time_s':0.,'targets':{pelvis:poses[pelvis].tolist(),**{n:T.tolist() for n,T in feet.items()}},'contacts':{n:T.tolist() for n,T in feet.items()},'phase':'double support'}]
        stride=.06
        for step,swing in enumerate((left,right)):
            stance=right if swing==left else left;start=feet[swing].copy()
            distance=stride if step==0 else 2*stride
            for k,u in enumerate((.25,.5,.75,1.)):
                T=start.copy();T[0,3]+=distance*u;T[2,3]+=.035*np.sin(np.pi*u)
                if u==1:T[2,3]=start[2,3]
                root=poses[pelvis].copy();root[0,3]=stride*(step+u)/2
                targets={pelvis:root.tolist(),stance:feet[stance].tolist(),swing:T.tolist()}
                contacts={stance:feet[stance].tolist()}
                if u==1:contacts[swing]=T.tolist();feet[swing]=T.copy()
                samples.append({'time_s':.5*len(samples),'targets':targets,'contacts':contacts,'phase':('double support / touchdown' if u==1 else swing+' swing')})
        provenance={'kind':'authored_example','source':'Repository-authored short-step contact/EE input, not an executed CITO optimizer','requires_external_cito_data':True,'scope':'Kinematic reconstruction only; no balance, forces, full-body collision or dynamics certification'}
        grasps=[]
    else:
        scenario={'lasdra-lemniscate':'lasdra','dual-carry':'dual','quad-carry':'quad'}.get(identifier)
        if scenario is None:raise ValueError('Unknown trajectory')
        model,q,_,grasps=make_scenario(scenario,7);poses=model.fk(q)
        link='lasdra_ee' if scenario=='lasdra' else 'object';start=poses[link]
        samples=[]
        for k,theta in enumerate(np.linspace(0,2*np.pi,17)):
            if scenario=='lasdra':
                T=start.copy();T[:3,3]+=np.array([.03*(np.cos(2*theta)-1),.10*np.sin(theta),.10*np.sin(theta)*np.cos(theta)])
            else:T=transform([.025*np.sin(theta),.02*(1-np.cos(theta)),.015*np.sin(2*theta)],[.02*np.sin(theta),.015*np.sin(theta),.03*np.sin(theta)])@start
            samples.append({'time_s':k*.25,'targets':{link:T.tolist()},'contacts':{},'phase':'rigid object tracking' if scenario!='lasdra' else 'Cartesian lemniscate'})
        provenance={'kind':'authored_example','source':'Adapted synthetic tracking task; LASDRA axes/lengths from supplied C++, not a reproduction of the original obstacle trial'}
    plan={'version':1,'id':identifier,'provenance':provenance,'samples':samples}
    return model,q,validate_plan(model,plan),grasps


def solve_trajectory(model,q0,plan,grasps=(),method='maximal-schur',settings=None):
    validate_plan(model,plan)
    cfg=replace(settings or Settings(),record_frames=False,strict_closure_check=True)
    if method not in METHODS and method not in NATIVE_METHODS:raise ValueError('Unknown trajectory solver')
    if cfg.splitting_backend!='numpy' and method not in ('maximal-admm','maximal-drs','maximal-qp-drs'):raise ValueError('Compiled kernels apply to ADMM/DRS methods')
    if method in NATIVE_METHODS:
        import importlib.util
        if importlib.util.find_spec(method.split('-')[0]) is None:raise ValueError('Native baseline unavailable; run bash scripts/setup_benchmarks.sh')
    q=model.validate_q(q0).copy();frames=[];history=[];knots=[];targets_by_frame=[]
    wall_start=perf_counter()
    for index,sample in enumerate(plan['samples']):
        started=perf_counter();targets={n:se3(T) for n,T in sample['targets'].items()}
        contacts=[(model.root,n,se3(T)) for n,T in sample.get('contacts',{}).items()]
        active=[*grasps,*contacts];failure=None
        try:
            if method in NATIVE_METHODS:
                from .benchmarks.native import solve_native
                result=solve_native(method.split('-')[0],model,q,targets,active,cfg)
            else:result=solve_ik(model,q,targets,active,method,cfg)
            candidate=np.asarray(result['final_q'])
            if not np.all(np.isfinite(candidate)):raise ValueError('Nonfinite configuration')
            q=candidate;outcome=result['status'];failure=result['failure'];loop_ms=result['metrics']['wall_time_ms'];iterations=result['metrics']['iterations']
            if cfg.wall_time_limit_s is not None and loop_ms>cfg.wall_time_limit_s*1000:outcome='timeout'
        except (ValueError,RuntimeError,np.linalg.LinAlgError) as exc:
            outcome='solver_failed';failure=str(exc);loop_ms=None;iterations=0
        poses=model.fk(q);position,rotation=errors(poses,targets);cp,cr=closure_errors(poses,active)
        limit=float(max(np.max(model.lower-q,initial=0),np.max(q-model.upper,initial=0)))
        endpoint_ok=position<=cfg.position_tolerance and rotation<=cfg.rotation_tolerance and max(cp,cr,limit)<=cfg.constraint_tolerance
        if outcome=='converged' and not endpoint_ok:outcome='validation_failed'
        elapsed=(perf_counter()-started)*1000
        interval=None if index==0 else (sample['time_s']-plan['samples'][index-1]['time_s'])*1000
        row={'iteration':index,'time_s':sample['time_s'],'elapsed_ms':(perf_counter()-wall_start)*1000,'position_error_m':position,'rotation_error_rad':rotation,'joint_residual':0.,'grasp_residual':max(cp,cr),'joint_limit_violation':limit,'energy':None,'min_clearance_m':None}
        history.append(row);frames.append({n:T.tolist() for n,T in poses.items()});targets_by_frame.append(sample['targets'])
        knots.append({'index':index,'time_s':sample['time_s'],'phase':sample.get('phase',''),'contacts':list(sample.get('contacts',{})),'status':outcome,'failure':failure,'validated':bool(endpoint_ok),'wall_time_ms':elapsed,'loop_time_ms':loop_ms,'iterations':iterations,'position_error_m':position,'rotation_error_rad':rotation,'contact_position_error_m':cp,'contact_rotation_error_rad':cr,'joint_limit_violation':limit,'sample_interval_ms':interval,'exceeds_sample_interval':interval is not None and elapsed>interval,'q':q.tolist()})
    durations=[k['wall_time_ms'] for k in knots];successes=sum(k['status']=='converged' for k in knots)
    total=(perf_counter()-wall_start)*1000
    return {'method':method,'method_label':NATIVE_METHODS.get(method,METHODS.get(method,method)),'status':'converged' if successes==len(knots) else 'tracking_incomplete','failure':None if successes==len(knots) else f'{len(knots)-successes} knots did not converge or failed validation',
            'metrics':{**history[-1],'wall_time_ms':total,'iterations':len(knots),'inner_iterations':sum(k['iterations'] for k in knots),'trajectory_min_clearance_m':None},
            'history':history,'frames':frames,'frame_times_s':[s['time_s'] for s in plan['samples']],'frame_targets':targets_by_frame,'targets':targets_by_frame[-1],'final_q':q.tolist(),'qnames':model.qnames,'settings':asdict(cfg),
            'tracking':{'knots':knots,'successes':successes,'samples':len(knots),'median_knot_ms':float(np.median(durations)),'p95_knot_ms':float(np.percentile(durations,95)),'rms_position_error_m':float(np.sqrt(np.mean([k['position_error_m']**2 for k in knots]))),'max_position_error_m':max(k['position_error_m'] for k in knots),'max_contact_position_error_m':max(k['contact_position_error_m'] for k in knots),'interval_misses':sum(k['exceeds_sample_interval'] for k in knots),'plan':plan,'plan_sha256':sha256(json.dumps(plan,sort_keys=True).encode()).hexdigest()},
            'provenance':{'implementation':'Actual per-knot IK, previous joint configuration warm start; virtual velocity and inner splitting state restart at each knot','timing':'Knot times include setup, solver and endpoint FK validation; total includes frame serialization','execution':'computed','scope':'Offline sampled kinematics; no continuous collision, forces, balance or physical trajectory guarantee'}}
