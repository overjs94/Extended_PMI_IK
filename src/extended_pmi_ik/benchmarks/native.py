"""Execute actual Pink/Mink libraries on the same URDF kinematic problem.

Adapters implement coordinate conversion and the declared sphere/box inequality
rows. They do not replace either external library's IK/QP implementation.
"""
from dataclasses import asdict
from time import perf_counter
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from ..constraints import ConstraintGraph
from ..distance import point_distance_gradient
from ..guidance import PreviewGuide
from ..lie import inverse,skew
from ..solver import errors,closure_errors


class UnsupportedProblem(ValueError):pass


class NativeIK:
    def __init__(self,backend,model,q0,targets,grasps,cfg):
        self.kind,self.robot,self.cfg=backend,model,cfg
        self.targets,self.grasps=targets,grasps
        if any(j.mimic for j in model.joints):
            raise UnsupportedProblem('Native mimic-joint coupling conversion is not yet validated')
        self.active=[j for j in model.joints if j.index>=0]
        self.tasks,self.constraints=[],[]
        if backend=='pink':self._pink(q0)
        elif backend=='mink':self._mink(q0)
        else:raise ValueError('Unknown native backend')
        self.update_targets(targets)
        # Hard relative tasks preserve object grasps and anchored feet.
        for a,b,T in grasps:
            if backend=='pink':
                task=self.library.tasks.RelativeFrameTask(b,a,position_cost=1.,orientation_cost=1.,gain=cfg.contraction)
            else:
                task=self.library.RelativeFrameTask(b,self.frame_type(b),a,self.frame_type(a),position_cost=1.,orientation_cost=1.,gain=cfg.contraction)
            task.set_target(self.se3(T));self.constraints.append(task)
        self.check_parity(q0)
        # Exercise nonzero joint values as well, not only a zero-angle conversion.
        probe=np.asarray(q0).copy()
        for j in self.active:
            if j.kind!='floating':probe[j.index]=np.clip(probe[j.index]+.013,j.lower,j.upper)
        self.check_parity(probe)
        self.configuration.update(self.to_native(q0))

    def _pink(self,q0):
        import pink
        import pinocchio as pin
        self.library,self.engine=pink,pin
        root=ET.fromstring(self.robot.xml)
        for link in root.findall('link'):
            for tag in ['visual','collision']:
                for node in list(link.findall(tag)):link.remove(node)
        self.model=pin.buildModelFromXML(ET.tostring(root,encoding='unicode'))
        self.indices={j.name:self.model.getJointId(j.name) for j in self.active}
        self.configuration=pink.Configuration(self.model,self.model.createData(),self.to_native(q0))
        self.limits=[pink.limits.ConfigurationLimit(self.model,config_limit_gain=self.cfg.contraction)]

    def _mink(self,q0):
        import mink
        import mujoco
        self.library,self.engine=mink,mujoco
        root=ET.Element('mujoco',model=self.robot.name)
        ET.SubElement(root,'compiler',angle='radian',autolimits='true',fusestatic='false')
        world=ET.SubElement(root,'worldbody')
        ET.SubElement(world,'site',name=self.robot.root,pos='0 0 0',size='.001')
        bodies={self.robot.root:world}
        for j in self.robot.joints:
            quat=Rotation.from_matrix(j.T[:3,:3]).as_quat()[[3,0,1,2]]
            body=ET.SubElement(bodies[j.parent],'body',name=j.child,pos=' '.join(map(str,j.T[:3,3])),quat=' '.join(map(str,quat)))
            bodies[j.child]=body
            # These inertias only make a valid kinematic MuJoCo model; no
            # physical dynamics are evaluated by the differential IK baseline.
            ET.SubElement(body,'inertial',pos='0 0 0',mass='.1',diaginertia='.01 .01 .01')
            if j.kind=='fixed':continue
            if j.kind=='floating':
                if j.parent!=self.robot.root:raise UnsupportedProblem('Nested floating joints are not validated by the MuJoCo converter')
                ET.SubElement(body,'freejoint',name=j.name);continue
            attrs=dict(name=j.name,type='slide' if j.kind=='prismatic' else 'hinge',axis=' '.join(map(str,j.axis)))
            if np.isfinite(j.lower) and np.isfinite(j.upper):attrs['range']=f'{j.lower} {j.upper}'
            else:attrs['limited']='false'
            ET.SubElement(body,'joint',**attrs)
        self.model=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))
        self.indices={j.name:mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_JOINT,j.name) for j in self.active}
        self.configuration=mink.Configuration(self.model,self.to_native(q0))
        self.limits=[mink.ConfigurationLimit(self.model,gain=self.cfg.contraction)]

    def frame_type(self,name):return 'site' if name==self.robot.root else 'body'

    def se3(self,T):
        return self.engine.SE3(T[:3,:3],T[:3,3]) if self.kind=='pink' else self.library.SE3.from_matrix(T)

    def to_native(self,q):
        x=self.engine.neutral(self.model) if self.kind=='pink' else self.model.qpos0.copy()
        poses=None
        for j in self.active:
            i=self.indices[j.name]
            if self.kind=='pink':
                if i==0:raise UnsupportedProblem(f'Pinocchio did not preserve joint {j.name}')
                start,size=self.model.idx_qs[i],self.model.nqs[i]
            else:start,size=self.model.jnt_qposadr[i],7 if j.kind=='floating' else 1
            if j.kind=='floating':
                if self.kind=='pink':
                    x[start:start+3]=q[j.index:j.index+3]
                    x[start+3:start+7]=Rotation.from_euler('xyz',q[j.index+3:j.index+6]).as_quat()
                else:
                    if poses is None:poses=self.robot.fk(q)
                    T=poses[j.child];x[start:start+3]=T[:3,3];x[start+3:start+7]=Rotation.from_matrix(T[:3,:3]).as_quat()[[3,0,1,2]]
            elif size==2:x[start:start+2]=[np.cos(q[j.index]),np.sin(q[j.index])]
            else:x[start]=q[j.index]
        return x

    def from_native(self):
        x=np.array(self.configuration.q,copy=True)
        q=np.zeros(self.robot.dof)
        for j in self.active:
            i=self.indices[j.name]
            if self.kind=='pink':start,size=self.model.idx_qs[i],self.model.nqs[i]
            else:start,size=self.model.jnt_qposadr[i],7 if j.kind=='floating' else 1
            if j.kind=='floating':
                if self.kind=='pink':q[j.index:j.index+6]=np.r_[x[start:start+3],Rotation.from_quat(x[start+3:start+7]).as_euler('xyz')]
                else:
                    T=np.eye(4);T[:3,3]=x[start:start+3];T[:3,:3]=Rotation.from_quat(x[start+3:start+7][[1,2,3,0]]).as_matrix()
                    relative=inverse(j.T)@T;q[j.index:j.index+6]=np.r_[relative[:3,3],Rotation.from_matrix(relative[:3,:3]).as_euler('xyz')]
            elif size==2:q[j.index]=np.arctan2(x[start+1],x[start])
            else:q[j.index]=x[start]
        return q

    def pose(self,name):
        if self.kind=='pink':return self.configuration.get_transform_frame_to_world(name).homogeneous
        return self.configuration.get_transform_frame_to_world(name,self.frame_type(name)).as_matrix()

    def jacobian(self,name):
        if self.kind=='pink':return self.configuration.get_frame_jacobian(name)
        return self.configuration.get_frame_jacobian(name,self.frame_type(name))

    def check_parity(self,q):
        self.configuration.update(self.to_native(q))
        for name,T in self.robot.fk(q).items():
            if not np.allclose(self.pose(name),T,atol=2e-8):
                raise UnsupportedProblem(f'{self.kind} coordinate/FK mismatch at {name}; benchmark rejected')
        if not np.allclose(self.from_native(),q,atol=2e-8):raise UnsupportedProblem('Joint coordinate round trip failed')

    def update_targets(self,targets):
        if not self.tasks:
            for name in targets:
                task=(self.library.tasks.FrameTask(name,1.,1.,gain=.5,lm_damping=1e-3) if self.kind=='pink' else
                      self.library.FrameTask(name,self.frame_type(name),1.,1.,gain=.5,lm_damping=1e-3))
                self.tasks.append(task)
        for task,T in zip(self.tasks,targets.values()):task.set_target(self.se3(T))

    def distance_rows(self):
        h,J=[],[]
        for sphere in self.robot.collision_spheres:
            T=self.pose(sphere['link']);R=T[:3,:3];point=np.asarray(sphere.get('center',[0,0,0]))
            center=T[:3,3]+R@point
            frameJ=self.jacobian(sphere['link'])
            pointJ=R@(frameJ[:3]-skew(point)@frameJ[3:])
            for obstacle in self.robot.obstacles:
                d,n=point_distance_gradient(center,obstacle)
                h.append(d-sphere['radius']);J.append(n@pointJ)
        return np.asarray(h),np.asarray(J)

    def step(self):
        owner=self
        class DistanceLimit:
            def compute_qp_objective(self,configuration):return np.zeros((owner.model.nv,owner.model.nv)),np.zeros(owner.model.nv)
            def compute_qp_inequalities(self,configuration,dt):
                h,J=owner.distance_rows();G,rhs=-J,owner.cfg.contraction*h
                if owner.kind=='pink':return G,rhs
                from mink.limits import Constraint
                return Constraint(G,rhs)
        collision=[DistanceLimit()] if self.robot.obstacles else []
        kwargs=dict(solver='osqp',damping=1e-6,constraints=self.constraints,eps_abs=1e-8,eps_rel=1e-8,max_iter=10000,verbose=False)
        if self.kind=='pink':kwargs.update(limits=self.limits,barriers=collision)
        else:kwargs.update(limits=[*self.limits,*collision],safety_break=True)
        velocity=self.library.solve_ik(self.configuration,self.tasks,self.cfg.dt,**kwargs)
        self.configuration.integrate_inplace(velocity,self.cfg.dt)


def solve_native(backend,model,q0,targets,grasps,cfg):
    setup=perf_counter();native=NativeIK(backend,model,q0,targets,grasps,cfg)
    graph=ConstraintGraph(model,grasps)
    setup_ms=(perf_counter()-setup)*1000
    history,frames=[],[];status='iteration_limit';failure=None;native_ms=0.
    guide=PreviewGuide(model.obstacles,model.collision_spheres[0]['radius']) if cfg.preview and model.obstacles else None
    start=perf_counter()
    for iteration in range(cfg.iterations+1):
        q=native.from_native();poses=model.fk(q);X={n:poses[n] for n in model.bodies}
        g,J,h,Jh=graph.evaluate(X);pos,rot=errors(poses,targets)
        row=dict(iteration=iteration,elapsed_ms=(perf_counter()-start)*1000,position_error_m=pos,rotation_error_rad=rot,
                 joint_residual=float(np.max(np.abs(g[:graph.joint_rows]),initial=0)),grasp_residual=float(np.max(np.abs(g[graph.joint_rows:]),initial=0)),
                 joint_limit_violation=float(np.max(-h[:graph.limit_rows],initial=0)),min_clearance_m=float(np.min(h[graph.limit_rows:])) if len(h)>graph.limit_rows else None)
        history.append(row)
        if cfg.record_frames:frames.append({n:T.tolist() for n,T in poses.items()})
        closure_ok=not cfg.strict_closure_check or max(closure_errors(poses,grasps))<=cfg.constraint_tolerance
        if pos<=cfg.position_tolerance and rot<=cfg.rotation_tolerance and max(row['joint_residual'],row['grasp_residual'],row['joint_limit_violation'])<=cfg.constraint_tolerance and closure_ok and (row['min_clearance_m'] is None or row['min_clearance_m']>=-cfg.constraint_tolerance):
            status='converged';break
        if iteration==cfg.iterations:break
        if cfg.wall_time_limit_s is not None and perf_counter()-start>=cfg.wall_time_limit_s:status='timeout';break
        try:
            references=guide.reference(poses,targets) if guide else targets
            tick=perf_counter();native.update_targets(references);native.step();native_ms+=(perf_counter()-tick)*1000
        except Exception as error:status='solver_failed';failure=f'{type(error).__name__}: {error}';break
    return dict(method=f'{backend}-osqp',method_label=f'{backend.capitalize()} native / OSQP',status=status,failure=failure,
                final_q=native.from_native().tolist(),qnames=model.qnames,frames=frames,history=history,settings=asdict(cfg),targets={n:T.tolist() for n,T in targets.items()},
                metrics={**history[-1],'wall_time_ms':(perf_counter()-start)*1000,'setup_time_ms':setup_ms,'native_step_time_ms':native_ms,'iterations':history[-1]['iteration'],
                         'trajectory_min_clearance_m':min((r['min_clearance_m'] for r in history if r['min_clearance_m'] is not None),default=None)},
                provenance={'implementation':f'actual {backend} library; native FK checked against common URDF at initial and perturbed q',
                            'collision_checking':'shared explicit sphere/axis-aligned-box rows, not full-mesh collision checking',
                            'native_task_gain':.5,'native_task_lm_damping':1e-3,'qp_damping':1e-6,'velocity_limits':False,'parallel':False})
