"""Generated sparse pairwise constraints, differentiated in body twists."""
import casadi as ca
import numpy as np
from scipy.linalg import null_space
from .kinematics import homogeneous
from .distance import symbolic_point_distance


def vee(R):
    return ca.vertcat(R[2,1]-R[1,2],R[0,2]-R[2,0],R[1,0]-R[0,1])/2


class ConstraintGraph:
    def __init__(self,model,grasps):
        self.model = model
        nb = len(model.bodies)
        x = ca.SX.sym('poses',12*nb)
        u = ca.SX.sym('twist',6*nb)
        poses = {model.root:ca.DM.eye(4)}
        for i,name in enumerate(model.bodies):
            R = ca.reshape(x[12*i:12*i+9],3,3)
            p = x[12*i+9:12*i+12]
            # First-order right perturbation; evaluate at u=0 after differentiation.
            poses[name] = homogeneous(R@(ca.SX.eye(3)+ca.skew(u[6*i:6*i+3])),p+R@u[6*i+3:6*i+6])
        equalities,limits,angles = [],[],{}
        for j,parent,offset in model.edges:
            if j.kind == 'floating': continue
            A, B = poses[parent]@ca.DM(offset),poses[j.child]
            R = A[:3,:3].T@B[:3,:3]
            p = A[:3,:3].T@(B[:3,3]-A[:3,3])
            E = ca.DM(null_space(j.axis.reshape(1,3)))
            if j.kind in ('revolute','continuous'):
                equalities.extend([p,E.T@R@ca.DM(j.axis)])
                # atan2 gives the signed angle about the declared joint axis.
                a, b = E[:,0],E[:,1]
                # E is not necessarily right-handed relative to axis.
                b = ca.cross(ca.DM(j.axis),a)
                angle = ca.atan2(b.T@R@a,a.T@R@a)
            else:
                equalities.extend([E.T@p,vee(R)])
                angle = ca.DM(j.axis).T@p
            angles[j.name] = angle
            if np.isfinite(j.lower): limits.append(angle-j.lower)
            if np.isfinite(j.upper): limits.append(j.upper-angle)
        for j,_,_ in model.edges:
            if j.mimic:
                master,mult,offset = j.mimic
                equalities.append(angles[j.name]-mult*angles[master]-offset)
        self.joint_rows = sum(int(e.numel()) for e in equalities)
        for a,b,relative in grasps:
            ba,oa = model.attachments[a]; bb,ob = model.attachments[b]
            A,B = poses[ba]@ca.DM(oa)@ca.DM(relative),poses[bb]@ca.DM(ob)
            equalities.extend([A[:3,:3].T@(B[:3,3]-A[:3,3]),vee(A[:3,:3].T@B[:3,:3])])
        self.limit_rows = len(limits)
        for sphere in model.collision_spheres:
            body,offset = model.attachments[sphere['link']]
            T = poses[body]@ca.DM(offset)
            center = T[:3,:3]@ca.DM(sphere.get('center',[0,0,0]))+T[:3,3]
            for obstacle in model.obstacles:
                limits.append(symbolic_point_distance(center,obstacle)-sphere['radius'])
        g = ca.vertcat(*equalities) if equalities else ca.SX.zeros(0,1)
        h = ca.vertcat(*limits) if limits else ca.SX.zeros(0,1)
        J,Jh = ca.jacobian(g,u),ca.jacobian(h,u)
        outputs = [ca.substitute(v,u,ca.DM.zeros(6*nb)) for v in [g,J,h,Jh]]
        self.function = ca.Function('constraints',[x],outputs)

    def evaluate(self,X):
        data = np.concatenate([np.r_[X[n][:3,:3].reshape(9,order='F'),X[n][:3,3]] for n in self.model.bodies])
        g,J,h,Jh = [np.asarray(v) for v in self.function(data)]
        return g.ravel(),J,h.ravel(),Jh
