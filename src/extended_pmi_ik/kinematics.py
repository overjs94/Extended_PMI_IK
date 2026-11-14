"""Parse URDF once and generate CasADi FK and exact geometric Jacobians.

Fixed links are merged into maximal bodies while retaining their visual frames.
Floating joints use xyz + roll/pitch/yaw for reduced-coordinate initialization only.
"""
from dataclasses import dataclass
from pathlib import Path
import casadi as ca
import numpy as np
from defusedxml import ElementTree as ET
from defusedxml.common import DefusedXmlException
from xml.etree.ElementTree import ParseError
from .lie import transform


def numbers(s, default='0 0 0'):
    x = np.array([float(v) for v in (s or default).split()])
    if len(x) != 3 or not np.all(np.isfinite(x)):
        raise ValueError('Expected three finite numbers')
    return x


def origin(node):
    o = node.find('origin')
    return np.eye(4) if o is None else transform(numbers(o.get('xyz')), numbers(o.get('rpy')))


def rotation(axis, q):
    W = ca.skew(ca.DM(axis))
    return ca.DM.eye(3) + ca.sin(q)*W + (1-ca.cos(q))*(W@W)


def homogeneous(R, p):
    return ca.vertcat(ca.horzcat(R, p), ca.DM([[0, 0, 0, 1]]))


@dataclass
class Joint:
    name: str
    kind: str
    parent: str
    child: str
    T: np.ndarray
    axis: np.ndarray
    lower: float
    upper: float
    index: int = -1
    mimic: tuple | None = None


class RobotModel:
    def __init__(self, xml: str):
        if len(xml.encode()) > 2_000_000:
            raise ValueError('URDF exceeds 2 MB')
        try:
            root = ET.fromstring(xml)
        except (ParseError,DefusedXmlException) as exc:
            raise ValueError(f'Invalid or unsafe URDF XML: {exc}') from exc
        if root.tag != 'robot':
            raise ValueError('Expected a URDF <robot>')
        self.xml, self.name = xml, root.get('name', 'robot')
        self.obstacles, self.collision_spheres = [], []
        self.links = [n.get('name') for n in root.findall('link')]
        if not self.links or not all(self.links) or len(self.links) > 240 or len(set(self.links)) != len(self.links):
            raise ValueError('URDF requires 1–240 uniquely named links')
        self.visuals, self.collisions = [], []
        for link in root.findall('link'):
            for kind,collection in [('visual',self.visuals),('collision',self.collisions)]:
                for v in link.findall(kind):
                    geom = v.find('geometry')
                    if geom is None or not len(geom): continue
                    g = geom[0]
                    attrs = {k:value for k,value in g.attrib.items() if k!='type'}
                    material = v.find('material')
                    if material is not None:
                        color = material.find('color')
                        if color is None:
                            definition = next((m for m in root.findall('material') if m.get('name')==material.get('name')),None)
                            if definition is not None:color = definition.find('color')
                        if color is not None:attrs['rgba'] = [float(a) for a in color.get('rgba').split()]
                    collection.append(dict(link=link.get('name'), origin=origin(v).tolist(), type=g.tag, **attrs))
        joints = []
        for n in root.findall('joint'):
            if not n.get('name') or n.find('parent') is None or n.find('child') is None:
                raise ValueError('Every joint needs a name, parent and child')
            kind = n.get('type')
            if kind not in ('fixed', 'revolute', 'continuous', 'prismatic', 'floating'):
                raise ValueError(f'Unsupported joint type: {kind}')
            axis_node, limit = n.find('axis'), n.find('limit')
            axis = numbers(axis_node.get('xyz') if axis_node is not None else '1 0 0')
            if kind in ('fixed','floating'):axis = np.array([1.,0,0])
            if np.linalg.norm(axis) < 1e-12:
                raise ValueError('Joint axis cannot be zero')
            axis /= np.linalg.norm(axis)
            lo = float(limit.get('lower', '-inf')) if limit is not None else -np.inf
            hi = float(limit.get('upper', 'inf')) if limit is not None else np.inf
            if kind == 'continuous': lo, hi = -np.inf, np.inf
            if np.isnan(lo) or np.isnan(hi) or lo > hi:
                raise ValueError('Invalid joint limits')
            j = Joint(n.get('name'), kind, n.find('parent').get('link'), n.find('child').get('link'), origin(n), axis, lo, hi)
            mimic = n.find('mimic')
            if mimic is not None:
                if kind not in ('revolute','continuous','prismatic'):
                    raise ValueError('Only scalar joints may mimic another joint')
                j.mimic = (mimic.get('joint'),float(mimic.get('multiplier','1')),float(mimic.get('offset','0')))
            joints.append(j)
        if len({j.name for j in joints}) != len(joints):
            raise ValueError('Duplicate joint names')
        children = [j.child for j in joints]
        roots = set(self.links)-set(children)
        if len(roots) != 1 or len(set(children)) != len(children) or any(j.parent not in self.links or j.child not in self.links for j in joints):
            raise ValueError('URDF must be one connected, rooted tree')
        self.root = roots.pop()
        known, ordered = {self.root}, []
        while joints:
            ready = [j for j in joints if j.parent in known]
            if not ready:
                raise ValueError('Cycle or disconnected URDF')
            for j in ready:
                known.add(j.child); ordered.append(j); joints.remove(j)
        self.joints = ordered
        self.dof = 0
        self.lower, self.upper, self.qnames = [], [], []
        for j in ordered:
            if j.kind != 'fixed' and j.mimic is None:
                j.index = self.dof
                size = 6 if j.kind == 'floating' else 1
                self.dof += size
                self.lower.extend([j.lower] if size == 1 else [-np.inf]*6)
                self.upper.extend([j.upper] if size == 1 else [np.inf]*6)
                self.qnames.extend([j.name] if size == 1 else [f'{j.name}/{a}' for a in ['x','y','z','roll','pitch','yaw']])
        self.lower, self.upper = np.array(self.lower), np.array(self.upper)
        q = ca.SX.sym('q', self.dof)
        values,byname = {},{j.name:j for j in ordered}
        def affine(j,stack=()):
            if j.name in stack:raise ValueError('Cyclic mimic joints')
            if not j.mimic:return j.index,1.,0.
            name,mult,offset=j.mimic
            if name not in byname or byname[name].kind not in ('revolute','continuous','prismatic'):
                raise ValueError('Mimic master must be a scalar movable joint')
            index,a,c=affine(byname[name],(*stack,j.name))
            return index,mult*a,mult*c+offset
        for j in ordered:
            if not j.mimic:continue
            index,a,c=affine(j)
            if abs(a)<1e-14:
                if not j.lower<=c<=j.upper:raise ValueError('Constant mimic value violates limits')
                continue
            lo,hi=sorted(((j.lower-c)/a,(j.upper-c)/a))
            self.lower[index]=max(self.lower[index],lo);self.upper[index]=min(self.upper[index],hi)
        if np.any(self.lower>self.upper):raise ValueError('Inconsistent mimic joint limits')
        def coordinate(j,stack=()):
            if j.name in values:return values[j.name]
            if j.name in stack:raise ValueError('Cyclic mimic joints')
            if j.mimic:
                name,mult,offset = j.mimic
                if name not in byname or byname[name].kind not in ('revolute','continuous','prismatic'):
                    raise ValueError('Mimic master must be a scalar movable joint')
                values[j.name] = mult*coordinate(byname[name],(*stack,j.name))+offset
            else:values[j.name] = q[j.index]
            return values[j.name]
        poses = {self.root: ca.DM.eye(4)}
        # (body name, fixed body-to-link offset). Root is anchored and not a variable.
        self.attachments = {self.root: (self.root, np.eye(4))}
        self.bodies, self.edges = [], []
        for j in ordered:
            motion = ca.SX.eye(4)
            if j.kind in ('revolute', 'continuous'):
                motion[:3, :3] = rotation(j.axis, coordinate(j))
            elif j.kind == 'prismatic':
                motion[:3, 3] = ca.DM(j.axis)*coordinate(j)
            elif j.kind == 'floating':
                a = q[j.index:j.index+6]
                motion = homogeneous(rotation([0,0,1],a[5])@rotation([0,1,0],a[4])@rotation([1,0,0],a[3]),a[:3])
            poses[j.child] = poses[j.parent] @ ca.DM(j.T) @ motion
            body, offset = self.attachments[j.parent]
            if j.kind == 'fixed':
                self.attachments[j.child] = (body, offset@j.T)
            else:
                self.bodies.append(j.child)
                self.attachments[j.child] = (j.child, np.eye(4))
                self.edges.append((j, body, offset@j.T))
        self._fk = ca.Function('fk', [q], [ca.horzcat(*[poses[n] for n in self.links])])
        self._jac = {}
        angular = {}
        for name, T in poses.items():
            dp = ca.jacobian(T[:3,3], q)
            body,_ = self.attachments[name]
            if body not in angular:
                R = poses[body][:3,:3]
                dR = ca.jacobian(ca.reshape(R,9,1),q)
                columns = []
                for i in range(self.dof):
                    W = ca.reshape(dR[:,i],3,3)@R.T
                    columns.append(ca.vertcat(W[2,1],W[0,2],W[1,0]))
                angular[body] = ca.horzcat(*columns) if columns else ca.SX.zeros(3,0)
            dw = angular[body]
            self._jac[name] = ca.Function('jacobian', [q], [ca.vertcat(dw,dp)])

    @classmethod
    def from_file(cls, path):
        return cls(Path(path).read_text())

    def fk(self, q):
        q = self.validate_q(q)
        p = np.asarray(self._fk(q))
        return {n:p[:,4*i:4*i+4] for i,n in enumerate(self.links)}

    def jacobian(self, q, link):
        return np.asarray(self._jac[link](self.validate_q(q)))

    def validate_q(self, q):
        q = np.asarray(q, dtype=float)
        if q.shape != (self.dof,) or not np.all(np.isfinite(q)):
            raise ValueError(f'q must contain {self.dof} finite values')
        return q

    def link_poses(self, X):
        return {n:(np.eye(4) if b == self.root else X[b])@offset for n,(b,offset) in self.attachments.items()}

    def coordinates(self,X):
        """Recover diagnostic q from maximal poses (no pose projection)."""
        from scipy.spatial.transform import Rotation
        from .lie import inverse
        q = np.zeros(self.dof)
        for j,parent,offset in self.edges:
            if j.mimic: continue
            A = (np.eye(4) if parent==self.root else X[parent])@offset
            relative = inverse(A)@X[j.child]
            if j.kind in ('revolute','continuous'):
                q[j.index] = Rotation.from_matrix(relative[:3,:3]).as_rotvec()@j.axis
            elif j.kind=='prismatic': q[j.index] = relative[:3,3]@j.axis
            elif j.kind=='floating': q[j.index:j.index+6] = np.r_[relative[:3,3],Rotation.from_matrix(relative[:3,:3]).as_euler('xyz')]
        return q

    def export(self, directory):
        """Generate standalone C FK/Jacobian functions + headers (no ROS needed)."""
        import os
        directory = Path(directory).resolve(); directory.mkdir(parents=True, exist_ok=True)
        previous = Path.cwd()
        try:
            os.chdir(directory)
            gen = ca.CodeGenerator('kinematics.c', {'with_header':True})
            gen.add(self._fk)
            # Unique names for independently callable generated Jacobians.
            for i,(name,f) in enumerate(self._jac.items()):
                q = ca.SX.sym('q',self.dof)
                gen.add(ca.Function(f'jac_{i}', [q], [f(q)]))
            gen.generate()
            import json
            (directory/'manifest.json').write_text(json.dumps({'links':self.links,'qnames':self.qnames,'jacobians':{n:f'jac_{i}' for i,n in enumerate(self._jac)}},indent=2))
        finally:
            os.chdir(previous)
