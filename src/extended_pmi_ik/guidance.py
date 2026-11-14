"""Receding geometric look-ahead; a reference generator, not full MPC.

The internal PMI solver still solves instantaneous IK with h >= 0. Preview
waypoints break the straight-line local minimum without changing the VI/QP.
Safety is assessed separately by actual sphere clearances after integration.
"""
import numpy as np
from .distance import bounding_radius


def segment_clearance(a,b,center,radius):
    d = b-a
    s = np.clip(np.dot(center-a,d)/max(np.dot(d,d),1e-14),0,1)
    return float(np.linalg.norm(a+s*d-center)-radius)


class PreviewGuide:
    def __init__(self,obstacles,sphere_radius=.025):
        self.obstacles,self.radius = obstacles,sphere_radius
        self.waypoint = None

    def reference(self,poses,targets):
        link = next(iter(targets)); final = targets[link]
        start,goal = poses[link][:3,3],final[:3,3]
        blocking = [o for o in self.obstacles if segment_clearance(start,goal,np.array(o['center']),bounding_radius(o)+self.radius+.012)<0]
        if not blocking:
            self.waypoint = None
            return targets
        if self.waypoint is None:
            o = blocking[0]; c = np.array(o['center']); radius = bounding_radius(o)+self.radius+.035
            direction = goal-start; direction /= max(np.linalg.norm(direction),1e-12)
            basis = np.cross(direction,[0,0,1])
            if np.linalg.norm(basis)<.1: basis = np.cross(direction,[0,1,0])
            basis /= np.linalg.norm(basis)
            second = np.cross(direction,basis)
            choices = []
            for angle in np.linspace(0,2*np.pi,16,endpoint=False):
                candidate = c+1.6*radius*(np.cos(angle)*basis+np.sin(angle)*second)
                safe = all(min(segment_clearance(start,candidate,np.array(ob['center']),bounding_radius(ob)+self.radius+.008),segment_clearance(candidate,goal,np.array(ob['center']),bounding_radius(ob)+self.radius+.008))>=0 for ob in self.obstacles)
                if safe: choices.append((np.linalg.norm(candidate-start)+np.linalg.norm(goal-candidate),candidate))
            if not choices: raise RuntimeError('No collision-free preview waypoint found')
            self.waypoint = min(choices,key=lambda p:p[0])[1]
        reference = final.copy(); reference[:3,3] = self.waypoint
        return {link:reference}
