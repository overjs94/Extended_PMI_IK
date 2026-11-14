"""Signed point-to-primitive distances used by all benchmark adapters.

Robot geometry is a declared sphere proxy; obstacles are spheres or axis-aligned
boxes. These are not mesh/self-collision checks.
"""
import numpy as np


def point_distance_gradient(point, obstacle):
    d = np.asarray(point)-np.asarray(obstacle['center'])
    if obstacle.get('type','sphere') == 'sphere':
        length = float(np.linalg.norm(d))
        return length-obstacle['radius'], d/max(length,1e-12)
    if obstacle['type'] != 'box':raise ValueError('Unsupported obstacle primitive')
    q = np.abs(d)-np.asarray(obstacle['half_extents'])
    outside = np.maximum(q,0)
    length = float(np.linalg.norm(outside))
    distance = length+min(float(np.max(q)),0.)
    if length>1e-12:
        gradient = np.sign(d)*outside/length
    else:
        gradient = np.zeros(3);i=int(np.argmax(q));gradient[i]=1 if d[i]>=0 else -1
    return distance,gradient


def symbolic_point_distance(point, obstacle):
    import casadi as ca
    d = point-ca.DM(obstacle['center'])
    if obstacle.get('type','sphere') == 'sphere':
        return ca.sqrt(ca.dot(d,d)+1e-16)-obstacle['radius']
    if obstacle['type'] != 'box':raise ValueError('Unsupported obstacle primitive')
    q = ca.fabs(d)-ca.DM(obstacle['half_extents'])
    outside = ca.fmax(q,0)
    # fmax avoids the 0/0 derivative of sqrt at a box's interior.
    length = ca.if_else(ca.dot(outside,outside)>1e-20,ca.sqrt(ca.fmax(ca.dot(outside,outside),1e-20)),0)
    return length+ca.fmin(ca.mmax(q),0)


def bounding_radius(obstacle):
    return obstacle['radius'] if obstacle.get('type','sphere')=='sphere' else float(np.linalg.norm(obstacle['half_extents']))
