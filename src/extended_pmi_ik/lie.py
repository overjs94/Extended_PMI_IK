import numpy as np
from scipy.spatial.transform import Rotation


def skew(x):
    a, b, c = x
    return np.array([[0., -c, b], [c, 0., -a], [-b, a, 0.]])


def transform(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
    T[:3, 3] = xyz
    return T


def inverse(T):
    out = np.eye(4)
    out[:3, :3] = T[:3, :3].T
    out[:3, 3] = -out[:3, :3] @ T[:3, 3]
    return out


def exp_se3(v):
    w, u = np.asarray(v[:3]), np.asarray(v[3:])
    theta = np.linalg.norm(w)
    W = skew(w)
    if theta < 1e-7:
        J = np.eye(3) + .5*W + W@W/6
    else:
        J = np.eye(3) + (1-np.cos(theta))/theta**2*W + (theta-np.sin(theta))/theta**3*(W@W)
    T = np.eye(4)
    T[:3, :3] = Rotation.from_rotvec(w).as_matrix()
    T[:3, 3] = J @ u
    return T


def gyro(G, v):
    """White note (11): G ad_v - ad_v.T G, angular-first body twists."""
    ad = np.zeros((6, 6))
    ad[:3, :3] = ad[3:, 3:] = skew(v[:3])
    ad[3:, :3] = skew(v[3:])
    return G @ ad - ad.T @ G
