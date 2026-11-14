"""Symmetric QP adapters. Skew/VI operators must never enter these adapters."""
import itertools
import casadi as ca
import numpy as np
import osqp
from scipy import sparse

_ids = itertools.count()


class QPBackend:
    def __init__(self, name, n, m):
        self.name = name
        self.n, self.m = n,m
        if name == 'qpoases':
            self.solver = ca.conic(f'qp_{next(_ids)}','qpoases',{'h':ca.Sparsity.dense(n,n),'a':ca.Sparsity.dense(m,n)},
                                   {'printLevel':'none','print_time':False,'error_on_fail':False,'nWSR':2000})
        elif name != 'osqp':
            raise ValueError('QP backend must be osqp or qpoases')

    def solve(self,H,b,A,lower,upper):
        if not np.allclose(H,H.T,atol=1e-10):
            raise ValueError('A nonsymmetric monotone operator is not a QP Hessian')
        if self.name == 'osqp':
            solver = osqp.OSQP()
            solver.setup(P=sparse.csc_matrix(np.triu(H)),q=-b,A=sparse.csc_matrix(A),l=lower,u=upper,
                         verbose=False,eps_abs=1e-8,eps_rel=1e-8,max_iter=10000,polishing=bool(len(A)>self.n))
            result = solver.solve(raise_error=False)
            if result.info.status_val != 1:
                raise RuntimeError(f'OSQP: {result.info.status}')
            return result.x,int(result.info.iter)
        result = self.solver(h=H,g=-b,a=A,lba=lower,uba=upper,lbx=-np.inf,ubx=np.inf)
        stats = self.solver.stats()
        if not stats.get('success',False):
            raise RuntimeError(f'qpOASES: {stats.get("return_status")}')
        return np.asarray(result['x']).ravel(),int(stats.get('iter_count',0))
