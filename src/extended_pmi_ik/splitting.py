"""Body/factor product-space splitting, vectorized on CPU.

All body resolvents and factor projections are independent within each phase.
This reference executes NumPy batches, not distributed workers or GPU kernels.
"""
import numpy as np


class ConsensusSolver:
    def __init__(self,kind,tolerance=2e-7,max_iterations=3000,backend='numpy',workers=1):
        self.kind,self.tol,self.maxiter = kind,tolerance,max_iterations
        if backend not in ('numpy','numba'):raise ValueError('Unknown CPU kernel backend')
        if workers<1:raise ValueError('workers must be positive')
        if backend=='numpy' and workers!=1:raise ValueError('NumPy reference does not launch separate workers')
        self.backend,self.workers=backend,workers
        self.state = None

    def solve(self,D,b,J,e,Jh,lower):
        n = len(b); nb = n//6
        A = np.vstack([J,Jh]); lo = np.r_[-e,lower]
        hi = np.r_[-e,np.full(len(lower),np.inf)]
        # Store only incident body copies: O(edges), rather than edges x bodies.
        incidence = np.linalg.norm(A.reshape(len(A),nb,6),axis=2)>1e-12
        width = int(np.max(incidence.sum(axis=1),initial=1))*6
        indices = np.zeros((len(A),width),dtype=int)
        mask = np.zeros((len(A),width))
        factors = np.zeros((len(A),width))
        for row in range(len(A)):
            columns = np.concatenate([np.arange(6*i,6*i+6) for i in np.flatnonzero(incidence[row])]) if incidence[row].any() else np.array([],dtype=int)
            indices[row,:len(columns)] = columns
            mask[row,:len(columns)] = 1
            factors[row,:len(columns)] = A[row,columns]
        norm2 = np.einsum('mi,mi->m',factors,factors)
        if np.any(norm2<1e-20): raise RuntimeError('Degenerate factor row')
        def gather(v):return v.ravel()[indices]*mask
        def scatter(copies):return np.bincount(indices.ravel(),weights=(copies*mask).ravel(),minlength=n).reshape(nb,6)
        degree = scatter(mask)
        blocks = np.array([D[i:i+6,i:i+6] for i in range(0,n,6)])
        body_b = b.reshape(nb,6)
        def projection(z):
            value = np.einsum('mi,mi->m',factors,z)
            correction = (np.clip(value,lo,hi)-value)/norm2
            return (z + correction[:,None]*factors)*mask
        def matvec(B,v): return np.einsum('bij,bj->bi',B,v)
        def feasible(v):
            value = A@v.ravel()
            return max(float(np.max(lo-value,initial=0)),float(np.max(value-hi,initial=0)))
        scale = max(float(np.mean(np.diag(D))),1.)
        if self.backend=='numba':
            from . import parallel_kernels
            import numba
            pointers,copies=parallel_kernels.inverse_incidence(indices,mask,n)
            previous=numba.get_num_threads()
            try:
                numba.set_num_threads(self.workers)
                if self.kind=='admm':
                    if not np.allclose(D,D.T):raise ValueError('QP ADMM requires a symmetric operator; use DRS for Skew-PMI')
                    inv=np.linalg.inv(blocks+scale*degree[:,:,None]*np.eye(6))
                    z,u=self.state if self.state is not None else (np.zeros_like(factors),np.zeros_like(factors))
                    v,iterations,residual,z,u=parallel_kernels.admm(inv,body_b,factors,indices,mask,norm2,lo,hi,pointers,copies,scale,z,u,self.tol,self.maxiter)
                    self.state=(z,u)
                else:
                    gamma=1/scale;inv=np.linalg.inv(np.eye(6)+gamma*blocks)
                    zb,zf=self.state if self.state is not None else (np.zeros((nb,6)),np.zeros_like(factors))
                    v,iterations,residual,zb,zf=parallel_kernels.drs(inv,body_b,factors,indices,mask,norm2,lo,hi,pointers,copies,gamma,zb,zf,self.tol,self.maxiter)
                    self.state=(zb,zf)
            finally:numba.set_num_threads(previous)
            if residual>1e-5:raise RuntimeError(f'{self.kind.upper()} compiled inner limit: residual={residual:.3g}, iterations={iterations}')
            return v,iterations,residual
        if self.kind == 'admm':
            if not np.allclose(D,D.T): raise ValueError('QP ADMM requires a symmetric operator; use DRS for Skew-PMI')
            rho = scale
            inv = np.linalg.inv(blocks+rho*degree[:,:,None]*np.eye(6))
            z,u = self.state if self.state is not None else (np.zeros_like(factors),np.zeros_like(factors))
            for iteration in range(self.maxiter):
                v = matvec(inv,body_b+rho*scatter(z-u))
                old = z
                z = projection(gather(v)+u)
                primal = (gather(v)-z)*mask
                u = u+primal
                dual = rho*scatter(z-old)
                residual = max(float(np.max(np.abs(primal),initial=0)),float(np.max(np.abs(dual),initial=0)),feasible(v))
                if residual<self.tol: break
            self.state = (z,u)
        else:
            gamma = 1/scale
            inv = np.linalg.inv(np.eye(6)+gamma*blocks)
            zb,zf = self.state if self.state is not None else (np.zeros((nb,6)),np.zeros_like(factors))
            for iteration in range(self.maxiter):
                pb = matvec(inv,zb+gamma*body_b)
                pf = projection(zf)
                # Orthogonal projection onto all incident copies agreeing.
                v = (2*pb-zb+scatter(2*pf-zf))/(1+degree)
                rb,rf = v-pb,(gather(v)-pf)*mask
                zb,zf = zb+rb,zf+rf
                residual = max(float(np.max(np.abs(rb))),float(np.max(np.abs(rf),initial=0)),feasible(v))
                if residual<self.tol: break
            self.state = (zb,zf)
        if residual>1e-5:
            raise RuntimeError(f'{self.kind.upper()} inner limit: residual={residual:.3g}, iterations={iteration+1}')
        return v.ravel(),iteration+1,residual
