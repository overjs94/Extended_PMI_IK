"""Compiled, GIL-free CPU ADMM/DRS phases using real Numba worker threads.

Each factor owns its copies; each body/coordinate owns its output. Consensus
uses inverse-incidence CSR lists, so parallel updates never scatter into shared
locations. No fastmath or relaxed stopping criterion is used.
"""
import numpy as np
from numba import njit,prange


def inverse_incidence(indices,mask,n):
    lists=[[] for _ in range(n)]
    width=indices.shape[1]
    for row in range(len(indices)):
        for slot in range(width):
            if mask[row,slot]:lists[indices[row,slot]].append(row*width+slot)
    pointers=np.zeros(n+1,dtype=np.int64)
    for i,values in enumerate(lists):pointers[i+1]=pointers[i]+len(values)
    return pointers,np.asarray([value for values in lists for value in values],dtype=np.int64)


@njit(parallel=True,nogil=True,cache=True)
def admm(inv,b,factors,indices,mask,norm2,lower,upper,pointers,copies,rho,z,u,tolerance,maxiter):
    nb=b.shape[0];n=6*nb;m,width=factors.shape
    v=np.zeros(n);rhs=np.empty(n);old=np.empty_like(z)
    factor_res=np.empty(m);dual_res=np.empty(n)
    zflat=z.reshape(-1);uflat=u.reshape(-1);oldflat=old.reshape(-1)
    residual=np.inf
    for iteration in range(maxiter):
        for body in prange(nb):
            for c in range(6):
                i=6*body+c;total=0.
                for k in range(pointers[i],pointers[i+1]):
                    copy=copies[k];total+=zflat[copy]-uflat[copy]
                rhs[i]=b[body,c]+rho*total
            for c in range(6):
                total=0.
                for d in range(6):total+=inv[body,c,d]*rhs[6*body+d]
                v[6*body+c]=total
        for row in prange(m):
            value=0.;actual=0.
            for slot in range(width):
                if mask[row,slot]:
                    vv=v[indices[row,slot]];value+=factors[row,slot]*(vv+u[row,slot]);actual+=factors[row,slot]*vv
            correction=(min(max(value,lower[row]),upper[row])-value)/norm2[row]
            r=max(0.,lower[row]-actual,actual-upper[row])
            for slot in range(width):
                old[row,slot]=z[row,slot]
                if mask[row,slot]:
                    vv=v[indices[row,slot]]
                    z[row,slot]=vv+u[row,slot]+correction*factors[row,slot]
                    primal=vv-z[row,slot];u[row,slot]+=primal;r=max(r,abs(primal))
            factor_res[row]=r
        for i in prange(n):
            total=0.
            for k in range(pointers[i],pointers[i+1]):
                copy=copies[k];total+=zflat[copy]-oldflat[copy]
            dual_res[i]=abs(rho*total)
        residual=0.
        for row in range(m):residual=max(residual,factor_res[row])
        for i in range(n):residual=max(residual,dual_res[i])
        if residual<tolerance:break
    return v,iteration+1,residual,z,u


@njit(parallel=True,nogil=True,cache=True)
def drs(inv,b,factors,indices,mask,norm2,lower,upper,pointers,copies,gamma,zb,zf,tolerance,maxiter):
    nb=b.shape[0];n=6*nb;m,width=factors.shape
    v=np.zeros(n);pb=np.empty(n);pf=np.empty_like(zf)
    factor_res=np.empty(m);body_res=np.empty(n)
    zbflat=zb.reshape(-1);zfflat=zf.reshape(-1);pfflat=pf.reshape(-1)
    residual=np.inf
    for iteration in range(maxiter):
        for body in prange(nb):
            for c in range(6):
                total=0.
                for d in range(6):total+=inv[body,c,d]*(zb[body,d]+gamma*b[body,d])
                pb[6*body+c]=total
        for row in prange(m):
            value=0.
            for slot in range(width):value+=factors[row,slot]*zf[row,slot]
            correction=(min(max(value,lower[row]),upper[row])-value)/norm2[row]
            for slot in range(width):pf[row,slot]=(zf[row,slot]+correction*factors[row,slot])*mask[row,slot]
        for i in prange(n):
            total=2*pb[i]-zbflat[i]
            for k in range(pointers[i],pointers[i+1]):
                copy=copies[k];total+=2*pfflat[copy]-zfflat[copy]
            v[i]=total/(1+pointers[i+1]-pointers[i])
            rb=v[i]-pb[i];zbflat[i]+=rb;body_res[i]=abs(rb)
        for row in prange(m):
            r=0.;actual=0.
            for slot in range(width):
                if mask[row,slot]:
                    vv=v[indices[row,slot]];rf=vv-pf[row,slot]
                    zf[row,slot]+=rf;r=max(r,abs(rf));actual+=factors[row,slot]*vv
            factor_res[row]=max(r,lower[row]-actual,actual-upper[row])
        residual=0.
        for row in range(m):residual=max(residual,factor_res[row])
        for i in range(n):residual=max(residual,body_res[i])
        if residual<tolerance:break
    return v,iteration+1,residual,zb,zf
