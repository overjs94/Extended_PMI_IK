import numpy as np
import pytest
from extended_pmi_ik.solver import vi_step
from extended_pmi_ik.splitting import ConsensusSolver


@pytest.mark.parametrize('kind',['admm','drs'])
@pytest.mark.parametrize('workers',[1,2,4])
def test_real_cpu_threads_preserve_the_constrained_solution(kind,workers):
    import numba
    if workers>numba.config.NUMBA_NUM_THREADS:pytest.skip('Worker count exceeds available Numba pool')
    rng=np.random.default_rng(19)
    H=np.diag(np.linspace(2,4,24));C=np.zeros_like(H)
    if kind=='drs':
        for i in range(0,24,6):
            block=rng.normal(size=(6,6));C[i:i+6,i:i+6]=.1*(block-block.T)
    b=rng.normal(size=24);J=np.zeros((3,24))
    for i in range(3):J[i,6*i]=1;J[i,6*(i+1)]=-1
    Jh=np.eye(24)[:2];lo=np.array([.3,-.2]);e=np.zeros(3)
    expected,_,_=vi_step(H+C,b,J,e,Jh,lo)
    v,it,res=ConsensusSolver(kind,backend='numba',workers=workers).solve(H+C,b,J,e,Jh,lo)
    np.testing.assert_allclose(v,expected,atol=2e-6)
    assert res<1e-5
    # Warm-start the exact same problem; parallel reuse must remain correct.
    solver=ConsensusSolver(kind,backend='numba',workers=workers)
    solver.solve(H+C,b,J,e,Jh,lo);again,_,_=solver.solve(H+C,b,J,e,Jh,lo)
    np.testing.assert_allclose(again,expected,atol=2e-6)
