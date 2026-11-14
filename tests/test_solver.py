import numpy as np
import pytest
from extended_pmi_ik.backends import QPBackend
from extended_pmi_ik.solver import METHODS,vi_step,solve_ik,Settings
from extended_pmi_ik.splitting import ConsensusSolver
from extended_pmi_ik.scenarios import make_scenario


@pytest.mark.parametrize('backend',['osqp','qpoases'])
def test_qp_backend_active_inequality(backend):
    q=QPBackend(backend,2,1)
    v,_=q.solve(np.eye(2),np.array([-1.,2.]),np.array([[1.,0.]]),np.array([0.]),np.array([np.inf]))
    np.testing.assert_allclose(v,[0,2],atol=1e-6)
    with pytest.raises(ValueError):q.solve(np.array([[1.,1.],[0,1]]),np.zeros(2),np.zeros((1,2)),np.zeros(1),np.ones(1))


@pytest.mark.parametrize('kind',['admm','drs'])
def test_splitting_matches_dense_vi_with_active_inequality(kind):
    rng=np.random.default_rng(3)
    H=np.diag(np.linspace(2,4,12));C=np.zeros_like(H)
    if kind=='drs':
        for i in [0,6]:
            A=rng.normal(size=(6,6));C[i:i+6,i:i+6]=.1*(A-A.T)
    b=rng.normal(size=12);J=np.zeros((1,12));J[0,[0,6]]=[1,-1]
    Jh=np.eye(12)[:2];lower=np.array([.3,-.2]);e=np.zeros(1)
    dense,_,_=vi_step(H+C,b,J,e,Jh,lower)
    schur,_,_=vi_step(H+C,b,J,e,Jh,lower,True)
    split,_,residual=ConsensusSolver(kind).solve(H+C,b,J,e,Jh,lower)
    np.testing.assert_allclose(schur,dense,atol=1e-8)
    np.testing.assert_allclose(split,dense,atol=2e-6)


@pytest.mark.parametrize('method',list(METHODS))
def test_reachable_panda_and_recorded_final_state(method):
    m,q,t,g=make_scenario('panda')
    r=solve_ik(m,q,t,g,method)
    assert r['status']=='converged',r['failure']
    assert r['metrics']['position_error_m']<1e-4
    assert r['metrics']['joint_residual']<1e-5
    assert len(r['frames'])==len(r['history'])==r['metrics']['iterations']+1
    assert r['provenance']['parallel'] is False


def test_iteration_budget_is_not_reported_as_success():
    m,q,t,g=make_scenario('dual')
    r=solve_ik(m,q,t,g,settings=Settings(iterations=1))
    assert r['status']=='iteration_limit'


def test_closed_loop_multi_arm_and_humanoid():
    for scenario in ['dual','quad','humanoid']:
        m,q,t,g=make_scenario(scenario)
        r=solve_ik(m,q,t,g,method='maximal-schur')
        assert r['status']=='converged',(scenario,r['metrics'])
        assert r['metrics']['grasp_residual']<1e-5
        assert r['metrics']['joint_residual']<1e-5


def test_obstacle_detour_has_positive_actual_clearance():
    m,q,t,g=make_scenario('obstacle')
    guided=solve_ik(m,q,t,g,method='maximal-skew')
    assert guided['status']=='converged',guided['metrics']
    assert guided['metrics']['trajectory_min_clearance_m']>=-1e-5
    assert any(h['preview_active'] for h in guided['history'])
