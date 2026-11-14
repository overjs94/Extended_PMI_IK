import json
import numpy as np
import pytest
from extended_pmi_ik.benchmarks.cases import freeze_case,thaw_case
from extended_pmi_ik.benchmarks.runner import run_case,score,summaries
from extended_pmi_ik.benchmarks.native import NativeIK
from extended_pmi_ik.distance import point_distance_gradient,symbolic_point_distance
from extended_pmi_ik.solver import Settings
from extended_pmi_ik.scenarios import make_scenario
from extended_pmi_ik.constraints import ConstraintGraph
from extended_pmi_ik.lie import transform


@pytest.mark.parametrize('point,expected',[
    ([2,0,0],1.),([0,0,0],-1.),([2,2,0],np.sqrt(2.)),([.2,.3,.4],-.6),
])
def test_signed_box_distance_and_derivative(point,expected):
    import casadi as ca
    box={'type':'box','center':[0,0,0],'half_extents':[1,1,1]}
    value,gradient=point_distance_gradient(point,box)
    assert value==pytest.approx(expected)
    p=ca.SX.sym('p',3);d=symbolic_point_distance(p,box);fun=ca.Function('d',[p],[d,ca.gradient(d,p)])
    symbolic,jac=fun(point);assert float(symbolic)==pytest.approx(expected)
    if point not in ([0,0,0],):np.testing.assert_allclose(np.asarray(jac).ravel(),gradient,atol=1e-10)


def test_frozen_cases_cannot_be_silently_changed():
    case=freeze_case('panda','reachable-random',7)
    m,q,t,g=thaw_case(case)
    assert len(t)==1
    case['q_initial'][0]+=.01
    with pytest.raises(ValueError,match='checksum'):thaw_case(case)


def test_clutter_start_and_goal_are_valid_and_seeds_change_cases():
    first=freeze_case('panda','clutter-boxes',7);second=freeze_case('panda','clutter-boxes',11)
    assert first['sha256']!=second['sha256']
    m,q,t,g=thaw_case(first);graph=ConstraintGraph(m,g);poses=m.fk(q)
    _,_,h,_=graph.evaluate({n:poses[n] for n in m.bodies})
    assert h[graph.limit_rows:].min()>.002
    assert len(m.collision_spheres)>1 and len(m.obstacles)==3


def test_geodesic_closure_validator_rejects_pi_false_zero():
    m,q,t,g=make_scenario('panda');poses=m.fk(q)
    target={'a0_panda_link8':poses['a0_panda_link8']}
    wrong=poses['a0_panda_link8']@transform(rpy=[0,0,np.pi])
    result=score(m,q,target,[(m.root,'a0_panda_link8',wrong)],Settings())
    assert result['closure_rotation_rad']==pytest.approx(np.pi)
    assert not result['endpoint_feasible']


def test_common_deadline_and_unsupported_are_distinct():
    case=freeze_case('panda','nearby',7)
    row=run_case(case,'maximal-schur',Settings(wall_time_limit_s=1e-9,record_frames=False),warmup=False)
    assert row['status']=='timeout' and not row['within_budget']
    row=run_case(case,'not-a-real-library',Settings(),warmup=False)
    assert row['status']=='unsupported'


def test_unreachable_control_is_not_a_feasible_success_rate_case():
    case=freeze_case('panda','unreachable',7)
    assert not case['expected_feasible']
    assert case['feasibility_certificate']['target_root_distance_m']>case['feasibility_certificate']['maximum_root_reach_m']


@pytest.mark.parametrize('backend',['pink','mink'])
@pytest.mark.parametrize('scenario',['panda','dual','unitree-g1'])
def test_native_library_fk_conversion_and_solve(backend,scenario):
    pytest.importorskip(backend)
    from extended_pmi_ik.scenarios import ROOT
    if scenario=='unitree-g1' and not (ROOT/'models/external/unitree-g1/robot.urdf').exists():pytest.skip('Optional vendor data not downloaded')
    case=freeze_case(scenario,'closed-loop' if scenario!='panda' else 'nearby',7)
    row=run_case(case,backend+'-osqp',Settings(iterations=100,record_frames=False,strict_closure_check=True),warmup=False)
    assert row['status']=='success',row['failure']
    assert row['assessment']['endpoint_feasible']


@pytest.mark.parametrize('backend',['pink','mink'])
def test_native_distance_rows_match_shared_constraint_and_finite_difference(backend):
    pytest.importorskip(backend)
    m,q,t,g=thaw_case(freeze_case('panda','clutter-boxes',7))
    cfg=Settings(preview=False);native=NativeIK(backend,m,q,t,g,cfg)
    h,J=native.distance_rows();graph=ConstraintGraph(m,g);poses=m.fk(q)
    _,_,common,_=graph.evaluate({n:poses[n] for n in m.bodies})
    np.testing.assert_allclose(h,common[graph.limit_rows:],atol=1e-10)
    original=native.configuration.q.copy();v=np.random.default_rng(4).normal(size=native.model.nv)
    native.configuration.integrate_inplace(v,1e-7);perturbed,_=native.distance_rows()
    np.testing.assert_allclose((perturbed-h)/1e-7,J@v,atol=3e-6)
    native.configuration.update(original)


def test_slsqp_is_real_constrained_nlp_not_paper_solver_substitution():
    case=freeze_case('panda','nearby',7)
    row=run_case(case,'scipy-slsqp',Settings(record_frames=False),warmup=False)
    assert row['status']=='success'
    assert 'scipy.optimize' in row['provenance']['implementation']
