import subprocess
import casadi as ca
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from extended_pmi_ik.kinematics import RobotModel
from extended_pmi_ik.scenarios import make_scenario
from extended_pmi_ik.lie import exp_se3,gyro
from extended_pmi_ik.constraints import ConstraintGraph


def test_urdf_jacobian_matches_finite_differences():
    m,q,_,_ = make_scenario('panda')
    link = 'a0_panda_link8'; T = m.fk(q)[link]; J = m.jacobian(q,link)
    for i in range(m.dof):
        perturbed = q.copy();perturbed[i] += 1e-7
        U = m.fk(perturbed)[link]
        finite = np.r_[Rotation.from_matrix(U[:3,:3]@T[:3,:3].T).as_rotvec(),U[:3,3]-T[:3,3]]/1e-7
        np.testing.assert_allclose(J[:,i],finite,atol=1e-6)


def test_maximal_constraint_jacobian_and_collision_derivative():
    m,q,_,g = make_scenario('obstacle')
    X = {n:T for n,T in m.fk(q).items() if n in m.bodies}
    graph = ConstraintGraph(m,g); eq,J,h,Jh = graph.evaluate(X)
    v = np.random.default_rng(9).normal(size=6*len(m.bodies))
    perturbed = {n:T@exp_se3(1e-7*v[i*6:i*6+6]) for i,(n,T) in enumerate(X.items())}
    eq2,_,h2,_ = graph.evaluate(perturbed)
    np.testing.assert_allclose((eq2-eq)/1e-7,J@v,atol=3e-6)
    np.testing.assert_allclose((h2-h)/1e-7,Jh@v,atol=3e-6)


def test_generated_c_runs_and_matches_python(tmp_path):
    m,q,_,_ = make_scenario('panda'); m.export(tmp_path)
    subprocess.run(['gcc','-O2','-shared','-fPIC',str(tmp_path/'kinematics.c'),'-o',str(tmp_path/'kinematics.so'),'-lm'],check=True)
    external = ca.external('fk',str(tmp_path/'kinematics.so'))
    np.testing.assert_allclose(external(q),m._fk(q),atol=1e-12)


def test_skew_zero_power_and_spd_symmetric_part():
    rng=np.random.default_rng(4); A=rng.normal(size=(6,6));G=A.T@A+np.eye(6);v=rng.normal(size=6)
    C=gyro(G,v)
    np.testing.assert_allclose(C+C.T,0,atol=1e-12)
    assert abs(v@C@v)<1e-12
    assert np.linalg.eigvalsh(((G+C)+(G+C).T)/2).min()>0


def test_mimic_substitution_preserves_dependent_limits():
    xml='''<robot name="mimic"><link name="a"/><link name="b"/><link name="c"/>
    <joint name="master" type="revolute"><parent link="a"/><child link="b"/><axis xyz="0 0 1"/><limit lower="-1" upper="1"/></joint>
    <joint name="slave" type="revolute"><parent link="b"/><child link="c"/><axis xyz="0 0 1"/><limit lower="-.4" upper=".4"/><mimic joint="master" multiplier="2"/></joint></robot>'''
    m=RobotModel(xml)
    assert m.dof==1
    np.testing.assert_allclose([m.lower[0],m.upper[0]],[-.2,.2])
    np.testing.assert_allclose(Rotation.from_matrix(m.fk([.1])['c'][:3,:3]).as_rotvec(),[0,0,.3],atol=1e-12)
    graph=ConstraintGraph(m,[]);poses=m.fk([.1])
    eq,_,h,_=graph.evaluate({n:poses[n] for n in m.bodies})
    np.testing.assert_allclose(eq,0,atol=1e-12)
    assert h.min()>=0


@pytest.mark.parametrize('xml',[
    '<robot name="x"><link name="a"/><link name="a"/></robot>',
    '<robot name="x"><link name="a"/><link name="b"/></robot>',
    '<robot name="x"><link name="a"/><joint name="j" type="planar"><parent link="a"/><child link="b"/></joint></robot>',
])
def test_invalid_models_fail_explicitly(xml):
    with pytest.raises(ValueError):RobotModel(xml)
