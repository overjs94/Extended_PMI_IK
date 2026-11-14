import copy
import ctypes
from pathlib import Path
import shutil
import subprocess
import casadi
import numpy as np
import pytest
from extended_pmi_ik.lasdra import lasdra_scenario
from extended_pmi_ik.scenarios import make_scenario
from extended_pmi_ik.trajectories import validate_plan,solve_trajectory,make_trajectory
from extended_pmi_ik.solver import Settings


def test_lasdra_matches_unmodified_original_cpp(tmp_path):
    """Independent original FK and all ten original centre frames, not a mirror."""
    if not shutil.which('g++'):pytest.skip('C++ compiler unavailable')
    reference=Path(__file__).resolve().parents[1]/'reference/pmi_ik/include'
    source=tmp_path/'bridge.cpp'
    source.write_text('''#include "LASDRA_kinematics/LASDRA_kinematics.h"
extern "C" void original_fk(const double* x,double* out){
 Vectornd q; for(int i=0;i<DoF;i++)q(i)=x[i];
 Vector3d p; Matrix3d R;
 LASDRA_KINEMATICS_::LASDRA_Kinematics_::Fkine_ee(q,p,R);
 Matrix<double,3*n_link,1> lp; Matrix<double,3*n_link,3> lr;
 LASDRA_KINEMATICS_::LASDRA_Kinematics_::LASDRA_Pose_link(q,lp,lr);
 for(int k=0;k<=n_link;k++)for(int i=0;i<4;i++)for(int j=0;j<4;j++){
  double v=(i==j); if(i<3 && j<3)v=k==0?R(i,j):lr(3*(k-1)+i,j);
  if(i<3 && j==3)v=k==0?p(i):lp(3*(k-1)+i);
  out[16*k+4*i+j]=v;
 }
}
''')
    library=tmp_path/'original.so'
    subprocess.run(['g++','-O2','-shared','-fPIC','-I'+str(reference),'-I'+str(Path(casadi.__file__).parent/'include'),str(source),str(reference/'LASDRA_kinematics/LASDRA_kinematics.cpp'),'-o',str(library)],check=True,capture_output=True)
    fn=ctypes.CDLL(str(library)).original_fk
    ptr=np.ctypeslib.ndpointer(dtype=np.float64,flags='C_CONTIGUOUS');fn.argtypes=[ptr,ptr]
    model,_,_,_=lasdra_scenario();rng=np.random.default_rng(913)
    for q in [np.zeros(21),*rng.uniform(-.8,.8,(12,21))]:
        out=np.zeros((11,4,4));fn(q,out);poses=model.fk(q)
        for k,name in enumerate(['lasdra_ee',*[f'lasdra_link_{i}' for i in range(1,11)]]):
            np.testing.assert_allclose(poses[name],out[k],atol=2e-12)
    assert model.dof==21


def test_contact_plan_rejects_sliding_stance_and_false_cito_provenance():
    m,q,t,_=make_scenario('panda');name=next(iter(t));T=m.fk(q)[name].tolist()
    plan={'version':1,'provenance':{'kind':'authored_example'},'samples':[{'time_s':i,'targets':{name:copy.deepcopy(T)},'contacts':{name:copy.deepcopy(T)}} for i in (0.,1.)]}
    validate_plan(m,plan)
    broken=copy.deepcopy(plan);broken['samples'][1]['contacts'][name][0][3]+=.01
    broken['samples'][1]['targets'][name][0][3]+=.01
    with pytest.raises(ValueError):validate_plan(m,broken)
    broken=copy.deepcopy(plan);broken['provenance']={'kind':'external_cito_output'}
    with pytest.raises(ValueError,match='actual source'):validate_plan(m,broken)


def test_timed_trajectory_executes_real_warm_started_ik():
    m,q,targets,_=make_scenario('panda');name=next(iter(targets));T=m.fk(q)[name]
    plan={'version':1,'provenance':{'kind':'authored_example'},'samples':[{'time_s':0.,'targets':{name:T.tolist()}},{'time_s':1.,'targets':{name:targets[name].tolist()}}]}
    result=solve_trajectory(m,q,plan,method='reduced-osqp',settings=Settings(wall_time_limit_s=2.))
    assert result['status']=='converged'
    assert result['tracking']['successes']==2
    assert result['tracking']['knots'][1]['iterations']>0
    assert result['tracking']['max_position_error_m']<=1e-4
    assert result['frame_times_s']==[0.,1.]
    assert result['tracking']['knots'][1]['wall_time_ms']>0
    assert result['frames'][0]!=result['frames'][1]


def test_g1_walking_has_floating_base_contact_release_and_touchdown():
    if not (Path(__file__).resolve().parents[1]/'models/external/unitree-g1/robot.urdf').exists():pytest.skip('Optional G1 assets')
    model,_,plan,_=make_trajectory('g1-contact-walk')
    assert model.dof==35
    assert [len(s['contacts']) for s in plan['samples']]==[2,1,1,1,2,1,1,1,2]
    assert plan['provenance']['kind']=='authored_example'
    assert plan['samples'][-1]['targets']['pelvis'][0][3]>0


def test_api_executes_imported_contact_ee_input():
    from fastapi.testclient import TestClient
    from extended_pmi_ik.api import app
    client=TestClient(app)
    metadata=client.get('/api/trajectories/dual-carry').json()
    plan=metadata['plan'];plan['samples']=plan['samples'][:2]
    r=client.post('/api/trajectory/solve',json={'trajectory':'dual-carry','method':'reduced-osqp','plan':plan})
    assert r.status_code==200
    result=r.json()
    assert result['provenance']['execution']=='computed'
    assert result['tracking']['samples']==2
    assert result['tracking']['successes']==2
    plan['samples'][1]['time_s']=plan['samples'][0]['time_s']
    assert client.post('/api/trajectory/solve',json={'trajectory':'dual-carry','plan':plan}).status_code==422
