import json
from pathlib import Path
import numpy as np
import pytest
from extended_pmi_ik.scenarios import ROOT,REGISTRY,make_scenario
from extended_pmi_ik.constraints import ConstraintGraph


def test_seven_real_humanoid_families_are_pinned():
    humanoids=[e for e in REGISTRY if e['category']=='humanoid']
    assert len(humanoids)==7
    assert all(len(e['commit'])==40 for e in REGISTRY)
    assert len({e['id'] for e in REGISTRY})==len(REGISTRY)


@pytest.mark.parametrize('spec',REGISTRY,ids=lambda s:s['id'])
def test_downloaded_model_initial_constraints_and_fk(spec):
    if not (ROOT/'models/external'/spec['id']/'robot.urdf').exists():
        pytest.skip('Optional public model cache not downloaded')
    m,q,t,g=make_scenario(spec['id'])
    poses=m.fk(q)
    for target in t:assert target in poses
    graph=ConstraintGraph(m,g)
    eq,J,h,Jh=graph.evaluate({n:poses[n] for n in m.bodies})
    assert np.max(np.abs(eq),initial=0)<1e-7
    assert np.min(h,initial=0)>-1e-7
    assert np.all(np.isfinite(J)) and np.all(np.isfinite(Jh))
    for n,T in poses.items():
        np.testing.assert_allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-10)
