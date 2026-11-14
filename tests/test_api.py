from fastapi.testclient import TestClient
from extended_pmi_ik.api import app

client=TestClient(app)


def test_api_solves_and_validates_inputs():
    assert client.get('/api/health').status_code==200
    assert len(client.get('/api/catalog').json()['scenarios'])>=6
    r=client.post('/api/solve',json={'scenario':'panda','method':'maximal-skew'})
    assert r.status_code==200
    assert r.json()['status']=='converged'
    assert client.post('/api/solve',json={'iterations':0}).status_code==422
    assert client.get('/api/scenarios/not-a-robot').status_code==422
    assert client.post('/api/urdf/inspect',json={'xml':'<robot/>'}).status_code==422
