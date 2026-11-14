#!/usr/bin/env python3
"""External post-deploy check, executed by the Pages workflow."""
import gzip
import json
import os
import time
import urllib.request
from urllib.parse import urljoin

base=os.environ['SITE_URL'].rstrip('/')+'/'
def get(path):
    request=urllib.request.Request(urljoin(base,path),headers={'User-Agent':'Extended-PMI-IK-deployment-check'})
    with urllib.request.urlopen(request,timeout=25) as response:
        if response.status!=200:raise RuntimeError('Unexpected status')
        return response.read()
last=None
for attempt in range(20):
    try:
        assert b'execution-notice' in get('')
        catalog=json.loads(gzip.decompress(get('data/catalog.json.gz')))
        research=json.loads(gzip.decompress(get('data/research.json.gz')))
        assert len(catalog['scenarios'])>=21 and research['pilot']['rows']
        record=json.loads(gzip.decompress(get('data/runs/dual/maximal-schur.json.gz')))
        assert record['frames'] and record['recording']['seed']==7
        assert len(get('models/franka_description/meshes/visual/link0.dae'))>1000
        tracking=json.loads(gzip.decompress(get('data/tracking/runs/g1-contact-walk/pink-osqp.json.gz')))
        assert tracking['tracking']['samples']==9 and len(tracking['frames'])==9
        assert 'lasdra' in {s['id'] for s in catalog['scenarios']}
        print('Verified public homepage:',base,'with measured records, benchmark evidence and realistic robot meshes')
        break
    except Exception as error:
        last=error
        if attempt==19:raise
        time.sleep(3)
