#!/usr/bin/env python3
"""Assemble the static Pages artifact from measured recordings and local assets."""
import gzip
import json
from pathlib import Path
import shutil

ROOT=Path(__file__).resolve().parents[1]
DIST=ROOT/'web/dist'


def main():
    if not (DIST/'index.html').exists():raise SystemExit('Build the Vite static mode first')
    recordings=ROOT/'web/recordings'
    catalog=json.loads(gzip.decompress((recordings/'catalog.json.gz').read_bytes()))
    for item in catalog['scenarios']:
        if not (recordings/'scenarios'/f'{item["id"]}.json.gz').exists():raise SystemExit('Missing scenario '+item['id'])
        for method in catalog['methods']:
            if not (recordings/'runs'/item['id']/f'{method}.json.gz').exists():raise SystemExit('Missing measured recording '+item['id']+'/'+method)
    shutil.copytree(recordings,DIST/'data',dirs_exist_ok=True)
    research={}
    for key,path in {'literature':'benchmarks/literature.json','pilot':'benchmarks/reports/pilot.json','cpu_kernels':'benchmarks/reports/cpu-kernels.json','cpu_batch':'benchmarks/reports/cpu-batch.json'}.items():
        research[key]=json.loads((ROOT/path).read_text()) if (ROOT/path).exists() else None
    (DIST/'data/research.json.gz').write_bytes(gzip.compress(json.dumps(research,separators=(',',':'),allow_nan=False).encode(),mtime=0))
    shutil.copytree(ROOT/'models/franka_description',DIST/'models/franka_description',dirs_exist_ok=True)
    for source in (ROOT/'models/external').iterdir():
        if not source.is_dir():continue
        target=DIST/'models/external'/source.name
        if (source/'assets').exists():shutil.copytree(source/'assets',target/'assets',dirs_exist_ok=True)
        for path in source.iterdir():
            if path.is_file() and (path.name=='provenance.json' or path.name.upper().startswith(('LICENSE','COPYING','NOTICE'))):
                target.mkdir(parents=True,exist_ok=True);shutil.copy2(path,target/path.name)
    reports=DIST/'reports';reports.mkdir(exist_ok=True)
    for path in (ROOT/'benchmarks/reports').glob('*'):
        if path.suffix in ('.json','.csv','.md'):shutil.copy2(path,reports/path.name)
    (DIST/'.nojekyll').write_text('')
    (DIST/'DATA-PROVENANCE.txt').write_text('This site replays measured solver records; it performs no live IK optimization.\nRobot assets retain upstream provenance and available licenses in models/external/<id>/.\nNo original-paper reproduction, GPU speedup, full-mesh collision certificate or physical trajectory certification is claimed.\n')
    print('Assembled',len(catalog['scenarios']),'scenes with',len(catalog['methods']),'measured methods each')


if __name__=='__main__':main()
