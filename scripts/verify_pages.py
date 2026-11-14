#!/usr/bin/env python3
"""Fail publication if data is missing, nonfinite or missing its replay label."""
import gzip
import json
from pathlib import Path
import math

ROOT=Path(__file__).resolve().parents[1]
DIST=ROOT/'web/dist'


def check_numbers(value):
    if isinstance(value,float) and not math.isfinite(value):raise ValueError('Nonfinite result')
    if isinstance(value,dict):
        for v in value.values():check_numbers(v)
    elif isinstance(value,list):
        for v in value:check_numbers(v)


def main():
    html=(DIST/'index.html').read_text()
    if 'execution-notice' not in html:raise ValueError('Missing recorded-execution notice')
    catalog=json.loads(gzip.decompress((DIST/'data/catalog.json.gz').read_bytes()))
    count=0
    for item in catalog['scenarios']:
        scene=json.loads(gzip.decompress((DIST/'data/scenarios'/f'{item["id"]}.json.gz').read_bytes()))
        for geom in scene['visuals']+scene['collisions']:
            filename=geom.get('filename','')
            if filename.startswith('/models/external/') and not (DIST/filename.lstrip('/')).is_file():raise ValueError('Missing local mesh: '+filename)
        for method in catalog['methods']:
            record=json.loads(gzip.decompress((DIST/'data/runs'/item['id']/f'{method}.json.gz').read_bytes()))
            check_numbers(record)
            assert record['recording']['seed']==7 and record['frames']
            assert record['frame_iterations'][0]==0
            assert record['frame_iterations'][-1]==record['recording']['original_frame_count']-1
            assert set(record['frames'][-1])==set(scene['initial'])
            count+=1
    print('Verified static publication:',len(catalog['scenarios']),'scenes,',count,'measured records and local meshes')


if __name__=='__main__':main()
