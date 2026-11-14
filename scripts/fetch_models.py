#!/usr/bin/env python3
"""Fetch pinned, public robot descriptions without cloning entire repositories.

Third-party assets remain in an ignored cache. Keep source/license/checksum
metadata alongside every derived URDF. No network access occurs in the solver.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import posixpath
import urllib.parse
import urllib.request
import urllib.error
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]


def fetch(spec):
    directory=ROOT/'models/external'/spec['id'];directory.mkdir(parents=True,exist_ok=True)
    hostpath=spec['repository'].split('github.com/',1)[1]
    base=f'https://raw.githubusercontent.com/{hostpath}/{spec["commit"]}/'
    def download(path):
        url=base+urllib.parse.quote(path,safe='/')
        with urllib.request.urlopen(url,timeout=45) as response:return response.read()
    info={**spec,'status':'unavailable','files':{},'missing_meshes':[]}
    try:
        data=download(spec['urdf'])
        (directory/'source.urdf').write_bytes(data)
        info['source_sha256']=hashlib.sha256(data).hexdigest()
        robot=ET.fromstring(data)
        for filename in ['LICENSE','LICENSE.md','LICENSE.txt','LICENSE.TXT','COPYING']:
            try:
                license_data=download(filename);(directory/filename).write_bytes(license_data)
                info['license_file']=filename;break
            except urllib.error.HTTPError as error:
                if error.code!=404:raise
        # URDF geometry references are resolved relative to their source package.
        paths={}
        for mesh in robot.findall('.//mesh'):
            original=mesh.get('filename','')
            if original.startswith('package://'):
                package,tail=original[10:].split('/',1)
                package_root=spec['package_root']
                if package in ('drake','example-robot-data'):package_root=''
                if package=='allegro_hand_description' and spec['id']=='kuka-allegro':
                    package_root='diff_robot_data/allegro_hand_description'
                path=posixpath.normpath(posixpath.join(package_root,tail))
            elif original.startswith(('file://','http://','https://','/')):
                info['missing_meshes'].append(original);continue
            else:path=posixpath.normpath(posixpath.join(spec['package_root'] if original.startswith('meshes/') else posixpath.dirname(spec['urdf']),original))
            if path.startswith('../'):info['missing_meshes'].append(original);continue
            paths[original]=path
        def asset(pair):
            original,path=pair
            key=hashlib.sha256(path.encode()).hexdigest()[:16]+Path(path).suffix.lower()
            target=directory/'assets'/key;target.parent.mkdir(exist_ok=True)
            if path.lower().endswith('.dae'):
                source=directory/'source_assets'/key;source.parent.mkdir(exist_ok=True)
                if not source.exists():source.write_bytes(download(path))
                target.write_bytes(source.read_bytes())
            elif not target.exists():target.write_bytes(download(path))
            return original,path,key,hashlib.sha256(target.read_bytes()).hexdigest()
        resolved={}
        with ThreadPoolExecutor(max_workers=6) as pool:
            futures=[pool.submit(asset,pair) for pair in paths.items()]
            for (original,_),future in zip(paths.items(),futures):
                try:
                    original,path,key,checksum=future.result();resolved[original]=f'/models/external/{spec["id"]}/assets/{key}'
                    info['files'][path]={'local':key,'sha256':checksum}
                except Exception as error:info['missing_meshes'].append(f'{original}: {error}')
        # DAE texture names must remain local and reproducible too.
        for source,record in list(info['files'].items()):
            if not source.lower().endswith('.dae'):continue
            target=directory/'assets'/record['local']
            dae=ET.fromstring(target.read_bytes())
            changed=False
            for image in dae.iter():
                if image.tag.split('}')[-1]!='image':continue
                for ref in image:
                    name=(ref.text or '').strip()
                    if ref.tag.split('}')[-1]!='init_from' or not name:continue
                    try:
                        if '://' in name or name.startswith('/'):raise ValueError('External texture URI rejected')
                        path=posixpath.normpath(posixpath.join(posixpath.dirname(source),name))
                        key=hashlib.sha256(path.encode()).hexdigest()[:16]+Path(path).suffix.lower()
                        image_file=directory/'assets'/key
                        if not image_file.exists():image_file.write_bytes(download(path))
                        ref.text=key;changed=True
                        info['files'][path]={'local':key,'sha256':hashlib.sha256(image_file.read_bytes()).hexdigest()}
                    except Exception as error:info['missing_meshes'].append(f'texture {name}: {error}')
            if changed:
                # Preserve valid Collada namespace serialization for browser loaders.
                ET.register_namespace('', 'http://www.collada.org/2005/11/COLLADASchema')
                target.write_bytes(ET.tostring(dae,encoding='utf-8',xml_declaration=True))
                record['derived_sha256']=hashlib.sha256(target.read_bytes()).hexdigest()
        for mesh in robot.findall('.//mesh'):
            if mesh.get('filename') in resolved:mesh.set('filename',resolved[mesh.get('filename')])
        (directory/'robot.urdf').write_text(ET.tostring(robot,encoding='unicode'))
        info['status']='downloaded' if not info['missing_meshes'] else 'partial_meshes'
        info['dof_raw']=sum(j.get('type') not in ('fixed',) for j in robot.findall('joint'))
        info['mimic_joints']=[j.get('name') for j in robot.findall('joint') if j.find('mimic') is not None]
    except Exception as error:info['error']=str(error)
    (directory/'provenance.json').write_text(json.dumps(info,indent=2)+'\n')
    print(spec['id'],info['status'],'meshes',len(info['files']),'missing',len(info['missing_meshes']),info.get('error',''),flush=True)
    return info


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--only',nargs='*');args=p.parse_args()
    specs=json.loads((ROOT/'models/registry.json').read_text())
    if args.only:specs=[s for s in specs if s['id'] in args.only]
    with ThreadPoolExecutor(max_workers=3) as pool:results=list(pool.map(fetch,specs))
    incomplete=[r['id'] for r in results if r['status']!='downloaded']
    if incomplete:raise SystemExit('Incomplete model assets: '+', '.join(incomplete))
