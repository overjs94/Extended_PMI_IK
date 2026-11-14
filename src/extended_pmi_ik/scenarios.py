"""Deterministic scenarios made from the Panda URDF supplied by the author."""
from pathlib import Path
import copy
import json
import xml.etree.ElementTree as ET
import numpy as np
from .kinematics import RobotModel
from .lie import inverse, transform

ROOT = Path(__file__).resolve().parents[2]
CATALOG = [
    dict(id='panda', name='Panda / pose reach', arms=1, description='Author-supplied 7-DoF Panda kinematics'),
    dict(id='dual', name='Dual Panda / rigid object', arms=2, description='14 actuated DoF + 6 object DoF · rigid grasp equalities'),
    dict(id='quad', name='Four-arm / cooperative lift', arms=4, description='28 actuated DoF + 6 object DoF · synthetic assembly'),
    dict(id='octo', name='Eight-arm / scaling study', arms=8, description='56 actuated DoF + 6 object DoF · synthetic scaling assembly'),
    dict(id='humanoid', name='Humanoid / whole-body carry', arms=2, description='32 actuated DoF + 6 object DoF · synthetic humanoid, fixed feet and rigid grasps'),
    dict(id='obstacle', name='Panda / obstacle detour', arms=1, description='7 DoF · end-effector sphere avoidance + receding geometric preview'),
]
REGISTRY = json.loads((ROOT/'models/registry.json').read_text())
for entry in REGISTRY:
    CATALOG.append(dict(id=entry['id'],name=f'{entry["category"].capitalize()} / {entry["name"]}',category=entry['category'],
                        description=f'Public URDF · {entry["repository"].split("github.com/")[-1]} · pinned source',source=entry))


def external_scenario(spec,seed):
    path = ROOT/'models/external'/spec['id']/'robot.urdf'
    if not path.exists():raise ValueError(f'Model assets not installed. Run python scripts/fetch_models.py --only {spec["id"]}')
    model = RobotModel.from_file(path)
    q = np.clip(np.zeros(model.dof),model.lower,model.upper)
    rng = np.random.default_rng(seed)
    qgoal = q.copy()
    for j in model.joints:
        if j.index<0 or j.kind in ('fixed','floating'):continue
        upper_body = any(s in j.name.lower() for s in ['arm','shoulder','elbow','wrist','waist','torso','neck','head'])
        if spec['category']!='humanoid' or upper_body:
            qgoal[j.index] = np.clip(q[j.index]+rng.uniform(-.07,.07),j.lower,j.upper)
    initial,goal = model.fk(q),model.fk(qgoal)
    links = spec.get('target_links')
    if not links:
        leaves = set(model.links)-{j.parent for j in model.joints}
        links = [max(leaves,key=lambda n:np.linalg.norm(goal[n][:3,3]-initial[n][:3,3]))]
    targets = {n:goal[n] for n in links}
    grasps = [('world_anchor',n,initial[n]) for n in spec.get('anchor_links',[])]
    # The URDF root is the world anchor regardless of its vendor-specific name.
    grasps = [(model.root,n,T) for _,n,T in grasps]
    return model,q,targets,grasps


def humanoid_xml():
    """Synthetic 32-actuator humanoid; dimensions are illustrative, not a vendor model."""
    root = ET.Element('robot',name='synthetic_humanoid_32')
    ET.SubElement(root,'link',name='world')
    def chain(prefix,parent,start,axes,lengths):
        for i,(axis,delta) in enumerate(zip(axes,lengths)):
            name = f'{prefix}_{i}'
            link = ET.SubElement(root,'link',name=name)
            v = ET.SubElement(link,'visual'); g = ET.SubElement(v,'geometry')
            ET.SubElement(g,'sphere',radius='.055' if prefix!='neck' else '.07')
            j = ET.SubElement(root,'joint',name=name+'_joint',type='revolute')
            ET.SubElement(j,'parent',link=parent);ET.SubElement(j,'child',link=name)
            ET.SubElement(j,'origin',xyz=' '.join(map(str,start if i==0 else delta)))
            ET.SubElement(j,'axis',xyz=' '.join(map(str,axis)))
            ET.SubElement(j,'limit',lower='-2.5',upper='2.5',effort='100',velocity='2')
            parent = name
        return parent
    z,y,x = [0,0,1],[0,1,0],[1,0,0]
    torso = chain('torso','world',[0,0,.92],[z,y,x],[[0,0,0],[0,0,.08],[0,0,.08]])
    for side,sign in [('left',1),('right',-1)]:
        chain(side+'_leg','world',[0,sign*.13,.92],[z,x,y,y,y,x],[[0,0,0],[0,0,0],[0,0,0],[0,0,-.4],[0,0,-.4],[.08,0,-.08]])
        chain(side+'_arm',torso,[0,sign*.23,.2],[y,x,z,y,z,y,x],[[0,0,0],[0,0,0],[.12,sign*.05,0],[.23,0,0],[.2,0,0],[.08,0,0],[.06,0,0]])
    chain('neck',torso,[0,0,.3],[z,y,x],[[0,0,0],[0,0,.04],[0,0,.06]])
    obj = ET.SubElement(root,'link',name='object'); v = ET.SubElement(obj,'visual');g = ET.SubElement(v,'geometry');ET.SubElement(g,'box',size='.25 .55 .12')
    j = ET.SubElement(root,'joint',name='object_free',type='floating');ET.SubElement(j,'parent',link='world');ET.SubElement(j,'child',link='object');ET.SubElement(j,'origin',xyz='.6 0 1.15')
    return ET.tostring(root,encoding='unicode')


def assembly(arms):
    source = ET.fromstring((ROOT/'models/panda.urdf').read_text())
    robot = ET.Element('robot',name=f'panda_{arms}_arm')
    ET.SubElement(robot,'link',name='world')
    for a in range(arms):
        prefix = f'a{a}_'
        for el in source:
            if el.tag not in ('link','joint') or el.get('name') in ('world','world_joint'):
                continue
            n = copy.deepcopy(el); n.set('name',prefix+n.get('name'))
            for tag in ('parent','child'):
                for child in n.findall(tag): child.set('link',prefix+child.get('link'))
            robot.append(n)
        yaw = a*2*np.pi/arms
        radius = 0 if arms == 1 else .68
        j = ET.SubElement(robot,'joint',name=prefix+'base',type='fixed')
        ET.SubElement(j,'parent',link='world'); ET.SubElement(j,'child',link=prefix+'panda_link0')
        ET.SubElement(j,'origin',xyz=f'{-radius*np.cos(yaw)} {-radius*np.sin(yaw)} 0',rpy=f'0 0 {yaw}')
    if arms > 1:
        link = ET.SubElement(robot,'link',name='object')
        visual = ET.SubElement(link,'visual'); geom = ET.SubElement(visual,'geometry')
        ET.SubElement(geom,'box',size='.28 .28 .08')
        j = ET.SubElement(robot,'joint',name='object_free',type='floating')
        ET.SubElement(j,'parent',link='world'); ET.SubElement(j,'child',link='object')
        ET.SubElement(j,'origin',xyz='0 0 .55')
    return ET.tostring(robot,encoding='unicode')


def make_scenario(scenario='dual', seed=7):
    external = next((e for e in REGISTRY if e['id']==scenario),None)
    if external:return external_scenario(external,seed)
    item = next((x for x in CATALOG if x['id']==scenario),None)
    if item is None: raise ValueError('Unknown scenario')
    arms = item['arms']
    model = RobotModel(humanoid_xml() if scenario=='humanoid' else assembly(arms))
    q = np.zeros(model.dof)
    for j in model.joints:
        if j.kind == 'revolute' and 'panda_joint' in j.name:
            q[j.index] = [0,-.55,0,-1.9,0,1.45,.7][int(j.name[-1])-1]
        elif j.kind == 'revolute' and '_arm_' in j.name:
            q[j.index] = .2 if j.name.endswith('_3_joint') else 0
        elif j.kind == 'revolute' and '_leg_' in j.name:
            q[j.index] = -.15 if j.name.endswith(('_2_joint','_4_joint')) else (.3 if j.name.endswith('_3_joint') else 0)
    initial = model.fk(q)
    rng = np.random.default_rng(seed)
    grasps = []
    if scenario == 'humanoid':
        targets = {'object':transform([.625,.015,1.18],[.01,.01,.02])}
        for side in ['left','right']:
            ee = side+'_arm_6'; foot = side+'_leg_5'
            grasps.append(('object',ee,inverse(initial['object'])@initial[ee]))
            grasps.append(('world',foot,initial[foot]))
    elif arms == 1:
        if scenario=='obstacle':
            qgoal = np.clip(q+np.array([.55,.22,-.2,-.25,.08,.22,.05]),model.lower,model.upper)
        else:
            qgoal = np.clip(q+rng.uniform(-.15,.15,model.dof),model.lower,model.upper)
        targets = {'a0_panda_link8':model.fk(qgoal)['a0_panda_link8']}
        if scenario == 'obstacle':
            a,b = initial['a0_panda_link8'][:3,3],targets['a0_panda_link8'][:3,3]
            model.obstacles = [dict(center=((a+b)/2).tolist(),radius=float(min(.055,np.linalg.norm(b-a)*.18)))]
            model.collision_spheres = [dict(link='a0_panda_link8',center=[0,0,0],radius=.025)]
    else:
        targets = {'object':transform([.035+float(rng.uniform(-.01,.01)),.025,.58],[.025,-.025,.045])}
        for a in range(arms):
            ee = f'a{a}_panda_link8'
            # Each grasp preserves its initial rigid object-to-tool transform.
            grasps.append(('object',ee,inverse(initial['object'])@initial[ee]))
    return model,q,targets,grasps
