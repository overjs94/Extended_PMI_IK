"""URDF transcription of the author's supplied 10-link LASDRA/EAMM model.

Axes, one-metre translations and link-centre frames follow the original C++.
Display geometry is schematic: the upload contains no LASDRA vendor meshes.
"""
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from .kinematics import RobotModel


def lasdra_xml():
    root=ET.Element('robot',name='LASDRA_EAMM_21DoF')
    ET.SubElement(root,'link',name='world')
    parent='world'
    for i in range(21):
        name=f'lasdra_joint_frame_{i}'
        link=ET.SubElement(root,'link',name=name)
        joint=ET.SubElement(root,'joint',name=f'lasdra_q{i}',type='continuous')
        ET.SubElement(joint,'parent',link=parent);ET.SubElement(joint,'child',link=name)
        ET.SubElement(joint,'origin',xyz='1 0 0' if i>=3 and i%2 else '0 0 0')
        ET.SubElement(joint,'axis',xyz='0 0 1' if i==0 else '0 1 0' if i%2 else '1 0 0')
        parent=name
        if i>=2 and i%2==0:
            centre=f'lasdra_link_{i//2}'
            body=ET.SubElement(root,'link',name=centre)
            fixed=ET.SubElement(root,'joint',name=centre+'_centre',type='fixed')
            ET.SubElement(fixed,'parent',link=parent);ET.SubElement(fixed,'child',link=centre)
            ET.SubElement(fixed,'origin',xyz='.5 0 0')
            for kind in ('visual','collision'):
                node=ET.SubElement(body,kind);geom=ET.SubElement(node,'geometry')
                ET.SubElement(geom,'box',size='1 .055 .055')
                if kind=='visual':
                    material=ET.SubElement(node,'material',name='schematic_carbon')
                    ET.SubElement(material,'color',rgba='.35 .52 .58 1')
            node=ET.SubElement(body,'visual');geom=ET.SubElement(node,'geometry')
            ET.SubElement(geom,'sphere',radius='.085')
    ET.SubElement(root,'link',name='lasdra_ee')
    joint=ET.SubElement(root,'joint',name='lasdra_tool',type='fixed')
    ET.SubElement(joint,'parent',link=parent);ET.SubElement(joint,'child',link='lasdra_ee')
    ET.SubElement(joint,'origin',xyz='1 0 0')
    return ET.tostring(root,encoding='unicode')


def lasdra_scenario(seed=7):
    model=RobotModel.from_file(Path(__file__).resolve().parents[2]/'models/lasdra.urdf')
    q=np.zeros(21);q[0]=.18;q[1]=-.32
    q[3::2]=.045;q[2::2]=.025
    rng=np.random.default_rng(seed)
    goal=q+rng.uniform(-.015,.015,21)
    return model,q,{'lasdra_ee':model.fk(goal)['lasdra_ee']},[]
